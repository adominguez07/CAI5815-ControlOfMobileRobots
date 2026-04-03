from HamBot.src.robot_systems.robot import HamBot
import time

# ============================================================
# Tuneable parameters
# ============================================================

# Wall side — change to 'left' or 'right' before each run
WALL_FOLLOW_SIDE    = 'right'

# Goal
GOAL_DISTANCE_MM    = 250.0
CAMERA_WIDTH        = 640
CAMERA_CENTER_X     = CAMERA_WIDTH // 2

# Loop timing
DT                  = 0.032     # main loop period (s)

# Wall geometry
SIDE_TARGET_MM      = 220.0     # desired clearance to followed wall
FRONT_STOP_MM       = 250.0     # obstacle / corner threshold
FRONT_SLOW_MM       = 430.0     # start slowing below this
ROTATE_EXIT_MM      = 370.0     # exit in-place rotate when front clears this
TRACK_MM            = 120.0     # wheel track (mm)
NO_WALL_MM          = 4000.0    # beyond this = no wall

# Motor speeds
CRUISE_RPM          = 18.0
ROTATE_RPM          = 15.0
ROTATE_MIN_RPM      = 5.0
SCAN_RPM            = 9.0
MAX_APPROACH_RPM    = 30.0
SEARCH_RPM          = 16.0
SLEW_RPM_PER_TICK   = 1.4       # max RPM change per DT (acceleration limiter)
TURN_CAP_RPM        = 12.5
STEER_TO_RPM        = 0.24

# Wall-follow PD gains
KP_SIDE             = 0.10
KD_SIDE             = 1.90
DERIV_CLIP          = 1500.0
EMA_ALPHA           = 0.55      # side-distance smoothing

# Corner-wrap parameters
WRAP_OPEN_MM        = 70.0
WRAP_DS_TRIG        = 26.0
WRAP_MIN_TIME       = 0.12
WRAP_MAX_TIME       = 0.80
WRAP_SPEED_FAC      = 0.64
WRAP_SPEED_BOOST    = 1.22
WRAP_BOOST_T        = 0.12
WRAP_MIN_TURN       = 6.4
WRAP_TURN_ALPHA     = 0.55
WRAP_K_OPEN         = 0.0116
WRAP_K_RATE         = 0.0080
DIAG_CAPTURE_MM     = 420.0
WRAP_EXIT_BAND_MM   = 30.0
WRAP_RAMP_RANGE_MM  = 220.0
WRAP_DS_EXIT        = -10.0
WRAP_COOLDOWN_S     = 0.20
WRAP_R_TARGET_MM    = 170.0
WRAP_PD_SCALE       = 0.28

# Goal-approach PID
KP_CENTER           = 0.05
KI_CENTER           = 0.001
KD_CENTER           = 0.02
CENTER_DEAD_ZONE    = 60        # pixel half-width — no correction inside band
CENTER_TURN_CAP     = 8.0
CENTER_WINDUP       = 300.0
KP_SPEED            = 0.04
KD_SPEED            = 0.005


# ============================================================
# Utility
# ============================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def saturation(bot, rpm):
    m = getattr(bot, 'max_motor_speed', 60)
    return clamp(rpm, -m, m)

class Slew:
    """Rate-limits motor command changes — prevents jerky starts."""
    def __init__(self, max_delta):
        self.prev = 0.0
        self.maxd = float(max_delta)

    def step(self, target):
        d = clamp(target - self.prev, -self.maxd, self.maxd)
        self.prev += d
        return self.prev

def robust_min(vals, keep=5):
    xs = [v for v in vals if v and v > 0]
    if not xs:
        return float('inf')
    xs.sort()
    k = min(keep, len(xs))
    return sum(xs[:k]) / k

def front_mm(bot):
    return robust_min(bot.get_range_image()[176:186], keep=5)

def front_wide_mm(bot):
    """40-degree window — catches goal pillar slightly off axis."""
    return robust_min(bot.get_range_image()[160:201], keep=5)

def side_mm(bot, side):
    scan = bot.get_range_image()
    return robust_min(scan[84:96] if side == 'left' else scan[264:276], keep=7)

def diag_mm(bot, side):
    scan = bot.get_range_image()
    return robust_min(scan[128:142] if side == 'left' else scan[218:232], keep=5)


# ============================================================
# WallFollower — EMA-smoothed PD with open-corner wrap
# (structure taken from Lab2_Task2)
# ============================================================

class WallFollower:
    def __init__(self, bot, wall_side='left'):
        self.bot            = bot
        self.side           = wall_side
        self.side_ema       = None
        self.prev_side      = None
        self.prev_err       = 0.0
        self.mode           = 'follow'
        self.t0_wrap        = None
        self.last_wrap_exit = None
        self.t0_rotate      = None
        self.lost_since     = None
        self.turn_hold      = 0.0
        self.l_slew         = Slew(SLEW_RPM_PER_TICK)
        self.r_slew         = Slew(SLEW_RPM_PER_TICK)

    def _front_speed(self, f):
        if f <= FRONT_STOP_MM:  return 0.0
        if f >= FRONT_SLOW_MM:  return CRUISE_RPM
        return CRUISE_RPM * clamp((f - FRONT_STOP_MM) / (FRONT_SLOW_MM - FRONT_STOP_MM), 0.0, 1.0)

    def _rotate_pair(self):
        # CW for left-wall, CCW for right-wall
        if self.side == 'left':
            return (+ROTATE_RPM, -ROTATE_RPM)
        return (-ROTATE_RPM, +ROTATE_RPM)

    def _wrap_ff_turn(self, s, ds, sign):
        e_open = WRAP_OPEN_MM if s == float('inf') else max(0.0, s - (SIDE_TARGET_MM + WRAP_OPEN_MM))
        turn   = STEER_TO_RPM * (WRAP_K_OPEN * e_open + WRAP_K_RATE * max(0.0, ds)) * sign
        min_s  = WRAP_MIN_TURN * sign
        if sign > 0 and turn < min_s: turn = min_s
        if sign < 0 and turn > min_s: turn = min_s
        return clamp(turn, -TURN_CAP_RPM, TURN_CAP_RPM)

    def _radius_turn(self, base, sign, R_mm=None):
        R_mm  = max(180.0, R_mm or SIDE_TARGET_MM)
        turn  = clamp((TRACK_MM * base) / (2.0 * R_mm) * (1 if sign > 0 else -1),
                      -TURN_CAP_RPM, TURN_CAP_RPM)
        min_s = WRAP_MIN_TURN * sign
        if sign > 0 and turn < min_s:  turn = min_s
        if sign < 0 and turn > min_s:  turn = min_s
        return turn

    def step(self):
        f         = front_mm(self.bot)
        s_raw     = side_mm(self.bot, self.side)
        d         = diag_mm(self.bot, self.side)
        sign      = +1 if self.side == 'left' else -1
        side_seen = s_raw < NO_WALL_MM

        # EMA-smooth side distance
        if s_raw != float('inf'):
            self.side_ema = s_raw if self.side_ema is None else (
                EMA_ALPHA * s_raw + (1.0 - EMA_ALPHA) * self.side_ema)
        s = self.side_ema if self.side_ema is not None else s_raw

        # Opening rate
        ds = (s - self.prev_side) if (self.prev_side is not None and s != float('inf')) else 0.0
        self.prev_side = s if s != float('inf') else self.prev_side

        now = time.time()
        if side_seen:
            self.lost_since = None
        elif self.lost_since is None:
            self.lost_since = now

        # ── Mode transitions ──────────────────────────────────
        if self.mode == 'follow':
            if f < FRONT_STOP_MM:
                self.mode = 'rotate'; self.t0_rotate = now
            else:
                cool   = self.last_wrap_exit is None or (now - self.last_wrap_exit) > WRAP_COOLDOWN_S
                open_f = s != float('inf') and (s - SIDE_TARGET_MM) > WRAP_OPEN_MM
                if cool and f > FRONT_STOP_MM and (open_f or ds > WRAP_DS_TRIG):
                    self.mode = 'wrap'; self.t0_wrap = now
                    self.turn_hold = self._wrap_ff_turn(s, ds, sign)

        elif self.mode == 'rotate':
            t = now - (self.t0_rotate or now)
            if t >= 0.90 or (t >= 0.26 and (f > ROTATE_EXIT_MM or side_seen or d < DIAG_CAPTURE_MM)):
                self.mode = 'follow'; self.t0_rotate = None

        elif self.mode == 'wrap':
            t      = now - (self.t0_wrap or now)
            close  = s != float('inf') and s <= SIDE_TARGET_MM + WRAP_EXIT_BAND_MM
            clsing = s != float('inf') and ds < WRAP_DS_EXIT
            if f < FRONT_STOP_MM:
                self.mode = 'rotate'; self.t0_rotate = now; self.t0_wrap = None; self.turn_hold = 0.0
            elif t >= WRAP_MAX_TIME or (t >= WRAP_MIN_TIME and (d < DIAG_CAPTURE_MM or close or clsing)):
                self.mode = 'follow'; self.last_wrap_exit = now; self.t0_wrap = None; self.turn_hold = 0.0
            else:
                self.turn_hold = ((1.0 - WRAP_TURN_ALPHA) * self.turn_hold
                                  + WRAP_TURN_ALPHA * self._wrap_ff_turn(s, ds, sign))

        # ── Command generation ────────────────────────────────
        if self.mode == 'rotate':
            l_cmd, r_cmd = self._rotate_pair()
            l_cmd *= 0.95; r_cmd *= 0.95

        elif self.mode == 'search':
            l_cmd = SEARCH_RPM - 4.5 * sign
            r_cmd = SEARCH_RPM + 4.5 * sign

        else:
            base = self._front_speed(f)
            if side_seen and s != float('inf'):
                e    = s - SIDE_TARGET_MM
                derr = clamp((e - self.prev_err) / DT, -DERIV_CLIP, DERIV_CLIP)
                steer_pd = KP_SIDE * e + KD_SIDE * derr
                self.prev_err = e
            else:
                steer_pd = 0.0

            if self.mode == 'wrap':
                base *= WRAP_SPEED_FAC
                if (time.time() - (self.t0_wrap or time.time())) > WRAP_BOOST_T:
                    base *= WRAP_SPEED_BOOST
                r_turn = self._radius_turn(base, sign, R_mm=WRAP_R_TARGET_MM)
                ramp   = 1.0 if s == float('inf') else clamp(
                    (s - (SIDE_TARGET_MM + WRAP_EXIT_BAND_MM)) / WRAP_RAMP_RANGE_MM, 0.0, 1.0)
                turn = (ramp * (0.8 * r_turn + 0.2 * self.turn_hold)
                        + STEER_TO_RPM * WRAP_PD_SCALE * steer_pd * sign)
            else:
                turn = STEER_TO_RPM * steer_pd * sign

            turn  = clamp(turn, -TURN_CAP_RPM, TURN_CAP_RPM)
            l_cmd = base - turn
            r_cmd = base + turn

        return (saturation(self.bot, self.l_slew.step(l_cmd)),
                saturation(self.bot, self.r_slew.step(r_cmd)))


# ============================================================
# Navigation primitives
# ============================================================

def detect_nearest_wall(bot):
    """Return 'left' or 'right' based on which wall is closer at startup."""
    scan = bot.get_range_image()
    l    = robust_min(scan[85:95],   keep=5)
    r    = robust_min(scan[265:275], keep=5)
    return 'left' if l <= r else 'right'


def rotate_360_scan(bot):
    """
    Rotate one full turn at SCAN_RPM.
    Returns True (and stops) the moment a landmark is spotted.
    Returns False after completing the full rotation without seeing the goal.
    """
    if bot.camera.find_landmarks():
        return True

    total    = 0.0
    last_hdg = bot.get_heading()

    while total < 360.0:
        hdg   = bot.get_heading()
        delta = (hdg - last_hdg + 180) % 360 - 180
        total    += abs(delta)
        last_hdg  = hdg

        if total >= 358.0:
            break

        rem = 360.0 - total
        rpm = SCAN_RPM * max(ROTATE_MIN_RPM / SCAN_RPM, min(1.0, rem / 360.0))
        bot.set_left_motor_speed(-rpm)
        bot.set_right_motor_speed(rpm)
        time.sleep(DT)

        if bot.camera.find_landmarks():
            bot.set_left_motor_speed(0.0)
            bot.set_right_motor_speed(0.0)
            return True

    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)
    return False


def rotate_90(bot, wall_side):
    """
    IMU-guided 90-degree turn with tapered (P-controller) speed.
    Left-wall following turns CW; right-wall following turns CCW.
    """
    sign  = +1 if wall_side == 'left' else -1
    start = bot.get_heading()

    while True:
        hdg   = bot.get_heading()
        delta = (hdg - start + 540) % 360 - 180
        rem   = max(0.0, 90.0 - abs(delta))
        if rem <= 2.0:
            break
        rpm = ROTATE_RPM * max(ROTATE_MIN_RPM / ROTATE_RPM, min(1.0, rem / 90.0))
        bot.set_left_motor_speed(sign * rpm)
        bot.set_right_motor_speed(-sign * rpm)
        time.sleep(DT)

    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)


# ============================================================
# Bug Zero state machine
# ============================================================

def run_bug_zero():
    bot = HamBot(lidar_enabled=True, camera_enabled=True)
    bot.max_motor_speed = 60
    bot.camera.set_target_colors([(206, 56, 79)])
    time.sleep(0.5)

    # Slew limiters for goal-approach commands
    l_slew = Slew(SLEW_RPM_PER_TICK)
    r_slew = Slew(SLEW_RPM_PER_TICK)

    # PID state for goal approach
    ctr_pid = {'integral': 0.0, 'prev_err': 0.0}
    spd_pid = {'prev_err': 0.0}

    def reset_approach():
        ctr_pid['integral'] = 0.0
        ctr_pid['prev_err'] = 0.0
        spd_pid['prev_err'] = 0.0
        l_slew.prev = 0.0
        r_slew.prev = 0.0

    # ── Startup: wall side set by WALL_FOLLOW_SIDE variable ──
    wall_side = WALL_FOLLOW_SIDE
    print("Wall side:", wall_side)

    # ── Startup: 360 scan for goal ────────────────────────────
    print("Initial 360 scan...")
    if rotate_360_scan(bot):
        print("Goal spotted at startup")
        state = 'MOTION_TO_GOAL'
        ctrl  = None
    else:
        print("No goal found — wall following:", wall_side)
        state = 'WALL_FOLLOWING'
        ctrl  = WallFollower(bot, wall_side=wall_side)

    try:
        while True:
            landmarks = bot.camera.find_landmarks()
            f         = front_mm(bot)
            f_wide    = front_wide_mm(bot)

            # Global stop: goal is close enough
            if landmarks and f_wide < GOAL_DISTANCE_MM:
                print("Goal reached — stopping")
                bot.stop_motors()
                break

            # ══ MOTION TO GOAL ════════════════════════════════
            if state == 'MOTION_TO_GOAL':

                # Real obstacle (not the goal) — switch to wall follow
                if f < FRONT_STOP_MM and not landmarks:
                    print("Obstacle — switching to wall follow")
                    bot.stop_motors()
                    reset_approach()
                    ctrl  = WallFollower(bot, wall_side=wall_side)
                    state = 'WALL_FOLLOWING'
                    time.sleep(DT)
                    continue

                # Camera centering PID
                if landmarks:
                    lm    = max(landmarks, key=lambda lm: lm.width * lm.height)
                    err_x = lm.x - CAMERA_CENTER_X
                    if abs(err_x) <= CENTER_DEAD_ZONE:
                        turn = 0.0
                        ctr_pid['integral'] = 0.0
                        ctr_pid['prev_err'] = 0.0
                    else:
                        ctr_pid['integral'] = clamp(
                            ctr_pid['integral'] + err_x * DT,
                            -CENTER_WINDUP, CENTER_WINDUP)
                        d_err = (err_x - ctr_pid['prev_err']) / DT
                        turn  = clamp(
                            KP_CENTER * err_x
                            + KI_CENTER * ctr_pid['integral']
                            + KD_CENTER * d_err,
                            -CENTER_TURN_CAP, CENTER_TURN_CAP)
                        ctr_pid['prev_err'] = err_x
                else:
                    # Goal not in frame — drive straight toward where it was
                    turn = 0.0
                    ctr_pid['integral'] = 0.0
                    ctr_pid['prev_err'] = 0.0

                # Speed PID — decelerates to zero at GOAL_DISTANCE_MM
                spd_err = f - GOAL_DISTANCE_MM
                d_spd   = (spd_err - spd_pid['prev_err']) / DT
                base    = clamp(KP_SPEED * spd_err + KD_SPEED * d_spd, 0.0, MAX_APPROACH_RPM)
                spd_pid['prev_err'] = spd_err

                bot.set_left_motor_speed(
                    l_slew.step(clamp(base + turn, -MAX_APPROACH_RPM, MAX_APPROACH_RPM)))
                bot.set_right_motor_speed(
                    r_slew.step(clamp(base - turn, -MAX_APPROACH_RPM, MAX_APPROACH_RPM)))

            # ══ WALL FOLLOWING ════════════════════════════════
            elif state == 'WALL_FOLLOWING':

                if landmarks:
                    print("Goal visible — heading to goal")
                    bot.stop_motors()
                    reset_approach()
                    ctrl  = None
                    state = 'MOTION_TO_GOAL'
                    time.sleep(DT)
                    continue

                l_rpm, r_rpm = ctrl.step()
                bot.set_left_motor_speed(l_rpm)
                bot.set_right_motor_speed(r_rpm)

                if f < FRONT_STOP_MM:
                    bot.stop_motors()
                    print("Corner — 360 scan")

                    if rotate_360_scan(bot):
                        print("Goal found at corner — heading to goal")
                        reset_approach()
                        ctrl  = None
                        state = 'MOTION_TO_GOAL'
                    else:
                        print("No goal — turning 90 and resuming")
                        rotate_90(bot, wall_side)
                        # Brief forward drive to clear the corner before re-attaching to wall
                        t0 = time.time()
                        while time.time() - t0 < 0.8 and front_mm(bot) > FRONT_STOP_MM:
                            bot.set_left_motor_speed(CRUISE_RPM)
                            bot.set_right_motor_speed(CRUISE_RPM)
                            time.sleep(DT)
                        bot.stop_motors()
                        ctrl = WallFollower(bot, wall_side=wall_side)

            time.sleep(DT)

    finally:
        bot.stop_motors()


if __name__ == '__main__':
    run_bug_zero()
