from robot_systems.robot import HamBot
import time, math, cv2

# ============================================================
# Bug Zero parameters
# ============================================================
WALL_FOLLOW_SIDE  = 'right'      # 'left' for Run 1, 'right' for Run 2
GOAL_DISTANCE_MM  = 250         # stop within this distance of the goal
MAX_SPEED         = 35          # absolute motor RPM cap during goal approach
CAMERA_WIDTH      = 640
CAMERA_CENTER_X   = CAMERA_WIDTH // 2

# Centering PID (keeps goal centered in camera frame)
KP_CENTER         = 0.05
KI_CENTER         = 0.001
KD_CENTER         = 0.02
CENTER_DEAD_ZONE  = 60          # pixel half-width — no correction within this band
CENTER_TURN_CAP   = 8.0         # max RPM differential from centering
CENTER_WINDUP     = 300.0       # integral windup cap (pixels)

# Speed PID (speeds up when far, slows down when close to goal)
KP_SPEED          = 0.04        # at 1000mm away → ~30 RPM
KD_SPEED          = 0.005
MIN_APPROACH_RPM  = 0.0         # allow speed to reach zero at goal

# ============================================================
# Wall follower timing and targets
# ============================================================
DT              = 0.032
SIDE_TARGET_MM  = 220.0
FRONT_TARGET_MM = 250.0
TRACK_MM        = 120.0

# ============================================================
# Wall follower speeds
# ============================================================
CRUISE_RPM        = 18.0
SEARCH_RPM        = 16.0
ROTATE_RPM        = 15.0
ROTATE_MIN_RPM    = 5.0
TURN_CAP_RPM      = 12.5
STEER_TO_RPM      = 0.24
SLEW_RPM_PER_TICK = 1.4

FRONT_STOP_MM  = FRONT_TARGET_MM
FRONT_SLOW_MM  = FRONT_TARGET_MM + 180.0
ROTATE_EXIT_MM = FRONT_TARGET_MM + 120.0

# ============================================================
# Wall follower PD gains
# ============================================================
KP_SIDE    = 0.10
KD_SIDE    = 1.90
DERIV_CLIP = 1500.0

EMA_ALPHA  = 0.55
NO_WALL_MM = 4000.0

# ============================================================
# Corner wrap parameters
# ============================================================
WRAP_OPEN_MM      = 70.0
WRAP_DS_TRIG      = 26.0
WRAP_MIN_TIME     = 0.12
WRAP_MAX_TIME     = 0.80
WRAP_SPEED_FAC    = 0.64
WRAP_SPEED_BOOST  = 1.22
WRAP_BOOST_T      = 0.12
WRAP_MIN_TURN     = 6.4
WRAP_TURN_ALPHA   = 0.55
WRAP_K_OPEN       = 0.0116
WRAP_K_RATE       = 0.0080
DIAG_CAPTURE_MM   = 420.0

WRAP_EXIT_BAND_MM  = 30.0
WRAP_RAMP_RANGE_MM = 220.0
WRAP_DS_EXIT       = -10.0
WRAP_COOLDOWN_S    = 0.20
WRAP_R_TARGET_MM   = 170.0
WRAP_PD_SCALE      = 0.28

WALL_DETECT_MM     = 600.0
SCAN_RPM           = 9.0


# ============================================================
# Utility
# ============================================================
def clamp(value, lo, hi):
    return max(lo, min(hi, value))

def saturation(bot, rpm):
    max_rpm = getattr(bot, 'max_motor_speed', 60)
    if rpm >  max_rpm: return  max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

class Slew:
    def __init__(self, max_delta):
        self.prev = 0.0
        self.maxd = float(max_delta)

    def step(self, target):
        d = target - self.prev
        if d >  self.maxd: d =  self.maxd
        if d < -self.maxd: d = -self.maxd
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
    scan = bot.get_range_image()
    return robust_min(scan[176:186], keep=5)

def side_mm(bot, side):
    scan = bot.get_range_image()
    if side == 'left':
        return robust_min(scan[84:96], keep=7)
    else:
        return robust_min(scan[264:276], keep=7)

def diag_mm(bot, side):
    scan = bot.get_range_image()
    if side == 'left':
        return robust_min(scan[128:142], keep=5)
    else:
        return robust_min(scan[218:232], keep=5)


# ============================================================
# WallFollower — EMA-smoothed PD with wrap and slew limiting
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

    def _front_speed(self, f_mm):
        if f_mm <= FRONT_STOP_MM:
            return 0.0
        if f_mm >= FRONT_SLOW_MM:
            return CRUISE_RPM
        a = (f_mm - FRONT_STOP_MM) / (FRONT_SLOW_MM - FRONT_STOP_MM)
        return CRUISE_RPM * max(0.0, min(1.0, a))

    def _rotate_pair(self):
        if self.side == 'left':
            return (+ROTATE_RPM, -ROTATE_RPM)
        else:
            return (-ROTATE_RPM, +ROTATE_RPM)

    def _wrap_ff_turn(self, s, ds, sign):
        if s == float('inf'):
            e_open = WRAP_OPEN_MM
        else:
            e_open = max(0.0, s - (SIDE_TARGET_MM + WRAP_OPEN_MM))
        ff   = WRAP_K_OPEN * e_open + WRAP_K_RATE * max(0.0, ds)
        turn = STEER_TO_RPM * ff * sign
        min_signed = WRAP_MIN_TURN * (1 if sign > 0 else -1)
        if sign > 0:
            if turn < min_signed: turn = min_signed
        else:
            if turn > min_signed: turn = min_signed
        if turn >  TURN_CAP_RPM: turn =  TURN_CAP_RPM
        if turn < -TURN_CAP_RPM: turn = -TURN_CAP_RPM
        return turn

    def _radius_turn(self, base_rpm, sign, R_mm=None):
        if R_mm is None:
            R_mm = SIDE_TARGET_MM
        R_mm  = max(180.0, R_mm)
        delta = (TRACK_MM * base_rpm) / (2.0 * R_mm)
        turn  = delta * (1 if sign > 0 else -1)
        if turn >  TURN_CAP_RPM: turn =  TURN_CAP_RPM
        if turn < -TURN_CAP_RPM: turn = -TURN_CAP_RPM
        min_signed = WRAP_MIN_TURN * (1 if sign > 0 else -1)
        if sign > 0 and turn < min_signed:  turn = min_signed
        if sign < 0 and turn > -min_signed: turn = -min_signed
        return turn

    def step(self):
        f         = front_mm(self.bot)
        s_raw     = side_mm(self.bot, self.side)
        d         = diag_mm(self.bot, self.side)
        sign      = +1 if self.side == 'left' else -1
        side_seen = s_raw < NO_WALL_MM

        if s_raw != float('inf'):
            if self.side_ema is None:
                self.side_ema = s_raw
            else:
                self.side_ema = EMA_ALPHA * s_raw + (1.0 - EMA_ALPHA) * self.side_ema
        s = self.side_ema if self.side_ema is not None else s_raw

        ds = 0.0
        if self.prev_side is not None and s != float('inf'):
            ds = s - self.prev_side
        self.prev_side = s if s != float('inf') else self.prev_side

        now = time.time()
        if side_seen:
            self.lost_since = None
        else:
            if self.lost_since is None:
                self.lost_since = now

        if self.mode == 'follow':
            if f < FRONT_STOP_MM:
                self.mode      = 'rotate'
                self.t0_rotate = now
            else:
                open_far  = (s != float('inf')) and ((s - SIDE_TARGET_MM) > WRAP_OPEN_MM)
                open_fast = ds > WRAP_DS_TRIG
                cool_ok   = (self.last_wrap_exit is None) or ((now - self.last_wrap_exit) > WRAP_COOLDOWN_S)
                if cool_ok and (open_far or open_fast) and f > FRONT_STOP_MM:
                    self.mode      = 'wrap'
                    self.t0_wrap   = now
                    self.turn_hold = self._wrap_ff_turn(s, ds, sign)

        elif self.mode == 'rotate':
            t         = now - (self.t0_rotate or now)
            can_exit  = t >= 0.26
            must_exit = t >= 0.90
            clear     = f > ROTATE_EXIT_MM
            diag_ok   = d < DIAG_CAPTURE_MM
            if must_exit or (can_exit and (clear or side_seen or diag_ok)):
                self.mode      = 'follow'
                self.t0_rotate = None

        elif self.mode == 'wrap':
            t           = now - (self.t0_wrap or now)
            done_time   = t >= WRAP_MIN_TIME
            too_long    = t >= WRAP_MAX_TIME
            diag_close  = d < DIAG_CAPTURE_MM
            side_caught = (s != float('inf')) and (s <= SIDE_TARGET_MM + WRAP_EXIT_BAND_MM)
            closing_ok  = (s != float('inf')) and (ds < WRAP_DS_EXIT)

            if f < FRONT_STOP_MM:
                self.mode      = 'rotate'
                self.t0_rotate = now
                self.t0_wrap   = None
                self.turn_hold = 0.0
            elif too_long or (done_time and (diag_close or side_caught or closing_ok)):
                self.mode           = 'follow'
                self.last_wrap_exit = now
                self.t0_wrap        = None
                self.turn_hold      = 0.0
            else:
                new_ff         = self._wrap_ff_turn(s, ds, sign)
                self.turn_hold = (1.0 - WRAP_TURN_ALPHA) * self.turn_hold + WRAP_TURN_ALPHA * new_ff

        if self.mode == 'rotate':
            l_cmd, r_cmd = self._rotate_pair()
            l_cmd *= 0.95
            r_cmd *= 0.95

        elif self.mode == 'search':
            base  = SEARCH_RPM
            turn  = 4.5 * (1 if sign > 0 else -1)
            l_cmd = base - turn
            r_cmd = base + turn

        else:
            base = self._front_speed(f)

            if side_seen and s != float('inf'):
                e     = s - SIDE_TARGET_MM
                dterm = (e - self.prev_err) / DT
                if dterm >  DERIV_CLIP: dterm =  DERIV_CLIP
                if dterm < -DERIV_CLIP: dterm = -DERIV_CLIP
                steer_pd      = KP_SIDE * e + KD_SIDE * dterm
                self.prev_err = e
            else:
                steer_pd = 0.0

            if self.mode == 'wrap':
                base *= WRAP_SPEED_FAC
                if (time.time() - (self.t0_wrap or time.time())) > WRAP_BOOST_T:
                    base *= WRAP_SPEED_BOOST
                turn_radius = self._radius_turn(base, sign, R_mm=WRAP_R_TARGET_MM)
                if s == float('inf'):
                    ramp = 1.0
                else:
                    over = s - (SIDE_TARGET_MM + WRAP_EXIT_BAND_MM)
                    ramp = max(0.0, min(1.0, over / WRAP_RAMP_RANGE_MM))
                turn = (ramp * (0.8 * turn_radius + 0.2 * self.turn_hold)
                        + STEER_TO_RPM * WRAP_PD_SCALE * steer_pd * (1 if sign > 0 else -1))
            else:
                turn = STEER_TO_RPM * steer_pd * (1 if sign > 0 else -1)

            if turn >  TURN_CAP_RPM: turn =  TURN_CAP_RPM
            if turn < -TURN_CAP_RPM: turn = -TURN_CAP_RPM
            l_cmd = base - turn
            r_cmd = base + turn

        l_out = saturation(self.bot, self.l_slew.step(l_cmd))
        r_out = saturation(self.bot, self.r_slew.step(r_cmd))
        return l_out, r_out


# ============================================================
# 360-degree scan — stops early and returns True if goal found
# ============================================================
def rotate_360_scan(bot):
    total_rotated = 0.0
    last_heading  = bot.get_heading()

    if len(bot.camera.find_landmarks()) > 0:
        return True

    while total_rotated < 360.0:
        cur   = bot.get_heading()
        delta = (cur - last_heading + 180) % 360 - 180
        total_rotated += abs(delta)
        last_heading   = cur

        remaining = 360.0 - total_rotated
        if remaining <= 2.0:
            break

        scale = max(ROTATE_MIN_RPM / SCAN_RPM, min(1.0, remaining / 360.0))
        rpm   = SCAN_RPM * scale
        bot.set_left_motor_speed(-rpm)
        bot.set_right_motor_speed(rpm)
        time.sleep(DT)

        if len(bot.camera.find_landmarks()) > 0:
            bot.set_left_motor_speed(0.0)
            bot.set_right_motor_speed(0.0)
            return True

    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)
    return False


# ============================================================
# IMU-guided 90-degree corner turn (tapered)
# ============================================================
def rotate_90(bot, wall_side):
    target_deg = 90.0
    sign       = +1 if wall_side == 'left' else -1
    start      = bot.get_heading()
    while True:
        cur   = bot.get_heading()
        delta = (cur - start + 540) % 360 - 180
        rem   = max(0.0, target_deg - abs(delta))
        if rem <= 2.0:
            break
        scale = max(ROTATE_MIN_RPM / ROTATE_RPM, min(1.0, rem / target_deg))
        rpm   = ROTATE_RPM * scale
        bot.set_left_motor_speed(sign * rpm)
        bot.set_right_motor_speed(-sign * rpm)
        time.sleep(DT)
    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)


# ============================================================
# Nearest wall detection
# ============================================================
def find_nearest_wall_side(bot, default_side):
    scan       = bot.get_range_image()
    left_dist  = robust_min(scan[85:95],   keep=5)
    right_dist = robust_min(scan[265:275], keep=5)
    left_near  = left_dist  < WALL_DETECT_MM
    right_near = right_dist < WALL_DETECT_MM
    if left_near and right_near:
        return 'left' if left_dist <= right_dist else 'right'
    if left_near:  return 'left'
    if right_near: return 'right'
    return default_side


# ============================================================
# PID state helper — resets all terms to zero
# ============================================================
def reset_pid(pid):
    pid['integral']  = 0.0
    pid['prev_err']  = 0.0


# ============================================================
# Bug Zero state machine
# ============================================================
def run_bug_zero(wall_side=WALL_FOLLOW_SIDE):
    bot = HamBot(lidar_enabled=True, camera_enabled=True)
    bot.camera.set_target_colors([(234, 213, 45)], tolerance=0.08)
    time.sleep(0.5)

    # PID state for centering (camera x error)
    center_pid = {'integral': 0.0, 'prev_err': 0.0}
    # PID state for speed (front LIDAR distance error)
    speed_pid  = {'integral': 0.0, 'prev_err': 0.0}

    # Slew limiters so motor commands ramp up smoothly
    l_slew = Slew(SLEW_RPM_PER_TICK)
    r_slew = Slew(SLEW_RPM_PER_TICK)

    # Startup: 360 scan before anything else
    print("Startup: scanning for goal")
    if rotate_360_scan(bot):
        print("Goal found at startup, heading toward it")
        state = 'MOTION_TO_GOAL'
        ctrl  = None
    else:
        print("No goal found, wall following on:", wall_side, "side")
        state = 'WALL_FOLLOWING'
        ctrl  = WallFollower(bot, wall_side=wall_side)

    try:
        while True:
            landmarks = bot.camera.find_landmarks()
            f         = front_mm(bot)

            # Camera feed
            frame = bot.camera.get_frame()
            if frame is not None:
                display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                for lm in landmarks:
                    x1 = lm.x - lm.width  // 2
                    y1 = lm.y - lm.height // 2
                    x2 = lm.x + lm.width  // 2
                    y2 = lm.y + lm.height // 2
                    cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(display, (lm.x, lm.y), 4, (0, 255, 0), -1)
                cv2.putText(display, state, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
                cv2.imshow('HamBot Camera', display)
                cv2.waitKey(1)

            # Stop when goal is close enough (wider window catches slightly off-axis pillar)
            scan_now  = bot.get_range_image()
            f_wide    = robust_min(scan_now[160:201], keep=5)
            if landmarks and f_wide < GOAL_DISTANCE_MM:
                print("Goal reached, stopping.")
                bot.stop_motors()
                break

            # ── MOTION TO GOAL ────────────────────────────────────────────
            if state == 'MOTION_TO_GOAL':

                if f < FRONT_STOP_MM:
                    # Hit a wall that is not the goal — switch to wall follow
                    print("Obstacle hit, switching to WALL_FOLLOWING")
                    bot.stop_motors()
                    reset_pid(center_pid)
                    reset_pid(speed_pid)
                    l_slew.prev = 0.0
                    r_slew.prev = 0.0
                    ctrl  = WallFollower(bot, wall_side=wall_side)
                    state = 'WALL_FOLLOWING'
                    continue

                if landmarks:
                    lm      = max(landmarks, key=lambda l: l.width * l.height)
                    error_x = lm.x - CAMERA_CENTER_X

                    # Centering PID
                    if abs(error_x) <= CENTER_DEAD_ZONE:
                        turn = 0.0
                        reset_pid(center_pid)
                    else:
                        center_pid['integral'] += error_x * DT
                        center_pid['integral']  = clamp(center_pid['integral'],
                                                        -CENTER_WINDUP, CENTER_WINDUP)
                        d_err = (error_x - center_pid['prev_err']) / DT
                        turn  = (KP_CENTER * error_x
                                 + KI_CENTER * center_pid['integral']
                                 + KD_CENTER * d_err)
                        turn  = clamp(turn, -CENTER_TURN_CAP, CENTER_TURN_CAP)
                        center_pid['prev_err'] = error_x

                    # Speed PID — slow down as front distance approaches goal
                    spd_err = f - GOAL_DISTANCE_MM
                    d_spd   = (spd_err - speed_pid['prev_err']) / DT
                    base    = KP_SPEED * spd_err + KD_SPEED * d_spd
                    base    = clamp(base, 0, MAX_SPEED)
                    speed_pid['prev_err'] = spd_err

                    left_spd  = clamp(base + turn, -MAX_SPEED, MAX_SPEED)
                    right_spd = clamp(base - turn, -MAX_SPEED, MAX_SPEED)
                    bot.set_left_motor_speed(l_slew.step(left_spd))
                    bot.set_right_motor_speed(r_slew.step(right_spd))

                else:
                    # Goal out of frame — drive straight until hitting a wall
                    reset_pid(center_pid)
                    base = clamp(KP_SPEED * (f - GOAL_DISTANCE_MM), 0, MAX_SPEED)
                    bot.set_left_motor_speed(l_slew.step(base))
                    bot.set_right_motor_speed(r_slew.step(base))

            # ── WALL FOLLOWING ────────────────────────────────────────────
            elif state == 'WALL_FOLLOWING':

                if landmarks:
                    print("Goal visible, switching to MOTION_TO_GOAL")
                    bot.stop_motors()
                    reset_pid(center_pid)
                    reset_pid(speed_pid)
                    l_slew.prev = 0.0
                    r_slew.prev = 0.0
                    ctrl  = None
                    state = 'MOTION_TO_GOAL'
                    continue

                l_rpm, r_rpm = ctrl.step()
                bot.set_left_motor_speed(l_rpm)
                bot.set_right_motor_speed(r_rpm)

                if f < FRONT_STOP_MM:
                    bot.stop_motors()
                    print("Corner: scanning 360 for goal")
                    if rotate_360_scan(bot):
                        print("Goal found, switching to MOTION_TO_GOAL")
                        reset_pid(center_pid)
                        reset_pid(speed_pid)
                        l_slew.prev = 0.0
                        r_slew.prev = 0.0
                        ctrl  = None
                        state = 'MOTION_TO_GOAL'
                    else:
                        rotate_90(bot, wall_side)
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
        cv2.destroyAllWindows()


if __name__ == '__main__':
    run_bug_zero()
