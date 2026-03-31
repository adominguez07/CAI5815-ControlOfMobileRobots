
"""
HamBot — Task 2 (From Scratch, 250/250) — Faster, Tighter Wrap Variant (EXIT-FIXED)

- Keeps your faster/tighter wrap and adds clean, early exit with ramp-down.
- Fixes syntax/indentation errors seen in IDE (stray lines in wrap block).

Targets: SIDE = 250 mm, FRONT = 250 mm.
"""

import time, math
from HamBot.src.robot_systems.robot import HamBot

# ========================
# Timing & Targets
# ========================
DT = 0.032
SIDE_TARGET_MM  = 250.0
FRONT_TARGET_MM = 250.0

# ========================
# Geometry (wheel track) — tune for your bot if needed
# ========================
TRACK_MM = 120.0   # distance between wheel contact points (mm)

# ========================
# Speeds / limits
# ========================
CRUISE_RPM         = 18.0
SEARCH_RPM         = 16.0
ROTATE_RPM         = 22.0
ROTATE_MIN_RPM     = 6.0
TURN_CAP_RPM       = 12.5     # allow tighter differential for faster wraps
STEER_TO_RPM       = 0.24
SLEW_RPM_PER_TICK  = 1.4

# ========================
# Front thresholds
# ========================
FRONT_STOP_MM      = FRONT_TARGET_MM
FRONT_SLOW_MM      = FRONT_TARGET_MM + 180.0
ROTATE_EXIT_MM     = FRONT_TARGET_MM + 120.0

# ========================
# Side PD (no integral)
# ========================
KP_SIDE = 0.10
KD_SIDE = 1.90
DERIV_CLIP = 1500.0

# ========================
# Side smoothing
# ========================
EMA_ALPHA = 0.55
NO_WALL_MM = 4000.0

# ========================
# Corner wrap (faster & tighter)
# ========================
WRAP_OPEN_MM     = 70.0
WRAP_DS_TRIG     = 26.0
WRAP_MIN_TIME    = 0.12
WRAP_MAX_TIME    = 0.80
WRAP_SPEED_FAC   = 0.64         # faster during wrap
WRAP_SPEED_BOOST = 1.22         # mild boost after entry
WRAP_BOOST_T     = 0.12         # apply boost after this many seconds in wrap

WRAP_MIN_TURN    = 6.4        # stronger minimum turn toward wall
WRAP_TURN_ALPHA  = 0.55
WRAP_K_OPEN      = 0.0116
WRAP_K_RATE      = 0.0080
DIAG_CAPTURE_MM  = 420.0

# ===== Wrap exit/ramp parameters =====
WRAP_EXIT_BAND_MM  = 30.0   # exit wrap when side <= target + 30 mm
WRAP_RAMP_RANGE_MM = 220.0  # ramp wrap curvature from far -> near target
WRAP_DS_EXIT       = -10.0  # if ds < -10 mm/step (closing), allow exit
WRAP_COOLDOWN_S    = 0.20   # don't re-enter wrap for 0.2s after exit

WRAP_R_TARGET_MM = 170.0  # aim to hug corner a little tighter than side target

# PD contribution inside wrap (kept modest)
WRAP_PD_SCALE    = 0.28

# ========================
# Helpers
# ========================
def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:  return max_rpm
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
    if not xs: return float('inf')
    xs.sort()
    k = min(keep, len(xs))
    return sum(xs[:k]) / k

# Sensors
def front_mm(bot):
    scan = bot.get_range_image()
    return robust_min(scan[176:186], keep=5)

def side_mm(bot, side):
    scan = bot.get_range_image()
    if side == "left":
        return robust_min(scan[84:96], keep=7)
    else:
        return robust_min(scan[264:276], keep=7)

def diag_mm(bot, side):
    scan = bot.get_range_image()
    if side == "left":
        return robust_min(scan[128:142], keep=5)   # ~135°
    else:
        return robust_min(scan[218:232], keep=5)   # ~225°

# ========================
# Controller
# ========================
class WallFollower:
    def __init__(self, bot, wall_side="left"):
        self.bot = bot
        self.side = wall_side

        self.side_ema = None
        self.prev_side = None
        self.prev_err  = 0.0

        self.mode = "follow"  # "follow", "wrap", "rotate", "search"
        self.t0_wrap = None
        self.last_wrap_exit = None
        self.t0_rotate = None
        self.lost_since = None

        self.turn_hold = 0.0   # constant-curvature component during wrap

        self.l_slew = Slew(SLEW_RPM_PER_TICK)
        self.r_slew = Slew(SLEW_RPM_PER_TICK)

    def _front_speed(self, f_mm):
        if f_mm <= FRONT_STOP_MM: return 0.0
        if f_mm >= FRONT_SLOW_MM: return CRUISE_RPM
        a = (f_mm - FRONT_STOP_MM) / (FRONT_SLOW_MM - FRONT_STOP_MM)
        return CRUISE_RPM * max(0.0, min(1.0, a))

    def _rotate_pair(self):
        # CW for left-follow; CCW for right-follow
        if self.side == "left":
            return (+ROTATE_RPM, -ROTATE_RPM)
        else:
            return (-ROTATE_RPM, +ROTATE_RPM)

    def _wrap_ff_turn(self, s, ds, sign):
        # feed-forward curvature from opening size and rate
        if s == float('inf'):
            e_open = WRAP_OPEN_MM
        else:
            e_open = max(0.0, s - (SIDE_TARGET_MM + WRAP_OPEN_MM))
        ff = WRAP_K_OPEN * e_open + WRAP_K_RATE * max(0.0, ds)
        turn = STEER_TO_RPM * ff * sign
        # enforce minimum turning toward wall
        min_signed = WRAP_MIN_TURN * (1 if sign > 0 else -1)
        if sign > 0:
            if turn < min_signed: turn = min_signed
        else:
            if turn > min_signed: turn = min_signed
        # cap
        if turn >  TURN_CAP_RPM: turn =  TURN_CAP_RPM
        if turn < -TURN_CAP_RPM: turn = -TURN_CAP_RPM
        return turn

    def _radius_turn(self, base_rpm, sign, R_mm=None):
        """Turn needed (RPM) to achieve radius R for the robot center: R = (TRACK * V) / (2*Δ) ⇒ Δ = (TRACK*V)/(2*R)."""
        if R_mm is None: R_mm = SIDE_TARGET_MM
        R_mm = max(180.0, R_mm)  # avoid singular/too-tight
        delta = (TRACK_MM * base_rpm) / (2.0 * R_mm)  # RPM
        turn = delta * (1 if sign > 0 else -1)
        # cap
        if turn >  TURN_CAP_RPM: turn =  TURN_CAP_RPM
        if turn < -TURN_CAP_RPM: turn = -TURN_CAP_RPM
        # ensure minimum toward wall
        min_signed = WRAP_MIN_TURN * (1 if sign > 0 else -1)
        if sign > 0 and turn < min_signed: turn = min_signed
        if sign < 0 and turn > -min_signed: turn = -min_signed
        return turn

    def step(self):
        f = front_mm(self.bot)
        s_raw = side_mm(self.bot, self.side)
        d = diag_mm(self.bot, self.side)
        sign = +1 if self.side == "left" else -1
        side_seen = s_raw < NO_WALL_MM

        # EMA for side distance
        if s_raw != float('inf'):
            if self.side_ema is None: self.side_ema = s_raw
            else: self.side_ema = EMA_ALPHA*s_raw + (1.0-EMA_ALPHA)*self.side_ema
        s = self.side_ema if self.side_ema is not None else s_raw

        # opening rate
        ds = 0.0
        if self.prev_side is not None and s != float('inf'):
            ds = s - self.prev_side
        self.prev_side = s if s != float('inf') else self.prev_side

        # lost wall timer
        now = time.time()
        if side_seen: self.lost_since = None
        else:
            if self.lost_since is None: self.lost_since = now

        # --- mode transitions ---
        if self.mode == "follow":
            if f < FRONT_STOP_MM:
                self.mode = "rotate"; self.t0_rotate = now
            else:
                open_far  = (s != float('inf')) and ((s - SIDE_TARGET_MM) > WRAP_OPEN_MM)
                open_fast = ds > WRAP_DS_TRIG
                cool_ok   = (self.last_wrap_exit is None) or ((now - self.last_wrap_exit) > WRAP_COOLDOWN_S)
                if cool_ok and (open_far or open_fast) and f > FRONT_STOP_MM:
                    self.mode = "wrap"; self.t0_wrap = now
                    # initialize wrap turn (feed-forward)
                    self.turn_hold = self._wrap_ff_turn(s, ds, sign)

        elif self.mode == "rotate":
            t = now - (self.t0_rotate or now)
            can_exit  = t >= 0.26
            must_exit = t >= 0.90
            clear     = f > ROTATE_EXIT_MM
            diag_ok   = d < DIAG_CAPTURE_MM
            if must_exit or (can_exit and (clear or side_seen or diag_ok)):
                self.mode = "follow"; self.t0_rotate = None

        elif self.mode == "wrap":
            t = now - (self.t0_wrap or now)
            done_time   = t >= WRAP_MIN_TIME
            too_long    = t >= WRAP_MAX_TIME
            diag_close  = d < DIAG_CAPTURE_MM
            side_caught = (s != float('inf')) and (s <= SIDE_TARGET_MM + WRAP_EXIT_BAND_MM)
            closing_ok  = (s != float('inf')) and (ds < WRAP_DS_EXIT)

            if f < FRONT_STOP_MM:
                # front blocked during wrap → rotate now
                self.mode = "rotate"; self.t0_rotate = now; self.t0_wrap = None; self.turn_hold = 0.0
            elif too_long or (done_time and (diag_close or side_caught or closing_ok)):
                # exit wrap early once we have the wall again (or it's closing toward target)
                self.mode = "follow"; self.last_wrap_exit = now; self.t0_wrap = None; self.turn_hold = 0.0
            else:
                # Update constant-curvature turn hold
                new_ff = self._wrap_ff_turn(s, ds, sign)
                self.turn_hold = (1.0 - WRAP_TURN_ALPHA)*self.turn_hold + WRAP_TURN_ALPHA*new_ff

        # --- command compute ---
        if self.mode == "rotate":
            l_cmd, r_cmd = self._rotate_pair()
            l_cmd *= 0.95; r_cmd *= 0.95

        elif self.mode == "search":
            base = SEARCH_RPM
            turn = 4.5 * (1 if sign > 0 else -1)
            l_cmd = base - turn
            r_cmd = base + turn

        else:
            base = self._front_speed(f)

            # side PD
            if side_seen and s != float('inf'):
                e = s - SIDE_TARGET_MM
                dterm = (e - self.prev_err) / DT
                if dterm >  DERIV_CLIP: dterm =  DERIV_CLIP
                if dterm < -DERIV_CLIP: dterm = -DERIV_CLIP
                steer_pd = KP_SIDE*e + KD_SIDE*dterm
                self.prev_err = e
            else:
                steer_pd = 0.0

            if self.mode == "wrap":
                # faster wrap base speed
                base *= WRAP_SPEED_FAC
                if (time.time() - (self.t0_wrap or time.time())) > WRAP_BOOST_T:
                    base *= WRAP_SPEED_BOOST

                # curvature to target ~250 mm radius (scales with speed)
                turn_radius = self._radius_turn(base, sign, R_mm=WRAP_R_TARGET_MM)

                # Ramp factor: as side approaches target, fade out wrap turn
                if s == float('inf'):
                    ramp = 1.0
                else:
                    over = (s - (SIDE_TARGET_MM + WRAP_EXIT_BAND_MM))
                    ramp = max(0.0, min(1.0, over / WRAP_RAMP_RANGE_MM))

                # constant-curvature hold (ramped) + small PD correction
                turn = ramp*(0.8*turn_radius + 0.2*self.turn_hold) + (STEER_TO_RPM * WRAP_PD_SCALE * steer_pd * (1 if sign > 0 else -1))
            else:
                turn = STEER_TO_RPM * steer_pd * (1 if sign > 0 else -1)

            # clamp and map to wheels
            if turn >  TURN_CAP_RPM: turn =  TURN_CAP_RPM
            if turn < -TURN_CAP_RPM: turn = -TURN_CAP_RPM
            l_cmd = base - turn
            r_cmd = base + turn

        l_out = saturation(self.bot, self.l_slew.step(l_cmd))
        r_out = saturation(self.bot, self.r_slew.step(r_cmd))
        return l_out, r_out

# ========================
# Rotation utility (tapered ~90°)
# ========================
def rotate_90(bot, wall_side):
    target_deg = 90.0
    sign = +1 if wall_side == "left" else -1  # +: CW, -: CCW
    start = bot.get_heading()
    while True:
        cur = bot.get_heading()
        delta = (cur - start + 540) % 360 - 180
        prog = abs(delta); rem = max(0.0, target_deg - prog)
        if rem <= 2.0:
            break
        scale = max(ROTATE_MIN_RPM/ROTATE_RPM, min(1.0, rem/target_deg))
        rpm = ROTATE_RPM * scale
        bot.set_left_motor_speed( sign * rpm)
        bot.set_right_motor_speed(-sign * rpm)
        time.sleep(DT)
    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)

# ========================
# Main
# ========================
if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60

    wall_side = "left"  # or "right"
    ctrl = WallFollower(Bot, wall_side=wall_side)

    try:
        while True:
            l_rpm, r_rpm = ctrl.step()
            Bot.set_left_motor_speed(l_rpm)
            Bot.set_right_motor_speed(r_rpm)

            # explicit rotate on front block
            f = front_mm(Bot)
            if f < FRONT_STOP_MM:
                Bot.set_left_motor_speed(0.0)
                Bot.set_right_motor_speed(0.0)
                rotate_90(Bot, wall_side)

            time.sleep(DT)
    except KeyboardInterrupt:
        Bot.set_left_motor_speed(0)
        Bot.set_right_motor_speed(0)