from HamBot.src.robot_systems.robot import HamBot
import time
import math


# -----------------------------
# Helpers
# -----------------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    return clamp(rpm, -max_rpm, max_rpm)

def median(xs):
    xs = sorted(xs)
    n = len(xs)
    return xs[n // 2] if n else None


def get_front_lidar_mm(bot, center_deg=180, half_window=4):
    scan = bot.get_range_image()  # mm
    lo = max(0, center_deg - half_window)
    hi = min(359, center_deg + half_window)
    window = [v for v in scan[lo:hi + 1] if v and v > 0]
    if not window:
        return float("inf")
    return median(window)


def get_heading_deg(bot):
    """
    Tries common HamBot IMU/compass heading method names.
    Returns heading in degrees [0, 360).
    """
    candidates = [
        "get_heading",
        "get_heading_degrees",
        "get_compass_heading",
        "get_compass_heading_degrees",
        "get_imu_heading",
        "get_imu_heading_degrees",
        "get_yaw",
        "get_yaw_degrees",
    ]
    for name in candidates:
        if hasattr(bot, name):
            h = getattr(bot, name)()
            # normalize
            h = float(h) % 360.0
            return h
    raise AttributeError(
        "No heading function found on HamBot. Look in the repo for the IMU/compass API "
        "and update get_heading_deg() candidates."
    )


def angle_error_deg(target, current):
    """
    Smallest signed error target-current in degrees in [-180, 180]
    """
    e = (target - current + 180.0) % 360.0 - 180.0
    return e


class PID:
    def __init__(self, kp, ki, kd, dt, i_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.i = 0.0
        self.prev_e = 0.0
        self.i_limit = i_limit

    def reset(self):
        self.i = 0.0
        self.prev_e = 0.0

    def step(self, e):
        self.i += e * self.dt
        if self.i_limit is not None:
            self.i = clamp(self.i, -self.i_limit, self.i_limit)
        d = (e - self.prev_e) / self.dt
        self.prev_e = e
        return self.kp * e + self.ki * self.i + self.kd * d


# -----------------------------
# Task 1: LiDAR PID — Stop at the Wall (approach)
# -----------------------------
def lidar_pid_to_distance(bot, target_mm, dt=0.03, stop_band_mm=15, min_rpm=6, max_rpm=40):
    """
    Drives forward/backward (no rotation) until front LiDAR distance ~= target_mm.
    Speed ramps down near target; stops smoothly.
    """
    pid = PID(kp=0.12, ki=0.00, kd=0.90, dt=dt, i_limit=3000)

    while True:
        dist = get_front_lidar_mm(bot)

        # error positive => too far => move forward
        e = dist - target_mm

        if dist != float("inf") and abs(e) <= stop_band_mm:
            bot.stop_motors()
            return dist

        u = pid.step(e)  # rpm-ish command

        # soft cap near target to avoid overshoot + "creep forever"
        cap = clamp(0.06 * abs(e), min_rpm, max_rpm)   # rpm cap shrinks as you get close
        u = math.copysign(min(abs(u), cap), u)

        # deadband to prevent jerking creep
        if abs(u) < min_rpm:
            u = math.copysign(min_rpm, u)

        u = saturation(bot, u)
        bot.set_left_motor_speed(u)
        bot.set_right_motor_speed(u)
        time.sleep(dt)


# -----------------------------
# Task 2: Encoder PID — Drive Forward 0.5m / 1ft
# -----------------------------
# HamBot repo: 20 CPR motor * 48:1 gearbox = 960 ticks / wheel revolution
WHEEL_RADIUS_M = 0.045
TICKS_PER_WHEEL_REV = 960

def distance_mm_to_ticks(distance_mm):
    dist_m = distance_mm / 1000.0
    circ = 2.0 * math.pi * WHEEL_RADIUS_M
    revs = dist_m / circ
    return revs * TICKS_PER_WHEEL_REV

def encoder_pid_drive_forward(bot, distance_mm, dt=0.03, stop_band_mm=10, min_rpm=6, max_rpm=40):
    """
    Uses average of left/right encoder readings. Straight-line only.
    """
    bot.reset_encoders()
    pid = PID(kp=0.006, ki=0.000, kd=0.020, dt=dt, i_limit=5000)

    target_ticks = distance_mm_to_ticks(distance_mm)
    stop_ticks = distance_mm_to_ticks(stop_band_mm)

    while True:
        left = bot.get_left_encoder_reading()
        right = bot.get_right_encoder_reading()
        avg = (left + right) / 2.0

        e = target_ticks - avg  # ticks remaining

        if abs(e) <= stop_ticks:
            bot.stop_motors()
            return avg

        u = pid.step(e)

        # limit and deadband
        u = clamp(u, -max_rpm, max_rpm)
        if abs(u) < min_rpm:
            u = math.copysign(min_rpm, u)

        u = saturation(bot, u)
        bot.set_left_motor_speed(u)
        bot.set_right_motor_speed(u)
        time.sleep(dt)


# -----------------------------
# Task 3: LiDAR PID — Backup to target distance (no rotation)
# -----------------------------
def lidar_pid_backup_to_distance(bot, target_mm, dt=0.03, stop_band_mm=15, min_rpm=6, max_rpm=35):
    """
    Assumes robot is too close. Drives backward only until front distance ~= target_mm.
    """
    pid = PID(kp=0.12, ki=0.00, kd=0.90, dt=dt, i_limit=3000)

    while True:
        dist = get_front_lidar_mm(bot)
        e = dist - target_mm  # negative when too close

        if dist != float("inf") and abs(e) <= stop_band_mm:
            bot.stop_motors()
            return dist

        u = pid.step(e)

        # Force backward if too close (e < 0) and avoid any forward creep
        if e < 0:
            u = -abs(u)
        else:
            u = 0.0  # if somehow too far, don't move forward in "backup" task

        cap = clamp(0.06 * abs(e), min_rpm, max_rpm)
        u = -min(abs(u), cap)

        if abs(u) < min_rpm:
            u = -min_rpm

        u = saturation(bot, u)
        bot.set_left_motor_speed(u)
        bot.set_right_motor_speed(u)
        time.sleep(dt)


# -----------------------------
# Task 4: IMU/Compass PID — Rotate Clockwise 180°
# -----------------------------
def imu_pid_rotate_clockwise(bot, delta_deg=180.0, dt=0.03, stop_band_deg=2.0, min_rpm=8, max_rpm=35):
    """
    Rotate in place clockwise by delta_deg.
    Uses heading feedback; slows near target and stops smoothly.
    """
    start = get_heading_deg(bot)
    target = (start - delta_deg) % 360.0  # clockwise decreases heading (common convention)
    pid = PID(kp=0.9, ki=0.00, kd=0.25, dt=dt, i_limit=200)

    while True:
        cur = get_heading_deg(bot)
        e = angle_error_deg(target, cur)  # signed shortest error

        if abs(e) <= stop_band_deg:
            bot.stop_motors()
            return (start, target, cur)

        u = pid.step(e)

        # For clockwise-only rotation: if we're "past" target direction, still allow correction
        # but prefer clockwise motion when |e| is large.
        # Convert to turn command: positive u => rotate to increase heading (CCW)
        # We want to drive based on sign of e: if e is negative, rotate clockwise.
        turn = clamp(u, -max_rpm, max_rpm)

        # shrink cap near target
        cap = clamp(0.8 * abs(e), min_rpm, max_rpm)
        turn = math.copysign(min(abs(turn), cap), turn)

        if abs(turn) < min_rpm:
            turn = math.copysign(min_rpm, turn)

        # Differential drive rotate in place:
        # CCW: left negative, right positive
        # CW : left positive, right negative
        left_cmd = -turn
        right_cmd = turn

        bot.set_left_motor_speed(saturation(bot, left_cmd))
        bot.set_right_motor_speed(saturation(bot, right_cmd))
        time.sleep(dt)


# -----------------------------
# Run Lab Tasks
# -----------------------------
if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60

    # Choose units: 1 m or 2 ft
    # 1 m = 1000 mm
    # 2 ft ≈ 609.6 mm
    wall_target_mm = 1000  # change to 610 for 2 ft

    # Task 1: LiDAR stop at wall target distance
    print("\nTASK 1: Approach and stop at", wall_target_mm, "mm")
    final_dist = lidar_pid_to_distance(Bot, wall_target_mm)
    print("Task 1 done. Final front distance (mm):", final_dist)
    time.sleep(1)

    # Task 2: Encoder drive forward 0.5 m or 1 ft
    drive_target_mm = 500  # change to 305 for 1 ft
    print("\nTASK 2: Encoder drive forward", drive_target_mm, "mm")
    final_ticks = encoder_pid_drive_forward(Bot, drive_target_mm)
    print("Task 2 done. Avg ticks:", final_ticks)
    time.sleep(1)

    # Task 3: LiDAR backup to wall target distance (assumes too close)
    print("\nTASK 3: Backup to", wall_target_mm, "mm")
    final_dist = lidar_pid_backup_to_distance(Bot, wall_target_mm)
    print("Task 3 done. Final front distance (mm):", final_dist)
    time.sleep(1)

    # Task 4: IMU/Compass rotate clockwise 180°
    print("\nTASK 4: Rotate clockwise 180°")
    s, t, c = imu_pid_rotate_clockwise(Bot, 180.0)
    print("Task 4 done. start/target/current:", s, t, c)

    Bot.stop_motors()