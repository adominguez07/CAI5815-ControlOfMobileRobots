from HamBot.src.robot_systems.robot import HamBot
import time
import math


# -----------------------------
# Utility Functions
# -----------------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    return clamp(rpm, -max_rpm, max_rpm)


def get_front_distance(bot):
    scan = bot.get_range_image()
    window = [v for v in scan[175:185] if v and v > 0]
    if not window:
        return float("inf")
    window.sort()
    return window[len(window) // 2]


def get_heading(bot):
    for name in [
        "get_heading",
        "get_heading_degrees",
        "get_compass_heading",
        "get_imu_heading",
        "get_yaw",
    ]:
        if hasattr(bot, name):
            return float(getattr(bot, name)()) % 360
    raise AttributeError("No heading function found on HamBot.")


def angle_error(target, current):
    return (target - current + 180) % 360 - 180


# -----------------------------
# FAST LiDAR PID Controller
# -----------------------------
class LidarPID:
    def __init__(self):
        self.Kp = 0.20
        self.Kd = 1.2
        self.dt = 0.02
        self.prev_error = 0.0
        self.stop_band = 12  # tighter stop

    def reset(self):
        self.prev_error = 0.0

    def compute(self, bot, target_mm):
        dist = get_front_distance(bot)
        error = dist - target_mm

        if dist != float("inf") and abs(error) <= self.stop_band:
            self.prev_error = 0
            return 0.0, dist

        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        u = self.Kp * error + self.Kd * derivative

        # Faster speed ramp
        cap = clamp(0.15 * abs(error), 10, 60)
        u = math.copysign(min(abs(u), cap), u)

        return saturation(bot, u), dist


# -----------------------------
# IMU PID Controller (faster)
# -----------------------------
class IMUPID:
    def __init__(self):
        self.Kp = 1.2
        self.Kd = 0.35
        self.dt = 0.02
        self.prev_error = 0.0
        self.stop_band = 2

    def reset(self):
        self.prev_error = 0.0

    def compute(self, bot, target_heading):
        current = get_heading(bot)
        error = angle_error(target_heading, current)

        if abs(error) <= self.stop_band:
            self.prev_error = 0
            return 0.0, current

        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        u = self.Kp * error + self.Kd * derivative
        cap = clamp(1.0 * abs(error), 12, 60)
        u = math.copysign(min(abs(u), cap), u)

        return saturation(bot, u), current


# -----------------------------
# Helper Functions
# -----------------------------
def lidar_move(bot, pid, target_mm):
    while True:
        vel, dist = pid.compute(bot, target_mm)
        print("LiDAR | target:", target_mm, "| v:", vel, "| dist:", dist)

        if vel == 0:
            bot.stop_motors()
            break

        bot.set_left_motor_speed(vel)
        bot.set_right_motor_speed(vel)
        time.sleep(pid.dt)

    pid.reset()
    time.sleep(0.5)


def imu_rotate(bot, pid, delta_deg):
    start = get_heading(bot)
    target = (start + delta_deg) % 360

    while True:
        turn, heading = pid.compute(bot, target)
        print("Rotate | target:", target, "| heading:", heading, "| turn:", turn)

        if turn == 0:
            bot.stop_motors()
            break

        bot.set_left_motor_speed(-turn)
        bot.set_right_motor_speed(turn)
        time.sleep(pid.dt)

    pid.reset()
    time.sleep(0.5)


# -----------------------------
# Main Program
# -----------------------------
if __name__ == "__main__":

    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60

    lidar_pid = LidarPID()
    imu_pid = IMUPID()

    TWO_FEET = 610
    ONE_FOOT = 305

    # Tasks 1–3
    print("\nTask 1: Stop at 2 ft")
    lidar_move(Bot, lidar_pid, TWO_FEET)

    print("\nTask 2: Move to 1 ft")
    lidar_move(Bot, lidar_pid, ONE_FOOT)

    print("\nTask 3: Backup to 2 ft")
    lidar_move(Bot, lidar_pid, TWO_FEET)

    # Task 4
    print("\nTask 4: Rotate 180° clockwise")
    imu_rotate(Bot, imu_pid, -180)

    # Tasks 5–7
    print("\nTask 5: Stop at 2 ft (repeat)")
    lidar_move(Bot, lidar_pid, TWO_FEET)

    print("\nTask 6: Move to 1 ft (repeat)")
    lidar_move(Bot, lidar_pid, ONE_FOOT)

    print("\nTask 7: Backup to 2 ft (repeat)")
    lidar_move(Bot, lidar_pid, TWO_FEET)

    # Task 8
    print("\nTask 8: Rotate 180° counterclockwise")
    imu_rotate(Bot, imu_pid, 180)

    # Task 9
    print("\nTask 9: Final forward drive (2 ft)")
    lidar_move(Bot, lidar_pid, ONE_FOOT)

    Bot.stop_motors()
    print("\nAll tasks complete.")