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
    # Try common heading function names in HamBot
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
# LiDAR PID
# -----------------------------
class LidarPID:
    def __init__(self):
        self.Kp = 0.12
        self.Kd = 0.8
        self.dt = 0.03
        self.prev_error = 0.0
        self.stop_band = 15  # mm

    def reset(self):
        self.prev_error = 0.0

    def compute(self, bot, target_mm):
        distance = get_front_distance(bot)
        error = distance - target_mm

        if distance != float("inf") and abs(error) <= self.stop_band:
            self.prev_error = 0
            return 0.0, distance

        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        u = self.Kp * error + self.Kd * derivative

        cap = clamp(0.06 * abs(error), 6, 40)
        u = math.copysign(min(abs(u), cap), u)

        return saturation(bot, u), distance


# -----------------------------
# IMU PID for Rotation
# -----------------------------
class IMUPID:
    def __init__(self):
        self.Kp = 0.9
        self.Kd = 0.25
        self.dt = 0.03
        self.prev_error = 0.0
        self.stop_band = 2  # degrees

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

        cap = clamp(0.8 * abs(error), 8, 35)
        u = math.copysign(min(abs(u), cap), u)

        return saturation(bot, u), current


# -----------------------------
# Main Program
# -----------------------------
if __name__ == "__main__":

    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60

    lidar_pid = LidarPID()
    imu_pid = IMUPID()

    # =========================
    # TASK 1: Stop at 2 ft
    # =========================
    print("\n--- Task 1: Move to 2 ft (610 mm) ---")
    target_2ft = 610

    while True:
        velocity, dist = lidar_pid.compute(Bot, target_2ft)
        print("2ft | v:", velocity, "| dist:", dist)

        if velocity == 0:
            Bot.stop_motors()
            break

        Bot.set_left_motor_speed(velocity)
        Bot.set_right_motor_speed(velocity)
        time.sleep(lidar_pid.dt)

    time.sleep(1)
    lidar_pid.reset()

    # =========================
    # TASK 2: Move to 1 ft
    # =========================
    print("\n--- Task 2: Move to 1 ft (305 mm) ---")
    target_1ft = 305

    while True:
        velocity, dist = lidar_pid.compute(Bot, target_1ft)
        print("1ft | v:", velocity, "| dist:", dist)

        if velocity == 0:
            Bot.stop_motors()
            break

        Bot.set_left_motor_speed(velocity)
        Bot.set_right_motor_speed(velocity)
        time.sleep(lidar_pid.dt)

    time.sleep(1)
    lidar_pid.reset()

    # =========================
    # TASK 3: Backup to 2 ft
    # =========================
    print("\n--- Task 3: Backup to 2 ft ---")

    while True:
        velocity, dist = lidar_pid.compute(Bot, target_2ft)

        # Force backward motion if too close
        if dist < target_2ft:
            velocity = -abs(velocity)

        print("Backup | v:", velocity, "| dist:", dist)

        if velocity == 0:
            Bot.stop_motors()
            break

        Bot.set_left_motor_speed(velocity)
        Bot.set_right_motor_speed(velocity)
        time.sleep(lidar_pid.dt)

    time.sleep(1)
    lidar_pid.reset()

    # =========================
    # TASK 4: Rotate 180° Clockwise
    # =========================
    print("\n--- Task 4: Rotate 180° Clockwise ---")

    start_heading = get_heading(Bot)
    target_heading = (start_heading - 180) % 360

    while True:
        turn_speed, heading = imu_pid.compute(Bot, target_heading)
        print("Rotate | turn:", turn_speed, "| heading:", heading)

        if turn_speed == 0:
            Bot.stop_motors()
            break

        # Clockwise rotation
        Bot.set_left_motor_speed(-turn_speed)
        Bot.set_right_motor_speed(turn_speed)
        time.sleep(imu_pid.dt)

    Bot.stop_motors()
    print("\nAll tasks complete.")