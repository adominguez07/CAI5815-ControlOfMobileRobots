from HamBot.src.robot_systems.robot import HamBot
import time
import math
WHEEL_RADIUS = 0.045  # meters
TICKS_PER_REV = 960   # 20 CPR * 48:1 gearbox

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
        self.stop_band = 12

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
        cap = clamp(0.15 * abs(error), 10, 60)
        u = math.copysign(min(abs(u), cap), u)

        return saturation(bot, u), dist


# -----------------------------
# FAST IMU PID Controller
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

    pid.reset()

    # Determine desired rotation direction
    direction = 1 if delta_deg > 0 else -1

    while True:
        current = get_heading(bot)
        error = (target - current + 180) % 360 - 180

        if abs(error) <= pid.stop_band:
            bot.stop_motors()
            break

        derivative = (error - pid.prev_error) / pid.dt
        pid.prev_error = error

        turn = pid.Kp * error + pid.Kd * derivative

        # Force rotation direction
        turn = direction * abs(turn)

        cap = clamp(0.8 * abs(error), 8, 40)
        turn = direction * min(abs(turn), cap)

        # Apply motor command
        bot.set_left_motor_speed(-turn)
        bot.set_right_motor_speed(turn)

        time.sleep(pid.dt)

    bot.stop_motors()
    time.sleep(0.5)

def distance_to_ticks(distance_mm):
    distance_m = distance_mm / 1000.0
    wheel_circumference = 2 * math.pi * WHEEL_RADIUS
    revolutions = distance_m / wheel_circumference
    return revolutions * TICKS_PER_REV

def encoder_forward_2ft(bot):
    target_mm = 610
    target_ticks = distance_to_ticks(target_mm)

    bot.reset_encoders()

    Kp = 0.02
    Kd = 0.05
    dt = 0.02

    prev_error = 0
    stop_band = distance_to_ticks(8)

    while True:
        left = bot.get_left_encoder_reading()
        right = bot.get_right_encoder_reading()
        avg_ticks = (left + right) / 2

        error = target_ticks - avg_ticks

        print("AVG:", avg_ticks, "ERR:", error)

        if abs(error) <= stop_band:
            bot.stop_motors()
            break

        derivative = (error - prev_error) / dt
        prev_error = error

        u = Kp * error + Kd * derivative

        # MUCH stronger speed cap
        cap = clamp(0.02 * abs(error), 20, 60)
        u = math.copysign(min(abs(u), cap), u)

        bot.set_left_motor_speed(saturation(bot, u))
        bot.set_right_motor_speed(saturation(bot, u))

        time.sleep(dt)

    bot.stop_motors()
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

    print("\nTask 1: Stop at 2 ft")
    lidar_move(Bot, lidar_pid, TWO_FEET)

    print("\nTask 2: Move to 1 ft")
    lidar_move(Bot, lidar_pid, ONE_FOOT)

    print("\nTask 3: Backup to 2 ft")
    lidar_move(Bot, lidar_pid, TWO_FEET)

    print("\nTask 4: Rotate 180° clockwise")
    imu_rotate(Bot, imu_pid, -180)

    print("\nTask 5: Stop at 2 ft (repeat)")
    lidar_move(Bot, lidar_pid, TWO_FEET)

    print("\nTask 6: Move to 1 ft (repeat)")
    lidar_move(Bot, lidar_pid, ONE_FOOT)

    print("\nTask 7: Backup to 2 ft (repeat)")
    lidar_move(Bot, lidar_pid, TWO_FEET)

    print("\nTask 8: Rotate 180° counterclockwise")
    imu_rotate(Bot, imu_pid, 180)

    print("\nTask 9: Encoder PID drive forward 2 ft")
    encoder_forward_2ft(Bot)
    
    Bot.stop_motors()
    print("\nAll tasks complete.")