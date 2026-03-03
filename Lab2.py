from HamBot.src.robot_systems.robot import HamBot
import time
import math


def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    return max(-max_rpm, min(max_rpm, rpm))


class LidarController:
    def __init__(self):
        self.Kp = 0.12
        self.Kd = 0.8
        self.dt = 0.03
        self.prev_error = 0.0
        self.stop_band = 15  # mm tolerance

    def get_front_distance(self, bot):
        scan = bot.get_range_image()
        window = [a for a in scan[175:185] if a > 0]
        if not window:
            return float("inf")
        return min(window)

    def forward_control(self, bot, target_mm):
        distance = self.get_front_distance(bot)
        error = distance - target_mm

        if abs(error) <= self.stop_band:
            self.prev_error = 0
            return 0.0, distance

        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        u = self.Kp * error + self.Kd * derivative
        return saturation(bot, u), distance


if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60

    controller = LidarController()

    # ===== STEP 1: STOP 2 FEET AWAY =====
    target_2ft = 610  # mm

    while True:
        u, dist = controller.forward_control(Bot, target_2ft)
        print("2ft stage | v:", u, "dist:", dist)

        if u == 0:
            Bot.stop_motors()
            break

        Bot.set_left_motor_speed(u)
        Bot.set_right_motor_speed(u)
        time.sleep(controller.dt)

    time.sleep(1)

    # ===== STEP 2: STOP 1 FOOT AWAY =====
    target_1ft = 305  # mm

    while True:
        u, dist = controller.forward_control(Bot, target_1ft)
        print("1ft stage | v:", u, "dist:", dist)

        if u == 0:
            Bot.stop_motors()
            break

        Bot.set_left_motor_speed(u)
        Bot.set_right_motor_speed(u)
        time.sleep(controller.dt)

    time.sleep(1)

    # ===== STEP 3: TURN 180° =====
    print("Turning 180 degrees")

    turn_speed = 30
    turn_time = 2.1   # adjust slightly if needed

    Bot.set_left_motor_speed(turn_speed)
    Bot.set_right_motor_speed(-turn_speed)

    time.sleep(turn_time)
    Bot.stop_motors()

    print("Done.")