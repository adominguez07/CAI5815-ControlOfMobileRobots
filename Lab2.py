from HamBot.src.robot_systems.robot import HamBot
import time, math


def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    return max(-max_rpm, min(max_rpm, rpm))


# ======== Encoder Constants from HamBot Repo ========
WHEEL_RADIUS = 0.045  # meters
TICKS_PER_REV = 960   # 20 CPR * 48:1 gearbox


def distance_to_ticks(distance_mm):
    distance_m = distance_mm / 1000.0
    wheel_circumference = 2 * math.pi * WHEEL_RADIUS
    revolutions = distance_m / wheel_circumference
    return revolutions * TICKS_PER_REV


def ticks_to_distance(ticks):
    wheel_circumference = 2 * math.pi * WHEEL_RADIUS
    revolutions = ticks / TICKS_PER_REV
    return revolutions * wheel_circumference * 1000  # mm


class Definitions():
    def __init__(self):
        self.K_p = 0.10
        self.K_i = 0.15
        self.K_d = 1.5
        self.Timestep = 0.025
        self.Integral = 0.0
        self.PrevError = 0.0

        self.StopBand = 10.0
        self.I_Limit = 200.0
        self.ApproachSlope = 0.5
        self.MinApproachRPM = 6.0

    def forward_PID(self, bot, desired_distance):
        scan = bot.get_range_image()
        window = [a for a in scan[175:180] if a and a > 0]
        if not window:
            return 0.0

        measured_distance = min(window)
        error = measured_distance - desired_distance

        if abs(error) <= self.StopBand:
            self.Integral = 0.0
            self.PrevError = 0.0
            return 0.0

        self.Integral += error * self.Timestep
        self.Integral = max(-self.I_Limit, min(self.Integral, self.I_Limit))

        derivative = (error - self.PrevError) / self.Timestep
        self.PrevError = error

        u = (self.K_p * error) + (self.K_i * self.Integral) + (self.K_d * derivative)

        cap = max(self.MinApproachRPM, self.ApproachSlope * abs(error))
        u = math.copysign(min(abs(u), cap), u)

        return saturation(bot, u)


if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60

    desired_distance = 600  # mm
    controller = Definitions()

    # ===== LIDAR WALL APPROACH =====
    while True:
        forward_distance = min(
            [a for a in Bot.get_range_image()[175:180] if a > 0] or [float("inf")]
        )

        forward_velocity = controller.forward_PID(Bot, desired_distance)

        print("v=", forward_velocity, "dist=", forward_distance)

        if forward_distance != float("inf") and \
           abs(forward_distance - desired_distance) <= controller.StopBand:
            Bot.stop_motors()
            break

        Bot.set_left_motor_speed(forward_velocity)
        Bot.set_right_motor_speed(forward_velocity)
        time.sleep(controller.Timestep)

    # ===== ENCODER MOVE FORWARD 300 mm =====
    Bot.reset_encoders()

    target_ticks = distance_to_ticks(300)

    while True:
        left_ticks = Bot.get_left_encoder_ticks()
        error = target_ticks - left_ticks

        if abs(ticks_to_distance(error)) <= controller.StopBand:
            Bot.stop_motors()
            break

        Kp_enc = 0.003  # scaled down because ticks are large now
        u_enc = saturation(Bot, Kp_enc * error)

        Bot.set_left_motor_speed(u_enc)
        Bot.set_right_motor_speed(u_enc)

        time.sleep(controller.Timestep)