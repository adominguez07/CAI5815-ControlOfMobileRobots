from HamBot.src.robot_systems.robot import HamBot
import time, math

def saturation(bot, rpm):
    max_rpm = getattr(bot, "max_motor_speed", 60)
    if rpm > max_rpm:  return max_rpm
    if rpm < -max_rpm: return -max_rpm
    return rpm

class Defintions():
    def __init__(self):
        self.K_p = 0.10
        self.K_i = 0.15
        self.K_d = 1.5
        self.Timestep = 0.025
        self.Integral = 0.0
        self.PrevError = 0.0

        # NEW: gentle-stop helpers
        self.StopBand = 10.0           # mm (tune 8–15)
        self.I_Limit  = 200.0          # integral clamp
        self.ApproachSlope = 0.5       # rpm per mm (speed cap shrinks near target)
        self.MinApproachRPM = 6.0      # don't crawl too slowly far out

    def forward_PID(self, bot, desired_distance):
        # NOTE: use the passed-in bot, not the global Bot
        scan = bot.get_range_image()
        window = [a for a in scan[175:180] if a and a > 0]
        if not window:
            return 0.0

        measured_distance = min(window)
        error = measured_distance - desired_distance

        # NEW: clean stop when close, and prevent re-accel
        if abs(error) <= self.StopBand:
            self.Integral = 0.0
            self.PrevError = 0.0
            return 0.0

        # PID terms
        self.Integral += error * self.Timestep
        # NEW: anti-windup clamp
        self.Integral = max(-self.I_Limit, min(self.Integral, self.I_Limit))

        derivative = (error - self.PrevError) / self.Timestep
        self.PrevError = error

        u = (self.K_p * error) + (self.K_i * self.Integral) + (self.K_d * derivative)

        # NEW: soft approach – limit RPM based on how close you are
        cap = max(self.MinApproachRPM, self.ApproachSlope * abs(error))
        u = math.copysign(min(abs(u), cap), u)

        return saturation(bot, u)


if __name__ == "__main__":
    Bot = HamBot(lidar_enabled=True, camera_enabled=False)
    Bot.max_motor_speed = 60
    d_distance = 600
    pp = Defintions()

    while True:
        forward_distance = min([a for a in Bot.get_range_image()[175:180] if a > 0] or [float("inf")])
        forward_velocity = pp.forward_PID(Bot, d_distance)
        print("v=", forward_velocity, "dist=", forward_distance)
        # stop when inside the band
        if forward_distance != float("inf") and abs(forward_distance - d_distance) <= pp.StopBand:
            Bot.stop_motors()
            break

        Bot.set_left_motor_speed(forward_velocity)
        Bot.set_right_motor_speed(forward_velocity)
        time.sleep(pp.Timestep)

    #Encoder PID moving 300 mm forward
    Bot.reset_encoders()
    target_ticks = Bot.distance_to_ticks(300)
    while True:
        current_ticks = Bot.get_left_encoder_ticks()
        error = target_ticks - current_ticks
        if abs(error) <= Bot.ticks_to_distance(pp.StopBand):
            Bot.stop_motors()
            break

        # Simple P controller for encoder-based movement
        Kp_enc = 0.5  # Proportional gain for encoder control (tune as needed)
        u_enc = Kp_enc * error
        u_enc = saturation(Bot, u_enc)

        Bot.set_left_motor_speed(u_enc)
        Bot.set_right_motor_speed(u_enc)
        time.sleep(pp.Timestep)

    