from HamBot.src.robot_systems.robot import HamBot
import time
import math

radius = 0.045
wheelbase = 0.184
def rectangle(bot, length = 2, width = 4):
    #width is the starting side
    #robot start
    #move half of the width
    bot.set_right_motor_speed(50)
    bot.set_left_motor_speed(50)
    time.sleep(width/2)
    #turn 90 degrees
    bot.set_right_motor_speed(-50)
    bot.set_left_motor_speed(50)
    time.sleep(.5)  # Adjust sleep time as needed
    #move the length (east)
    bot.set_right_motor_speed(75)
    bot.set_left_motor_speed(75)
    time.sleep(length)
    #turn 90 degrees    bot.set_right_motor_speed(-50)
    bot.set_left_motor_speed(50)
    bot.set_right_motor_speed(-50)
    time.sleep(.5)
    #width(facing south)
    bot.set_right_motor_speed(75)
    bot.set_left_motor_speed(75)
    time.sleep(width)
    #turn 90 degrees
    bot.set_right_motor_speed(-50)  
    bot.set_left_motor_speed(50)
    time.sleep(.5)
    #length(facing west)
    bot.set_right_motor_speed(75)
    bot.set_left_motor_speed(75)
    time.sleep(length)
    #turn 90 degrees
    bot.set_right_motor_speed(-50)
    bot.set_left_motor_speed(50)
    time.sleep(.5)
    #width(facing north)
    bot.set_right_motor_speed(50)
    bot.set_left_motor_speed(50)
    time.sleep(width/2)

def turn(bot, angle, rpm = 50):
    theta = math.radians(angle)
    #50 rpm approx 5.233 rad/s
    #rad/s * radius = velocity of the wheel
    #rpm / 75 *= 7.85
    angular_velocity_R = rpm / 75 * 7.85
    angular_velocity_L = rpm / 75 * 7.85
    V_right = angular_velocity_R * radius
    V_left = angular_velocity_L * radius

    angular_robot = (V_right - V_left) / wheelbase
    time_to_turn = theta / angular_robot

    bot.set_right_motor_speed(-rpm)
    bot.set_left_motor_speed(rpm)
    time.sleep(time_to_turn)

def CCW_circle(bot, radius = 1):
    pass
def CW_circle(bot, radius = 2):
    pass
def main():
    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    #rectangle(bot)
    turn(bot, 90)
    bot.disconnect_robot()
    return 0


if __name__ == "__main__":
    main()