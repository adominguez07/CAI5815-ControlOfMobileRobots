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
    turn(bot, 90)
    #move the length (east)
    bot.set_right_motor_speed(75)
    bot.set_left_motor_speed(75)
    time.sleep(length)
    #turn 90 degrees   
    turn(bot, 90)
    #width(facing south)
    bot.set_right_motor_speed(75)
    bot.set_left_motor_speed(75)
    time.sleep(width)
    #turn 90 degrees
    #length(facing west)
    turn(bot, 90)
    bot.set_right_motor_speed(75)
    bot.set_left_motor_speed(75)
    time.sleep(length)
    turn(bot, 90)
    #width(facing north)
    bot.set_right_motor_speed(50)
    bot.set_left_motor_speed(50)
    time.sleep(width/2)

def turn(bot, angle, rpm = 20):
    theta = math.radians(angle)
    #50 rpm approx 5.233 rad/s
    #rad/s * radius = velocity of the wheel
    #rpm / 75 *= 7.85
    angular_velocity_R = rpm / 75 * 7.85
    angular_velocity_L = rpm / 75 * 7.85
    V_right = angular_velocity_R * radius
    V_left = angular_velocity_L * radius
    V_right = -V_right

    angular_robot = (V_left - V_right) / wheelbase
    time_to_turn = theta / angular_robot

    bot.set_right_motor_speed(-rpm)
    bot.set_left_motor_speed(rpm)
    time.sleep(time_to_turn)

def CCW_circle(bot, radius_arc = 1):
    #velocity needed to complete in 5 sec
    angular_velocity_robot = 2 * math.pi / 25
    #velocity_robot = angular_velocity_robot * radius_arc
    velocity_right = angular_velocity_robot * (radius_arc + wheelbase/2)
    velocity_left = angular_velocity_robot * (radius_arc - wheelbase/2)
    angular_velocity_R = velocity_right / radius
    angular_velocity_L = velocity_left / radius
    rpm_R = angular_velocity_R / 7.85 * 75
    rpm_L = angular_velocity_L / 7.85 * 75
    T = 2 * math.pi/ angular_velocity_robot
    bot.set_right_motor_speed(rpm_R)
    bot.set_left_motor_speed(rpm_L)   
    time.sleep(T)
def CW_circle(bot, radius_arc = 0.5):
    angular_velocity_robot = 2 * math.pi / 25
    #velocity_robot = angular_velocity_robot * radius_arc
    velocity_right = angular_velocity_robot * (radius_arc - wheelbase/2)
    velocity_left = angular_velocity_robot * (radius_arc + wheelbase/2)
    angular_velocity_R = velocity_right / radius
    angular_velocity_L = velocity_left / radius
    rpm_R = angular_velocity_R / 7.85 * 75
    rpm_L = angular_velocity_L / 7.85 * 75
    T = 2 * math.pi / angular_velocity_robot
    bot.set_right_motor_speed(rpm_R)
    bot.set_left_motor_speed(rpm_L)   
    time.sleep(T)
    pass
def main():
    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    #CCW_circle(bot, 1)
    CW_circle(bot, 0.5)
    #rectangle(bot)
    #turn(bot, 90)
    bot.disconnect_robot()
    return 0


if __name__ == "__main__":
    main()