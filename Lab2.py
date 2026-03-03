from HamBot.src.robot_systems.robot import HamBot
import time
import math

radius = 0.045
wheelbase = 0.184

#use Lidar and PID to approach the wall and maintain a distance of 0.5 m from the wall

def approach_wall(bot, distance = 0.5):
    Kp = 1
    Ki = 0.1
    Kd = 0.05
    error_sum = 0
    last_error = 0

    while True:
        #get distance from Lidar
        current_distance = bot.get_range_image()
        current_distance = current_distance[180] #get the distance in front of the robot
        error = distance - current_distance
        error_sum += error
        error_diff = error - last_error

        #PID control
        control_signal = Kp * error + Ki * error_sum + Kd * error_diff

        #set motor speeds based on control signal
        bot.set_right_motor_speed(control_signal)
        bot.set_left_motor_speed(control_signal)

        last_error = error

        #break the loop if the robot is close enough to the wall
        if abs(error) < 0.01:
            break

def main():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)

    import time
    time.sleep(3)   # let lidar warm up

    # Force clear input buffer
    bot.lidar.lidar.clear_input()

    for i in range(5):
        scan = bot.get_range_image()
        print(scan[:20])   # print first 20 values
        time.sleep(1)


if __name__ == "__main__":
    main()