from HamBot.src.robot_systems.robot import HamBot
import time

def main():
    bot = HamBot(lidar_enabled=False, camera_enabled=False)
    
    bot.set_left_motor_speed(50)
    bot.set_right_motor_speed(50)
    time.sleep(2)  # Run for 2 seconds
    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)
    return 0
if __name__ == "__main__":
    main()