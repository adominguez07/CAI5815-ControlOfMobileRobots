from HamBot.src.robot_systems.robot import Hambot
import time

def main():
    bot = Hambot()
    bot.start()
    bot.left_wheel.set_speed(50)
    bot.right_wheel.set_speed(50)
    time.sleep(2)  # Run for 2 seconds
    bot.stop()
    
if __name__ == "__main__":
    main()