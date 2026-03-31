import math, time
import Lab2_Task2
from HamBot.src.robot_systems import camera
from HamBot.src.robot_systems.robot import HamBot
Lab2_Task2.CRUISE_RPM = 25
def first_scan(Bot):
    scan = Bot.get_range_image()
    # Get distances: left (90°), right (270°)
    
    # Define threshold for wall detection (in mm)
    WALL_THRESHOLD = 500  # Consider wall present if closer than 500mm
    
    # Check left wall (index around 90)
    left_window = [d for d in scan[85:95] if d and d > 0]
    left_dist = min(left_window) if left_window else float('inf')
    
    # Check right wall (index around 270)
    right_window = [d for d in scan[265:275] if d and d > 0]
    right_dist = min(right_window) if right_window else float('inf')
    
    print(f"Walls detected - Left: {left_dist:.0f}mm, Right: {right_dist:.0f}mm")
    
    # Decide which wall to follow based on which is closer
    if left_dist < WALL_THRESHOLD and right_dist < WALL_THRESHOLD:
        # Both walls present, follow the closer one
        if left_dist <= right_dist:
            print("Both walls detected. Following left wall (closer).")
            return "left"
        else:
            print("Both walls detected. Following right wall (closer).")
            return "right"
    elif left_dist < WALL_THRESHOLD:
        print("Left wall detected. Following left wall.")
        return "left"
    elif right_dist < WALL_THRESHOLD:
        print("Right wall detected. Following right wall.")
        return "right"
    else:
        print("No walls detected. Stopping.")
        exit()

def rotate_360(bot, direction="left", check_landmarks=None):

    sign = -1 if direction == "left" else +1
    start_heading = bot.get_heading()
    ROTATE_RPM = 9.0
    ROTATE_MIN_RPM = 5.0
    DT = 0.05
    CHECK_INTERVAL = 0.1
    total_rotated = 0.0
    last_heading = start_heading

    time.sleep(CHECK_INTERVAL)
    if check_landmarks is not None and check_landmarks():
        return True
    
    while total_rotated < 360.0:
        cur_heading = bot.get_heading()

        delta = (cur_heading - last_heading + 180) % 360 - 180
        total_rotated += abs(delta)
        last_heading = cur_heading

        remaining = 360.0 - total_rotated
        if remaining <= 2.0:
            break
        scale = max(ROTATE_MIN_RPM/ROTATE_RPM, min(1.0, remaining/360.0))
        rpm = ROTATE_RPM * scale

        bot.set_left_motor_speed(sign * rpm)
        bot.set_right_motor_speed(-sign * rpm)
        time.sleep(DT)

        if check_landmarks is not None and check_landmarks():
            # Stop motors
            bot.set_left_motor_speed(0.0)
            bot.set_right_motor_speed(0.0)
            return True
    
    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)
    return False

def Landmark_checking(bot):

    return rotate_360(bot, direction="left", 
                      check_landmarks=lambda: len(bot.camera.find_landmarks()) > 0)
if __name__ == "__main__":
    Bot = HamBot(lidar_enabled = True, camera_enabled = True)
    Bot.max_motor_speed = 60
    Bot.camera.set_target_colors((234, 213, 45))
    landmark_distance = 250


    if(Landmark_checking(Bot) == True):
        print("Landmark detected! Driving to target distance...")
        target = landmark_distance
        v = min(50, getattr(Bot, "max_motor_speed", 60))
        while True:
            scan = Bot.get_range_image()
            window = [a for a in scan[170:185] if a and a > 0]
            if not window:
                break
            actual = min(window)
            if actual <= target:
                break
            Bot.set_left_motor_speed(v)
            Bot.set_right_motor_speed(v)
            time.sleep(0.03)
        
        # Stop motors after reaching target
        Bot.set_left_motor_speed(0)
        Bot.set_right_motor_speed(0)
        print("Target reached! Program complete.")
        exit()
    
    # Scan for walls and determine which to follow
    wall_side = first_scan(Bot)
    ctrl = Lab2_Task2.WallFollower(Bot, wall_side=wall_side)

    try:
        while True:
            l_rpm, r_rpm = ctrl.step()
            Bot.set_left_motor_speed(l_rpm)
            Bot.set_right_motor_speed(r_rpm)
            # explicit rotate on front block
            f = Lab2_Task2.front_mm(Bot)
            if f < Lab2_Task2.FRONT_STOP_MM:
                Bot.set_left_motor_speed(0.0)
                Bot.set_right_motor_speed(0.0)
                
                # Check for landmarks when hitting a wall
                if Landmark_checking(Bot):
                    print("Landmark detected! Driving to target distance...")
                    target = landmark_distance
                    v = min(50, getattr(Bot, "max_motor_speed", 60))
                    while True:
                        scan = Bot.get_range_image()
                        window = [a for a in scan[175:180] if a and a > 0]
                        if not window:
                            break
                        actual = min(window)
                        if actual <= target:
                            break
                        Bot.set_left_motor_speed(v)
                        Bot.set_right_motor_speed(v)
                        time.sleep(0.03)
                    
                    # Stop motors after reaching target
                    Bot.set_left_motor_speed(0)
                    Bot.set_right_motor_speed(0)
                    print("Target reached! Program complete.")
                    exit()
                else:
                    # No landmark found, continue wall following
                    Lab2_Task2.rotate_90(Bot, wall_side)
                
            time.sleep(Lab2_Task2.DT)


    except KeyboardInterrupt:
        Bot.set_left_motor_speed(0)
        Bot.set_right_motor_speed(0)