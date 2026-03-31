from robot_systems.robot import HamBot
import time

# Tunable constants
WALL_FOLLOW_SIDE      = 'right'  # 'left' for Run 1, 'right' for Run 2
OBSTACLE_THRESHOLD_MM = 300      # front distance that triggers wall-follow state
GOAL_DISTANCE_MM      = 250      # stop when this close to goal
DESIRED_WALL_MM       = 220      # target side-wall distance during wall-follow
BASE_SPEED            = 20       # nominal forward RPM
MAX_SPEED             = 35       # hard cap on any motor command
CAMERA_WIDTH          = 640      # horizontal resolution (pixels)
CAMERA_CENTER_X       = CAMERA_WIDTH // 2

# PD gains for wall following
KP_WALL = 0.15
KD_WALL = 0.05

# P gain for camera-based heading correction in motion-to-goal
KP_GOAL = 0.06


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def get_front_mm(scan):
    """Return the median of a +/-5 degree window around index 180 (forward).
    Guards against 0/inf readings from the RPLidar."""
    window = [scan[i] for i in range(175, 186) if scan[i] and scan[i] > 0]
    if not window:
        return float('inf')
    window.sort()
    return window[len(window) // 2]


def get_side_mm(scan, side):
    """Return the median of a +/-5 degree window for left (index 90) or right (index 270)."""
    center = 90 if side == 'left' else 270
    window = [scan[i] for i in range(center - 5, center + 6) if scan[i] and scan[i] > 0]
    if not window:
        return float('inf')
    window.sort()
    return window[len(window) // 2]


def run_bug_zero(wall_side=WALL_FOLLOW_SIDE):
    bot = HamBot(lidar_enabled=True, camera_enabled=True)
    bot.camera.set_target_colors([(234, 213, 45)], tolerance=0.08)
    time.sleep(0.5)  # let camera warm up

    state = 'MOTION_TO_GOAL'
    prev_wall_error = 0.0

    try:
        while True:
            scan      = bot.get_range_image()
            landmarks = bot.camera.find_landmarks()
            front     = get_front_mm(scan)

            # Stop when goal is visible and within reach
            if landmarks and front < GOAL_DISTANCE_MM:
                print("Goal reached, stopping.")
                bot.stop_motors()
                break

            if state == 'MOTION_TO_GOAL':
                if front < OBSTACLE_THRESHOLD_MM:
                    print("Obstacle detected, switching to WALL_FOLLOWING")
                    bot.stop_motors()
                    state = 'WALL_FOLLOWING'
                    prev_wall_error = 0.0
                    time.sleep(0.1)
                    continue

                if landmarks:
                    # Steer toward the largest visible landmark
                    lm = max(landmarks, key=lambda l: l.width * l.height)
                    error_x  = lm.x - CAMERA_CENTER_X
                    turn     = KP_GOAL * error_x
                    left_spd  = clamp(BASE_SPEED + turn, -MAX_SPEED, MAX_SPEED)
                    right_spd = clamp(BASE_SPEED - turn, -MAX_SPEED, MAX_SPEED)
                else:
                    # Goal not visible, drive straight
                    left_spd  = BASE_SPEED
                    right_spd = BASE_SPEED

                bot.set_left_motor_speed(left_spd)
                bot.set_right_motor_speed(right_spd)

            elif state == 'WALL_FOLLOWING':
                if landmarks:
                    print("Goal visible, switching to MOTION_TO_GOAL")
                    bot.stop_motors()
                    state = 'MOTION_TO_GOAL'
                    time.sleep(0.1)
                    continue

                side_dist       = get_side_mm(scan, wall_side)
                wall_error      = side_dist - DESIRED_WALL_MM
                d_error         = wall_error - prev_wall_error
                prev_wall_error = wall_error
                correction      = KP_WALL * wall_error + KD_WALL * d_error

                if front < OBSTACLE_THRESHOLD_MM:
                    # Front blocked during wall follow, turn in place to clear corner
                    if wall_side == 'left':
                        bot.set_left_motor_speed(BASE_SPEED)
                        bot.set_right_motor_speed(-BASE_SPEED)
                    else:
                        bot.set_left_motor_speed(-BASE_SPEED)
                        bot.set_right_motor_speed(BASE_SPEED)
                else:
                    if wall_side == 'left':
                        left_spd  = clamp(BASE_SPEED - correction, -MAX_SPEED, MAX_SPEED)
                        right_spd = clamp(BASE_SPEED + correction, -MAX_SPEED, MAX_SPEED)
                    else:
                        left_spd  = clamp(BASE_SPEED + correction, -MAX_SPEED, MAX_SPEED)
                        right_spd = clamp(BASE_SPEED - correction, -MAX_SPEED, MAX_SPEED)

                    bot.set_left_motor_speed(left_spd)
                    bot.set_right_motor_speed(right_spd)

            time.sleep(0.05)

    finally:
        bot.stop_motors()


if __name__ == '__main__':
    run_bug_zero()
