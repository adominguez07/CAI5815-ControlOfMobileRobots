from HamBot.src.robot_systems.robot import HamBot
from scipy.optimize import minimize
import numpy as np
import time

# ============================================================
# Tuneable parameters — calibrate on the physical maze
# ============================================================

FORWARD_SPEED   = 30          # motor % for forward travel
TURN_SPEED      = 25          # motor % for in-place turns
TURN_90_SEC     = 0.65        # seconds for a 90-degree turn (tune on actual floor)
SETTLE_SEC      = 0.15        # pause after motion before reading sensors
LIDAR_WIN       = 5           # half-window around scan index 180 for averaging
MAX_RANGE_MM    = 5000        # discard readings above this
LANDMARK_TOL    = 0.10        # color tolerance for camera detection

# Landmark positions in cm (continuous coordinates, origin = center of cell 13)
LANDMARKS = {
    'yellow': ((-150,  150), (255, 180,  0)),   # top-left corner
    'blue':   (( 150,  150), (  0,  80, 255)),   # top-right corner
    'green':  ((-150, -150), (  0, 180,  50)),   # bottom-left corner
    'pink':   (( 150, -150), (255,   0, 180)),   # bottom-right corner
}


# ============================================================
# Utility
# ============================================================

def get_front_distance_mm(bot):
    """Return average of valid LIDAR readings in a window around index 180 (front)."""
    scan = bot.get_range_image()
    vals = []
    for i in range(180 - LIDAR_WIN, 180 + LIDAR_WIN + 1):
        v = scan[i % 360]
        if v and 0 < v < MAX_RANGE_MM:
            vals.append(v)
    if not vals:
        return None
    return sum(vals) / len(vals)


def angle_diff(a, b):
    """Signed shortest angular distance from b to a (degrees)."""
    return (a - b + 180) % 360 - 180


def turn_to_heading(bot, target_deg):
    """
    IMU-guided turn: rotate in place until the robot faces target_deg.
    Uses short timed bursts and checks heading each iteration.
    """
    for _ in range(200):
        hdg = bot.get_heading()
        if hdg is None:
            time.sleep(0.05)
            continue
        err = angle_diff(target_deg, hdg)
        if abs(err) < 3.0:
            break
        # Proportional speed, clamped to a useful range
        spd = max(10, min(TURN_SPEED, 0.6 * abs(err)))
        if err > 0:   # need to turn left (CCW from robot perspective)
            bot.set_left_motor_speed(-spd)
            bot.set_right_motor_speed(spd)
        else:
            bot.set_left_motor_speed(spd)
            bot.set_right_motor_speed(-spd)
        time.sleep(0.05)
    bot.stop_motors()
    time.sleep(SETTLE_SEC)


def landmark_center_x(landmarks):
    """Return horizontal center of the largest detected landmark bounding box."""
    if not landmarks:
        return None
    biggest = max(landmarks, key=lambda lm: lm.width * lm.height)
    return biggest.x


def rotate_until_landmark_centered(bot, camera_width=640):
    """
    Spin slowly until the target landmark is roughly centered (±30 px).
    Returns True if centered, False if no landmark found after a full sweep.
    """
    center = camera_width // 2
    dead_zone = 30

    for _ in range(400):
        lms = bot.camera.find_landmarks()
        if lms:
            cx = landmark_center_x(lms)
            err = cx - center
            if abs(err) <= dead_zone:
                bot.stop_motors()
                return True
            # Small proportional correction
            spd = max(8, min(18, 0.08 * abs(err)))
            if err > 0:
                bot.set_left_motor_speed(spd)
                bot.set_right_motor_speed(-spd)
            else:
                bot.set_left_motor_speed(-spd)
                bot.set_right_motor_speed(spd)
        else:
            # No landmark in view — keep scanning
            bot.set_left_motor_speed(14)
            bot.set_right_motor_speed(-14)
        time.sleep(0.03)

    bot.stop_motors()
    return False


# ============================================================
# Trilateration
# ============================================================

def residuals(pos, landmark_positions, distances):
    """Sum of squared range residuals — objective for least-squares trilateration."""
    x, y = pos
    total = 0.0
    for (lx, ly), d in zip(landmark_positions, distances):
        total += (np.sqrt((x - lx) ** 2 + (y - ly) ** 2) - d) ** 2
    return total


def trilaterate(landmark_positions, distances):
    """Return (x, y) in cm that best fits the measured distances."""
    result = minimize(residuals, x0=[0.0, 0.0],
                      args=(landmark_positions, distances),
                      method='Nelder-Mead')
    return result.x


def xy_to_cell(x, y):
    """Map continuous (x, y) in cm to the 1-25 grid cell index."""
    col = int((x + 150) // 60)
    row = int((y + 150) // 60)
    col = max(0, min(4, col))
    row = max(0, min(4, row))
    return row * 5 + col + 1


# ============================================================
# Main routine
# ============================================================

def run_trilateration():
    bot = HamBot(lidar_enabled=True, camera_enabled=True)
    time.sleep(0.5)

    # Register all four landmark colors so find_landmarks() picks them up
    colors = [v[1] for v in LANDMARKS.values()]
    bot.camera.set_landmark_colors(colors, tolerance=LANDMARK_TOL)

    measured_positions = []   # (lx, ly) in cm for each measured landmark
    measured_distances = []   # distance in cm for each

    print("Starting landmark survey...")

    try:
        for name, ((lx, ly), rgb) in LANDMARKS.items():
            # Set the camera to look for only this color to avoid ambiguity
            bot.camera.set_landmark_colors([rgb], tolerance=LANDMARK_TOL)
            time.sleep(0.1)

            print(f"\nSearching for {name} landmark...")

            # Sweep until this landmark is centered
            found = rotate_until_landmark_centered(bot)

            if not found:
                print(f"  {name}: not found, skipping")
                continue

            time.sleep(SETTLE_SEC)

            # Read LIDAR distance to the landmark directly ahead
            dist_mm = get_front_distance_mm(bot)
            if dist_mm is None:
                print(f"  {name}: LIDAR reading invalid, skipping")
                continue

            dist_cm = dist_mm / 10.0
            measured_positions.append((lx, ly))
            measured_distances.append(dist_cm)
            print(f"  {name}: distance = {dist_cm:.1f} cm  (landmark at {lx}, {ly})")

        if len(measured_distances) < 3:
            print(f"\nOnly {len(measured_distances)} landmarks measured — need at least 3. Aborting.")
            return

        # Run trilateration
        est_x, est_y = trilaterate(measured_positions, measured_distances)
        cell = xy_to_cell(est_x, est_y)

        print(f"\nEstimated position: x = {est_x:.1f} cm, y = {est_y:.1f} cm")
        print(f"Estimated cell: {cell}")

    finally:
        bot.stop_motors()


if __name__ == '__main__':
    run_trilateration()
