"""
Task 1 — Trilateration with HamBot sensors.

This version uses:
 - HamBot.camera.find_landmarks() for color detection.
 - HamBot.get_range_image() to grab a range reading toward each detected landmark.

Assumptions/notes:
 - Camera is forward-facing; horizontal FOV assumed ~62° (PiCamera v2).
 - Lidar frame: index 180 is robot-front, 90 is left, 270 is right, 0 is back.
 - Heading is not needed to extract the relative bearing for the landmark; we
   derive bearing from the pixel x-offset in the image.
 - Distances are kept in meters; lidar reports mm, so we convert.

If your setup has a different camera FOV or camera mounting, adjust CAM_FOV_DEG
or add an extra mounting offset.
"""
#hi
import math
import time
from typing import Dict, Tuple

from HamBot.src.robot_systems.robot import HamBot

GRID_SIZE = 4            # 4 x 4 grid
CELL_SIZE = 0.6          # 0.6 m (600 mm) per cell
CAM_FOV_DEG = 62.0       # Approx horizontal FOV of PiCamera v2

LANDMARK_POSITIONS = {
    "orange": (-1.2,  1.2),   # top-left
    "blue":   ( 1.2,  1.2),   # top-right
    "green":  (-1.2, -1.2),   # bottom-left
    "pink":   ( 1.2, -1.2),   # bottom-right
}


def normalize_color(color: Tuple[float, float, float]) -> Tuple[float, float, float]:
    r, g, b = color
    max_c = max(r, g, b)
    if max_c > 1.5:   # likely 0..255
        return r / 255.0, g / 255.0, b / 255.0
    return r, g, b


def classify_landmark_by_color(color: Tuple[float, float, float]) -> str | None:
    """
    Classify based on nearest target color in normalized RGB space.
    """
    r, g, b = normalize_color(color)
    targets = {
        "orange": (1.0, 0.6, 0.2),
        "blue":   (0.2, 0.4, 0.8),
        "green":  (0.2, 0.8, 0.5),
        "pink":   (0.8, 0.2, 0.4),  # magenta-ish
    }
    best = None
    best_dist = 1e9
    for name, (tr, tg, tb) in targets.items():
        dist = (r - tr) ** 2 + (g - tg) ** 2 + (b - tb) ** 2
        if dist < best_dist:
            best_dist = dist
            best = name
    # Simple gate: require reasonably close match
    if best_dist <= 0.25:  # tune if needed
        return best
    return None


def trilaterate_position(landmark_positions: Dict[str, Tuple[float, float]],
                         measurements: Dict[str, float]) -> Tuple[float, float]:
    names = [n for n in measurements.keys() if n in landmark_positions]
    if len(names) < 3:
        raise ValueError("Need at least 3 landmarks for trilateration")

    ref = names[0]
    x0, y0 = landmark_positions[ref]
    r0 = measurements[ref]

    A11 = A12 = A22 = b1 = b2 = 0.0
    for name in names[1:]:
        xi, yi = landmark_positions[name]
        ri = measurements[name]

        a_i = 2.0 * (xi - x0)
        b_i = 2.0 * (yi - y0)
        c_i = (xi * xi + yi * yi - ri * ri) - (x0 * x0 + y0 * y0 - r0 * r0)

        A11 += a_i * a_i
        A12 += a_i * b_i
        A22 += b_i * b_i
        b1  += a_i * c_i
        b2  += b_i * c_i

    det = A11 * A22 - A12 * A12
    if abs(det) < 1e-8:
        raise ValueError("Degenerate landmark configuration (det ~ 0)")

    x_est = ( b1 * A22 - b2 * A12) / det
    y_est = (-b1 * A12 + b2 * A11) / det
    return x_est, y_est


def position_to_cell_index(x: float, y: float,
                           grid_size: int = GRID_SIZE,
                           cell_size: float = CELL_SIZE) -> Tuple[int, int, int]:
    grid_center = (grid_size - 1) / 2.0  # e.g., 2 for 5x5
    col = int(round(x / cell_size + grid_center))      # 0..4
    row = int(round(grid_center - y / cell_size))      # 0..4 (y up)

    col = max(0, min(grid_size - 1, col))
    row = max(0, min(grid_size - 1, row))
    cell_index = row * grid_size + col + 1
    return cell_index, row, col


def pixel_to_lidar_index(px: int, img_width: int) -> int:
    """
    Map landmark pixel x-position to a lidar bearing index.
    Camera frame: x=0 is left; x=width is right.
    Lidar frame: 180 = forward, 90 = left, 270 = right.
    """
    if img_width <= 0:
        return 180
    offset_deg = ((px / img_width) - 0.5) * CAM_FOV_DEG  # left is negative
    lidar_angle = int(round((180 + offset_deg) % 360))
    return lidar_angle


def measure_landmark_distances(bot: HamBot,
                               min_area: int = 80,
                               max_range_m: float = 3.0,
                               debug: bool = False) -> Dict[str, float]:
    """
    Detect colored landmarks and fuse with lidar to estimate range.
    Returns a dict: {landmark_name: distance_m}.
    """
    cam = getattr(bot, "camera", None)
    if cam is None:
        print("Camera not available on HamBot object.")
        return {}

    scan = bot.get_range_image()
    frame = cam.get_frame(copy=False)
    detections = cam.find_landmarks(min_area=min_area)

    distances_m: Dict[str, float] = {}
    if frame is None or scan == -1 or not detections:
        return distances_m

    img_h, img_w = frame.shape[:2]
    for lm in detections:
        name = classify_landmark_by_color((lm.r, lm.g, lm.b))
        if not name:
            continue
        lidar_idx = pixel_to_lidar_index(lm.x, img_w)
        raw_mm = scan[lidar_idx] if 0 <= lidar_idx < len(scan) else -1
        if raw_mm is None or raw_mm <= 0:
            continue
        dist_m = raw_mm / 1000.0
        if dist_m > max_range_m:
            continue
        if name not in distances_m or dist_m < distances_m[name]:
            distances_m[name] = dist_m
        if debug:
            print(f"Detected {name} at pixel x={lm.x}, rgb=({lm.r},{lm.g},{lm.b}), "
                  f"lidar_idx={lidar_idx}, dist={dist_m:.2f}m")
    return distances_m


def rotate_and_collect(bot: HamBot,
                       measurements: Dict[str, float],
                       rpm: float = 9.0,
                       dt: float = 0.05,
                       max_time_s: float = 15.0):
    """
    Rotate in place once, collecting landmark distances as they enter view.
    Logic based on Lab3.rotate_360 to avoid premature stopping.
    """
    start_heading = bot.get_heading(blocking=True, wait_timeout=0.5)
    if start_heading is None:
        print("Cannot rotate: IMU heading unavailable.")
        return

    total_rotated = 0.0
    last_heading = start_heading
    t_start = time.time()

    print("Rotating 360° to collect landmarks...")
    while total_rotated < 360.0:

        cur_heading = bot.get_heading()
        if cur_heading is not None:
            delta = (cur_heading - last_heading + 180) % 360 - 180
            total_rotated += abs(delta)
            last_heading = cur_heading

        remaining = 360.0 - total_rotated
        if remaining <= 2.0:
            break

        # Slow slightly near the end for smoother stop
        scale = max(0.3, min(1.0, remaining / 360.0))
        rpm_scaled = rpm * scale
        bot.set_left_motor_speed(-rpm_scaled)
        bot.set_right_motor_speed(rpm_scaled)

        new_meas = measure_landmark_distances(bot, debug=False)
        for name, dist in new_meas.items():
            if name not in measurements or dist < measurements[name]:
                measurements[name] = dist

        time.sleep(dt)

    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)


def main():
    bot = HamBot(lidar_enabled=True, camera_enabled=True)
    time.sleep(2.0)  # let sensors warm up

    # Configure target landmark colors (RGB) and tolerance.
    target_colors = [
        (178, 16, 65),   # pink/mag
        (113, 177, 185),     #light blue
        (219, 120, 0),   # orange
        (255, 0, 183),  # unknown rn
    ]
    if getattr(bot, "camera", None):
        bot.camera.set_target_colors(target_colors, tolerance=0.25)

    start_time = time.time()
    measurements: Dict[str, float] = {}
    attempts = 0

    # Rotate once to sweep all landmarks into view.
    print(f"the robot heading is %d", bot.get_heading())
    rotate_and_collect(bot, measurements, rpm=9.0, dt=0.05, max_time_s=12.0)

    if len(measurements) < 3:
        print("ERROR: saw fewer than 3 landmarks, cannot trilaterate.")
        print("Visible landmarks:", measurements)
        return

    print("Measured landmark distances (m):")
    for name, d in measurements.items():
        print(f"  {name}: {d:.3f}")

    x_est, y_est = trilaterate_position(LANDMARK_POSITIONS, measurements)
    cell_index, row, col = position_to_cell_index(x_est, y_est)

    print("\n=== Trilateration Result ===")
    print(f"Estimated position: x = {x_est:.3f} m, y = {y_est:.3f} m")
    print(f"Estimated grid cell index: {cell_index} (row={row}, col={col})")

    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)


if __name__ == "__main__":
    main()