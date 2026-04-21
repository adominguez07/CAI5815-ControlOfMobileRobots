"""
U90971243 — Lab 4 Task 1: Trilateration with HamBot sensors.

Uses:
 - HamBot.camera.find_landmarks() for color detection.
 - HamBot.get_range_image() to grab a range reading toward each detected landmark.

Assumptions:
 - Camera is forward-facing; horizontal FOV assumed ~62° (PiCamera v2).
 - Lidar frame: index 180 = front, 90 = left, 270 = right, 0 = back.
 - Bearing to each landmark is derived from pixel x-offset in the image.
 - Distances kept in meters; lidar reports mm.
"""

import time
from typing import Dict, Tuple

from HamBot.src.robot_systems.robot import HamBot

# ============================================================
# Grid / camera constants
# ============================================================

GRID_SIZE    = 4       # 4 x 4 grid
CELL_SIZE    = 0.6     # 0.6 m per cell
CAM_FOV_DEG  = 62.0    # approximate horizontal FOV of PiCamera v2

# ============================================================
# Landmark map — (x, y) in meters, origin = grid center
# ============================================================

LANDMARK_POSITIONS = {
    "crimson": (-1.2,  1.2),   # top-left
    "teal":    ( 1.2,  1.2),   # top-right
    "green":   (-1.2, -1.2),   # bottom-left
    "orange":  ( 1.2, -1.2),   # bottom-right
}

# RGB colors the camera searches for (matched to physical markers)
TARGET_COLORS = [
    (8, 32, 145),   # top-left    — Blue
    (199,24,94),   # top-right   — red
    (245, 227,83),   # bottom-left — yello
    (34,159,128),   # bottom-right — green
]


# ============================================================
# Color classification (no numpy)
# ============================================================

def normalize_color(color: Tuple[float, float, float]) -> Tuple[float, float, float]:
    r, g, b = color
    max_c = max(r, g, b)
    if max_c > 1.5:   # 0..255 range
        return r / 255.0, g / 255.0, b / 255.0
    return r, g, b


def classify_landmark_by_color(color: Tuple[float, float, float]):
    """Return the landmark name whose target color is nearest in normalized RGB space."""
    r, g, b = normalize_color(color)
    targets = {
        "crimson": (0.698, 0.063, 0.255),
        "teal":    (0.443, 0.694, 0.725),
        "green":   (0.000, 0.706, 0.196),
        "orange":  (0.859, 0.471, 0.000),
    }
    best = None
    best_dist = 1e9
    for name, (tr, tg, tb) in targets.items():
        dist = (r - tr) ** 2 + (g - tg) ** 2 + (b - tb) ** 2
        if dist < best_dist:
            best_dist = dist
            best = name
    # Gate: reject if too far from every target (tune if needed)
    if best_dist <= 0.25:
        return best
    return None


# ============================================================
# Pure-Python trilateration (no scipy / numpy)
# ============================================================

def trilaterate_position(landmark_positions: Dict[str, Tuple[float, float]],
                         measurements: Dict[str, float]) -> Tuple[float, float]:
    """
    Least-squares trilateration solved analytically via normal equations.
    Requires at least 3 landmarks.
    """
    names = [n for n in measurements if n in landmark_positions]
    if len(names) < 3:
        raise ValueError(f"Need at least 3 landmarks, got {len(names)}")

    ref = names[0]
    x0, y0 = landmark_positions[ref]
    r0 = measurements[ref]

    A11 = A12 = A22 = b1 = b2 = 0.0
    for name in names[1:]:
        xi, yi = landmark_positions[name]
        ri = measurements[name]

        ai = 2.0 * (xi - x0)
        bi = 2.0 * (yi - y0)
        ci = (xi**2 + yi**2 - ri**2) - (x0**2 + y0**2 - r0**2)

        A11 += ai * ai
        A12 += ai * bi
        A22 += bi * bi
        b1  += ai * ci
        b2  += bi * ci

    det = A11 * A22 - A12 * A12
    if abs(det) < 1e-8:
        raise ValueError("Degenerate landmark configuration (landmarks may be collinear)")

    x_est = ( b1 * A22 - b2 * A12) / det
    y_est = (-b1 * A12 + b2 * A11) / det
    return x_est, y_est


# ============================================================
# Grid cell mapping
# ============================================================

def position_to_cell_index(x: float, y: float,
                            grid_size: int = GRID_SIZE,
                            cell_size: float = CELL_SIZE) -> Tuple[int, int, int]:
    grid_center = (grid_size - 1) / 2.0
    col = int(round(x / cell_size + grid_center))
    row = int(round(grid_center - y / cell_size))
    col = max(0, min(grid_size - 1, col))
    row = max(0, min(grid_size - 1, row))
    cell_index = row * grid_size + col + 1
    return cell_index, row, col


# ============================================================
# Sensor fusion helpers
# ============================================================

def pixel_to_lidar_index(px: int, img_width: int) -> int:
    """Map camera pixel x-position to a LIDAR bearing index."""
    if img_width <= 0:
        return 180
    offset_deg = ((px / img_width) - 0.5) * CAM_FOV_DEG
    lidar_angle = int(round((180 + offset_deg) % 360))
    return lidar_angle


def measure_landmark_distances(bot: HamBot,
                                min_area: int = 80,
                                min_range_m: float = 0.40,
                                max_range_m: float = 3.0,
                                lidar_window: int = 10,
                                debug: bool = False) -> Dict[str, float]:
    """
    Detect colored landmarks and fuse with LIDAR to estimate range.
    Uses the maximum reading in a window around the computed LIDAR angle
    to avoid hitting nearby walls instead of the actual landmark.
    Returns {landmark_name: distance_m}.
    """
    cam = getattr(bot, "camera", None)
    if cam is None:
        print("Camera not available.")
        return {}

    scan = bot.get_range_image()
    frame = cam.get_frame(copy=False)
    detections = cam.find_landmarks(min_area=min_area)

    distances_m: Dict[str, float] = {}
    if frame is None or scan == -1 or not detections:
        return distances_m

    img_w = frame.shape[1]
    for lm in detections:
        name = classify_landmark_by_color((lm.r, lm.g, lm.b))
        if not name:
            continue
        lidar_idx = pixel_to_lidar_index(lm.x, img_w)

        # Take the maximum valid reading in a window around the computed angle.
        # Max = farthest reading = most likely through a corridor to the marker,
        # not a close wall in the foreground.
        window_vals = []
        for i in range(lidar_idx - lidar_window, lidar_idx + lidar_window + 1):
            v = scan[i % 360]
            if v and v > 0:
                window_vals.append(v)
        if not window_vals:
            continue
        raw_mm = max(window_vals)

        dist_m = raw_mm / 1000.0
        if dist_m < min_range_m or dist_m > max_range_m:
            if debug:
                print(f"  {name}: REJECTED dist={dist_m:.2f} m (out of range), "
                      f"rgb=({lm.r},{lm.g},{lm.b})")
            continue
        if name not in distances_m or dist_m < distances_m[name]:
            distances_m[name] = dist_m
        if debug:
            print(f"  {name}: pixel x={lm.x}, lidar_idx={lidar_idx}, "
                  f"dist={dist_m:.2f} m, rgb=({lm.r},{lm.g},{lm.b})")
    return distances_m


# ============================================================
# 360-degree sweep
# ============================================================

def rotate_and_collect(bot: HamBot,
                       measurements: Dict[str, float],
                       rpm: float = 9.0,
                       dt: float = 0.05):
    """
    Rotate in place one full revolution, collecting landmark distances
    as they enter view. Keeps the best (closest) reading per landmark.
    """
    start_heading = bot.get_heading(blocking=True, wait_timeout=0.5)
    if start_heading is None:
        print("Cannot rotate: IMU heading unavailable.")
        return

    total_rotated = 0.0
    last_heading = start_heading

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

        scale = max(0.3, min(1.0, remaining / 360.0))
        bot.set_left_motor_speed(-rpm * scale)
        bot.set_right_motor_speed( rpm * scale)

        new_meas = measure_landmark_distances(bot, debug=True)
        for name, dist in new_meas.items():
            if name not in measurements or dist < measurements[name]:
                measurements[name] = dist

        time.sleep(dt)

    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)


# ============================================================
# Main
# ============================================================

def main():
    bot = HamBot(lidar_enabled=True, camera_enabled=True)
    time.sleep(2.0)   # let sensors warm up

    if getattr(bot, "camera", None):
        bot.camera.set_target_colors(TARGET_COLORS, tolerance=0.25)

    measurements: Dict[str, float] = {}

    print(f"Starting heading: {bot.get_heading()}")
    rotate_and_collect(bot, measurements, rpm=9.0, dt=0.05)

    print("\nMeasured landmark distances (m):")
    for name, d in measurements.items():
        print(f"  {name}: {d:.3f}")

    if len(measurements) < 3:
        print(f"\nERROR: only {len(measurements)} landmark(s) visible — need at least 3.")
        return

    x_est, y_est = trilaterate_position(LANDMARK_POSITIONS, measurements)
    cell_index, row, col = position_to_cell_index(x_est, y_est)

    print("\n=== Trilateration Result ===")
    print(f"Estimated position: x = {x_est:.3f} m, y = {y_est:.3f} m")
    print(f"Estimated grid cell: {cell_index}  (row={row}, col={col})")

    bot.set_left_motor_speed(0.0)
    bot.set_right_motor_speed(0.0)


if __name__ == "__main__":
    main()
