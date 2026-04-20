from HamBot.src.robot_systems.robot import HamBot
from collections import Counter
import random
import time

# ============================================================
# Tuneable parameters — calibrate on the physical maze
# ============================================================

FORWARD_SPEED      = 30      # motor % for one-cell forward move
FORWARD_CELL_SEC   = 1.20    # seconds to travel one 60 cm cell (tune on actual floor)
TURN_SPEED         = 25      # motor % for in-place turns
TURN_90_SEC        = 0.65    # seconds for 90-degree turn (tune on actual floor)
SETTLE_SEC         = 0.20    # pause after any motion before reading LIDAR
WALL_THRESHOLD_MM  = 400     # LIDAR reading below this = wall present in that direction
MAX_RANGE_MM       = 5000    # discard readings above this

# ============================================================
# Grid / particle filter constants
# ============================================================

NUM_PARTICLES = 250
CELLS         = list(range(1, 26))
DIRECTIONS    = ['N', 'E', 'S', 'W']

# Wall map for Maze 2 — (N, E, S, W), 1 = wall present, 0 = open
# Verify this against the physical maze before running.
WALL_MAP = {
    1:  (0, 0, 1, 1),
    2:  (0, 0, 1, 0),
    3:  (0, 0, 1, 0),
    4:  (0, 0, 1, 0),
    5:  (0, 1, 1, 0),

    6:  (0, 0, 0, 1),
    7:  (1, 0, 0, 0),
    8:  (0, 0, 0, 0),
    9:  (0, 1, 0, 0),
    10: (0, 1, 0, 0),

    11: (0, 0, 0, 1),
    12: (0, 1, 1, 0),
    13: (0, 0, 1, 1),
    14: (0, 0, 0, 0),
    15: (0, 1, 0, 0),

    16: (0, 0, 0, 1),
    17: (0, 0, 0, 0),
    18: (1, 1, 0, 0),
    19: (1, 0, 0, 0),
    20: (0, 1, 0, 0),

    21: (1, 0, 0, 1),
    22: (1, 0, 0, 0),
    23: (1, 0, 0, 0),
    24: (1, 0, 0, 0),
    25: (1, 1, 0, 0),
}

# ============================================================
# Particle filter
# ============================================================

def init_particles():
    """Spread 250 particles evenly: 10 per cell, random facing direction."""
    particles = []
    for cell in CELLS:
        for _ in range(10):
            particles.append({'cell': cell, 'dir': random.choice(DIRECTIONS)})
    return particles


def get_neighbor(cell, direction):
    """Cell one step in `direction` from `cell`, or None if blocked by boundary."""
    row = (cell - 1) // 5
    col = (cell - 1) % 5
    deltas = {'N': (1, 0), 'S': (-1, 0), 'E': (0, 1), 'W': (0, -1)}
    dr, dc = deltas[direction]
    nr, nc = row + dr, col + dc
    if 0 <= nr < 5 and 0 <= nc < 5:
        return nr * 5 + nc + 1
    return None


def move_particle(particle, command):
    """Update particle state for a motion command (forward / left / right)."""
    cell   = particle['cell']
    facing = particle['dir']
    walls  = WALL_MAP[cell]
    wall_in_dir = {'N': walls[0], 'E': walls[1], 'S': walls[2], 'W': walls[3]}

    if command == 'forward':
        if wall_in_dir[facing] == 0:
            neighbor = get_neighbor(cell, facing)
            if neighbor:
                particle['cell'] = neighbor
    elif command == 'left':
        order = ['N', 'W', 'S', 'E']
        particle['dir'] = order[(order.index(facing) + 1) % 4]
    elif command == 'right':
        order = ['N', 'E', 'S', 'W']
        particle['dir'] = order[(order.index(facing) + 1) % 4]


def sensor_probability(observed_walls, cell):
    """
    Likelihood that a robot in `cell` would produce `observed_walls`.
    Sensor model: P(correct) = 0.9, P(error) = 0.1.
    """
    true_walls = WALL_MAP[cell]
    prob = 1.0
    for obs, true in zip(observed_walls, true_walls):
        if true == 1:
            prob *= 0.9 if obs == 1 else 0.1
        else:
            prob *= 0.1 if obs == 1 else 0.9
    return prob


def update_particle_filter(particles, command, observed_walls):
    """One full filter cycle: motion update → weight → resample."""
    for p in particles:
        move_particle(p, command)

    weights = [sensor_probability(observed_walls, p['cell']) for p in particles]

    total = sum(weights)
    if total == 0:
        weights = [1.0 / len(particles)] * len(particles)
    else:
        weights = [w / total for w in weights]

    new_particles = random.choices(particles, weights=weights, k=NUM_PARTICLES)
    return [{'cell': p['cell'], 'dir': random.choice(DIRECTIONS)} for p in new_particles]


def print_particle_state(particles):
    """Print a 5×5 grid of particle counts and return True when 80% converge."""
    counts = Counter(p['cell'] for p in particles)
    total  = len(particles)

    print("\nParticle distribution:")
    for row in range(4, -1, -1):
        print("".join(f"{counts.get(row * 5 + col + 1, 0):4d}" for col in range(5)))

    mode_cell, mode_count = counts.most_common(1)[0]
    pct = mode_count / total * 100
    print(f"\nBest estimate: cell {mode_cell} ({mode_count}/{total} particles, {pct:.1f}%)")

    if mode_count / total >= 0.80:
        print("Localization complete — 80% threshold reached.")
        return True
    return False


# ============================================================
# LIDAR wall reading
# ============================================================

def is_wall(scan, index, window=5):
    """Return 1 if the minimum valid reading in a window around `index` is below threshold."""
    readings = [scan[i % 360] for i in range(index - window, index + window + 1)]
    valid = [r for r in readings if r and 0 < r < MAX_RANGE_MM]
    if not valid:
        return 0
    return 1 if min(valid) < WALL_THRESHOLD_MM else 0


def read_walls_robot_frame(bot):
    """
    Return (front, right, back, left) wall flags relative to the robot body.
    LIDAR: 180 = front, 270 = right, 0 = back, 90 = left.
    """
    scan = bot.get_range_image()
    return (
        is_wall(scan, 180),   # front
        is_wall(scan, 270),   # right
        is_wall(scan, 0),     # back
        is_wall(scan, 90),    # left
    )


def rotate_to_compass(robot_walls, heading_deg):
    """
    Rotate robot-frame wall observations (front, right, back, left) to compass
    order (N, E, S, W) based on the robot's current heading.

    The IMU returns degrees from East, with startup direction treated as North (90°).
    We compute which compass direction the front of the robot faces and rotate accordingly.
    """
    front, right, back, left = robot_walls

    # Map heading to the nearest cardinal direction index: N=0, E=1, S=2, W=3
    compass = ['N', 'E', 'S', 'W']
    facing_idx = round(((90 - heading_deg) % 360) / 90) % 4

    # Robot-frame order starting at front and going clockwise: front, right, back, left
    robot_order = [front, right, back, left]

    # Rotate so that the robot's front maps to the correct compass index
    result = {}
    for offset, val in enumerate(robot_order):
        compass_dir = compass[(facing_idx + offset) % 4]
        result[compass_dir] = val

    return (result['N'], result['E'], result['S'], result['W'])


# ============================================================
# Robot motion primitives
# ============================================================

def move_forward_one_cell(bot):
    """Drive the robot forward one grid cell (~60 cm)."""
    bot.set_left_motor_speed(-FORWARD_SPEED)
    bot.set_right_motor_speed(FORWARD_SPEED)
    time.sleep(FORWARD_CELL_SEC)
    bot.stop_motors()
    time.sleep(SETTLE_SEC)


def turn_left(bot):
    """Rotate the robot 90 degrees counter-clockwise in place."""
    bot.set_left_motor_speed(-TURN_SPEED)
    bot.set_right_motor_speed(-TURN_SPEED)
    time.sleep(TURN_90_SEC)
    bot.stop_motors()
    time.sleep(SETTLE_SEC)


def turn_right(bot):
    """Rotate the robot 90 degrees clockwise in place."""
    bot.set_left_motor_speed(TURN_SPEED)
    bot.set_right_motor_speed(TURN_SPEED)
    time.sleep(TURN_90_SEC)
    bot.stop_motors()
    time.sleep(SETTLE_SEC)


# ============================================================
# Navigation decision
# ============================================================

def decide_next_move(bot):
    """
    Simple wall-following heuristic: prefer forward, then left, then right.
    This keeps the robot exploring without needing a global plan.
    """
    scan  = bot.get_range_image()
    front = is_wall(scan, 180)
    left  = is_wall(scan, 90)

    if not front:
        return 'forward'
    if not left:
        return 'left'
    return 'right'


def execute_command(bot, command):
    """Execute a motion command on the physical robot."""
    if command == 'forward':
        move_forward_one_cell(bot)
    elif command == 'left':
        turn_left(bot)
    elif command == 'right':
        turn_right(bot)


# ============================================================
# Main routine
# ============================================================

def run_particle_filter():
    bot = HamBot(lidar_enabled=True, camera_enabled=False)
    time.sleep(0.5)

    particles = init_particles()

    print("Particle filter started — 250 particles across 25 cells.")
    print("IMPORTANT: Verify WALL_MAP matches the physical maze before running!\n")

    try:
        while True:
            # Pick and execute the next motion
            command = decide_next_move(bot)
            print(f"Command: {command}")
            execute_command(bot, command)

            # Read walls in robot frame, then rotate to compass frame
            robot_walls   = read_walls_robot_frame(bot)
            heading       = bot.get_heading()
            compass_walls = rotate_to_compass(robot_walls, heading if heading else 90.0)

            print(f"Heading: {heading}°  |  Walls (N,E,S,W): {compass_walls}")

            # Update filter and check convergence
            particles = update_particle_filter(particles, command, compass_walls)
            done = print_particle_state(particles)
            if done:
                break

    finally:
        bot.stop_motors()


if __name__ == '__main__':
    run_particle_filter()
