
import time, random
from dataclasses import dataclass
from collections import Counter
import numpy as np
from robot_systems.robot import HamBot




#GRID 
GRID_SIZE = 4
NUM_CELLS = GRID_SIZE * GRID_SIZE  # 16

N_PARTICLES = 160  # 10 per cell

# LIDAR returns mm
LIDAR_MM_TO_M = 1.0 / 1000.0

SECTOR_HALF_WIDTH = 8  # degrees around center ray

#PF observation threshold
WALL_OBS_THRESH_M = 0.55  # slightly increased for 60cm cells  

#driving safety thresholds 
FRONT_STOP_M         = 0.30   # increased for more stopping distance
FRONT_DIAG_ENTER_M   = 0.42
FRONT_DIAG_EXIT_M    = 0.48
FRONT_DIAG_STOP_M    = 0.25

FRONT_LEFT_DIAG_DEG  = 135
FRONT_RIGHT_DIAG_DEG = 225

#motion tuning 
FWD_RPM       = 30   # reduced from 35 for safer movement
MIN_RAMP_RPM  = 15   # reduced from 18
RAMP_TIME_S   = 0.8
CTRL_DT       = 0.05

TURN_RPM      = 25
TURN_TIME_S   = 1.25  

RAD_PER_CELL  = 10   # tuned for 60cm cells
MAX_FWD_TIME_S = 8.0  # more time to complete longer distance

# Drift correction (encoders)
ENC_KP   = 8.0

# Side-centering correction
SIDE_KP  = 0.8   # reduced to prevent over-correction

#PF sensor model (lab spec)
P_Z1_S1 = 0.8
P_Z0_S1 = 0.2
P_Z1_S0 = 0.4
P_Z0_S0 = 0.6

# Stable convergence requirement
STABLE_CONV_STEPS = 1  # stop immediately at 80% per lab spec



# 4x4 MAZE MAP (N,E,S,W)
# Grid layout:
#  1  2  3  4
#  5  6  7  8
#  9 10 11 12
# 13 14 15 16
MAZE_MAP = {
 1:  (1,0,0,1),
 2:  (1,0,1,0),
 3:  (1,0,1,0),
 4:  (1,1,0,0),

 5:  (0,1,0,1),
 6:  (1,0,1,1),
 7:  (1,0,1,0),
 8:  (0,1,1,0),

 9:  (0,0,0,1),
10:  (1,0,1,0),
11:  (1,1,1,0),
12:  (1,1,0,1),

13:  (0,0,1,1),
14:  (1,0,1,0),
15:  (1,0,1,0),
16:  (0,1,1,0),
}


#PARTICLES 

ORIENTATIONS = ["N", "E", "S", "W"]
ORIENT_TO_VEC = {
    "N": (0, -1),  # (dc, dr)
    "E": (1, 0),
    "S": (0, 1),
    "W": (-1, 0),
}
LEFT_TURN  = {"N": "W", "W": "S", "S": "E", "E": "N"}
RIGHT_TURN = {"N": "E", "E": "S", "S": "W", "W": "N"}

@dataclass
class Particle:
    cell: int
    orient: str

def init_particles_even():
    parts = []
    per_cell = N_PARTICLES // NUM_CELLS  # 10 each, remainder added random
    for cell in range(1, NUM_CELLS+1):
        for _ in range(per_cell):
            parts.append(Particle(cell, random.choice(ORIENTATIONS)))
    while len(parts) < N_PARTICLES:
        parts.append(Particle(random.randint(1, NUM_CELLS),
                              random.choice(ORIENTATIONS)))
    return parts


#MAP UTILS

def cell_to_rc(cell):
    r, c = divmod(cell - 1, GRID_SIZE)
    return r, c

def rc_to_cell(r, c):
    return r * GRID_SIZE + c + 1

def in_bounds(r, c):
    return 0 <= r < GRID_SIZE and 0 <= c < GRID_SIZE

def wall_in_direction(cell, direction):
    N, E, S, W = MAZE_MAP[cell]
    return {"N": N, "E": E, "S": S, "W": W}[direction]


#MOTION UPDATE 

def motion_update(particles, command):
    """
    Perfect deterministic model:
      F: move one cell forward if no wall
      L/R: rotate in place
      S: stay (used when forward aborts early)
    """
    new_parts = []
    for p in particles:
        if command == "S":
            new_parts.append(Particle(p.cell, p.orient))

        elif command == "L":
            new_parts.append(Particle(p.cell, LEFT_TURN[p.orient]))

        elif command == "R":
            new_parts.append(Particle(p.cell, RIGHT_TURN[p.orient]))

        elif command == "F":
            if wall_in_direction(p.cell, p.orient) == 1:
                new_parts.append(Particle(p.cell, p.orient))
            else:
                r, c = cell_to_rc(p.cell)
                dc, dr = ORIENT_TO_VEC[p.orient]
                nr, nc = r + dr, c + dc
                if in_bounds(nr, nc):
                    new_parts.append(Particle(rc_to_cell(nr, nc), p.orient))
                else:
                    new_parts.append(Particle(p.cell, p.orient))
        else:
            raise ValueError("command must be F/L/R/S")
    return new_parts


#SENSOR UTILITIES

def median_sector(scan, center_deg):
    idxs = [(center_deg + d) % 360 for d in range(-SECTOR_HALF_WIDTH, SECTOR_HALF_WIDTH+1)]
    vals = [scan[i] for i in idxs if scan[i] > 0]
    if not vals:
        return None
    vals.sort()
    return vals[len(vals)//2] * LIDAR_MM_TO_M

def heading_to_orient(heading_deg):
    if heading_deg is None:
        return None
    cardinals = [(0,"E"), (90,"N"), (180,"W"), (270,"S"), (360,"E")]
    best = min(cardinals, key=lambda x: abs((heading_deg - x[0] + 180) % 360 - 180))
    return best[1]

def get_wall_observation(robot):
    scan = robot.get_range_image()
    if scan == -1:
        raise RuntimeError("LIDAR not enabled.")

    heading = robot.get_heading(fresh_within=0.5, blocking=True, wait_timeout=1.0)
    orient = heading_to_orient(heading)
    if orient is None:
        raise RuntimeError("IMU heading unavailable.")

    d_front = median_sector(scan, 180)
    d_right = median_sector(scan, 270)
    d_left  = median_sector(scan, 90)
    d_back  = median_sector(scan, 0)

    def is_wall(d):
        return 1 if (d is not None and d <= WALL_OBS_THRESH_M) else 0

    z_robot = {
        "front": is_wall(d_front),
        "right": is_wall(d_right),
        "back":  is_wall(d_back),
        "left":  is_wall(d_left),
    }

    if orient == "N":
        z_global = {"N": z_robot["front"], "E": z_robot["right"], "S": z_robot["back"], "W": z_robot["left"]}
    elif orient == "E":
        z_global = {"E": z_robot["front"], "S": z_robot["right"], "W": z_robot["back"], "N": z_robot["left"]}
    elif orient == "S":
        z_global = {"S": z_robot["front"], "W": z_robot["right"], "N": z_robot["back"], "E": z_robot["left"]}
    else:  # "W"
        z_global = {"W": z_robot["front"], "N": z_robot["right"], "E": z_robot["back"], "S": z_robot["left"]}

    return (z_global["N"], z_global["E"], z_global["S"], z_global["W"]), z_robot

def get_wall_observation_majority(robot, samples=3, dt=0.12):
    zs = []
    for _ in range(samples):
        z, _ = get_wall_observation(robot)
        zs.append(z)
        time.sleep(dt)
    arr = np.array(zs, dtype=int)
    maj = (arr.sum(axis=0) >= (samples/2)).astype(int)
    return tuple(int(x) for x in maj)

def likelihood(z, s):
    if s == 1 and z == 1: return P_Z1_S1
    if s == 1 and z == 0: return P_Z0_S1
    if s == 0 and z == 1: return P_Z1_S0
    return P_Z0_S0

def sensor_update(particles, observation):
    zN,zE,zS,zW = observation
    w = np.zeros(len(particles), dtype=float)
    for i,p in enumerate(particles):
        sN,sE,sS,sW = MAZE_MAP[p.cell]
        w[i] = (likelihood(zN,sN)*likelihood(zE,sE)*
                likelihood(zS,sS)*likelihood(zW,sW))
    total = w.sum()
    if total == 0:
        w[:] = 1.0/len(w)
    else:
        w /= total
    return w


#RESAMPLING 

def resample_particles(particles, weights):
    idxs = np.random.choice(len(particles), size=len(particles), p=weights)
    new_parts = [Particle(particles[j].cell, random.choice(ORIENTATIONS)) for j in idxs]
    return new_parts


#PRINTING / STOP CHECK
def particle_counts_grid(particles):
    counts = Counter(p.cell for p in particles)
    grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
    for cell,k in counts.items():
        r,c = cell_to_rc(cell)
        grid[r,c] = k
    return grid, counts

def print_grid(grid):
    print("\nParticle count grid (row 0 = top):")
    for r in range(GRID_SIZE):
        print(" ".join(f"{grid[r,c]:3d}" for c in range(GRID_SIZE)))

def mode_cell_and_fraction(counts):
    mode_cell = max(counts.keys(), key=lambda c: counts[c])
    mode_count = counts[mode_cell]
    frac = mode_count / N_PARTICLES
    return mode_cell, mode_count, frac


#AUTONOMOUS DRIVING

# Turn tuning
TURN_TOLERANCE_DEG = 5.0   # degrees tolerance for target heading
TURN_TIMEOUT_S = 4.0       # max time to complete turn

def normalize_angle(angle):
    """Normalize angle to 0-360 range."""
    while angle < 0:
        angle += 360
    while angle >= 360:
        angle -= 360
    return angle

def angle_diff(target, current):
    """Shortest signed difference between two angles."""
    diff = target - current
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff

def execute_turn(robot, cmd):
    """Execute 90-degree turn using IMU heading feedback."""
    # Get current heading
    start_heading = robot.get_heading(fresh_within=0.5, blocking=True, wait_timeout=1.0)
    if start_heading is None:
        # Fallback to time-based if IMU unavailable
        print("WARN: IMU unavailable, using time-based turn")
        if cmd == "R":
            robot.set_left_motor_speed(TURN_RPM)
            robot.set_right_motor_speed(-TURN_RPM)
        else:
            robot.set_left_motor_speed(-TURN_RPM)
            robot.set_right_motor_speed(TURN_RPM)
        time.sleep(TURN_TIME_S)
        robot.stop_motors()
        time.sleep(0.2)
        robot.reset_encoders()
        time.sleep(0.1)
        return
    
    # Calculate target heading
    # R = clockwise = heading decreases (in this IMU: -90)
    # L = counter-clockwise = heading increases (in this IMU: +90)
    if cmd == "R":
        target_heading = normalize_angle(start_heading - 90)
    else:
        target_heading = normalize_angle(start_heading + 90)
    
    # Motor directions (same as time-based fallback)
    # R: left forward, right backward = clockwise
    # L: left backward, right forward = counter-clockwise
    if cmd == "R":
        robot.set_left_motor_speed(TURN_RPM)
        robot.set_right_motor_speed(-TURN_RPM)
    else:
        robot.set_left_motor_speed(-TURN_RPM)
        robot.set_right_motor_speed(TURN_RPM)
    
    start_time = time.time()
    
    while True:
        # Check timeout
        if time.time() - start_time > TURN_TIMEOUT_S:
            print("WARN: turn timeout")
            break
        
        # Get current heading
        current_heading = robot.get_heading(fresh_within=0.2, blocking=True, wait_timeout=0.5)
        if current_heading is None:
            time.sleep(0.05)
            continue
        
        # Check if we've reached target
        diff = abs(angle_diff(target_heading, current_heading))
        if diff <= TURN_TOLERANCE_DEG:
            break
        
        time.sleep(0.02)
    
    robot.stop_motors()
    time.sleep(0.2)
    robot.reset_encoders()
    time.sleep(0.1)

def do_forward_one_cell(robot):

    if not hasattr(do_forward_one_cell, "slowing"):
        do_forward_one_cell.slowing = False
    if not hasattr(do_forward_one_cell, "fld_hits"):
        do_forward_one_cell.fld_hits = 0
    if not hasattr(do_forward_one_cell, "frd_hits"):
        do_forward_one_cell.frd_hits = 0

    robot.reset_encoders()
    start_time = time.time()
    aborted_reason = None
    avg_rad = 0.0

    while True:
        l_rad, r_rad = robot.get_encoder_readings()
        avg_rad = (abs(l_rad) + abs(r_rad)) / 2.0

        if avg_rad >= RAD_PER_CELL:
            break

        if time.time() - start_time > MAX_FWD_TIME_S:
            print("WARN: forward move timeout hit.")
            aborted_reason = "timeout"
            break

        scan = robot.get_range_image()
        if scan == -1:
            print("WARN: LIDAR unavailable during forward.")
            aborted_reason = "lidar"
            break

        d_front = median_sector(scan, 180)
        d_left  = median_sector(scan, 90)
        d_right = median_sector(scan, 270)
        d_fld   = median_sector(scan, FRONT_LEFT_DIAG_DEG)
        d_frd   = median_sector(scan, FRONT_RIGHT_DIAG_DEG)

        if d_front is not None and d_front <= FRONT_STOP_M:
            print("EMERGENCY STOP: front wall too close.")
            aborted_reason = "front"
            break

        # consecutive diag hits
        if d_fld is not None and d_fld <= FRONT_DIAG_STOP_M:
            do_forward_one_cell.fld_hits += 1
        else:
            do_forward_one_cell.fld_hits = 0

        if d_frd is not None and d_frd <= FRONT_DIAG_STOP_M:
            do_forward_one_cell.frd_hits += 1
        else:
            do_forward_one_cell.frd_hits = 0

        if do_forward_one_cell.fld_hits >= 2:
            print("EMERGENCY STOP: front-left diag too close (persistent).")
            aborted_reason = "diagL"
            break
        if do_forward_one_cell.frd_hits >= 2:
            print("EMERGENCY STOP: front-right diag too close (persistent).")
            aborted_reason = "diagR"
            break

        # ramp
        t = time.time() - start_time
        ramp_frac = min(1.0, t / RAMP_TIME_S)
        base = max(MIN_RAMP_RPM, FWD_RPM * ramp_frac)

        # drift correction
        drift_err = (l_rad - r_rad)
        drift_corr = ENC_KP * drift_err

        # side centering
        side_corr = 0.0
        if d_left is not None and d_right is not None:
            side_err = d_left - d_right
            side_corr = SIDE_KP * side_err

        # diagonal slowdown + steering
        diag_corr = 0.0
        diag_slow = 1.0
        if d_fld is not None and d_frd is not None:
            min_diag = min(d_fld, d_frd)

            if do_forward_one_cell.slowing:
                if min_diag > FRONT_DIAG_EXIT_M:
                    do_forward_one_cell.slowing = False
            else:
                if min_diag < FRONT_DIAG_ENTER_M:
                    do_forward_one_cell.slowing = True

            if do_forward_one_cell.slowing:
                diag_slow = max(0.7, min_diag / FRONT_DIAG_ENTER_M)

            diag_err = d_fld - d_frd
            diag_corr = (SIDE_KP * 0.8) * diag_err

        total_corr = drift_corr + side_corr + diag_corr
        base *= diag_slow

        robot.set_left_motor_speed(base - total_corr)
        robot.set_right_motor_speed(base + total_corr)

        time.sleep(CTRL_DT)

    robot.stop_motors()
    time.sleep(0.2)

    # reset hit counters after any abort
    if aborted_reason is not None:
        do_forward_one_cell.fld_hits = 0
        do_forward_one_cell.frd_hits = 0
        do_forward_one_cell.slowing = False

    # if aborted before 90% of cell, treat as NO MOVE
    if aborted_reason is not None and avg_rad < 0.90 * RAD_PER_CELL:
        return False, aborted_reason

    return True, "ok"


def choose_autonomous_command(z_robot, last_cmd=None):
    front_clear = (z_robot["front"] == 0)
    left_clear  = (z_robot["left"] == 0)
    right_clear = (z_robot["right"] == 0)

    if front_clear:
        return "F"

    opts = []
    if left_clear:  opts.append("L")
    if right_clear: opts.append("R")

    if opts:
        if last_cmd in opts and len(opts) > 1:
            opts.remove(last_cmd)
        return random.choice(opts)

    return random.choice(["L", "R"])


#MAIN LOOP 

def run_particle_filter():
    print("Initializing Bot")
    robot = HamBot(lidar_enabled=True, camera_enabled=False)
    time.sleep(2.0)

    particles = init_particles_even()
    last_cmd = None
    stable = 0

    try:
        step = 0
        while True:
            step += 1
            print(f"\n FILTER STEP {step} ")

            #observe for policy
            _, z_robot = get_wall_observation(robot)

            #autonomous cmd
            cmd = choose_autonomous_command(z_robot, last_cmd=last_cmd)
            print(f"Autonomous command chosen: {cmd}")
            last_cmd = cmd

            #execute physically and get ACTUAL cmd (with recovery)
            if cmd == "F":
                moved_full, reason = do_forward_one_cell(robot)

                if moved_full:
                    actual_cmd = "F"

                else:
                    if reason == "diagR":
                        recovery_turn = "L"  # right diag too close -> turn left
                    elif reason == "diagL":
                        recovery_turn = "R"  # left diag too close -> turn right
                    elif reason == "front":
                        recovery_turn = random.choice(["L", "R"])
                    else:
                        recovery_turn = None

                    if recovery_turn is not None:
                        execute_turn(robot, recovery_turn)
                        actual_cmd = recovery_turn  # PF must match what really happened
                        last_cmd = recovery_turn  # prevent choosing same turn twice
                    else:
                        actual_cmd = "S"  # just stay if weird abort
            else:
                execute_turn(robot, cmd)
                actual_cmd = cmd

            # 3) PF prediction with ACTUAL cmd
            particles = motion_update(particles, actual_cmd)

            # 4) PF correction
            obs = get_wall_observation_majority(robot, samples=3)
            weights = sensor_update(particles, obs)

            # 5) resample
            particles = resample_particles(particles, weights)

            # 6) report
            grid, counts = particle_counts_grid(particles)
            print_grid(grid)

            mode_cell, mode_count, frac = mode_cell_and_fraction(counts)
            print(f"\nMode cell estimate: {mode_cell} with {mode_count}/{N_PARTICLES} particles ({frac*100:.1f}%)")
            print(f"Converged? {'YES' if frac >= 0.80 else 'NO'}")

            # stable convergence
            if frac >= 0.80:
                stable += 1
            else:
                stable = 0

            if stable >= STABLE_CONV_STEPS:
                print(f"\n SUCCESS: ≥80% for {STABLE_CONV_STEPS} consecutive steps.")
                break

    finally:
        print("\nShutting down HamBot...")
        robot.disconnect_robot()


if __name__ == "__main__":
    run_particle_filter()