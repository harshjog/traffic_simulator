import numpy as np
import matplotlib.pyplot as plt
import random
from math import ceil, floor
import matplotlib.animation as animation
from matplotlib import colors
from win32comext.adsi.demos.scp import verbose
from vehicle_arrangement_v3_config import *

# Module-level override for parameter sweeps
_p_vehicle_class_override = None

# -----------------------------
# Utility functions
# -----------------------------
def draw_width_m():
    # Use override if set, otherwise use config value
    p_class = _p_vehicle_class_override if _p_vehicle_class_override is not None else p_vehicle_class
    
    if random.random() < p_class:
        # type 2 (cars)
        mu, sigma, w_min, w_max = mu_w2, sigma_w2, w2_min, w2_max
    else:
        # type 1 (bikes)
        mu, sigma, w_min, w_max = mu_w1, sigma_w1, w1_min, w1_max
    
    # truncated normal
    while True:
        w = (random.gauss(mu, sigma)) # 
        if w_min <= w <= w_max:
            return w

def width_to_cells(w_m):
    return max(1, int(round(w_m / cell_size)))

# -----------------------------
# Vehicle class
# -----------------------------
class Vehicle:
    _id = 0
    def __init__(self, x_front):
        self.id = Vehicle._id; Vehicle._id += 1
        self.width_m = draw_width_m()
        self.w_cells = width_to_cells(self.width_m)
        self.length_cells = int(round(4.5 / cell_size))  # ~4.5 m vehicle length
        self.x_front = x_front  # front cell index (0 at start)
        self.speed = v_max
        self.y_center = random.choice(lane_centers)  + int(random.gauss(0, 1))
        self.attempted_lateral_move = False
        self.attempted_move_step_counter = 0  # counts how many steps to be attempted
        self.attempted_move_direction = 0  # -1 left, 0 none, +1 right
        

    def lateral_bounds(self):
        half = self.w_cells // 2
        if self.w_cells % 2 == 1:
            left = self.y_center - half
            right = self.y_center + half
        else:
            left = self.y_center - half
            right = self.y_center + half - 1
        return max(0, left), min(W_cells-1, right) #they might hang out of bounds but that's ok

# -----------------------------
# Grid
# -----------------------------
def empty_grid():
    return -np.ones((L_cells, W_cells), dtype=int)  # store vehicle id or -1

def is_position_clear(grid, veh, candidate_y, lon_min, lon_max, Cmin_cells, W_cells, veh_id_to_ignore=None):
    """
    Check if a candidate lateral position is clear of obstacles.
    
    Args:
        grid: The occupancy grid
        veh: Vehicle object
        candidate_y: Candidate y_center position to test
        lon_min, lon_max: Longitudinal bounds to check
        Cmin_cells: Clearance margin in cells
        W_cells: Total grid width
        veh_id_to_ignore: Vehicle ID to treat as empty (default: veh.id)
    
    Returns:
        bool: True if position is clear, False otherwise
    """
    if veh_id_to_ignore is None:
        veh_id_to_ignore = veh.id
    
    half = veh.w_cells // 2
    if veh.w_cells % 2 == 1:
        left = candidate_y - half
        right = candidate_y + half
    else:
        left = candidate_y - half
        right = candidate_y + half - 1
    
    col_min = max(0, left - Cmin_cells)
    col_max = min(W_cells - 1, right + Cmin_cells)
    
    subrect = grid[lon_min:lon_max+1, col_min:col_max+1]
    if subrect.size == 0:
        return False
    
    tmp = subrect.copy()
    tmp[tmp == veh_id_to_ignore] = -1
    return np.all(tmp == -1)

# occupancy test
def place_vehicle_on_grid(grid, veh):
    left, right = veh.lateral_bounds()
    rear = veh.x_front - veh.length_cells + 1
    if rear < 0 or veh.x_front >= L_cells:
        return False
    # check occupancy
    if np.any(grid[rear:veh.x_front+1, left:right+1] != -1):
        return False
    grid[rear:veh.x_front+1, left:right+1] = veh.id
    return True

def clear_vehicle_from_grid(grid, veh):
    grid[grid == veh.id] = -1



# -----------------------------
# Simulation
# -----------------------------
def simulate(seed=0, p_lateral_change_override=None, p_vehicle_class_override=None):
    global _p_vehicle_class_override
    _p_vehicle_class_override = p_vehicle_class_override
    
    random.seed(seed); np.random.seed(seed)
    
    # Use override if provided, otherwise use config value
    p_lateral_change_val = p_lateral_change_override if p_lateral_change_override is not None else p_lateral_change
    
    vehicles = []
    grid = empty_grid()
    flows = []
    lateral_centers = []
    lateral_centers_evol = []
    grid_history = []

    for t in range(T):
        # 1) spawn with probability at x_front = 0+(length-1)
        #print(grid)

       
        if random.random() < spawn_prob:
            v = Vehicle(x_front = v.length_cells - 1 if (v := None) else 0)  # small hack: create then set x
            # set initial x_front near start
            v.x_front = v.length_cells - 1
            # place if free
            if place_vehicle_on_grid(grid, v):
                vehicles.append(v)
            else:
                # could not spawn — skip
                Vehicle._id -= 1


        # 2) longitudinal update: simple Nagel-Schreckenberg
        vehicles.sort(key=lambda V: V.x_front, reverse=True)  # update from frontmost to backmost
        # Step 1: Decide next states based on *current* grid
        grid_snapshot = grid.copy()
        for veh in vehicles:
            # clear old position to test moves
            old_x = veh.x_front
            veh.prev_x = old_x
            veh.prev_y = veh.y_center
            old_left, old_right = veh.lateral_bounds()
            old_center = veh.y_center
            old_speed = veh.speed
            veh.old_speed = old_speed
            
            #clear_vehicle_from_grid(grid_snapshot, veh)

            # compute gap to next vehicle ahead in same lane-band area: approximate by checking cells ahead within vehicle's lateral bounds
            front_search = old_x+1
            gap = 0
            while front_search < L_cells and np.all(grid_snapshot[front_search, old_left:old_right+1] == -1):
                gap += 1
                front_search += 1
                if gap >= v_max:
                    break
            # speed update
            veh.speed = min(old_speed + 1, v_max, gap)
            if random.random() < p_slow:
                veh.speed = max(0, veh.speed - 1)
            # tentative x
            veh.x_front = min(L_cells-1, old_x + veh.speed)
        
        # # 3) lateral decisions 

            # require Cmin_cells clearance around the vehicle in both lateral and longitudinal directions
            left_bound = max(Cmin_cells + veh.w_cells//2, 0 + veh.w_cells//2)
            right_bound = min(W_cells - 1 - (veh.w_cells//2), W_cells-1 - Cmin_cells - veh.w_cells//2)
            best = old_center
            best_score = 0
            # vehicle longitudinal span (rear .. front)
            rear = max(0, veh.x_front - veh.length_cells + 1)
            # check a longitudinal margin of Cmin_cells in front and behind
            lon_min = max(0, rear - Cmin_cells)
            lon_max = min(L_cells - 1, veh.x_front + Cmin_cells)

            prev_move_attempted = getattr(veh, "attempted_lateral_move", False)

            if veh.attempted_lateral_move == False:
                # decide whether this vehicle will attempt a lateral shift this tick
                p_attempt_move = random.random()
                if p_attempt_move < p_lateral_change_val:
                    search_range = lateral_search_cells
                    veh.attempted_lateral_move = True
                else:
                    search_range = jitter  # only evaluate staying in place

            # if vehicle just started moving laterally, set direction
            if veh.attempted_lateral_move != prev_move_attempted:
                
                for dy in range(-search_range, search_range + 1):  # search window (may be 0)
                    cand = old_center + dy
                    if cand < 0 or cand >= W_cells:
                        continue

                    if is_position_clear(grid_snapshot, veh, cand, lon_min, lon_max, Cmin_cells, W_cells):
                        score = dy
                        
                        if abs(score) > abs(best_score) or (abs(score) == abs(best_score) and random.random() < 0.5): # or (score == best_score and abs(dy) < abs(best - old_center)):
                            best_score = score
                            best = cand

                veh.attempted_move_direction = -1 if best < old_center else 1 if best > old_center else 0
                c_steps = abs(best - old_center)
                step_size = lateral_speed if best > old_center else -1*lateral_speed if best < old_center else 0
                veh.attempted_move_step_counter =  int(c_steps/step_size) if step_size !=0 else 0      
                veh.y_center = old_center + step_size if c_steps >= step_size else best
                old_center = veh.y_center

            # if already attempting lateral move, continue stepping
            elif veh.attempted_lateral_move:
                if veh.attempted_move_step_counter > 0:
                    step_size = lateral_speed if veh.attempted_move_direction == 1 else -3
                    c_steps = abs(best - old_center)
                    attempted_y = old_center + step_size
                    if is_position_clear(grid, veh, attempted_y, lon_min, lon_max, Cmin_cells, W_cells):
                        veh.y_center = old_center + step_size if c_steps >= step_size else best
                        veh.attempted_move_step_counter -= 1
                        old_center = veh.y_center
                else:
                    # completed lateral move
                    veh.attempted_lateral_move = False
                    veh.y_center = best
                    old_center = veh.y_center

            # elif not veh.attempted_lateral_move:
            #     # not attempting lateral move: add small jitter around lane center
            #     veh.y_center = old_center + int(random.gauss(0, jitter/2))
            #     # clamp to bounds
            #     veh.y_center = max(left_bound, min(right_bound, veh.y_center))        
            elif not veh.attempted_lateral_move:
                veh.y_center = old_center + int(random.gauss(0, jitter/2))
                # Don't clamp to left_bound/right_bound—let natural bounds handle it
                veh.y_center = max(veh.w_cells//2, min(W_cells - 1 - veh.w_cells//2, veh.y_center))

        # 4) place vehicles back onto grid (resolve conflicts by precedence of x_front descending)        
        # Two-stage placement to avoid movers overwriting others:
        #  A) Reserve previous positions for all vehicles (this keeps existing cars from vanishing)
        #  B) Let movers attempt to claim their intended position; if they fail, revert them to prev and slow down.
        grid = empty_grid()
        
        # Stage A: try to place everyone at their previous position (if known). This reserves space.
        for veh in sorted(vehicles, key=lambda v: getattr(v, "prev_x", v.x_front), reverse=True):
            prev = getattr(veh, "prev_x", None)
            prev_y = getattr(veh, "prev_y", None)

            if prev is not None and prev_y is not None:
                # temporarily set to prev to attempt reservation
                saved_x, saved_y = veh.x_front, veh.y_center
                veh.x_front, veh.y_center = prev, prev_y
                if place_vehicle_on_grid(grid, veh):
                    veh._reserved_prev = True
                else:
                    veh._reserved_prev = False
                # restore intended for now
                veh.x_front, veh.y_center = saved_x, saved_y
            else:
                # new spawns / no prev info: mark not reserved
                veh._reserved_prev = False
                

        # Stage B: attempt to move into intended positions. Movers must clear their own prev reservation before trying.
        survivors = []
        for veh in sorted(vehicles, key=lambda v: v.x_front, reverse=True):
            intended_x, intended_y = veh.x_front, veh.y_center
            prev = getattr(veh, "prev_x", None)
            prev_y = getattr(veh, "prev_y", None)

            # If this vehicle was reserved at prev and intends to stay at prev -> keep it
            if getattr(veh, "_reserved_prev", False) and prev == intended_x and prev_y == intended_y:
                survivors.append(veh)
                veh.attempted_lateral_move = False  # completed: lateral move finished (stayed in place)
                continue

            # If not reserved previously, try to place at intended now
            if not getattr(veh, "_reserved_prev", False):
                if place_vehicle_on_grid(grid, veh):
                    survivors.append(veh)
                    # Check if lateral move completed
                    if veh.y_center != prev_y and veh.attempted_move_step_counter == 0:
                        veh.attempted_lateral_move = False  # lateral move finished
                    continue

            # Otherwise this vehicle had a prev reservation or couldn't place directly:
            # Clear its prev reservation (so it can try to move out of it)
            if getattr(veh, "_reserved_prev", False):
                # remove only this vehicle's id cells (safe even if none present)
                clear_vehicle_from_grid(grid, veh)

            # Try to place at intended position
            if place_vehicle_on_grid(grid, veh):
                survivors.append(veh)
                continue

            # Couldn't place at intended: try cancelling lateral change (stay in intended x, previous y)
            # if prev_y is not None:
            #     saved_y = veh.y_center
            #     veh.y_center = prev_y
            #     # slow a bit
            #     #veh.speed = max(0, veh.speed - 1)
            #     if place_vehicle_on_grid(grid, veh):
            #         veh.attempted_lateral_move = False
            #         survivors.append(veh)
            #         continue
            #     # restore intended lateral for subsequent attempts
            #     veh.y_center = saved_y

            # # Revert to previous x/y (stay put) and reduce forward speed
            # if prev is not None and prev_y is not None:
            #     veh.x_front, veh.y_center = prev, prev_y
            #     #veh.speed = 0
            #     #slow a bit
            #     veh.speed = max(0, veh.speed - 1)
            #     if place_vehicle_on_grid(grid, veh):
            #         veh.attempted_lateral_move = False
            #         survivors.append(veh)
            #         continue
            # Revert to previous x/y (stay put) and reduce forward speed
            if prev is not None and prev_y is not None:
                veh.x_front, veh.y_center = prev, prev_y
                veh.speed = max(0, veh.speed - 1)
                veh.attempted_lateral_move = False  # RESET lateral move!
                if place_vehicle_on_grid(grid, veh):
                    survivors.append(veh)
                    continue


            # Revert to previous x/y (stay put) and zero forward speed
            if prev is not None and prev_y is not None:
                veh.x_front, veh.y_center = prev, prev_y
                veh.speed = 0
                if place_vehicle_on_grid(grid, veh):
                    veh.attempted_lateral_move = False
                    survivors.append(veh)
                    continue

            # As a last resort try stepping back longitudinally until placeable
            placed = False
            start_back = (prev - 1) if (prev is not None) else (intended_x - 1)
            for x in range(start_back, -1, -1):
                veh.x_front = x
                if place_vehicle_on_grid(grid, veh):
                    survivors.append(veh)
                    placed = True
                    veh.attempted_lateral_move = False
                    break
            if not placed:
                # couldn't find space -> drop this vehicle for now (rare)
                pass

        # cleanup temporary flags
        for veh in vehicles:
            if hasattr(veh, "_reserved_prev"):
                del veh._reserved_prev
        vehicles = survivors


        # 5) record metrics
        centers_this_step = [v.y_center for v in vehicles]
        lateral_centers.extend(centers_this_step)
        # flow: vehicles that passed a cross-section near end
        passed = [v for v in vehicles if v.x_front >= L_cells-2]
        flows.append(len(passed))
        # optionally remove vehicles at the end
        vehicles = [v for v in vehicles if v.x_front < L_cells-1]
        lateral_centers_evol.append(centers_this_step)
        grid_history.append(grid.copy())

    return lateral_centers, flows, lateral_centers_evol, grid_history

def histogram_probs(centers, n_bins):
    """
    centers: 1D array-like of lateral center positions (cells or continuous)
    n_bins: integer number of bins (e.g., W_cells)
    returns: p (length n_bins) normalized to sum to 1
    """
    hist, edges = np.histogram(centers, bins=n_bins, range=(0, n_bins), density=False)
    total = hist.sum()
    if total == 0:
        return np.ones(n_bins) / n_bins  # empty -> uniform
    p = hist.astype(float) / total
    return p

# def order_entropy(centers, n_bins):
#     p = histogram_probs(centers, n_bins)
#     # avoid log(0)
#     nonzero = p[p > 0]
#     H = -np.sum(nonzero * np.log(nonzero))
#     H_norm = H / np.log(n_bins) if n_bins > 1 else 0.0
#     return float(1.0 - H_norm)  # 1 = ordered, 0 = uniform

def order_entropy(centers, lane_centers):
    """
    Measure how well vehicles cluster around lane centers.
    
    Returns RMS distance from each vehicle to nearest lane center, normalized to 0-1:
    - 0 = all vehicles exactly at lane centers
    - 1 = vehicles maximally dispersed
    """
    if len(centers) == 0:
        return 0
    
    centers = np.array(centers, dtype=float)
    lane_centers = np.array(lane_centers, dtype=float).flatten()
    
    # Distance from each vehicle to its nearest lane center
    distances = np.abs(centers[:, np.newaxis] - lane_centers[np.newaxis, :])
    min_distances = np.min(distances, axis=1)
    
    # RMS (root mean square) distance
    rms_distance = np.sqrt(np.mean(min_distances ** 2))
    
    # Max possible RMS: vehicles at edges of road
    road_width = lane_centers.max() - lane_centers.min()
    max_rms = road_width / 2  # half road width as theoretical max
    
    # Normalize to 0-1
    return float(np.clip(rms_distance / max_rms, 0, 1))