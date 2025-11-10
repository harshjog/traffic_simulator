import numpy as np
import matplotlib.pyplot as plt
import random
from math import ceil, floor
import matplotlib.animation as animation
from matplotlib import colors
from win32comext.adsi.demos.scp import verbose

# -----------------------------
# Parameters (tweak these)
# -----------------------------
cell_size = 0.1                 # meters per cell
road_length_m = 200.0           # simulation length in meters
lane_width = 3.5                # meters
number_lanes = 4
road_width_m = number_lanes*lane_width              # meters (example)
W_cells = int(round(road_width_m / cell_size))
L_cells = int(round(road_length_m / cell_size))
lane_centers = np.linspace(W_cells / (2*number_lanes),
                                   W_cells - W_cells / (2 * number_lanes),
                                   number_lanes).astype(int)

# Vehicle properties
mu_w1, sigma_w1 = 0.7, 0.05       # mean vehicle 1 width (m), std dev (m)
w1_min, w1_max = 0.5, 0.9

mu_w2,sigma_w2 = 1.7, 0.1      # mean vehicle 2 width (m), std dev (m)
w2_min, w2_max = 1.5, 1.9

p_vehicle_class = 0.0    # prob of vehicle being class 2 (wider)

C_min = 0.2     # minimum clearance (m)
Cmin_cells = int(ceil(C_min / cell_size))

v_max = 5        # cells per tick
p_slow = 0.1     # prob of random slow-down
spawn_prob = 0.3 # prob of spawning a vehicle each time step at start

p_lateral_change = 0.2    # probability a vehicle will attempt a lateral shift this tick
lateral_search_cells = int(lane_width/cell_size)  # how far (cells) to search left/right when attempting shift


T = 1000         # time steps

# -----------------------------
# Utility functions
# -----------------------------
def draw_width_m():
    if random.random() < p_vehicle_class:
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
        self.y_center = random.choice(lane_centers)  + int(random.gauss(0, 2))
        self.attempted_lateral_move = False



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
def simulate(seed=0):
    random.seed(seed); np.random.seed(seed)
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
        # # 3) lateral decisions (simple local search ±2 cells)

            # require Cmin_cells clearance around the vehicle in both lateral and longitudinal directions
            left_bound = max(Cmin_cells + veh.w_cells//2, 0 + veh.w_cells//2)
            right_bound = min(W_cells - 1 - (veh.w_cells//2), W_cells-1 - Cmin_cells - veh.w_cells//2)
            best = old_center
            best_score = -1
            # vehicle longitudinal span (rear .. front)
            rear = max(0, veh.x_front - veh.length_cells + 1)
            # check a longitudinal margin of Cmin_cells in front and behind
            lon_min = max(0, rear - Cmin_cells)
            lon_max = min(L_cells - 1, veh.x_front + Cmin_cells)

            # decide whether this vehicle will attempt a lateral shift this tick
            if random.random() < p_lateral_change:
                search_range = lateral_search_cells
                veh.attempted_lateral_move = True
            else:
                search_range = 4  # only evaluate staying in place

            for dy in range(-search_range, search_range + 1):  # search window (may be 0)
                cand = old_center + dy
                if cand < 0 or cand >= W_cells:
                    continue
                half = veh.w_cells // 2
                if veh.w_cells % 2 == 1:
                    left = cand - half
                    right = cand + half
                else:
                    left = cand - half
                    right = cand + half - 1
                # enforce lateral margins to road edges
                if left < Cmin_cells or right > W_cells - 1 - Cmin_cells:
                    continue
                # define lateral margin to check (include lateral clearance)
                col_min = max(0, left - Cmin_cells)
                col_max = min(W_cells - 1, right + Cmin_cells)
                # check entire rectangle (lon_min..lon_max, col_min..col_max) for occupancy
                subrect = grid_snapshot[lon_min:lon_max+1, col_min:col_max+1]
                if subrect.size == 0:
                    # occupied within clearance rectangle -> reject
                    continue
                # treat this vehicle's current cells as empty when evaluating candidates,
                # but consider all other vehicles as occupied
                tmp = subrect.copy()
                tmp[tmp == veh.id] = -1
                if np.any(tmp != -1):
                    continue
                # compute a score that prefers shifting into larger free lateral space and smaller moves
                # also slightly prefer moves that reduce deviation from nearest lane center (optional)
                cleft = left - col_min
                cright = col_max - right
                score = abs((cleft + cright) / 2 - abs(dy))
                # break ties by preferring the smaller absolute dy (less abrupt)
                if score > best_score or (score == best_score and abs(dy) < abs(best - old_center)):
                    best_score = score
                    best = cand
            c_steps = abs(best - old_center)
            step_size = 3 if (best > old_center and getattr(veh, "attempted_lateral_move")) else -3 if best < old_center else 0        
            veh.y_center = old_center + step_size if c_steps >= step_size else best
        

        # 4) place vehicles back onto grid (resolve conflicts by precedence of x_front descending)
        vehicles.sort(key=lambda V: V.x_front, reverse=True)
 
        grid = empty_grid()
        survivors = []
        for veh in vehicles:
            ok = place_vehicle_on_grid(grid, veh)
            if ok:
                survivors.append(veh)
                continue
            # collision at intended position -> try to stay at previous position (no forward progress)
            # if prev_x not set for some reason, treat as can't place and drop
            veh.attempted_lateral_move = False  # reset
            prev = getattr(veh, "prev_x", None)
            prev_y = getattr(veh, "prev_y", None)
            prev_speed = getattr(veh, "old_speed", 0)
            if prev is None:
                # no previous position recorded -> drop vehicle (rare)
                continue
            else:
                veh.y_center = prev_y
                # optionally slow down a bit when forced to cancel lateral move
                veh.speed = max(0, veh.speed - 1)
                if place_vehicle_on_grid(grid, veh):
                    veh.attempted_lateral_move = False
                    survivors.append(veh)
                    continue
                # restore intended lateral for subsequent logic
                veh.y_center = getattr(veh, "y_center", prev_y)
            # revert to previous x (stay put) and zero forward speed    
            veh.x_front = prev
            for s in range(prev_speed, -1, -1):
                veh.speed = s
                if place_vehicle_on_grid(grid, veh):
                    survivors.append(veh)
                    placed = True
                    break
            # previous spot also blocked (others may have moved into it) -> try stepping back until placeable
            # placed = False
            # for x in range(prev-1, -1, -1):
            #     veh.x_front = x
            #     veh.speed = max(0, x - prev)  # will be 0 or negative; keep non-negative
            #     if place_vehicle_on_grid(grid, veh):
            #         survivors.append(veh)
                    
            # if not placed:
            #     # couldn't find space -> drop vehicle (or keep it dropped for now)
            #     pass
        vehicles = survivors
        # 5) record metrics
        centers_this_step = [v.y_center for v in vehicles]
        lateral_centers.extend(centers_this_step)
        # flow: vehicles that passed a cross-section near end
        passed = [v for v in vehicles if v.x_front >= L_cells-2]
        flows.append(len(passed))
        # optionally remove vehicles at the end
        vehicles = [v for v in vehicles if v.x_front < L_cells-1]
        # clear and re-place for simplicity (already cleared earlier)
        # grid = empty_grid()
        # for v in vehicles:
        #     place_vehicle_on_grid(grid, v)

        lateral_centers_evol.append(centers_this_step)
        grid_history.append(grid.copy())

    return lateral_centers, flows, lateral_centers_evol, grid_history

# -----------------------------
# Run and plot
# -----------------------------
seed_sim=random.randint(1,100)
centers, flows, centers_evol, grid_history = simulate(seed_sim)
plt.hist(centers, bins=W_cells, density=True)
plt.title("Lateral center distribution (cells), seed = " + str(seed_sim))
plt.xlabel("Lateral cell index")
plt.show()
plt.plot(np.convolve(flows, np.ones(10)/10, mode='valid'))
plt.title("Smoothed flow")
plt.show()

# determine vmax across all recorded frames so color mapping is stable
##vmax = int(max(0, max(np.max(g) for g in grid_history)))
norm = colors.Normalize(vmin=0, vmax=1) #v_max

print([id(g) for g in grid_history[:5]])

fig, ax = plt.subplots(figsize=(8,4))

# Initialize display with the first frame
im = ax.imshow(grid_history[0].T, cmap='viridis', origin='lower',
               interpolation='nearest',norm=norm ) # vmin = -1aspect='auto'

ax.set_title("Traffic Simulation")
ax.set_xlabel("Longitudinal position (x)")
ax.set_ylabel("Lateral position (y)")

# Animation update function
def update(frame):
    im.set_data(grid_history[frame].T)
    ax.set_title(f"Traffic Simulation (t = {frame})")
    return [im]

# Create animation
anim = animation.FuncAnimation(fig, update, frames=len(grid_history), interval=100, blit=False)

plt.show()
