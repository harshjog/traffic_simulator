import numpy as np
import matplotlib.pyplot as plt
import random
from math import ceil, floor
import matplotlib.animation as animation

from win32comext.adsi.demos.scp import verbose

# -----------------------------
# Parameters (tweak these)
# -----------------------------
cell_size = 0.1                 # meters per cell
road_length_m = 200.0           # simulation length in meters
lane_width = 2                # meters
number_lanes = 4
road_width_m = number_lanes*lane_width              # meters (example)
W_cells = int(round(road_width_m / cell_size))
L_cells = int(round(road_length_m / cell_size))
lane_centers = np.linspace(W_cells / (2*number_lanes),
                                   W_cells - W_cells / (2 * number_lanes),
                                   number_lanes).astype(int)

# Vehicle properties
mu_w1 = 0.5       # mean vehicle 1 width (m)
sigma_w1 = 0.1  # std dev (m)
w_min1 = 0.3
w_max1 = 0.7

mu_w2 = 1.5       # mean vehicle 1 width (m)
sigma_w2 = 0.1  # std dev (m)
w_min2 = 1.3
w_max2 = 1.7


C_min = 0.2     # minimum clearance (m)
Cmin_cells = int(ceil(C_min / cell_size))

v_max = 5        # cells per tick
p_slow = 0.1     # prob of random slow-down
spawn_prob = 0.7 # prob of spawning a vehicle each time step at start

T = 1000         # time steps

# -----------------------------
# Utility functions
# -----------------------------
def draw_width_m():
    # truncated normal
    while True:
        w = (random.gauss(mu_w1, sigma_w1)) # + random.gauss(mu_w2, sigma_w2)
        if w_min1 <= w <= w_max1:
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

        ## start centered laterally
        ##self.y_center = W_cells // 2

        # Random lateral starting position (respecting clearances)
        #margin = Cmin_cells + width_to_cells(self.width_m) // 2
        #self.y_center = random.randint(margin, W_cells - margin)



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

def compute_next_state(grid, veh, L_cells, v_max, p_slow):
    left, right = veh.lateral_bounds()
    front_search = veh.x_front + 1
    gap = 0

    # --- check available empty space ahead ---
    while front_search < L_cells and np.all(grid[front_search, left:right+1] == -1):
        gap += 1
        front_search += 1
        if gap >= v_max:
            break

    # --- update speed based on NaSch logic ---
    new_speed = min(veh.speed + 1, v_max, gap)
    if random.random() < p_slow:
        new_speed = max(0, new_speed - 1)

    # --- compute next longitudinal position ---
    new_x_front = min(L_cells - 1, veh.x_front + new_speed)

    # (Optional: lateral logic if you simulate lane shifts)
    new_y_center = veh.y_center  # or apply lateral drift logic

    return new_x_front, new_y_center, new_speed


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

        for veh in vehicles:
            # clear old position to test moves
            old_x = veh.x_front
            old_left, old_right = veh.lateral_bounds()
            old_center = veh.y_center
            old_speed = veh.speed
            clear_vehicle_from_grid(grid, veh)

            # compute gap to next vehicle ahead in same lane-band area: approximate by checking cells ahead within vehicle's lateral bounds

            #left, right = veh.lateral_bounds()
            #front_search = veh.x_front+1
            front_search = old_x+1
            gap = 0
            while front_search < L_cells and np.all(grid[front_search, old_left:old_right+1] == -1):
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

        # 3) lateral decisions (simple local search ±2 cells)

            left_bound = max(Cmin_cells + veh.w_cells//2, 0 + veh.w_cells//2)
            right_bound = min(W_cells - 1 - (veh.w_cells//2), W_cells-1 - Cmin_cells - veh.w_cells//2)
            # search candidates
            best = old_center
            best_score = -1
            for dy in range(-3,4):  # search window
                cand = old_center + dy
                if cand < 0 or cand >= W_cells: continue
                # compute left/right clearances in cells
                half = veh.w_cells // 2
                if veh.w_cells % 2 == 1:
                    left = cand - half
                    right = cand + half
                else:
                    left = cand - half
                    right = cand + half - 1
                if left < Cmin_cells or right > W_cells - 1 - Cmin_cells:
                    continue
                # compute clearance to nearest vehicle horizontally (scan neighbors)
                # For speed, approximate by scanning in lateral direction at veh.x_front
                row = grid[veh.x_front, :]
                # find contiguous empty cells around cand covering veh.w_cells
                if np.any(row[left:right+1] != -1):
                    continue
                # score = min(clearance left, clearance right)
                # clearance left
                cleft = 0
                j = left-1
                while j >= 0 and row[j] == -1:
                    cleft += 1; j -= 1
                cright = 0
                j = right+1
                while j < W_cells and row[j] == -1:
                    cright += 1; j += 1
                score = (cleft + cright)/2 - abs(dy)  # prefer center and small moves
                if score > best_score:
                    best_score = score; best = cand
            veh.y_center = best

        # 4) place vehicles back onto grid (resolve conflicts by precedence of x_front descending)
        vehicles.sort(key=lambda V: V.x_front, reverse=True)
        # survivors = []
        # for veh in vehicles:
        #     ok = place_vehicle_on_grid(grid, veh)
        #     if ok:
        #         survivors.append(veh)
        #     else:
        #         # collision/block: reduce speed and move back
        #         veh.x_front = max(0, veh.x_front - veh.speed)
        #         if place_vehicle_on_grid(grid, veh):
        #             survivors.append(veh)
        #         else:
        #             # drop vehicle for now (rare)
        #             pass
        # vehicles = survivors

        # 5) record metrics
        centers_this_step = [v.y_center for v in vehicles]
        lateral_centers.extend(centers_this_step)
        # flow: vehicles that passed a cross-section near end
        passed = [v for v in vehicles if v.x_front >= L_cells-2]
        flows.append(len(passed))
        # optionally remove vehicles at the end
        vehicles = [v for v in vehicles if v.x_front < L_cells-1]
        # clear and re-place for simplicity (already cleared earlier)
        grid = empty_grid()
        for v in vehicles:
            place_vehicle_on_grid(grid, v)

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


print([id(g) for g in grid_history[:5]])

fig, ax = plt.subplots(figsize=(8,4))

# Initialize display with the first frame
im = ax.imshow(grid_history[0], cmap='viridis', origin='lower',
               interpolation='nearest', vmin=-1)

ax.set_title("Traffic Simulation")
ax.set_xlabel("Lateral position (y)")
ax.set_ylabel("Longitudinal position (x)")

# Animation update function
def update(frame):
    im.set_data(grid_history[frame])
    ax.set_title(f"Traffic Simulation (t = {frame})")
    return [im]

# Create animation
anim = animation.FuncAnimation(fig, update, frames=len(grid_history), interval=100, blit=False)

plt.show()
