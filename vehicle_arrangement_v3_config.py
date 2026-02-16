import numpy as np
from math import ceil, floor

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

p_vehicle_class = 1   # prob of vehicle being class 2 (wider)

C_min = 0.1     # minimum clearance (m)
Cmin_cells = int(ceil(C_min / cell_size))

v_max = 10        # cells per tick
p_slow = 0.0     # prob of random slow-down
spawn_prob = 0.3 # prob of spawning a vehicle each time step at start

p_lateral_change = 0.001    # probability a vehicle will attempt a lateral shift this tick
lateral_search_cells = int(lane_width/cell_size)  # how far (cells) to search left/right when attempting shift. 
# As a matter of "discipline", should this be limited to 1 lane or some multiple of vehicle width?

lateral_speed = 3 # how many cells to shift laterally when changing lanes

jitter = 2  # max lateral jitter (cells) when not attempting shift

T = int(500/(spawn_prob+0.00001))         # time steps, scaled by spawn_prob to keep similar vehicle counts across different spawn rates
