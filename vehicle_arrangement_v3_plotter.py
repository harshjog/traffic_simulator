import numpy as np
import matplotlib.pyplot as plt
import random
from math import ceil, floor
import matplotlib.animation as animation
from matplotlib import colors
from win32comext.adsi.demos.scp import verbose
import vehicle_arrangement_v3_backend as va
from vehicle_arrangement_v3_config import *

# -----------------------------
# Run and plot
# -----------------------------

# sweep_p_lateral = np.logspace(-3, 0, 10)
# orders = []
# for p_lat in sweep_p_lateral:
#     p_lateral_change = p_lat
#     seed_sim = 42
#     centers, flows, centers_evol, grid_history = simulate(seed_sim)
#     order = order_entropy(centers, W_cells)
#     orders.append(order)

# plt.figure(figsize=(7,4))
# plt.plot(sweep_p_lateral, orders, marker='o')
# plt.xscale("log")
# plt.xlabel("Discipline (probability of lateral change)")
# plt.ylabel("Order (entropy-based)")
# plt.title("Effect of vehicle discipline on lateral order")
# plt.grid(True)
# plt.show()    

seed_sim=random.randint(1,100)
centers, flows, centers_evol, grid_history = va.simulate(seed_sim)
order = va.order_entropy(centers, lane_centers)

# Plot lateral center distribution histogram
fig, ax = plt.subplots(figsize=(8, 4))
ax.hist(centers, bins=W_cells, density=True, color='steelblue', edgecolor='black')
ax.vlines(lane_centers, ymin=0, ymax=ax.get_ylim()[1], colors='red', linestyles='dashed', label='Lane Centers')
ax.set_title(f"Lateral center distribution (cells), seed = {seed_sim}")
ax.set_xlabel("Lateral cell index")
ax.set_ylabel("Probability density")
text = ax.text(
    0.02, 0.95,                       # (x, y) in normalized coordinates
    f"Order = {order}",                   # text
    color="white",
    fontsize=10,
    ha="left", va="top",
    transform=ax.transAxes,           # keeps it fixed in the top-left corner
    bbox=dict(boxstyle="round,pad=0.3", fc="black", ec="white", alpha=0.7)
)
plt.show()
plt.plot(np.convolve(flows, np.ones(10)/10, mode='valid'))
plt.title("Smoothed flow")
plt.show()

# Animation of grid evolution
norm = colors.Normalize(vmin=0, vmax=1) #v_max

#print([id(g) for g in grid_history[:5]])

fig, ax = plt.subplots(figsize=(8,4))

# Initialize display with the first frame
im = ax.imshow(grid_history[0].T, cmap='viridis', origin='lower',
               interpolation='nearest',norm=norm ) # vmin = -1aspect='auto'

ax.set_title("Traffic Simulation")
ax.set_xlabel("Longitudinal position (x)")
ax.set_ylabel("Lateral position (y)")

# Add lane boundary lines (vertical lines separating lanes)
lane_boundaries_m = np.arange(0, road_width_m + lane_width, lane_width)
lane_boundaries_cells = lane_boundaries_m / cell_size
for boundary in lane_boundaries_cells:
    ax.axhline(y=boundary, color='white', linestyle='--', linewidth=0.8, alpha=0.6)

# Animation update function
def update(frame):
    im.set_data(grid_history[frame].T)
    ax.set_title(f"Traffic Simulation (t = {frame})")
    return [im]

# Create animation
anim = animation.FuncAnimation(fig, update, frames=len(grid_history), interval=100, blit=False)

plt.show()


