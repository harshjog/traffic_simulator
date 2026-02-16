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

### This small snippet is to sweep over different parameters and look at the lane order.

# sweep_p_lateral = np.logspace(-3, 0, 10)
# sweep_p_vehicle_class = [0.0, 0.25, 0.5, 0.75, 1.0]

# fig, ax = plt.subplots(figsize=(10, 6))

# # Use a colormap for different p_vehicle_class values
# colors_list = plt.cm.viridis(np.linspace(0, 1, len(sweep_p_vehicle_class)))

# for p_class, color in zip(sweep_p_vehicle_class, colors_list):
#     orders = []
#     for p_lat in sweep_p_lateral:
#         seed_sim = 42
#         centers, flows, centers_evol, grid_history = va.simulate(
#             seed_sim, 
#             p_lateral_change_override=p_lat,
#             p_vehicle_class_override=p_class
#         )
#         order = va.order_entropy(centers, lane_centers)
#         orders.append(order)
    
#     ax.plot(sweep_p_lateral, orders, marker='o', color=color, label=f'p_vehicle_class={p_class}', linewidth=2)

# ax.set_xscale("log")
# ax.set_xlabel("Discipline (probability of lateral change)", fontsize=12)
# ax.set_ylabel("Order (RMS distance from lane centers)", fontsize=12)
# ax.set_title("Effect of vehicle discipline and class on lateral order", fontsize=13)
# ax.legend(loc='best')
# ax.grid(True, alpha=0.3)
# plt.tight_layout()
# plt.show()    


# ### Following code is good for looking at a single run in more detail. This include an animation of the traffic.

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


