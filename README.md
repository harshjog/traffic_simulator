# Vehicle Arrangement Simulator (v3)

A Nagel-Schreckenberg traffic flow simulation with lateral lane-changing behavior. This project models vehicle dynamics on a multi-lane road, including longitudinal acceleration/deceleration and intelligent lateral positioning.

## Overview

The simulator models vehicles on a discrete grid where:
- **Longitudinal (x)**: Forward motion, gap-based acceleration following Nagel-Schreckenberg rules
- **Lateral (y)**: Lane-changing behavior with two modes: disciplined lane-seeking or random jitter

Key features:
- Configurable vehicle widths (two classes via Gaussian distributions)
- Clearance constraints in both longitudinal and lateral directions
- Two-stage placement resolution to prevent vehicle collisions
- Quantitative ordering metric based on lane clustering

## Files

### `vehicle_arrangement_v3_config.py`
Configuration file with all tunable parameters:
- **Road geometry**: `cell_size`, `road_length_m`, `lane_width`, `number_lanes`
- **Vehicle properties**: Width distributions (`mu_w1`, `sigma_w1`, etc.), class probabilities
- **Dynamics**: Max speed (`v_max`), spawn probability (`spawn_prob`), clearance (`C_min`)
- **Lateral behavior**: 
  - `p_lateral_change`: Probability a vehicle attempts an active lane change each timestep
  - `lateral_search_cells`: Search radius for finding alternative lanes
  - `jitter`: Random lateral perturbation when not changing lanes

### `vehicle_arrangement_v3_backend.py`
Core simulation engine:
- **`Vehicle` class**: Tracks position, speed, width, lateral movement state
- **Grid management**: Occupancy grid for collision detection
- **Simulation loop** (`simulate()` function):
  1. Spawn vehicles at road entrance
  2. Longitudinal update: speed based on gap to vehicle ahead
  3. Lateral decisions: each vehicle decides whether to seek a new lane or stay
  4. Two-stage placement: reserve previous positions, then attempt intended moves
  5. Record metrics: vehicle positions, flow, grid history
- **Metrics**:
  - `order_entropy()`: RMS distance from vehicles to nearest lane center (0=perfect order, 1=dispersed)
  - `histogram_probs()`: Distribution histogram of lateral positions

### `vehicle_arrangement_v3_plotter.py`
Visualization and analysis:
- **Histogram**: Lateral position distribution with lane center markers
- **Flow plot**: Vehicle throughput over time (smoothed)
- **Animation**: Real-time grid visualization with lane boundary markings (white dashed lines)

## Usage

### Basic Run
```python
from vehicle_arrangement_v3_config import *
import vehicle_arrangement_v3_backend as va

# Run simulation
centers, flows, centers_evol, grid_history = va.simulate(seed=42)

# Calculate ordering metric
order = va.order_entropy(centers, lane_centers)
print(f"Order: {order:.4f}")
```

### Parameter Sweep (Discipline Effect)
Edit `vehicle_arrangement_v3_config.py` to vary `p_lateral_change` and re-run to study how vehicle "discipline" affects lane formation.

### Visualization
Run the plotter script to generate:
1. Histogram of vehicle lateral positions
2. Flow rate over time
3. Animated grid showing vehicle movement and lane boundaries

## Key Concepts

### Lateral Movement Logic
- **Attempting lane change**: Vehicle searches within `lateral_search_cells` for an available position using `is_position_clear()`
- **Stepping**: Movement happens incrementally (`lateral_speed` parameter)  to avoid sudden repositioning
- **Non-attempting**: Small Gaussian jitter around current position
- **Reset**: Failed placement attempts reset lateral movement to allow re-evaluation

### Collision Resolution
Two-stage placement prevents vehicles from "pushing through" obstacles:
1. **Stage A**: Reserve previous positions for all vehicles
2. **Stage B**: Mover vehicles attempt intended positions; if blocked, revert and slow down

### Ordering Metric
$$\text{Order} = \frac{\text{RMS}(d_i)}{\text{max RMS}} = \frac{\sqrt{\frac{1}{N}\sum (y_i - \text{nearest lane center})^2}}{\text{road width}/2}$$

- **0**: All vehicles centered in lanes (perfect order)
- **1**: Vehicles maximally dispersed across road

## Parameters to Explore

| Parameter | Effect |
|-----------|--------|
| `p_lateral_change` | Higher = more lane changes = less stable order |
| `spawn_prob` | Higher = more traffic density |
| `lateral_search_cells` | Larger = vehicles can reach further lanes |
| `lateral_speed` | Larger = vehicles switch lanes faster |
| `jitter` | Larger = more random wandering within lanes |
| `v_max` | Higher = faster traffic |
| `C_min` | Larger = more space required around vehicles |

## Dependencies
- NumPy
- Matplotlib
- Python 3.7+

## Future Extensions
- Multi-model traffic (heterogeneous vehicle types)
- Adaptive lane-changing strategies
- Speed-dependent clearance
- Real-world calibration against empirical traffic data
