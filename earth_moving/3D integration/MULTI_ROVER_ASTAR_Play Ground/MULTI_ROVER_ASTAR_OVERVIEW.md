# Multi-Rover A* Earth-Moving Playground

This folder is a self-contained runnable bundle for the A* based rover planning
experiments. It contains the notebook, the real PyBullet scripts launched by the
notebook, the URDF files, and one consolidated explanation document.

## What To Open First

Open:

```text
Hybrid Orchestrator/MULTI_ROVER_ASTAR_PLAYGROUND.ipynb
```

The notebook is intentionally not a toy rewrite. It launches the actual project
files:

```text
Path Tracking/astar_path_following_flowfield.py
Path Tracking/multi_astar_priority_scheduling3.py
Hybrid Orchestrator/orchestrator_hybrid_multi_astar_scheduled.py
```

The notebook is split into three runnable demonstrations:

1. Single-rover A* with many static pebbles, relaxation/filtering, timing
   output, and PyBullet tracking.
2. Standalone multi-rover A* priority scheduling and collision avoidance.
3. Integrated multi-rover earth-moving orchestrator with shared 2D task
   allocation, A* approach trajectories, task-preserving push trajectories, and
   collision avoidance.

## Folder Layout

```text
MULTI_ROVER_ASTAR_Play Ground/
  MULTI_ROVER_ASTAR_OVERVIEW.md
  requirements.txt

  Hybrid Orchestrator/
    MULTI_ROVER_ASTAR_PLAYGROUND.ipynb
    orchestrator_hybrid_multi_astar_scheduled.py
    orchestrator_hybrid_multi_shared_safe.py
    orchestrator_hybrid_multi_safe.py
    Federico orchestrator_hybrid_multi.py
    multi_agent_collision_safety.py
    multi_agent_deadlock_safety.py
    shared_path_allocator.py
    main.py, env.py, search.py, cell.py, coordinate_converter.py
    spillage_model.py, visualizer.py, flowfield_base.py
    navigation_manager.py, path_tracker.py, orca_navigator.py
    2_wheel_rover.urdf, pebbles.urdf

  Path Tracking/
    astar_path_following_flowfield.py
    multi_astar_priority_scheduling3.py
    notebook_multi_astar_priority_launcher.py
    flowfield_pybullet.py
    path_following_flowfield.py
    multi_astar_rovers_eta.py
    2_wheel_rover.urdf, pebbles.urdf
```

The two sibling folders `Hybrid Orchestrator` and `Path Tracking` should remain
together. The notebook and the hybrid scripts use this relative layout.

## Python Setup

Recommended from the package root:

```powershell
python -m venv .venv
.\.venv\Scripts\activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m ipykernel install --user --name multi-rover-astar --display-name "Multi Rover AStar"
jupyter notebook "Hybrid Orchestrator\MULTI_ROVER_ASTAR_PLAYGROUND.ipynb"
```

In Jupyter, select the `Multi Rover AStar` kernel. The notebook uses
`sys.executable`, so the PyBullet scripts run with the same Python interpreter
as the notebook kernel.

When a section is run with GUI mode enabled, PyBullet opens a separate simulator
window. Close the PyBullet window or wait for the configured max time to stop a
run.

## Quick Notebook Use

Run Section 0 first. It prints whether the three real scripts were found.

Then run the settings cell for the section you want to test, edit the values in
that settings cell, and run the following execution cell.

Useful edits:

```python
ASTAR_SCENARIO = "random"
ASTAR_RANDOM_PEBBLES = 450
ASTAR_RANDOM_SEED = 41
```

```python
SCHED_SCENARIO = "2_head_on"
SCHED_PEBBLES = "random"
SCHED_NUM_PEBBLES = 150
SCHED_MAX_TIME = 90.0
```

```python
HYBRID_ROVERS = 3
HYBRID_PEBBLES = 50
HYBRID_MAP_INTERVAL = 12.0
HYBRID_DRAW_PATHS = False
```

## Conceptual System Summary

The system solves two connected problems:

1. Earth-moving task planning: choose object paths that push material toward a
   target zone.
2. Physical multi-rover motion planning: move the rovers in PyBullet while
   tracking those paths and avoiding rover-rover collisions.

The final integrated runner is:

```text
Hybrid Orchestrator/orchestrator_hybrid_multi_astar_scheduled.py
```

It combines:

- a shared 2D earth-moving map
- object/path reservations across rovers
- A* approach planning from current rover pose to a gate behind the push path
- shovel-point tracking of the approach and push trajectories
- extra push extension into the target zone
- task-aware priority safety: PUSH > TURN_TO_PUSH > APPROACH > ROLLBACK
- approach-only replanning when approach rovers conflict
- completed-path-driven shared map refresh with a minimum interval

## Phase 1: 2D Earth-Moving Map And Task Path Selection

The 2D planner converts PyBullet pebble positions into a grid. Let the physical
world point be:

```text
p_world = (x, y)
```

and let the grid index be:

```text
c = (i, j)
```

`CoordinateConverter` maps between physical meters and 2D cells using the
environment radius, target-zone radius, and shovel coverage. Each cell stores:

```text
num_objects
visible_cells_target
distance_to_children_target
best_path_target
best_path_highway
heat_map
total_objects_path_target
```

The target zone is a disk in grid space:

```text
T = {c : ||center(c) - center(grid)|| <= target_zone_radius}
```

Object cells inside this disk are tracked separately as target-zone cells. This
is one reason an object can be visible in green but have little or no heat-map
value around it: it may already be classified as inside the target zone, or it
may have no valid target path/value propagation.

For each object cell, the 2D layer computes visibility toward the target zone.
The target visibility relation can be read as:

```text
cell a sees child b if:
  b is in the allowed angular cone from a toward the target
  and b is reachable/consistent with target-directed motion
```

The planner then searches for candidate paths to the target. A candidate path
has a path score based on expected delivered objects and path cost. With the
spillage model enabled, the raw object count is reduced by an estimated loss
along the path:

```text
delivered_objects = raw_objects - spilled_objects
```

The path allocator uses three stages:

1. Target paths with positive delivered objects.
2. Highway fallback paths with positive objects.
3. Raw target fallback if the spillage model predicts zero delivery.

For rover `r`, a candidate path starting at cell `c0` is scored approximately
as:

```text
score = expected_objects - 0.5 * distance(rover_position, world(c0))
```

The shared allocator rejects any candidate whose path cells overlap:

```text
reserved_path_cells
```

or whose object cells overlap:

```text
reserved_object_cells union consumed_object_cells
```

This prevents two rovers from selecting the same physical corridor or the same
stale object work from an old map.

## Phase 2: Single-Rover A* Through Static Pebbles

The single-rover experiment is implemented in:

```text
Path Tracking/astar_path_following_flowfield.py
```

It demonstrates the basic A* planning stack before multi-agent scheduling.

### Obstacle Inflation

Each pebble is treated as a disk obstacle. The rover is converted to a point
robot by inflating each pebble by:

```text
R_block = R_rover + R_pebble + clearance
```

With the current common values:

```text
R_rover = 0.15 m
R_pebble = 0.05 m
clearance = 0.01 m
R_block = 0.21 m
```

A grid cell with center `x_cell` is blocked if:

```text
min_i ||x_cell - p_i|| <= R_block
```

where `p_i` is pebble center `i`.

### A* Grid Search

A* runs on an 8-connected grid. For node `n`:

```text
f(n) = g(n) + h(n)
```

where:

```text
g(n) = accumulated cost from the start
h(n) = Euclidean distance to the goal
```

Cardinal moves cost `1`. Diagonal moves cost `sqrt(2)`. A diagonal move is only
accepted if the two adjacent cardinal cells are free, so the path cannot cut
through the corner between two blocked cells.

If the requested start or goal cell is occupied, the script moves it to the
nearest free grid cell before running A*.

### What Happens If No Valid A* Path Exists

Sometimes the full inflated obstacle mask blocks all routes. In this project,
small isolated pebbles are not always hard obstacles because the rover can
physically push them. The fallback is progressive isolated-cluster filtering.

Build an undirected graph over pebbles:

```text
V = all pebbles
E = {(i, j) : ||p_i - p_j|| <= RELAX_CLUSTER_LINK_RADIUS}
```

The connected components of this graph are pebble clusters:

```text
C_1, C_2, ..., C_m
```

At relaxation threshold `T`, clusters are classified as:

```text
hard obstacle if |C_k| >= T
ignored in the A* mask if |C_k| < T
```

The physical pebbles are still spawned in PyBullet. "Ignored" means ignored by
the planning mask only, so the rover may push those pebbles if needed.

Progressive relaxation does:

```text
1. Try A* with all pebbles hard.
2. If it fails, try T = 2: ignore isolated single pebbles.
3. If it fails, try T = 3: ignore clusters of size <= 2.
4. Continue until a route is found or the relaxation limit is reached.
```

The notebook prints the resulting mode, for example:

```text
planning mode: full
planning mode: relaxed_ignore_clusters_le_2
hard obstacles: 450 / 450
relaxed pebbles: 0
```

This makes it easy to show both the conservative case and the practical
"pushable small clutter" fallback.

### Trajectory Generation

The raw A* path is a grid polyline:

```text
p_0, p_1, ..., p_N
```

It may be shortcut, smoothed, and resampled. The important smoothing operation
is collision-checked Chaikin smoothing. For a segment from `a` to `b`:

```text
q = (1 - alpha) a + alpha b
r = alpha a + (1 - alpha) b
```

The current smoothing uses repeated corner cutting. Because smoothing can move
the path inward near obstacles, the smoothed polyline is checked against the
inflated obstacle mask. If it collides, the code falls back to the unsmoothed
base path and only resamples it.

The notebook prints:

```text
A* path planning complete:
  planning mode:    ...
  hard obstacles:   ...
  relaxed pebbles:  ...
  raw nodes:        ...
  shortcut nodes:   ...
  trajectory base:  ...
  trajectory nodes: ...
  raw length:       ... m
  trajectory length:... m

==== PLANNING TIMING ====
A* calculation:        ... s
Trajectory generation: ... s
Path flow-field build: ... s
Guidance mode:         ...
```

## Phase 3: Shovel-Point Path Tracking Control

The tracking controller does not control only the rover center. The task point
is the shovel point in front of the rover:

```text
p_s = p_base + L_shovel [cos(psi), sin(psi)]
```

where:

```text
p_base = (x, y)
psi = rover yaw
L_shovel = 0.17 m
```

The path is parameterized by arc length `s`:

```text
s_0 = 0
s_i = sum_{j=1..i} ||p_j - p_{j-1}||
L = s_N
```

To avoid jumping to a later nearby segment of a folded path, the projection is
limited to a progress window:

```text
s_min = max(0, s_ref - PROGRESS_BACKTRACK_M)
s_max = min(L, s_ref + PROGRESS_LOOKAHEAD_M)
```

For segment endpoints `a`, `b`, with `d = b - a`, the projection of the shovel
point is:

```text
u = ((p_s - a) dot d) / (d dot d)
u_clamped = clamp(u, u_min, u_max)
p_path = a + u_clamped d
```

The local path tangent and normal are:

```text
t_hat = d / ||d||
n_hat = [-t_y, t_x]
```

The signed cross-track error is:

```text
e_n = (p_s - p_path) dot n_hat
```

The desired local direction is:

```text
v_des = k_t t_hat - k_n e_n n_hat
u_des = v_des / ||v_des||
```

This is a Stanley-like vector form. The desired heading is:

```text
theta_des = atan2(u_des_y, u_des_x)
```

and the heading error is:

```text
e_theta = wrap(theta_des - psi)
```

The angular command is:

```text
w_cmd = clip(k_theta e_theta, -w_max, w_max)
```

The forward command is reduced when the rover is not aligned:

```text
align = max(0, cos(e_theta))
v_cmd = v_max * align * distance_factor
```

Large heading error means low forward speed; when the rover is aligned, forward
speed approaches `v_max`.

The differential-drive wheel commands are:

```text
v_L = v_cmd - w_cmd * track_width / 2
v_R = v_cmd + w_cmd * track_width / 2

omega_L = -v_L / wheel_radius
omega_R = -v_R / wheel_radius
```

They are clipped by the configured wheel-speed and torque limits before being
sent to PyBullet.

## Phase 4: Offline And Online Time Estimation

The multi-rover scheduling layer needs more than geometry. It needs to predict
when rovers will occupy future cells.

### Geometric ETA

For a smoothed trajectory, cumulative arc length is:

```text
s_i = sum ||p_i - p_{i-1}||
```

Curvature is approximated from heading change:

```text
kappa_i ~= delta_theta_i / delta_s_i
```

The nominal speed is reduced on high-curvature segments:

```text
v_i = v_ref / (1 + k_curv * |kappa_i|^p)
v_i = max(v_i, v_min)
```

Time is integrated along the path:

```text
T_0 = 0
T_{i+1} = T_i + ds_i / (0.5 * (v_i + v_{i+1}))
```

The code keeps lower and upper bounds:

```text
T_lower(s) = alpha_lower * T_base(s)
T_upper(s) = alpha_upper * T_base(s)
```

### Cell-Time Windows

For scheduler cell conflicts, each raw A* cell receives a time window.
Effective cell speed is:

```text
requested_speed = CELL_TIME_SPEED_FRACTION_OF_VMAX * V_MAX
wheel_speed_cap = CELL_TIME_WHEEL_SPEED_FRACTION
                  * configured_max_wheel_speed()
                  * WHEEL_RADIUS
effective_speed = min(requested_speed, wheel_speed_cap, speed_cap)
```

The nominal time for one cell step is:

```text
dt = distance / effective_speed + turn_penalty * |delta_heading|
```

Uncertainty grows along the path:

```text
sigma(i) = sigma_base + sigma_growth * i
```

So the time window for cell `i` is:

```text
t_low_i = max(sim_time, t_nominal_i - sigma(i))
t_high_i = t_nominal_i + sigma(i) + extra_upper_motion_time
```

Two agents have a time conflict when:

```text
a_low <= b_high + CONFLICT_TIME_BUFFER
and
b_low <= a_high + CONFLICT_TIME_BUFFER
```

This gives the scheduler a conservative "might overlap" test rather than a
single overconfident arrival time.

## Phase 5: Multi-Rover A* Priority Scheduling

The standalone scheduler is:

```text
Path Tracking/multi_astar_priority_scheduling3.py
```

Each rover stores two path representations:

```text
raw A* cell path:  used for conflict detection and ETA windows
smooth trajectory: used for continuous tracking control
```

The scheduler periodically refreshes raw cells because raw A* is cheaper than
regenerating a smooth trajectory. Smooth replans are done only when needed.

### Path Conflict Detection

For each rover, future raw cells are inflated by a clearance radius. A pairwise
candidate conflict exists if inflated future cell sets overlap or pass within
the configured conflict distance. The conflict becomes active if the ETA windows
also overlap.

For each active conflict:

```text
winner = rover with smaller priority value
yielder = rover with larger priority value
```

The yielder can:

```text
continue normally
slow down to arrive after the winner
stop at a hold point
replan around the conflict
hard-clear if it is already blocking the winner path
reverse briefly if it is too close to turn safely
```

### Time-Reserved A*

Higher-priority rovers reserve future cells:

```text
reservation[cell] = [t_low, t_high, rover_id]
```

Lower-priority A* then expands nodes in a time-aware way. If a candidate cell
arrival overlaps a reservation, the cell is not automatically impossible.
Instead, the candidate is delayed:

```text
arrival_time = reserved_high + CELL_RESERVATION_TIME_MARGIN
```

This makes A* optimize arrival time rather than only geometric length. A short
wait can be better than a long detour.

### Slow-Yield Control

If the yielder has enough room before the conflict, it slows instead of fully
stopping. Let:

```text
d_hold = hold_s - current_s
t_release = wait_until - sim_time
```

The yielder speed limit is:

```text
v_yield = d_hold / max(t_release, epsilon)
```

Then:

```text
v_cmd = min(v_path_following, v_yield)
```

This makes the yielder arrive just after the winner clears when possible.

### Hard-Clear And Emergency Stop

If the yielder is already on the winner corridor, waiting is wrong because the
winner cannot pass. In that case, the winner corridor is converted into proxy
obstacles for the yielder's A*. The yielder plans a path that clears the winner
path.

The emergency stop layer is a fallback, not the main planner. It checks close
physical distance, path blocking, closing speed, and priority. When a high
priority rover is stopped because a lower priority rover blocks its path, that
pair is fed back into the scheduler as a blocking conflict.

## Phase 6: Integrated Multi-Rover Hybrid A* Orchestrator

The final integrated runner is:

```text
Hybrid Orchestrator/orchestrator_hybrid_multi_astar_scheduled.py
```

It starts from Federico's multi-agent orchestrator idea but changes the motion
planning layers:

```text
shared 2D earth-moving map
-> per-rover task/path allocation
-> A* approach to a gate behind the selected push path
-> turn-to-push alignment
-> shovel tracking of the task-critical push path
-> rollback/sync
-> completed-path map refresh
```

### Approach Path Versus Push Path

The approach path is free navigation:

```text
current rover pose -> gate behind selected 2D path start
```

The push path is task-critical:

```text
selected 2D object path -> extended path into target zone
```

The push path is followed by the shovel point, because the physical task is to
move objects with the shovel, not merely place the rover center on the curve.

The end of the push path can be extended by:

```python
DEFAULT_PUSH_EXTRA_DISTANCE = 0.25
```

This pushes material farther into the target zone.

### Task-Aware Priority

In the hybrid runner, collision priority depends on task phase:

```text
PUSH > TURN_TO_PUSH > APPROACH > ROLLBACK
```

A rover following a push path should not yield to a rover merely approaching a
gate. If both are close and one is on a task path, the approach rover should
yield, back away, or replan.

### Approach Replanning

Approach paths are allowed to replan periodically and when priority safety asks
for it:

```python
DEFAULT_APPROACH_REPLAN_INTERVAL = 2.5
```

This is different from push-path replanning. Push paths are task preserving and
should not be replaced by a direct goal-to-goal route that no longer moves
objects.

### Shared Map Refresh

The 2D map is not rebuilt continuously. Rebuilding after every rover finishes a
push path can be too expensive when several finish close together. The current
policy is:

```text
rebuild after a push path finishes
but coalesce rebuild requests with a minimum interval
```

The edit-friendly setting is:

```python
DEFAULT_MAP_UPDATE_INTERVAL = 12.0
```

Completed object cells remain blocked until the next map refresh, so stale work
from the old map is not immediately selected again.

### Useful Direct-Edit Settings

At the top of the integrated script:

```python
SHOW_3D_PATHS = False
SHOW_3D_CONFLICT_MARKERS = False
FLOW_FIELD_VIS_MODE = "never"
DEFAULT_PUSH_EXTRA_DISTANCE = 0.25
DEFAULT_MAP_UPDATE_INTERVAL = 12.0
DEFAULT_APPROACH_REPLAN_INTERVAL = 2.5
```

These are intended for editing in the file before clicking Play.

## Why Some Objects May Have No Heat

The 2D visualizer draws green object dots separately from heat. A green object
can have little or no heat around it if:

- the object is inside the target zone
- the cell has no target visibility
- A* could not produce a target or highway path for that cell
- the spillage model predicts zero delivered objects
- the path value was not propagated through `total_objects_path_target`
- the cell is blocked by current reservations or consumed-cell bookkeeping

So the heat map is not a raw object-density map. It is closer to a value map for
useful, currently selectable object-pushing work.

## Important Implementation Files

Single-rover A*:

```text
Path Tracking/astar_path_following_flowfield.py
```

Standalone scheduler:

```text
Path Tracking/multi_astar_priority_scheduling3.py
Path Tracking/notebook_multi_astar_priority_launcher.py
```

Integrated hybrid runner:

```text
Hybrid Orchestrator/orchestrator_hybrid_multi_astar_scheduled.py
```

Shared map allocation:

```text
Hybrid Orchestrator/shared_path_allocator.py
```

Safety layers:

```text
Hybrid Orchestrator/multi_agent_collision_safety.py
Hybrid Orchestrator/multi_agent_deadlock_safety.py
```

2D task map:

```text
Hybrid Orchestrator/main.py
Hybrid Orchestrator/env.py
Hybrid Orchestrator/search.py
Hybrid Orchestrator/spillage_model.py
```

## Suggested Demonstration Order

1. In Section 1 of the notebook, run `ASTAR_SCENARIO = "random"` with many
   pebbles. Show full A* and the printed planning timing.
2. Change Section 1 to `ASTAR_SCENARIO = "edge_gate_relaxation"` or
   `two_gates_relaxation`. Show that full A* can fail and relaxation/filtering
   opens a route.
3. Run Section 2 with `SCHED_SCENARIO = "2_head_on"` or `"4_crossing"`. Show
   priority scheduling, ETA windows, slow-yield, hard-clear, and emergency
   fallback behavior.
4. Run Section 3 with 2 or 3 rovers. Show the integrated workflow: shared map,
   A* approach, turn-to-push, task-critical push path, and map refresh after
   push completion.

## Known Limitations

- The A* state is position-only, not full `(x, y, yaw)`. The code uses
  start-direction filters and turn-in-place behavior instead of orientation
  state in the search.
- Time-reserved A* models waiting at the cell level, not a full continuous
  velocity trajectory optimization.
- Push paths are intentionally protected. The original scheduler replanner is
  goal-directed and can destroy a useful object-pushing path, so it is disabled
  by default in the integrated hybrid runner.
- The 2D heat map is recomputed from snapshots. During a push, the old map can
  be stale until the next coalesced refresh.
- PyBullet contact dynamics, wheel slip, and pebble motion are only approximated
  by the planning models.

## Why There Are Still Several Python Files

The folder is intentionally minimal, but it cannot be only one `.ipynb` and three
scripts. The notebook launches real project files, and those scripts import
shared helpers:

- `flowfield_pybullet.py`, `path_following_flowfield.py`, and
  `multi_astar_rovers_eta.py` are required by the A* path-tracking and scheduler
  experiments.
- `main.py`, `env.py`, `search.py`, `cell.py`, `coordinate_converter.py`,
  `spillage_model.py`, and `visualizer.py` are required by the 2D earth-moving
  map used by the integrated orchestrator.
- `orchestrator_hybrid_multi_shared_safe.py`,
  `orchestrator_hybrid_multi_safe.py`, `Federico orchestrator_hybrid_multi.py`,
  `shared_path_allocator.py`, `multi_agent_collision_safety.py`, and
  `multi_agent_deadlock_safety.py` are required by the final hybrid
  multi-agent runner.

Files not needed to run the notebook, such as older orchestrator variants,
standalone historical notes, calibration outputs, and old experiments, were
left out of this runnable bundle.
