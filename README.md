# Autonomous Robot Brain

A miniature autonomous-vehicle software stack: a differential-drive robot in
Gazebo estimates its own pose from noisy IMU, wheel-encoder and 2D LiDAR data,
maps its surroundings, plans a collision-free route to a goal, and keeps doing
so while its sensors are deliberately degraded and failed.

> **Project status: Phases 0-10 of 12 complete and verified.**
> A six-state EKF fuses wheel odometry, the IMU gyroscope and LiDAR scan
> matching against a map the robot builds itself. Against ground truth over a
> 10 m route it cuts position RMSE by **68 %** and heading RMSE by **61 %**
> versus wheel odometry alone. The robot explores an unknown 20 x 14 m world by
> wall following, builds an occupancy grid, plans through it with A*, and drives
> the plan: **16.8 m from the west corridor through a 1.5 m doorway into Room E
> in 36.5 s, collision-free, arriving 0.30 m from the goal.** Two controllers —
> a DWA local planner and a pure-pursuit/PID follower — are measured against
> each other on an identical course. 332 unit tests and 126 integration tests
> pass. Six fault scenarios measure what the estimator does when its sensors
> degrade: the EKF beats dead reckoning by 1.6-2.8x throughout, and a ramped
> one-wheel encoder bias defeats it outright — the map is built at the estimate
> it is supposed to correct. The benchmark sweep (Phase 11) is next. See
> [Verification status](#verification-status) for exactly what has been run.

---

## Problem statement

Indoors there is no GPS. A robot's only sense of position comes from integrating
its own motion, which drifts without bound, and from recognising places in a map
it built itself, which makes the error self-referential. On top of that, real
sensors do not fail cleanly — they get noisier, they develop slow biases, they
drop measurements, and occasionally they stop entirely.

This project asks a narrow, measurable question:

> **How much sensor degradation can a well-built EKF-based localisation stack
> absorb before the robot can no longer reach its goal — and which failure modes
> hurt most?**

Everything in the repository exists to answer that with numbers rather than a
demo video.

---

## Architecture

```
Sensors ─▶ Preprocessing ─▶ State Estimation ─▶ Localization
                                   │
            Mapping ◀──────────────┤
               │                   │
        Path Planning ─▶ Local Avoidance ─▶ Controller ─▶ Robot Motion
```

Full diagram, package decomposition, frame conventions, topic contract and QoS
policy: **[docs/architecture.md](docs/architecture.md)**.

| Package | Responsibility | Phase |
|---|---|---|
| `arb_core` | Header-only math: angle algebra, `Pose2D` (SE(2)) | 0 |
| `arb_msgs` | `EkfDiagnostics`, `FaultState`, `PlannerMetrics` | 0 |
| `arb_description` | URDF/xacro robot model, RViz config | 1 |
| `arb_simulation` | Gazebo worlds, `ros_gz` bridge | 1 |
| `arb_sensors` | Preprocessing, noise model, fault injection | 2, 10 |
| `arb_localization` | Wheel odometry, EKF, LiDAR scan matching | 3, 4 |
| `arb_mapping` | Occupancy grid, log-odds fusion | 6 |
| `arb_planning` | A*, Dijkstra, Dynamic Window Approach | 7, 8 |
| `arb_control` | PID trajectory tracking | 9 |
| `arb_bringup` | Top-level launch composition | 1+ |
| `arb_evaluation` | Fault-scenario scheduler, metrics, benchmark reports (Python) | 5, 10-11 |

### One rule worth stating up front

Gazebo knows the robot's exact pose. That pose is published on
`/arb/ground_truth/pose` and is **evaluation-only**. No estimation, mapping,
planning or control node may subscribe to it. The rule is enforced mechanically
by [`tests/integration/test_ground_truth_isolation.py`](tests/integration/test_ground_truth_isolation.py),
which greps the estimation packages and also inspects the live node graph.

An estimator that secretly reads ground truth produces a flawless plot and a
meaningless result. The check exists because that failure is silent.

---

## Installation

Validated target: **Ubuntu 24.04 LTS** + **ROS 2 Jazzy Jalisco** +
**Gazebo Harmonic**.

Those three are the officially paired Tier-1 combination for Noble. Jazzy is
supported until May 2029; Gazebo Classic reached end of life in January 2025 and
is deliberately not used.

```bash
# 1. Toolchain
sudo apt update && sudo apt install -y \
  build-essential cmake git python3-pip python3-venv libeigen3-dev \
  curl gnupg lsb-release software-properties-common

# 2. ROS 2 apt repository
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# 3. ROS 2 Jazzy + Gazebo Harmonic + analysis stack
sudo apt install -y \
  ros-jazzy-desktop ros-dev-tools ros-jazzy-ros-gz \
  ros-jazzy-tf-transformations python3-colcon-common-extensions \
  python3-numpy python3-matplotlib python3-scipy python3-pytest python3-pandas

# 4. rosdep
sudo rosdep init 2>/dev/null; rosdep update
```

Verify:

```bash
./scripts/check_environment.sh
```

A Docker environment is provided for CI and cross-machine reproducibility; see
[docker/README.md](docker/README.md). Native install is the recommended
development path because Gazebo and RViz both want direct GPU and X11 access.

---

## Known environment pitfalls

**The workspace path must be ASCII-only.** ROS 2's `rosidl` CMake code splits
interface paths on non-ASCII bytes. A workspace under, say, the Turkish desktop
folder `~/Masaüstü/` fails with a message that names a path which does not
exist:

```
CMake Error ... Target dependency '/home/user/Masa' does not exist
CMake Error ... list index: 1 out of range (-1, 0)
```

Nothing in that error mentions encoding, which makes it expensive to diagnose.
This repository therefore lives at an ASCII path and both
`scripts/check_environment.sh` and `scripts/build.sh` refuse to run if the
resolved workspace path contains non-ASCII characters. The build scripts use
`pwd -P` so the build still works when the repo is reached through a symlink
placed in a non-ASCII directory.

**ROS setup scripts are incompatible with `set -u`.** `/opt/ros/jazzy/setup.bash`
dereferences `AMENT_TRACE_SETUP_FILES` before defining it, so any script with
`set -euo pipefail` aborts on `source`. All scripts here use `set -eo pipefail`.

---

## Build and test

```bash
./scripts/build.sh          # colcon build (Release)
./scripts/run_tests.sh      # all unit tests + summary
pytest tests/integration -v # architecture guards
```

---

## Usage

Every shell that talks to the robot needs the same environment. One line does
it, and it checks the things that silently break a ROS workspace (a non-ASCII
path, a missing distro, an unbuilt workspace):

```bash
source scripts/env.sh
```

It also pins `ROS_DOMAIN_ID=42`. Every pane and every script must agree on that
or they will not see each other's topics.

The whole stack in one command:

```bash
ros2 launch arb_bringup bringup.launch.py
```

```bash
ros2 launch arb_bringup bringup.launch.py controller:=pid spawn:=control_test
```

```bash
ros2 launch arb_bringup bringup.launch.py headless:=true rviz:=false
```

Or start each stage yourself, which is what the rest of this section does:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# Simulation with GUI and RViz
ros2 launch arb_simulation simulation.launch.py

# Headless, for batch experiments
ros2 launch arb_simulation simulation.launch.py headless:=true rviz:=false

# Spawn somewhere else (names come from configs/world.yaml)
ros2 launch arb_simulation simulation.launch.py spawn:=goal_room_c
```

Drive it by hand:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Verify the simulation is actually producing usable data (rates, frame ids,
noise, beam geometry, and an open-loop motion check):

```bash
python3 scripts/verify_simulation.py
```

Overlay a live scan on the known floor plan:

```bash
python3 scripts/plot_scan_vs_world.py -o results/phase1/scan_vs_world.png
```

Bring up the sensor preprocessing layer, and verify it:

```bash
ros2 launch arb_sensors sensors.launch.py
python3 scripts/verify_sensors.py
```

Inject a fault at runtime, with the stack running:

```bash
ros2 param set /imu_preprocessor fault.noise_scale 10.0
```

```bash
ros2 param set /encoder_preprocessor fault.right_bias_rad_s 0.05
```

```bash
ros2 param set /lidar_preprocessor fault.dropout_ratio 0.6
```

Bring up wheel odometry, measure its drift, and recalibrate it:

```bash
ros2 launch arb_localization odometry.launch.py
```

```bash
python3 scripts/evaluate_odometry.py
```

```bash
python3 scripts/calibrate_odometry.py
```

Compare every estimator against ground truth and write the Phase 5 plots:

```bash
python3 scripts/evaluate_localization.py --tag nominal
```

Run the odometry alone, as the baseline the filter is measured against:

```bash
ros2 launch arb_localization odometry.launch.py ekf:=false
```

Build a map by exploring, and score it against the known floor plan:

```bash
ros2 launch arb_mapping mapping.launch.py
```

```bash
python3 scripts/evaluate_mapping.py --duration 240
```

See what dead reckoning alone does to the map:

```bash
ros2 launch arb_localization odometry.launch.py scan_match:=false
```

Plan a path, and benchmark every heuristic on the map the robot built:

```bash
ros2 launch arb_planning planning.launch.py
```

```bash
ros2 topic pub --once /arb/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 7.0, y: 5.0}, orientation: {w: 1.0}}}"
```

```bash
ros2 run arb_planning planner_benchmark
```

Drive the plan. The two controllers both publish `/cmd_vel`, so exactly one of
them runs at a time — pick one:

```bash
ros2 launch arb_planning planning.launch.py local_planner:=true
```

```bash
ros2 launch arb_planning planning.launch.py local_planner:=false
```

```bash
ros2 launch arb_control control.launch.py
```

Measure a controller. The reference path is published directly, so the planner
and the map are out of the loop and what is measured is the controller:

```bash
python3 scripts/evaluate_navigation.py --mode tracking --controller pid
```

```bash
python3 scripts/evaluate_navigation.py --mode tracking --controller dwa
```

Re-derive every metric and plot from the saved samples, without re-running the
simulator:

```bash
python3 scripts/evaluate_navigation.py --mode recompute
```

Run the whole stack end to end against one goal. `unknown_is_lethal:=false`
lets the planner route through space the robot has not explored yet, which it
must from a cold start:

```bash
ros2 launch arb_planning planning.launch.py local_planner:=true unknown_is_lethal:=false
```

```bash
python3 scripts/evaluate_navigation.py --mode goal --goal goal_room_e
```

Play a fault scenario and score what the estimator did. The scheduler applies
the scenario's knob timeline at runtime; the runner drives a fixed open-loop
route so the TRUE trajectory is identical in every scenario:

```bash
python3 scripts/run_scenario.py --scenario configs/scenarios/case_c_encoder_drift.yaml
```

Or drive the scheduler on its own, against a stack you are watching in RViz:

```bash
ros2 launch arb_evaluation scenario.launch.py scenario:=configs/scenarios/case_e_sensor_outage.yaml
```

Collect every scenario's metrics into the comparison table and chart:

```bash
python3 scripts/fault_report.py
```

Sweep one fault parameter across a range, repeated over seeds. Each run
restarts the whole stack, so this takes a while — `--dry-run` prints the plan
and the estimate first:

```bash
python3 scripts/run_benchmark.py --dry-run
```

```bash
python3 scripts/run_benchmark.py --experiment encoder_bias
```

```bash
python3 scripts/run_benchmark.py --aggregate-only
```

The fifth experiment tests the local planner instead: navigate the corridor
with and without an obstacle that is **not on the map**, paired per seed.

```bash
python3 scripts/run_navigation_benchmark.py --seeds 1 2 3
```

Regenerate the world after editing `configs/world.yaml`:

```bash
python3 scripts/generate_world.py
```

---

## What each subsystem does, and what it measured

One section per phase, each ending in a number that came from running it. The
derivations are in [`docs/`](docs/); this is the tour.

### Simulation

![LiDAR scan overlaid on the generated floor plan](results/phase1/scan_vs_world.png)

A live 360-beam LiDAR scan (red) plotted over the floor plan the world was
generated from. Every beam is placed using the sensor extrinsics from
`configs/robot.yaml` and the robot's ground-truth pose. Agreement between the
red points and the black walls exercises the whole chain at once: world
generation, physics, GPU LiDAR, beam indexing, sensor mounting and the
`ros_gz` bridge.

Measured residual of each beam endpoint to the nearest modelled surface:

| Statistic | Value |
|---|---|
| Mean | -0.0007 m |
| Median | -0.0006 m |
| Standard deviation | **0.0149 m** |
| Configured LiDAR noise sigma | **0.015 m** |
| Beams within 5 cm of a surface | 99.7 % |
| Beams used | 360 / 360 |

The residual spread equals the configured measurement noise, so the geometric
pipeline contributes no bias and no additional error.

### Sensor preprocessing

`arb_sensors` turns the simulator's idealised output into something the
corresponding hardware could actually have produced, and is the single place
faults are injected.

| Stage | What it models |
|---|---|
| IMU | Gauss-Markov bias (exact Ornstein-Uhlenbeck update), scale error, startup bias calibration, honest covariance, `orientation_covariance[0] = -1` to declare that this is not an AHRS |
| Encoders | Quadrature quantisation (1024x4 counts/rev = 1.534 mrad = 0.092 mm of arc), velocity by differencing counts, per-wheel slip and injectable drift |
| LiDAR | One sentinel for every unmeasurable beam, geometric despeckling, range noise, per-beam and bursty dropout |

Three properties are worth calling out, because each was a bug first:

* **Noise is not double-counted.** Gazebo already applies the datasheet noise.
  Adding another full sigma combines as `sqrt(2) * sigma`, making the stream
  41 % noisier than the part being simulated and every published covariance
  wrong by the same factor. Independent variances add, so this stage injects
  only `sqrt((scale*sigma)^2 - sigma^2)` -- exactly zero at nominal.
* **Degrading a sensor widens its reported covariance.** A fault that worsens
  the data without telling the filter teaches it nothing; the EKF keeps
  trusting a sensor that has failed.
* **The fault knob is calibrated.** Despeckling runs before dropout, because
  with it afterwards a commanded 60 % beam loss measured 75.5 %: most survivors
  of a random 60 % cull have no surviving neighbour, so the filter removed them
  too.

### Odometry

`arb_localization/wheel_odometry` integrates the preprocessed encoders and
publishes `nav_msgs/Odometry` plus the `odom -> base_link` transform. It uses
the **exact** SE(2) arc update rather than forward Euler:

```
x     += v dt * sinc(a/2) * cos(theta + a/2)      a = omega * dt
y     += v dt * sinc(a/2) * sin(theta + a/2)      sinc(u) = sin(u)/u
theta += a
```

No singularity at `omega = 0`, midpoint heading, and `sinc` is the chord-to-arc
ratio. Both covariance Jacobians are verified against central finite
differences.

**The effective track width is a calibration parameter, not a measurement.**
A cylindrical wheel touches the floor along a line, and the traction that turns
the robot does not act at the wheel's centre plane. Measured effective width:
**0.3154 m** against a geometric 0.350 m — the distance between the wheels'
inner edges, to within a millimetre.

| Metric over a 10 m route | Geometric `L` | Calibrated `L` |
|---|---|---|
| Final position error | 0.989 m (9.86 %) | **0.095 m (0.94 %)** |
| Final heading error | −72.36° | **6.96°** |
| RMS position error | 0.540 m | **0.055 m** |

![Odometry drift](results/phase3/odometry_drift.png)

The heading error is a staircase: it steps at every turn and is flat on every
straight. That residual is what Phase 4 exists to bound.

### State estimation

A six-state EKF, `[x, y, theta, v, omega, b_gyro]`, fusing the wheel odometry
twist with the IMU gyroscope.

`v` and `omega` are **states rather than control inputs** because two
independent sensors measure them, and that is what lets the filter fuse both
opinions. `b_gyro` is observable for the same reason: the encoders measure
`omega` independently, so the persistent gap between gyro and encoders *is* the
bias. `v_y` is deliberately absent — a differential drive cannot translate
sideways, and an unobservable state is not free.

Measured over 10.03 m, two laps of a rectangle:

| Metric | Wheel odometry | EKF | Change |
|---|---|---|---|
| Position RMSE | 0.0486 m | **0.0155 m** | **−68.0 %** |
| Final position error | 0.0946 m (0.94 %) | **0.0134 m (0.13 %)** | −86 % |
| Heading RMSE | 3.631° | **1.432°** | **−60.6 %** |
| Final heading error | +6.626° | **+0.977°** | −85 % |
| Samples inside reported 2σ | 100 % | 100 % | — |

Estimation latency: median 55 ms — the cost of the timestamp-ordered
measurement buffer that lets a 200 Hz IMU and a 50 Hz encoder stream be fused
in order.

![Covariance evolution](results/phase5/5_covariance_nominal.png)

The odometry's error and its claimed uncertainty both grow without bound; the
EKF's error stays flat. The gyro bounds the *rate* at which heading uncertainty
accumulates, not the position error itself — that needs an absolute measurement,
which arrives in Phase 6 when there is a map to match scans against.

### Mapping and the absolute correction

`arb_mapping` maintains a log-odds occupancy grid, 480 x 360 cells at 5 cm,
integrating each scan with Bresenham ray casting and an inverse sensor model.
The sensor pose comes from TF **at the scan's timestamp**, so tf2 interpolates:
taking the newest pose instead rotates the whole scan by however far the robot
turned while the estimate was catching up, which paints a starburst of free
rays straight through the walls.

`arb_localization/scan_matcher` matches each scan against that map using a
likelihood field and a coarse-to-fine exhaustive search, and publishes a pose
the EKF fuses as an **absolute** measurement. That is the only absolute
measurement in the stack: encoders and gyroscopes measure change, so without it
the position error can only grow. Measured without it, the map came out
structurally correct and rotated several degrees after 82 m of exploration.

| Metric | Reported 1σ, no scan match | With scan match |
|---|---|---|
| Position | 1.003 m | **0.017 m** |
| Heading | 4.10° | **0.33°** |

![Occupancy grid](results/phase6/map_scanmatch.png)

Scored against the floor plan the world was generated from, with a one-cell
tolerance: **precision 0.503, recall 0.436, free-space accuracy 0.949**, 65 % of
cells observed over an 80 m wall-following traverse. The tolerance is strict —
a wall boundary genuinely falls somewhere inside a cell, and the map is a
raster.

The map is built from the robot's own estimate and the estimate is corrected by
the map, so a perfect match proves the scan agrees with the robot's beliefs
rather than with the world. The reported covariance therefore has a floor.
Breaking that circle needs loop closure and pose-graph optimisation, which is
out of scope here and listed under limitations.

### Global path planning

A* and Dijkstra share one implementation: setting the heuristic to zero turns
A* into Dijkstra, because A* *is* Dijkstra told which direction the goal is in.

The tight admissible heuristic for an 8-connected grid is the octile distance,
`(sqrt(2)-1)*min(dx,dy) + max(dx,dy)`, which equals the true cost on an empty
grid. Manhattan is the common trap: correct for a 4-connected grid, and
**inadmissible** with diagonals, where it claims `2d` for a pure diagonal whose
true cost is `1.41d`. Over an 18-instance sweep it was suboptimal on 10 of
them, worst case 8.9 % longer — and found the optimum on the other 8, which is
what makes it hard to notice.

Twelve queries generated from the map the robot built, median of five runs:

| Algorithm | Heuristic | Weight | Length [m] | Time [ms] | Expanded | Excess |
|---|---|---|---|---|---|---|
| Dijkstra | none | 1.0 | 13.211 | 6.764 | 45 836 | 0.00 % |
| A* | euclidean | 1.0 | 13.211 | 2.691 | 13 171 | 0.00 % |
| **A*** | **octile** | **1.0** | **13.211** | **2.812** | **10 947** | **0.00 %** |
| A* | manhattan | 1.0 | 13.174 | 1.658 | 6 573 | +1.30 % |
| A* | octile | 2.0 | 13.322 | 0.959 | 3 896 | +13.99 % |
| A* | octile | 5.0 | 13.544 | 0.669 | 2 847 | +47.00 % |

A* expands **4.2x fewer nodes than Dijkstra** for an identical path. Octile
expands 17 % fewer than Euclidean as its tightness predicts — but generates
more, and the wall-clock is a wash: expansions are the algorithmic metric, time
is the engineering one, and they do not always agree.

Obstacles are inflated by the robot's radius so the planner can treat the robot
as a point, with a decaying soft cost beyond that to keep paths in the middle
of a corridor. Full derivation and the details that are not details:
[docs/path-planning.md](docs/path-planning.md).

### Local obstacle avoidance and control

Two controllers turn the global path into wheel commands, and they are
**alternatives, not companions** — two publishers on `/cmd_vel` is a robot that
does neither thing.

* **DWA** searches *velocities*, not positions, so every candidate is by
  construction reachable within one control period. Obstacles come straight
  from the live scan, never from the map: the map is at best two seconds old and
  cannot contain something that has just walked in front of the robot.
* **Pure pursuit + PID** is the baseline. It tracks the path precisely and has
  *no* obstacle avoidance at all, which is what makes it the right thing to
  measure DWA against: without it there is no way to say what avoidance costs.

The safety rule in DWA is not "reject anything whose rollout hits something" —
that stops a robot with 0.45 m of room dead. It is the **admissible velocity**
condition, `v <= sqrt(2 d a_max)`: the robot may choose any speed it can still
stop from. The same condition now applies to the goal, with `d` the arc length
*remaining along the path*, so the robot arrives instead of overshooting and
being told to halt.

Both controllers are measured on the same 15.60 m three-cornered course, with
the robot started **0.6 m off the path on purpose** — that offset is a step
input, and a step is the only input for which settling time and overshoot have
agreed definitions.

![Path following, PID vs DWA](results/phase9/trajectory.png)

| metric | PID + pure pursuit | DWA |
|---|---|---|
| time | **25.6 s** | 32.0 s |
| cross-track RMSE (steady) | **0.084 m** | 0.085 m |
| worst steady error (a corner) | 0.249 m | **0.222 m** |
| settling time, 0.6 m step | **2.11 s** | 3.55 s |
| overshoot | 4.24 % | **0.00 %** |
| command smoothness, angular | 2.514 rad/s² | **0.234 rad/s²** |
| steering sign changes | 40 | **2** |
| collision-free | yes | yes |

![Step response](results/phase9/step_response.png)

The two agree on steady-state tracking accuracy to within a millimetre and
differ enormously everywhere else. PID converges 1.4 s faster with 4 %
overshoot; DWA converges monotonically with none, and its command stream is
**11x smoother in the angular channel**. Tracking error alone would call these
controllers equivalent — nothing in it shows that one of them is sawing at its
wheels.

Three defects found by running this, none of which any unit test caught first,
are written up in [docs/control.md](docs/control.md): a carrot measured from
the path's first vertex instead of the robot's projection onto it (which
shipped **twice, independently**, in both controllers), a PID differentiating a
wrapped angle, and a rate limiter anchored to the measured velocity that latched
the robot at its maximum turn rate for 117 consecutive seconds.

### End-to-end navigation

The whole stack, one goal, no human in the loop: simulator → sensors → EKF +
scan matching → mapping → A* → DWA → wheels.

![End-to-end navigation to Room E](results/phase9/navigation_goal_room_e.png)

| | `goal_far_east` | `goal_room_e` |
|---|---|---|
| arrived (ground truth, 0.30 m) | yes | yes |
| time | 36.9 s | 36.5 s |
| distance driven | 16.92 m | 16.77 m |
| final error | 0.299 m | 0.300 m |
| EKF error at arrival | 0.084 m | 0.040 m |
| closest approach to anything | 0.365 m | 0.401 m |
| collision-free | yes | yes |

Down the corridor swinging south of the pillar at (−7, 0), through the 1.5 m
doorway at x ∈ [2.0, 3.5], into Room E and around the drum at (5, −4.5). The
planner runs with `unknown_is_lethal:=false` because the robot has not explored
the corridor yet, replanning at 1 Hz as the map fills in.

**Arrival accuracy is limited by localisation, not by control.** The stack stops
when it *believes* it has arrived within 0.20 m; whether it has is a question
about the EKF.

### Fault injection

Six scenarios, one estimator, and the question of what a filter does when its
sensors stop telling the truth. Faults are injected in the **preprocessing
layer**, never in the simulator, so the wheels receive identical commands and
the **true trajectory is the same in all six cases** — any difference is
estimation error and nothing else. The route is open loop for the same reason:
a controller steering on a corrupted estimate would send the robot somewhere
physically different and confound the two.

![Per-scenario error](results/phase10/fault_comparison.png)

| Case | Scenario | Position RMSE [m] | vs A | Heading RMSE [deg] | vs A | Degraded-phase pos. RMSE [m] | Recovery [s] | Inside 2-sigma |
|---|---|---|---|---|---|---|---|---|
| A | Nominal | 0.0452 | 1.00x | 2.97 | 1.00x | -- | n/a | 76 % |
| B | IMU noise x10 | 0.0621 | 1.37x | 4.30 | 1.45x | 0.0535 | 14.9 | 65 % |
| C | Encoder drift | 0.1646 | 3.64x | 12.54 | 4.22x | 0.1162 | **never** | 47 % |
| D | LiDAR dropout 60 % | 0.0352 | 0.78x | 2.60 | 0.88x | 0.0332 | not perturbed | 92 % |
| E | Sensor outage 10 s | 0.0578 | 1.28x | 3.20 | 1.08x | 0.0647 | 0.9 | 56 % |
| F | Multi-degradation | 0.0644 | 1.43x | 4.38 | 1.47x | 0.0544 | 1.4 | 70 % |

Wheel odometry runs alongside as a **control**: cases B, D and E come out at
0.98-1.00x of nominal for it, because none of them touches a wheel encoder.

* **The EKF beats dead reckoning in every case**, by 1.60x to 2.77x on position.
* **The failure modes rank as the theory predicts.** An encoder bias (3.6x) is
  far worse than gyro noise (1.4x), worse than a total sensor outage (1.3x),
  worse than losing 60 % of the LiDAR beams (no measurable effect). What makes a
  fault dangerous is not how much signal it destroys but **whether anything else
  can contradict it**.
* **Case C defeats the stack, and that is the most useful result here.** The
  filter reports ±0.31° of heading confidence while being wrong by 12°, and the
  scan matcher accepts 253 of 253 matches at score 0.96. The map is built at the
  EKF's own estimated pose, so as the estimate drifts the map drifts with it and
  then *confirms* the drifted pose. This is what having no loop closure costs,
  measured.
* **The filter is overconfident**, wrong about its own 2-sigma between 8 % and
  53 % of the time depending on the case.

Every number is a **single run** — read a 2x change as real and anything smaller
as noise. Full write-up, including why Case F did *not* compound:
[docs/fault-injection.md](docs/fault-injection.md).

### Topic graph

```
            gz sim (physics + sensors)
                      |  gz-transport, NOT DDS
              ros_gz_bridge   <- config: arb_simulation/config/ros_gz_bridge.yaml
                      |  ROS 2 DDS
  /clock      /imu/data_raw     /joint_states      /scan_raw
  1000 Hz        200 Hz             50 Hz            10 Hz

  /arb/ground_truth/odom  100 Hz   <- EVALUATION ONLY

  /cmd_vel  --(ROS to GZ)-->  DiffDrive plugin  -->  wheel torques
  /robot_description  -->  robot_state_publisher  -->  /tf, /tf_static

                            arb_sensors
  /imu/data_raw   -->  imu_preprocessor      -->  /arb/imu           200 Hz
  /joint_states   -->  encoder_preprocessor  -->  /arb/wheel_states   50 Hz
  /scan_raw       -->  lidar_preprocessor    -->  /arb/scan           10 Hz

                            arb_localization
  /arb/wheel_states  -->  wheel_odometry  -->  /arb/odom           50 Hz
                                          -->  TF odom->base_link

  /arb/odom + /arb/imu + /arb/scan_match_pose
                       -->  ekf  -->  /arb/pose            50 Hz
                                 -->  /ekf/diagnostics     50 Hz
                                 -->  TF map->odom

                            arb_mapping
  /arb/scan + TF(map->lidar_link) --> mapping --> /arb/map    2 Hz

                    arb_localization/scan_matcher
  /arb/map + /arb/scan --> scan_matcher --> /arb/scan_match_pose  2.5 Hz

                            arb_planning
  /arb/map + /arb/goal + TF --> global_planner --> /arb/global_path     1 Hz
                                              --> /arb/planner_metrics
```

Gazebo and ROS 2 run on separate middlewares. A sensor that is configured
perfectly in the URDF but absent from the bridge config publishes happily
inside Gazebo and is completely invisible to ROS, which looks identical to a
broken sensor.

---

## Configuration

No magic numbers in source. Everything numeric lives in YAML:

| File | Contents |
|---|---|
| [`configs/robot.yaml`](configs/robot.yaml) | Physical geometry, wheel radius/separation, actuator limits, sensor extrinsics |
| [`configs/sensors.yaml`](configs/sensors.yaml) | Nominal sensor specs and the noise model (noise density, bias stability, random walk) |
| [`configs/scenarios/*.yaml`](configs/scenarios/) | The six fault-injection scenarios (Cases A–F) |
| `ros2_ws/src/<pkg>/config/` | Per-node ROS parameters |

`configs/robot.yaml` is the single source of truth for geometry. A mismatch
between the simulated wheel radius and the one used by the odometry equations
produces a systematic scale error that looks exactly like sensor drift — one of
the hardest bugs in this whole project to find after the fact.

---

## Fault-injection scenarios

| Case | Scenario | What it tests |
|---|---|---|
| A | Nominal | Baseline every other case is measured against |
| B | IMU noise ×10 | Does the filter down-weight a degraded sensor, or blindly trust it? |
| C | Wheel encoder drift | A one-wheel bias is indistinguishable from a real turn using encoders alone |
| D | LiDAR beam dropout (60 %, bursty) | Does the NIS chi-square gate reject bad scan matches? |
| E | Total sensor outage (10 s) | Does the filter keep predicting and re-converge, or freeze/diverge? |
| F | Multiple simultaneous degradation | Compounding faults — the interesting case |

---

## Results

Every figure quoted anywhere in this repository is in [`results/`](results/),
one directory per phase, alongside the raw metrics it was derived from.

| Directory | What is in it |
|---|---|
| `results/phase1/` | LiDAR scan overlaid on the generated floor plan |
| `results/phase3/` | Odometry drift before and after track-width calibration |
| `results/phase5/` | Trajectory comparison, per-axis error, covariance evolution, latency |
| `results/phase6/` | Occupancy grids and their scores against the known floor plan |
| `results/phase7/` | Planner benchmark CSV: A* vs Dijkstra over 12 queries |
| `results/phase9/` | Path following and end-to-end navigation, PID vs DWA |
| `results/phase10/` | Six fault scenarios, per-case error traces and `summary.md` |
| `results/phase11/` | Parameter sweeps repeated over seeds, and `raw/` per-run metrics |

The `raw/` metrics are the record; every table and plot is derived from them
and can be re-derived without re-running the simulator:

```bash
python3 scripts/run_benchmark.py --aggregate-only
```

---

## Verification status

This project distinguishes between *written* and *verified*. Nothing is claimed
to work until it has been executed.

| Item | Status | Evidence |
|---|---|---|
| Environment (ROS 2 Jazzy, Gazebo Sim 8.15.0, g++ 13.3, CMake 3.28, Eigen 3.4) | ✅ verified | `./scripts/check_environment.sh` → *Environment OK* |
| `colcon build` | ✅ verified | 11/11 packages, zero stderr |
| Unit tests (`colcon test`) | ✅ verified | **332 tests, 0 failures** |
| Integration tests (`pytest tests/integration`) | ✅ verified | **126 passed, 1 skipped** |
| Unit tests are not vacuous | ✅ verified | **60 code mutations** injected across six packages; all caught (11 escaped first and the tests were strengthened; 1 was an equivalent mutant and was removed) |
| Guard tests actually bite | ✅ verified | 8 probes against the isolation, config-consistency and docs-consistency guards; every one fails the guard it targets and passes once reverted |
| Planner benchmark | ✅ measured | [results/phase7/planner_benchmark.csv](results/phase7/planner_benchmark.csv) |
| Ground-truth isolation guard | ✅ verified | passes, and fails correctly when a violation is injected |
| URDF matches `configs/robot.yaml` | ✅ verified | every dimension compared in `test_urdf_model.py` |
| Node params match the repo config | ✅ verified | `test_config_consistency.py` (caught one real drift) |
| Robot spawns, drives, rests level | ✅ verified | pitch 0.000000 rad, z 0.100000 m |
| Scan matches floor plan | ✅ verified | residual sigma 0.0149 m vs 0.015 m configured noise |
| Odometry integrates a circle exactly | ✅ verified | closes to 1e-9 m, independent of step size |
| Odometry, EKF and matcher Jacobians | ✅ verified | central finite differences |
| Track width calibration | ✅ verified | 0.3500 → 0.3154 m, direction asymmetry 0.00 % |
| Odometry drift | ✅ measured | 0.94 % of distance over 10.03 m |
| EKF drift | ✅ measured | **0.13 %** of distance over 10.03 m |
| EKF improves on odometry | ✅ measured | position RMSE **−68.0 %**, heading RMSE **−60.6 %** |
| EKF covariance is consistent | ✅ verified | 100 % of samples inside the reported 2σ; average NIS 2.0 ± 0.15 against 2 dof |
| Gyro bias is observable and estimated | ✅ verified | converges to a 0.03 rad/s injected bias within 0.004 |
| Every sensor stream is actually fused | ✅ verified | odom 966/983, imu 3843/3913, scan-match 30/30, 0 out of order |
| Scan matching bounds the uncertainty | ✅ measured | reported 1σ position 1.003 m → **0.017 m**, heading 4.10° → **0.33°** |
| Occupancy grid vs known floor plan | ✅ measured | precision 0.503, recall 0.436, free-space accuracy 0.949 at one-cell tolerance |
| Exploration coverage | ✅ measured | 65 % of cells observed over 80 m of wall following |
| Estimation latency | ✅ measured | median 55 ms; the published estimate is predicted forward to the current time |
| A* matches Dijkstra's cost | ✅ verified | identical to 1e-9 on every admissible heuristic |
| A* expands fewer nodes | ✅ measured | **4.2x fewer than Dijkstra** over 12 queries |
| An inadmissible heuristic loses optimality | ✅ measured | Manhattan suboptimal on 10 of 18 instances, worst 8.9 % |
| Diagonal moves do not cut corners | ✅ verified | `DiagonalMovesDoNotCutCorners` |
| Costmap inflation and soft cost | ✅ verified | paths stay within 2 cells of a corridor centreline |
| DWA makes progress on a path held still | ✅ verified | `Dwa.MakesProgressAlongAPathFixedInTheWorld` — caught a carrot bug that covered 1.99 m of a 15.60 m course |
| DWA slows for the end of the path | ✅ verified | `v <= sqrt(2 d a)` applied to the goal, distance measured along the path |
| PID anti-windup and angle-aware derivative | ✅ verified | `Pid.AntiWindupStopsTheIntegralGrowingWhileSaturated`, `Pid.DerivativeOfAWrappedAngleUsesTheAngularDifference` |
| Commands never leave the velocity limits | ✅ verified | `Follower.CommandsStayInsideTheVelocityLimits`, and `EscapesSaturationEvenWhenThePlantOvershootsTheLimit` reproduces the 11 % plant overshoot that latched the controller for 117 s |
| Path following, PID | ✅ measured | cross-track RMSE **0.084 m**, settling 2.11 s, overshoot 4.2 %, collision-free ([results/phase9](results/phase9)) |
| Path following, DWA | ✅ measured | cross-track RMSE **0.085 m**, settling 3.55 s, overshoot **0.0 %**, angular command **11x smoother** |
| One-command bringup | ✅ verified | `ros2 launch arb_bringup bringup.launch.py` brings up all 11 nodes and drives to a goal; RViz config loads clean |
| A boxed-in robot commits to one escape direction | ✅ verified | `Dwa.CommitsToOneDirectionWhenItHasToTurnOnTheSpot` — caught a live stall of 0.3 m in 50 s |
| End-to-end navigation to a goal | ✅ measured | 16.8 m in 36.5 s through a doorway, min clearance 0.40 m, final error 0.300 m |
| Fault scenario timeline (Cases A-F) | ✅ verified | `fault_scheduler` plays all six; ramp, bursts and outages confirmed live (102 parameter sets, 0 rejected, nominal restored) |
| Fault timeline logic is not vacuous | ✅ verified | 57 unit tests; 13 injected mutations, all caught (one escaped first because Case D's burst onset is an exact multiple of its period) |
| Scheduler's nominal table matches the parameter files | ✅ verified | checked both directions — a knob it does not know is a knob it cannot clear |
| Fault state is isolated from every estimator | ✅ verified | same guard as ground truth, and it caught a hole that had made every C++ `#include` line invisible to the static check |
| IMU gyro and accelerometer noise scale independently | ✅ verified | `GyroAndAccelerometerNoiseScaleIndependently` — `affect_accelerometer` was previously a comment, not a setting |
| Fault scenarios measured (Cases A-F) | ✅ measured | [results/phase10/summary.md](results/phase10/summary.md) — EKF 1.60-2.77x better than odometry; Case C not survived |
| Benchmark sweep, 4 estimation experiments | ✅ measured | 72 runs, 3 seeds per point ([results/phase11](results/phase11)) |
| Encoder-bias recovery threshold | ✅ measured | between **0.04 and 0.05 rad/s**; confirms the hypothesis Phase 10 could only raise |
| Filter down-weights a noisy gyro | ✅ measured | **40x** noise change produces **1.7x** error change; no measurable effect below 5x |
| Scan matching survives extreme beam loss | ✅ measured | no degradation at **97 %** dropout (11 of 360 beams); fault confirmed applied (139,546 beams dropped) |
| Filter survives losing its only absolute reference | ✅ measured | 30 s LiDAR outage costs ~2 mm; outage confirmed by counting scan matches (0 in 20 s vs 23 after) |
| Local planner vs a crossing obstacle | ✅ measured | **collides in 3/3 runs** — DWA has no obstacle-velocity model and first brakes at 0.027 m of clearance while needing 0.096 m to stop |
| Collision verdict is not taken from the LiDAR alone | ✅ verified | the scan is blind inside `range_min`, where collisions are; the geometric test from ground truth caught 3 collisions the scan-based one scored as clean |
| Moving obstacle is visible to the LiDAR | ✅ verified | spawns, sweeps its segment, and 50/99 samples return a beam at its range — not an invisible `<actor>` |


---

## Limitations

- Localisation is 2D (SE(2)). Slopes, ramps and non-planar motion are out of scope.
- Mapping is a static occupancy grid, not full SLAM — there is no loop closure
  and no pose-graph optimisation, so long traverses accumulate map error.
- The Gazebo sensor models are idealised approximations of real hardware;
  absolute error figures are meaningful *relative to each other*, not as
  predictions of real-world performance.
- On this hybrid Intel/NVIDIA machine Gazebo logs `libEGL warning: egl: failed
  to create dri2 screen` at startup. The GPU LiDAR still produces correct data
  at the full 10 Hz (verified), but rendering falls back to a slower path.

---

## Future improvements

- Replace the EKF with an Unscented or Invariant EKF and compare consistency (NEES).
- Graph-based SLAM with loop closure instead of a static grid.
- MPC in place of PID for trajectory tracking.
- Hardware validation on a physical differential-drive platform.

---

## Project report

[`docs/project-report.md`](docs/project-report.md) is the single document to
read before explaining this project to somebody else: repository tree,
architecture, algorithms, the mathematics that matters, test and benchmark
results, what every file does, the run commands, what to learn next, and how to
describe it on a CV and in an interview.

## Learning notes

Turkish-language notes explaining the reasoning behind each phase live in
[`docs/learning-notes/`](docs/learning-notes/).

## License

MIT — see [LICENSE](LICENSE).
