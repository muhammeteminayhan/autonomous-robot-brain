# Autonomous Robot Brain — Project Report

The document to read before explaining this project to somebody else. It
cross-references rather than repeats: the derivations live in the per-subsystem
docs, and every number here is reproducible from `results/`.

1. [Repository tree](#1-repository-tree)
2. [Architecture](#2-architecture)
3. [Algorithms](#3-algorithms)
4. [The mathematics that matters](#4-the-mathematics-that-matters)
5. [Test results](#5-test-results)
6. [Benchmark results](#6-benchmark-results)
7. [What each file does](#7-what-each-file-does)
8. [Run commands](#8-run-commands)
9. [What to learn next](#9-what-to-learn-next)
10. [How to talk about it](#10-how-to-talk-about-it)

---

## 1. Repository tree

```
autonomous-robot-brain/
├── configs/                    ONE source of truth, read by URDF + nodes + scripts
│   ├── robot.yaml              chassis, drive, sensor mounts
│   ├── sensors.yaml            noise densities, rates, beam geometry
│   ├── world.yaml              floor plan; the Gazebo world is GENERATED from it
│   ├── benchmarks.yaml         Phase 11 sweep definitions
│   └── scenarios/              Phase 10 fault timelines, cases A-F
├── ros2_ws/src/
│   ├── arb_core/               header-only SE(2), angles, path geometry
│   ├── arb_msgs/               EkfDiagnostics, FaultState, PlannerMetrics
│   ├── arb_description/        URDF/xacro, RViz configs
│   ├── arb_simulation/         Gazebo world + ros_gz bridge
│   ├── arb_sensors/            preprocessing, noise model, fault knobs
│   ├── arb_localization/       wheel odometry, 6-state EKF, scan matcher
│   ├── arb_mapping/            log-odds occupancy grid
│   ├── arb_planning/           A*/Dijkstra, Dynamic Window Approach
│   ├── arb_control/            pure pursuit + PID path follower
│   ├── arb_bringup/            top-level launch, navigation RViz view
│   └── arb_evaluation/         fault scheduler, moving obstacle (Python)
├── scripts/                    verification and measurement, never in the loop
├── tests/integration/          cross-package invariants, no ROS needed
├── docs/                       English reference + Turkish learning notes
├── docker/                     reproducible toolchain
└── results/                    every figure quoted anywhere, per phase
```

**13,700 lines of C++** and **8,300 lines of Python** across 11 packages.

The division is deliberate: **C++ for anything in the loop** (estimation,
mapping, planning, control), **Python for anything that measures it**
(evaluation, scheduling, plotting). Nothing under `scripts/` or
`arb_evaluation/` can affect what the robot does — which is what makes it safe
for them to read ground truth.

---

## 2. Architecture

```
Gazebo ──raw──> arb_sensors ──┬──> wheel odometry ──┐
                              │                     ├──> EKF ──> map ──> A* ──> DWA ──> wheels
                              ├──> IMU  ────────────┤            │
                              └──> LiDAR ───────────┴──> scan matcher
```

Full version with frames and QoS: [architecture.md](architecture.md).

Three decisions worth being able to defend:

**REP-105 frames, `map → odom → base_link`.** `odom → base_link` is smooth and
drifts; `map → odom` is discontinuous and global. Separating them means a
controller can consume a continuous pose while a scan match jumps the global
correction, and neither has to know about the other.

**Ground truth is evaluation-only, and that is enforced, not intended.**
`tests/integration/test_ground_truth_isolation.py` scans the estimation
packages statically and the live graph at runtime. It also covers Phase 10's
fault state, for the same reason: a filter *told* which sensor is lying is
solving a different and much easier problem than one that has to notice.

**One configuration source.** `configs/` is read by the URDF (via
`xacro.load_yaml`), by the nodes, and by the analysis scripts.
`test_config_consistency.py` asserts the two layers agree — it has already
caught one real drift.

---

## 3. Algorithms

| Subsystem | Algorithm | Why this one |
|---|---|---|
| Odometry | Exact SE(2) exponential map | Closes a circle to 1e-9 m *independently of step size*; the Euler approximation does not |
| Fusion | 6-state EKF, Joseph-form update | `[x, y, θ, v, ω, b_gyro]` — the gyro bias is observable because the encoders measure ω independently |
| Outlier rejection | Chi-square NIS gate | An EKF has no other defence against a wrong measurement |
| Mapping | Log-odds occupancy grid, Bresenham | Addition instead of multiplication; numerically stable and clamped |
| Localisation | Likelihood-field scan matching, coarse-to-fine | Bilinear interpolation; no explicit correspondences |
| Global planning | A* with octile heuristic | A* *is* Dijkstra told where the goal is — `h = 0` recovers it exactly |
| Local planning | Dynamic Window Approach | Searches **velocities**, so every candidate is reachable by construction |
| Control | Pure pursuit + PID | Geometry picks where to aim; PID decides how hard |

Detail: [state-estimation.md](state-estimation.md),
[path-planning.md](path-planning.md), [control.md](control.md).

---

## 4. The mathematics that matters

**SE(2) exponential map.** Integrating a constant twist exactly:

```
a = ω·dt,  k = sinc(a/2)
x' = x + v·dt·k·cos(θ + a/2)
y' = y + v·dt·k·sin(θ + a/2)
```

`sinc(u) = sin(u)/u` must be stable through zero, and its **derivative** is
needed for the Jacobian — omitting it left a 0.045 % finite-difference mismatch.

**Process noise scales with `dt`, not `dt²`.** A random walk's variance grows
linearly in time. Writing `(σ·dt)²` instead of `σ²·dt` made the NIS gate reject
real decelerations as outliers, and the resulting position RMSE was **0.298 m
instead of 0.0155 m** — a factor of 19, from one exponent.

**Observability.** `b_gyro` is estimable because two sensors measure ω. `v_y` is
deliberately absent: a differential drive cannot translate sideways, so no
measurement constrains it, and the covariance would grow without bound along a
direction the robot cannot move in. **An unobservable state is not free.**

**Admissibility.** A* returns the optimal path iff `h` never overestimates.
Octile, `(√2−1)·min(dx,dy) + max(dx,dy)`, is exact on an empty 8-connected
grid. Manhattan is the common trap — correct for 4-connected, **inadmissible**
with diagonals — and losing admissibility does not raise an error, it returns a
plausible slightly-wrong path. Measured over 18 instances: suboptimal on 10,
worst case 8.9 %.

**Admissible velocity.** `v ≤ √(2·d·a_max)` — the robot may choose any speed it
can still stop from. This is *not* the same as requiring a clear horizon, which
stops a robot with 0.45 m of room dead.

**The derivative of an angle is not the difference of two angles.** A heading
error sliding from −3.12 to +3.10 rad changed by 0.06, not 6.22. The raw
difference reports +124 rad/s² at 20 Hz, and because a robot turning one way
always wraps the same way, **the spike always has the same sign and reinforces
the rotation that produced it**.

---

## 5. Test results

| | Count |
|---|---|
| Unit tests (`colcon test`) | 332 |
| Integration tests (`pytest tests/integration`) | 127 |
| Code mutations injected, across six packages | 60 |
| …that escaped first and forced a stronger test | 11 |
| …that turned out to be an equivalent mutant, and were removed | 1 |
| Guard probes verifying a test actually bites | 8 |

**Mutation testing is the part worth explaining.** A green suite proves the
tests run, not that they would notice a bug. So defects were deliberately
reintroduced and the suite re-run. Eleven escaped the first time, and each one
exposed a test that was passing for the wrong reason:

* Every controller test **rebuilt its path around the robot each step**. A path
  that slides with the robot makes "the carrot from vertex 0" and "the carrot
  from the robot's projection" the same point forever — which is exactly the
  condition under which that bug is invisible. Two controllers shipped it
  independently.
* A burst-timing test could not distinguish "phase from the fault's onset" from
  "phase from the scenario's start", because Case D's onset (25 s) happens to
  be an exact multiple of its burst period (5 s).
* One mutation turned out to be an **equivalent mutant** — swapping two clamps
  that provably commute once the rate limiter is anchored to the previous
  command. It was removed from the harness rather than chased with a test that
  cannot exist.

Headline measurements, each from `results/`:

| | Odometry | EKF |
|---|---|---|
| Position RMSE over 10.03 m | 0.0486 m | **0.0155 m** (−68.0 %) |
| As a fraction of distance | 0.943 % | **0.134 %** |
| Heading RMSE | 3.63° | **1.43°** (−60.6 %) |

* Scan residual against the floor plan: **σ = 0.0149 m** vs 0.015 m configured
* Track-width calibration: 0.3500 → **0.3154 m**, drift 9.86 % → **0.94 %**
* Scan matching: reported 1σ position **1.003 m → 0.017 m**, heading 4.10° → 0.33°
* A* vs Dijkstra: **4.2× fewer expansions** for an identical path
* Occupancy grid (best run): precision **0.819**, recall 0.731, free-space 0.966
* Path following: cross-track RMSE **0.084 m** (PID) / 0.085 m (DWA)
* End-to-end navigation: 16.8 m through a 1.5 m doorway in 36.5 s, collision-free

---

## 6. Benchmark results

Phase 10 measured each failure mode once; Phase 11 swept them and repeated over
seeds. Tables: [fault-injection.md](fault-injection.md#5-results) and
[benchmarks.md](benchmarks.md#4-results), generated into
`results/phase10/summary.md` and `results/phase11/benchmark_report.md`.

The three findings worth leading with:

**The EKF beats dead reckoning in every scenario, 1.60× to 2.77×.** That is the
whole argument for fusing, measured rather than asserted.

**Failure modes rank as the theory predicts.** An encoder bias (3.6× nominal
error) is far worse than gyro noise (1.4×), worse than a total sensor outage
(1.3×), worse than losing 60 % of the LiDAR beams (no measurable effect). What
makes a fault dangerous is not how much signal it destroys but **whether
anything else can contradict it**.

**A ramped one-wheel encoder bias defeats the stack, and that is the most
useful result in the project.** The filter reports ±0.31° of heading confidence
while being wrong by 12°, and the scan matcher accepts 253 of 253 matches at
score 0.96. The map is built at the EKF's own estimated pose, so as the
estimate drifts the map drifts with it and then *confirms* the drifted pose.
The NIS gate is working — 319 gated measurements are the biased ones — and it
cannot help, because the only alternative reference is corrupted in the same
direction. This is precisely what having no loop closure costs.

---

## 7. What each file does

### Core and interfaces

| File | Role |
|---|---|
| `arb_core/angles.hpp` | `normalizeAngle` via `std::remainder` — O(1) and NaN-safe, unlike a `while` loop |
| `arb_core/pose2d.hpp` | `Pose2D` as an SE(2) element: `compose`, `inverse`, `relativeTo` |
| `arb_core/path_geometry.hpp` | `projectOntoPath`, `carrotAt` — shared after the same carrot bug shipped twice |
| `arb_msgs/*.msg` | Diagnostics, fault state, planner metrics |

### Estimation

| File | Role |
|---|---|
| `arb_sensors/noise_model.hpp` | Gauss-Markov bias, exact OU discretisation, `added_sigma` |
| `arb_sensors/{imu,encoder,lidar}_processor.cpp` | Per-sensor noise, quantisation, despeckle, fault knobs |
| `arb_localization/differential_drive_odometry.cpp` | Exact SE(2) integration with analytic Jacobians |
| `arb_localization/ekf.cpp` | 6-state filter, Joseph form, NIS gating |
| `arb_localization/ekf_node.cpp` | Timestamp-ordered measurement buffer, forward-predicted publishing |
| `arb_localization/scan_matcher.cpp` | Likelihood field, bilinear interpolation, coarse-to-fine search |
| `arb_mapping/occupancy_grid.cpp` | Log-odds fusion, direction-canonicalised Bresenham |

### Decision

| File | Role |
|---|---|
| `arb_planning/grid_planner.cpp` | A*/Dijkstra, costmap inflation, BFS distance transform |
| `arb_planning/dynamic_window.cpp` | Velocity-space search, admissible-velocity condition |
| `arb_control/path_follower.cpp` | Pure pursuit, PID with anti-windup and angle-aware derivative |

### Measurement

| File | Role |
|---|---|
| `arb_evaluation/scenario.py` | `knobs_at(scenario, t)` — pure, ROS-free, 58 tests |
| `arb_evaluation/fault_scheduler.py` | Plays a scenario on sim time, publishes `FaultState` |
| `arb_evaluation/moving_obstacle.py` | Spawns and drives an obstacle absent from the map |
| `scripts/evaluate_localization.py` | Phase 5: RMSE, covariance consistency, latency |
| `scripts/evaluate_mapping.py` | Phase 6: map scored against the known floor plan |
| `scripts/evaluate_navigation.py` | Phase 9: tracking error, settling, overshoot, smoothness |
| `scripts/run_scenario.py` | Phase 10: one scenario, driven and scored |
| `scripts/run_benchmark.py` | Phase 11: sweeps, repeated over seeds |
| `scripts/check_environment.sh` | Runs first; assumes no tool exists until it is found |

---

## 8. Run commands

```bash
source scripts/env.sh
```

```bash
ros2 launch arb_bringup bringup.launch.py
```

Then click **2D Goal Pose** in RViz. Everything else — per-stage launches,
evaluation scripts, benchmark sweeps — is in the README's
[Usage](../README.md#usage) section.

```bash
./scripts/build.sh && ./scripts/run_tests.sh
```

---

## 9. What to learn next

Ordered by how much it would improve *this* project:

1. **Graph SLAM with loop closure.** The single biggest weakness, and Phase 10
   measured its cost exactly. A pose graph with loop closure is what stops the
   map confirming the drift that produced it. Read: Grisetti et al., *A Tutorial
   on Graph-Based SLAM*.
2. **Particle filters / AMCL.** An EKF's Gaussian cannot represent "I am in one
   of two identical corridors". A particle filter can, and knowing when each is
   the right tool is a standard interview question.
3. **Observability and consistency analysis.** This filter is measurably
   overconfident (wrong about its own 2σ between 8 % and 53 % of the time).
   Understanding *why* needs the theory, not more tuning.
4. **MPC instead of PID.** PID reacts; MPC plans over a horizon subject to
   constraints. The DWA rollout is already a crude one-step version.
5. **`ros2_control`.** Real robots use it for hardware interfaces and
   controller lifecycle; this project drives `/cmd_vel` directly.
6. **Behaviour trees / Nav2.** Recovery behaviours are the piece this stack has
   none of — a robot that gets genuinely stuck stays stuck.

---

## 10. How to talk about it

### On a CV

> **Autonomous Robot Brain** — ROS 2 / C++ / Gazebo
> Built a full autonomy stack for a differential-drive robot: sensor
> preprocessing with calibrated noise models, a 6-state EKF fusing wheel
> odometry, gyroscope and LiDAR scan matching, log-odds occupancy mapping, A*
> global planning and DWA local avoidance. Reduced position error by **68 %**
> versus dead reckoning (0.0155 m RMSE over 10 m) and validated under six
> injected fault scenarios plus parameter sweeps repeated over seeds. 332 unit
> and 127 integration tests, hardened against 60 deliberately
> injected mutations.

### In the interview

Lead with a **failure**, not the architecture. Anyone can list nodes; being
able to say what broke and how you found it is the differentiator.

The three strongest stories:

**"The bug that shipped twice."** A pure-pursuit carrot measured from the
path's first vertex instead of the robot's projection onto it. Invisible for
the first second of every run, then fatal. It passed every unit test because
every test rebuilt the path around the robot — and it hid behind the global
planner's 1 Hz replanning in normal operation, surfacing only when a path was
held still. Fixed by moving the geometry into a shared module with its own
tests. *Shows: you understand why a test can pass for the wrong reason.*

**"The filter that lied."** Under a ramped one-wheel encoder bias the EKF
reported ±0.31° of heading confidence while being 12° wrong, and the scan
matcher confirmed it at score 0.96 — because the map was built at the pose it
was supposed to be correcting. *Shows: you check covariance consistency, not
just error, and you can reason about a whole system rather than a component.*

**"The planner that could not see it coming."** DWA collided with a crossing
obstacle in every one of three runs, first braking at 0.027 m of clearance
while needing 0.096 m to stop. Not carelessness: DWA rolls a trajectory forward
through the *current* obstacle positions, and a crossing obstacle enters that
swept volume after the rollout was computed. The admissible-velocity condition
`v ≤ √(2·d·a_max)` works exactly as designed — `d` assumes the world holds
still. The fix is velocity obstacles or a predicted-motion costmap layer, not
tuning. *Shows: you can tell a bug from a design limit, and you know the
standard answer.*

**"One exponent, a factor of 19."** Process noise written `(σ·dt)²` instead of
`σ²·dt`. The NIS gate started rejecting real decelerations as outliers and
position RMSE was 0.298 m instead of 0.0155 m. *Shows: you know why random-walk
variance is linear in time, and that you found it by measuring rather than by
staring at the code.*

### What to say about the limits

Volunteer them. "No loop closure, so systematic drift corrupts the map that
should correct it — I measured that, it's the encoder-bias case." "No recovery
behaviours, so a genuinely stuck robot stays stuck." "Three seeds per benchmark
point, which shows whether an effect exceeds the scatter and not much more."

Knowing exactly where your work stops being trustworthy is the most senior
thing you can demonstrate.
