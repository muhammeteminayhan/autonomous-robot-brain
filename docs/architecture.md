# Architecture

## 1. Problem statement

A mobile robot is placed in an unknown indoor environment. It must:

1. estimate **where it is** from noisy, partially faulty proprioceptive and
   exteroceptive sensors,
2. build a **map** of what is around it,
3. plan a **collision-free path** to a commanded goal,
4. **follow** that path while reacting to obstacles that were not in the map,
5. keep doing all of the above when sensors degrade or fail.

No global positioning is available. The only absolute information in the system
comes from matching the current LiDAR scan against the map the robot itself
built — which means errors are self-referential and can compound. That is the
central engineering difficulty and the reason the estimator is the heart of the
project rather than the planner.

## 2. Pipeline

```
   Gazebo Harmonic  (physics + sensor simulation)
           │
           │  /imu/data_raw   /joint_states   /scan_raw   (via ros_gz_bridge)
           ▼
   ┌───────────────────────────────────────────────┐
   │ arb_sensors                                   │  PHASE 2 / 10
   │  · time alignment, unit conversion            │
   │  · configurable noise + bias + dropout        │
   │  · fault injection (6 scenarios)              │
   └───────────────────────────────────────────────┘
           │  /arb/imu  /arb/wheel_states  /arb/scan  /arb/fault_state
           ▼
   ┌───────────────────────────────────────────────┐
   │ arb_localization                              │  PHASE 3 / 4
   │  ┌─────────────────────┐                      │
   │  │ wheel_odometry      │ diff-drive kinematics│
   │  └──────────┬──────────┘                      │
   │             │ /arb/odom  (dead reckoning)     │
   │  ┌──────────▼──────────┐                      │
   │  │ ekf_node            │ predict / update     │
   │  │  + scan_matcher     │ LiDAR correction     │
   │  └─────────────────────┘                      │
   └───────────────────────────────────────────────┘
           │  /arb/pose  (map→odom TF)  /ekf/diagnostics
           ├──────────────────────────────┐
           ▼                              ▼
   ┌────────────────────┐      ┌──────────────────────┐
   │ arb_mapping        │      │ arb_control          │  PHASE 9
   │  occupancy grid    │      │  PID path follower   │
   │  log-odds fusion   │      └──────────┬───────────┘
   └─────────┬──────────┘                 │
             │ /arb/map                   │
             ▼                            │
   ┌────────────────────────────┐         │
   │ arb_planning               │ PHASE 7/8         
   │  global_planner  (A*/Dijkstra)       │
   │  local_planner   (DWA)     ├─────────┘
   └────────────────────────────┘
             │ /cmd_vel
             ▼
        Gazebo diff-drive plugin  →  robot moves
```

The loop is closed: motion changes the sensor readings, which change the
estimate, which changes the map, which changes the plan. Every error source is
therefore *recursive*, and stability — not peak accuracy — is the design goal.

## 3. Package decomposition and why it differs from the initial sketch

The requested layout had a flat `ros2_ws/src` plus top-level `configs/` and
`tests/`. The implemented layout keeps those directories but adds structure:

| Package | Responsibility | Language | Phase |
|---|---|---|---|
| `arb_core` | Header-only math: angle algebra, `Pose2D` (SE(2)), running statistics | C++ (INTERFACE lib) | 0 |
| `arb_msgs` | Custom interfaces: `EkfDiagnostics`, `FaultState`, `PlannerMetrics` | IDL | 0 |
| `arb_description` | URDF/xacro robot model, RViz config | XML | 1 |
| `arb_simulation` | Gazebo worlds, `ros_gz` bridge config, spawn launch | SDF/Python | 1 |
| `arb_sensors` | Sensor preprocessing, noise model, fault injection | C++ | 2, 10 |
| `arb_localization` | Wheel odometry, EKF, LiDAR scan matching | C++ | 3, 4 |
| `arb_mapping` | 2D occupancy grid, log-odds, inverse sensor model | C++ | 6 |
| `arb_planning` | A*, Dijkstra, Dynamic Window Approach | C++ | 7, 8 |
| `arb_control` | PID trajectory tracking | C++ | 9 |
| `arb_bringup` | Top-level launch composition and parameter sets | CMake/Python | 1+ |
| `arb_evaluation` | Fault-scenario scheduler, moving obstacle, offline metrics and benchmark reports | Python | 5, 10-11 |

### Why `arb_core` was added

Angle normalisation, SE(2) composition and running statistics are needed by
odometry, the EKF, the planner *and* the controller. Without a shared package
each of them grows its own copy, the copies diverge, and a bug fixed in one
place survives in three others. It is header-only (`INTERFACE` CMake target)
so there is no linking cost and no ABI surface.

### Why `arb_msgs` is separate from the nodes that publish it

Interface packages must be buildable without their producers, otherwise any
consumer pulls in the full node dependency tree. This is standard ROS 2
practice and it keeps `colcon build --packages-up-to arb_msgs` fast.

### Why configuration lives in two places

- `configs/` (repo root) holds **experiment-level** configuration: the robot's
  physical constants and the six fault scenarios. These are read by scripts and
  by the evaluation toolkit, which run *outside* any ROS package.
- `<package>/config/` holds **node-level** ROS parameters, because that is
  where `ament_index` and `launch_ros` expect to find them.

Splitting them means a reviewer can change "what experiment am I running"
without touching "how is this node tuned", and vice versa.

### Why tests live in two places

- `<package>/test/` holds **unit tests** (gtest for C++, pytest for Python),
  discovered automatically by `colcon test`.
- `tests/integration/` holds **system tests** that span packages — for example,
  the ground-truth isolation check below, which must inspect the whole node
  graph and cannot belong to any single package.

## 4. Ground truth isolation — a hard architectural rule

Gazebo can publish the robot's exact pose. It is published on
`/arb/ground_truth/pose` and it is **evaluation-only**.

The rule: no node in `arb_localization`, `arb_mapping`, `arb_planning` or
`arb_control` may subscribe to any ground-truth topic, directly or indirectly.

Why this deserves to be enforced rather than merely intended: ground truth
leaking into an estimator is the single most common way robotics demos become
meaningless. The trajectory plot looks perfect, the RMSE is 2 mm, and the stack
does not work on hardware. The failure is invisible unless you go looking.

Enforcement is mechanical, in `tests/integration/test_ground_truth_isolation.py`:
the test brings up the full graph, enumerates every subscription of every
estimation node, and fails if a ground-truth topic appears. A second, cheaper
check greps the source tree so the violation is caught at review time too.

## 5. Coordinate frames (REP-105)

```
map ──(EKF correction, slow, discontinuous)──▶ odom ──(wheel odometry, fast, smooth)──▶ base_link
                                                                                          ├─▶ imu_link
                                                                                          ├─▶ lidar_link
                                                                                          └─▶ base_footprint
```

- `odom → base_link` is published by the wheel odometry node. It is **smooth
  and continuous** but drifts without bound. Controllers use it because a jump
  in this transform would jerk the robot.
- `map → odom` is published by the EKF. It is **globally consistent but
  discontinuous** — it jumps whenever a scan match corrects accumulated drift.
  Planners use `map` because they need global consistency.

Keeping the correction in `map → odom` rather than overwriting `odom →
base_link` is what lets both properties coexist. This is the single most
misunderstood part of the ROS navigation stack and it is worth being able to
explain in an interview.

## 6. Topic contract

| Topic | Type | Publisher | Rate | Consumers |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `arb_control` | 20 Hz | Gazebo diff-drive |
| `/imu/data_raw` | `sensor_msgs/Imu` | Gazebo bridge | 200 Hz | `arb_sensors` |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo bridge | 50 Hz | `arb_sensors` |
| `/scan_raw` | `sensor_msgs/LaserScan` | Gazebo bridge | 10 Hz | `arb_sensors` |
| `/arb/imu` | `sensor_msgs/Imu` | `arb_sensors` | 200 Hz | `arb_localization` |
| `/arb/wheel_states` | `sensor_msgs/JointState` | `arb_sensors` | 50 Hz | `arb_localization` |
| `/arb/scan` | `sensor_msgs/LaserScan` | `arb_sensors` | 10 Hz | `arb_localization`, `arb_mapping`, `arb_planning` |
| `/arb/fault_state` | `arb_msgs/FaultState` | `arb_sensors` | 10 Hz | logging only |
| `/arb/odom` | `nav_msgs/Odometry` | `arb_localization` | 50 Hz | `arb_localization` (EKF) |
| `/arb/pose` | `geometry_msgs/PoseWithCovarianceStamped` | `arb_localization` | 50 Hz | `arb_planning`, `arb_control` |
| `/ekf/diagnostics` | `arb_msgs/EkfDiagnostics` | `arb_localization` | 50 Hz | logging, `arb_evaluation` |
| `/arb/map` | `nav_msgs/OccupancyGrid` | `arb_mapping` | 2 Hz | `arb_planning` |
| `/arb/global_path` | `nav_msgs/Path` | `arb_planning` | on demand | `arb_control` |
| `/arb/ground_truth/pose` | `geometry_msgs/PoseStamped` | Gazebo bridge | 100 Hz | **`arb_evaluation` only** |

## 7. QoS policy

| Data class | Reliability | Durability | Depth | Rationale |
|---|---|---|---|---|
| Sensor streams | `BEST_EFFORT` | `VOLATILE` | 5 | A dropped scan is better than a late scan; retransmission would deliver stale data into a real-time filter. |
| `/cmd_vel` | `RELIABLE` | `VOLATILE` | 1 | A lost command means the robot keeps its previous velocity. Depth 1 because only the newest command is meaningful. |
| `/arb/map` | `RELIABLE` | `TRANSIENT_LOCAL` | 1 | A planner that starts late must still receive the current map. |
| TF | `RELIABLE` | `VOLATILE` | 100 | Standard `tf2` requirement. |
| Diagnostics | `RELIABLE` | `VOLATILE` | 50 | Analysis must not silently lose samples. |

## 8. Execution model

Each subsystem is a separate process during development so that a crash is
localised and `ros2 topic hz` gives honest per-node timing.

A composed-container launch (`rclcpp_components`), loading the estimation nodes
into one process for intra-process zero-copy, was planned for Phase 12 and was
**not built** — Phase 12 became documentation and verification instead. It is
listed under Future improvements in the README rather than left as a promise
here, because the case for it is unproven: at 10 Hz for the LiDAR and 200 Hz
for the IMU nothing in this stack is bandwidth-bound, and the measured
estimation latency is 55 ms median, of which serialisation is not the
dominant term.

The EKF runs on a fixed-rate timer rather than being driven by measurement
arrival, so prediction cadence stays constant even during a sensor outage
(Case E). Measurements are buffered and applied by timestamp, not by arrival
order.
