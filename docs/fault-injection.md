# Fault Injection

Phase 10. Six scenarios, one estimator, and the question of what a filter does
when its sensors stop telling the truth.

Every number here comes from `results/phase10/` and is reproducible with
`scripts/run_scenario.py` followed by `scripts/fault_report.py`.

---

## 1. Where a fault is injected, and why it matters

Faults are applied in the **sensor preprocessing layer** — `arb_sensors` — not
in the simulator:

```
Gazebo  --raw-->  arb_sensors  --processed-->  EKF / mapping / planning
                       ^
                       |  fault.* parameters, set at runtime
                  fault_scheduler
```

That placement is the whole experiment. The wheels receive the same commands in
every scenario and the simulator knows nothing about the fault, so **the true
trajectory is identical across all six cases**. Any difference between Case A
and Case C is estimation error and nothing else.

Injecting inside Gazebo instead would change the physics, the robot would
physically drive somewhere different, and the resulting error would confound
estimation quality with trajectory difference — with no way to separate them
afterwards.

The same reasoning forces an **open-loop route**. The robot is driven by timed
velocity commands, never towards a goal. A closed-loop route would steer on the
corrupted estimate and destroy the property the comparison rests on.

---

## 2. The three pieces

| Piece | Where | What it is |
|---|---|---|
| Knob timeline | `arb_evaluation/scenario.py` | A **pure function** of (scenario, time). No ROS. |
| Scheduler | `arb_evaluation/fault_scheduler.py` | Reads the clock, calls parameter services, publishes `FaultState`. |
| Runner + scorer | `scripts/run_scenario.py` | Drives the route, records, scores. |

The split is deliberate. `knobs_at(scenario, t)` being pure is the only reason
the timeline is testable at all: asserted against a running simulator, a wrong
ramp and a slow parameter service look exactly the same. It has 56 unit tests
and 13 injected mutations, all caught.

### Sim time, not wall time

Gazebo's `real_time_factor` is uncapped so batch experiments run fast, and this
world runs well above 1.0. A scheduler on wall time would apply a fault
declared for t = 20 s at whatever simulated instant happened to coincide, the
robot would be somewhere different every run, and Case A would not be a
baseline for anything.

### Polling, not a precomputed schedule

A list of edges is tempting and wrong: an encoder bias **ramps continuously**
over five seconds, so there is no finite set of instants at which the correct
value changes. Polling at `rate_hz` makes the ramp's resolution explicit rather
than hidden.

### Composition, not file order

Faults overlap — that is the point of Case F. Where two touch the same knob the
**stronger wins**: a multiplier takes the larger factor, a bias the larger
magnitude, an outage wins outright. File order is not a physical property and
must not decide what the robot experiences.

---

## 3. The six cases

| Case | Scenario | The question it asks |
|---|---|---|
| A | Nominal | The baseline every other case is measured against |
| B | IMU noise x10 from 20 s | Does the filter **down-weight** a degraded sensor, or keep trusting it? |
| C | Right-wheel bias, ramped to 0.05 rad/s | A one-wheel bias **is a turn** as far as the encoders can tell |
| D | 60 % LiDAR beams dropped, bursty | Does the NIS gate **reject** a scan match computed from too few beams? |
| E | Total IMU outage 30-40 s, LiDAR outage 70-80 s | Does the filter keep **predicting** and re-converge, or freeze? |
| F | Encoder drift + 5x IMU noise + 30 % dropout, overlapping | Individually survivable faults **compounding** |

### Every fault clears before the scenario ends

Each fault stops 25 s before the run does. The first version of these files ran
every fault to the last second, which made **recovery time** — one of the three
numbers this phase exists to report — unmeasurable for four of the six cases:
there was simply no interval in which the sensors were healthy again.
`test_every_scenario_with_a_fault_leaves_a_recovery_window` now enforces the
tail.

### Case C is the nasty one

A constant bias on one wheel is **indistinguishable from a genuine turn** using
encoders alone. Dead reckoning curves away silently and consistently — there is
no residual, no disagreement, nothing to detect. Only an independent heading
source (the gyro) or an absolute one (LiDAR scan matching) can catch it, and
that is exactly why the EKF fuses both.

The bias **ramps** over five seconds rather than stepping. A step is easy to
detect and is rarely how hardware fails: a loosening wheel coupling or a warming
MEMS element degrades over seconds, and there is no single instant at which
something obviously changed.

---

## 4. What is measured

**Position RMSE** and **heading RMSE**, each split three ways — the healthy
phase before the first fault, the degraded phase, and the recovered phase after
the last fault clears. A single number over a run that was healthy for a third
of it says nothing about either half.

**Recovery time**: seconds from the last fault clearing until the position
error is back inside the band it held *while healthy in that same run*, and
**stays** there for 3 s. A single sample dipping under the threshold is noise.

The band is measured against the run's own healthy phase, not against Case A.
Two runs differ by more than the noise floor for reasons that have nothing to do
with the fault — physics seeds, where the map happened to be built — and a
threshold taken from a different run would make recovery time an artefact of
that difference.

The result is reported with an **outcome**, not just a number: `no fault`,
`no post-fault window`, `never left the band`, `recovered`, `did not recover`.
Collapsing "there was no fault" and "it recovered instantly" both to 0.0 would
make them indistinguishable in the table.

### The phase split is confounded with distance, and the table says so

A dead-reckoning estimator's error grows with **distance travelled**, not with
the fault. Wheel odometry does not consume the IMU at all, yet its Case B
numbers still read healthy 0.011 m, degraded 0.084 m, recovered 0.154 m — that
is drift accumulating over four laps, not a response to a fault it cannot see.

So the phase split is a **diagnostic, not a comparison**. Two comparisons are
sound and the table is built from them:

* **EKF vs odometry within one case** — same run, same distance, same
  trajectory, and only one of the two can see the fault.
* **Each case's EKF against Case A's** — same route, same duration, same total
  distance, differing only in what was injected. This is the `vs A` column.

Odometry is in the table as a **control**: it shows what the phase split looks
like for an estimator the fault provably cannot touch.

**Covariance consistency**: the fraction of samples whose error fell inside the
filter's own reported 2-sigma. This is the difference between a filter that
degrades and one that **lies**. A filter whose error grows while its covariance
stays tight is worse than a bad filter, because everything downstream believes
it.

---

## 5. Results

<!-- FILLED IN FROM results/phase10/summary.md -->

---

## 6. Case C defeats the stack, and the reason is worth the whole phase

Case C is the one that does not end well, and the diagnosis is more useful than
a pass would have been.

| | odometry | EKF |
|---|---|---|
| position RMSE | 0.435 m | **0.165 m** |
| heading RMSE | 35.4 deg | **12.5 deg** |
| position RMSE *after the fault cleared* | 0.657 m | 0.279 m |
| heading RMSE *after the fault cleared* | 54.1 deg | 20.9 deg |
| recovered? | **no** | **no** |
| error inside its own 2-sigma (heading) | 41 % | **13 %** |

The EKF cuts the error by 2.6x versus dead reckoning, and **neither recovers**.
Worse, both are further off *after* the fault clears than during it.

The logs say exactly why:

```
scan match: 253 accepted / 253 attempted (0 rejected) |
            last score 0.963, correction 0.014 m 0.23 deg
EKF: 1-sigma pos=0.018 m heading=0.31 deg | odom 6447/6464 fused (319 gated)
```

The filter believes its heading to **±0.31 degrees while being wrong by 12**.
The scan matcher accepts every match, scores them 0.96+, and applies
corrections of a centimetre.

**The map is built at the EKF's estimated pose.** As the estimate drifts, the
map drifts with it, and scan matching against that map then *confirms* the
drifted pose with high confidence. There is no external reference anywhere in
the loop — the absolute correction is only absolute relative to something the
estimate itself drew.

The NIS gate is working: `319 gated` odometry measurements are the biased ones
being rejected. It cannot help, because the alternative reference is corrupted
in the same direction.

This is the cost of the limitation the README has stated since Phase 6 — a
static occupancy grid with **no loop closure and no pose-graph optimisation**.
Case C is what that costs when the drift is systematic rather than random. The
fix is not a better filter; it is SLAM with loop closure, or a landmark the
robot did not draw itself.

---

## 7. A knob that could not do what the config claimed

`configs/scenarios/*.yaml` declares `affect_accelerometer` per IMU fault: Case B
degrades the whole package, Case F only the gyro. The preprocessor had a single
`fault.noise_scale` that scaled **both** channels, so that setting was a comment
rather than a setting — Case F degraded the accelerometer anyway and nothing
said so.

`fault.accel_noise_scale` now exists separately. It cannot change the pose
estimate, and that is the point: **the EKF does not fuse the accelerometer at
all**. For a planar robot at walking pace the useful signal is buried under
gravity leakage from millidegree chassis tilt, and integrating it twice to get
position is strictly worse than the wheel encoders. Case B measures that claim
rather than asserting it.

A config value that is silently ignored is worse than no config value.

---

## 8. What is not tested here

* **No fault detection.** Nothing in the stack decides a sensor has failed and
  acts on it. The NIS gate rejects individual bad measurements, which is not the
  same thing as noticing that a sensor is broken and reconfiguring around it.
* **No sensor recovery logic.** After an outage the filter simply resumes
  fusing. A real system would re-check calibration before trusting the sensor
  again.
* **Faults are injected downstream of the simulator**, so they cannot model a
  fault that changes the physics — a slipping wheel, a dragging brake. Those
  would need the simulator itself to change.
