# State Estimation

> Phases 3–5. Phase 3 (wheel odometry) is complete and measured; the EKF is
> Phase 4.

## 1. Differential-drive kinematics

Two wheels of radius `r`, a distance `L` apart, rolling without slipping. Each
contact point moves at `v = r * omega_wheel`. The body is rigid, so both share
one angular velocity:

```
v_r = v + omega * L/2          v     = (v_r + v_l) / 2 = r (omega_r + omega_l) / 2
v_l = v - omega * L/2   ==>    omega = (v_r - v_l) / L = r (omega_r - omega_l) / L
```

A differential drive cannot move sideways, so the body-frame velocity is always
`(v, 0, omega)` and

```
x_dot = v cos(theta),   y_dot = v sin(theta),   theta_dot = omega
```

## 2. Exact arc integration

Forward Euler (`x += v cos(theta) dt`) assumes the heading is constant over the
step and therefore always cuts the corner while turning. The error is
systematic, not random: a robot driving in a circle spirals inwards.

With `v` and `omega` constant over the step the path is a circular arc, and the
exact update follows from the sum-to-product identities with `a = omega * dt`:

```
x     += v dt * sinc(a/2) * cos(theta + a/2)
y     += v dt * sinc(a/2) * sin(theta + a/2)
theta += a
```

with `sinc(u) = sin(u)/u`, `sinc(0) = 1`. This is the exponential map of SE(2)
applied to the body twist, so it is exact rather than merely accurate. It has
no singularity at `omega = 0`, uses the midpoint heading (second-order
accuracy), and the `sinc` factor is precisely the chord-to-arc ratio.

Implementation: `arb_localization/src/differential_drive_odometry.cpp`.

## 3. Covariance propagation

```
P <- Fx P Fx' + Fu Q Fu'
```

`Q` is diagonal in wheel-arc space and proportional to `|arc|`: a wheel that
slips does so in proportion to how far it turned, so variance is linear in
distance and standard deviation grows as `sqrt(distance)`. The two wheels are
independent, which is the only reason heading uncertainty grows at all.

`Fu` includes the derivative of the `sinc` factor. The textbook Jacobian omits
it because the textbook update is the midpoint approximation, which has no
`sinc` to differentiate. Integrating exactly and differentiating the
approximation leaves `F` inconsistent with `f`, measured here as a 0.045 %
disagreement with a central finite difference — small, systematic, and exactly
what makes an EKF quietly overconfident.

Both Jacobians are verified against central finite differences in
`test_odometry.cpp`.

## 4. Calibration: L is not a geometric measurement

The URDF says the wheels are 0.350 m apart, and it is right — that is where
their centre planes are. The physics disagrees, because a cylindrical wheel
touches the floor along a **line**, and the traction that turns the robot does
not act at the centre plane.

Measured with `scripts/calibrate_odometry.py` (three full turns in each
direction, direction asymmetry 0.00 %):

| Parameter | Geometric | Calibrated | Difference |
|---|---|---|---|
| Track width `L` | 0.3500 m | **0.3154 m** | −9.90 % |
| Wheel radius `r` | 0.06000 m | 0.05997 m | −0.05 % |

0.3154 m is the distance between the wheels' **inner edges**
(0.350 − 0.035 = 0.315 m) to within a millimetre.

Effect on a 10 m route of two rectangle laps:

| Metric | Geometric `L` | Calibrated `L` |
|---|---|---|
| Final position error | 0.989 m (9.86 % of distance) | **0.095 m (0.94 %)** |
| Final heading error | −72.36° | **6.96°** |
| RMS position error | 0.540 m | **0.055 m** |

A tenfold improvement from one number. A systematic error is not noise, and no
Kalman filter removes a bias it has not been told about — which is why this is
fixed in Phase 3 rather than tuned around in Phase 4.

On real hardware, where ground truth is unavailable, the standard substitute is
UMBmark (Borenstein & Feng, 1996): drive a square clockwise and anticlockwise
and solve the same two unknowns from the pattern the end-point errors make.

## 5. Where the residual error lives

![Odometry drift](../results/phase3/odometry_drift.png)

The heading-error trace is a staircase: it steps at every turn and is flat on
every straight. After calibration roughly 0.87° remains per 90° turn. Straight
running contributes almost nothing.

This is the measurement that motivates Phase 4. Dead reckoning has no absolute
reference, so its uncertainty can only grow; an independent heading source (the
gyro) bounds the rate of growth, and an absolute source (LiDAR scan matching
against the map) bounds the error itself.

## 6. Frames

```
map --(EKF correction: slow, DISCONTINUOUS)--> odom --(odometry: fast, SMOOTH)--> base_link
```

`arb_localization/wheel_odometry` publishes `odom -> base_link` only. The
`map -> odom` correction belongs to the EKF. Writing a correction into
`odom -> base_link` destroys the smoothness controllers depend on, and is the
most common way to break a ROS navigation stack.
