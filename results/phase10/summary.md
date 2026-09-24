# Phase 10 -- fault injection results

## ekf

| Case | Scenario | Position RMSE [m] | vs A | Heading RMSE [deg] | vs A | Degraded-phase pos. RMSE [m] | Recovery [s] | Inside 2-sigma |
|---|---|---|---|---|---|---|---|---|
| A | Nominal | 0.0452 | 1.00x | 2.97 | 1.00x | -- | n/a | 76 % |
| B | IMU noise x10 | 0.0621 | 1.37x | 4.30 | 1.45x | 0.0535 | 14.9 | 65 % |
| C | Encoder drift | 0.1646 | 3.64x | 12.54 | 4.22x | 0.1162 | **never** | 47 % |
| D | LiDAR dropout 60 % | 0.0352 | 0.78x | 2.60 | 0.88x | 0.0332 | not perturbed | 92 % |
| E | Sensor outage 10 s | 0.0578 | 1.28x | 3.20 | 1.08x | 0.0647 | 0.9 | 56 % |
| F | Multi-degradation | 0.0644 | 1.43x | 4.38 | 1.47x | 0.0544 | 1.4 | 70 % |

## odometry

| Case | Scenario | Position RMSE [m] | vs A | Heading RMSE [deg] | vs A | Degraded-phase pos. RMSE [m] | Recovery [s] | Inside 2-sigma |
|---|---|---|---|---|---|---|---|---|
| A | Nominal | 0.1000 | 1.00x | 7.76 | 1.00x | -- | n/a | 100 % |
| B | IMU noise x10 | 0.0994 | 0.99x | 7.75 | 1.00x | 0.0838 | **never** | 100 % |
| C | Encoder drift | 0.4351 | 4.35x | 35.43 | 4.56x | 0.3688 | **never** | 67 % |
| D | LiDAR dropout 60 % | 0.0976 | 0.98x | 7.60 | 0.98x | 0.0842 | **never** | 100 % |
| E | Sensor outage 10 s | 0.0977 | 0.98x | 7.60 | 0.98x | 0.0818 | **never** | 100 % |
| F | Multi-degradation | 0.1747 | 1.75x | 13.10 | 1.69x | 0.1562 | **never** | 100 % |

