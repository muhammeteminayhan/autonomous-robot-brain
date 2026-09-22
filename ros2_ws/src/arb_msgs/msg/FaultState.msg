# Currently active sensor fault injection, published by arb_sensors so that
# every downstream log and plot can be annotated with the scenario that
# produced it. This is instrumentation only -- no estimator may consume it.

std_msgs/Header header

uint8 FAULT_NONE=0
uint8 FAULT_IMU_HIGH_NOISE=1
uint8 FAULT_ENCODER_DRIFT=2
uint8 FAULT_LIDAR_DROPOUT=3
uint8 FAULT_SENSOR_OUTAGE=4
uint8 FAULT_MULTI_DEGRADATION=5

string scenario_id              # e.g. "case_c_encoder_drift"
uint8[] active_faults           # one or more of the constants above

bool imu_degraded
bool encoder_degraded
bool lidar_degraded

float64 imu_noise_scale         # multiplier applied to nominal IMU sigma
float64 encoder_bias_rad_s      # additive bias on right wheel angular rate
float64 lidar_dropout_ratio     # fraction of beams discarded, [0, 1]
float64 elapsed_since_onset_s   # 0 while nominal
