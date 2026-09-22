# Per-cycle introspection of the Extended Kalman Filter.
# Published on /ekf/diagnostics so that estimator health can be judged online
# and replayed offline, without ever touching ground truth.

std_msgs/Header header

# --- State estimate (mirrors the filter's internal state vector) ---
float64[] state                 # x_hat, length n
float64[] covariance_diagonal   # diag(P), length n -- per-state variance

# --- Scalar summaries of the covariance matrix ---
float64 position_std            # sqrt(P[0,0] + P[1,1]) [m]   1-sigma position spread
float64 heading_std             # sqrt(P[2,2])          [rad]
float64 covariance_trace        # trace(P) -- overall uncertainty scalar

# --- Innovation (measurement residual) of the most recent update ---
string   last_update_source     # "imu" | "wheel_odom" | "lidar_scan_match" | ""
float64[] innovation            # y = z - h(x_hat)
float64  normalized_innovation_squared  # NIS = y^T S^-1 y  (chi-square gated)
bool     update_accepted        # false if the NIS gate rejected the measurement

# --- Timing ---
float64 predict_duration_ms
float64 update_duration_ms
float64 measurement_age_ms      # arrival time minus measurement stamp
