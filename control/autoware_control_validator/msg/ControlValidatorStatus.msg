builtin_interfaces/Time stamp

# states
bool is_valid_max_distance_deviation
bool is_valid_acc
bool is_rolling_back
bool is_over_velocity
bool has_overrun_stop_point
bool will_overrun_stop_point
bool is_valid_latency

# values
float64 max_distance_deviation
float64 desired_acc
float64 measured_acc
float64 target_vel
float64 vehicle_vel
float64 dist_to_stop
float64 pred_dist_to_stop
float64 nearest_trajectory_vel
float64 latency

int64 invalid_count
