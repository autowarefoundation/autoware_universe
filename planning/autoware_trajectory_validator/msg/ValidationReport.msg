# This message represents a validation report for a trajectory.

# Possible levels of validation report severity.
uint8 SAFE = 0
uint8 LOW_CAUTION = 1
uint8 HIGH_CAUTION = 2
uint8 DANGER = 3

builtin_interfaces/Time trajectory_stamp
unique_identifier_msgs/UUID generator_id
string generator_name
uint8 level
MetricReport[] metrics
