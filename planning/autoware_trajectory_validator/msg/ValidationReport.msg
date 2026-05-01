# This message represents a validation report for a trajectory.

# Possible levels of validation report severity.
uint8 OK = 0
uint8 WARN = 1
uint8 ERROR = 2

builtin_interfaces/Time trajectory_stamp
unique_identifier_msgs/UUID generator_id
string generator_name
uint8 level
MetricReport[] metrics
