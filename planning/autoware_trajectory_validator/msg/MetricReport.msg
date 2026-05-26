# This message represents a metric report from a trajectory validator.

# Possible levels for a metric report.
uint8 OK = 0
uint8 WARN = 1
uint8 ERROR = 2

# The name of the validator that generated this report.
string validator_name
# The category of the validator that generated this report.
string validator_category
# The name of the metric that was reported.
string metric_name
# The value of the metric that was reported.
float64 metric_value
# The level of the metric report.
uint8 level
