# This message represents a metric report from a trajectory validator.

# Possible levels for a metric report.
uint8 SAFE = 0
uint8 LOW_CAUTION = 1
uint8 HIGH_CAUTION = 2
uint8 DANGER = 3

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
