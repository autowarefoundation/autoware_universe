# This message represents a metric report from a trajectory validator.

# The name of the validator that generated this report.
string validator_name
# The category of the validator that generated this report.
string validator_category
# The name of the metric that was reported.
string metric_name
# The value of the metric that was reported.
float64 metric_value
# The risk severity of the metric report.
RiskLevel risk
