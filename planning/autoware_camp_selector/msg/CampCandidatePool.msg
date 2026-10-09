# Original ordered candidates and their deployment atoms in one atomic packet.
# Host computes atoms from same-tick online inputs before trajectory postprocessing.
# No logged future or training teacher may be used to construct this packet.
uint8 OBSERVED=0
uint8 NOT_APPLICABLE=1
uint8 TYPED_MISSING=2
std_msgs/Header header
# Producer's monotonically increasing pool identity; do not reuse after reset.
uint64 pool_id
autoware_internal_planning_msgs/CandidateTrajectories candidates
uint8[16] atom_status
# Row-major K x 16 raw atoms; inactive columns may be NaN.
float64[] raw_atoms
