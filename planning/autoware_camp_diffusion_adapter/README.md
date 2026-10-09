# CAMP Diffusion Planner adapter

`autoware_camp_diffusion_adapter` hosts the public upstream `DiffusionPlannerCore`
and connects it to the independent `autoware_camp_selector`. Diffusion Planner and
trajectory ranker remain unchanged. Another ML planner can provide its own adapter
to the selector's original-candidate pool interface.

## Runtime contract

Each planning tick creates the upstream input once, retains raw-unit tensors for
CAMP atoms, applies the upstream observation normalization and runs inference once.
The frozen integration uses K=8, temperatures `[0, 1, 1, 1, 1, 1, 1, 1]`, ten solver
steps and FP32. Snapping, x-shift and delay-step are disabled to preserve the original
candidate construction and alignment. Candidate0 is row0; the fixed scoring JSON,
atom formulas and endpoint states are unchanged.

The adapter publishes one ordered pool and keeps one bounded pending selection.
Before accepting feedback it checks the pool ID, full header, scores and complete
original candidate. Stale, duplicate or malformed feedback cannot advance the
publication ledger. Accepted trajectory, turn indication and actor output use the
same selected row. Continuity commits after publication and resets at episode,
route/map and backwards-clock boundaries; pool IDs are not reused. Traffic-signal
updates continue while a selection is pending. The upstream core's private history
and the CAMP publication ledger have separate commit points.

Only the adapter's checked outputs should feed the planning consumer. The launch
file remaps the selector's direct outputs to debug topics to avoid a second publisher.
No future labels, scene-selection override or command override is used.

## Build and run

Use a compatible ROS Humble / Autoware workspace with CUDA, TensorRT, the upstream
multi-step generator models, matching messages and maps provisioned separately.
Large generator checkpoints, TensorRT engines and bags are not package resources.

```sh
colcon build --packages-up-to autoware_camp_diffusion_adapter
source install/setup.bash
ros2 launch autoware_camp_diffusion_adapter camp_diffusion_adapter.launch.xml \
  model_directory:=/path/to/provisioned/diffusion_planner/models
```

The launch file provides arguments for model/plugin paths, vehicle parameters,
fixed-weight resource, simulation time and all input/output topic remappings. It
starts both the adapter and selector. Route, vector map, localization, tracked
objects and traffic signals must be supplied by the host graph. Connect the checked
trajectory through the normal planning validator and controller chain.

`scripts/verify_recorded_selection.py` compares recorded message fields against
the original selected rows. It is an offline identity check, not a deployable
oracle or proof of internal controller consumption.

## Recorded integration

The independent packages built with ROS Humble, CUDA 12.8 and TensorRT 10.9. A
same-frame normalization audit compared 15 tensors / 2,627,296 Float32 values with
the unchanged upstream normalizer and captured noise: zero bitwise differences.
Isolated CPU ONNX and GPU TensorRT graph checks each passed 42/42 comparisons;
these do not establish complete-sampler or outcome parity.

The final genuine Planning Simulator recording lasts 75 seconds, with native
MPC/PID control, a real RViz window and no compositing. All 259 recorded selections
match original candidate rows; 257 select a nonzero candidate. The validator outputs
retain all 259 original-row bindings. All 2,562 recorded Controls have finite fields;
their acceleration applicability flags are false, so stored acceleration numbers
do not establish commanded or physical braking. Validator flags and retained solver,
curvature and TF warnings are separate observations.

Two previously failing native-controller bags were replayed to their recorded end
with the isolated repair: 64 + 104 Trajectories and 603 + 828 finite Controls. Their
input-stream counts independently matched each original bag's metadata. The recording
and native replay build used the recorded Autoware 0.51 underlay. The controller
commit in this PR ports the repair to current upstream APIs while preserving temporal
trajectory handling; that current-head port has not been rebuilt in the recorded ROS
environment. The recording is evidence for its recorded build, rather than a claim
that every later source revision was run there.

This is a completed package and simulator integration demonstration. Safety,
road/rules, comfort, progress, ADE/FDE, feasibility and latency benchmarks were not
evaluated. Original recordings, raw observations and historical diagnostics remain
retained separately; no additional acceptance task is implied by this package.
