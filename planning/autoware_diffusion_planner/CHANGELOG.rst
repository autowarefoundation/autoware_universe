^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_diffusion_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat: default artifact paths to ~/autoware_data/ml_models (`#12523 <https://github.com/mitsudome-r/autoware_universe/issues/12523>`_)
  feat(launches,configs): default artifact paths to ~/autoware_data/ml_models
  Roll every per-package `data_path` / `model_path` launch-arg default
  from `$(env HOME)/autoware_data[/...]` to
  `$(env HOME)/autoware_data/ml_models[/...]` so standalone universe
  launches resolve artifacts under the new `~/autoware_data/ml_models/`
  layout (`autowarefoundation/autoware#7068 <https://github.com/autowarefoundation/autoware/issues/7068>`_).
  When invoked through autoware_launch the parent overrides cascade and
  already pin the new root (`autowarefoundation/autoware_launch#1835 <https://github.com/autowarefoundation/autoware_launch/issues/1835>`_); this
  commit closes the gap for users who launch a perception / localization /
  sensing / planning component directly with `ros2 launch <pkg>`.
  22 launch files updated (one-line default change each):
  - e2e/autoware_tensorrt_vad/launch/vad_carla_tiny.launch.xml
  - localization/yabloc/yabloc_pose_initializer/launch/yabloc_pose_initializer.launch.xml
  - perception/autoware_bevfusion/launch/bevfusion.launch.xml
  - perception/autoware_camera_streampetr/launch/streampetr.launch.xml
  - perception/autoware_image_projection_based_fusion/launch/pointpainting_fusion.launch.xml
  - perception/autoware_lidar_apollo_instance_segmentation/launch/lidar_apollo_instance_segmentation.launch.xml
  - perception/autoware_lidar_centerpoint/launch/lidar_centerpoint.launch.xml
  - perception/autoware_lidar_frnet/launch/lidar_frnet.launch.xml
  - perception/autoware_lidar_transfusion/launch/lidar_transfusion.launch.xml
  - perception/autoware_ptv3/launch/ptv3.launch.xml
  - perception/autoware_shape_estimation/launch/shape_estimation.launch.xml
  - perception/autoware_simpl_prediction/launch/simpl.launch.xml
  - perception/autoware_tensorrt_bevdet/launch/tensorrt_bevdet.launch.xml
  - perception/autoware_tensorrt_bevformer/launch/bevformer.launch.xml
  - perception/autoware_tensorrt_yolox/launch/{yolox_traffic_light_detector,yolox_tiny,yolox_s_plus_opt}.launch.xml
  - perception/autoware_traffic_light_classifier/launch/{car,pedestrian}_traffic_light_classifier.launch.xml
  - perception/autoware_traffic_light_fine_detector/launch/traffic_light_fine_detector.launch.xml
  - planning/autoware_diffusion_planner/launch/diffusion_planner.launch.xml
  - sensing/autoware_calibration_status_classifier/launch/calibration_status_classifier.launch.xml
  Drive-by README and test fixes:
  - e2e/autoware_tensorrt_vad/{README.md,docs/design.md}: also migrate the
  `$HOME/autoware_map/Town01` examples to `$HOME/autoware_data/maps/Town01`.
  - localization/yabloc/{README.md,yabloc_pose_initializer/README.md}: also
  migrate `$HOME/autoware_map/sample-map-rosbag` to
  `$HOME/autoware_data/maps/demos/sample-map-rosbag`.
  - control/autoware_smart_mpc_trajectory_follower/README.md: migrate the
  `map_path:=$HOME/autoware_map/sample-map-planning` example to
  `$HOME/autoware_data/maps/demos/sample-map-planning`.
  - simulator/autoware_carla_interface/README.md: migrate every
  `$HOME/autoware_map/Town01/...` reference to
  `$HOME/autoware_data/maps/Town01/...`.
  - perception/{autoware_bevfusion,autoware_image_projection_based_fusion,autoware_lidar_centerpoint,autoware_tensorrt_bevformer}/README.md: copy-paste examples updated to `~/autoware_data/ml_models/<pkg>`.
  - perception/autoware_camera_streampetr/config/ml_package_camera_streampetr.param.yaml: header comment updated.
  - planning/autoware_diffusion_planner/README.md: prerequisites snippet updated.
  - sensing/autoware_calibration_status_classifier/test/{test_model_inference,test_calibration_status_classifier}.cpp: hardcoded fallback ONNX path updated.
  Users on the legacy layout can pin the old root with
  `data_path:=$HOME/autoware_data` (or the per-package equivalent) on the
  command line.
  Refs: https://github.com/autowarefoundation/autoware/issues/7068
* refactor(diffusion_planner,bevformer,vad): use launch arg paths (`#12521 <https://github.com/mitsudome-r/autoware_universe/issues/12521>`_)
  refactor(autoware_diffusion_planner,autoware_tensorrt_bevformer,autoware_tensorrt_vad): use launch substitutions for artifact paths
  Move the absolute $(env HOME)/autoware_data/<model> strings out of the
  param YAMLs and have each YAML reference a launch substitution. The
  launch arg defaults still resolve to the same paths, so behaviour is
  unchanged; downstream consumers (autoware_launch) can now flip a single
  data_path default to point at the upcoming ~/autoware_data/ml_models
  layout.
  - autoware_diffusion_planner: add a data_path arg in
  diffusion_planner.launch.xml defaulting to
  $(env HOME)/autoware_data/diffusion_planner; switch the param YAML to
  $(var data_path). Drop the unused artifact_dir from the schema (left
  over after the corresponding launch arg was removed) and align the
  schema's onnx_model_path and args_path defaults with the YAML.
  - autoware_tensorrt_bevformer: rewrite model_params.engine_file and
  model_params.onnx_file defaults to use $(var data_path); the launch
  already declares the arg.
  - autoware_tensorrt_vad: rewrite model_param_path to use
  $(var model_path); the launch already declares the arg.
  Also fixes a stale v3.1 reference in the diffusion_planner README to
  match the YAML default of v4.0.
  Refs: https://github.com/autowarefoundation/autoware/issues/7068
* docs(diffusion_planner): point artifact links at artifacts README (`#12506 <https://github.com/mitsudome-r/autoware_universe/issues/12506>`_)
  docs(diffusion_planner): point artifact links at artifacts role README
  setup-dev-env.sh is being removed on 2026-05-24 per
  `autowarefoundation/autoware#7052 <https://github.com/autowarefoundation/autoware/issues/7052>`_, and the autoware-documentation
  source-installation page is being updated alongside it. Both artifact
  download references in this README pointed at those soon-to-be-deleted
  entrypoints; redirect them to the canonical artifacts role README which
  documents the install_dev_env --tags artifacts invocation.
* feat(diffusion_planner): add planning factor interface (`#12387 <https://github.com/mitsudome-r/autoware_universe/issues/12387>`_)
  * add planning factor interface to diffusion planner
  * align default parameters
  * refactor & add tests
  * split functions
  * fix
  * fix test
  ---------
* feat(diffusion_planner): diffusion planner v4 (`#12348 <https://github.com/mitsudome-r/autoware_universe/issues/12348>`_)
  * Implemented turn_indicator_hold_duration
  * Fixed data_converter.cpp
  * Updated `WEIGHT_MAJOR_VERSION` to 3
  * Added ego_shape
  * Fixed to use `ego_wheel_base`
  * Fixed timestamp
  * Removed `to_string`
  * Fixed
  * Fixed
  * Fixed
  * Fixed
  * Fixed AgentData
  * Fixed AgentData to map
  * Fixed max_num_agent\_
  * Removed unused functions
  * Added `INPUT_T_WITH_CURRENT`
  * Removed num_agent
  * Removed unused functions
  * Fixed flatten_histories_to_vector
  * Fixed AgentData
  * Fixed AgentData
  * Removed pad_history
  * Removed unused functions
  * Fixed constructor
  * Fixed
  * Fixed
  * Removed latest_time\_
  * Fixed
  * Fixed fixed_queue
  * Removed autoware_label\_
  * Removed `yaw\_`
  * Removed unused functions
  * Removed `_`
  * Added const
  * Removed `tracked_object`
  * Fixed
  * Removed `get_latest_state_position`
  * Removed the default value
  * Fixed
  * Fixed
  * Fixed
  * Fixed tests
  * Fixed a bug
  * Fixed shift_x
  * Fixed shift length
  * Fixed the type of `ego_history\_` to `nav_msgs::msg::Odometry`
  * Added timestamp to AgentState
  * Removed FixedQueue
  * Removed a comment
  * Fixed time_from_start
  * v3.0
  * Fixed the description of `shift_x`
  * Fixed the default value of `turn_indicator_keep_offset`
  * Added comments to turn_indicator_manager.hpp
  * Fixed `keep_offset` as a parameter
  * Added `const`
  * Added `static_cast`
  * Added a comment
  * Fixed types
  * Removed redundant shift_x
  * Implemented
  * Fixed has_previous_output
  * Fixed
  * Fixed delay_step
  * Added normalization
  * Fixed
  * Fixed
  * Fixed copy steps
  * Reduced `print`
  * Fixed to use `pose_to_matrix4d`
  * Fixed the root
  * Fixed debug output
  * Fixed output PREDICTION
  * Fixed msg
  * Fixed initialization of ego
  * Fixed the standard point
  * Added unit tests
  * Fixed replicate_for_batch
  * Fixed to use `create_ego_current_state`
  * Added min_distance
  * Fixed route
  * Added LineStringType
  * Fixed to 60
  * Fixed delay_step
  * feat(autoware_diffusion_planner): linestring resampling and visualization (`#2714 <https://github.com/mitsudome-r/autoware_universe/issues/2714>`_)
  * Add roadborder publishing
  * add resampling of linestrings to get better details
  * Updated the publisher to publish both road borders and stop lines
  * use tensor layout notation to get linestring type
  * undo path changes in param / config file
  * copilot reviews
  ---------
  * Fixed the default value of `publish_debug_linestrings` to `true`
  * Fixed the version to 4.0
  * Removed tools
  * Removed `BUILD_TOOL`
  * Fixed the year in the copyright text
  * Potential fix for pull request finding
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
  * Fixed issues
  * Fixed minors
  * Added `#include <cmath>`
  * Changed the default value of `publish_debug_linestrings`
  * Updated README.md
  ---------
  Co-authored-by: danielsanchezaran <daniel.sanchez@tier4.jp>
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
* chore(planning): remove unused lanelet2_extension header (`#12294 <https://github.com/mitsudome-r/autoware_universe/issues/12294>`_)
  * unused lanelet2_extension in planning component
  * unused lanelet2_extension in planning component (2)
  * unused lanelet2_extension in planning component (3)
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(diffusion_planner): optimize TRT inference pipeline (`#12241 <https://github.com/mitsudome-r/autoware_universe/issues/12241>`_)
  * feat(diffusion_planner): optimize TRT inference pipeline
  - Move setInputShape/setTensorAddress calls from per-infer to one-time
  bindBuffers(), eliminating 34 redundant CUDA API calls per frame
  - Use pinned host memory (cudaHostAlloc) for D2H output transfers
  - Always set maxAuxStreams(0) to force single-stream execution, reducing
  scratch memory from 384MB to 27MB regardless of CUDA Graph setting
  - Add CUDA Graph support with graceful fallback for GPUs using TRT
  Compiler Backend (e.g. Blackwell SM 12.0) where stream capture fails
  - Switch default model to onnx-simplified variant for better TRT
  optimization
  - Add cuda_graph_enable ROS parameter (default: false)
  - Add standalone engine benchmark tool (scripts/benchmark_engine.cpp)
  and evaluation script for comparing legacy vs optimized inference
  Benchmark on RTX PRO 6000 Blackwell (300 runs):
  OLD engine (14759b0719): scratch=384MB, 5 aux streams, GPU=2672MiB
  NEW engine (this PR):    scratch=27MB,  single-stream, GPU=2298MiB
  Latency: 5.33ms -> 5.13ms (mean), GPU memory savings: 374 MiB
  Depends-on: tensorrt_common getContext() PR
  * docs(diffusion_planner): simplify benchmark README and fix model filenames
  * style(pre-commit): autofix
  * refactor(diffusion_planner): move benchmark scripts to Diffusion-Planner repo
  Benchmark tool moved to tier4/Diffusion-Planner cpp_tools/benchmark/.
  * refactor(diffusion_planner): remove CUDA graph optimization
  CUDA graph capture fails on both Blackwell (SM 12.0) and 40-series GPUs
  with current TensorRT, so remove all CUDA graph code from the PR.
  The setMaxAuxStreams(0) optimization is kept as it independently reduces
  GPU scratch memory (~384MB to ~27MB).
  * feat(diffusion_planner): update model path to v3.1
  Move from v3.0/diffusion_planner_simplified.onnx to v3.1/diffusion_planner.onnx.
  Add v3.1 entry to model version history table.
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* fix(diffusion_planner): remove `predict_neighbor_trajectory` (`#12237 <https://github.com/mitsudome-r/autoware_universe/issues/12237>`_)
  Removed `predict_neighbor_trajectory`
* feat(diffusion_planner): ego interpolation (`#12213 <https://github.com/mitsudome-r/autoware_universe/issues/12213>`_)
  * Implemented interpolation
  * Added `use_time_interpolation`
  * Applied `pre-commit run -a`
  * Added a test
  * Replaced `autoware::universe_utils::calcInterpolatedPose` to `autoware_utils_geometry::calc_interpolated_pose`
  ---------
* fix(diffusion_planner): add `turn_indicator_batch_idx` (`#12175 <https://github.com/mitsudome-r/autoware_universe/issues/12175>`_)
  Fixed `turn_indicator_batch_idx`
* fix(diffusion_planner): add `VehicleSpec` (`#12174 <https://github.com/mitsudome-r/autoware_universe/issues/12174>`_)
  Added `VehicleSpec`
* fix(diffusion_planner): removed `result.predictions.clear();` (`#12171 <https://github.com/mitsudome-r/autoware_universe/issues/12171>`_)
  Removed `result.predictions.clear();`
* refactor(diffusion_planner): move postprocessing logic from node to core (`#12162 <https://github.com/mitsudome-r/autoware_universe/issues/12162>`_)
  * Moved postprocessing logit from node to core
  * Removed `get_previous_turn_indicator_report`
  * Fixed a comment
  * Removed unnecessary headers
  * Added a TODO comment
  * Added `std::string()`
  * Wrapped `create_planner_output` with try/catch
  ---------
* refactor(diffusion_planner): separate core logic (`#12151 <https://github.com/mitsudome-r/autoware_universe/issues/12151>`_)
  * Separated the core logic
  * Removed unnecessary includes and using
  * Fixed build
  * Fixed an error message
  * Fixed route_ptr
  * Fixed route_ptr\_
  * Fixed route_ptr\_
  * Fixed
  * Fixed load_model and header
  * Fixed to 2026
  * Fixed comments in planning/autoware_diffusion_planner/include/autoware/diffusion_planner/diffusion_planner_node.hpp
  ---------
* feat: enable traffic light debugging outputs for signal display (`#12135 <https://github.com/mitsudome-r/autoware_universe/issues/12135>`_)
  * feat: enable traffic light debugging outputs for signal display
  * fix: pre-commit
  * feat: align const argument; fix not needed clear
  * feat: add documentation for interface changes
  * feat: add test for basic function
  * feat: fix traffic light element error
  ---------
* fix(diffusion_planner): modify loading model (`#12125 <https://github.com/mitsudome-r/autoware_universe/issues/12125>`_)
  * Added `load_model`
  * Fixed to fail
  * Fixed
  * Removed `reject\_` from the result variable
  * Update planning/autoware_diffusion_planner/include/autoware/diffusion_planner/diffusion_planner_node.hpp
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  * Fixed the timing of checking `build_only`
  ---------
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
* Contributors: Kotakku, Max-Bin, Mete Fatih Cırıt, SakodaShintaro, Sarun MUKDAPITAK, Yuxuan Liu, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* docs(diffusion_planner): fix README.md (`#12067 <https://github.com/autowarefoundation/autoware_universe/issues/12067>`_)
  * Fixed README.md
  * Apply suggestion from @Copilot
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  * Apply suggestion from @Copilot
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  * Deleted the change of `launch_traffic_light_module`
  ---------
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
* fix(diffusion_planner): fix the stopping logic (`#12041 <https://github.com/autowarefoundation/autoware_universe/issues/12041>`_)
  Fixed the stopping logic
* refactor(diffusion_planner): change `ego_current_state` class to a function (`#12022 <https://github.com/autowarefoundation/autoware_universe/issues/12022>`_)
  * Refactored `ego_current_state`
  * Added `CreateEgoCurrentState` test
  * Added `const`
  * Sorted include files
  * Removed `std::abs` from `linear_vel`
  ---------
* refactor(diffusion_planner): refactor `replicate_for_batch` (`#12016 <https://github.com/autowarefoundation/autoware_universe/issues/12016>`_)
  * Fixed break lines
  * Moved `replicate_for_batch` from diffusion_planner_node.hpp into utils.hpp
  * Fixed comments
  * Added `const`
  * Applied the formatter
  ---------
* refactor(diffusion_planner): refactor `AgentState` (`#11946 <https://github.com/autowarefoundation/autoware_universe/issues/11946>`_)
  * Refactored `AgentState`
  * Fixed to use rotation_matrix_to_cos_sin
  ---------
* refactor(diffusion_planner): refactor the transformation to map coordinates (`#11931 <https://github.com/autowarefoundation/autoware_universe/issues/11931>`_)
  * Refactored transform to map
  * Fixed
  * Added 0.0
  ---------
* chore(autoware_diffusion_planner): remove cudnn dependency (`#11901 <https://github.com/autowarefoundation/autoware_universe/issues/11901>`_)
* feat(diffusion_planner): support reconfiguration `onnx_model_path` (`#11870 <https://github.com/autowarefoundation/autoware_universe/issues/11870>`_)
  * Added tensorrt_inference.hpp/cpp
  * Added `#include <unordered_map>`
  * Added `num_elements`
  * Fixed converting float to bool
  ---------
* refactor(diffusion_planner): refactoring of map processing (`#11869 <https://github.com/autowarefoundation/autoware_universe/issues/11869>`_)
  * Removed `traffic_rules_ptr\_`
  * Removed `routing_graph_ptr\_`
  * Removed `is_map_loaded\_`
  * Removed `remove_const`
  * Applied `pre-commit run -a`
  ---------
* feat(diffusion_planner): diffusion_planner v3 (`#11849 <https://github.com/autowarefoundation/autoware_universe/issues/11849>`_)
  * v3.0
  * Fixed the description of `shift_x`
  * Fixed the default value of `turn_indicator_keep_offset`
  * Added comments to turn_indicator_manager.hpp
  * Fixed `keep_offset` as a parameter
  * Added `const`
  * Added `static_cast`
  * Added a comment
  * Fixed types
  ---------
* Contributors: Amadeusz Szymko, Ryohsuke Mitsudome, SakodaShintaro

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* fix(diffusion_planner): traffic_light_id_map (`#11812 <https://github.com/autowarefoundation/autoware_universe/issues/11812>`_)
  Revert https://github.com/autowarefoundation/autoware_universe/pull/11805/changes/b6058db6f9ba83df47e5c5015a1aa8ee5ba058c8
* fix(diffusion_planner): calculate the distance with the z position (`#11808 <https://github.com/autowarefoundation/autoware_universe/issues/11808>`_)
  Fixed to calculate the distance with the z position
* feat(autoware_lanelet2_utils): replace from/toBinMsg (Planning and Control Component) (`#11784 <https://github.com/autowarefoundation/autoware_universe/issues/11784>`_)
  * planning component toBinMsg replacement
  * control component fromBinMsg replacement
  * planning component fromBinMsg replacement
  ---------
* refactor(diffusion_planner): add `FrameContext` (`#11805 <https://github.com/autowarefoundation/autoware_universe/issues/11805>`_)
  * Refactored diffusion_planner
  * Added a comment
  * Added a comment
  * Replaced `autoware_utils` into `autoware_utils_debug`
  * Moved `traffic_light_id_map` into FrameContext
  * Fixed `do_inference_trt`
  ---------
* fix(diffusion_planner): route input (`#11780 <https://github.com/autowarefoundation/autoware_universe/issues/11780>`_)
  * Fixed route input
  * Renamed `start_inside` to `has_entered_valid_region`
  ---------
* fix(diffusion_planner): fix `build_only` (`#11770 <https://github.com/autowarefoundation/autoware_universe/issues/11770>`_)
  Fixed `build_only`
* chore(diffusion_planner): change default values (`#11757 <https://github.com/autowarefoundation/autoware_universe/issues/11757>`_)
  * Changed the default value of `temperature` from 0.5 to 0.0
  * Changed the default value of `stopping_threshold` from 0.0 to 0.3
  ---------
* fix(diffusion_planner): fix the first value of the output velocity (`#11692 <https://github.com/autowarefoundation/autoware_universe/issues/11692>`_)
  Fixed the first value of the output velocity
* feat(diffusion_planner): diffusion_planner v2 (`#11690 <https://github.com/autowarefoundation/autoware_universe/issues/11690>`_)
  * Fixed for v2
  * Applied cpplint
  * Fixed to v2.0
  * Fixed as cpplint
  * Fixed the model path
  * Fixed ConvertLaneletManyInterpolationPoints
  * Fixed ConvertToLaneSegments
  * Updated a comment
  * Updated README.md
  * Removed trailing spaces
  * Added specific notations
  * Fixed the position of `ego_history\_` and `turn_indicators_history\_`
  * Added checking `centerline.size() < 2`
  * Added a comment
  * Applied `pre-commit run -a`
  * Added turn_indicators into the input/output table of README.md
  * Applied `pre-commit run -a`
  ---------
* docs(diffusion_planner): add `How to use` (`#11685 <https://github.com/autowarefoundation/autoware_universe/issues/11685>`_)
  * Added `How to use`
  * Added Note
  * Applied pre-commit
  ---------
* fix(diffusion_planner): fix `process_traffic_signals` (`#11662 <https://github.com/autowarefoundation/autoware_universe/issues/11662>`_)
  * Fixed process_traffic_signals
  * Fixed buffer size
  * Added `#include <vector>`
  * Added `#include <vector>`
  ---------
* Contributors: Ryohsuke Mitsudome, SakodaShintaro, Sarun MUKDAPITAK

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(diffusion_planner): fix poses in postprocess (`#11618 <https://github.com/autowarefoundation/autoware_universe/issues/11618>`_)
  * Refactored tensor_data
  * Fixed to parse_predictions
  * Added an error handling
  * Removed an unused `&`
  * Added `reserve`
  * Fixed typecasting from `int` to `uint32_t`
  ---------
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* fix(diffusion_planner): fix `is_segment_inside` (`#11602 <https://github.com/autowarefoundation/autoware_universe/issues/11602>`_)
  Fixed is_segment_inside
* feat(diffusion_planner): add `stopping_threshold` (`#11541 <https://github.com/autowarefoundation/autoware_universe/issues/11541>`_)
  * Added stopping_threshold
  * Applied clang-format
  * Fixed stopping logic
  * Added enable_force_stop
  * Fixed create_ego_trajectory
  * Fixed ego_kinematic_state\_
  * Added a comment
  * Fixed the condition
  ---------
* fix(diffusion_planner): change the default parameters (`#11520 <https://github.com/autowarefoundation/autoware_universe/issues/11520>`_)
  Changed the default parameters
* refactor(diffusion_planner): remove `use_route_handler` (`#11518 <https://github.com/autowarefoundation/autoware_universe/issues/11518>`_)
  * Removed use_route_handler
  * Applied clang-format
  ---------
* fix(diffusion_planner): wrong agent index (`#11454 <https://github.com/autowarefoundation/autoware_universe/issues/11454>`_)
  * fix(diffusion_planner): wrong agent index
  * fix(diffusion_planner): correct method call for neighbor agent data retrieval
  ---------
* fix(diffusion_planner): fix remaining velocity (`#11443 <https://github.com/autowarefoundation/autoware_universe/issues/11443>`_)
  Fixed remaining velocity
* refactor(diffusion_planner): fix ego_history (`#11395 <https://github.com/autowarefoundation/autoware_universe/issues/11395>`_)
  * Fixed ego_history
  * Added error handling
  * Fixed tests
  * Fixed to use Eigen::Isometry3d(mat).inverse()
  * Fixed the function comment
  * Add detailed comments
  ---------
* feat(diffusion_planner): add acceleration (`#11392 <https://github.com/autowarefoundation/autoware_universe/issues/11392>`_)
  * Fixed const dimensions
  * Refactored create_neighbor_trajectories
  * Fixed create_ego_trajectory
  * Removed `to_candidate_trajectories_msg`
  * Removed get_prediction_matrix
  * Removed create_neighbor_trajectories
  * Fixed create_predicted_objects
  * Added velocity_smoothing_window
  ---------
* refactor(diffusion_planner): move postprocessing internal functions to cpp (`#11390 <https://github.com/autowarefoundation/autoware_universe/issues/11390>`_)
  * Moved postprocessing internal functions
  * Removed tests related to internal functions
  ---------
* refactor(diffusion_planner): refactor lane_segment (`#11378 <https://github.com/autowarefoundation/autoware_universe/issues/11378>`_)
  * Refactored
  * Fixed using
  * Added `epsilon`
  * Added const
  * Changed `MOVING_VELOCITY_THRESHOLD_MPS` back to a float
  * Added `reserve`
  * Removed the unused constants `VisualizationParams`
  * Applied the formatter clang-format-21.1.1 (not 17.0.5)
  ---------
* fix(diffusion_planner): fixed `const` to `constexpr` (`#11367 <https://github.com/autowarefoundation/autoware_universe/issues/11367>`_)
  Fixed `const` to `constexpr`
* fix(diffusion_planner): add version `v1.0` to the model path (`#11366 <https://github.com/autowarefoundation/autoware_universe/issues/11366>`_)
  Fix
* feat(diffusion_planner): add `sampled_trajectories` to the model input and add a new parameter `temperature` (`#11348 <https://github.com/autowarefoundation/autoware_universe/issues/11348>`_)
  * Implemented sampled_trajectories
  * FIxed skipping
  * Added temperature to update_param
  * Fixed to std::vector<double>
  ---------
* feat(diffusion-planner): update ONNX model versioning details and limitations (`#11352 <https://github.com/autowarefoundation/autoware_universe/issues/11352>`_)
  * feat(readme): update ONNX model versioning details and limitations
  * fix: update release date
  * cosmetic change for pre-commit
  * fix readme
  ---------
* feat(diffusion_planner): use line type (`#11339 <https://github.com/autowarefoundation/autoware_universe/issues/11339>`_)
  * Added LineType
  * Fixed
  * Added lane_segments\_
  * Buildable but can't work
  * Fixed to use lane_segments\_
  * Fixed dimensions.hpp
  * Fixed config
  * Fixed
  * Removed add_line_type_encoding_to_segment
  * Fixed the config
  * Fixed `onehot` to `one_hot`
  * Added includes
  * Added error handling
  ---------
* feat(diffusion_planner): add weight version validation (`#11351 <https://github.com/autowarefoundation/autoware_universe/issues/11351>`_)
  * Added checking the version of weight
  * Added WEIGHT_MAJOR_VERSION to constants.hpp
  * Added calling
  * Removed an unnecessary loop
  ---------
* refactor(diffusion_planner): add const to the arg of `AgentHistory::update` (`#11338 <https://github.com/autowarefoundation/autoware_universe/issues/11338>`_)
  Added const
* refactor(diffusion_planner): parse traffic_light_id in initialization (`#11333 <https://github.com/autowarefoundation/autoware_universe/issues/11333>`_)
  * Fixed to use lane_segments\_
  * Fixed traffic_light_id
  * Fixed a comment
  * Added const
  ---------
* refactor(diffusion_planner): fix to use `lane_segments\_` (`#11330 <https://github.com/autowarefoundation/autoware_universe/issues/11330>`_)
  * Fixed to use lane_segments\_
  * Added `seg_idx`
  ---------
* refactor(diffusion_planner): lanelet.hpp (`#11322 <https://github.com/autowarefoundation/autoware_universe/issues/11322>`_)
  * Refactored lanelet.hpp
  * Fixed the format
  * Added a break
  * Added const
  ---------
* fix(diffusion_planner): fix mask_range and traffic_light issues (`#11319 <https://github.com/autowarefoundation/autoware_universe/issues/11319>`_)
  * Fixed compute_distances
  * Fixed the condition for turn_direction==LaneSegment::TURN_DIRECTION_NONE
  ---------
* fix(diffusion_planner): change float to double (`#11310 <https://github.com/autowarefoundation/autoware_universe/issues/11310>`_)
  * Changed float to double
  * Fixed conversion
  * Applied pre-commit
  ---------
* feat(diffusion_planner): use `turn_direction` for arrow traffic signals (`#11303 <https://github.com/autowarefoundation/autoware_universe/issues/11303>`_)
  * Added `turn_direction`
  * Fixed clang-format
  * Added `#include <string>`
  * Fixed using
  * Added constants
  * Removed comments
  * Fixed to use std::map
  * Replace from`Eigen::Matrix` to `Eigen::Vector`
  * Added `#include <map>`
  ---------
* fix(diffusion_planner): modify inside (`#11299 <https://github.com/autowarefoundation/autoware_universe/issues/11299>`_)
  * Modified inside
  * Refactored distance_squared
  * Refactored map_lane_segments_matrix\_ accessing
  ---------
* feat(diffusion_planner): add diagnostics (`#11284 <https://github.com/autowarefoundation/autoware_universe/issues/11284>`_)
  * Added diagnostics
  * Added batch_idx
  * Added #include <limits>
  ---------
* feat(diffusion_planner): add `turn_indicators_command` (`#11264 <https://github.com/autowarefoundation/autoware_universe/issues/11264>`_)
  * Added pub_turn_indicators\_
  * Added output_turn_indicators
  * Fixed
  * Replaced "logits" to "logit"
  * Fixed initialization of sum
  * Use TURN_INDICATOR_LOGIT_SHAPE
  * Fixed the type of `i`
  * Use `turn_indicator_logit.size()`
  * Fixed launch.xml
  ---------
* refactor(diffusion_planner): move `create_ego_agent_past` to preprocessing_utils (`#11260 <https://github.com/autowarefoundation/autoware_universe/issues/11260>`_)
  * Fixed create_ego_agent_past
  * Removed skipping messages
  * Fix pre-commit
  * Fixed end_idx
  ---------
* fix(diffusion_planner): change the order of width and length (`#11246 <https://github.com/autowarefoundation/autoware_universe/issues/11246>`_)
  Fixed agent.cpp
* feat(diffusion_planner): multibatch inference (`#11197 <https://github.com/autowarefoundation/autoware_universe/issues/11197>`_)
  * Fixed
  * Fixed sort
  * Fixed extract_lane\_*
  * Fixed to run
  * Implemented multi trajectories
  * Fixed
  * Fixed engine name
  * Fixed a comment
  * Fixed parsing
  * Fixed name
  * Fix
  ---------
* fix(autoware_diffusion_planner): remove unused function (`#11184 <https://github.com/autowarefoundation/autoware_universe/issues/11184>`_)
* refactor(diffusion_planner): fix `transform_and_select_rows` (`#11168 <https://github.com/autowarefoundation/autoware_universe/issues/11168>`_)
  * Fixed
  * Fixed sort
  * Fixed extract_lane\_*
  * Added anonymous namespace
  * Removed the second return value of `transform_points_and_add_traffic_info`
  * Fixed a comment `columns` -> `rows`
  ---------
* refactor(diffusion_planner): fix `LaneSegmentContext` constructor (`#11163 <https://github.com/autowarefoundation/autoware_universe/issues/11163>`_)
  * Refactored the constructor of LaneSegmentContext
  * Fixed tests
  * Improved LaneSegmentContextFunctionality
  * Improved test
  * Fixed comments
  * Fixed cpplint issues
  ---------
* refactor(diffusion_planner): add `LaneSegmentContext` (`#11143 <https://github.com/autowarefoundation/autoware_universe/issues/11143>`_)
  * Added LaneSegmentContext
  * Use `LaneSegmentContext`
  * Removed lanelet_converter_ptr\_
  * Fixed lane_segments
  * Fixed to private
  * Removed old get_route_segments
  * Removed transform_and_select_rows
  * Removed `add_traffic_light_one_hot_encoding_to_segment` and `transform_points_and_add_traffic_info`
  * Removed `apply_transforms`
  * Fixed
  * Removed compute_distances
  * Removed getters
  * Fixed variables
  ---------
* fix(diffusion_planner): fix clang-tidy issues (`#11152 <https://github.com/autowarefoundation/autoware_universe/issues/11152>`_)
  * Fixed clang-tidy issues
  * Fixed the comment
  * Removed `LaneletConverterParams`
  ---------
* Contributors: Ryohsuke Mitsudome, Ryuta Kambe, SakodaShintaro, Tim Clephas, Yukihiro Saito

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* refactor(diffusion_planner): remove unused code (`#11137 <https://github.com/autowarefoundation/autoware_universe/issues/11137>`_)
  Removed unused code
* feat: update diffusion planner inputs/outputs (`#11093 <https://github.com/autowarefoundation/autoware_universe/issues/11093>`_)
  * Updated
  * Fixed ego_shape
  * Removed unnecessary returns
  * Fixed for cpplint
  * Applied the formatter
  * Removed test for traffic light state
  * Fixed lane_segments_test
  * Moved AddTrafficLightOneHotEncodingToSegmentNoTrafficLight from `lane_segments_test.cpp` to `lanelet_integration_test.cpp`
  * Added `#include <map>`
  * Added EGO_AGENT_PAST_IDX_X
  * Fix
  * Fix
  * Fixed remap params
  * Fixed nits
  ---------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* docs(autoware_diffusion_planner): remove obsolete information (`#11061 <https://github.com/autowarefoundation/autoware_universe/issues/11061>`_)
  * docs(autoware_diffusion_planner): remove obsolete information
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(diffusion_planner): modify unread parameters (`#11025 <https://github.com/autowarefoundation/autoware_universe/issues/11025>`_)
  * fix(diffusion_planner): modify unread parameters
  * fix(diffusion_planner): remove unused artifact_dir argument
  ---------
* feat(autoware_diffusion_planner): add diffusion-based trajectory planner (`#10957 <https://github.com/autowarefoundation/autoware_universe/issues/10957>`_)
  * feat(autoware_diffusion_planner): add diffusion-based trajectory planner
  * fix: dead links in README.md
  * fix: fix by pre-commit
  * fix: modify spell for cspell
  * refactor: reorganize CMakeLists.txt for better structure and clarity
  * fix: modify for pre-commit ci
  * fix: update for cppcheck
  * fix: update for pre-commit
  * cosmetic change
  * rename test dir
  * fix: modify for pre-commit
  * change output topic name
  * add maintainer
  * remove unnecessary section in readme
  * fixed no install in cmake
  * fix wrong syntax in launch
  * refactor: consolidate geometry conversion functions into a template
  * fix: remove redundant return statement and improve string formatting in to_string methods
  * cosmetic change
  * fix: remove example configuration section from README
  * fix: remove outdated link to Autoware Universe from README
  * fix: remove unused parameters from launch files and restore default build_only value
  * fix: update input and output sections in README for clarity and consistency
  * fix: update diffusion planner parameters and remove unused launch file
  * fix: add JSON schema for diffusion planner parameters and update README
  * fix: update JSON schema path for diffusion planner parameters in README
  ---------
* Contributors: Mete Fatih Cırıt, SakodaShintaro, Shintaro Tomie, Yukihiro Saito
