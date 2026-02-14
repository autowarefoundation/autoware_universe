^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_diffusion_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
