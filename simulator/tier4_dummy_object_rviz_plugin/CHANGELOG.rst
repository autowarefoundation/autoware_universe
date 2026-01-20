^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tier4_dummy_object_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_dummy_perception_publisher): dummy object predicted path functionality (`#11011 <https://github.com/autowarefoundation/autoware_universe/issues/11011>`_)
  * feat: implement euclidean distance-based mapping for dummy perception publisher
  - Add euclidean distance-based mapping between dummy objects and predicted objects
  - Implement 2-second grace period before using predicted paths
  - Add distance-based interpolation along predicted paths using dummy object speed
  - Use predictions at least 1 second old for better movement calculation
  - Maintain one-to-one mapping between dummy and predicted objects
  - Add predicted objects subscription and buffer management
  - Preserve dummy object velocity throughout movement
  * feat: enhance euclidean distance-based mapping with prediction stability
  - Implement prediction keeping logic for 1-second intervals to reduce jitter
  - Add prediction storage maps for stable object tracking
  - Remove debug output and clean up prediction selection logic
  - Use object creation time instead of mapping time for 2-second grace period
  - Improve handling of predicted object ID changes with proper remapping
  - Clean up unused mapping time parameter from ObjectInfo constructor
  * feat: increase prediction stability duration to 3 seconds
  - Change MIN_KEEP_DURATION from 1.0 to 3.0 seconds for more stable object tracking
  - Add debug output to track prediction usage
  - Fix code formatting and whitespace
  * feat: implement trajectory extrapolation and lost object handling
  - Add trajectory extrapolation when objects overshoot predicted paths
  - Use last two trajectory points to determine direction for extrapolation
  - Maintain last known orientation during extrapolation
  - Keep using last predicted trajectory for lost objects
  - Add proper fallbacks for edge cases (single point paths, zero-length segments)
  - Clean up debug output to avoid duplication
  * feat: add trajectory validation to prevent unrealistic direction changes
  - Add isTrajectoryValid function to validate new predictions against current ones
  - Implement thresholds for yaw change (45°), direction change (60°), and speed change (200%)
  - Skip trajectory updates when validation fails to maintain smooth object movement
  - Handle edge cases including empty paths and low-speed scenarios
  - Add detailed logging for debugging trajectory rejections
  * feat: relax trajectory validation thresholds for more realistic movement
  - Increase yaw change threshold from 45° to 90°
  - Increase direction change threshold from 60° to 120°
  - Increase speed change threshold from 200% to 500%
  - Allow for more realistic vehicle maneuvers while still preventing physics violations
  * feat: add path length validation to trajectory comparison
  - Add MAX_PATH_LENGTH_CHANGE_RATIO threshold (300% change)
  - Calculate path lengths for both current and new trajectories
  - Compare path length ratios to prevent unrealistic path changes
  - Skip validation for very short paths (less than 0.1m)
  - Add detailed logging for path length validation failures
  * feat: add comprehensive validation for object remapping candidates
  - Add isValidRemappingCandidate function to validate remapping candidates
  - Implement position, trajectory, and heading similarity checks
  - Add arePathsSimilar function to compare path shapes and directions
  - Use validation in both remapping and unmapped object scenarios
  - Add detailed logging for debugging validation failures
  - Set appropriate thresholds for distance, yaw, and path comparison
  * fix: remove unused variables and parameters to resolve build warnings
  - Mark unused parameter current_time in isValidRemappingCandidate
  - Mark unused parameter dummy_uuid_str in arePathsSimilar
  - Remove unused MAX_DIRECTION_CHANGE and MAX_VELOCITY_CHANGE_RATIO constants
  - Clean up warnings to improve code quality
  * refactor: remove unused parameters from function signatures
  - Remove unused current_time parameter from isValidRemappingCandidate
  - Remove unused dummy_uuid_str parameter from arePathsSimilar
  - Update function calls to match simplified signatures
  - Clean up function interfaces to remove unnecessary parameters
  * feat: improve remapping candidate validation using trajectory-based position calculation
  - Add calculateExpectedPosition function to predict object position based on trajectory
  - Use elapsed time and object speed to calculate expected position along predicted path
  - Update isValidRemappingCandidate to use trajectory-based position validation
  - Add detailed logging with coordinate information for debugging
  - Improve position comparison accuracy for remapping decisions
  * refactor: remove average path distance check from arePathsSimilar
  - Remove MAX_AVERAGE_PATH_DISTANCE constant and related validation logic
  - Simplify path similarity comparison by removing average distance calculations
  - Keep endpoint distance and overall direction validation
  - Reduce complexity of path similarity checking
  * fix: improve path similarity validation by comparing current vs expected positions
  - Replace complex path overlap logic with expected position comparison
  - Use calculateExpectedPosition to determine where object should be now
  - Compare candidate's current position with expected position based on last trajectory
  - Add detailed logging with coordinate information for debugging
  - Maintain overall direction validation for trajectory consistency
  * feat: add path length comparison to trajectory similarity validation
  - Add calculatePathLength lambda function to compute total distance along path
  - Compare path lengths with maximum ratio of 200% difference allowed
  - Skip length comparison for very short paths (less than 0.1m)
  - Add detailed logging for path length validation failures
  - Enhance trajectory similarity validation with length consistency checks
  * feat: add speed comparison validation for object remapping
  - Add MAX_SPEED_DIFFERENCE_RATIO constant (10% tolerance)
  - Compare dummy object speed with candidate predicted object speed
  - Get dummy object speed from objects\_ vector and candidate speed from twist
  - Skip speed comparison for very slow objects (less than 0.1 m/s)
  - Reduce path length ratio tolerance from 200% to 10% for stricter validation
  - Add detailed logging for speed validation failures
  * refactor: improve dummy perception publisher validation and logging
  - Extract position calculation logic into static methods calculateStraightLinePosition and calculateTrajectoryBasedPosition
  - Replace std::cerr with proper ROS logging (RCLCPP_DEBUG) throughout
  - Tighten validation thresholds: distance 5.0→2.0m, yaw 45°→15°, speed 10%→5% tolerance
  - Add stricter speed validation with 50%-150% range checks for moving objects
  - Require minimum 5 path points for predicted trajectories
  - Reduce direction difference tolerance from 60° to 30°
  - Improve position difference tolerance from 3.0m to 1.5m
  * fix: use current position for dummy object mapping instead of initial position
  - Use last known position from dummy_last_known_positions\_ when available
  - Calculate current position using straight-line movement model as fallback
  - Replace usage of initial_state.pose_covariance.pose.position with calculated current position
  - Improve mapping accuracy by using actual object positions rather than initial positions
  * comment
  * feat: add random path selection for pedestrians in dummy perception publisher
  - Add random number generation for path selection (mt19937 + uniform_real_distribution)
  - Implement pedestrian-specific path selection logic with random path choice
  - Add class-specific validation thresholds (pedestrians vs vehicles)
  - Make validation more lenient for pedestrians due to unpredictable movement patterns
  - Enhance trajectory-based position calculation to handle different object types
  - Add path reordering to put randomly selected pedestrian paths first
  - Improve logging with object type information for better debugging
  * feat: add ROS2 parameters for configuration
  - Add comprehensive ROS2 parameter declarations for all configuration options
  - Create separate random generator for pedestrian path selection with configurable seed
  - Replace hardcoded constants with configurable parameters
  - Add parameter groups for general, vehicle, and pedestrian-specific settings
  - Enable runtime configuration of validation thresholds and behavior
  - Make predicted_path_delay, min_keep_duration, and other key values configurable
  * refactor: replace hardcoded constants with ROS2 parameters
  - Replace all hardcoded validation constants with configurable parameter variables
  - Use max_yaw_change\_ instead of hardcoded MAX_YAW_CHANGE
  - Use max_path_length_change_ratio\_ instead of hardcoded MAX_PATH_LENGTH_CHANGE_RATIO
  - Apply class-specific parameter values dynamically based on object type
  - Enable runtime configuration of validation thresholds without recompilation
  * refactor: begin extracting helper methods from updateDummyToPredictedMapping
  - Extract collectAvailablePredictedUUIDs to gather available predicted objects
  - Extract findDisappearedPredictedObjects to identify objects needing remapping
  - Extract collectDummyObjectPositions to gather dummy object positions
  - Begin breaking down large updateDummyToPredictedMapping function into smaller, focused methods
  - Improve code organization and readability by separating concerns
  * refactor: complete refactoring of updateDummyToPredictedMapping
  - Extract findBestPredictedObjectMatch to find the closest valid predicted object
  - Extract createRemappingsForDisappearedObjects to handle remapping logic
  - Simplify main updateDummyToPredictedMapping function by using extracted helper methods
  - Improve code modularity and testability by breaking down complex logic
  - Make the mapping process more maintainable with focused, single-purpose functions
  * feat: update launch file to use parameter config file
  - Add parameter configuration file loading to launch file
  - Enable external configuration of dummy perception publisher parameters
  - Allow runtime customization of validation thresholds and behavior
  - Improve deployment flexibility with external parameter files
  * revert: remove ROS2 parameter configuration and config file
  - Revert back to hardcoded constants in validation functions
  - Remove ROS2 parameter declarations and configuration
  - Simplify random generator setup by removing separate pedestrian generator
  - Remove parameter configuration file and launch file integration
  - Return to simpler configuration approach
  * feat: add ROS2 parameters with JSON schema validation
  - Create JSON schema file defining all parameters with types and constraints
  - Add config file with all parameters following the schema structure
  - Declare all parameters in node constructor with proper namespacing
  - Replace all hardcoded constants with configurable parameters
  - Update launch files to load config file
  - Install schema and config folders in CMakeLists.txt
  This provides runtime configurability while maintaining validation
  through the JSON schema, similar to other Autoware packages.
  * Update dummy perception publisher to use predictions only for PREDICT action
  - Only use predicted trajectories when dummy object action is PREDICT
  - Fall back to straight-line motion for all other actions (ADD, MODIFY, DELETE)
  - Add PREDICT case to objectCallback method to handle predicted objects
  * Add PREDICT action support to RViz dummy object plugin
  - Add keyboard shortcut 'P' to enable prediction mode for selected objects
  - Add getTargetObject() and getTargetUuid() methods to InteractiveObjectCollection
  - Update tool description to explain new prediction mode functionality
  - Objects with PREDICT action will use predicted trajectories instead of straight-line motion
  * Add 'Predicted' checkbox property to all RViz dummy object tools
  - Add predicted_property\_ to header file for all object tools
  - Initialize predicted checkbox in car_pose.cpp, pedestrian_pose.cpp, and unknown_pose.cpp
  - Modify onPoseSet to use PREDICT action when predicted checkbox is enabled
  - Update tool description to mention prediction mode functionality
  - Objects created with predicted checkbox enabled will use trajectory predictions instead of straight-line motion
  * feat: implement euclidean distance-based mapping for dummy perception publisher
  - Add euclidean distance-based mapping between dummy objects and predicted objects
  - Implement 2-second grace period before using predicted paths
  - Add distance-based interpolation along predicted paths using dummy object speed
  - Use predictions at least 1 second old for better movement calculation
  - Maintain one-to-one mapping between dummy and predicted objects
  - Add predicted objects subscription and buffer management
  - Preserve dummy object velocity throughout movement
  * move predicted object functionality to its own file
  * fix back interactive object src file
  * Revert "move predicted object functionality to its own file"
  This reverts commit db769321b1bcee3d924abc4096508781c0452f3e.
  * remove unused include
  * rename findDisappearedPredictedObjects to findDisappearedPredictedObjectUUIDs
  * fix suggestions
  * pre-commit changes
  * re-add suggestions
  * add new line
  * make switch_time_threshold a parameter
  * make path selection random or not with parameter
  * refactor
  * refactor to simplify
  * refactor code
  * wip
  * simplify code
  * simplify logic
  * fix config file name
  * remove extra line
  * simplify code for pose interpolation
  * refactor speed calculation
  * move distance check earlier to speed up processing time
  * use akima spline for smoother paths
  * remove pedestrian random seed generator as it is not needed
  * Create stopAtZeroVelocity function to reduce duplicated code
  * move comment to improve readability
  * move check for path size to beggining of function
  * remove const qualifier
  * change bracket query for at()
  * refactor: migrate dummy_to_predicted_uuid_map\_ to PredictedDummyObjectInfo struct
  * refactor: migrate dummy_last_known_positions\_ to PredictedDummyObjectInfo struct
  * refactor: migrate dummy_creation_timestamps\_ to PredictedDummyObjectInfo struct
  * refactor: migrate dummy_last_used_predictions\_ to PredictedDummyObjectInfo struct
  * refactor: migrate dummy_last_used_prediction_times\_ to PredictedDummyObjectInfo struct
  * refactor: migrate dummy_prediction_update_timestamps\_ to PredictedDummyObjectInfo struct
  * refactor: migrate dummy_mapping_timestamps\_ to PredictedDummyObjectInfo struct
  * move all predictedObject tracking info to a single struct for readability
  * move predicted object mode parameters to its own struct
  * add base plugin and predicted plugin
  * WIP plugin architecture
  * WIP move predicted movement calculation to a separate plugin
  * move straight line movement to its own plugin
  * finish implementing plugins
  * make plugin vector
  * update README
  * add library for movement to get a cleaner code
  * update README
  * clarify predicted object
  * simplify code
  * remove unused includes
  * remove unused includes
  * remove unused includes
  * undo change in parameter
  * refactor: simplify code
  * fix indexing issue
  * remove unused dependencies and use autoware utils calc_distance2d
  * simplify code
  * remove extra return, small refactor
  * remove unnecessary check
  * remove typo in README
  ---------
* Contributors: Ryohsuke Mitsudome, danielsanchezaran

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* fix(cpplint): include what you use - simulator (`#9572 <https://github.com/autowarefoundation/autoware_universe/issues/9572>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix(tier4_dummy_object_rviz_plugin): fix missing dependency (`#9306 <https://github.com/autowarefoundation/autoware_universe/issues/9306>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix(tier4_dummy_object_rviz_plugin): fix missing dependency (`#9306 <https://github.com/autowarefoundation/autoware_universe/issues/9306>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* style: update rviz plugin icons to match the theme (`#8868 <https://github.com/autowarefoundation/autoware_universe/issues/8868>`_)
* fix(dummy_perception_publisher, tier4_dummy_object_rviz_plugin): separate dummy object msg (`#8828 <https://github.com/autowarefoundation/autoware_universe/issues/8828>`_)
  * fix: dummy object rviz plugin dependency
  * fix: remove message from dummy perception publisher
  * fix: node name
  ---------
* fix(tier4_dummy_object_rviz_plugin): fix unusedFunction (`#8844 <https://github.com/autowarefoundation/autoware_universe/issues/8844>`_)
  fix:unusedFunction
* fix(tier4_dummy_object_rviz_plugin): fix functionConst (`#8830 <https://github.com/autowarefoundation/autoware_universe/issues/8830>`_)
  fix:functionConst
* fix(tier4_perception_rviz_plugin): relocate tier4_perception_rviz_plugin and rename it to tier4_dummy_object_rviz_plugin (`#8818 <https://github.com/autowarefoundation/autoware_universe/issues/8818>`_)
  * chore: move tier4_perception_rviz_plugin package to simulation folder
  * chore: reorder codeowners
  * rename package to tier4_dummy_perception_rviz_plugin
  * chore: rename to tier4_dummy_object_rviz_plugin
  ---------
* Contributors: Khalil Selyan, Taekjin LEE, Yutaka Kondo, kobayu858

0.26.0 (2024-04-03)
-------------------
