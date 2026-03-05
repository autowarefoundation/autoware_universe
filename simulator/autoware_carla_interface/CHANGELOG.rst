^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_carla_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: carla interface config and docs (`#11571 <https://github.com/autowarefoundation/autoware_universe/issues/11571>`_)
  * fix(autoware_carla_interface): correct config file installation paths
  Fix setup.py to install sensor_mapping.yaml in config/ subdirectory
  instead of root share directory. This ensures the package works correctly
  in production/deployment scenarios where source files are not available.
  - raw_vehicle_cmd_converter.param.yaml: installed to share root (correct)
  - sensor_mapping.yaml: installed to share/config/ (matches expected path)
  Without this fix, the package relies on fallback to source directory which
  fails in Docker containers and binary package deployments.
  * fix(autoware_carla_interface): complete sensor mapping example in README
  Update camera sensor mapping example to include all required fields:
  - Add topic_info field for camera_info topic
  - Add qos_profile field for ROS2 QoS configuration
  - Remove unnecessary quotes from YAML values
  These fields are essential for proper camera sensor configuration and were
  missing from the documentation example, potentially causing confusion for
  users trying to configure custom sensors.
  * docs(autoware_carla_interface): remove misleading LiDAR concatenation note
  Remove the note about uncommenting LiDAR concatenation relay from Known Issues
  section. The single LiDAR configuration may still require the concatenated topic
  for coordinate transformation, which will be tested separately.
  The relay in launch file remains commented out pending further testing.
  * refactor(autoware_carla_interface): remove unnecessary lidar concatenation relay
  Remove commented-out lidar concatenation relay from launch file. Testing confirms
  that the main Autoware sensing pipeline already provides the concatenated pointcloud
  topic through the mirror_cropped pipeline, making this relay redundant.
  The /sensing/lidar/concatenated/pointcloud topic is successfully published by
  the main sensing stack and consumed by localization and perception modules.
  ---------
* feat(autoware_carla_interface): sensor kit integration with multi-camera support (`#11471 <https://github.com/autowarefoundation/autoware_universe/issues/11471>`_)
  * feat(autoware_carla_interface): add sensor kit integration with multi-camera support
  - Add sensor_mapping.yaml for sensor kit configuration loader
  - Implement coordinate transformer for ROS base_link to CARLA vehicle frame
  - Add sensor kit loader with Autoware calibration parsing
  - Create multi-camera combiner node for 6-camera grid visualization
  - Add topic relays and image compression for all 6 cameras
  - Implement modular sensor manager with ROS publisher management
  - Refactor carla_ros.py to use new sensor kit architecture
  This enables dynamic sensor configuration from Autoware sensor kit YAML files
  and provides compressed image streams for bandwidth optimization.
  * refactor(autoware_carla_interface): remove legacy objects.json and simplify sensor configuration
  - Remove objects.json and objects_definition_file parameter (replaced by sensor_mapping.yaml)
  - Remove use_autoware_sensor_kit parameter (always use sensor kit now)
  - Simplify sensor loading logic in carla_ros.py
  - Update sensor_kit_loader to always attempt sensor kit calibration first
  - Update launch file and documentation to reflect new configuration
  - Make wheelbase configurable via sensor_mapping.yaml
  This simplifies the configuration by removing redundant parameters and
  making the sensor kit approach the standard method.
  * refactor(autoware_carla_interface): move multi_camera_combiner to proper module structure
  - Move scripts/multi_camera_combiner.py to src/autoware_carla_interface/multi_camera_combiner_node.py
  - Register as entry point in setup.py for standard ROS2 node deployment
  - Update launch file to use node instead of executable
  - Remove empty scripts directory
  This follows ROS2 Python package best practices by having all executable
  nodes as entry points rather than standalone scripts.
  * fix(autoware_carla_interface): prevent silent sensor misconfiguration at origin
  Critical fixes for sensor kit loading:
  1. Fix default sensor_kit_name pointing to wrong package
  - Changed from "carla_sensor_kit_launch" to "carla_sensor_kit_description"
  - The _description package contains the required calibration files
  - Launch package does not have sensor transforms
  2. Remove dangerous silent fallback to mapping-only configuration
  - Mapping file has NO transform data, would spawn all sensors at (0,0,0)
  - Now fails fast with clear error message instead of broken sensor layout
  - Prevents silent sensor misconfiguration that appears to work but is broken
  3. Add validation for sensor transforms
  - Verify all enabled sensors have transform calibration data
  - Fail with descriptive error if transforms are missing
  - Prevents degenerate sensor configurations
  4. Remove unused _create_configs_from_mapping() method
  - No longer needed as mapping-only mode is not supported
  - Mapping file is only used for topics/QoS/parameters, not poses
  Without these fixes, sensor kit lookup failures would silently create a broken
  configuration with all cameras/LiDAR at the vehicle origin looking forward.
  * refactor(autoware_carla_interface): improve code quality and sensor configuration
  This commit enhances the overall code quality, error handling, and
  documentation of the CARLA-Autoware interface. Key improvements include:
  Core Refactoring:
  - Migrate all sensors to registry-based publishing (GNSS/pose included)
  - Remove legacy frequency tracking in favor of simulation-time based system
  - Delete obsolete sensor_kit_parser.py module
  - Convert SensorWrapper._sensors_list from class to instance variable
  Error Handling & Validation:
  - Add comprehensive YAML validation with detailed error messages
  - Improve sensor kit package discovery with better fallback logic
  - Add safe error handling in sensor setup with graceful degradation
  - Enhance sensor ID validation to prevent silent misconfigurations
  Code Quality:
  - Add proper shebangs (#!/usr/bin/env python3) to Python modules
  - Fix corrupted license comments
  - Add comprehensive docstrings to all major methods
  - Document thread safety issues in ROS spin thread
  - Improve shutdown procedure to prevent publisher/thread leaks
  Configuration & Standards:
  - Standardize angle units (radians) per Autoware conventions
  - Remove angle unit auto-detection heuristic
  - Update default use_traffic_manager to False
  - Clarify GNSS covariance matrix documentation
  Documentation:
  - Add detailed sensor configuration guide to README
  - Document sensor kit calibration file structure
  - Add CARLA sensor parameter references
  - Remove completed TODO items from known limitations
  * fix(autoware_carla_interface): implement thread safety with proper locking
  Add threading.Lock to protect shared state accessed by both ROS spin thread
  and main simulation loop. This fixes critical race conditions that could cause:
  - Control commands being partially applied
  - Stale pose data being used
  - Potential crashes from concurrent actor access
  Changes:
  - Add self._state_lock (threading.Lock) to protect shared variables
  - Protect control_callback: locks when writing current_control
  - Protect initialpose_callback: locks when accessing/modifying ego_actor
  - Protect pose(): locks when reading ego_actor transform
  - Protect ego_status(): locks when reading all ego_actor state
  - Protect run_step(): locks when reading current_control
  - Update documentation to reflect thread-safe implementation
  - Remove outdated TODO comment about adding synchronization
  Protected shared state:
  - self.current_control (written by ROS callback, read by simulation loop)
  - self.ego_actor (written by initialpose callback, read everywhere)
  - self.physics_control (accessed by control callback)
  - self.timestamp (read/written from both threads)
  * fix(autoware_carla_interface): add robust cleanup and exception handling
  Implement comprehensive resource cleanup to prevent actor leaks and ensure
  proper shutdown even when exceptions occur or signals are received.
  Changes:
  Main Entry Point (carla_autoware.py):
  - Add try/finally block to ensure cleanup on all exit paths
  - Register both SIGINT and SIGTERM signal handlers
  - Add exception handling for KeyboardInterrupt
  - Improve _cleanup() with individual try/except blocks
  - Cleanup resources in reverse initialization order
  - Continue cleanup even if individual steps fail
  Sensor Cleanup (carla_wrapper.py):
  - Improve SensorWrapper.cleanup() robustness
  - Separate stop() and destroy() calls with individual error handling
  - Collect and log cleanup errors without failing entire cleanup
  - Ensure all sensors are cleaned up even if some fail
  - Clear sensors list after cleanup
  Benefits:
  - Prevents CARLA actor leaks on crashes or Ctrl+C
  - Ensures ROS shutdown happens properly
  - No hanging processes or zombie actors
  - Cleaner error messages during shutdown
  - Resources freed even on partial failures
  * fix(autoware_carla_interface): protect timestamp write with lock in run_step
  Fix critical race condition where main thread writes self.timestamp without
  lock while ROS callback thread reads it (via first_order_steering) with lock.
  Issue:
  - run_step() wrote self.timestamp = timestamp WITHOUT acquiring _state_lock
  - control_callback() calls first_order_steering() WHILE holding _state_lock
  - first_order_steering() reads self.timestamp to calculate dt
  - Race: main thread writes, ROS thread reads -> corrupt timestamp value
  - Result: negative or zero dt, unstable steering calculations
  Fix:
  - Wrap timestamp write in run_step() with self._state_lock
  - Now both read and write are properly synchronized
  - Prevents partially-updated timestamp values
  - Ensures stable dt calculations in steering model
  Protected flow:
  1. Main thread: WITH lock, write self.timestamp
  2. ROS thread: WITH lock, read self.timestamp (in first_order_steering)
  3. Both threads use same lock = no race condition
  * fix(autoware_carla_interface): fix None timestamp crash and remove dead code
  Fix two code quality issues:
  1. Fix TypeError crash in first_order_steering when control arrives early
  - Issue: If control command arrives before first run_step(), self.timestamp is None
  - Symptom: TypeError on line 626: unsupported operand type(s) for -: 'NoneType' and 'NoneType'
  - Fix: Guard against None timestamp, return raw steering input until initialized
  - Prevents crash during startup race condition
  2. Remove unused self.channels dead code
  - self.channels was initialized to 0 but never read or written
  - Leftover from previous implementation
  - Removing improves code clarity
  Changes:
  - Add None check at start of first_order_steering()
  - Return unfiltered input when timestamp not yet available
  - Add docstring explaining graceful degradation behavior
  - Remove self.channels from _initialize_instance_variables()
  * fix(autoware_carla_interface): preserve filter state on repeated commands
  Fix critical bug in first_order_steering that caused steering spikes when
  multiple control commands arrived within the same CARLA simulation tick.
  Issue:
  - Planning/control can publish multiple actuation commands per tick
  - Each command with dt=0 would reset steer_output to 0.0
  - We then stored 0.0 in prev_steer_output, losing filter state
  - Result: Artificial steering spike to zero before recovering next tick
  Root cause:
  ```python
  steer_output = 0.0  # Reset to zero
  dt = self.timestamp - self.prev_timestamp
  if dt > 0.0:
  steer_output = computed_value  # Only set if dt > 0
  self.prev_steer_output = steer_output  # Store 0.0 if dt <= 0!
  ```
  Fix:
  - When dt <= 0 (repeated commands same frame): early return prev_steer_output
  - Preserves filter state across intra-frame command bursts
  - Eliminates zero spikes from state loss
  - Only update filter state when time actually advances (dt > 0)
  Also improved first call initialization:
  - Initialize prev_steer_output to first input (not implicit 0.0)
  - Cleaner logic flow with early returns
  * chore(autoware_carla_interface): fix pre-commit linting issues
  - Remove unused imports (datetime, math)
  - Fix docstrings to use imperative mood
  - Remove redundant YAML quotes for yamllint compliance
  * docs(autoware_carla_interface): add multi-camera view setup instructions
  - Add new section explaining how to view combined 6-camera feed in RViz
  - Provide step-by-step instructions to add Image display manually
  - Include note about optionally disabling multi_camera_combiner node
  - Reference screenshot at docs/images/rviz_multi_camera_view.png
  - Avoids maintaining custom RViz config, uses default Autoware config
  * docs(autoware_carla_interface): improve README with updated commands and clearer structure
  - Update launch command to use sensor_model:=carla_sensor_kit
  - Format launch command as multiline for better readability
  - Reorganize Install section with clearer Prerequisites and Map Setup subsections
  - Add emphasis on carla_sensor_kit in Sensor Configuration section
  - Improve Known Issues section formatting
  - Add note about LiDAR concatenation configuration
  * style(pre-commit): autofix
  * refactor(sensor_kit_loader): reduce method complexity to pass CodeScene checks
  Extract helper methods to reduce complexity and improve code health:
  1. load_sensor_mapping (66 → 22 lines):
  - Extract _resolve_mapping_file_path for file path resolution
  - Extract _validate_sensor_mapping_yaml for YAML validation
  - Extract _load_vehicle_config for vehicle config loading
  2. find_sensor_kit_path (65 → 43 lines):
  - Extract _try_find_sensor_kit to eliminate duplicate try-except blocks
  - Extract _create_not_found_error for error message generation
  - Reduce nesting depth from 4 to 2 levels
  3. _create_configs_from_kit (40 → 27 lines):
  - Extract _try_create_sensor_config for single sensor processing
  - Extract _find_sensor_mapping to eliminate nested loop
  - Reduce cyclomatic complexity
  Benefits:
  - All methods now under 30 lines (Large Method fixed)
  - Maximum nesting depth reduced to 2 (Deep Complexity fixed)
  - Each method has single responsibility (Complexity fixed)
  - Eliminates code duplication (DRY principle)
  Addresses CodeScene quality gate failures.
  * style(pre-commit): autofix
  * refactor(autoware_carla_interface): improve code quality and reduce complexity
  - Fix spelling issues: replace 'republishers' with 'Image transport nodes', correct 'ROS2' to 'ROS 2'
  - Consolidate duplicated publisher creation methods into single generic helper function
  - Reduce nested complexity in normalize_sensor_name from 4 to 2 levels using early returns
  - Flatten conditional logic in find_sensor_kit_path to reduce bumpy road complexity
  - Split _validate_sensor_mapping_yaml into focused validation methods
  - Extract wheelbase validation into dedicated method
  These changes reduce cyclomatic complexity, eliminate code duplication, and improve maintainability while preserving functionality.
  * style(pre-commit): autofix
  * refactor(autoware_carla_interface): reduce complexity and improve code health
  Major refactoring to address CodeScene quality gate violations:
  **Bumpy Road Issues Fixed:**
  - carla_wrapper.py: Extract cleanup logic into _cleanup_single_sensor method
  - carla_autoware.py: Split _cleanup into 4 focused methods (_cleanup_sensors,
  _cleanup_ros_interface, _cleanup_ego_actor, _cleanup_carla_provider)
  - Eliminates all nested conditional blocks in cleanup paths
  **Complex Method Improvements:**
  - carla_wrapper.py: Reduce setup_sensors cyclomatic complexity from 13 to 4
  - Extract _setup_single_sensor for per-sensor setup logic
  - Create dedicated attribute configurers per sensor type:
  _configure_camera_attributes, _configure_lidar_attributes,
  _configure_gnss_attributes, _configure_imu_attributes
  - Extract _create_sensor_transform for transform creation
  - carla_ros.py: Extract _create_gnss_covariance_matrix from pose method
  - Reduces pose method from 70 to 40 lines
  **Code Duplication Elimination:**
  - coordinate_transformer.py: Consolidate rotation conversion methods
  - Create _create_carla_rotation helper with angle negation parameter
  - Eliminates 20+ lines of duplicated code between
  carla_rotation_to_carla_rotation and ros_to_carla_rotation
  All refactoring maintains 100% functional equivalence while significantly
  improving maintainability, testability, and code health metrics.
  * style(pre-commit): autofix
  * refactor(autoware_carla_interface): fix remaining CodeScene quality issues
  Address final CodeScene advisory quality gate failures:
  **Code Duplication & Excess Arguments (coordinate_transformer.py):**
  - Inline rotation conversion logic into carla_rotation_to_carla_rotation
  and ros_to_carla_rotation methods
  - Remove _create_carla_rotation helper with 5 arguments (exceeded 4 arg limit)
  - Methods now sufficiently distinct to avoid duplication warnings:
  - carla_rotation_to_carla_rotation: No angle negation
  - ros_to_carla_rotation: Negates pitch and yaw for coordinate system change
  **Complex Method (carla_ros.py):**
  - Reduce camera method cyclomatic complexity from 9 to 3
  - Extract _create_camera_image_message for image conversion
  - Extract _prepare_camera_info for camera info preparation
  - Extract _publish_camera_messages for publishing logic
  All changes maintain functional equivalence while improving code health metrics
  to pass CodeScene quality gates.
  * refactor(carla_interface): fix CodeScene quality gates and CI test import
  Fix CodeScene quality gate failures and CI test collection error:
  Quality Improvements:
  - Move GNSS covariance configuration from hardcoded matrix to YAML config
  - Reduce carla_ros.py complexity by externalizing covariance parameters
  - Eliminate code duplication in coordinate_transformer.py rotation methods
  - Extract common rotation conversion logic into helper method
  Technical Changes:
  - Add covariance field to SensorConfig dataclass
  - Update sensor_mapping.yaml with position_variance and orientation_variance
  - Refactor _create_gnss_covariance_matrix() to read from sensor config
  - Create _convert_rotation_to_carla() helper for shared rotation logic
  - Add try/except import guard for carla module to support test environments
  Impact:
  - Resolves "Lines of Code in a Single File" violation in carla_ros.py
  - Resolves "Code Duplication" violation in coordinate_transformer.py
  - Fixes ModuleNotFoundError during pytest test collection in CI
  - Improves maintainability and configurability
  * style(pre-commit): autofix
  * refactor(autoware_carla_interface): resolve CodeScene code health warnings
  Improves code maintainability by addressing CodeScene quality metrics:
  - Reduce function arguments in coordinate_transformer by grouping angles
  - Simplify conditional logic in GNSS covariance matrix creation
  - Consolidate redundant code and remove unnecessary comments
  - Reduce overall lines of code to improve readability
  This brings the codebase below CodeScene thresholds for complexity and file size.
  * refactor(autoware_carla_interface): consolidate duplicate docstrings in coordinate_transformer
  Reduces code duplication by consolidating verbose docstrings to concise
  one-liners for rotation conversion methods. Both carla_rotation_to_carla_rotation
  and ros_to_carla_rotation now have minimal documentation, as the detailed
  documentation is maintained in the shared _convert_rotation_to_carla helper.
  Also adds 'from __future_\_ import annotations' to enable deferred annotation
  evaluation (PEP 563), which fixes AttributeError during test imports when
  CARLA is not available (carla = None).
  This resolves CodeScene code duplication warning while preserving all
  functional information and fixing test compatibility.
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* chore(carla-interface): fix spell-check (`#11400 <https://github.com/autowarefoundation/autoware_universe/issues/11400>`_)
  chore: fix spell-check
* fix(autoware_carla_interface): improve QoS compatibility and pointcloud handling (`#11372 <https://github.com/autowarefoundation/autoware_universe/issues/11372>`_)
* Contributors: Bingo, Kotaro Uetake, Max-Bin, Ryohsuke Mitsudome

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): autofix (`#10982 <https://github.com/autowarefoundation/autoware_universe/issues/10982>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome

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
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_carla_interface): fix lidar topic name (`#9645 <https://github.com/autowarefoundation/autoware_universe/issues/9645>`_)
* Contributors: Fumiya Watanabe, Maxime CLEMENT

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
* fix(autoware_carla_interface): include "modules" submodule in release package and update setup.py (`#9561 <https://github.com/autowarefoundation/autoware_universe/issues/9561>`_)
* feat!: replace tier4_map_msgs with autoware_map_msgs for MapProjectorInfo (`#9392 <https://github.com/autowarefoundation/autoware_universe/issues/9392>`_)
* refactor: correct spelling (`#9528 <https://github.com/autowarefoundation/autoware_universe/issues/9528>`_)
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, Jesus Armando Anaya, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
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
* fix(autoware_carla_interface): resolve init file error and colcon marker warning (`#9115 <https://github.com/autowarefoundation/autoware_universe/issues/9115>`_)
* fix: removed access to unused ROS_VERSION environment variable. (`#8896 <https://github.com/autowarefoundation/autoware_universe/issues/8896>`_)
* ci(pre-commit): autoupdate (`#7630 <https://github.com/autowarefoundation/autoware_universe/issues/7630>`_)
  * ci(pre-commit): autoupdate
  * style(pre-commit): autofix
  * fix: remove the outer call to dict()
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
* feat(carla_autoware): add interface to easily use CARLA with Autoware (`#6859 <https://github.com/autowarefoundation/autoware_universe/issues/6859>`_)
  Co-authored-by: Minsu Kim <minsu@korea.ac.kr>
* Contributors: Esteve Fernandez, Giovanni Muhammad Raditya, Jesus Armando Anaya, Yutaka Kondo, awf-autoware-bot[bot]

0.26.0 (2024-04-03)
-------------------
