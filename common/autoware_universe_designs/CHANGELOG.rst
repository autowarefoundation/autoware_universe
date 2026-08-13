^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_universe_designs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_universe_designs): add node designs for localization nodes (`#12801 <https://github.com/autowarefoundation/autoware_universe/issues/12801>`_)
  * feat(autoware_universe_designs): add node design for localization error monitor
  * feat(autoware_universe_designs): add node design for geo pose projector
  * feat(autoware_universe_designs): add node design for pose instability detector
  * feat(autoware_universe_designs): add node design for pose covariance modifier
  * feat(autoware_universe_designs): add node design for pose estimator arbiter
  * feat(autoware_universe_designs): add node design for ar tag based localizer
  * feat(autoware_universe_designs): add node design for lidar marker localizer
  * fix(autoware_universe_designs): add remap_target for debug and nested output topics in localization nodes
  ---------
* feat(autoware_universe_designs): add node designs for yabloc common, particle filter, pose initializer, and monitor nodes (`#12829 <https://github.com/autowarefoundation/autoware_universe/issues/12829>`_)
  * feat(autoware_universe_designs): add node design for yabloc ground server
  * feat(autoware_universe_designs): add node design for yabloc ll2 decomposer
  * feat(autoware_universe_designs): add node design for yabloc camera particle corrector
  * feat(autoware_universe_designs): add node design for yabloc gnss particle corrector
  * feat(autoware_universe_designs): add node design for yabloc predictor
  * feat(autoware_universe_designs): add node design for yabloc camera pose initializer
  * feat(autoware_universe_designs): add node design for yabloc monitor
  ---------
* feat(autoware_universe_designs): add node designs for yabloc image processing nodes (`#12824 <https://github.com/autowarefoundation/autoware_universe/issues/12824>`_)
  * feat(autoware_universe_designs): add node design for yabloc graph segment
  * feat(autoware_universe_designs): add node design for yabloc lanelet2 overlay
  * feat(autoware_universe_designs): add node design for yabloc line segment detector
  * feat(autoware_universe_designs): add node design for yabloc line segments overlay
  * feat(autoware_universe_designs): add node design for yabloc segment filter
  * feat(autoware_universe_designs): add node design for yabloc undistort
  ---------
* feat(autoware_universe_designs): add node designs for control nodes (`#12783 <https://github.com/autowarefoundation/autoware_universe/issues/12783>`_)
  * fix(autoware_universe_designs): correct StopModeOperator plugin name and use relative param path
  * feat(autoware_universe_designs): add node design for autonomous mode transition flag
  * feat(autoware_universe_designs): add node design for spheric collision detector
  * feat(autoware_universe_designs): add node design for control performance analysis
  * feat(autoware_universe_designs): add node design for joy controller
  * fix(autoware_universe_designs): correct socket names and remap targets in control node designs
  Launch-test verification (ros2 node info vs generated remaps) revealed
  socket names containing input/output prefixes generate doubled remap
  sources (~/input/input/...) that match no code topic. Corrected per
  each node's actual topic declarations:
  - ControlPerformanceAnalysis: bare socket names (code uses ~/input/<name>,
  ~/output/<name>); process trigger/outcome references updated to match
  - SphericCollisionDetector: bare names + relative remap_target (code
  subscribes to relative input/<name>); processing_time_ms publisher
  corrected to ~/debug/processing_time_ms (autoware_utils
  ProcessingTimePublisher default)
  - AutowareJoyController: bare names + relative remap_target for all
  topics and the emergency_stop service client (code uses relative names)
  - AutonomousModeTransitionFlag: relative remap_target for polling
  subscribers (code uses bare relative names); debug_info corrected
  to ~/debug_info
  All remaps verified at runtime against /localization/kinematic_state,
  /planning/trajectory, /control/command/control_cmd,
  /control/predicted_trajectory, /vehicle/status/steering_status.
  ---------
* refactor(vehicle_velocity_converter): update plugin name to VehicleVelocityConverterNode (`#12830 <https://github.com/autowarefoundation/autoware_universe/issues/12830>`_)
  Follow the node class rename from extracting VehicleVelocityConverterNode into a separate translation unit in autoware_core.
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* feat(autoware_universe_designs): add node designs for common packages (`#12749 <https://github.com/autowarefoundation/autoware_universe/issues/12749>`_)
  * feat(autoware_universe_designs): add path distance calculator node
  * feat(autoware_universe_designs): add node design for autoware_goal_distance_calculator
  * feat(autoware_universe_designs): add node designs for remaining common packages
  * fix(autoware_universe_designs): use relative param file path per review
  * fix(autoware_universe_designs): remove GlogComponent design (already in autoware_sample_designs)
  ---------
* feat(autoware_universe_designs): autware system designer planning and control nodes (`#12408 <https://github.com/autowarefoundation/autoware_universe/issues/12408>`_)
  * add planning/control and fix traffic light msg type
  * add rviz config
  * fixing planning nodes/modules
  * move to common/autoware_universe_designs
  * add missing params
  * improve control/planning
  * Update Autoware system design format to 0.3.0 and restructure node configurations across control and planning modules. Added package information, updated parameters, and refined interfaces for various nodes including AutonomousEmergencyBraking, CollisionDetector, and others. Removed deprecated ControlContainer node. Enhanced connections and subscriber/publisher definitions for improved modularity and clarity.
  * Refactor node configurations to standardize param_values across various modules in the Autoware universe design. This update ensures consistency by initializing param_values as empty lists in multiple node YAML files, enhancing clarity and maintainability.
  * Remove deprecated planning and control module configurations, including Control.module.yaml, LaneDriving.module.yaml, Planning.module.yaml, ScenarioPlanning.module.yaml, BehaviorPlanning.module.yaml, MissionPlanning.module.yaml, MotionPlanning.module.yaml, and Rviz.node.yaml. This cleanup enhances modularity and prepares for future updates.
  * Standardize boolean parameter values in ControlEvaluator and BehaviorVelocityPlanner node configurations from 'False' to 'false' for consistency across YAML files.
  * Update parameter file paths in planning node configurations to reference autoware_core_planning instead of autoware_planning_common. This change ensures consistency across multiple nodes, including BehaviorVelocityPlanner, ExternalVelocityLimitSelector, VelocitySmoother, and ElasticBandSmoother.
  * update planning control node execution info
  * Enhance node configurations across control and planning modules by updating message types, adding remap targets, and refining subscriber/publisher definitions. Key changes include the introduction of new message types in AutonomousEmergencyBraking, CollisionDetector, and other nodes, as well as the standardization of parameters for improved clarity and maintainability.
  * style(pre-commit): autofix
  * Update parameter file paths in planning node configurations to use relative paths instead of package share references. This change applies to multiple nodes including BehaviorPathPlanner, BehaviorVelocityPlanner, PathGenerator, CostmapGenerator, PlanningEvaluator, PlanningValidator, RemainingDistanceTimeCalculator, and VelocitySmoother, enhancing consistency and maintainability across the configuration files.
  * Enhance planning node configurations by adding new subscribers and publishers, updating message types, and refining parameter definitions. Key updates include the introduction of remap targets in ManualLaneChangeHandler and GoalPoseVisualizer, as well as the addition of new parameters in PlanningEvaluator, PlanningValidator, and VelocitySmoother for improved functionality and clarity.
  * Add new boolean parameters to PlanningEvaluator and PlanningValidator configurations for enhanced functionality. The parameters include 'output_metrics' in PlanningEvaluator and 'use_intra_process_comms' in PlanningValidator, improving clarity and control over node behavior.
  * style(pre-commit): autofix
  * Refactor PlanningValidator configuration by renaming 'planning_validator_launch_modules' to 'launch_modules' for improved clarity and consistency in parameter definitions.
  * Enhance MissionPlanner and RouteSelector configurations by adding remap targets for subscribers, publishers, and service servers/clients. This update improves clarity in message routing and aligns with the overall design structure for better integration with upstream and downstream components.
  * Add subscribers to StopModeOperator configuration for improved message handling. New subscribers include RouteState, SteeringReport, and VelocityReport, enhancing the node's integration with planning and vehicle control systems.
  * Add new subscribers and servers to ManualLaneChangeHandler configuration for enhanced functionality. The updates include a new reroute availability subscriber and a set preferred lane service server, improving the node's integration with planning and rerouting processes.
  * Enhance ManualLaneChangeHandler configuration by adding a new subscriber for LaneletMapBin and a client for SetPreferredPrimitive service. Additionally, implement a warning in the route callback to handle cases where the lanelet map is not yet loaded, improving robustness and integration with planning processes.
  * Update message types in BehaviorPathPlanner and MissionPlanner configurations to use autoware_planning_msgs for modified_goal. This change enhances consistency in message handling across planning nodes.
  * Add new publisher for traffic light recognition in SimplePlanningSimulator configuration. This update enhances the simulator's ability to process traffic signal data, improving overall functionality and integration with perception components.
  * style(pre-commit): autofix
  * fix path optimizer, /planning/trajectory published
  * Refactor node configurations: update message types and add new parameters for improved functionality
  * Enhance BehaviorPathPlanner and BehaviorVelocityPlanner configurations: add new parameter files and update existing parameters for improved module management and functionality.
  * style(pre-commit): autofix
  * Refactor node configurations: update message types and add new subscribers for improved functionality in various control and planning nodes.
  * Refactor node configurations: update subscriber and publisher names for consistency and clarity in BehaviorPathPlanner, BehaviorVelocityPlanner, and MotionVelocityPlanner nodes.
  * Refactor ControlValidator node: add remap targets for trajectory and predicted_trajectory subscribers for improved message routing
  * Refactor VehicleCmdGate node: update message types, add new subscribers and publishers, and enhance service configurations for improved functionality and clarity.
  * Refactor node configurations: update subscriber names for consistency and clarity in ExternalCmdSelector, add new publishers in OperationModeTransitionManager, and rename emergency command publisher in VehicleCmdGate.
  * Refactor ExternalCmdConverter node: add remap targets for subscribers and publisher for improved message routing
  * remap some engage and route topics
  * Refactor OperationModeTransitionManager and VehicleCmdGate nodes: update message types and remove global remap targets for improved clarity and consistency
  * Add steering_offset_update subscriber to TrajectoryFollower node for enhanced debugging
  * Update autoware_system_designer to version 0.4.0 in workflow and pre-commit configuration
  * Update autoware_system_designer to version 0.4.1 in workflow and pre-commit configuration
  ---------
  Co-authored-by: Maxime CLEMENT <maxime.clement@tier4.jp>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* refactor(`dummy_perception_publisher`): replace `tier4_simulation_msgs` by `autoware_simulation_msgs` (`#12499 <https://github.com/autowarefoundation/autoware_universe/issues/12499>`_)
  * chore: replace `tier4_simulation_msgs` by `autoware_simulation_msgs`
  * missing fix: to the previous commit (see below)
  * This commit applies the missing fix for the following commit
  - https://github.com/autowarefoundation/autoware_universe/pull/12499/changes/f0efc76abc007ab79bbff8e6a40ac2d12f8bc943
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * bug: use "simulated" for consistency, except its package name
  * Apply the following review comment
  - https://github.com/autowarefoundation/autoware_universe/pull/12499#discussion_r3161153194
  * fix: code formatting by `pre-commit`
  * bug: fix documentation bugs
  * style(pre-commit): autofix
  * Update simulator/autoware_dummy_perception_publisher/README.md
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
  * Update simulator/autoware_dummy_perception_publisher/README.md
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
  * Update simulator/autoware_dummy_perception_publisher/README.md
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
  * bug: fix to handle missing `default:` case
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
* Contributors: Junya Sasaki, Taekjin LEE, Takahisa Ishikawa, Vishal Chauhan, github-actions

0.51.0 (2026-05-01)
-------------------
* chore: align package versions to 0.50.0 and reset changelogs
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_universe_designs): introduce autoware system designer (`#12070 <https://github.com/mitsudome-r/autoware_universe/issues/12070>`_)
  * feat: add new design files for autoware universe and tier4 wrapper modules
  - Introduced CMakeLists.txt and package.xml for autoware_universe_designs and tier4_wrapper_modules.
  - Added various node configurations for perception, sensing, and vehicle control functionalities, enhancing the overall architecture and modularity of the system.
  feat: enhance Tier4LocalizationWrapper with additional input message types
  - Added new input message types for gnss_pose_with_covariance, camera_image, camera_info, and initial_pose to the Tier4LocalizationWrapper configuration.
  - This update improves the wrapper's capability to handle various sensor data, enhancing localization functionalities.
  refactor: remove initial_pose input from Tier4LocalizationWrapper configuration
  - Removed the initial_pose input message type from the Tier4LocalizationWrapper node configuration to streamline the input handling.
  - This change focuses on optimizing the wrapper's input structure for better performance and clarity.
  feat: add Tier4ADAPIWrapper configuration for Autoware system
  - Introduced a new YAML configuration file for the Tier4ADAPIWrapper node, defining inputs and outputs for localization, planning, control, vehicle status, perception, and system functionalities.
  - This addition enhances the modularity and integration of the Autoware system, facilitating better communication between components.
  feat: add additional parameters to Tier4LocalizationWrapper configuration
  - Introduced new parameters for ndt_enabled, gnss_enabled, ekf_enabled, yabloc_enabled, and stop_check_enabled in the Tier4LocalizationWrapper node configuration.
  - This enhancement allows for greater flexibility and control over the localization functionalities within the system.
  feat: add user-defined initial pose parameters to Tier4LocalizationWrapper configuration
  - Introduced new parameters for user_defined_initial_pose/enable and user_defined_initial_pose/pose in the Tier4LocalizationWrapper node configuration.
  - This enhancement allows users to specify an initial pose for localization, improving the flexibility and accuracy of the system's localization capabilities.
  feat: add new point cloud concatenation node configurations
  - Introduced new YAML configuration files for ConcatenateAndTimeSync_3_inputs and ConcatenateAndTimeSync_8_inputs nodes, enhancing the point cloud preprocessing capabilities.
  - The configurations allow for the synchronization and concatenation of multiple PointCloud2 inputs, improving data handling for sensing applications.
  - Updated the base ConcatenateAndTimeSync node configuration to support the new input structures and processes.
  feat: update node configurations with remapping targets and additional parameters
  - Added remap_target fields for inputs and outputs in ImuCorrector, VehicleVelocityConverter, and DistortionCorrector node configurations to enhance data handling.
  - Introduced new parameters related to vehicle dimensions in the Tier4ADAPIWrapper configuration, improving the system's adaptability to different vehicle specifications.
  feat: update GnssPoser configuration with remapping targets and new parameters
  - Added remap_target fields for inputs and outputs in the GnssPoser node configuration to improve data routing.
  - Introduced a new parameter, use_gnss_ins_orientation, enhancing the node's configurability for GNSS and INS integration.
  refactor: remove Tier4ADAPIWrapper configuration file
  - Deleted the Tier4ADAPIWrapper.node.yaml configuration file, streamlining the project structure by removing unused or deprecated configurations.
  - This change focuses on maintaining a cleaner codebase and improving overall project organization.
  feat: add initialization_state input to Tier4LocalizationWrapper configuration
  - Introduced a new input message type, initialization_state, to the Tier4LocalizationWrapper node configuration, enhancing its ability to process localization initialization data.
  - This addition improves the overall functionality and adaptability of the localization system.
  refactor: update node configurations to use 'base' instead of 'inheritance'
  * refactor: remove deprecated perception module configurations
  - Deleted several YAML configuration files related to object recognition and traffic light recognition modules, including ObjectRecognitionCLMerged, ObjectRecognitionCLRMerged, ObjectRecognitionCLRParallel, ObjectRecognitionLidar, ObjectDetectionCLMerged, ObjectDetectionLidar, and various traffic light recognition modules.
  - This cleanup focuses on streamlining the project structure by removing unused or outdated configurations, enhancing maintainability and clarity in the codebase.
  * feat: enhance Tier4ControlWrapper and Tier4PlanningWrapper with new input and output message types
  - Added new input message types for 'objects' and 'obstacle_pointcloud' in both Tier4ControlWrapper and Tier4PlanningWrapper configurations, improving the wrappers' capabilities to handle perception data.
  - Introduced a new output message type 'trajectory' in Tier4PlanningWrapper, enhancing the planning functionalities.
  - Updated parameters to include topic names for the new inputs, facilitating better integration and data flow within the system.
  * feat: add new parameters to Tier4ControlWrapper configuration
  - Introduced new parameters for trajectory_follower_mode and various launch options, enhancing the configurability and functionality of the Tier4ControlWrapper.
  - This update improves the integration of additional features related to control and safety checks within the system.
  * feat: add use_sim_time parameter to Tier4ControlWrapper configuration
  - Introduced a new parameter, use_sim_time, with a default value of false, enhancing the configurability of the Tier4ControlWrapper.
  - This addition allows for better simulation time management within the system.
  * feat: add TrafficLightMultiCameraFusion input node configuration
  - Introduced a new YAML configuration file for TrafficLightMultiCameraFusion_1_input.node, defining input parameters and processes for traffic light detection using multiple camera inputs.
  - This addition enhances the perception capabilities of the system by allowing for the fusion of data from specified camera namespaces.
  * feat: update TrafficLightMultiCameraFusion configurations with remap targets
  - Added remap_target fields for camera inputs in TrafficLightMultiCameraFusion_4_inputs.node.yaml and TrafficLightMultiCameraFusion.node.yaml, enhancing data routing for traffic light detection.
  - This update improves the system's ability to handle multiple camera inputs effectively, ensuring better integration and data flow.
  * feat: add Tier4SystemWrapper configuration file
  - Introduced a new YAML configuration file for Tier4SystemWrapper.node, defining inputs, outputs, parameters, and processes for system diagnostics and command mode management.
  - This addition enhances the modularity and functionality of the Autoware system, facilitating better integration and communication between components.
  * feat: add new input message types to Tier4ControlWrapper configuration
  - Introduced new input message types for 'imu' and 'velocity_status' in the Tier4ControlWrapper.node.yaml configuration.
  - This enhancement improves the wrapper's capability to process additional sensor data, facilitating better integration and control within the Autoware system.
  * refactor: remove deprecated input parameters from Tier4ControlWrapper configuration
  - Removed input_objects_topic_name and input_pointcloud_topic_name parameters from the Tier4ControlWrapper.node.yaml configuration.
  - This cleanup enhances the clarity and maintainability of the configuration by eliminating unused parameters.
  * feat: add global field to diagnostics input in Tier4SystemWrapper configuration
  - Introduced a global field for the diagnostics input in the Tier4SystemWrapper.node.yaml configuration, enhancing the accessibility of diagnostic messages across the system.
  - This update improves the integration and communication of diagnostic data within the Autoware framework.
  * add ring outlier filter
  * fix laserscan ogm
  * feat: add new perception publisher and RViz plugin configurations
  - Introduced DummyPerceptionPublisher.module.yaml to define the perception publisher's structure and connections.
  - Added DummyPerceptionPublisher.node.yaml for the perception publisher node, detailing inputs, outputs, and processes.
  - Created RvizPluginDummyObject.node.yaml for the RViz plugin, specifying its output and periodic publishing process.
  - These additions enhance the perception capabilities and visualization within the Autoware system.
  * fix: update output names in DummyPerceptionPublisher.module.yaml
  - Changed output name from 'object_recognition/detection/objects' to 'detected_objects' for clarity and consistency.
  - Updated connection references accordingly to reflect the new output naming.
  * feat: add parameters for DummyPerceptionPublisher configuration
  * feat: add PoseInitializer node configuration
  - Introduced PoseInitializer.node.yaml to define the configuration for the PoseInitializer node, including inputs, outputs, parameters, and processes.
  - This addition enhances the localization capabilities within the Autoware system by providing a structured approach to pose initialization.
  * feat: add SimplePlanningSimulator node configuration
  - Introduced SimplePlanningSimulator.node.yaml to define the configuration for the Simple Planning Simulator node, including inputs, outputs, parameters, and processes.
  - This addition enhances simulation capabilities within the Autoware system by providing a structured approach to planning simulation.
  * feat: add actuation command input to SimplePlanningSimulator configuration
  - Added a new input for actuation commands in SimplePlanningSimulator.node.yaml, enhancing the simulator's capability to process vehicle actuation commands.
  - This update improves the overall functionality and realism of the planning simulation within the Autoware system.
  * fix: update output names and add parameters in PoseInitializer and RawVehicleCmdConverter configurations
  * fix: update remap targets for inputs and outputs in DummyPerceptionPublisher configuration
  * fix: update output names in DummyPerceptionPublisher module and PointcloudToLaserscan configuration
  * feat: add parameters for obstacle pointcloud input in LaserscanBasedOccupancyGridMap configuration
  - Introduced new parameters for inputting obstacle pointcloud and both obstacle and raw pointcloud in LaserscanBasedOccupancyGridMap.node.yaml.
  - This enhancement improves the flexibility and functionality of the occupancy grid mapping process within the Autoware system.
  * feat: add VehicleDoorSimulator node configuration
  - Introduced VehicleDoorSimulator.node.yaml to define the configuration for the Vehicle Door Simulator node, including inputs, outputs, parameters, and processes.
  - This addition enhances the simulation capabilities within the Autoware system by providing a structured approach to vehicle door simulation.
  * feat: update PointcloudToLaserscan configuration with new remap targets and additional outputs
  - Changed remap targets for pointcloud and laserscan outputs to use a structured naming convention.
  - Added new outputs for pointcloud, ray, and stixel to enhance the data flow and visualization capabilities within the occupancy grid mapping process.
  * feat: add autoware_universe_designs package and multiple perception node configurations
  - Introduced the autoware_universe_designs package with CMakeLists.txt and package.xml for project setup.
  - Added various perception node configurations including Pose2Twist, PoseInitializer, ClusterMerger, and others, enhancing the system's capabilities for localization and perception tasks.
  - Each node configuration includes detailed inputs, outputs, parameters, and processes to facilitate integration within the Autoware framework.
  * move module files to autoware_launch
  * move wrappers to autoware_launch
  * feat: update autoware_system_design_format to v0.2.0 and add package information for multiple perception nodes
  - Updated the autoware_system_design_format to v0.2.0 across various node configurations.
  - Added package name and provider details for each perception node, enhancing clarity and organization within the autoware_universe_designs package.
  - This update includes nodes such as Pose2Twist, ClusterMerger, and TrafficLightClassifier, among others, improving the overall structure and usability of the system.
  * feat: update autoware_system_design_format to 0.2.0 across multiple node configurations
  - Changed the autoware_system_design_format from v0.2.0 to 0.2.0 in various node YAML files, ensuring consistency in the design format.
  - This update affects numerous perception, localization, sensing, and simulation nodes, enhancing the overall structure and usability of the autoware_universe_designs package.
  * feat: add autoware_ament_auto_package to CMakeLists.txt
  * refactor: remove dependency on autoware_system_designer from package.xml
  * chore: fix mis spell and exception
  * fix: standardize formatting in YAML files and CMakeLists.txt
  - Added missing newlines and corrected spacing in various YAML node configurations to ensure consistent formatting.
  - Updated CMakeLists.txt to include a newline at the end of the file for better compatibility with tools that require it.
  * fix yamllint
  * fix yamllint 2
  * fix: correct spacing in DistortionCorrector.node.yaml for YAML compliance
  * fix node yaml
  * Update autoware_system_design_format to 0.3.0 across multiple node configurations
  - Changed the autoware_system_design_format from 0.2.0 to 0.3.0 in various YAML files for perception, localization, and sensing nodes.
  - Standardized parameter file naming from `parameter_files` to `param_files` and `parameters` to `param_values` for consistency across configurations.
  This update enhances the overall structure and usability of the autoware_universe_designs package.
  * refactor: update interface originated configurations
  * fix: update output remap target
  * replace input field
  * replace output field
  * fix: update plugin and executable names in ConcatenatePointcloud8 and DistortionCorrector node configurations
  * remove container_name field from multiple perception and sensing node configurations for consistency
  * remove use_container field from multiple perception and sensing node configurations for consistency
  * fix: update plugin path for LowIntensityClusterFilter node in YAML configuration
  * fix: update autoware_system_designer version to v0.3.1 in workflow and pre-commit config
  ---------
  Co-authored-by: Takagi, Isamu <isamu.takagi@tier4.jp>
* Contributors: Taekjin LEE, github-actions
