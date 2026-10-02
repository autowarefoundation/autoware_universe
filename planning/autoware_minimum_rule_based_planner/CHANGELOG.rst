^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_minimum_rule_based_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* chore(autoware_trajectory_processor)!: rename to autoware_trajectory_modifier (`#13389 <https://github.com/autowarefoundation/autoware_universe/issues/13389>`_)
  rename processor -> modifier
* feat(trajectory_modifier, minimum_rule_based_planner): integrate semseg pointcloud into modifier and backup planner modules (`#13265 <https://github.com/autowarefoundation/autoware_universe/issues/13265>`_)
  * fix get_nearest_object_collision function to return distance to projected collision point instead of didistance to initial object state
  * modify get_object_polygon lambda to simply polygon expansion for shape types other than POLYGON
  * fix format
  * refactor get_predicted_obj_pose_at_time() to use highest confidence non empty predicted path
  * support new perception pointcloud interface using point type PointXYZCPE, and update pointcloud processing code in modifier and backup planner
  - Switch obstacle-stop point type from pcl::PointXYZ to PointXYZCPE
  - Filter input points by configurable target class labels (class_id) and axis-aligned range/height bounds
  - Remove voxel-grid downsampling, Euclidean clustering, and convex-hull extraction from the obstacle-stop PCD pipeline
  - Replace PCL CropBox / transform_pointcloud usage with manual x/y/z filtering and transform (PointXYZCPE has no PCL .data member)
  - Add PointCloudClassification -> ObjectType mapping for semantic labels
  - Replace voxel_grid_filter / clustering params with pointcloud.target_types in config, parameter structs, and schemas for both packages
  - Default target_types to hazard, structure, and vegetation
  - Add autoware_point_types dependency to trajectory_modifier
  - Update trajectory_modifier obstacle-stop integration test params for the simplified filter path
  - Share the simplified PointCloudFilter path with minimum_rule_based_planner obstacle_stop
  * align PCD crop and clean up cluster debug leftovers
  - Match MRBP obstacle-stop crop AABB to trajectory_modifier using std::minmax and a 1.0 m buffer so inverted corners after yaw do not empty the crop box
  - Remove dead cluster_points debug path after clustering removal
  - Publish filtered_points from MRBP obstacle_stop and update debug text to filtered -> target in both packages
  * preserve PointXYZCPE fields in obstacle tracker
  - Store full PointXYZCPE in PersistentPoint instead of xyz-only geometry_msgs positions
  - Keep class_id, probability, and entropy when emitting active tracked points
  * clean up code
  * filter surround obstacle pointclouds by type and range
  - Parse PointXYZCPE clouds in surround_obstacle_stop and keep only configured semantic labels before proximity checks
  - Crop to the ego footprint expanded by front/side/back thresholds and hysteresis, then convert to PointXYZ for ProximityChecker
  - Add target_objects.pointcloud params (default hazard/structure/vegetation) to config, parameter structs, and schemas in both packages
  - Allow unknown labels in trajectory_modifier surround integration tests so XYZ-only fixtures still exercise the PCD path
  * Update planning/autoware_minimum_rule_based_planner/config/minimum_rule_based_planner.param.yaml
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * Update planning/autoware_minimum_rule_based_planner/config/minimum_rule_based_planner.param.yaml
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * Update planning/autoware_minimum_rule_based_planner/param/minimum_rule_based_planner_parameters.yaml
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * Update planning/autoware_minimum_rule_based_planner/param/minimum_rule_based_planner_parameters.yaml
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * Update planning/autoware_trajectory_modifier/config/trajectory_modifier.param.yaml
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * Update planning/autoware_trajectory_modifier/src/trajectory_modifier_plugins/surround_obstacle_stop.cpp
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * Update planning/autoware_minimum_rule_based_planner/schema/minimum_rule_based_planner.schema.json
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * Update planning/autoware_minimum_rule_based_planner/schema/minimum_rule_based_planner.schema.json
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * Apply suggestion from @ktro2828
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * fix incorrect namespace
  * update unit tests
  * add dependency to CMakeLists.txt
  * use ament_target_dependencies instead of target_link_libraries
  * add autoware_point_types to ament_export_dependencies
  ---------
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
* feat(minimum_rule_based_planner): apply agnocast_wrapper::Node to minimum_rule_based_planner (`#13257 <https://github.com/autowarefoundation/autoware_universe/issues/13257>`_)
  * feat: apply agnocast_wrapper::Node to trajectory_processor
  * feat(minimum_rule_based_planner): apply agnocast_wrapper::Node
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(trajectory_modifier, minimum_rule_based_planner): improve obstacle stop feature (`#13255 <https://github.com/autowarefoundation/autoware_universe/issues/13255>`_)
  * fix(trajectory_modifier): fix obstacle stop unstable stop wall (`#3196 <https://github.com/autowarefoundation/autoware_universe/issues/3196>`_)
  * fix stop wall appears behind ego
  * always publish modifier debug markers
  * fix extend_trajectory() function
  - use path curvature at end instead of relying on trajecory point orientations
  - for low end speed trajectory, default to straight extension
  * fix insert_stop_point logic to prevent wrong orientation stop pose
  * introduce minimum stop margin below which duplicate_check_threshold is ignored
  * handle zero stop_point_arc_length in insert_stop_point function
  ---------
  * refactor(obstacle_stop): optimize collision check logic (`#3024 <https://github.com/autowarefoundation/autoware_universe/issues/3024>`_)
  refactor obstacle stop utility function get_nearest_object_collision, update default param values
  * feat(trajectory_modifier): remove max object velocity threshold for obstacle stop (`#3234 <https://github.com/autowarefoundation/autoware_universe/issues/3234>`_)
  * remove max_velocity_th param, fix collision check logic
  * tune rss params
  * apply pre-commit checks
  ---------
  * feat(trajectory_modifier, backup_planner): enable filtering objects by type and shape (`#3255 <https://github.com/autowarefoundation/autoware_universe/issues/3255>`_)
  * enable filtering objects by type and shape
  - update obstacle_stop params in modifier and backup planner to specify enabled types per shape
  - update surround_obstacle_stop params in modifier and backup planner to specify enabled types per shape
  - modify ObjectFilter code in obstacle_stop_utils to support filtering by type and shape
  - modify ObstacleProximityChecker class to support filtering by type and shape
  - fix ObstacleTracker logic for ignoring orientation change
  * Update planning/autoware_trajectory_modifier/include/autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * apply pre-commit checks
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * fix cherry-pick errors
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* feat(trajectory_processor): unify Modifier and Optimizer nodes into Processor node (`#13165 <https://github.com/autowarefoundation/autoware_universe/issues/13165>`_)
* feat(trajectory_processor): unify plugin interface (`#13152 <https://github.com/autowarefoundation/autoware_universe/issues/13152>`_)
* chore(pre-commit): update clang-format to v22.1.5 (`#13126 <https://github.com/autowarefoundation/autoware_universe/issues/13126>`_)
  * chore(pre-commit): update clang-format to v22.1.5
  * style(pre-commit): autofix
  ---------
* chore(minimum_rule_based_planner): add maintainer (`#13017 <https://github.com/autowarefoundation/autoware_universe/issues/13017>`_)
  add maintainer
* feat(trajectory_processor): unify parameters handling (`#12988 <https://github.com/autowarefoundation/autoware_universe/issues/12988>`_)
* feat(trajectory_processor): combine include folders (`#12962 <https://github.com/autowarefoundation/autoware_universe/issues/12962>`_)
* fix(minimum_rule_based_planner): update optimizer/modifier to processor (`#12954 <https://github.com/autowarefoundation/autoware_universe/issues/12954>`_)
* feat(minimum_rule_based_planner): introduce minimum_rule_based_planner package (`#12947 <https://github.com/autowarefoundation/autoware_universe/issues/12947>`_)
  * add minimum_rule_based_planner
  * migration
  * fix cppcheck
  ---------
* Contributors: Kotakku, Maxime CLEMENT, Mete Fatih Cırıt, Ryohsuke Mitsudome, Yutaro Kobayashi, mkquda
