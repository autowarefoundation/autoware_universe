^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory_modifier
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(obstacle_stop): ignore orientation difference for cylinder objects in obstacle tracker (`#12862 <https://github.com/autowarefoundation/autoware_universe/issues/12862>`_)
  * ignore orientation difference for cylinder objects in obstacle tracker
  * add maintainer
  * refactor lambda function get_closest_object_uuid
  ---------
* feat(trajectory_modifier): improve trajectory modifier obstacle stop (`#12651 <https://github.com/autowarefoundation/autoware_universe/issues/12651>`_)
  * feat(trajectory_modifier): improve obstacle stop rss collision check (`#2880 <https://github.com/autowarefoundation/autoware_universe/issues/2880>`_)
  * Improve logic at multiple places
  - fix extend_trajectory() function to account for turning at end
  - fix update_velocities() function to ensure ego can stop at target
  - fix pcd filter bounds calculation
  * apply rss check at multiple trajectory points within time horizon
  - modify rss check to check future states within specified time horizon instead of checking only current state
  - publish debug string
  - specify plugin names in parameter yaml instead of launch file
  - update schema
  * fix get_nearest_object_collision() function
  * fix format with pre-commit
  * update param values
  * use target trajectory velocity for rss check instead of fixed ego velocity
  ---------
  * fix(trajectory_modifier): fix node crash (`#2886 <https://github.com/autowarefoundation/autoware_universe/issues/2886>`_)
  prevent accessing out of range index
  - ensure trajectory has at least 2 points wherever necessary
  - use glog in trajectory modifer launch file
  * feat(trajectory_modifier): improve obstacle stop to better handle crossing objects (`#2931 <https://github.com/autowarefoundation/autoware_universe/issues/2931>`_)
  * filter out temporary crossing objects
  - add function filter_by_target_area() to ObjectFilter struct
  - filter out objects not overlaping with target area
  - for remaining moving objects, check if objects are moving along trajectory direction or not
  - filter out objects that are leaving target area before ego reaches relevant region
  * check for nearby existing stop point before inserting stop point
  * use object lateral velocity componenet instead of direction vector to determine crossing objects
  * parameterize lateral velocity threshold
  * add docstrings to obstacle_stop_utils.hpp
  * improve function filter_by_target_area()
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * feat(trajectory_modifier): add parameterized safety buffer around obstacle footprint (`#2953 <https://github.com/autowarefoundation/autoware_universe/issues/2953>`_)
  add parameterized safety buffer around obstacle footprint
  * refactor(trajectory_modifier): remove unused parameter (`#2965 <https://github.com/autowarefoundation/autoware_universe/issues/2965>`_)
  remove unused parameter from trajectory_modifier parameter struct yaml
  * fix(trajecotry_modifier): make sure planning_factor distance value is not NaN (`#2973 <https://github.com/autowarefoundation/autoware_universe/issues/2973>`_)
  * make sure planning_factor distance value is not NaN
  * remove duplicate function definition
  ---------
  Co-authored-by: Yuxuan Liu <619684051@qq.com>
  * feat(trajectory_modifier): move velocity modification logic to a separate plugin (`#2974 <https://github.com/autowarefoundation/autoware_universe/issues/2974>`_)
  * move velocity modification logic to a separate plugin
  - add new plugin velocity_modifier
  - move velocity update logic from obstacle_stop to velocity_modifier
  - update trajectory_modifier parameters struct
  - update trajectory_modifier param yaml and schema
  - update trajectory_modifier CMakeLists and plugins.xml
  * remove unnecessary comments and includes
  * fix log statements, remove unnecessary member variable
  * add integration test for velocity_modifier plugin
  * update obstacle_stop integration test
  * fix spelling and format
  * remove unused parameter
  * add missing include
  * minor refactor
  ---------
  * fix(obstacle_stop): always process objects/pointcloud (`#2997 <https://github.com/autowarefoundation/autoware_universe/issues/2997>`_)
  * process objects/pointcloud even if input data is empty
  * change arg names in trajectory_modifier launch file
  ---------
  * fix obstacle_stop integration tests
  * fix format, add missing includes
  * feat(trajectory_modifier): improve obstacle stop crossing object assessment (`#3068 <https://github.com/autowarefoundation/autoware_universe/issues/3068>`_)
  * improve time to object calculation to take into account ego front offset, update trajectory modifier readme
  * add obstacle type (dynamic or static) to debug string
  * improve get_predicted_obj_pose_at_time() to extrapolate obj pose from kinematics if there is no predicted path
  * reduce time buffer for crossing object time overlap check
  * update default param values
  * fix obstacle stop integration tests
  ---------
  * guard against division by zero, force last interpolated velocity sample to zero when clamped to s_max
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yuxuan Liu <619684051@qq.com>
* feat(planning): apply autoware_agnocast_wrapper to diffussion planner trajectory pipeline nodes for CIE (`#12779 <https://github.com/autowarefoundation/autoware_universe/issues/12779>`_)
  * feat(autoware_diffusion_planner): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_optimizer): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_adapter): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_ranker): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_selector): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_modifier): apply autoware_agnocast_wrapper for CIE
  ---------
* test(autoware_trajectory_modifier): add integration test for obstacle_stop plugin (`#12568 <https://github.com/autowarefoundation/autoware_universe/issues/12568>`_)
  * test(autoware_trajectory_modifier): add integration test for obstacle_stop plugin
  Add a node-level integration test that exercises the obstacle_stop plugin
  through the trajectory_modifier node interface. The test serves as a
  characterization test capturing the current behavior prior to refactoring.
  * test(autoware_trajectory_modifier): specify expectation
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(autoware_trajectory_modifier): split per-frame inputs from long-lived context (`#12550 <https://github.com/autowarefoundation/autoware_universe/issues/12550>`_)
  * refactor(autoware_trajectory_modifier): pass per-frame inputs as method arguments
  * refactor(autoware_trajectory_modifier): rename FrameInputs to InputData
  Rename the per-frame plugin-input struct and its header file from
  FrameInputs/frame_inputs.hpp to InputData/input_data.hpp, plus the
  associated factory method make_frame_inputs() to make_input_data().
  * refactor(autoware_trajectory_modifier): rename TrajectoryModifierData to TrajectoryModifierContext
  Rename the long-lived plugin-shared struct and its header file from
  TrajectoryModifierData/trajectory_modifier_structs.hpp to
  TrajectoryModifierContext/trajectory_modifier_context.hpp, and rename
  the corresponding member/parameter (data\_/data) to context\_/context.
  * chore(autoware_trajectory_modifier): remove redundant comments
  * test(autoware_trajectory_modifier): declare input variables using helper functions
  * refactor(autoware_trajectory_modifier): rename `inputs` parameter to `input`
  The parameter name `inputs` (plural) was misleading because the type is a
  singular `InputData` struct, not a collection. The `s` suffix conventionally
  suggests a vector/container, so rename to `input` to match the type.
  Test helper struct `DataInputs` and the `apply_inputs` function keep their
  plural form since they intentionally aggregate multiple data streams.
  * chore(autoware_trajectory_modifier): remove unused includes
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* Contributors: Takahisa Ishikawa, atsushi yano, github-actions, mkquda

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(trajectory_modifier): add obstacle stop feature to trajectory modifier (`#12338 <https://github.com/mitsudome-r/autoware_universe/issues/12338>`_)
  * refactor(trajectory_modifier): refactor trajectory_modifier node (`#2697 <https://github.com/mitsudome-r/autoware_universe/issues/2697>`_)
  * refactor trajectory_modifier node
  applies following refactors to the trajectory_modifier node:
  - Use ros2 plugin framework to load submodules
  - Use generate_parameter_library to create parameter struct from schema on build
  - Use ParamListener to handle initial param loading and online param updates
  * remove unnecessary launch-prefix tag
  * modify param schema
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * feat(trajectory_modifier): implement obstacle stop plugin (`#2723 <https://github.com/mitsudome-r/autoware_universe/issues/2723>`_)
  * refactor trajectory_modifier node
  applies following refactors to the trajectory_modifier node:
  - Use ros2 plugin framework to load submodules
  - Use generate_parameter_library to create parameter struct from schema on build
  - Use ParamListener to handle initial param loading and online param updates
  * remove unnecessary launch-prefix tag
  * add obstacle stop plugin framework
  > - add obstacle stop parameters
  > - add obstacle stop plugin
  > - update package.xml, plugins.xml, and CMakeLists.txt
  > - add obstacle stop utils
  > - implement pointcloud filtering and clustering logic
  - implement object filtering logic
  - refactor TrajectoryModifierData struct
  * implement collision check and stop point logic
  - add function to generate trajectory polygon from footprints
  - add function to get nearest pcd collision point
  - add function to get nearest object collision point
  - add logic to track detected collision points for hysterisis
  - add logic to insert stop point
  * fix minimum rule base planner code
  * implement stop point logic
  - add function to calculate stop point index and modify trajectory
  - don't use hysterisis logic (on/off time buffers)
  - check for plugin name string instead of using isClassLoaded()
  * implement smooth stopping logic
  - compute stopping trajectory from jerk and accel limits, assuming highest initial velocity
  - use skima spline trajectory interpolator to generate stopping trajectory
  - use a constant dt to compute s for each interpolated point
  * add debug data and markers
  * use motion_utils::calculate_stop_distance function
  * pre-commit fix
  * don't skip entire trajectory modification for missing pcd or predicted objects, disble stop point setting
  ---------
  * fix(trajectory_modifier): fix node crash (`#2764 <https://github.com/mitsudome-r/autoware_universe/issues/2764>`_)
  check for null ptr
  * feat(trajectory_modifier): improve trajectory modifier stopping behavior (`#2770 <https://github.com/mitsudome-r/autoware_universe/issues/2770>`_)
  * improve logic for generating stopping trajectory
  * log name of plugins that modified trajectory
  ---------
  * fix stop_point_fixer_integration_test
  * feat(trajectory_modifier): add parameter to enable/disable stopping behavior (`#2783 <https://github.com/mitsudome-r/autoware_universe/issues/2783>`_)
  add parameter to enable/disable stopping
  * fix(trajectory_modifier): add missing checks for obstacle stop flags (`#2785 <https://github.com/mitsudome-r/autoware_universe/issues/2785>`_)
  add check for use_objects and use_pointcloud flags
  * refactor(obstacle_stop): refactor stop trajectory generation (`#2788 <https://github.com/mitsudome-r/autoware_universe/issues/2788>`_)
  use set_longitudinal_velocity_interpolator() in trajectory_interpolation_util instead of creating separate velocity spline
  * feat(trajectory_modifier): separate shadow mode obstacle check for pcd and objects (`#2793 <https://github.com/mitsudome-r/autoware_universe/issues/2793>`_)
  separate shadow mode obstacle check for pcd and objects
  - add parameters to enable/disable stopping for pcd and objects separately
  - rename parameters and remove unnecessary unit suffix
  - log obstacle check result for pcd and objects separately
  * feat(trajectory_modifier): add debug data (`#2806 <https://github.com/mitsudome-r/autoware_universe/issues/2806>`_)
  add debug data
  - add processing time debug details
  - add target objects debug markers
  - add target pcd points debug markers
  - refactor pcd clustering function
  * perf(trajectory_modifier): optimize pointcloud processing for obstacle_stop (`#2809 <https://github.com/mitsudome-r/autoware_universe/issues/2809>`_)
  * optimize pointcloud processing
  - add utility struct PointcloudFilter
  - set pcd filtering range based on trajectory bounding box
  - filter out pcd points that belong to a detected object
  - refactor pointcloud filtering logic in modifier obstacle_stop plugin
  - refactor pointcloud filtering logic in rule_based_planner obstacle_stop plugin
  * add safety factors to planning factor
  ---------
  * feat(trajectory_modifier): implement obstacle tracker to get persistent obstacles (`#2810 <https://github.com/mitsudome-r/autoware_universe/issues/2810>`_)
  * implement obstacle tracker to get persistent objects/points
  * add docstring
  * rename PlanningFactor topic name
  * update param schema
  ---------
  * feat(trajectory_modifier): improve obstacle detection (`#2824 <https://github.com/mitsudome-r/autoware_universe/issues/2824>`_)
  * improve obstacle detection
  - Use nominal deceleration distance to get trajectory checking polygon
  - Move target stopping distance to satisfy maximum deceleration limit
  - clear obsolete tracked obstacles before matching new ones
  * add configuration params
  - add obstacle tracking params
  - add yaw diff threshold for object tracking
  - fix braking distance calculation
  * use rclcpp time instead of system clock
  ---------
  * feat(trajectory_modifier): fix obstacle detection near end of trajectory (`#2831 <https://github.com/mitsudome-r/autoware_universe/issues/2831>`_)
  * fix obstacle detection near end of traj
  - ensure detection range includes stop margin beyond traj end
  - refactor get_trajectory_shape function
  - fix arc length computation for obstacles beyond traj end
  * fix stop point setting logic
  * trim trajectory and remove duplicate points
  - diffusion planner trajectory is time based, so it can have duplicate/overlapped points at low velocities
  - trim trajectory after zero velocity point
  - remove overlapped points
  * add comments
  ---------
  * feat(trajectory_modifier): publish processing time (`#2843 <https://github.com/mitsudome-r/autoware_universe/issues/2843>`_)
  publish trajectory_modifier processing_time_ms
  * feat(trajectory_modifier): implement rss check for obstacle stop (`#2853 <https://github.com/mitsudome-r/autoware_universe/issues/2853>`_)
  * implement rss to evaluate safety of detected obstacles
  * improve velocity updating to prevent discontinuity at start
  * minor fixes
  * refactor object filtering
  * fix get_trajectory_shape function
  ---------
  * update trajectory modifier readme, and parameter schema
  * fix shadowed declaration
  * fix build issues
  * specify plugin names to load in the param yaml instead of launch file
  * fix yaml format
  * fix update_velocities() function to ensure stopping is successful
  * set proper bounds for parameters in struct yaml
  * clean up code
  * remove unused code and parameters
  * check for empty vector vefore access
  * fix pcd filter bounds, improve trajectory extension to account for turning at end
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_trajectory_modifier): use wildcard to prevent namespacing issues (`#12269 <https://github.com/mitsudome-r/autoware_universe/issues/12269>`_)
  * use wildcard to prevent namespacing issues
  * set default speed to 0.1 m/s
  * Revert "set default speed to 0.1 m/s"
  This reverts commit 5d8d0b1854f4e79b9b4298195b08e3dd7d840987.
  * fix schema and test to not depend on hardcoded values
  ---------
* feat(trajectory_modifier): modify long stopped trajectories (`#12256 <https://github.com/mitsudome-r/autoware_universe/issues/12256>`_)
  * add modifier flag for long stopped trajectories
  * update param
  * update README
  * update schema and header file
  * reviewers comments, fix time comparison to use nanosec too
  * add edge case test
  * make it so the tests dont depend on hard coded default values
  ---------
* Contributors: danielsanchezaran, github-actions, mkquda

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(trajectory modifier): add trajectory modifier (`#11277 <https://github.com/autowarefoundation/autoware_universe/issues/11277>`_)
  * first commit, add general plugin structure
  * added stop point fixer
  * parameter declaration fixed
  * update README
  * add tests
  * clangd suggestions
  * pre-commit
  * add missing line at end of file
  * fix tests
  * add more maintainers
  * split autoware_utils
  * Fix review comments
  * fix removing sub
  * pre-commit
  ---------
  Co-authored-by: Go Sakayori <gsakayori@gmail.com>
* Contributors: Ryohsuke Mitsudome, danielsanchezaran
