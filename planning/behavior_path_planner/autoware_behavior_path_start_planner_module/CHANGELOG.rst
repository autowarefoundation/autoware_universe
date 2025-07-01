^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_path_start_planner_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(behavior_path_planner): organize a part of behavior path info/debug markers (`#10729 <https://github.com/autowarefoundation/autoware_universe/issues/10729>`_)
  * feat(behavior_path_planner): organize a part of behavior path info/debug markers
  * Update planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/src/goal_planner_module.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * Update planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/src/util.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* feat!: remove obstacle_stop_planner and obstacle_cruise_planner (`#10695 <https://github.com/autowarefoundation/autoware_universe/issues/10695>`_)
  * feat: remove obstacle_stop_planner and obstacle_cruise_planner
  * update
  * fix
  ---------
* Contributors: TaikiYamada4, Takayuki Murooka

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* refactor(departure_checker): move lane departure checker class to departure_checker  (`#10337 <https://github.com/autowarefoundation/autoware_universe/issues/10337>`_)
  * RT1-9640: separate lane departure checker library
  * move back parameter
  * separating parameters
  * renamed to boundary departure checker
  * pre-commit
  * remove trajectory deviation
  * rename namespace
  * move boundary departure checker to common folder
  * rename class name
  ---------
* feat(start_planner): add virtual stop detail (`#10573 <https://github.com/autowarefoundation/autoware_universe/issues/10573>`_)
  * feat(start_planner_module): add isInsideLanelets method to check vehicle footprint within lanelets
  ---------
* Contributors: Kyoichi Sugahara, TaikiYamada4, Zulfaqar Azmi

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(start_planner): use waypoints as centerline if available (`#10238 <https://github.com/autowarefoundation/autoware_universe/issues/10238>`_)
  * fix(start_planner): use waypoints as centerline if available
  * update function name
  * rename function name
  ---------
* fix(start_planner): fix segmentation fault when generating backward path (`#10393 <https://github.com/autowarefoundation/autoware_universe/issues/10393>`_)
  * feat(behavior_path_planner): handle empty backward path case
  ---------
* feat(start/goal_planner): use common max steer angle parameter from vehicle_info (`#10321 <https://github.com/autowarefoundation/autoware_universe/issues/10321>`_)
  * fix(autoware_behavior_path_start_planner_module): update parameter name for geometric pull out max steer angle
  * fix(docs): update unit for max_steer_angle_margin_scale in README
  * fix dead link
  ---------
* Contributors: Kyoichi Sugahara, Mehmet Dogru, Ryohsuke Mitsudome

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* feat: adaption to ROS nodes guidelines about directory structure (`#10268 <https://github.com/autowarefoundation/autoware_universe/issues/10268>`_)
* Contributors: Hayato Mizushima, NorahXiong, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat(autoware_vehicle_info_utils): replace autoware_universe_utils with autoware_utils (`#10167 <https://github.com/autowarefoundation/autoware_universe/issues/10167>`_)
* feat!: replace tier4_planning_msgs/PathWithLaneId with autoware_internal_planning_msgs/PathWithLaneId (`#10023 <https://github.com/autowarefoundation/autoware_universe/issues/10023>`_)
* feat(start_planner): output object_of_interest  (`#10053 <https://github.com/autowarefoundation/autoware_universe/issues/10053>`_)
  feat(start_planner_module): integrate safety factor array from collision check
* Contributors: Fumiya Watanabe, Kyoichi Sugahara, Ryohsuke Mitsudome, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(start_planner): visualize planner evaluation table in rviz (`#10029 <https://github.com/autowarefoundation/autoware_universe/issues/10029>`_)
  visualize planner evaluation table in rviz
* fix(start_planner, goal_planner): refactor lane departure checker initialization (`#9944 <https://github.com/autowarefoundation/autoware_universe/issues/9944>`_)
* feat(planning_factor)!: remove velocity_factor, steering_factor and introduce planning_factor (`#9927 <https://github.com/autowarefoundation/autoware_universe/issues/9927>`_)
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: satoshi-ota <satoshi.ota928@gmail.com>
* test(start_planner): disable GenerateValidFreespacePullOutPath test (`#9937 <https://github.com/autowarefoundation/autoware_universe/issues/9937>`_)
* test(autoware_behavior_path_start_planner_module):  add test helper and implement unit tests for FreespacePullOut (`#9832 <https://github.com/autowarefoundation/autoware_universe/issues/9832>`_)
  * refactor(autoware_behavior_path_start_planner_module): remove unnecessary time_keeper parameter from pull-out planners
  * refactor(autoware_behavior_path_start_planner_module): remove TimeKeeper parameter from pull-out planners
  * refactor(lane_departure_checker): improve LaneDepartureChecker initialization and parameter handling
  * refactor(planner): add planner_data parameter to plan methods in pull out planners
  * refactor(autoware_behavior_path_start_planner_module): remove LaneDepartureChecker dependency from pull-out planners
  ---------
* refactor(lane_departure_checker): improve LaneDepartureChecker initialization and parameter handling (`#9791 <https://github.com/autowarefoundation/autoware_universe/issues/9791>`_)
  * refactor(lane_departure_checker): improve LaneDepartureChecker initialization and parameter handling
  ---------
* refactor(autoware_behavior_path_start_planner_module): remove unnecessary time_keeper parameter from pull-out planners (`#9827 <https://github.com/autowarefoundation/autoware_universe/issues/9827>`_)
  * refactor(autoware_behavior_path_start_planner_module): remove unnecessary time_keeper parameter from pull-out planners
  ---------
* fix(behavior_path_planner): add freespace_planning_algorithms dependency (`#9800 <https://github.com/autowarefoundation/autoware_universe/issues/9800>`_)
* test(autoware_behavior_path_start_planner_module): add unit tests for shift shift pull out planner (`#9776 <https://github.com/autowarefoundation/autoware_universe/issues/9776>`_)
  feat(behavior_path_planner): add unit tests for ShiftPullOut path planning
* refactor(autoware_behavior_path_start_planner_module): add data_structs.cpp and init method for StartPlannerParameters (`#9736 <https://github.com/autowarefoundation/autoware_universe/issues/9736>`_)
  feat(autoware_behavior_path_start_planner_module): add data_structs.cpp and init method for StartPlannerParameters
* test(autoware_behavior_path_start_planner_module): add unit tests for geometric shift pull out planner (`#9640 <https://github.com/autowarefoundation/autoware_universe/issues/9640>`_)
  * feat(behavior_path_planner): add unit tests for geometric pull-out planner and improve collision check
  * feat(behavior_path_planner): add boolean parameter for divide_pull_out_path and update tests
  ---------
* Contributors: Fumiya Watanabe, Kyoichi Sugahara, Mamoru Sobue, Takayuki Murooka

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* feat(behavior_path_planner): add detail text to virutal wall (`#9600 <https://github.com/autowarefoundation/autoware_universe/issues/9600>`_)
  * feat(behavior_path_planner): add detail text to virutal wall
  * goal is far
  * pull over start pose is far
  * fix lc build
  * fix build
  * Update planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/src/goal_planner_module.cpp
  ---------
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* fix(cpplint): include what you use - planning (`#9570 <https://github.com/autowarefoundation/autoware_universe/issues/9570>`_)
* fix(autoware_freespace_planner, autoware_freespace_planning_algorithms): modify freespace planner to use node clock instead of system clock (`#9152 <https://github.com/autowarefoundation/autoware_universe/issues/9152>`_)
  * Modified the autoware_freespace_planner and autoware_freespace_planning_algorithms packages to use the node clock instead of rclcpp detached clock. This allows the module to make use of sim time. Previously during simulation the parking trajectory would have system time in trajectory header messages causing downstream issues like non-clearance of trajectory buffers in motion planning based on elapsed time.
  * style(pre-commit): autofix
  * Updated the freespace planner instantiation call in the path planning modules
  * style(pre-commit): autofix
  * Updated tests for the utility functions
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Steven Brills <sbrills@oshkoshcorp.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(start_planner): use extended current lanes to fix turn signal issue (`#9487 <https://github.com/autowarefoundation/autoware_universe/issues/9487>`_)
  fix current lanes issue
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix(autoware_behavior_path_start_planner_module): fix clang-diagnostic-unused-variable (`#9405 <https://github.com/autowarefoundation/autoware_universe/issues/9405>`_)
  fix: clang-diagnostic-unused-variable
* feat(start_planner): output velocity factor (`#9347 <https://github.com/autowarefoundation/autoware_universe/issues/9347>`_)
* refactor(bpp): rework steering factor interface (`#9325 <https://github.com/autowarefoundation/autoware_universe/issues/9325>`_)
  * refactor(bpp): rework steering factor interface
  * refactor(soa): rework steering factor interface
  * refactor(AbLC): rework steering factor interface
  * refactor(doa): rework steering factor interface
  * refactor(lc): rework steering factor interface
  * refactor(gp): rework steering factor interface
  * refactor(sp): rework steering factor interface
  * refactor(sbp): rework steering factor interface
  * refactor(ss): rework steering factor interface
  ---------
* feat(start_planner, lane_departure_checker): speed up by updating polygons (`#9309 <https://github.com/autowarefoundation/autoware_universe/issues/9309>`_)
  speed up by updating polygons
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_behavior_path_start_planner_module): fix cppcheck unreadVariable (`#9277 <https://github.com/autowarefoundation/autoware_universe/issues/9277>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kosuke Takeuchi, M. Fatih Cırıt, Ryohsuke Mitsudome, Ryuta Kambe, Satoshi OTA, Yutaka Kondo, danielsanchezaran, kobayu858, stevenbrills

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(start_planner, lane_departure_checker): speed up by updating polygons (`#9309 <https://github.com/autowarefoundation/autoware_universe/issues/9309>`_)
  speed up by updating polygons
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_behavior_path_start_planner_module): fix cppcheck unreadVariable (`#9277 <https://github.com/autowarefoundation/autoware_universe/issues/9277>`_)
* Contributors: Esteve Fernandez, Ryuta Kambe, Yutaka Kondo, danielsanchezaran

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* feat(start_planner): update param to match launch (`#9158 <https://github.com/autowarefoundation/autoware_universe/issues/9158>`_)
  update param to match launch
* refactor(bpp_common, motion_utils): move path shifter util functions to autoware::motion_utils (`#9081 <https://github.com/autowarefoundation/autoware_universe/issues/9081>`_)
  * remove unused function
  * mover path shifter utils function to autoware motion utils
  * minor change in license header
  * fix warning message
  * remove header file
  ---------
* fix(behavior_path_planner_common): swap boolean for filterObjectsByVelocity (`#9036 <https://github.com/autowarefoundation/autoware_universe/issues/9036>`_)
  fix filter object by velocity
* refactor(bpp): simplify ExtendedPredictedObject and add new member variables (`#8889 <https://github.com/autowarefoundation/autoware_universe/issues/8889>`_)
  * simplify ExtendedPredictedObject and add new member variables
  * replace self polygon to initial polygon
  * comment
  * add comments to dist of ego
  ---------
* refactor(start_planner,raw_vechile_cmd_converter): align parameter with autoware_launch's parameter (`#8913 <https://github.com/autowarefoundation/autoware_universe/issues/8913>`_)
  * align autoware_raw_vehicle_cmd_converter's parameter
  * align start_planner's parameter
  ---------
* feat(start_planner): add skip_rear_vehicle_check parameter (`#8863 <https://github.com/autowarefoundation/autoware_universe/issues/8863>`_)
  Add the skip_rear_vehicle_check parameter to the start planner module configuration. This parameter allows disabling the rear vehicle check during collision detection. By default, the rear vehicle check is enabled.
* fix(autoware_behavior_path_planner): align the parameters with launcher (`#8790 <https://github.com/autowarefoundation/autoware_universe/issues/8790>`_)
  parameters in behavior_path_planner aligned
* fix(autoware_behavior_path_start_planner_module): fix unusedFunction (`#8709 <https://github.com/autowarefoundation/autoware_universe/issues/8709>`_)
  * fix:checkCollisionBetweenPathFootprintsAndObjects
  * fix:add const
  * fix:unusedFunction
  ---------
* fix(bpp): use common steering factor interface for same scene modules (`#8675 <https://github.com/autowarefoundation/autoware_universe/issues/8675>`_)
* refactor(start_planner, lane_departure_checker): remove redundant calculation in fuseLaneletPolygon (`#8682 <https://github.com/autowarefoundation/autoware_universe/issues/8682>`_)
  * remove redundant fused lanelet calculation
  * remove unnecessary change
  * add new function
  * fix spelling mistake
  * fix spelling mistake
  * use std::move and lambda funcion for better code
  * add comment for better understanding
  * fix cppcheck
  ---------
* fix(autoware_behavior_path_start_planner_module): fix unusedFunction (`#8659 <https://github.com/autowarefoundation/autoware_universe/issues/8659>`_)
  fix:unusedFunction
* refactor(start_planner): remove redundant calculation in shift pull out  (`#8623 <https://github.com/autowarefoundation/autoware_universe/issues/8623>`_)
  * fix redundant calculation
  * fix unneccesary modification for comment
  ---------
* feat(freespace_planning_algorithms): implement option for backward search from goal to start (`#8091 <https://github.com/autowarefoundation/autoware_universe/issues/8091>`_)
  * refactor freespace planning algorithms
  * fix error
  * use vector instead of map for a-star node graph
  * remove unnecessary parameters
  * precompute average turning radius
  * add threshold for minimum distance between direction changes
  * apply curvature weight and change in curvature weight
  * store total cost instead of heuristic cost
  * fix reverse weight application
  * fix parameter description in README
  * implement edt map to store distance to nearest obstacle for each grid cell
  * use obstacle edt in collision check
  * add cost for distance to obstacle
  * fix formats
  * add missing include
  * refactor functions
  * add missing include
  * implement backward search option
  * precompute number of margin cells to reduce out of range vertices check necessity
  * add reset data function
  * remove unnecessary code
  * add member function set() to AstarNode struct
  * implement adaptive expansion distance
  * remove unnecessary code
  * interpolate nodes with large expansion distance
  * minor refactor
  * fix interpolation for backward search
  * ensure expansion distance is larger than grid cell diagonal
  * compute collision free distance to goal map
  * use obstacle edt when computing collision free distance map
  * minor refactor
  * fix expansion cost function
  * set distance map before setting start node
  * refactor detect collision function
  * use flag instead of enum
  * add missing variable initialization
  * remove declared but undefined function
  * refactor makePlan() function
  * remove bool return statement for void function
  * remove unnecessary checks
  * minor fix
  * refactor computeEDTMap function
  * remove unnecessary code
  * set min and max expansion distance after setting costmap
  * refactor detectCollision function
  * remove unused function
  * change default parameter values
  * add missing last waypoint
  * fix computeEDTMap function
  * rename parameter
  * use linear function for obstacle distance cost
  * fix rrtstar obstacle check
  * add public access function to get distance to nearest obstacle
  * remove redundant return statements
  * check goal pose validity before setting collision free distance map
  * declare variables as const where necessary
  * compare front and back lengths when setting min and max dimension
  * add docstring and citation for computeEDTMap function
  * transform pose to local frame in getDistanceToObstacle funcion
  * update freespace planner parameter schema
  * refactor setPath function
  * fix function setPath
  * minor refactor
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* feat(start_planner): add time_keeper (`#8254 <https://github.com/autowarefoundation/autoware_universe/issues/8254>`_)
  * feat(start_planner): add time_keeper
  * fix
  * fix
  * fix shadow variables
  ---------
* fix(start/goal_planner): fix freespace planning error handling (`#8246 <https://github.com/autowarefoundation/autoware_universe/issues/8246>`_)
* refactor(freespace_planning_algorithm): refactor and improve astar search (`#8068 <https://github.com/autowarefoundation/autoware_universe/issues/8068>`_)
  * refactor freespace planning algorithms
  * fix error
  * use vector instead of map for a-star node graph
  * remove unnecessary parameters
  * precompute average turning radius
  * add threshold for minimum distance between direction changes
  * apply curvature weight and change in curvature weight
  * store total cost instead of heuristic cost
  * fix reverse weight application
  * fix parameter description in README
  * fix formats
  * add missing include
  * refactor functions
  * precompute number of margin cells to reduce out of range vertices check necessity
  * add reset data function
  * add member function set() to AstarNode struct
  * remove unnecessary code
  * minor refactor
  * ensure expansion distance is larger than grid cell diagonal
  * compute collision free distance to goal map
  * minor refactor
  * fix expansion cost function
  * set distance map before setting start node
  * minor fix
  * remove unnecessary code
  * change default parameter values
  * rename parameter
  * fix rrtstar obstacle check
  * remove redundant return statements
  * check goal pose validity before setting collision free distance map
  * declare variables as const where necessary
  ---------
* fix(autoware_behavior_path_start_planner_module): fix shadowVariable (`#7982 <https://github.com/autowarefoundation/autoware_universe/issues/7982>`_)
  * fix:shadowVariable
  * fix:shadowVariable
  * refactor:clang format
  * refactor:clang format
  * refactor:clang format
  * refactor: change of declaration location
  * fix:shadowVariable
  * fix:shadowVariable
  * fix:shadowVariable
  * refactor:clang format
  * refactor: namespace
  * refactor:clang format
  ---------
* feat(start_planner): add end_pose_curvature_threshold  (`#7901 <https://github.com/autowarefoundation/autoware_universe/issues/7901>`_)
  * feat(start_planner): add end_pose_curvature_threshold
  * Update planning/behavior_path_planner/autoware_behavior_path_start_planner_module/README.md
  Co-authored-by: Kyoichi Sugahara <kyoichi.sugahara@tier4.jp>
  * update max curvature discription
  * update readme
  ---------
  Co-authored-by: Kyoichi Sugahara <kyoichi.sugahara@tier4.jp>
* feat(start_planner): check current_pose and estimated_stop_pose for isPreventingRearVehicleFromPassingThrough (`#8112 <https://github.com/autowarefoundation/autoware_universe/issues/8112>`_)
* fix(start/goal_planner): fix addition of duplicate segments in calcBeforeShiftedArcLength (`#7902 <https://github.com/autowarefoundation/autoware_universe/issues/7902>`_)
  * fix(start/goal_planner): fix addition of duplicate segments in calcBeforeShiftedArcLength
  * Update trajectory.hpp
  Co-authored-by: Kyoichi Sugahara <kyoichi.sugahara@tier4.jp>
  * Update trajectory.hpp
  Co-authored-by: Kyoichi Sugahara <kyoichi.sugahara@tier4.jp>
  ---------
  Co-authored-by: Kyoichi Sugahara <kyoichi.sugahara@tier4.jp>
* feat(safety_check): filter safety check targe objects by yaw deviation between pose and lane (`#7828 <https://github.com/autowarefoundation/autoware_universe/issues/7828>`_)
  * fix(safety_check): filter by yaw deviation to check object belongs to lane
  * fix(static_obstacle_avoidance): check yaw only when the object is moving
  ---------
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware_universe/issues/7640>`_)
* feat(start_planner): yaw threshold for rss check (`#7657 <https://github.com/autowarefoundation/autoware_universe/issues/7657>`_)
  * add param to customize yaw th
  * add param to other modules
  * docs
  * update READMEs with params
  * fix LC README
  * use normalized yaw diff
  ---------
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* fix(autoware_behavior_path_start_planner_module): fix duplicateBreak warning (`#7583 <https://github.com/autowarefoundation/autoware_universe/issues/7583>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(route_handler)!: rename to include/autoware/{package_name}  (`#7530 <https://github.com/autowarefoundation/autoware_universe/issues/7530>`_)
  refactor(route_handler)!: rename to include/autoware/{package_name}
* refactor(freespace_planner)!: rename to include/autoware/{package_name}  (`#7525 <https://github.com/autowarefoundation/autoware_universe/issues/7525>`_)
  refactor(freespace_planner)!: rename to include/autoware/{package_name}
  refactor(start_planner): make autoware include dir
  refactor(goal_planner): make autoware include dir
  sampling planner module
  fix sampling planner build
  dynamic_avoidance
  lc
  side shift
  autoware_behavior_path_static_obstacle_avoidance_module
  autoware_behavior_path_planner_common
  make behavior_path dir
  pre-commit
  fix pre-commit
  fix build
  autoware_freespace_planner
  freespace_planning_algorithms
* refactor(control)!: refactor directory structures of the control checkers (`#7524 <https://github.com/autowarefoundation/autoware_universe/issues/7524>`_)
  * aeb
  * control_validator
  * lane_departure_checker
  * shift_decider
  * fix
  ---------
* refactor(behaivor_path_planner)!: rename to include/autoware/{package_name} (`#7522 <https://github.com/autowarefoundation/autoware_universe/issues/7522>`_)
  * refactor(behavior_path_planner)!: make autoware dir in include
  * refactor(start_planner): make autoware include dir
  * refactor(goal_planner): make autoware include dir
  * sampling planner module
  * fix sampling planner build
  * dynamic_avoidance
  * lc
  * side shift
  * autoware_behavior_path_static_obstacle_avoidance_module
  * autoware_behavior_path_planner_common
  * make behavior_path dir
  * pre-commit
  * fix pre-commit
  * fix build
  ---------
* Contributors: Go Sakayori, Kosuke Takeuchi, Kyoichi Sugahara, Ryuta Kambe, Satoshi OTA, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, Zhe Shen, Zulfaqar Azmi, danielsanchezaran, kobayu858, mkquda

0.26.0 (2024-04-03)
-------------------
