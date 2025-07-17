^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_validator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(planning_validator): add condition to check the yaw deviation (`#10818 <https://github.com/autowarefoundation/autoware_universe/issues/10818>`_)
* feat(planning_validator): implement a collision detection feature for rearward objects of the vehicle (`#10800 <https://github.com/autowarefoundation/autoware_universe/issues/10800>`_)
  * feat(planning_validator): add new flag
  * feat(rear_collision_checker): add new validator plugin
  * fix: current lane extraction range
  * fix: keep previous data when he estimated velocity may be an outlier
  * fix: check reachable distance
  * fix: remove unused param
  * fix: integrate new interface
  * fix: use common marker publisher
  * fix: rename diag
  * fix: parameterize
  * fix: keep checking
  * fix: author
  * fix: remove unused variable
  * chore: vru -> vulnerable_road_user
  * fix: early return
  * fix: cppcheck
  * fix: cppcheck
  * fix: cppcheck
  * fix: clang tidy
  ---------
* fix(planning_validator): fix the lateral distance calculation (`#10801 <https://github.com/autowarefoundation/autoware_universe/issues/10801>`_)
* feat(planning_validator): implement redundant collision prevention feature when ego makes a turn at intersection (`#10750 <https://github.com/autowarefoundation/autoware_universe/issues/10750>`_)
  * create planning latency validator plugin module
  * Revert "chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)"
  This reverts commit 871a8540ade845c7c9a193029d407b411a4d685b.
  * create planning trajectory validator plugin module
  * Update planning/planning_validator/autoware_planning_validator/src/manager.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/node.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * minor fix
  * refactor implementation
  * uncomment lines for adding pose markers
  * fix CMakeLists
  * add comment
  * update planning launch xml
  * Update planning/planning_validator/autoware_planning_latency_validator/include/autoware/planning_latency_validator/latency_validator.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/plugin_interface.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/types.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/src/node.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_latency_validator/src/latency_validator.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * apply pre-commit checks
  * rename plugins for consistency
  * rename directories and files to match package names
  * refactor planning validator tests
  * add packages maintainer
  * separate trajectory check parameters
  * add missing package dependencies
  * move trajectory diagnostics test to trajectory checker module
  * remove blank line
  * add new planning validator plugin collision_checker
  * add launch args for validator modules
  * logic for setting route handler
  * add poincloud filtering function
  * check for turn direction lane
  * compute target lanelets for collision check
  * fix format
  * refactor lanelets filtering
  * update launch files
  * move lanelet selection functions to utils file
  * check for collisions
  * check observation time of tracked object
  * add more config params, fix pcd object function
  * extend target lanes
  * add off timeout after collision is detected
  * define const value
  * improve overlap time estimation
  * Update planning/planning_validator/autoware_planning_validator_collision_checker/src/collision_checker.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * rename planning_validator collision_checker module
  * move pointcloud filtering to just before collision check
  * use proper name for module param
  * fix logic for extending target lanelets
  * change logging level
  ---------
  Co-authored-by: GitHub Action <action@github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* feat(planning_validator): subscribe additional topic for collision detection (`#10745 <https://github.com/autowarefoundation/autoware_universe/issues/10745>`_)
  * feat(planning_validator): subscribe pointcloud
  * feat(planning_validator): subscribe route and map
  * fix(planning_validator): load glog component
  * fix: unexpected test fail
  ---------
* refactor(planning_validator): implement plugin structure for planning validator node (`#10571 <https://github.com/autowarefoundation/autoware_universe/issues/10571>`_)
  * chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)
  not sync github-release
  * create planning latency validator plugin module
  * Revert "chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)"
  This reverts commit 871a8540ade845c7c9a193029d407b411a4d685b.
  * create planning trajectory validator plugin module
  * Update planning/planning_validator/autoware_planning_validator/src/manager.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/node.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * minor fix
  * refactor implementation
  * uncomment lines for adding pose markers
  * fix CMakeLists
  * add comment
  * update planning launch xml
  * Update planning/planning_validator/autoware_planning_latency_validator/include/autoware/planning_latency_validator/latency_validator.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/plugin_interface.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/types.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/src/node.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_latency_validator/src/latency_validator.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * apply pre-commit checks
  * rename plugins for consistency
  * rename directories and files to match package names
  * refactor planning validator tests
  * add packages maintainer
  * separate trajectory check parameters
  * add missing package dependencies
  * move trajectory diagnostics test to trajectory checker module
  * remove blank line
  * add launch args for validator modules
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: GitHub Action <action@github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* Contributors: Maxime CLEMENT, Satoshi OTA, TaikiYamada4, mkquda

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* feat(planning_validator): check for sudden shift in planning trajectory (`#10339 <https://github.com/autowarefoundation/autoware_universe/issues/10339>`_)
  * chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)
  not sync github-release
  * implement function to check for sudden shift in trajectory
  * syntax and format fixes
  * add diagnostic for trajectory shift
  * chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)
  not sync github-release
  * refactor planning validator parameters
  * check enable flag for all validity checks
  * add missing parameters
  * add soft stop feature to planning validator
  * add missing path in planning diagnostic config
  * add debug markers and clean up code
  * Revert "chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)"
  This reverts commit 7badf6e90d0bb1002527c409b62db61cd8b44f37.
  * set trajectory shift values in validation status
  * update planning validator readme
  * update planning validator test
  * run pre-commit checks
  * add missing include
  * add unit test for trajectory shift check
  * properly set is_critical_error\_ flag for all checks
  * Update planning/autoware_planning_validator/include/autoware/planning_validator/parameters.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * revise logic for setting longitudinal shift value
  * fix longitudinal shift check to prevent false positive at end of path
  * improve stop trajectory computation
  * fix spelling
  * fix test files
  * fix node interface tests and pubsub tests
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* feat(planning_validator): add lateral jerk validation feature (`#10440 <https://github.com/autowarefoundation/autoware_universe/issues/10440>`_)
  * feat(planning_validator): add lateral jerk validation and associated parameters
  ---------
* Contributors: Kyoichi Sugahara, TaikiYamada4, mkquda

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* build(autoware_planning_validator): fix missing angles dependency (`#10479 <https://github.com/autowarefoundation/autoware_universe/issues/10479>`_)
* refactor(planning_validator): separate validation check for steering and steering rate (`#10438 <https://github.com/autowarefoundation/autoware_universe/issues/10438>`_)
  * feat(planning_validator): refactor steering validation parameters and add steering_rate check
  * fix(planning_validator): enable validity checks by default and initialize parameters
  * feat(planning_validator): add steering rate validation parameters to README
  * feat(planning_validator): add steering rate validity checks to node options
  ---------
* chore(autoware_planning_validator): add new maintainers to planning_validator (`#10421 <https://github.com/autowarefoundation/autoware_universe/issues/10421>`_)
  (autoware_planning_validator): add new maintainers to package.xml
* fix(planning): apply THROTTLE to frequent log (`#10419 <https://github.com/autowarefoundation/autoware_universe/issues/10419>`_)
* refactor(planning_validator): restructure planning validator configuration (`#10401 <https://github.com/autowarefoundation/autoware_universe/issues/10401>`_)
  * refactor planning validator parameters
  * check enable flag for all validity checks
  * add missing parameters
  * add debug markers and clean up code
  * update planning validator readme
  * update planning validator test
  * properly set is_critical_error\_ flag for all checks
  * Update planning/autoware_planning_validator/include/autoware/planning_validator/parameters.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * run pre-commit checks
  * fix cherry-pick errors
  * remove unnecessary cherry-pick changes
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* feat(planning_validator): improve lateral acc calculation (`#10385 <https://github.com/autowarefoundation/autoware_universe/issues/10385>`_)
  * feat: add functions to calculate interval distance and lateral acceleration
  * refactor: rename array parameters to vector for clarity
  * fix: simplify lateral acceleration calculation using std::hypot
  ---------
* Contributors: Esteve Fernandez, Kyoichi Sugahara, Ryohsuke Mitsudome, Takayuki Murooka, mkquda

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* feat: adaption to ROS nodes guidelines about directory structure (`#10268 <https://github.com/autowarefoundation/autoware_universe/issues/10268>`_)
* feat(planning_validator): add yaw deviation metric (`#10258 <https://github.com/autowarefoundation/autoware_universe/issues/10258>`_)
* feat(planning_validator): add diag to check planning component latency (`#10241 <https://github.com/autowarefoundation/autoware_universe/issues/10241>`_)
  * feat(planning_validator): add diag to check planning component latency
  * fix: relax threshold
  * fix: lacking param
  * fix: relax threshold
  * fix: relax threshold
  * fix: add time stamp
  ---------
* Contributors: Hayato Mizushima, Maxime CLEMENT, NorahXiong, Satoshi OTA, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat(planning_test_manager): abstract message-specific functions (`#9882 <https://github.com/autowarefoundation/autoware_universe/issues/9882>`_)
  * abstract message-specific functions
  * include necessary header
  * adapt velocity_smoother to new test manager
  * adapt behavior_velocity_planner to new test manager
  * adapt path_optimizer to new test manager
  * fix output subscription
  * adapt behavior_path_planner to new test manager
  * adapt scenario_selector to new test manager
  * adapt freespace_planner to new test manager
  * adapt planning_validator to new test manager
  * adapt obstacle_stop_planner to new test manager
  * adapt obstacle_cruise_planner to new test manager
  * disable test for freespace_planner
  * adapt behavior_velocity_crosswalk_module to new test manager
  * adapt behavior_path_lane_change_module to new test manager
  * adapt behavior_path_avoidance_by_lane_change_module to new test manager
  * adapt behavior_path_dynamic_obstacle_avoidance_module to new test manager
  * adapt behavior_path_external_request_lane_change_module to new test manager
  * adapt behavior_path_side_shift_module to new test manager
  * adapt behavior_path_static_obstacle_avoidance_module to new test manager
  * adapt path_smoother to new test manager
  * adapt behavior_velocity_blind_spot_module to new test manager
  * adapt behavior_velocity_detection_area_module to new test manager
  * adapt behavior_velocity_intersection_module to new test manager
  * adapt behavior_velocity_no_stopping_area_module to new test manager
  * adapt behavior_velocity_run_out_module to new test manager
  * adapt behavior_velocity_stop_line_module to new test manager
  * adapt behavior_velocity_traffic_light_module to new test manager
  * adapt behavior_velocity_virtual_traffic_light_module to new test manager
  * adapt behavior_velocity_walkway_module to new test manager
  * adapt motion_velocity_planner_node_universe to new test manager
  * include necessary headers
  * Odometries -> Odometry
  ---------
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
* Contributors: Fumiya Watanabe, Mitsuhiro Sakamoto, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_planning_validator)!: tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_planning_validator (`#9911 <https://github.com/autowarefoundation/autoware_universe/issues/9911>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files planning/autoware_planning_validator
* Contributors: Fumiya Watanabe, Vishal Chauhan

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
* fix(cpplint): include what you use - planning (`#9570 <https://github.com/autowarefoundation/autoware_universe/issues/9570>`_)
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

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
* fix(autoware_planning_validator): fix unusedFunction (`#8646 <https://github.com/autowarefoundation/autoware_universe/issues/8646>`_)
  fix:unusedFunction
* fix(autoware_planning_validator): fix knownConditionTrueFalse (`#7817 <https://github.com/autowarefoundation/autoware_universe/issues/7817>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(planning_validator): rename to include/autoware/{package_name} (`#7514 <https://github.com/autowarefoundation/autoware_universe/issues/7514>`_)
  * feat(planning_validator): rename to include/autoware/{package_name}
  * fix
  ---------
* refactor(test_utils): move to common folder (`#7158 <https://github.com/autowarefoundation/autoware_universe/issues/7158>`_)
  * Move autoware planning test manager to autoware namespace
  * fix package share directory for behavior path planner
  * renaming files and directory
  * rename variables that has planning_test_utils in its name.
  * use autoware namespace for test utils
  * move folder to common
  * update .pages file
  * fix test error
  * removed obstacle velocity limiter test artifact
  * remove namespace from planning validator, it has using keyword
  ---------
* refactor(planning_validator)!: rename directory name  (`#7411 <https://github.com/autowarefoundation/autoware_universe/issues/7411>`_)
  change directory name
* Contributors: Kosuke Takeuchi, Kyoichi Sugahara, Ryuta Kambe, Takayuki Murooka, Yutaka Kondo, Zulfaqar Azmi, kobayu858

0.26.0 (2024-04-03)
-------------------
