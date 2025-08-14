^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_path_optimizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): autofix (`#10982 <https://github.com/autowarefoundation/autoware_universe/issues/10982>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* fix(path_optimizer): undefined behavior when optimizer failure (`#10726 <https://github.com/autowarefoundation/autoware_universe/issues/10726>`_)
  * change AND condition to OR
  * fix condition again
  Co-authored-by: Arjun Jagdish Ram <arjun.ram@tier4.jp>
  * fix build error
  * revert is_optimization_failed\_ condition
  * minor refactoring (`#1 <https://github.com/autowarefoundation/autoware_universe/issues/1>`_)
  ---------
  Co-authored-by: Arjun Jagdish Ram <arjun.ram@tier4.jp>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
* Contributors: Kento Yabuuchi, TaikiYamada4

0.45.0 (2025-05-22)
-------------------

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_path_optimizer): created the schema file,updated the readme file (`#10123 <https://github.com/autowarefoundation/autoware_universe/issues/10123>`_)
* build(autoware_path_optimizer): fix missing diagnostic_updater dependency (`#10478 <https://github.com/autowarefoundation/autoware_universe/issues/10478>`_)
* fix(autoware_path_optimizer): incorrect application of input velocity due to badly mapping output trajectory to input trajectory (`#10403 <https://github.com/autowarefoundation/autoware_universe/issues/10403>`_)
  * Changes for proper input velocity application
  * fix
  ---------
* Contributors: Arjun Jagdish Ram, Esteve Fernandez, Ryohsuke Mitsudome, Vishal Chauhan

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* fix(autoware_path_optimizer): hotfix for wrong logic triggering MRM on start in 3 seconds (`#10305 <https://github.com/autowarefoundation/autoware_universe/issues/10305>`_)
  fix
* feat(path_optimizer): additional failure logging and failure mode handling (`#10276 <https://github.com/autowarefoundation/autoware_universe/issues/10276>`_)
  MRM when MPT fails
* feat: apply splitting of autoware_utils_geometry  (`#10270 <https://github.com/autowarefoundation/autoware_universe/issues/10270>`_)
  * fix build error
  * merge namespace
  ---------
* feat: adaption to ROS nodes guidelines about directory structure (`#10268 <https://github.com/autowarefoundation/autoware_universe/issues/10268>`_)
* fix(path_optimizer): remove unnecesary optional (`#10181 <https://github.com/autowarefoundation/autoware_universe/issues/10181>`_)
* fix(planning): add missing exec_depend (`#10134 <https://github.com/autowarefoundation/autoware_universe/issues/10134>`_)
  * fix(planning): add missing exec_depend
  * fix find-pkg-share
  * fix find-pkg-share
  ---------
* Contributors: Arjun Jagdish Ram, Hayato Mizushima, NorahXiong, Takagi, Isamu, Takayuki Murooka, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat: remove dependency on autoware_universe_utils from autoware_interpolation and autoware_motion_utils (`#10092 <https://github.com/autowarefoundation/autoware_universe/issues/10092>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat!: replace tier4_planning_msgs/PathWithLaneId with autoware_internal_planning_msgs/PathWithLaneId (`#10023 <https://github.com/autowarefoundation/autoware_universe/issues/10023>`_)
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
* fix(path_optimizer): fix possible zero division (`#10022 <https://github.com/autowarefoundation/autoware_universe/issues/10022>`_)
* Contributors: Fumiya Watanabe, Mamoru Sobue, Mitsuhiro Sakamoto, Ryohsuke Mitsudome, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_path_optimizer)!: tier4_debug_msgs changed to autoware-internal_debug_msgs in autoware_path_optimizer (`#9907 <https://github.com/autowarefoundation/autoware_universe/issues/9907>`_)
  * feat: tier4_debug_msgs changed to autoware-internal_debug_msgs in files planning/autoware_path_optimizer
  * Update planning/autoware_path_optimizer/package.xml
  ---------
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* feat(motion_planning): use StringStamped in autoware_internal_debug_msgs (`#9742 <https://github.com/autowarefoundation/autoware_universe/issues/9742>`_)
  feat(motion planning): use StringStamped in autoware_internal_debug_msgs
* Contributors: Fumiya Watanabe, Takayuki Murooka, Vishal Chauhan

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
* refactor(global_parameter_loader): prefix package and namespace with autoware (`#9303 <https://github.com/autowarefoundation/autoware_universe/issues/9303>`_)
* docs: update the list styles (`#9555 <https://github.com/autowarefoundation/autoware_universe/issues/9555>`_)
* fix(path_optimizer): solve issue with time keeper (`#9483 <https://github.com/autowarefoundation/autoware_universe/issues/9483>`_)
  solve issue with time keeper
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* refactor(fake_test_node): prefix package and namespace with autoware (`#9249 <https://github.com/autowarefoundation/autoware_universe/issues/9249>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo, danielsanchezaran

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
* refactor(osqp_interface): added autoware prefix to osqp_interface (`#8958 <https://github.com/autowarefoundation/autoware_universe/issues/8958>`_)
* chore(path_optimizer): add warn msg for exceptional behavior (`#9033 <https://github.com/autowarefoundation/autoware_universe/issues/9033>`_)
* refactor(autoware_interpolation): prefix package and namespace with autoware (`#8088 <https://github.com/autowarefoundation/autoware_universe/issues/8088>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* fix(motion_planning): align the parameters with launcher (`#8792 <https://github.com/autowarefoundation/autoware_universe/issues/8792>`_)
  parameters in motion_planning aligned
* fix(autoware_path_optimizer): fix unusedFunction (`#8644 <https://github.com/autowarefoundation/autoware_universe/issues/8644>`_)
  fix:unusedFunction
* fix(autoware_path_optimizer): fix unreadVariable (`#8361 <https://github.com/autowarefoundation/autoware_universe/issues/8361>`_)
  * fix:unreadVariable
  * fix:unreadVariable
  ---------
* fix(autoware_path_optimizer): fix passedByValue (`#8190 <https://github.com/autowarefoundation/autoware_universe/issues/8190>`_)
  fix:passedByValue
* fix(path_optimizer): revert the feature of publishing processing time (`#8160 <https://github.com/autowarefoundation/autoware_universe/issues/8160>`_)
* feat(autoware_universe_utils): add TimeKeeper to track function's processing time (`#7754 <https://github.com/autowarefoundation/autoware_universe/issues/7754>`_)
* fix(autoware_path_optimizer): fix redundantContinue warnings (`#7577 <https://github.com/autowarefoundation/autoware_universe/issues/7577>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(path_optimizer): rename to include/autoware/{package_name} (`#7529 <https://github.com/autowarefoundation/autoware_universe/issues/7529>`_)
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
* refactor(vehicle_info_utils)!: prefix package and namespace with autoware (`#7353 <https://github.com/autowarefoundation/autoware_universe/issues/7353>`_)
  * chore(autoware_vehicle_info_utils): rename header
  * chore(bpp-common): vehicle info
  * chore(path_optimizer): vehicle info
  * chore(velocity_smoother): vehicle info
  * chore(bvp-common): vehicle info
  * chore(static_centerline_generator): vehicle info
  * chore(obstacle_cruise_planner): vehicle info
  * chore(obstacle_velocity_limiter): vehicle info
  * chore(mission_planner): vehicle info
  * chore(obstacle_stop_planner): vehicle info
  * chore(planning_validator): vehicle info
  * chore(surround_obstacle_checker): vehicle info
  * chore(goal_planner): vehicle info
  * chore(start_planner): vehicle info
  * chore(control_performance_analysis): vehicle info
  * chore(lane_departure_checker): vehicle info
  * chore(predicted_path_checker): vehicle info
  * chore(vehicle_cmd_gate): vehicle info
  * chore(obstacle_collision_checker): vehicle info
  * chore(operation_mode_transition_manager): vehicle info
  * chore(mpc): vehicle info
  * chore(control): vehicle info
  * chore(common): vehicle info
  * chore(perception): vehicle info
  * chore(evaluator): vehicle info
  * chore(freespace): vehicle info
  * chore(planning): vehicle info
  * chore(vehicle): vehicle info
  * chore(simulator): vehicle info
  * chore(launch): vehicle info
  * chore(system): vehicle info
  * chore(sensing): vehicle info
  * fix(autoware_joy_controller): remove unused deps
  ---------
* refactor(path_optimizer, velocity_smoother)!: prefix package and namespace with autoware (`#7354 <https://github.com/autowarefoundation/autoware_universe/issues/7354>`_)
  * chore(autoware_velocity_smoother): update namespace
  * chore(autoware_path_optimizer): update namespace
  ---------
* feat!: replace autoware_auto_msgs with autoware_msgs for planning modules (`#7246 <https://github.com/autowarefoundation/autoware_universe/issues/7246>`_)
  Co-authored-by: Cynthia Liu <cynthia.liu@autocore.ai>
  Co-authored-by: NorahXiong <norah.xiong@autocore.ai>
  Co-authored-by: beginningfan <beginning.fan@autocore.ai>
* chore(autoware_velocity_smoother, autoware_path_optimizer): rename packages (`#7202 <https://github.com/autowarefoundation/autoware_universe/issues/7202>`_)
  * chore(autoware_path_optimizer): rename package and namespace
  * chore(autoware_static_centerline_generator): rename package and namespace
  * chore: update module name
  * chore(autoware_velocity_smoother): rename package and namespace
  * chore(tier4_planning_launch): update module name
  * chore: update module name
  * fix: test
  * fix: test
  * fix: test
  ---------
* Contributors: Esteve Fernandez, Kosuke Takeuchi, Ryohsuke Mitsudome, Ryuta Kambe, Satoshi OTA, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, Zhe Shen, Zulfaqar Azmi, kobayu858

0.26.0 (2024-04-03)
-------------------
