^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_validator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
