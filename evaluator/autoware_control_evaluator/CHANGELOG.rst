^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_control_evaluator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat: change planning output topic name to /planning/trajectory (`#11135 <https://github.com/autowarefoundation/autoware_universe/issues/11135>`_)
  * change planning output topic name to /planning/trajectory
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(control_evaluator): add option to output_metrics only when ego is moving (`#11125 <https://github.com/autowarefoundation/autoware_universe/issues/11125>`_)
  * add output_metrics_only_moving
  * pre-commit
  * use emplace_back
  * fix build failures
  ---------
* fix(control_evaluator): publish dist to uncrossable boundary (`#11103 <https://github.com/autowarefoundation/autoware_universe/issues/11103>`_)
  fix(control_evaluator): Publish dist to uncrossable boundary
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(control_evaluator): add steering_angle_abs metric (`#10991 <https://github.com/autowarefoundation/autoware_universe/issues/10991>`_)
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
* feat(control_evaluator): boundary departure check in control evaluator (`#10859 <https://github.com/autowarefoundation/autoware_universe/issues/10859>`_)
  * feat(control_evaluator): add boundary departure check
  * slight refactoring
  * fix node dying due to invalid linestring reference
  * docstring fix
  * Revise  metric's messages and move function call
  * add comments to the extra margin
  ---------
* Contributors: Kazunori-Nakajima, Kem (TiankuiXian), Mete Fatih Cırıt, Yukihiro Saito, Zulfaqar Azmi

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(control_evaluator): change metrics to non-absolute and add absolute metrics (`#10820 <https://github.com/autowarefoundation/autoware_universe/issues/10820>`_)
  change metric name to _abs, and add no-abs metrics
* feat(autoware_control_evaluator): add closest_object_distance metric (`#10803 <https://github.com/autowarefoundation/autoware_universe/issues/10803>`_)
  * add closest_object_distance
  * pre-commit
  * add metric to list
  * pre-commit
  * fix TIER IV and autoware_utils_geometry
  * add distance filter
  * refactor code
  * fix miss
  ---------
* chore(control_evaluator): refactor control evaluator's logics. (`#10741 <https://github.com/autowarefoundation/autoware_universe/issues/10741>`_)
  * fix conflict
  * fix bug
  ---------
* fix(control_evaluator): add check route handler readiness (`#10719 <https://github.com/autowarefoundation/autoware_universe/issues/10719>`_)
  Co-authored-by: Jacek Namiota <jnamiota@autonomous-systems.pl>
* Contributors: Kem (TiankuiXian), TaikiYamada4, namiota

0.45.0 (2025-05-22)
-------------------

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(control_evaluator): fix bugs about output_metrics (`#10433 <https://github.com/autowarefoundation/autoware_universe/issues/10433>`_)
  * add val,acc,jark to metrics
  * accumulate goal metrics only when stop at goal
  * fix bug of stop-related metrics
  * fix unit test.
  ---------
* Contributors: Kem (TiankuiXian), Ryohsuke Mitsudome

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* feat(control_evaluator): add a new stop_deviation metric (`#10246 <https://github.com/autowarefoundation/autoware_universe/issues/10246>`_)
  * add metric of stop_deviation
  * fix bug
  * remove unused include.
  * add unit test and schema
  * pre-commit
  * update planning_evaluator schema
  ---------
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
* Contributors: Hayato Mizushima, Kem (TiankuiXian), Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat(autoware_vehicle_info_utils): replace autoware_universe_utils with autoware_utils (`#10167 <https://github.com/autowarefoundation/autoware_universe/issues/10167>`_)
* feat!: replace tier4_planning_msgs/PathWithLaneId with autoware_internal_planning_msgs/PathWithLaneId (`#10023 <https://github.com/autowarefoundation/autoware_universe/issues/10023>`_)
* Contributors: Fumiya Watanabe, Ryohsuke Mitsudome, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_control_evaluator): add new steering metrics (`#10012 <https://github.com/autowarefoundation/autoware_universe/issues/10012>`_)
* feat(autoware_control_evaluator): add new boundary_distance metrics (`#9984 <https://github.com/autowarefoundation/autoware_universe/issues/9984>`_)
  * add boundary_distance metric
  * pre-commit
  * use path topic instead of lanenet.
  * remove unused import
  * apply is_point_left_of_line
  * fix typo
  * fix test bug
  * manual pre-commit
  ---------
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
* feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in fil… (`#9858 <https://github.com/autowarefoundation/autoware_universe/issues/9858>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files evaluator/autoware_control_evaluator
* fix(autoware_control_evaluator): fix bugprone-exception-escape (`#9630 <https://github.com/autowarefoundation/autoware_universe/issues/9630>`_)
  * fix: bugprone-exception-escape
  * fix: cpplint
  ---------
* Contributors: Fumiya Watanabe, Kem (TiankuiXian), Vishal Chauhan, kobayu858

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
* fix(cpplint): include what you use - evaluator (`#9566 <https://github.com/autowarefoundation/autoware_universe/issues/9566>`_)
* fix(control_evaluator): correct goal_lateal_deviation (`#9532 <https://github.com/autowarefoundation/autoware_universe/issues/9532>`_)
* feat(control_evaluator, tier4_control_launch): add a trigger to choice whether to output metrics to log folder (`#9478 <https://github.com/autowarefoundation/autoware_universe/issues/9478>`_)
  * refactor and add output_metrics. a bug existing when psim.
  * refactored launch file.
  * output description
  * add parm to launch file.
  * move output_metrics from param config to launch file.
  * move output_metrics from config to launch.xml
  * fix unit test bug.
  * fix test bug again.
  * Update evaluator/autoware_control_evaluator/include/autoware/control_evaluator/control_evaluator_node.hpp
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(control_evaluator): add processing time publisher (`#9339 <https://github.com/autowarefoundation/autoware_universe/issues/9339>`_)
* test(autoware_control_evaluator): add unit test for utils autoware_control_evaluator (`#9307 <https://github.com/autowarefoundation/autoware_universe/issues/9307>`_)
  * update unit test of control_evaluator.
  * manual pre-commit.
  ---------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(tier4_metric_msgs): apply tier4_metric_msgs for scenario_simulator_v2_adapter, control_evaluator, planning_evaluator, autonomous_emergency_braking, obstacle_cruise_planner, motion_velocity_planner, processing_time_checker (`#9180 <https://github.com/autowarefoundation/autoware_universe/issues/9180>`_)
  * first commit
  * fix building errs.
  * change diagnostic messages to metric messages for publishing decision.
  * fix bug about motion_velocity_planner
  * change the diagnostic msg to metric msg in autoware_obstacle_cruise_planner.
  * tmp save for planning_evaluator
  * change the topic to which metrics published to.
  * fix typo.
  * remove unnesessary publishing of metrics.
  * mke planning_evaluator publish msg of MetricArray instead of Diags.
  * update aeb with metric type for decision.
  * fix some bug
  * remove autoware_evaluator_utils package.
  * remove diagnostic_msgs dependency of planning_evaluator
  * use metric_msgs for autoware_processing_time_checker.
  * rewrite diagnostic_convertor to scenario_simulator_v2_adapter, supporting metric_msgs.
  * pre-commit and fix typo
  * publish metrics even if there is no metric in the MetricArray.
  * modify the metric name of processing_time.
  * update unit test for test_planning/control_evaluator
  * manual pre-commit
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kazunori-Nakajima, Kem (TiankuiXian), M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* test(autoware_control_evaluator): add unit test for utils autoware_control_evaluator (`#9307 <https://github.com/autowarefoundation/autoware_universe/issues/9307>`_)
  * update unit test of control_evaluator.
  * manual pre-commit.
  ---------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(tier4_metric_msgs): apply tier4_metric_msgs for scenario_simulator_v2_adapter, control_evaluator, planning_evaluator, autonomous_emergency_braking, obstacle_cruise_planner, motion_velocity_planner, processing_time_checker (`#9180 <https://github.com/autowarefoundation/autoware_universe/issues/9180>`_)
  * first commit
  * fix building errs.
  * change diagnostic messages to metric messages for publishing decision.
  * fix bug about motion_velocity_planner
  * change the diagnostic msg to metric msg in autoware_obstacle_cruise_planner.
  * tmp save for planning_evaluator
  * change the topic to which metrics published to.
  * fix typo.
  * remove unnesessary publishing of metrics.
  * mke planning_evaluator publish msg of MetricArray instead of Diags.
  * update aeb with metric type for decision.
  * fix some bug
  * remove autoware_evaluator_utils package.
  * remove diagnostic_msgs dependency of planning_evaluator
  * use metric_msgs for autoware_processing_time_checker.
  * rewrite diagnostic_convertor to scenario_simulator_v2_adapter, supporting metric_msgs.
  * pre-commit and fix typo
  * publish metrics even if there is no metric in the MetricArray.
  * modify the metric name of processing_time.
  * update unit test for test_planning/control_evaluator
  * manual pre-commit
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Kem (TiankuiXian), Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* feat(control_evaluator): add goal accuracy longitudinal, lateral, yaw (`#9155 <https://github.com/autowarefoundation/autoware_universe/issues/9155>`_)
  * feat(control_evaluator): add goal accuracy longitudinal, lateral, yaw
  * style(pre-commit): autofix
  * fix: content of kosuke55-san comments
  * fix: variable name
  * fix: variable name
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* test(autoware_control_evaluator): add test for autoware_control_evaluator. (`#9114 <https://github.com/autowarefoundation/autoware_universe/issues/9114>`_)
  * init
  * tmp save.
  * save, there is a bug
  * update package.xml
  * coverage rate 64.5
  * remove comments.
  ---------
* docs(control_evaluator): update readme (`#8829 <https://github.com/autowarefoundation/autoware_universe/issues/8829>`_)
  * update readme
  * add maintainer
  * Update evaluator/autoware_control_evaluator/package.xml
  Add extra maintainer
  Co-authored-by: Tiankui Xian <1041084556@qq.com>
  ---------
  Co-authored-by: Tiankui Xian <1041084556@qq.com>
* feat(evalautor): rename evaluator diag topics (`#8152 <https://github.com/autowarefoundation/autoware_universe/issues/8152>`_)
  * feat(evalautor): rename evaluator diag topics
  * perception
  ---------
* refactor(autoware_universe_utils): changed the API to be more intuitive and added documentation (`#7443 <https://github.com/autowarefoundation/autoware_universe/issues/7443>`_)
  * refactor(tier4_autoware_utils): Changed the API to be more intuitive and added documentation.
  * use raw shared ptr in PollingPolicy::NEWEST
  * update
  * fix
  * Update evaluator/autoware_control_evaluator/include/autoware/control_evaluator/control_evaluator_node.hpp
  Co-authored-by: danielsanchezaran <daniel.sanchez@tier4.jp>
  ---------
  Co-authored-by: danielsanchezaran <daniel.sanchez@tier4.jp>
* feat(planning_evaluator,control_evaluator, evaluator utils): add diagnostics subscriber to planning eval (`#7849 <https://github.com/autowarefoundation/autoware_universe/issues/7849>`_)
  * add utils and diagnostics subscription to planning_evaluator
  * add diagnostics eval
  * fix input diag in launch
  ---------
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(control_evaluator): use class naming standard and use remapped param name (`#7782 <https://github.com/autowarefoundation/autoware_universe/issues/7782>`_)
  use class naming standard and use remapped param name
* feat(control_evaluator): add lanelet info to the metrics (`#7765 <https://github.com/autowarefoundation/autoware_universe/issues/7765>`_)
  * add route handler
  * add lanelet info to diagnostic
  * add const
  * add kinematic state info
  * clean
  * remove unusde subscriptions
  * clean
  * add shoulder lanelets
  * fix includes
  ---------
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(control_evaluator): rename to include/autoware/{package_name} (`#7520 <https://github.com/autowarefoundation/autoware_universe/issues/7520>`_)
  * feat(control_evaluator): rename to include/autoware/{package_name}
  * fix
  ---------
* Contributors: Kazunori-Nakajima, Kosuke Takeuchi, Takayuki Murooka, Tiankui Xian, Yukinari Hisaki, Yutaka Kondo, danielsanchezaran

0.26.0 (2024-04-03)
-------------------
