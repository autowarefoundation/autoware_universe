^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_evaluator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat: change planning output topic name to /planning/trajectory (`#11135 <https://github.com/autowarefoundation/autoware_universe/issues/11135>`_)
  * change planning output topic name to /planning/trajectory
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(planning_evaluaotr): prevent abnormal value for ttc (`#11138 <https://github.com/autowarefoundation/autoware_universe/issues/11138>`_)
* style(pre-commit): autofix (`#10982 <https://github.com/autowarefoundation/autoware_universe/issues/10982>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Kosuke Takeuchi, Ryohsuke Mitsudome, Yukihiro Saito

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat!: remove obstacle_stop_planner and obstacle_cruise_planner (`#10695 <https://github.com/autowarefoundation/autoware_universe/issues/10695>`_)
  * feat: remove obstacle_stop_planner and obstacle_cruise_planner
  * update
  * fix
  ---------
* fix(planning_evaluator): fix bug of abnormal_stop metric, and turn its threshold (`#10628 <https://github.com/autowarefoundation/autoware_universe/issues/10628>`_)
  * fix bug
  * change threeshold
  * update abnormal_deceleration_threshold_mps2
  * change back threshold
  * add takeuchi san as maintainer
  * rename func
  ---------
* Contributors: Kem (TiankuiXian), TaikiYamada4, Takayuki Murooka

0.45.0 (2025-05-22)
-------------------

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore(autoware_planning_evaluator): record goal_stop_deviation only when ego stop (`#10429 <https://github.com/autowarefoundation/autoware_universe/issues/10429>`_)
  * record modified_goal related output_metric only when the ego stop close to goal
  * change vel thr
  * pre-commit
  ---------
* feat(autoware_planning_evaluator): refactor planning_evaluator for new metrics (`#10368 <https://github.com/autowarefoundation/autoware_universe/issues/10368>`_)
  * tmp save.
  * tmp save.
  * WIP add accumulator-based metrics.
  * pre-commit
  * add unit test.
  * pre-commit
  * fix cppcheck
  * update readme.
  * pre-commit
  * polish readme.
  * pre-commit
  * change count to size_t
  * update config.
  * publish count.
  * fix stop decision bug
  * update parameters.
  * fix typo
  ---------
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
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
* feat(autoware_planning_evaluator): add resampled_relative_angle metrics (`#10020 <https://github.com/autowarefoundation/autoware_universe/issues/10020>`_)
  * feat(autoware_planning_evaluator): add new large_relative_angle metrics
  * fix copyright and vehicle_length_m
  * style(pre-commit): autofix
  * del: resample trajectory
  * del: traj points check
  * rename msg and speed optimization
  * style(pre-commit): autofix
  * add unit_test and fix resample_relative_angle
  * style(pre-commit): autofix
  * include tuple to test
  * target two point, update unit test value
  * fix abs
  * fix for loop bag and primitive type
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Kazunori-Nakajima, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in fil… (`#9859 <https://github.com/autowarefoundation/autoware_universe/issues/9859>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files evaluator/autoware_planning_evaluator
* fix(planning_evaluator): update lateral_trajectory_displacement to absolute value (`#9696 <https://github.com/autowarefoundation/autoware_universe/issues/9696>`_)
  * fix(planning_evaluator): update lateral_trajectory_displacement to absolute value
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_planning_evaluator): rename lateral deviation metrics (`#9801 <https://github.com/autowarefoundation/autoware_universe/issues/9801>`_)
  * refactor(planning_evaluator): rename and add lateral trajectory displacement metrics
  * fix typo
  ---------
* feat(planning_evaluator): add evaluation feature of trajectory lateral displacement (`#9718 <https://github.com/autowarefoundation/autoware_universe/issues/9718>`_)
  * feat(planning_evaluator): add evaluation feature of trajectory lateral displacement
  * feat(metrics_calculator): implement lookahead trajectory calculation and remove deprecated method
  * fix(planning_evaluator): rename lateral_trajectory_displacement to trajectory_lateral_displacement for consistency
  ---------
* fix(autoware_planning_evaluator): fix bugprone-exception-escape (`#9730 <https://github.com/autowarefoundation/autoware_universe/issues/9730>`_)
  fix: bugprone-exception-escape
* feat(planning_evaluator): add lateral trajectory displacement metrics (`#9674 <https://github.com/autowarefoundation/autoware_universe/issues/9674>`_)
  * feat(planning_evaluator): add nearest pose deviation msg
  * update comment contents
  * update variable name
  * Revert "update variable name"
  This reverts commit ee427222fcbd2a18ffbc20fecca3ad557f527e37.
  * move lateral_trajectory_displacement position
  * prev.dist -> prev_lateral_deviation
  ---------
* Contributors: Fumiya Watanabe, Kazunori-Nakajima, Kyoichi Sugahara, Vishal Chauhan, kobayu858

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
* feat(planning_evaluator): add a trigger to choice whether to output metrics to log folder (`#9476 <https://github.com/autowarefoundation/autoware_universe/issues/9476>`_)
  * tmp save
  * planning_evaluator build ok, test it.
  * add descriptions to output.
  * pre-commit.
  * add parm to launch file.
  * move output_metrics from config to launch file.
  * fix unit test bug.
  ---------
* refactor(evaluators, autoware_universe_utils): rename Stat class to Accumulator and move it to autoware_universe_utils (`#9459 <https://github.com/autowarefoundation/autoware_universe/issues/9459>`_)
  * add Accumulator class to autoware_universe_utils
  * use Accumulator on all evaluators.
  * pre-commit
  * found and fixed a bug. add more tests.
  * pre-commit
  * Update common/autoware_universe_utils/include/autoware/universe_utils/math/accumulator.hpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* feat(planning_evaluator): add processing time pub (`#9334 <https://github.com/autowarefoundation/autoware_universe/issues/9334>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix(evaluator): missing dependency in evaluator components (`#9074 <https://github.com/autowarefoundation/autoware_universe/issues/9074>`_)
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kazunori-Nakajima, Kem (TiankuiXian), M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo, ぐるぐる

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
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
* refactor(autoware_planning_evaluator): devops node dojo (`#8746 <https://github.com/autowarefoundation/autoware_universe/issues/8746>`_)
* feat(motion_velocity_planner,planning_evaluator): add  stop, slow_down diags (`#8503 <https://github.com/autowarefoundation/autoware_universe/issues/8503>`_)
  * tmp save.
  * publish diagnostics.
  * move clearDiagnostics func to head
  * change to snake_names.
  * remove a change of launch.xml
  * pre-commit run -a
  * publish diagnostics on node side.
  * move empty checking out of 'get_diagnostics'.
  * remove get_diagnostics; change reason str.
  * remove unused condition.
  * Update planning/motion_velocity_planner/autoware_motion_velocity_planner_node/src/planner_manager.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  * Update planning/motion_velocity_planner/autoware_motion_velocity_planner_node/src/planner_manager.cpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* fix(autoware_planning_evaluator): fix unreadVariable (`#8352 <https://github.com/autowarefoundation/autoware_universe/issues/8352>`_)
  * fix:unreadVariable
  * fix:unreadVariable
  ---------
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
* feat(cruise_planner,planning_evaluator): add cruise and slow down diags (`#7960 <https://github.com/autowarefoundation/autoware_universe/issues/7960>`_)
  * add cruise and slow down diags to cruise planner
  * add cruise types
  * adjust planning eval
  ---------
* feat(planning_evaluator,control_evaluator, evaluator utils): add diagnostics subscriber to planning eval (`#7849 <https://github.com/autowarefoundation/autoware_universe/issues/7849>`_)
  * add utils and diagnostics subscription to planning_evaluator
  * add diagnostics eval
  * fix input diag in launch
  ---------
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(planning_evaluator): add planning evaluator polling sub (`#7827 <https://github.com/autowarefoundation/autoware_universe/issues/7827>`_)
  * WIP add polling subs
  * WIP
  * update functions
  * remove semicolon
  * use last data for modified goal
  ---------
* feat(planning_evaluator): add lanelet info to the planning evaluator (`#7781 <https://github.com/autowarefoundation/autoware_universe/issues/7781>`_)
  add lanelet info to the planning evaluator
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(planning_evaluator): rename to include/autoware/{package_name} (`#7518 <https://github.com/autowarefoundation/autoware_universe/issues/7518>`_)
  * fix
  * fix
  ---------
* Contributors: Kosuke Takeuchi, Takayuki Murooka, Tiankui Xian, Yukinari Hisaki, Yutaka Kondo, danielsanchezaran, kobayu858, odra

0.26.0 (2024-04-03)
-------------------
