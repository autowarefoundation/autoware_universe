^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_processing_time_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* fix(design): align the system node designs with the packages they describe (`#13338 <https://github.com/autowarefoundation/autoware_universe/issues/13338>`_)
  * fix(design): declare the fixed-name interfaces of the system node designs as remap targets
  The diagnostic graph ports, the diagnostics array publisher and the automatic
  pose initializer's localization interfaces are pinned with global:, which keeps
  them out of the design graph: link_manager skips any connection whose target is
  a global input port and the exporter emits no remap. remap_target: keeps the
  same fixed topic and service names while letting the ports take part in the
  graph.
  * feat(autoware_default_adapi_universe): add node designs for the AD API nodes
  The package builds fifteen AD API components with no design of their own, so a
  system that composes the AD API from design modules cannot reference them.
  * feat(design): add the diagnostic graph and component state monitor node designs
  autoware_component_state_monitor and autoware_diagnostic_graph_aggregator build
  three components with no design of their own: the state monitor that aggregates
  topic monitor diagnostics into per-component availability, the aggregator that
  turns /diagnostics into the diagnostic graph, and the converter that derives
  operation mode availability from it.
  DiagnosticGraphLogging and ProcessingTimeChecker declare the parameters their
  launchers pass, which have no package param file to come from.
  ---------
* chore(autoware_trajectory_processor)!: rename to autoware_trajectory_modifier (`#13389 <https://github.com/autowarefoundation/autoware_universe/issues/13389>`_)
  rename processor -> modifier
* feat(trajectory_processor): unify Modifier and Optimizer nodes into Processor node (`#13165 <https://github.com/autowarefoundation/autoware_universe/issues/13165>`_)
* refactor(system): move node design files into each package (`#13103 <https://github.com/autowarefoundation/autoware_universe/issues/13103>`_)
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* style(processing_time_checker): adjust comment indentation (`#12780 <https://github.com/autowarefoundation/autoware_universe/issues/12780>`_)
  adjust comment indentation
  Co-authored-by: Junya Sasaki <junya.sasaki@tier4.jp>
* feat(processing_time_checker): apply `agnocast_wrapper::Node` to `processing_time_checker` (`#12761 <https://github.com/autowarefoundation/autoware_universe/issues/12761>`_)
  * apply agnocast_wrapper::Node to processing_time_checker
  * launch and Cmake
  * fix copilot review
  * fix to use ALLOCATE
  * fix cpplint
  ---------
  Co-authored-by: atsushi yano <55824710+atsushi421@users.noreply.github.com>
* Contributors: Koichi Imai, Maxime CLEMENT, Ryohsuke Mitsudome, Taekjin LEE, Taeseung Sohn

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(processing_time_cheker): add neural_network_based_planner node `processing_time_ms` (`#12529 <https://github.com/autowarefoundation/autoware_universe/issues/12529>`_)
  * feat(diffusion_planner, trajectory_optimizer): add processing_time_ms topic
  * feat: processing_time_checker
  ---------
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
* Contributors: Kazunori-Nakajima, github-actions

0.51.0 (2026-05-01)
-------------------

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(run_out, processing_time_checker): support processing_time for run_out and update processing time list (`#11515 <https://github.com/autowarefoundation/autoware_universe/issues/11515>`_)
  update processing time list and support run_out
* chore(run_out): rm the autoware_behavior_velocity_run_out module (`#11320 <https://github.com/autowarefoundation/autoware_universe/issues/11320>`_)
* fix(autoware_processing_time_checker): remove unused function (`#11213 <https://github.com/autowarefoundation/autoware_universe/issues/11213>`_)
* fix(autoware_processing_time_checker): remove unused function (`#11182 <https://github.com/autowarefoundation/autoware_universe/issues/11182>`_)
* Contributors: Kem (TiankuiXian), Maxime CLEMENT, Ryohsuke Mitsudome, Ryuta Kambe

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat(path_generator): publish processing time (`#10863 <https://github.com/autowarefoundation/autoware_universe/issues/10863>`_)
* Contributors: Kosuke Takeuchi

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(processing_time_checker): output 99th and 95th percentile of processing time statics (`#10810 <https://github.com/autowarefoundation/autoware_universe/issues/10810>`_)
  * implement digestible in processing_time_checker
  * remove debug
  * change to 100
  * license
  * fix copyright
  * pre-commit
  ---------
* docs(system): fix system documentation link (`#10707 <https://github.com/autowarefoundation/autoware_universe/issues/10707>`_)
* Contributors: Kem (TiankuiXian), TaikiYamada4, Yuxuan Liu

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
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* Contributors: Fumiya Watanabe, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_processing_time_checker)!: tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_processing_time_checker (`#9921 <https://github.com/autowarefoundation/autoware_universe/issues/9921>`_)
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* fix(autoware_processing_time_checker): fix bugprone-exception-escape (`#9780 <https://github.com/autowarefoundation/autoware_universe/issues/9780>`_)
  * fix: bugprone-error
  * fix: cpplint
  ---------
* Contributors: Fumiya Watanabe, Vishal Chauhan, kobayu858

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
* fix(autoware_processing_time_checker): fix typo (`#9504 <https://github.com/autowarefoundation/autoware_universe/issues/9504>`_)
* feat(autoware_processing_time_checker): add a trigger to choice whether to output metrics to log folder (`#9479 <https://github.com/autowarefoundation/autoware_universe/issues/9479>`_)
  * add output_metrics option.
  * move param set from config to launch file.
  * fix bug.
  ---------
* feat(processing_time_checker): update processing time list (`#9350 <https://github.com/autowarefoundation/autoware_universe/issues/9350>`_)
* 0.39.0
* update changelog
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kazunori-Nakajima, Kem (TiankuiXian), Ryohsuke Mitsudome, Ryuta Kambe, Yutaka Kondo

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
* feat(costmap_generator, control_validator, scenario_selector, surround_obstacle_checker, vehicle_cmd_gate): add processing time pub. (`#9065 <https://github.com/autowarefoundation/autoware_universe/issues/9065>`_)
  * feat(costmap_generator, control_validator, scenario_selector, surround_obstacle_checker, vehicle_cmd_gate): Add: processing_time_pub
  * fix: pre-commit
  * feat(costmap_generator): fix: No output when not Active.
  * fix: clang-format
  * Re: fix: clang-format
  ---------
* feat(processing_time_checker): add a new package (`#7957 <https://github.com/autowarefoundation/autoware_universe/issues/7957>`_)
  * feat(processing_time_checker): add a new package
  * fix
  * fix
  * update README and schema.json
  * fix
  * fix
  * fix
  ---------
* Contributors: Kazunori-Nakajima, Takayuki Murooka, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
