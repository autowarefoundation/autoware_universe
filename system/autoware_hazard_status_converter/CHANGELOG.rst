^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_hazard_status_converter
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
* feat(diagnostic_graph_utils): apply `agnocast_wrapper::Node` to `diagnostic_graph_utils` and `hazard_status_converter` (`#13075 <https://github.com/autowarefoundation/autoware_universe/issues/13075>`_)
  * feat(diagnostic_graph_utils): apply `agnocast_wrapper::Node` to `diagnostic_graph_utils` and `hazard_status_converter`
  * feat(diagnostic_graph_utils): add launch files for converter and dump nodes
  Both nodes were only documented as `ros2 run`, which does not preload the Agnocast heaphook.
  converter_node publishes /diagnostics_array through an agnocast publisher, so it needs the
  heaphook under ENABLE_AGNOCAST=1. Give both nodes a launch file that includes
  agnocast_env.launch.xml and sets LD_PRELOAD, as logging.launch.xml does, and point the docs at
  them.
  ---------
* refactor(system): move node design files into each package (`#13103 <https://github.com/autowarefoundation/autoware_universe/issues/13103>`_)
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* Contributors: Koichi Imai, Ryohsuke Mitsudome, Taekjin LEE

0.52.0 (2026-06-30)
-------------------

0.51.0 (2026-05-01)
-------------------

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(hazard_status_converter): use root output level for emergency holding (`#11954 <https://github.com/autowarefoundation/autoware_universe/issues/11954>`_)
* Contributors: Ryohsuke Mitsudome, Takagi, Isamu

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* feat(hazard_status_converter): use latch level for holding status (`#11721 <https://github.com/autowarefoundation/autoware_universe/issues/11721>`_)
* Contributors: Ryohsuke Mitsudome, Takagi, Isamu

0.48.0 (2025-11-18)
-------------------

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(diagnostic_graph_aggregator): support latch and dependent error (`#10829 <https://github.com/autowarefoundation/autoware_universe/issues/10829>`_)
  * update aggregator
  * update utils
  * update hazard converter
  * update adapi
  * fix build error
  * ignore spell check for yamls
  * fix copyright year
  * ignore spell check for timeline
  * change link index to fix out of bounds access
  * reflect link index change to utils
  * fix for cppcheck
  * fix for cppcheck
  ---------
* Contributors: TaikiYamada4, Takagi, Isamu

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
* feat: apply `autoware\_` prefix for `diagnostic_graph_utils` (`#9968 <https://github.com/autowarefoundation/autoware_universe/issues/9968>`_)
* feat: apply `autoware\_` prefix for `hazard_status_converter` (`#9971 <https://github.com/autowarefoundation/autoware_universe/issues/9971>`_)
* Contributors: Fumiya Watanabe, Junya Sasaki

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
* fix(cpplint): include what you use - system (`#9573 <https://github.com/autowarefoundation/autoware_universe/issues/9573>`_)
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(hazard_status_converter): hazard status converter subscribe emergency holding (`#9287 <https://github.com/autowarefoundation/autoware_universe/issues/9287>`_)
  * feat: add subscriber for emergency_holding
  * modify: fix value name
  * style(pre-commit): autofix
  * feat: add const to is_emergency_holding
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, TetsuKawa, Yutaka Kondo

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
* feat!: replace autoware_auto_msgs with autoware_msgs for system modules (`#7249 <https://github.com/autowarefoundation/autoware_universe/issues/7249>`_)
  Co-authored-by: Cynthia Liu <cynthia.liu@autocore.ai>
  Co-authored-by: NorahXiong <norah.xiong@autocore.ai>
  Co-authored-by: beginningfan <beginning.fan@autocore.ai>
* feat: remake diagnostic graph packages (`#6715 <https://github.com/autowarefoundation/autoware_universe/issues/6715>`_)
* fix(hazard_status_converter): check current operation mode (`#6733 <https://github.com/autowarefoundation/autoware_universe/issues/6733>`_)
  * fix: hazard status converter
  * fix: topic name and modes
  * fix check target mode
  * fix message type
  * Revert "fix check target mode"
  This reverts commit 8b190b7b99490503a52b155cad9f593c1c97e553.
  ---------
* Contributors: Ryohsuke Mitsudome, Takagi, Isamu, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
* feat(tier4_system_launch): add option to launch mrm handler (`#6660 <https://github.com/autowarefoundation/autoware_universe/issues/6660>`_)
* feat(hazard_status_converter): add package (`#6428 <https://github.com/autowarefoundation/autoware_universe/issues/6428>`_)
* Contributors: Takagi, Isamu
