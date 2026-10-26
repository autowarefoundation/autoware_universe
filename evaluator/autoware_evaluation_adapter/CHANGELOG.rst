^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_evaluation_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* fix(design): align the evaluator node designs with the packages they describe (`#13341 <https://github.com/autowarefoundation/autoware_universe/issues/13341>`_)
  The fixed-name ports of the evaluation adapters, the online perception
  evaluator and the metric converter are pinned with global:, which keeps them
  out of the design graph: link_manager skips any connection whose target is a
  global input port and the exporter emits no remap. remap_target: keeps the same
  topic and service names while letting the ports take part in the graph.
  /diagnostics stays global:.
  ControlEvaluator names a plugin class in the wrong namespace, publishes
  tier4_metric_msgs/MetricArray rather than the non-existent
  autoware_control_msgs/ControlEvaluation, and subscribes to eleven planning
  factor topics under /planning/planning_factors, two of which were missing.
  PlanningEvaluator publishes the same metric type.
  The evaluation adapter nodes are components with no executable of their own.
* refactor(evaluator): move node design files into each package (`#13099 <https://github.com/autowarefoundation/autoware_universe/issues/13099>`_)
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* Contributors: Ryohsuke Mitsudome, Taekjin LEE

0.52.0 (2026-06-30)
-------------------

0.51.0 (2026-05-01)
-------------------

0.50.0 (2026-02-14)
-------------------
* chore: match all package versions
* Merge remote-tracking branch 'origin/main' into humble
* feat(tier4_autoware_api_launch): remove tier4 api adapter (`#11831 <https://github.com/autowarefoundation/autoware_universe/issues/11831>`_)
  * feat(tier4_autoware_api_launch): remove tier4 api adapter
  * add evaluation adapter
  * remove unused interface
  * add launch option
  * style(pre-commit): autofix
  * update readme
  * fix typo
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Takagi, Isamu
