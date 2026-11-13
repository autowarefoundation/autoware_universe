^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_hazard_lights_selector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* fix(design): align the planning node designs with the packages they describe (`#13342 <https://github.com/autowarefoundation/autoware_universe/issues/13342>`_)
  * fix(design): align the planning node designs with the packages they describe
  HazardLightsSelector registers as HazardLightsSelector, and its second input
  maps to input/system/hazard_lights_command and carries the MRM command, so the
  port is named system_hazard_lights_cmd.
  SurroundObstacleChecker publishes the migrated autoware_internal_planning_msgs
  velocity limit types, and creates neither stop_reasons nor no_start_reason.
  RemainingDistanceTimeCalculator takes its velocity limit input through the
  design graph; global: keeps a port out of it, since link_manager skips any
  connection whose target is a global input port and the exporter emits no remap.
  ExternalVelocityLimitSelector's api_limit is the reverse case: it is published
  only by AD API adaptors outside the design graph, so it keeps global: alongside
  its remap_target.
  `#13165 <https://github.com/autowarefoundation/autoware_universe/issues/13165>`_ unified the Modifier and Optimizer nodes into TrajectoryProcessor but
  left both old designs in place, naming plugins, executables and param files
  that no longer exist, and put the new design in common/autoware_universe_designs,
  a bare directory that is not a ROS package. The design moves into
  autoware_trajectory_processor per the convention set by `#13102 <https://github.com/autowarefoundation/autoware_universe/issues/13102>`_, and its
  processing_time_detail publisher carries ProcessingTimeTree.
  * feat(autoware_path_sampler): add the packaged path sampler parameter file
  The package had no config/ directory and its INSTALL_TO_SHARE was commented
  out, so the parameter file the node design points at lived only in
  autoware_launch. The file is copied into the package and installed, and the
  design declares the three shared param files the launcher also passes, which is
  where ego_nearest_dist_threshold and ego_nearest_yaw_threshold come from.
  * feat(design): add the planning factor publishers to the BehaviorPathPlanner node design
  The node publishes nine planning factor arrays under
  /planning/planning_factors that the design did not declare, so nothing could
  connect them to the evaluator.
  ---------
* refactor(planning): move node design files into each package (`#13102 <https://github.com/autowarefoundation/autoware_universe/issues/13102>`_)
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* feat(hazard_lights_selector): apply `agnocast_wrapper::Node` to `autoware_hazard_lights_selector` (`#12850 <https://github.com/autowarefoundation/autoware_universe/issues/12850>`_)
  * apply agnocast_wrapper::Node
  * fix cpplint and delete unnecessary comments
  * fix for invalid parameter
  ---------
* Contributors: Koichi Imai, Ryohsuke Mitsudome, Taekjin LEE

0.52.0 (2026-06-30)
-------------------

0.51.0 (2026-05-01)
-------------------

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(planning, perception): replace wall_timer with generic timer (`#11005 <https://github.com/autowarefoundation/autoware_universe/issues/11005>`_)
  * feat(planning, perception): replace wall_timer with generic timer
  * use rclcpp::create_timer
  * remove period_ns
  ---------
* Contributors: Mamoru Sobue, Ryohsuke Mitsudome

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(hazard_lights_selector): add a hazard lights selector package (`#10692 <https://github.com/autowarefoundation/autoware_universe/issues/10692>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* Contributors: Makoto Kurihara, TaikiYamada4
