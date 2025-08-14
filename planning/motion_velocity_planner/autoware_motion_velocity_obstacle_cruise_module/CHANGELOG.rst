^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_obstacle_cruise_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat: change planning output topic name to /planning/trajectory (`#11135 <https://github.com/autowarefoundation/autoware_universe/issues/11135>`_)
  * change planning output topic name to /planning/trajectory
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* docs(obstacle_cruise): align the rss computation documentation with the codes (`#11049 <https://github.com/autowarefoundation/autoware_universe/issues/11049>`_)
  docs(obstacle_cruise): align the rss computation documentation with the codes.
* refactor(obstacle_slowdown_module, obstacle_cruise_module): follow function name change in autoware_core (`#10840 <https://github.com/autowarefoundation/autoware_universe/issues/10840>`_)
  change function name
* Contributors: Mete Fatih Cırıt, Yuki TAKAGI, Yukihiro Saito, Yuxuan Liu

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(autoware_motion_velocity_planner): only wait for required subscriptions (`#10732 <https://github.com/autowarefoundation/autoware_universe/issues/10732>`_)
* feat(obstacle_stop_module): maintain larger stop distance for opposing traffic (`#10647 <https://github.com/autowarefoundation/autoware_universe/issues/10647>`_)
  Changes for RT1-9226
* Contributors: Arjun Jagdish Ram, Ryohsuke Mitsudome, TaikiYamada4

0.45.0 (2025-05-22)
-------------------

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore(motion_velocity_planner): move common and node packages to core (`#10367 <https://github.com/autowarefoundation/autoware_universe/issues/10367>`_)
* fix(motion_velocity_obstacle_xxx_module): fix debug topic name (`#10322 <https://github.com/autowarefoundation/autoware_universe/issues/10322>`_)
* fix(motion_velocity_planner): remove Metric messages (`#10342 <https://github.com/autowarefoundation/autoware_universe/issues/10342>`_)
* Contributors: Maxime CLEMENT, Ryohsuke Mitsudome, Takayuki Murooka

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* feat(path_optimizer): additional failure logging and failure mode handling (`#10276 <https://github.com/autowarefoundation/autoware_universe/issues/10276>`_)
  MRM when MPT fails
* feat!: replace VelocityLimit messages with autoware_internal_planning_msgs (`#10273 <https://github.com/autowarefoundation/autoware_universe/issues/10273>`_)
* fix(obstacle_cruise_planner): fix obstacle filtering logic (`#10232 <https://github.com/autowarefoundation/autoware_universe/issues/10232>`_)
  * add absolute
  * fix find_yield_cruise_obstacles() calling
  ---------
* fix(obstacle_cruise_planner): ignore invalid stopping objects (`#10227 <https://github.com/autowarefoundation/autoware_universe/issues/10227>`_)
  * ignore not specified stopping objects
  * change debug print level
  * add ahead_stopped prameter
  * rename ahead_stopped -> side_stopped
  ---------
* feat(Autoware_planning_factor_interface): replace tier4_msgs with autoware_internal_msgs (`#10204 <https://github.com/autowarefoundation/autoware_universe/issues/10204>`_)
* Contributors: Arjun Jagdish Ram, Hayato Mizushima, Kento Yabuuchi, Ryohsuke Mitsudome, Yutaka Kondo, 心刚

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat(autoware_objects_of_interest_marker_interface): replace autoware_universe_utils with autoware_utils (`#10174 <https://github.com/autowarefoundation/autoware_universe/issues/10174>`_)
* feat: introduce motion_velocity_obstacle\_<stop/slow_down/cruise>_module (`#9807 <https://github.com/autowarefoundation/autoware_universe/issues/9807>`_)
  * implement obstacle stop, slow_down, and cruise
  * fix clang-tidy
  * revert obstacle_cruise_planner
  ---------
* Contributors: Fumiya Watanabe, Ryohsuke Mitsudome, Takayuki Murooka, 心刚

* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat(autoware_objects_of_interest_marker_interface): replace autoware_universe_utils with autoware_utils (`#10174 <https://github.com/autowarefoundation/autoware_universe/issues/10174>`_)
* feat: introduce motion_velocity_obstacle\_<stop/slow_down/cruise>_module (`#9807 <https://github.com/autowarefoundation/autoware_universe/issues/9807>`_)
  * implement obstacle stop, slow_down, and cruise
  * fix clang-tidy
  * revert obstacle_cruise_planner
  ---------
* Contributors: Fumiya Watanabe, Ryohsuke Mitsudome, Takayuki Murooka, 心刚
