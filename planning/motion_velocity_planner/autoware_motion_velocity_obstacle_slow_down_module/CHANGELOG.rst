^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_obstacle_slow_down_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(obstacle_slow_down): rework type specific params, split left/right (`#10993 <https://github.com/autowarefoundation/autoware_universe/issues/10993>`_)
* refactor(obstacle_slowdown_module, obstacle_cruise_module): follow function name change in autoware_core (`#10840 <https://github.com/autowarefoundation/autoware_universe/issues/10840>`_)
  change function name
* Contributors: Maxime CLEMENT, Mete Fatih Cırıt, Yuki TAKAGI

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(autoware_motion_velocity_planner): only wait for required subscriptions (`#10732 <https://github.com/autowarefoundation/autoware_universe/issues/10732>`_)
* feat(obstacle_slow_down): update parameter explanation (`#10721 <https://github.com/autowarefoundation/autoware_universe/issues/10721>`_)
  * feat(obstacle_slow_down): update parameter explanation
  * update
  ---------
* fix(obstacle_slow_down_module): fix object type specified params bug (`#10672 <https://github.com/autowarefoundation/autoware_universe/issues/10672>`_)
* Contributors: Ryohsuke Mitsudome, TaikiYamada4, Takayuki Murooka, Yuki TAKAGI

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* fix(autoware_motion_velocity_obstacle_slow_down_module): fix for mishandling lateral-distance (`#10559 <https://github.com/autowarefoundation/autoware_universe/issues/10559>`_)
  * Fix for mishandling lateral-distance
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_motion_velocity_planner): point-cloud clustering optimization (`#10477 <https://github.com/autowarefoundation/autoware_universe/issues/10477>`_)
  * Point-Cloud clustering optimization
  * style(pre-commit): autofix
  * fix
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Arjun Jagdish Ram, TaikiYamada4

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
* feat!: replace VelocityLimit messages with autoware_internal_planning_msgs (`#10273 <https://github.com/autowarefoundation/autoware_universe/issues/10273>`_)
* fix(obstacle stop/slow_down): early return without point cloud (`#10289 <https://github.com/autowarefoundation/autoware_universe/issues/10289>`_)
  * fix(obstacle stop/slow_down): early return without point cloud
  * update maintainer
  ---------
* feat(Autoware_planning_factor_interface): replace tier4_msgs with autoware_internal_msgs (`#10204 <https://github.com/autowarefoundation/autoware_universe/issues/10204>`_)
* Contributors: Hayato Mizushima, Ryohsuke Mitsudome, Takayuki Murooka, Yutaka Kondo, 心刚

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat(autoware_motion_velocity_obstacle\_<stop/slow_down>_module): brings Point Cloud handling to this module (`#10112 <https://github.com/autowarefoundation/autoware_universe/issues/10112>`_)
  pointcloud handling for slowdown and stop module
* feat(autoware_objects_of_interest_marker_interface): replace autoware_universe_utils with autoware_utils (`#10174 <https://github.com/autowarefoundation/autoware_universe/issues/10174>`_)
* feat: introduce motion_velocity_obstacle\_<stop/slow_down/cruise>_module (`#9807 <https://github.com/autowarefoundation/autoware_universe/issues/9807>`_)
  * implement obstacle stop, slow_down, and cruise
  * fix clang-tidy
  * revert obstacle_cruise_planner
  ---------
* Contributors: Arjun Jagdish Ram, Fumiya Watanabe, Ryohsuke Mitsudome, Takayuki Murooka, 心刚

* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat(autoware_motion_velocity_obstacle\_<stop/slow_down>_module): brings Point Cloud handling to this module (`#10112 <https://github.com/autowarefoundation/autoware_universe/issues/10112>`_)
  pointcloud handling for slowdown and stop module
* feat(autoware_objects_of_interest_marker_interface): replace autoware_universe_utils with autoware_utils (`#10174 <https://github.com/autowarefoundation/autoware_universe/issues/10174>`_)
* feat: introduce motion_velocity_obstacle\_<stop/slow_down/cruise>_module (`#9807 <https://github.com/autowarefoundation/autoware_universe/issues/9807>`_)
  * implement obstacle stop, slow_down, and cruise
  * fix clang-tidy
  * revert obstacle_cruise_planner
  ---------
* Contributors: Arjun Jagdish Ram, Fumiya Watanabe, Ryohsuke Mitsudome, Takayuki Murooka, 心刚
