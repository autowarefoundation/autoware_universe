^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_obstacle_stop_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* fix(obstacle_stop): use max_lat_margin_against_unknown only for predicted object (`#10269 <https://github.com/autowarefoundation/autoware_universe/issues/10269>`_)
* fix(obstacle stop/slow_down): early return without point cloud (`#10289 <https://github.com/autowarefoundation/autoware_universe/issues/10289>`_)
  * fix(obstacle stop/slow_down): early return without point cloud
  * update maintainer
  ---------
* fix(obstacle_stop): accounting for vehicle bumper length when handling point-cloud stop points (`#10250 <https://github.com/autowarefoundation/autoware_universe/issues/10250>`_)
  * fix for point-cloud stop point
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(Autoware_planning_factor_interface): replace tier4_msgs with autoware_internal_msgs (`#10204 <https://github.com/autowarefoundation/autoware_universe/issues/10204>`_)
* Contributors: Arjun Jagdish Ram, Hayato Mizushima, Takayuki Murooka, Yutaka Kondo, 心刚

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat(autoware_motion_velocity_obstacle\_<stop/slow_down>_module): brings Point Cloud handling to this module (`#10112 <https://github.com/autowarefoundation/autoware_universe/issues/10112>`_)
  pointcloud handling for slowdown and stop module
* feat(autoware_objects_of_interest_marker_interface): replace autoware_universe_utils with autoware_utils (`#10174 <https://github.com/autowarefoundation/autoware_universe/issues/10174>`_)
* chore(motion_velocity_obstacle_stop_module): remove unnecessary dependency (`#10109 <https://github.com/autowarefoundation/autoware_universe/issues/10109>`_)
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
* chore(motion_velocity_obstacle_stop_module): remove unnecessary dependency (`#10109 <https://github.com/autowarefoundation/autoware_universe/issues/10109>`_)
* feat: introduce motion_velocity_obstacle\_<stop/slow_down/cruise>_module (`#9807 <https://github.com/autowarefoundation/autoware_universe/issues/9807>`_)
  * implement obstacle stop, slow_down, and cruise
  * fix clang-tidy
  * revert obstacle_cruise_planner
  ---------
* Contributors: Arjun Jagdish Ram, Fumiya Watanabe, Ryohsuke Mitsudome, Takayuki Murooka, 心刚
