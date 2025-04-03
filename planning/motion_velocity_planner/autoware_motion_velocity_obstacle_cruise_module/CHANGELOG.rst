^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_obstacle_cruise_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
