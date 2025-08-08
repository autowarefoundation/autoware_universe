^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_obstacle_velocity_limiter_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(autoware_motion_velocity_planner): only wait for required subscriptions (`#10732 <https://github.com/autowarefoundation/autoware_universe/issues/10732>`_)
* Contributors: Ryohsuke Mitsudome, TaikiYamada4

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
* Contributors: Maxime CLEMENT, Ryohsuke Mitsudome

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat(motion_velocity_planner): common implementation for motion_velocity_obstacle\_<stop/slow_down/cruise>_module (`#10035 <https://github.com/autowarefoundation/autoware_universe/issues/10035>`_)
  * feat(motion_velocity_planner): prepare for motion_velocity\_<stop/slow_down/cruise>_module
  * update launch
  ---------
* Contributors: Fumiya Watanabe, Takayuki Murooka, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(motion_velocity_planner)!: add _universe suffix to autoware_motion_velocity_planner_common and autoware_motion_velocity_planner_node (`#9942 <https://github.com/autowarefoundation/autoware_universe/issues/9942>`_)
* feat(planning_factor)!: remove velocity_factor, steering_factor and introduce planning_factor (`#9927 <https://github.com/autowarefoundation/autoware_universe/issues/9927>`_)
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: satoshi-ota <satoshi.ota928@gmail.com>
* fix(autoware_motion_velocity_obstacle_velocity_limiter_module): remove cppcheck suppressions (`#9843 <https://github.com/autowarefoundation/autoware_universe/issues/9843>`_)
* feat(motion_velocity_planner): introduce Object/Pointcloud structure in PlannerData (`#9812 <https://github.com/autowarefoundation/autoware_universe/issues/9812>`_)
  * feat: new object/pointcloud struct in motion velocity planner
  * update planner_data
  * modify modules
  * fix
  ---------
* fix(autoware_motion_velocity_obstacle_velocity_limiter_module): fix bugprone-exception-escape (`#9779 <https://github.com/autowarefoundation/autoware_universe/issues/9779>`_)
  * fix: bugprone-error
  * fix: cppcheck
  * fix: cpplint
  ---------
* feat(motion_velocity_planner): remove unnecessary tier4_planning_msgs dependency (`#9757 <https://github.com/autowarefoundation/autoware_universe/issues/9757>`_)
  * feat(motion_velocity_planner): remove unnecessary tier4_planning_msgs dependency
  * fix
  ---------
* feat(motion_velocity_planner): use Float64Stamped in autoware_internal_debug_msgs (`#9745 <https://github.com/autowarefoundation/autoware_universe/issues/9745>`_)
* Contributors: Fumiya Watanabe, Mamoru Sobue, Ryohsuke Mitsudome, Ryuta Kambe, Takayuki Murooka, kobayu858

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
* fix(cpplint): include what you use - planning (`#9570 <https://github.com/autowarefoundation/autoware_universe/issues/9570>`_)
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

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
* refactor(autoware_grid_map_utils): prefix folder structure with autoware/ (`#9170 <https://github.com/autowarefoundation/autoware_universe/issues/9170>`_)
* fix(obstacle_velocity_limiter): more stable virtual wall (`#8499 <https://github.com/autowarefoundation/autoware_universe/issues/8499>`_)
* feat(out_of_lane): redesign to improve accuracy and performance (`#8453 <https://github.com/autowarefoundation/autoware_universe/issues/8453>`_)
* chore(obstacle_velocity_limiter): add Alqudah Mohammad as codeowner (`#8516 <https://github.com/autowarefoundation/autoware_universe/issues/8516>`_)
* fix(autoware_motion_velocity_obstacle_velocity_limiter_module): fix functionStatic (`#8483 <https://github.com/autowarefoundation/autoware_universe/issues/8483>`_)
  fix:functionStatic
* fix(autoware_motion_velocity_obstacle_velocity_limiter_module): fix unreadVariable (`#8366 <https://github.com/autowarefoundation/autoware_universe/issues/8366>`_)
  * fix:unreadVariable
  * fix:unreadVariable
  ---------
* fix(autoware_motion_velocity_obstacle_velocity_limiter_module): fix uninitMemberVar (`#8314 <https://github.com/autowarefoundation/autoware_universe/issues/8314>`_)
  fix:funinitMemberVar
* fix(autoware_motion_velocity_obstacle_velocity_limiter_module): fix funcArgNamesDifferent (`#8025 <https://github.com/autowarefoundation/autoware_universe/issues/8025>`_)
  fix:funcArgNamesDifferent
* fix(autoware_motion_velocity_obstacle_velocity_limiter_module): fix shadowVariable (`#7977 <https://github.com/autowarefoundation/autoware_universe/issues/7977>`_)
  * fix:shadowVariable
  * fix:shadowVariable
  ---------
* perf(motion_velocity_planner): resample trajectory after vel smoothing (`#7732 <https://github.com/autowarefoundation/autoware_universe/issues/7732>`_)
  * perf(dynamic_obstacle_stop): create rtree with packing algorithm
  * Revert "perf(out_of_lane): downsample the trajectory to improve performance (`#7691 <https://github.com/autowarefoundation/autoware_universe/issues/7691>`_)"
  This reverts commit 8444a9eb29b32f500be3724dd5662013b9b81060.
  * perf(motion_velocity_planner): resample trajectory after vel smoothing
  ---------
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware_universe/issues/7640>`_)
* perf(out_of_lane): downsample the trajectory to improve performance (`#7691 <https://github.com/autowarefoundation/autoware_universe/issues/7691>`_)
* feat(motion_velocity_planner, lane_departure_checker): add processing time Float64 publishers (`#7683 <https://github.com/autowarefoundation/autoware_universe/issues/7683>`_)
* feat(motion_velocity_planner): publish processing times (`#7633 <https://github.com/autowarefoundation/autoware_universe/issues/7633>`_)
* fix(autoware_motion_velocity_obstacle_velocity_limiter_module): fix unreadVariable warning (`#7625 <https://github.com/autowarefoundation/autoware_universe/issues/7625>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(motion_velocity_planner): rename include directories (`#7523 <https://github.com/autowarefoundation/autoware_universe/issues/7523>`_)
* refactor(grid_map_utils): add autoware prefix and namespace (`#7487 <https://github.com/autowarefoundation/autoware_universe/issues/7487>`_)
* feat(obstacle_velocity_limiter): move to motion_velocity_planner (`#7439 <https://github.com/autowarefoundation/autoware_universe/issues/7439>`_)
* Contributors: Esteve Fernandez, Kosuke Takeuchi, Maxime CLEMENT, Ryuta Kambe, Takayuki Murooka, Yutaka Kondo, kobayu858

0.26.0 (2024-04-03)
-------------------
