^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_dynamic_obstacle_stop_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Mete Fatih Cırıt

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(autoware_motion_velocity_planner): only wait for required subscriptions (`#10732 <https://github.com/autowarefoundation/autoware_universe/issues/10732>`_)
* Contributors: Ryohsuke Mitsudome, TaikiYamada4

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* fix(motion_velocity_planner): add missing header (`#10560 <https://github.com/autowarefoundation/autoware_universe/issues/10560>`_)
* Contributors: Masaki Baba, TaikiYamada4

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(pred_path_checker,dyn_obstacle_stop): use output of removeOverlaps (`#10375 <https://github.com/autowarefoundation/autoware_universe/issues/10375>`_)
* chore(motion_velocity_planner): move common and node packages to core (`#10367 <https://github.com/autowarefoundation/autoware_universe/issues/10367>`_)
* Contributors: Maxime CLEMENT, Ryohsuke Mitsudome

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* fix(dynamic_obstacle_stop): publish processing time when early return (`#10254 <https://github.com/autowarefoundation/autoware_universe/issues/10254>`_)
* Contributors: Hayato Mizushima, Maxime CLEMENT, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* fix: add missing includes to autoware_universe_utils (`#10091 <https://github.com/autowarefoundation/autoware_universe/issues/10091>`_)
* feat(motion_velocity_planner): common implementation for motion_velocity_obstacle\_<stop/slow_down/cruise>_module (`#10035 <https://github.com/autowarefoundation/autoware_universe/issues/10035>`_)
  * feat(motion_velocity_planner): prepare for motion_velocity\_<stop/slow_down/cruise>_module
  * update launch
  ---------
* Contributors: Fumiya Watanabe, Ryohsuke Mitsudome, Takayuki Murooka, 心刚

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
* chore(planning): move package directory for planning factor interface (`#9948 <https://github.com/autowarefoundation/autoware_universe/issues/9948>`_)
  * chore: add new package for planning factor interface
  * chore(surround_obstacle_checker): update include file
  * chore(obstacle_stop_planner): update include file
  * chore(obstacle_cruise_planner): update include file
  * chore(motion_velocity_planner): update include file
  * chore(bpp): update include file
  * chore(bvp-common): update include file
  * chore(blind_spot): update include file
  * chore(crosswalk): update include file
  * chore(detection_area): update include file
  * chore(intersection): update include file
  * chore(no_drivable_area): update include file
  * chore(no_stopping_area): update include file
  * chore(occlusion_spot): update include file
  * chore(run_out): update include file
  * chore(speed_bump): update include file
  * chore(stop_line): update include file
  * chore(template_module): update include file
  * chore(traffic_light): update include file
  * chore(vtl): update include file
  * chore(walkway): update include file
  * chore(motion_utils): remove factor interface
  ---------
* feat(planning_factor)!: remove velocity_factor, steering_factor and introduce planning_factor (`#9927 <https://github.com/autowarefoundation/autoware_universe/issues/9927>`_)
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: satoshi-ota <satoshi.ota928@gmail.com>
* feat(motion_velocity_planner): introduce Object/Pointcloud structure in PlannerData (`#9812 <https://github.com/autowarefoundation/autoware_universe/issues/9812>`_)
  * feat: new object/pointcloud struct in motion velocity planner
  * update planner_data
  * modify modules
  * fix
  ---------
* feat(motion_velocity_planner): remove unnecessary tier4_planning_msgs dependency (`#9757 <https://github.com/autowarefoundation/autoware_universe/issues/9757>`_)
  * feat(motion_velocity_planner): remove unnecessary tier4_planning_msgs dependency
  * fix
  ---------
* feat(motion_velocity_planner): use Float64Stamped in autoware_internal_debug_msgs (`#9745 <https://github.com/autowarefoundation/autoware_universe/issues/9745>`_)
* Contributors: Fumiya Watanabe, Mamoru Sobue, Ryohsuke Mitsudome, Satoshi OTA, Takayuki Murooka

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
* test(dynamic_obstacle_stop): add tests and do some refactoring (`#8250 <https://github.com/autowarefoundation/autoware_universe/issues/8250>`_)
* fix(autoware_motion_velocity_dynamic_obstacle_stop_module): fix funcArgNamesDifferent (`#8024 <https://github.com/autowarefoundation/autoware_universe/issues/8024>`_)
  fix:funcArgNamesDifferent
* perf(dynamic_obstacle_stop): construct rtree nodes in place (`#7753 <https://github.com/autowarefoundation/autoware_universe/issues/7753>`_)
* perf(dynamic_obstacle_stop): create rtree with packing algorithm (`#7730 <https://github.com/autowarefoundation/autoware_universe/issues/7730>`_)
* feat(motion_velocity_planner, lane_departure_checker): add processing time Float64 publishers (`#7683 <https://github.com/autowarefoundation/autoware_universe/issues/7683>`_)
* feat(motion_velocity_planner): publish processing times (`#7633 <https://github.com/autowarefoundation/autoware_universe/issues/7633>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(motion_velocity_planner): rename include directories (`#7523 <https://github.com/autowarefoundation/autoware_universe/issues/7523>`_)
* refactor(dynamic_obstacle_stop): move to motion_velocity_planner (`#7460 <https://github.com/autowarefoundation/autoware_universe/issues/7460>`_)
* Contributors: Kosuke Takeuchi, Maxime CLEMENT, Takayuki Murooka, Yutaka Kondo, kobayu858

0.26.0 (2024-04-03)
-------------------
