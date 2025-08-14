^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_mission_planner_universe
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* fix(autoware_trajectory_follower, autoware_mission_planner_universe, autoware_scenario_selector): use transient_local for operation_mode_state (`#11101 <https://github.com/autowarefoundation/autoware_universe/issues/11101>`_)
  * subscribe operation-mode with transient_local
  * fix mistake
  * fix unit test code
  * pre-commit
  ---------
* feat(mission_planner): print set route api type (`#10884 <https://github.com/autowarefoundation/autoware_universe/issues/10884>`_)
* Contributors: Kem (TiankuiXian), Kosuke Takeuchi

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat!: replace tier4_planning_msgs service with autoware_planning_msgs (`#10827 <https://github.com/autowarefoundation/autoware_universe/issues/10827>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(mission_planner): print route state when set_route fails (`#10697 <https://github.com/autowarefoundation/autoware_universe/issues/10697>`_)
  * feat(mission_planner): print route state when set_route fails
  * Update planning/autoware_mission_planner_universe/src/mission_planner/mission_planner.cpp
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  * snake_case
  * without macro
  ---------
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
* fix(mission_planner): fix check if goal footprint is inside route (`#10681 <https://github.com/autowarefoundation/autoware_universe/issues/10681>`_)
  * fix checking if goal footprint is in ego lanes
  * change goal footprint check method
  * fix unit test
  ---------
* Contributors: Kosuke Takeuchi, Mitsuhiro Sakamoto, Ryohsuke Mitsudome, TaikiYamada4

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
* fix(autoware_mission_planner_universe): add explicit test dependency (`#10261 <https://github.com/autowarefoundation/autoware_universe/issues/10261>`_)
* Contributors: Hayato Mizushima, Mete Fatih Cırıt, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat(mission_planner): tolerate goal footprint being inside the previous lanelets of closest lanelet (`#10179 <https://github.com/autowarefoundation/autoware_universe/issues/10179>`_)
* feat(autoware_vehicle_info_utils): replace autoware_universe_utils with autoware_utils (`#10167 <https://github.com/autowarefoundation/autoware_universe/issues/10167>`_)
* Contributors: Fumiya Watanabe, Mamoru Sobue, Ryohsuke Mitsudome, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_mission_planner)!: feat(autoware_mission_planner_universe)!: add _universe suffix to package name (`#9941 <https://github.com/autowarefoundation/autoware_universe/issues/9941>`_)
* Contributors: Fumiya Watanabe, Ryohsuke Mitsudome

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
* fix: autoware_glog_compontnt (`#9586 <https://github.com/autowarefoundation/autoware_universe/issues/9586>`_)
  Fixed autoware_glog_compontnt
* fix(cpplint): include what you use - planning (`#9570 <https://github.com/autowarefoundation/autoware_universe/issues/9570>`_)
* refactor(glog_component): prefix package and namespace with autoware (`#9302 <https://github.com/autowarefoundation/autoware_universe/issues/9302>`_)
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
* fix(mission_planner): fix initialization after route set (`#9457 <https://github.com/autowarefoundation/autoware_universe/issues/9457>`_)
* fix(autoware_mission_planner): fix clang-diagnostic-error (`#9432 <https://github.com/autowarefoundation/autoware_universe/issues/9432>`_)
* feat(mission_planner): add processing time publisher (`#9342 <https://github.com/autowarefoundation/autoware_universe/issues/9342>`_)
  * feat(mission_planner): add processing time publisher
  * delete extra line
  * update: mission_planner, route_selector, service_utils.
  * Revert "update: mission_planner, route_selector, service_utils."
  This reverts commit d460a633c04c166385963c5233c3845c661e595e.
  * Update to show that exceptions are not handled
  * feat(mission_planner,route_selector): add processing time publisher
  ---------
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kazunori-Nakajima, M. Fatih Cırıt, Ryohsuke Mitsudome, Ryuta Kambe, SakodaShintaro, Takagi, Isamu, Yutaka Kondo

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
* feat(mission_planner): reroute with current route start pose when triggered by modifed goal (`#9136 <https://github.com/autowarefoundation/autoware_universe/issues/9136>`_)
  * feat(mission_planner): reroute with current route start pose when triggered by modifed goal
  * check new ego goal is in original preffered lane as much as possible
  * check goal is in goal_lane
  ---------
* fix(mission_planner): return without change_route if new route is empty  (`#9101 <https://github.com/autowarefoundation/autoware_universe/issues/9101>`_)
  fix(mission_planner): return if new route is empty without change_route
* chore(mission_planner): fix typo (`#9053 <https://github.com/autowarefoundation/autoware_universe/issues/9053>`_)
* test(mission_planner): add test of default_planner (`#9050 <https://github.com/autowarefoundation/autoware_universe/issues/9050>`_)
* test(mission_planner): add unit tests of utility functions (`#9011 <https://github.com/autowarefoundation/autoware_universe/issues/9011>`_)
* refactor(mission_planner): move anonymous functions to utils and add namespace (`#9012 <https://github.com/autowarefoundation/autoware_universe/issues/9012>`_)
  feat(mission_planner): move functions to utils and add namespace
* feat(mission_planner): add option to prevent rerouting in autonomous driving mode (`#8757 <https://github.com/autowarefoundation/autoware_universe/issues/8757>`_)
* feat(mission_planner): make the "goal inside lanes" function more robuts and add tests (`#8760 <https://github.com/autowarefoundation/autoware_universe/issues/8760>`_)
* fix(mission_planner): improve condition to check if the goal is within the lane (`#8710 <https://github.com/autowarefoundation/autoware_universe/issues/8710>`_)
* fix(autoware_mission_planner): fix unusedFunction (`#8642 <https://github.com/autowarefoundation/autoware_universe/issues/8642>`_)
  fix:unusedFunction
* fix(autoware_mission_planner): fix noConstructor (`#8505 <https://github.com/autowarefoundation/autoware_universe/issues/8505>`_)
  fix:noConstructor
* fix(autoware_mission_planner): fix funcArgNamesDifferent (`#8017 <https://github.com/autowarefoundation/autoware_universe/issues/8017>`_)
  fix:funcArgNamesDifferent
* feat(mission_planner): reroute in manual driving (`#7842 <https://github.com/autowarefoundation/autoware_universe/issues/7842>`_)
  * feat(mission_planner): reroute in manual driving
  * docs(mission_planner): update document
  * feat(mission_planner): fix operation mode state receiving check
  ---------
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware_universe/issues/7640>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(route_handler)!: rename to include/autoware/{package_name}  (`#7530 <https://github.com/autowarefoundation/autoware_universe/issues/7530>`_)
  refactor(route_handler)!: rename to include/autoware/{package_name}
* feat(mission_planner): rename to include/autoware/{package_name} (`#7513 <https://github.com/autowarefoundation/autoware_universe/issues/7513>`_)
  * feat(mission_planner): rename to include/autoware/{package_name}
  * feat(mission_planner): rename to include/autoware/{package_name}
  * feat(mission_planner): rename to include/autoware/{package_name}
  ---------
* feat(mission_planner): use polling subscriber (`#7447 <https://github.com/autowarefoundation/autoware_universe/issues/7447>`_)
* fix(route_handler): route handler overlap removal is too conservative (`#7156 <https://github.com/autowarefoundation/autoware_universe/issues/7156>`_)
  * add flag to enable/disable loop check in getLaneletSequence functions
  * implement function to get closest route lanelet based on previous closest lanelet
  * refactor DefaultPlanner::plan function
  * modify loop check logic in getLaneletSequenceUpTo function
  * improve logic in isEgoOutOfRoute function
  * fix format
  * check if prev lanelet is a goal lanelet in getLaneletSequenceUpTo function
  * separate function to update current route lanelet in planner manager
  * rename function and add docstring
  * modify functions extendNextLane and extendPrevLane to account for overlap
  * refactor function getClosestRouteLaneletFromLanelet
  * add route handler unit tests for overlapping route case
  * fix function getClosestRouteLaneletFromLanelet
  * format fix
  * move test map to autoware_test_utils
  ---------
* refactor(route_handler): route handler add autoware prefix (`#7341 <https://github.com/autowarefoundation/autoware_universe/issues/7341>`_)
  * rename route handler package
  * update packages dependencies
  * update include guards
  * update includes
  * put in autoware namespace
  * fix formats
  * keep header and source file name as before
  ---------
* refactor(mission_planner)!: add autoware prefix and namespace (`#7414 <https://github.com/autowarefoundation/autoware_universe/issues/7414>`_)
  * refactor(mission_planner)!: add autoware prefix and namespace
  * fix svg
  ---------
* Contributors: Fumiya Watanabe, Kosuke Takeuchi, Maxime CLEMENT, Takayuki Murooka, Yutaka Kondo, kobayu858, mkquda

0.26.0 (2024-04-03)
-------------------
