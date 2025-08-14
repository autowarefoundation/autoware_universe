^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_velocity_intersection_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(intersection): remove reference "traffic_light_id" as it is deprecated (`#11008 <https://github.com/autowarefoundation/autoware_universe/issues/11008>`_)
* fix(intersection): clamp collision stopline ip index (`#11009 <https://github.com/autowarefoundation/autoware_universe/issues/11009>`_)
* feat(intersection_occlusion): request approval when occluded without traffic light (`#10947 <https://github.com/autowarefoundation/autoware_universe/issues/10947>`_)
* feat(intersection_module): add infomation to PlanningFactor topic (`#10835 <https://github.com/autowarefoundation/autoware_universe/issues/10835>`_)
  * feat(intersection_module): add infomation to safety factor
  * feat(intersection_module): add planning_factor_interface_for_occlusion\_
  * feat(intersection_module): add planning factor test case
  * some code fix
  * add planning factor test case for occlusion
  * fix comment typo
  * marked planning_factor_interface_for_occlusion_as maybe_unused.
  * fix safety_factors and detail
  * fix test case and add comment
  * fix planning factor detail when decision is safe
  ---------
* feat(intersection): adjust collision stopline position to be ahead of ego by braking distance (`#10900 <https://github.com/autowarefoundation/autoware_universe/issues/10900>`_)
* fix(intersection): stop at occlusion stopline and stop twice at intersection without traffic light (`#10911 <https://github.com/autowarefoundation/autoware_universe/issues/10911>`_)
  feat(intersection): stop at occlusion stopline and stop twice at intersection without traffic light
* feat(intersection_occlusion): adjustable occlusion wall position for intersection without traffic_light (`#10868 <https://github.com/autowarefoundation/autoware_universe/issues/10868>`_)
* Contributors: Kotakku, Mamoru Sobue, Mete Fatih Cırıt, Yuxuan Liu

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* fix(intersection): fix path lanelets generation (`#10843 <https://github.com/autowarefoundation/autoware_universe/issues/10843>`_)
* feat(intersection): consider brake feasiblity for stuck vehicle (`#10742 <https://github.com/autowarefoundation/autoware_universe/issues/10742>`_)
* feat(intersection): experimental overshoot in merging (`#10544 <https://github.com/autowarefoundation/autoware_universe/issues/10544>`_)
* Contributors: Mamoru Sobue, TaikiYamada4

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* fix(intersection): interpolate object polygons precisely (`#10626 <https://github.com/autowarefoundation/autoware_universe/issues/10626>`_)
  fix(intersection): debug collision check
* fix(behavior_velocity_planner): add missing header (`#10561 <https://github.com/autowarefoundation/autoware_universe/issues/10561>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(behavior_velocity_planner): extend stop line to path bound (`#10435 <https://github.com/autowarefoundation/autoware_universe/issues/10435>`_)
  * extend stop line to path bound
  * style(pre-commit): autofix
  * change signature of stop line extension function
  * add tests for detection area module
  * fix test for no stopping area module
  * fix tests for traffic light module
  * include necessary header
  * fix test for detection area module
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(behavior_velocity_planner): only wait for the required subscriptions (`#10546 <https://github.com/autowarefoundation/autoware_universe/issues/10546>`_)
* Contributors: Mamoru Sobue, Masaki Baba, Mitsuhiro Sakamoto, TaikiYamada4, Takayuki Murooka

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
* fix(behavior_velocity_planner): planning factor integration (`#10292 <https://github.com/autowarefoundation/autoware_universe/issues/10292>`_)
  * fix: blind_spot
  * fix: crosswalk
  * fix: detection_area
  * fix: intersection
  * fix: no_drivable_lane
  * fix: no_stopping_area
  * fix: run_out
  * fix: stop_line
  * fix: traffic_light
  * fix: virtual_traffic_light
  * fix: walk_way
  ---------
* feat: apply splitting of autoware_utils_geometry  (`#10270 <https://github.com/autowarefoundation/autoware_universe/issues/10270>`_)
  * fix build error
  * merge namespace
  ---------
* feat(Autoware_planning_factor_interface): replace tier4_msgs with autoware_internal_msgs (`#10204 <https://github.com/autowarefoundation/autoware_universe/issues/10204>`_)
* Contributors: Hayato Mizushima, Satoshi OTA, Takagi, Isamu, Yutaka Kondo, 心刚

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat(autoware_objects_of_interest_marker_interface): replace autoware_universe_utils with autoware_utils (`#10174 <https://github.com/autowarefoundation/autoware_universe/issues/10174>`_)
* feat: remove dependency on autoware_universe_utils from autoware_interpolation and autoware_motion_utils (`#10092 <https://github.com/autowarefoundation/autoware_universe/issues/10092>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix: add missing includes to autoware_universe_utils (`#10091 <https://github.com/autowarefoundation/autoware_universe/issues/10091>`_)
* feat!: replace tier4_planning_msgs/PathWithLaneId with autoware_internal_planning_msgs/PathWithLaneId (`#10023 <https://github.com/autowarefoundation/autoware_universe/issues/10023>`_)
* feat(planning_test_manager): abstract message-specific functions (`#9882 <https://github.com/autowarefoundation/autoware_universe/issues/9882>`_)
  * abstract message-specific functions
  * include necessary header
  * adapt velocity_smoother to new test manager
  * adapt behavior_velocity_planner to new test manager
  * adapt path_optimizer to new test manager
  * fix output subscription
  * adapt behavior_path_planner to new test manager
  * adapt scenario_selector to new test manager
  * adapt freespace_planner to new test manager
  * adapt planning_validator to new test manager
  * adapt obstacle_stop_planner to new test manager
  * adapt obstacle_cruise_planner to new test manager
  * disable test for freespace_planner
  * adapt behavior_velocity_crosswalk_module to new test manager
  * adapt behavior_path_lane_change_module to new test manager
  * adapt behavior_path_avoidance_by_lane_change_module to new test manager
  * adapt behavior_path_dynamic_obstacle_avoidance_module to new test manager
  * adapt behavior_path_external_request_lane_change_module to new test manager
  * adapt behavior_path_side_shift_module to new test manager
  * adapt behavior_path_static_obstacle_avoidance_module to new test manager
  * adapt path_smoother to new test manager
  * adapt behavior_velocity_blind_spot_module to new test manager
  * adapt behavior_velocity_detection_area_module to new test manager
  * adapt behavior_velocity_intersection_module to new test manager
  * adapt behavior_velocity_no_stopping_area_module to new test manager
  * adapt behavior_velocity_run_out_module to new test manager
  * adapt behavior_velocity_stop_line_module to new test manager
  * adapt behavior_velocity_traffic_light_module to new test manager
  * adapt behavior_velocity_virtual_traffic_light_module to new test manager
  * adapt behavior_velocity_walkway_module to new test manager
  * adapt motion_velocity_planner_node_universe to new test manager
  * include necessary headers
  * Odometries -> Odometry
  ---------
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
* Contributors: Fumiya Watanabe, Mitsuhiro Sakamoto, Ryohsuke Mitsudome, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(intersection): add wall marker for too late detect objects (`#10006 <https://github.com/autowarefoundation/autoware_universe/issues/10006>`_)
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
* feat(behavior_velocity_planner)!: remove velocity_factor completely (`#9943 <https://github.com/autowarefoundation/autoware_universe/issues/9943>`_)
  * feat(behavior_velocity_planner)!: remove velocity_factor completely
  * minimize diff
  ---------
* feat(planning_factor)!: remove velocity_factor, steering_factor and introduce planning_factor (`#9927 <https://github.com/autowarefoundation/autoware_universe/issues/9927>`_)
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: satoshi-ota <satoshi.ota928@gmail.com>
* feat(behavior_velocity_modules): add node test (`#9790 <https://github.com/autowarefoundation/autoware_universe/issues/9790>`_)
  * feat(behavior_velocity_crosswalk): add node test
  * fix
  * feat(behavior_velocity_xxx_module): add node test
  * fix
  * fix
  * fix
  * fix
  * change directory tests -> test
  ---------
* fix(autoware_behavior_velocity_intersection_module): fix bugprone-branch-clone (`#9702 <https://github.com/autowarefoundation/autoware_universe/issues/9702>`_)
  fix: bugprone-error
* refactor(behavior_velocity_planner_common): add behavior_velocity_rtc_interface and move RTC-related implementation (`#9799 <https://github.com/autowarefoundation/autoware_universe/issues/9799>`_)
  * split into planer_common and rtc_interface
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/autoware/behavior_velocity_planner_common/scene_module_interface.hpp
  Co-authored-by: Mamoru Sobue <mamoru.sobue@tier4.jp>
  * Update planning/behavior_velocity_planner/autoware_behavior_velocity_rtc_interface/include/autoware/behavior_velocity_rtc_interface/scene_module_interface_with_rtc.hpp
  Co-authored-by: Mamoru Sobue <mamoru.sobue@tier4.jp>
  * fix
  ---------
  Co-authored-by: Mamoru Sobue <mamoru.sobue@tier4.jp>
* feat(behavior_velocity_planner): use XXXStamped in autoware_internal_debug_msgs (`#9744 <https://github.com/autowarefoundation/autoware_universe/issues/9744>`_)
  * feat(behavior_velocity_planner): use XXXStamped in autoware_internal_debug_msgs
  * fix
  ---------
* feat(behavior_velocity_planner): remove unnecessary tier4_api_msgs (`#9692 <https://github.com/autowarefoundation/autoware_universe/issues/9692>`_)
* Contributors: Fumiya Watanabe, Mamoru Sobue, Satoshi OTA, Takayuki Murooka, kobayu858

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
* feat(behavior_velocity_planner)!: remove stop_reason (`#9452 <https://github.com/autowarefoundation/autoware_universe/issues/9452>`_)
* refactor: correct spelling (`#9528 <https://github.com/autowarefoundation/autoware_universe/issues/9528>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix(autoware_behavior_velocity_intersection_module): fix clang-diagnostic-unused-parameter (`#9409 <https://github.com/autowarefoundation/autoware_universe/issues/9409>`_)
  fix: clang-diagnostic-unused-parameter
* fix(autoware_behavior_velocity_intersection_module): fix clang-diagnostic-unused-lambda-capture (`#9407 <https://github.com/autowarefoundation/autoware_universe/issues/9407>`_)
  fix: clang-diagnostic-unused-parameter
* chore(autoware_behavior_velocity_intersection_module): include opencv as system (`#9330 <https://github.com/autowarefoundation/autoware_universe/issues/9330>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(bvp): remove expired module safely (`#9212 <https://github.com/autowarefoundation/autoware_universe/issues/9212>`_)
  * fix(bvp): remove expired module safely
  * fix: remove module id set
  * fix: use itr to erase expired module
  * fix: remove unused function
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Mamoru Sobue, Ryohsuke Mitsudome, Satoshi OTA, Yukinari Hisaki, Yutaka Kondo, kobayu858

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(bvp): remove expired module safely (`#9212 <https://github.com/autowarefoundation/autoware_universe/issues/9212>`_)
  * fix(bvp): remove expired module safely
  * fix: remove module id set
  * fix: use itr to erase expired module
  * fix: remove unused function
  ---------
* Contributors: Esteve Fernandez, Satoshi OTA, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* chore(intersection): print RTC status in diagnostic debug message (`#9007 <https://github.com/autowarefoundation/autoware_universe/issues/9007>`_)
  debug(intersection): print RTC status in diagnostic message
* fix(behavior_path_planner, behavior_velocity_planner): fix to not read invalid ID (`#9103 <https://github.com/autowarefoundation/autoware_universe/issues/9103>`_)
  * fix(behavior_path_planner, behavior_velocity_planner): fix to not read invalid ID
  * style(pre-commit): autofix
  * fix typo
  * fix(behavior_path_planner, behavior_velocity_planner): fix typo and indentation
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(intersection): handle pass judge after red/arrow-signal to ignore NPCs after the signal changed to green again (`#9119 <https://github.com/autowarefoundation/autoware_universe/issues/9119>`_)
* fix(intersection): set RTC enable (`#9040 <https://github.com/autowarefoundation/autoware_universe/issues/9040>`_)
  set rtc enable
* fix(interpolation): fix bug of interpolation (`#8969 <https://github.com/autowarefoundation/autoware_universe/issues/8969>`_)
  fix bug of interpolation
* refactor(autoware_interpolation): prefix package and namespace with autoware (`#8088 <https://github.com/autowarefoundation/autoware_universe/issues/8088>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* fix(intersection): fix typo (`#8911 <https://github.com/autowarefoundation/autoware_universe/issues/8911>`_)
  * fix(intersection): fix typo
  * fix(intersection): fix typo
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(bvp): fix rtc state update logic (`#8884 <https://github.com/autowarefoundation/autoware_universe/issues/8884>`_)
  * fix(bvp): fix rtc state update logic
  * fix(intersection): fix unexpected rtc state initialization
  ---------
* fix(autoware_behavior_velocity_intersection_module): fix unusedFunction (`#8666 <https://github.com/autowarefoundation/autoware_universe/issues/8666>`_)
  * fix:unusedFunction
  * fix:unusedFunction
  ---------
* fix(autoware_behavior_velocity_intersection_module): fix unreadVariable (`#8836 <https://github.com/autowarefoundation/autoware_universe/issues/8836>`_)
  fix:unreadVariable
* fix(autoware_behavior_velocity_intersection_module): fix virtualCallInConstructor (`#8835 <https://github.com/autowarefoundation/autoware_universe/issues/8835>`_)
  fix:virtualCallInConstructor
* fix(behavior_velocity_planner): align the parameters with launcher (`#8791 <https://github.com/autowarefoundation/autoware_universe/issues/8791>`_)
  parameters in behavior_velocity_planner aligned
* fix(intersection): additional fix for 8520 (`#8561 <https://github.com/autowarefoundation/autoware_universe/issues/8561>`_)
* feat(intersection): fix topological sort for complicated intersection (`#8520 <https://github.com/autowarefoundation/autoware_universe/issues/8520>`_)
  * for enclave occlusion detection lanelet
  * some refactorings and modify doxygen
  * fix ci
  ---------
  Co-authored-by: Y.Hisaki <yhisaki31@gmail.com>
* fix(behavior_velocity_planner): fix cppcheck warnings of virtualCallInConstructor (`#8376 <https://github.com/autowarefoundation/autoware_universe/issues/8376>`_)
  Co-authored-by: Ryuta Kambe <ryuta.kambe@tier4.jp>
* feat(intersection): add test map for intersection (`#8455 <https://github.com/autowarefoundation/autoware_universe/issues/8455>`_)
* fix(autoware_smart_mpc_trajectory_follower): fix unusedStructMember (`#8393 <https://github.com/autowarefoundation/autoware_universe/issues/8393>`_)
  * fix:unusedStructMember
  * fix:unusedStructMember
  * fix:clang format
  ---------
* fix(autoware_behavior_velocity_intersection_module): fix functionConst (`#8283 <https://github.com/autowarefoundation/autoware_universe/issues/8283>`_)
  fix:functionConst
* fix(autoware_behavior_velocity_intersection_module): fix funcArgNamesDifferent (`#8023 <https://github.com/autowarefoundation/autoware_universe/issues/8023>`_)
  * fix:funcArgNamesDifferent
  * fix:funcArgNamesDifferent
  * refactor:clang format
  * fix:funcArgNamesDifferent
  ---------
* refactor(probabilistic_occupancy_grid_map, occupancy_grid_map_outlier_filter): add autoware\_ prefix to package name (`#8183 <https://github.com/autowarefoundation/autoware_universe/issues/8183>`_)
  * chore: fix package name probabilistic occupancy grid map
  * fix: solve launch error
  * chore: update occupancy_grid_map_outlier_filter
  * style(pre-commit): autofix
  * refactor: update package name to autoware_probabilistic_occupancy_grid_map on a test
  * refactor: rename folder of occupancy_grid_map_outlier_filter
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* fix(autoware_behavior_velocity_intersection_module): fix shadowVariable (`#7976 <https://github.com/autowarefoundation/autoware_universe/issues/7976>`_)
* fix(autoware_behavior_velocity_intersection_module): fix shadowFunction (`#7835 <https://github.com/autowarefoundation/autoware_universe/issues/7835>`_)
  * fix(autoware_behavior_velocity_intersection_module): fix shadowFunction
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware_universe/issues/7640>`_)
* refactor(behavior_velocity_intersection): apply clang-tidy check (`#7552 <https://github.com/autowarefoundation/autoware_universe/issues/7552>`_)
  intersection
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* chore(behavior_velocity_planner): move packages (`#7526 <https://github.com/autowarefoundation/autoware_universe/issues/7526>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, Go Sakayori, Kosuke Takeuchi, Mamoru Sobue, Ryuta Kambe, Satoshi OTA, T-Kimura-MM, Takayuki Murooka, Yoshi Ri, Yukinari Hisaki, Yutaka Kondo, Zhe Shen, kobayu858, taisa1

0.26.0 (2024-04-03)
-------------------
