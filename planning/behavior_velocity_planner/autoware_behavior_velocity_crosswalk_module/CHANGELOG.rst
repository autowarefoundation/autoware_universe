^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_velocity_crosswalk_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(crosswalk): improve robustness to prevent stop decision from being canceled by transient noise (`#11027 <https://github.com/autowarefoundation/autoware_universe/issues/11027>`_)
  * feat(crosswalk): improve robustness to prevent stop decision from being canceled by transient noise
  * docs: readme
  * fix: small change
  ---------
* fix(crosswalk): incorrect stop point calculation (`#10922 <https://github.com/autowarefoundation/autoware_universe/issues/10922>`_)
* Contributors: Mete Fatih Cırıt, Satoshi OTA

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(crosswalk_module): add infomation to PlanningFactor topic in crosswalk_module (`#10759 <https://github.com/autowarefoundation/autoware_universe/issues/10759>`_)
  * chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)
  not sync github-release
  * feat(crosswalk_module): set infomation to PlanningFactor topic
  * feat(crosswalk_module): add test
  * wip: create test case
  * fix to parked vehicles stop
  * tidy up
  * fix cmake
  * Revert "chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)"
  This reverts commit 41f03b1e967cfe124197960e41ba7e920e4646aa.
  * apply some review, fix code style
  * apply some review for tests
  * fix out_of_rage error
  * fix: apply suggestions from code review
  * change safety factor detail and rename stuck vehicle to obstruction prevention
  * delete dead variable
  * fix some parameter name
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat(crosswalk_module): add parked vehicles stop feature (`#10667 <https://github.com/autowarefoundation/autoware_universe/issues/10667>`_)
* Contributors: Kotakku, Maxime CLEMENT, TaikiYamada4

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* feat(crosswalk): update stop position logic (`#10439 <https://github.com/autowarefoundation/autoware_universe/issues/10439>`_)
  * update stop pos logic
  * update doc
  * response to the review
  ---------
* fix(behavior_velocity_crosswalk_module): add missing header  (`#10552 <https://github.com/autowarefoundation/autoware_universe/issues/10552>`_)
* feat(behavior_velocity_planner): only wait for the required subscriptions (`#10546 <https://github.com/autowarefoundation/autoware_universe/issues/10546>`_)
* Contributors: Masaki Baba, TaikiYamada4, Takayuki Murooka, Yuki TAKAGI

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(crosswalk_module): consider objects on crosswalk when pedestrian traffic light is red (`#10332 <https://github.com/autowarefoundation/autoware_universe/issues/10332>`_)
* Contributors: Mehmet Dogru, Ryohsuke Mitsudome

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
* feat(Autoware_planning_factor_interface): replace tier4_msgs with autoware_internal_msgs (`#10204 <https://github.com/autowarefoundation/autoware_universe/issues/10204>`_)
* Contributors: Hayato Mizushima, Satoshi OTA, Yutaka Kondo, 心刚

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat(autoware_objects_of_interest_marker_interface): replace autoware_universe_utils with autoware_utils (`#10174 <https://github.com/autowarefoundation/autoware_universe/issues/10174>`_)
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
* docs(crosswalk): fix file add miss (`#10028 <https://github.com/autowarefoundation/autoware_universe/issues/10028>`_)
* docs(crosswalk): update ttc vs ttv docs (`#10025 <https://github.com/autowarefoundation/autoware_universe/issues/10025>`_)
* feat(crosswalk): update judgle time against the stopped objects (`#9988 <https://github.com/autowarefoundation/autoware_universe/issues/9988>`_)
* chore(crosswalk): port the same direction ignore block (`#9983 <https://github.com/autowarefoundation/autoware_universe/issues/9983>`_)
* feat(crosswalk): add pass marker (`#9952 <https://github.com/autowarefoundation/autoware_universe/issues/9952>`_)
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
* fix: remove unnecessary parameters (`#9935 <https://github.com/autowarefoundation/autoware_universe/issues/9935>`_)
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
* Contributors: Fumiya Watanabe, Mamoru Sobue, Satoshi OTA, Takayuki Murooka, Yuki TAKAGI

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
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(crosswalk)!: delete wide crosswalk corresponding function (`#9329 <https://github.com/autowarefoundation/autoware_universe/issues/9329>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(crosswalk): don't use vehicle stop checker to remove unnecessary callback (`#9234 <https://github.com/autowarefoundation/autoware_universe/issues/9234>`_)
* test(crosswalk): add unit test (`#9228 <https://github.com/autowarefoundation/autoware_universe/issues/9228>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Mamoru Sobue, Ryohsuke Mitsudome, Satoshi OTA, Yuki TAKAGI, Yutaka Kondo

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
* fix(crosswalk): don't use vehicle stop checker to remove unnecessary callback (`#9234 <https://github.com/autowarefoundation/autoware_universe/issues/9234>`_)
* test(crosswalk): add unit test (`#9228 <https://github.com/autowarefoundation/autoware_universe/issues/9228>`_)
* Contributors: Esteve Fernandez, Satoshi OTA, Yuki TAKAGI, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(autoware_grid_map_utils): prefix folder structure with autoware/ (`#9170 <https://github.com/autowarefoundation/autoware_universe/issues/9170>`_)
* fix(crosswalk): fix occlusion detection range calculation and add debug markers (`#9121 <https://github.com/autowarefoundation/autoware_universe/issues/9121>`_)
* fix(crosswalk): fix passing direction calclation for the objects (`#9071 <https://github.com/autowarefoundation/autoware_universe/issues/9071>`_)
* fix(crosswalk): change exceptional handling (`#8956 <https://github.com/autowarefoundation/autoware_universe/issues/8956>`_)
* refactor(autoware_interpolation): prefix package and namespace with autoware (`#8088 <https://github.com/autowarefoundation/autoware_universe/issues/8088>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(crosswalk)!: update stop position caluculation (`#8853 <https://github.com/autowarefoundation/autoware_universe/issues/8853>`_)
* feat(crosswalk): suppress restart when the ego is close to the next stop point (`#8817 <https://github.com/autowarefoundation/autoware_universe/issues/8817>`_)
  * feat(crosswalk): suppress restart when the ego is close to the next stop point
  * update
  * add comment
  ---------
* fix(behavior_velocity_planner): align the parameters with launcher (`#8791 <https://github.com/autowarefoundation/autoware_universe/issues/8791>`_)
  parameters in behavior_velocity_planner aligned
* fix(autoware_behavior_velocity_crosswalk_module): fix unusedFunction (`#8665 <https://github.com/autowarefoundation/autoware_universe/issues/8665>`_)
  fix:unusedFunction
* fix(crosswalk): fix findEgoPassageDirectionAlongPath finding front and back point logic (`#8459 <https://github.com/autowarefoundation/autoware_universe/issues/8459>`_)
  * fix(crosswalk): fix findEgoPassageDirectionAlongPath finding front and back point logic
  * define ego_crosswalk_passage_direction later
  ---------
* fix(behavior_velocity_planner): fix cppcheck warnings of virtualCallInConstructor (`#8376 <https://github.com/autowarefoundation/autoware_universe/issues/8376>`_)
  Co-authored-by: Ryuta Kambe <ryuta.kambe@tier4.jp>
* fix(autoware_behavior_velocity_crosswalk_module): fix passedByValue (`#8210 <https://github.com/autowarefoundation/autoware_universe/issues/8210>`_)
  * fix:passedByValue
  * fix:passedByValue
  ---------
* refactor(crosswalk): clean up the structure and create a brief flowchart (`#7868 <https://github.com/autowarefoundation/autoware_universe/issues/7868>`_)
  * refactor(crosswalk): clean up the structure and create a brief flowchart
  * update
  * fix
  * static stop pose -> default stop pose
  ---------
* fix(autoware_behavior_velocity_crosswalk_module): fix shadowVariable (`#7974 <https://github.com/autowarefoundation/autoware_universe/issues/7974>`_)
  * fix:shadowVariable
  * fix:shadowVariable
  ---------
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware_universe/issues/7640>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* chore(behavior_velocity_planner): move packages (`#7526 <https://github.com/autowarefoundation/autoware_universe/issues/7526>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kosuke Takeuchi, Maxime CLEMENT, Mehmet Dogru, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, Zhe Shen, kobayu858, taisa1

0.26.0 (2024-04-03)
-------------------
