^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_path_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* chore(default_adapi): rename package (`#10756 <https://github.com/autowarefoundation/autoware_universe/issues/10756>`_)
* feat(hazard_lights_selector): add a hazard lights selector package (`#10692 <https://github.com/autowarefoundation/autoware_universe/issues/10692>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* fix(lane_change, behavior_path_planner): fix failure to do lane change (`#10694 <https://github.com/autowarefoundation/autoware_universe/issues/10694>`_)
  * chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)
  not sync github-release
  * set initial state of LC module to waiting approval, update RTC status when no valid path
  * store deleted modules ptrs in vector
  * revert unnecessary changes
  * Revert "chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)"
  This reverts commit 871a8540ade845c7c9a193029d407b411a4d685b.
  * fix format
  * Update planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: GitHub Action <action@github.com>
  Co-authored-by: Mamoru Sobue <mamoru.sobue@tier4.jp>
* fix(planning): fix links in documentations (`#10704 <https://github.com/autowarefoundation/autoware_universe/issues/10704>`_)
  * fix(planning): fix links in documentations
  * fix pre-commit
  ---------
* feat(autoware_behavior_path_bidirectional_traffic_module): add a functionality for bidirectional traffic (`#10394 <https://github.com/autowarefoundation/autoware_universe/issues/10394>`_)
  * add bidirectional traffic module
  * first version
  * add virtual wall
  * WIP
  * apply new message type
  * revert autoware_trajectory change
  * WIP
  * developping...
  * fix bug
  * update module
  * remove stopping in BackToNormalLane mode
  * fix bug maybe
  * tidy
  * add document
  * rename images
  * add test, tidy.
  * catch up with latest branch
  * fix
  * style(pre-commit): autofix
  * update
  * downsizing svg files
  * fix
  * fix
  * tidy
  * fix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Makoto Kurihara, TaikiYamada4, Takagi, Isamu, Yukinari Hisaki, Yuxuan Liu, mkquda

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* fix(bpp): remove unnecessary enable_module in sampling_planner (`#7038 <https://github.com/autowarefoundation/autoware_universe/issues/7038>`_)
* refactor(departure_checker): move lane departure checker class to departure_checker  (`#10337 <https://github.com/autowarefoundation/autoware_universe/issues/10337>`_)
  * RT1-9640: separate lane departure checker library
  * move back parameter
  * separating parameters
  * renamed to boundary departure checker
  * pre-commit
  * remove trajectory deviation
  * rename namespace
  * move boundary departure checker to common folder
  * rename class name
  ---------
* Contributors: Shumpei Wakabayashi, TaikiYamada4, Zulfaqar Azmi

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(behavior_path_planner, behavior_static_obstacle_avoidance_module): crash during goal changes (`#10205 <https://github.com/autowarefoundation/autoware_universe/issues/10205>`_)
  * fix(behavior_path_planner, behavior_static_obstacle_avoidance_module): empty path handling
  * style(pre-commit): autofix
  * refactor: use optional
  * fix: std
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Shumpei Wakabayashi

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* feat!: replace VelocityLimit messages with autoware_internal_planning_msgs (`#10273 <https://github.com/autowarefoundation/autoware_universe/issues/10273>`_)
* feat: adaption to ROS nodes guidelines about directory structure (`#10268 <https://github.com/autowarefoundation/autoware_universe/issues/10268>`_)
* feat(behavior_path_planner_common): modify drivable area expansion to be able to avoid static objects (`#10220 <https://github.com/autowarefoundation/autoware_universe/issues/10220>`_)
  * modify drivable area expansion to avoid static objects
  * rename parameters and update drivable area design md
  * Update planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_drivable_area_design.md
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * correct parameters description
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* fix(planning, control): reuse stamp of subscribed topic to measure component latency (`#10201 <https://github.com/autowarefoundation/autoware_universe/issues/10201>`_)
  * fix(behavior_velocity_planner): reuse timestamp of recieved path
  * fix(behavior_path_planner): check timestamp first in timer driven callback
  * fix(trajectory_follower_node): check timestamp first in timer driven callback
  * fix(vehicle_cmd_gate): reuse timestamp of recieved path
  ---------
* Contributors: Hayato Mizushima, NorahXiong, Ryohsuke Mitsudome, Satoshi OTA, Yutaka Kondo, mkquda

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat!: replace scenario msg from tier4_planning_msgs to autoware_internal_planning_msgs (`#10180 <https://github.com/autowarefoundation/autoware_universe/issues/10180>`_)
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
* feat(start_planner): visualize planner evaluation table in rviz (`#10029 <https://github.com/autowarefoundation/autoware_universe/issues/10029>`_)
  visualize planner evaluation table in rviz
* feat(autoware_planning_test_manager): remove dependency of tier4_planning_msgs::msg::LateralOffset (`#9967 <https://github.com/autowarefoundation/autoware_universe/issues/9967>`_)
  * feat(autoware_planning_test_manager): remove dependency of tier4_planning_msgs::msg::LateralOffset
  * fix
  ---------
* refactor(behavior_path_planner): common test functions (`#9963 <https://github.com/autowarefoundation/autoware_universe/issues/9963>`_)
  * feat: common test code in behavior_path_planner
  * deal with other modules
  * fix typo
  * update
  ---------
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
* fix(planning): text revisions (`#9886 <https://github.com/autowarefoundation/autoware_universe/issues/9886>`_)
  * fix(planning): text revisions
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* docs(bpp): revise explanation for Failure modules (`#9863 <https://github.com/autowarefoundation/autoware_universe/issues/9863>`_)
* feat(behavior_path_planner): use autoware internal stamped messages (`#9750 <https://github.com/autowarefoundation/autoware_universe/issues/9750>`_)
  * feat(behavior_path_planner): use autoware internal stamped messages
  * fix universe_utils
  ---------
* Contributors: Atto Armoo, Fumiya Watanabe, Kyoichi Sugahara, Mamoru Sobue, Satoshi OTA, Takayuki Murooka, Zulfaqar Azmi

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
* fix(bpp)!: remove stop reason (`#9449 <https://github.com/autowarefoundation/autoware_universe/issues/9449>`_)
  fix(bpp): remove stop reason
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(bpp): add velocity interface (`#9344 <https://github.com/autowarefoundation/autoware_universe/issues/9344>`_)
  * feat(bpp): add velocity interface
  * fix(adapi): subscribe additional velocity factors
  ---------
* refactor(factor): move steering factor interface to motion utils (`#9337 <https://github.com/autowarefoundation/autoware_universe/issues/9337>`_)
* refactor(bpp): rework steering factor interface (`#9325 <https://github.com/autowarefoundation/autoware_universe/issues/9325>`_)
  * refactor(bpp): rework steering factor interface
  * refactor(soa): rework steering factor interface
  * refactor(AbLC): rework steering factor interface
  * refactor(doa): rework steering factor interface
  * refactor(lc): rework steering factor interface
  * refactor(gp): rework steering factor interface
  * refactor(sp): rework steering factor interface
  * refactor(sbp): rework steering factor interface
  * refactor(ss): rework steering factor interface
  ---------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Satoshi OTA, Yutaka Kondo

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
* fix(autoware_behavior_path_planner): fix cppcheck unusedVariable (`#9193 <https://github.com/autowarefoundation/autoware_universe/issues/9193>`_)
* fix(behavior_path_planner): suppress reseting root lanelet (`#9129 <https://github.com/autowarefoundation/autoware_universe/issues/9129>`_)
  fix(behavior_path_planner): suppress resseting root lanelet
* refactor(object_recognition_utils): add autoware prefix to object_recognition_utils (`#8946 <https://github.com/autowarefoundation/autoware_universe/issues/8946>`_)
* test(bpp_common): add test for object related functions (`#9062 <https://github.com/autowarefoundation/autoware_universe/issues/9062>`_)
  * add test for object related functions
  * use EXPECT_DOUBLE_EQ instead of EXPECT_NEAR
  * fix build error
  ---------
* refactor(autoware_interpolation): prefix package and namespace with autoware (`#8088 <https://github.com/autowarefoundation/autoware_universe/issues/8088>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(signal_processing): prefix package and namespace with autoware (`#8541 <https://github.com/autowarefoundation/autoware_universe/issues/8541>`_)
* chore(planning): consistent parameters with autoware_launch (`#8915 <https://github.com/autowarefoundation/autoware_universe/issues/8915>`_)
  * chore(planning): consistent parameters with autoware_launch
  * update
  * fix json schema
  ---------
* fix(autoware_behavior_path_planner): fix syntaxError (`#8834 <https://github.com/autowarefoundation/autoware_universe/issues/8834>`_)
  fix:syntaxError
* fix(autoware_behavior_path_planner): align the parameters with launcher (`#8790 <https://github.com/autowarefoundation/autoware_universe/issues/8790>`_)
  parameters in behavior_path_planner aligned
* refactor(behavior_path_planner): planner data parameter initializer function (`#8767 <https://github.com/autowarefoundation/autoware_universe/issues/8767>`_)
* chore(autoware_default_adapi)!: prefix autoware to package name (`#8533 <https://github.com/autowarefoundation/autoware_universe/issues/8533>`_)
* fix(docs): fix dead links in behavior path planner manager (`#8309 <https://github.com/autowarefoundation/autoware_universe/issues/8309>`_)
  * fix dead links
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(behavior_path_planner, spellchecks): spell checks in behavior path planner (`#8307 <https://github.com/autowarefoundation/autoware_universe/issues/8307>`_)
  * fix spell checks in behavior path planner
  * try re-routable
  ---------
* feat(behavior_path _planner): divide planner manager modules into dependent slots (`#8117 <https://github.com/autowarefoundation/autoware_universe/issues/8117>`_)
* fix(behavior_path_planner_common): fix dynamic drivable area expansion with few input bound points (`#8136 <https://github.com/autowarefoundation/autoware_universe/issues/8136>`_)
* refactor(autoware_universe_utils): changed the API to be more intuitive and added documentation (`#7443 <https://github.com/autowarefoundation/autoware_universe/issues/7443>`_)
  * refactor(tier4_autoware_utils): Changed the API to be more intuitive and added documentation.
  * use raw shared ptr in PollingPolicy::NEWEST
  * update
  * fix
  * Update evaluator/autoware_control_evaluator/include/autoware/control_evaluator/control_evaluator_node.hpp
  Co-authored-by: danielsanchezaran <daniel.sanchez@tier4.jp>
  ---------
  Co-authored-by: danielsanchezaran <daniel.sanchez@tier4.jp>
* feat(autoware_behavior_path_planner): prevent infinite loop in approving scene module process (`#7881 <https://github.com/autowarefoundation/autoware_universe/issues/7881>`_)
  * prevent infinite loop
  * calculate max_iteration_num from number of scene modules
  * add doxygen explanation for calculateMaxIterationNum
  ---------
* feat(autoware_behavior_path_planner_common,autoware_behavior_path_lane_change_module): add time_keeper to bpp (`#8004 <https://github.com/autowarefoundation/autoware_universe/issues/8004>`_)
  * feat(autoware_behavior_path_planner_common,autoware_behavior_path_lane_change_module): add time_keeper to bpp
  * update
  ---------
* feat(autoware_behavior_path_planner): remove max_module_size param (`#7764 <https://github.com/autowarefoundation/autoware_universe/issues/7764>`_)
  * feat(behavior_path_planner): remove max_module_size param
  The max_module_size param has been removed from the behavior_path_planner scene_module_manager.param.yaml file. This param was unnecessary and has been removed to simplify the configuration.
  ---------
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware_universe/issues/7640>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(behaivor_path_planner)!: rename to include/autoware/{package_name} (`#7522 <https://github.com/autowarefoundation/autoware_universe/issues/7522>`_)
  * refactor(behavior_path_planner)!: make autoware dir in include
  * refactor(start_planner): make autoware include dir
  * refactor(goal_planner): make autoware include dir
  * sampling planner module
  * fix sampling planner build
  * dynamic_avoidance
  * lc
  * side shift
  * autoware_behavior_path_static_obstacle_avoidance_module
  * autoware_behavior_path_planner_common
  * make behavior_path dir
  * pre-commit
  * fix pre-commit
  * fix build
  ---------
* Contributors: Esteve Fernandez, Go Sakayori, Kosuke Takeuchi, Kyoichi Sugahara, Mamoru Sobue, Maxime CLEMENT, Ryuta Kambe, Takagi, Isamu, Takayuki Murooka, Yukinari Hisaki, Yutaka Kondo, Yuxuan Liu, Zhe Shen, kobayu858

0.26.0 (2024-04-03)
-------------------
