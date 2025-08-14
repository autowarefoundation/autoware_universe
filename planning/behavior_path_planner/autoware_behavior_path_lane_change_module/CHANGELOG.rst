^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_path_lane_change_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* fix(lane_change): properly handle force activation when there is no valid LC path (`#11059 <https://github.com/autowarefoundation/autoware_universe/issues/11059>`_)
  * properly handle force activation when there is no valid LC path
  * invoke lambda in place
  ---------
* fix(lane_change): prevent cancel if ego is near terminal end (`#10980 <https://github.com/autowarefoundation/autoware_universe/issues/10980>`_)
* style(pre-commit): autofix (`#10982 <https://github.com/autowarefoundation/autoware_universe/issues/10982>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(lane_change): fix lc rtc update (`#10819 <https://github.com/autowarefoundation/autoware_universe/issues/10819>`_)
  * update rtc status when no valid path
  * Revert "chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)"
  This reverts commit 871a8540ade845c7c9a193029d407b411a4d685b.
  * reset lc module params when no valid path
  ---------
  Co-authored-by: GitHub Action <action@github.com>
* feat(lane_change): add countermeasures for preventing high curvature lc path (`#10951 <https://github.com/autowarefoundation/autoware_universe/issues/10951>`_)
  * add implementation for using entire remaining distance for frenet lc path
  * refactor code
  * update parameter decription
  * capture by value
  * fix for clang-tidy
  ---------
* fix(lane_change): add publish planning factor condition for abort and detail (`#10941 <https://github.com/autowarefoundation/autoware_universe/issues/10941>`_)
  * add publish planning factor condition for abort and detail
  * delete true condition for debugging
  ---------
* fix(lane_change): add curvature threshold for frenet planner (`#10920 <https://github.com/autowarefoundation/autoware_universe/issues/10920>`_)
  * fix(lane_change): add curvature threshold for frenet planner
  * Documentation
  * fix broken link
  * Add average curvature member variable
  * add curvature in metrics
  * rename paran
  ---------
* feat(lane_change): add infomation to PlanningFactor topic (`#10875 <https://github.com/autowarefoundation/autoware_universe/issues/10875>`_)
  * add infomation to planning_factor_interface\_
  * fix code sense analysis error
  * Revert "fix code sense analysis error"
  This reverts commit 2ff1288e58ce4004848cda5e5cec5d1b1d8426dd.
  * delete unnecessary files for tests.
  * delete dead functions for test.
  ---------
* fix(lane_change): fix code to prevent node crash (`#10866 <https://github.com/autowarefoundation/autoware_universe/issues/10866>`_)
  * fix comparator for sort algorithm
  * rearrange operands
  ---------
* Contributors: Kotakku, Ryohsuke Mitsudome, Zulfaqar Azmi, mkquda

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
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
* feat!: remove obstacle_stop_planner and obstacle_cruise_planner (`#10695 <https://github.com/autowarefoundation/autoware_universe/issues/10695>`_)
  * feat: remove obstacle_stop_planner and obstacle_cruise_planner
  * update
  * fix
  ---------
* Contributors: TaikiYamada4, Takayuki Murooka, mkquda

0.45.0 (2025-05-22)
-------------------

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(lane_change): set safety factor properly (`#10308 <https://github.com/autowarefoundation/autoware_universe/issues/10308>`_)
* Contributors: Ryohsuke Mitsudome, Satoshi OTA

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* fix(lane_change_module): fix planning factor issue (`#10244 <https://github.com/autowarefoundation/autoware_universe/issues/10244>`_)
  * when computing target lanes, don't include preceding lanes of lane change lane
  * dont insert stop point on target lane if next lc dist buffer is zero
  * return previous module output if LC module status is IDLE
  * disable faulty test
  ---------
* Contributors: Hayato Mizushima, Yutaka Kondo, mkquda

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* chore(lane_change, QC): remove unused function (`#10188 <https://github.com/autowarefoundation/autoware_universe/issues/10188>`_)
  chore(lane_change): remove unused function
* fix(lane_change): remove string literals from stopwatch toc (`#10121 <https://github.com/autowarefoundation/autoware_universe/issues/10121>`_)
* feat(lane_change): improve the calculation of the target lateral velocity in Frenet frame (`#10078 <https://github.com/autowarefoundation/autoware_universe/issues/10078>`_)
* fix(lane_change): fix end pose not connected to goal (RT1-9247) (`#10103 <https://github.com/autowarefoundation/autoware_universe/issues/10103>`_)
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
* Contributors: Fumiya Watanabe, Mamoru Sobue, Maxime CLEMENT, Mert Çolak, Mitsuhiro Sakamoto, Ryohsuke Mitsudome, Zulfaqar Azmi, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* docs(lane_change): update lane change documentation (`#9949 <https://github.com/autowarefoundation/autoware_universe/issues/9949>`_)
  * update lane change requirements documentation
  * remove unused function getNumToPreferredLane
  * update candidate path generation documentation
  * update prepare phase and lane changing phase documentation
  * update longitudinal acceleration sampling documentation
  * add prepare duration sampling documentation
  * update candidate path validity and safety documentation
  * fix formatting
  * update image and fix formatting
  * add overtaking turn lane documentation
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * add LC global flowchart to documentation
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  * reorganize lane change documentation
  * fix section title
  * add global flowchart description
  * add warning
  * apply pre-commit checks
  * fix spelling
  * edit some descriptions
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
* docs(lane_change): object filtering description (`#9947 <https://github.com/autowarefoundation/autoware_universe/issues/9947>`_)
  * docs(lane_change): object filtering description
  * Move section up
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  ---------
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
* refactor(behavior_path_planner): common test functions (`#9963 <https://github.com/autowarefoundation/autoware_universe/issues/9963>`_)
  * feat: common test code in behavior_path_planner
  * deal with other modules
  * fix typo
  * update
  ---------
* refactor(lane_change): add missing safety check parameter  (`#9928 <https://github.com/autowarefoundation/autoware_universe/issues/9928>`_)
  * refactor(lane_change): parameterize incoming object yaw threshold
  * Readme
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/manager.cpp
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * Add missing parameters
  * missing dot
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  * update readme
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
* feat(planning_factor)!: remove velocity_factor, steering_factor and introduce planning_factor (`#9927 <https://github.com/autowarefoundation/autoware_universe/issues/9927>`_)
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: satoshi-ota <satoshi.ota928@gmail.com>
* feat(lane_change): ensure path generation doesn't exceed time limit (`#9908 <https://github.com/autowarefoundation/autoware_universe/issues/9908>`_)
  * add time limit for lane change candidate path generation
  * apply time limit for frenet method as well
  * ensure param update value is valid
  * fix param update initial value
  * fix spelling
  * fix param update initial values
  ---------
* feat(lane_change_module): add update paramter function for non defined paramters (`#9887 <https://github.com/autowarefoundation/autoware_universe/issues/9887>`_)
  * feat(lane_change_module): add new parameters for collision check and delay lane change functionality
  * feat(lane_change_module): add validation for longitudinal and lateral acceleration sampling parameters
  * feat(lane_change): update parameter handling and add lateral acceleration mapping
  ---------
* feat(lane_change): using frenet planner to generate lane change path when ego near terminal (`#9767 <https://github.com/autowarefoundation/autoware_universe/issues/9767>`_)
  * frenet planner
  * minor refactoring
  * adding parameter
  * Add diff th param
  * limit curvature for prepare segment
  * minor refactoring
  * print average curvature
  * refactor
  * filter the path directly
  * fix some conflicts
  * include curvature smoothing
  * document
  * fix image folder
  * image size
  * doxygen
  * add debug for state
  * use sign function instead
  * rename argument
  * readme
  * fix failed test due to empty value
  * add additional note
  * fix conflict
  ---------
* feat(lane_change): append candidate path index to metric debug table (`#9885 <https://github.com/autowarefoundation/autoware_universe/issues/9885>`_)
  add candidate path index to metrics debug table
* docs(lane_change): fix broken link (`#9892 <https://github.com/autowarefoundation/autoware_universe/issues/9892>`_)
* docs(lane_change): explaining cancel and abort process (`#9845 <https://github.com/autowarefoundation/autoware_universe/issues/9845>`_)
  * docs(lane_change): explaining cancel and abort process
  * slight fix in formatting
  * rephrase sentence
  * rephrase and replace image for cancel
  * Cancel explanations and limitations
  * revise abort figure
  * revise flow chart
  * rephase sentence
  * minor fix
  * finish up
  * offers change to reduces for negative connotation
  * minor fix
  * move limitation all the way down
  * precommit
  * equation mistake
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * rename subheading
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* refactor(lane_change): refactor transit failure function (`#9835 <https://github.com/autowarefoundation/autoware_universe/issues/9835>`_)
  * refactor(lane_change): refactor transit failure function
  * fixed failed scenario
  * remove is abort from debug
  * set is abort state
  * add comments for clarity
  * include what you use.
  ---------
* feat(lane_change): implement terminal lane change feature (`#9592 <https://github.com/autowarefoundation/autoware_universe/issues/9592>`_)
  * implement function to compute terminal lane change path
  * push terminal path to candidate paths if no other valid candidate path is found
  * use terminal path in LC interface planWaitingApproval function
  * set lane changing longitudinal accel to zero for terminal lc path
  * rename function
  * chore: rename codeowners file
  * remove unused member variable prev_approved_path\_
  * refactor stop point insertion for terminal lc path
  * add flag to enable/disable terminal path feature
  * update README
  * add parameter to configure stop point placement
  * compute terminal path only when near terminal start
  * add option to disable feature near goal
  * set default flag value to false
  * add documentation for terminal lane change path
  * ensure actual prepare duration is always above minimum prepare duration threshold
  * explicitly return std::nullopt
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  * fix assignment
  * fix spelling
  * fix merge errors
  ---------
  Co-authored-by: tomoya.kimura <tomoya.kimura@tier4.jp>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
* feat(lane_change): add text display for candidate path sampling metrics (`#9810 <https://github.com/autowarefoundation/autoware_universe/issues/9810>`_)
  * display candidate path sampling metrics on rviz
  * rename struct
  ---------
* feat(lane_change): revise current lane objects filtering (`#9785 <https://github.com/autowarefoundation/autoware_universe/issues/9785>`_)
  * consider stopped front objects
  * simplify computation of dist to front current lane object
  * add flag to enable/disable keeping distance from front stopped vehicle
  * fix object filtering test
  ---------
* refactor(lane_change): replace sstream to fmt for marker's text (`#9775 <https://github.com/autowarefoundation/autoware_universe/issues/9775>`_)
* feat(lane_change): add info text to virtual wall (`#9783 <https://github.com/autowarefoundation/autoware_universe/issues/9783>`_)
  * specify reason for lane change stop line
  * add stop reason for incoming rear object
  ---------
* fix(lane_change): add metrics to valid paths visualization (`#9737 <https://github.com/autowarefoundation/autoware_universe/issues/9737>`_)
  * fix(lane_change): add metrics to valid paths visualization
  * fix cpp-check error
  ---------
* refactor(lane_change): separate path-related function to utils/path (`#9633 <https://github.com/autowarefoundation/autoware_universe/issues/9633>`_)
  * refactor(lane_change): separate path-related function to utils/path
  * remove old terminal lane change computation
  * doxygen comments
  * remove frenet planner header
  * minor refactoring by throwing instead
  * minor refactoring
  * fix docstring and remove redundant argument
  * get logger in header
  * add docstring
  * rename function is_colliding
  * Fix failing test
  * fix for failing scenario caused by prepare velocity
  * fix error message
  ---------
* fix(lane_change): fix prepare length too short at low speed (RT1-8909) (`#9735 <https://github.com/autowarefoundation/autoware_universe/issues/9735>`_)
  fix prepare length too short at low speed (RT1-8909)
* refactor(lane_change): separate structs to different folders (`#9625 <https://github.com/autowarefoundation/autoware_universe/issues/9625>`_)
* fix(lane_change): remove overlapping preceding lanes (`#9526 <https://github.com/autowarefoundation/autoware_universe/issues/9526>`_)
  * fix(lane_change): remove overlapping preceding lanes
  * fix cpp check
  * start searching disconnected lanes directly
  * just remove starting from overlapped found
  * return non reversed lanes
  * fix precommit
  ---------
* Contributors: Fumiya Watanabe, Kyoichi Sugahara, Mamoru Sobue, Takayuki Murooka, Zulfaqar Azmi, mkquda

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* feat(behavior_path_planner): add detail text to virutal wall (`#9600 <https://github.com/autowarefoundation/autoware_universe/issues/9600>`_)
  * feat(behavior_path_planner): add detail text to virutal wall
  * goal is far
  * pull over start pose is far
  * fix lc build
  * fix build
  * Update planning/behavior_path_planner/autoware_behavior_path_goal_planner_module/src/goal_planner_module.cpp
  ---------
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* fix(lane_change): check obj predicted path when filtering (`#9385 <https://github.com/autowarefoundation/autoware_universe/issues/9385>`_)
  * RT1-8537 check object's predicted path when filtering
  * use ranges view in get_line_string_paths
  * check only vehicle type predicted path
  * Refactor naming
  * fix grammatical
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/utils.cpp
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * precommit and grammar fix
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* feat(lane_change): reduce prepare duration when blinker has been activated (`#9185 <https://github.com/autowarefoundation/autoware_universe/issues/9185>`_)
  * add minimum prepare duration parameter
  * reduce prepare duration according to signal activation time
  * chore: update CODEOWNERS (`#9203 <https://github.com/autowarefoundation/autoware_universe/issues/9203>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  * refactor(time_utils): prefix package and namespace with autoware (`#9173 <https://github.com/autowarefoundation/autoware_universe/issues/9173>`_)
  * refactor(time_utils): prefix package and namespace with autoware
  * refactor(time_utils): prefix package and namespace with autoware
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * feat(rtc_interface): add requested field (`#9202 <https://github.com/autowarefoundation/autoware_universe/issues/9202>`_)
  * add requested feature
  * Update planning/autoware_rtc_interface/test/test_rtc_interface.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * fix(mpc_lateral_controller): correctly resample the MPC trajectory yaws (`#9199 <https://github.com/autowarefoundation/autoware_universe/issues/9199>`_)
  * fix(bpp): prevent accessing nullopt (`#9204 <https://github.com/autowarefoundation/autoware_universe/issues/9204>`_)
  fix(bpp): calcDistanceToRedTrafficLight null
  * refactor(autoware_map_based_prediction): split pedestrian and bicycle predictor (`#9201 <https://github.com/autowarefoundation/autoware_universe/issues/9201>`_)
  * refactor: grouping functions
  * refactor: grouping parameters
  * refactor: rename member road_users_history to road_users_history\_
  * refactor: separate util functions
  * refactor: Add predictor_vru.cpp and utils.cpp to map_based_prediction_node
  * refactor: Add explicit template instantiation for removeOldObjectsHistory function
  * refactor: Add tf2_geometry_msgs to data_structure
  * refactor: Remove unused variables and functions in map_based_prediction_node.cpp
  * Update perception/autoware_map_based_prediction/include/map_based_prediction/predictor_vru.hpp
  * Apply suggestions from code review
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * refactor(ndt_scan_matcher, ndt_omp): move ndt_omp into ndt_scan_matcher (`#8912 <https://github.com/autowarefoundation/autoware_universe/issues/8912>`_)
  * Moved ndt_omp into ndt_scan_matcher
  * Added Copyright
  * style(pre-commit): autofix
  * Fixed include
  * Fixed cast style
  * Fixed include
  * Fixed honorific title
  * Fixed honorific title
  * style(pre-commit): autofix
  * Fixed include hierarchy
  * style(pre-commit): autofix
  * Fixed include hierarchy
  * style(pre-commit): autofix
  * Fixed hierarchy
  * Fixed NVTP to NVTL
  * Added cspell:ignore
  * Fixed miss spell
  * style(pre-commit): autofix
  * Fixed include
  * Renamed applyFilter
  * Moved ***_impl.hpp from include/ to src/
  * style(pre-commit): autofix
  * Fixed variable scope
  * Fixed to pass by reference
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  * feat(autoware_test_utils): add traffic light msgs parser (`#9177 <https://github.com/autowarefoundation/autoware_universe/issues/9177>`_)
  * modify implementation to compute and keep actual prepare duration in transient data
  * if LC path is approved, set prepare duration in transient data from approved path prepare duration
  * change default value of LC param min_prepare_duration
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/utils.cpp
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  * add function to set signal activation time, add docstring for function calc_actual_prepare_duration
  * check for zero value max_acc to avoid division by zero
  * chore: rename codeowners file
  * chore: rename codeowners file
  * chore: rename codeowners file
  * allow decelerating in lane changing phase near terminal
  * fix spelling
  * fix units
  * allow decelerating in lane changing phase near terminal
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * run pre-commit check
  * fix spelling
  * fix format
  * allow decelerating in lane changing phase near terminal
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * run pre-commit check
  * fix spelling
  * fix format
  ---------
  Co-authored-by: awf-autoware-bot[bot] <94889083+awf-autoware-bot[bot]@users.noreply.github.com>
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Esteve Fernandez <33620+esteve@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Go Sakayori <go-sakayori@users.noreply.github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  Co-authored-by: SakodaShintaro <shintaro.sakoda@tier4.jp>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  Co-authored-by: tomoya.kimura <tomoya.kimura@tier4.jp>
* feat(lane_changing): improve computation of lane changing acceleration (`#9545 <https://github.com/autowarefoundation/autoware_universe/issues/9545>`_)
  * allow decelerating in lane changing phase near terminal
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * run pre-commit check
  * fix spelling
  * fix format
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* fix(cpplint): include what you use - planning (`#9570 <https://github.com/autowarefoundation/autoware_universe/issues/9570>`_)
* refactor(test_utils): return parser as optional (`#9391 <https://github.com/autowarefoundation/autoware_universe/issues/9391>`_)
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* fix(lane_change): cap ego's predicted path velocity (RT1-8505) (`#9341 <https://github.com/autowarefoundation/autoware_universe/issues/9341>`_)
  * fix(lane_change): cap ego's predicted path velocity (RT1-8505)
  * properly cap based on 0.0 instead of min lc vel
  * fix build error
  ---------
* fix(autoware_behavior_path_lane_change_module): fix clang-diagnostic-unused-variable (`#9401 <https://github.com/autowarefoundation/autoware_universe/issues/9401>`_)
* feat(lane_change): improve delay lane change logic (`#9480 <https://github.com/autowarefoundation/autoware_universe/issues/9480>`_)
  * implement function to check if lane change delay is required
  * refactor function isParkedObject
  * refactor delay lane change parameters
  * update lc param yaml
  * separate target lane leading objects based on behavior (RT1-8532)
  * fixed overlapped filtering and lanes debug marker
  * combine filteredObjects function
  * renaming functions and type
  * update some logic to check is stopped
  * rename expanded to stopped_outside_boundary
  * Include docstring
  * rename stopped_outside_boundary → stopped_at_bound
  * Update planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  * Update planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  * spell-check
  * add docstring for function is_delay_lane_change
  * remove unused functions
  * fix spelling
  * add delay parameters to README
  * add documentation for delay lane change behavior
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/utils.cpp
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/utils.cpp
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/utils.cpp
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  * run pre-commit checks
  * only check for delay lc if feature is enabled
  ---------
  Co-authored-by: Zulfaqar Azmi <zulfaqar.azmi@tier4.jp>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
* fix(autoware_behavior_path_lane_change_module): fix clang-diagnostic-error (`#9402 <https://github.com/autowarefoundation/autoware_universe/issues/9402>`_)
* fix(autoware_behavior_path_lane_change_module): fix clang-diagnostic-overloaded-virtual (`#9400 <https://github.com/autowarefoundation/autoware_universe/issues/9400>`_)
* feat(lane_change): parse predicted objects for lane change test (RT1-8251) (`#9256 <https://github.com/autowarefoundation/autoware_universe/issues/9256>`_)
  * RT1-8251 parse predicted objects
  * fix pre-commit and build error
  * add additional test and fix test failure
  * fix lint_cmake failure
  * use expect instead
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/test/test_lane_change_scene.cpp
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  ---------
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
* refactor(lane_change): refactor lane change parameters (`#9403 <https://github.com/autowarefoundation/autoware_universe/issues/9403>`_)
  * refactor lane change parameters
  * update lane change param yaml
  * update lane change README
  * regroup some parameters
  * run pre-commit prettier step
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/include/autoware/behavior_path_lane_change_module/utils/parameters.hpp
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/README.md
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  * apply pre-commit checks
  ---------
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* refactor(lane_change): separate target lane leading based on obj behavior (`#9372 <https://github.com/autowarefoundation/autoware_universe/issues/9372>`_)
  * separate target lane leading objects based on behavior (RT1-8532)
  * fixed overlapped filtering and lanes debug marker
  * combine filteredObjects function
  * renaming functions and type
  * update some logic to check is stopped
  * rename expanded to stopped_outside_boundary
  * Include docstring
  * rename stopped_outside_boundary → stopped_at_bound
  * Update planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  * Update planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  * spell-check
  ---------
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
* feat(lane_change): output velocity factor (`#9349 <https://github.com/autowarefoundation/autoware_universe/issues/9349>`_)
* refactor(lane_change): refactor extended object safety check (`#9322 <https://github.com/autowarefoundation/autoware_universe/issues/9322>`_)
  * refactor LC extended object collision check code
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  ---------
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
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
* refactor(lane_change): remove std::optional from lanes polygon (`#9288 <https://github.com/autowarefoundation/autoware_universe/issues/9288>`_)
* fix(lane_change): extending lane change path for multiple lane change (RT1-8427) (`#9268 <https://github.com/autowarefoundation/autoware_universe/issues/9268>`_)
  * RT1-8427 extending lc path for multiple lc
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  ---------
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(lane_change): correct computation of maximum lane changing length threshold (`#9279 <https://github.com/autowarefoundation/autoware_universe/issues/9279>`_)
  fix computation of maximum lane changing length threshold
* refactor(lane_change): revert "remove std::optional from lanes polygon" (`#9272 <https://github.com/autowarefoundation/autoware_universe/issues/9272>`_)
  Revert "refactor(lane_change): remove std::optional from lanes polygon (`#9267 <https://github.com/autowarefoundation/autoware_universe/issues/9267>`_)"
  This reverts commit 0c70ea8793985c6aae90f851eeffdd2561fe04b3.
* refactor(lane_change): remove std::optional from lanes polygon (`#9267 <https://github.com/autowarefoundation/autoware_universe/issues/9267>`_)
* fix(lane_change): enable cancel when ego in turn direction lane (`#9124 <https://github.com/autowarefoundation/autoware_universe/issues/9124>`_)
  * RT0-33893 add checks from prev intersection
  * fix shadow variable
  * fix logic
  * update readme
  * refactor get_ego_footprint
  ---------
* test(bpp_common): add unit test for safety check (`#9223 <https://github.com/autowarefoundation/autoware_universe/issues/9223>`_)
  * add test for object collision
  * add test for more functions
  * add docstring
  * fix lane change
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, Go Sakayori, Kosuke Takeuchi, M. Fatih Cırıt, Ryohsuke Mitsudome, Satoshi OTA, Yutaka Kondo, Zulfaqar Azmi, kobayu858, mkquda

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* refactor(lane_change): remove std::optional from lanes polygon (`#9288 <https://github.com/autowarefoundation/autoware_universe/issues/9288>`_)
* fix(lane_change): extending lane change path for multiple lane change (RT1-8427) (`#9268 <https://github.com/autowarefoundation/autoware_universe/issues/9268>`_)
  * RT1-8427 extending lc path for multiple lc
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  ---------
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(lane_change): correct computation of maximum lane changing length threshold (`#9279 <https://github.com/autowarefoundation/autoware_universe/issues/9279>`_)
  fix computation of maximum lane changing length threshold
* refactor(lane_change): revert "remove std::optional from lanes polygon" (`#9272 <https://github.com/autowarefoundation/autoware_universe/issues/9272>`_)
  Revert "refactor(lane_change): remove std::optional from lanes polygon (`#9267 <https://github.com/autowarefoundation/autoware_universe/issues/9267>`_)"
  This reverts commit 0c70ea8793985c6aae90f851eeffdd2561fe04b3.
* refactor(lane_change): remove std::optional from lanes polygon (`#9267 <https://github.com/autowarefoundation/autoware_universe/issues/9267>`_)
* fix(lane_change): enable cancel when ego in turn direction lane (`#9124 <https://github.com/autowarefoundation/autoware_universe/issues/9124>`_)
  * RT0-33893 add checks from prev intersection
  * fix shadow variable
  * fix logic
  * update readme
  * refactor get_ego_footprint
  ---------
* test(bpp_common): add unit test for safety check (`#9223 <https://github.com/autowarefoundation/autoware_universe/issues/9223>`_)
  * add test for object collision
  * add test for more functions
  * add docstring
  * fix lane change
  ---------
* Contributors: Esteve Fernandez, Go Sakayori, Yutaka Kondo, Zulfaqar Azmi, mkquda

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(behavior_path_planner, behavior_velocity_planner): fix to not read invalid ID (`#9103 <https://github.com/autowarefoundation/autoware_universe/issues/9103>`_)
  * fix(behavior_path_planner, behavior_velocity_planner): fix to not read invalid ID
  * style(pre-commit): autofix
  * fix typo
  * fix(behavior_path_planner, behavior_velocity_planner): fix typo and indentation
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(lane_change): refactor longitudinal acceleration sampling (`#9091 <https://github.com/autowarefoundation/autoware_universe/issues/9091>`_)
  * fix calc_all_max_lc_lengths function
  * remove unused functions
  * remove limit on velocity in calc_all_max_lc_lengths function
  * sample longitudinal acceleration separately for each prepater duration
  * refactor prepare phase metrics calculation
  * check for zero value prepare duration
  * refactor calc_lon_acceleration_samples function
  ---------
* feat(autoware_test_utils): add path with lane id parser (`#9098 <https://github.com/autowarefoundation/autoware_universe/issues/9098>`_)
  * add path with lane id parser
  * refactor parse to use template
  ---------
* feat(lane_change): add unit test for normal lane change class (RT1-7970) (`#9090 <https://github.com/autowarefoundation/autoware_universe/issues/9090>`_)
  * RT1-7970 testing base class
  * additional test
  * Added update lanes
  * check path generation
  * check is lane change required
  * fix PRs comment
  ---------
* refactor(lane_change): reducing clang-tidy warnings (`#9085 <https://github.com/autowarefoundation/autoware_universe/issues/9085>`_)
  * refactor(lane_change): reducing clang-tidy warnings
  * change function name to snake case
  ---------
* refactor(object_recognition_utils): add autoware prefix to object_recognition_utils (`#8946 <https://github.com/autowarefoundation/autoware_universe/issues/8946>`_)
* refactor(bpp_common, motion_utils): move path shifter util functions to autoware::motion_utils (`#9081 <https://github.com/autowarefoundation/autoware_universe/issues/9081>`_)
  * remove unused function
  * mover path shifter utils function to autoware motion utils
  * minor change in license header
  * fix warning message
  * remove header file
  ---------
* fix(lane_change): insert stop for current lanes object (RT0-33761)  (`#9070 <https://github.com/autowarefoundation/autoware_universe/issues/9070>`_)
  * RT0-33761 fix lc insert stop for current lanes object
  * fix wrong value used for comparison
  * ignore current lane object that is not on ego's path
  * remove print
  * update readme
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/utils.cpp
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  * revert is_within_vel_th removal
  * fix flowchart too wide
  * rename variable in has_blocking_target_object_for_stopping
  * Add docstring and rename function
  * change color
  ---------
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
* refactor(lane_change): refactor get_lane_change_lanes function (`#9044 <https://github.com/autowarefoundation/autoware_universe/issues/9044>`_)
  * refactor(lane_change): refactor get_lane_change_lanes function
  * Add doxygen comment for to_geom_msg_pose
  ---------
* refactor(lane_change): replace any code that can use transient data (`#8999 <https://github.com/autowarefoundation/autoware_universe/issues/8999>`_)
  * RT1-8004 replace hasEnoughLength
  * RT1-8004 Removed isNearEndOfCurrentLanes
  * RT1-8004 refactor sample longitudinal acc values
  * remove calc maximum lane change length
  * Revert "remove calc maximum lane change length"
  This reverts commit e9cc386e1c21321c59f518d2acbe78a3c668471f.
  * Revert "RT1-8004 refactor sample longitudinal acc values"
  This reverts commit 775bcdb8fa1817511741776861f9edb7e22fd744.
  * replace generateCenterLinePath
  * RT1-8004 simplify stuck detection
  * swap call to update filtered_objects and update transient data
  * RT1-8004 fix conflict
  * RT1-8004 Rename isVehicleStuck to is_ego_stuck()
  * RT1-8004 change calcPrepareDuration to snake case
  ---------
* refactor(lane_change): refactor code using transient data (`#8997 <https://github.com/autowarefoundation/autoware_universe/issues/8997>`_)
  * add target lane length and ego arc length along current and target lanes to transient data
  * refactor code using transient data
  * refactor get_lane_change_paths function
  * minor refactoring
  * refactor util functions
  * refactor getPrepareSegment function
  ---------
* refactor(bpp): simplify ExtendedPredictedObject and add new member variables (`#8889 <https://github.com/autowarefoundation/autoware_universe/issues/8889>`_)
  * simplify ExtendedPredictedObject and add new member variables
  * replace self polygon to initial polygon
  * comment
  * add comments to dist of ego
  ---------
* fix(lane_change): fix abort distance enough check (`#8979 <https://github.com/autowarefoundation/autoware_universe/issues/8979>`_)
  * RT1-7991 fix abort distance enough check
  * RT-7991 remove unused function
  ---------
* refactor(lane_change): add TransientData to store commonly used lane change-related variables. (`#8954 <https://github.com/autowarefoundation/autoware_universe/issues/8954>`_)
  * add transient data
  * reverted max lc dist in  calcCurrentMinMax
  * rename
  * minor refactoring
  * update doxygen comments
  ---------
* feat(lane_change): modify lane change target boundary check to consider velocity (`#8961 <https://github.com/autowarefoundation/autoware_universe/issues/8961>`_)
  * check if candidate path footprint exceeds target lane boundary when lc velocity is above minimum
  * move functions to relevant module
  * suppress unused function cppcheck
  * minor change
  ---------
* fix(autoware_behavior_path_lane_change_module): fix unusedFunction (`#8960 <https://github.com/autowarefoundation/autoware_universe/issues/8960>`_)
  * fix:unusedFunction
  * fix:unusedFunction
  * fix:unusedFunction
  * fix:pre_commit
  ---------
* refactor(lane_change): refactor getLaneChangePaths function (`#8909 <https://github.com/autowarefoundation/autoware_universe/issues/8909>`_)
  * refactor lane change utility funcions
  * LC utility function to get distance to next regulatory element
  * don't activate LC module when close to regulatory element
  * modify threshold distance calculation
  * move regulatory element check to canTransitFailureState() function
  * always run LC module if approaching terminal point
  * use max possible LC length as threshold
  * update LC readme
  * refactor implementation
  * update readme
  * refactor checking data validity
  * refactor sampling of prepare phase metrics and lane changing phase metrics
  * add route handler function to get pose from 2d arc length
  * refactor candidate path generation
  * refactor candidate path safety check
  * fix variable name
  * Update planning/autoware_route_handler/src/route_handler.cpp
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  * correct parameter name
  * set prepare segment velocity after taking max path velocity value
  * update LC README
  * minor changes
  * check phase length difference with previos valid candidate path
  * change logger name
  * change functions names to snake case
  * use snake case for function names
  * add colors to flow chart in README
  ---------
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
* refactor(autoware_interpolation): prefix package and namespace with autoware (`#8088 <https://github.com/autowarefoundation/autoware_universe/issues/8088>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(lane_change): add checks to ensure the edge of vehicle do not exceed target lane boundary when changing lanes (`#8750 <https://github.com/autowarefoundation/autoware_universe/issues/8750>`_)
  * check if LC candidate path footprint exceeds target lane far bound
  * add parameter to enable/disable check
  * check only lane changing section of cadidate path
  * fix spelling
  * small refactoring
  ---------
* fix(lane_change): set initail rtc state properly (`#8902 <https://github.com/autowarefoundation/autoware_universe/issues/8902>`_)
  set initail rtc state properly
* feat(lane_change): improve execution condition of lane change module (`#8648 <https://github.com/autowarefoundation/autoware_universe/issues/8648>`_)
  * refactor lane change utility funcions
  * LC utility function to get distance to next regulatory element
  * don't activate LC module when close to regulatory element
  * modify threshold distance calculation
  * move regulatory element check to canTransitFailureState() function
  * always run LC module if approaching terminal point
  * use max possible LC length as threshold
  * update LC readme
  * refactor implementation
  * update readme
  * check distance to reg element for candidate path only if not near terminal start
  ---------
* feat(rtc_interface, lane_change): check state transition for cooperate status (`#8855 <https://github.com/autowarefoundation/autoware_universe/issues/8855>`_)
  * update rtc state transition
  * remove transition from failuer and succeeded
  * fix
  * check initial state for cooperate status
  * change rtc cooperate status according to module status
  ---------
* fix(autoware_behavior_path_planner): align the parameters with launcher (`#8790 <https://github.com/autowarefoundation/autoware_universe/issues/8790>`_)
  parameters in behavior_path_planner aligned
* fix(autoware_behavior_path_lane_change_module): fix unusedFunction (`#8653 <https://github.com/autowarefoundation/autoware_universe/issues/8653>`_)
  fix:unusedFunction
* fix(bpp): use common steering factor interface for same scene modules (`#8675 <https://github.com/autowarefoundation/autoware_universe/issues/8675>`_)
* fix(lane_change): update rtc status for some failure condition (`#8604 <https://github.com/autowarefoundation/autoware_universe/issues/8604>`_)
  update rtc status for some failure condition
* fix(lane_change): activate turn signal as soon as we have the intention to change lanes (`#8571 <https://github.com/autowarefoundation/autoware_universe/issues/8571>`_)
  * modify lane change requested condition
  * modify lane change requested condition
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/calculation.cpp
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  * style(pre-commit): autofix
  * fix docstring
  * modify LC turn signal logic
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/include/autoware/behavior_path_lane_change_module/scene.hpp
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  * minor change
  ---------
  Co-authored-by: Muhammad Zulfaqar Azmi <zulfaqar.azmi@tier4.jp>
  Co-authored-by: Zulfaqar Azmi <93502286+zulfaqar-azmi-t4@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(lane_change): fix delay logic that caused timing to be late (`#8549 <https://github.com/autowarefoundation/autoware_universe/issues/8549>`_)
  * RT1-5067 fix delay logic that caused timing to be late
  * remove autoware namespace
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* fix(lane_change): modify lane change requested condition (`#8510 <https://github.com/autowarefoundation/autoware_universe/issues/8510>`_)
  * modify lane change requested condition
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/utils/calculation.cpp
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  * style(pre-commit): autofix
  * fix docstring
  ---------
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(lane_change): consider deceleration in safety check for cancel (`#7943 <https://github.com/autowarefoundation/autoware_universe/issues/7943>`_)
  * feat(lane_change): consider deceleration in safety check for cancel
  * docs(lane_change): fix document
  * fix conflicts and refactor
  * fix conflict
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Muhammad Zulfaqar Azmi <zulfaqar.azmi@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(lane_change): rename prepare_segment_ignore_object_velocity_thresh (`#8532 <https://github.com/autowarefoundation/autoware_universe/issues/8532>`_)
  change parameter name for more expressive name
* refactor(behavior_path_planner): apply clang-tidy check (`#7549 <https://github.com/autowarefoundation/autoware_universe/issues/7549>`_)
  * goal_planner
  * lane_change
  ---------
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
* feat(lane_change): ensure LC merging lane stop point is safe (`#8369 <https://github.com/autowarefoundation/autoware_universe/issues/8369>`_)
  * function to check for merging lane
  * function to compute distance from last fit width center line point to lane end
  * ensure lane width at LC stop point is larger than ego width
  * refactor function isMergingLane
  * improve implementation
  * apply logic only when current ego foot print is within lane
  * change implementation to use intersection points of buffered centerline and lane polygon
  * minor refactoring
  * overload function isEgoWithinOriginalLane to pass lane polygon directly
  ---------
* refactor(lane_change): update filtered objects only once (`#8489 <https://github.com/autowarefoundation/autoware_universe/issues/8489>`_)
* fix(lane_change): moving object is filtered in the extended target lanes (`#8218 <https://github.com/autowarefoundation/autoware_universe/issues/8218>`_)
  * object 3rd
  * named param
  ---------
* fix(lane_change): do not cancel when approaching terminal start (`#8381 <https://github.com/autowarefoundation/autoware_universe/issues/8381>`_)
  * do not cancel if ego vehicle approaching terminal start
  * Insert stop point if object is coming from rear
  * minor edit to fix conflict
  * rename function
  ---------
* fix(lane_change): fix invalid doesn't have stop point (`#8470 <https://github.com/autowarefoundation/autoware_universe/issues/8470>`_)
  fix invalid doesn't have stop point
* fix(lane_change): unify stuck detection to avoid unnecessary computation (`#8383 <https://github.com/autowarefoundation/autoware_universe/issues/8383>`_)
  unify stuck detection in getLaneChangePaths
* fix(turn_signal, lane_change, goal_planner): add optional to tackle lane change turn signal and pull over turn signal (`#8463 <https://github.com/autowarefoundation/autoware_universe/issues/8463>`_)
  * add optional to tackle LC turn signal and pull over turn signal
  * CPP file should not re-define default value; typo in copying from internal repos
  ---------
* refactor(lane_change): separate leading and trailing objects (`#8214 <https://github.com/autowarefoundation/autoware_universe/issues/8214>`_)
  * refactor(lane_change): separate leading and trailing objects
  * Refactor to use common function
  ---------
* fix(lane_change): skip generating path if longitudinal distance difference is less than threshold (`#8363 <https://github.com/autowarefoundation/autoware_universe/issues/8363>`_)
  * fix when prepare length is insufficient
  * add reason for comparing prev_prep_diff with eps for lc_length_diff
  ---------
* fix(lane_change): skip generating path if lane changing path is too long (`#8362 <https://github.com/autowarefoundation/autoware_universe/issues/8362>`_)
  rework. skip lane changing for insufficeient distance in target lane
* fix(lane_change): skip path computation if len exceed dist to terminal start (`#8359 <https://github.com/autowarefoundation/autoware_universe/issues/8359>`_)
  Skip computation if prepare length exceed distance to terminal start
* refactor(lane_change): refactor  debug print when  computing paths (`#8358 <https://github.com/autowarefoundation/autoware_universe/issues/8358>`_)
  Refactor debug print
* chore(lane_change): add codeowner (`#8387 <https://github.com/autowarefoundation/autoware_universe/issues/8387>`_)
* refactor(lane_change): check start point directly after getting start point (`#8357 <https://github.com/autowarefoundation/autoware_universe/issues/8357>`_)
  * check start point directly after getting start point
  * Update planning/behavior_path_planner/autoware_behavior_path_lane_change_module/src/scene.cpp
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* feat(lane_change): use different rss param to deal with parked vehicle (`#8316 <https://github.com/autowarefoundation/autoware_universe/issues/8316>`_)
  * different rss value for parked vehicle
  * Documentation and config file update
  ---------
* fix(lane_change): relax finish judge (`#8133 <https://github.com/autowarefoundation/autoware_universe/issues/8133>`_)
  * fix(lane_change): relax finish judge
  * documentation update
  * update readme explanations
  * update config
  ---------
* feat(lane_change): force deactivation in prepare phase (`#8235 <https://github.com/autowarefoundation/autoware_universe/issues/8235>`_)
  transfer to cancel state when force deactivated
* fix(autoware_behavior_path_lane_change_module): fix passedByValue (`#8208 <https://github.com/autowarefoundation/autoware_universe/issues/8208>`_)
  fix:passedByValue
* fix(lane_change): filtering object ahead of terminal (`#8093 <https://github.com/autowarefoundation/autoware_universe/issues/8093>`_)
  * employ lanelet based filtering before distance based filtering
  * use distance based to terminal check instead
  * remove RCLCPP INFO
  * update flow chart
  ---------
* fix(lane_change): delay lane change cancel (`#8048 <https://github.com/autowarefoundation/autoware_universe/issues/8048>`_)
  RT1-6955: delay lane change cancel
* feat(lane_change): enable force execution under unsafe conditions (`#8131 <https://github.com/autowarefoundation/autoware_universe/issues/8131>`_)
  add force execution conditions
* refactor(lane_change): update lanes and its polygons only  when it's updated (`#7989 <https://github.com/autowarefoundation/autoware_universe/issues/7989>`_)
  * refactor(lane_change): compute lanes and polygon only when updated
  * Revert accidental changesd
  This reverts commit cbfd9ae8a88b2d6c3b27b35c9a08bb824ecd5011.
  * fix spell check
  * Make a common getter for current lanes
  * add target lanes getter
  * some minor function refactoring
  ---------
* feat(autoware_behavior_path_planner_common,autoware_behavior_path_lane_change_module): add time_keeper to bpp (`#8004 <https://github.com/autowarefoundation/autoware_universe/issues/8004>`_)
  * feat(autoware_behavior_path_planner_common,autoware_behavior_path_lane_change_module): add time_keeper to bpp
  * update
  ---------
* fix(autoware_behavior_path_lane_change_module): fix shadowVariable (`#7964 <https://github.com/autowarefoundation/autoware_universe/issues/7964>`_)
  fix:shadowVariable
* refactor(lane_change): move struct to lane change namespace (`#7841 <https://github.com/autowarefoundation/autoware_universe/issues/7841>`_)
  * move struct to lane change namespace
  * Revert "move struct to lane change namespace"
  This reverts commit 306984a76103c427732f170a6f7eb5f94e895b0b.
  ---------
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware_universe/issues/7640>`_)
* fix(lane_change): prevent empty path when rerouting (`#7717 <https://github.com/autowarefoundation/autoware_universe/issues/7717>`_)
  fix(lane_change): prevent empty path when routing
* feat(start_planner): yaw threshold for rss check (`#7657 <https://github.com/autowarefoundation/autoware_universe/issues/7657>`_)
  * add param to customize yaw th
  * add param to other modules
  * docs
  * update READMEs with params
  * fix LC README
  * use normalized yaw diff
  ---------
* refactor(lane_change): use lane change namespace for structs (`#7508 <https://github.com/autowarefoundation/autoware_universe/issues/7508>`_)
  * refactor(lane_change): use lane change namespace for structs
  * Move lane change namespace to bottom level
  ---------
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(route_handler)!: rename to include/autoware/{package_name}  (`#7530 <https://github.com/autowarefoundation/autoware_universe/issues/7530>`_)
  refactor(route_handler)!: rename to include/autoware/{package_name}
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, Go Sakayori, Kosuke Takeuchi, Mamoru Sobue, Satoshi OTA, T-Kimura-MM, Takayuki Murooka, Yukinari Hisaki, Yutaka Kondo, Yuxuan Liu, Zhe Shen, Zulfaqar Azmi, danielsanchezaran, kobayu858, mkquda

0.26.0 (2024-04-03)
-------------------
