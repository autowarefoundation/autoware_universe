^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_lane_departure_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
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
* Contributors: TaikiYamada4, Zulfaqar Azmi

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* refactor(control): remove unimplemented function declarations (`#10314 <https://github.com/autowarefoundation/autoware_universe/issues/10314>`_)
  remove unimplemented function declarations
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* fix(lane_departure_checker): fix trajectory resampling logic to keep given interval (`#10221 <https://github.com/autowarefoundation/autoware_universe/issues/10221>`_)
  * fix(lane_departure_checker): fix trajectory resampling logic to keep given interval
  * test(lane_departure_checker): add test case for consecutive small distances followed by large distance
  ---------
* Contributors: Autumn60, Hayato Mizushima, Kyoichi Sugahara, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat(autoware_vehicle_info_utils): replace autoware_universe_utils with autoware_utils (`#10167 <https://github.com/autowarefoundation/autoware_universe/issues/10167>`_)
* feat!: replace tier4_planning_msgs/PathWithLaneId with autoware_internal_planning_msgs/PathWithLaneId (`#10023 <https://github.com/autowarefoundation/autoware_universe/issues/10023>`_)
* Contributors: Fumiya Watanabe, Ryohsuke Mitsudome, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(lane_departure_checker): improve LaneDepartureChecker initialization and parameter handling (`#9791 <https://github.com/autowarefoundation/autoware_universe/issues/9791>`_)
  * refactor(lane_departure_checker): improve LaneDepartureChecker initialization and parameter handling
  ---------
* feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in fil… (`#9846 <https://github.com/autowarefoundation/autoware_universe/issues/9846>`_)
  * feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files ontrol/autoware_mpc_lateral_controller
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Kyoichi Sugahara, Vishal Chauhan

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
* ci(pre-commit): update cpplint to 2.0.0 (`#9557 <https://github.com/autowarefoundation/autoware_universe/issues/9557>`_)
* test(lane_departure_checker): add unit test for createVehiclePassingAreas (`#9548 <https://github.com/autowarefoundation/autoware_universe/issues/9548>`_)
  * test(lane_departure_checker): add unit tests for createVehiclePassingAreas function
  ---------
* refactor(lane_departure_checker): move member functions to util functions (`#9547 <https://github.com/autowarefoundation/autoware_universe/issues/9547>`_)
  * refactor(lane_departure_checker): move member functions to util functions
  ---------
* fix(cpplint): include what you use - control (`#9565 <https://github.com/autowarefoundation/autoware_universe/issues/9565>`_)
* test(lane_departure_checker): add tests for calcTrajectoryDeviation(), calcMaxSearchLengthForBoundaries() (`#9029 <https://github.com/autowarefoundation/autoware_universe/issues/9029>`_)
  * move calcTrajectoryDeviation() to separate files
  * move calcMaxSearchLengthForBoundaries() to separate files
  * add tests for calcTrajectoryDeviation()
  * add tests for calcMaxSearchLengthForBoundaries()
  ---------
  Co-authored-by: Kyoichi Sugahara <32741405+kyoichi-sugahara@users.noreply.github.com>
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat: suppress warning/error of the empty predicted trajectory by MPC (`#9373 <https://github.com/autowarefoundation/autoware_universe/issues/9373>`_)
* feat(start_planner, lane_departure_checker): speed up by updating polygons (`#9309 <https://github.com/autowarefoundation/autoware_universe/issues/9309>`_)
  speed up by updating polygons
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kyoichi Sugahara, M. Fatih Cırıt, Mitsuhiro Sakamoto, Ryohsuke Mitsudome, Takayuki Murooka, Yutaka Kondo, awf-autoware-bot[bot], danielsanchezaran

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(start_planner, lane_departure_checker): speed up by updating polygons (`#9309 <https://github.com/autowarefoundation/autoware_universe/issues/9309>`_)
  speed up by updating polygons
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Yutaka Kondo, danielsanchezaran

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* test(lane_departure_checker): add tests for createVehicleFootprints (`#8928 <https://github.com/autowarefoundation/autoware_universe/issues/8928>`_)
  * move createVehicleFootprints() to seperate files
  * add tests for createVehicleFootprints()
  ---------
* test(lane_departure_checker): add tests for resampleTrajectory (`#8895 <https://github.com/autowarefoundation/autoware_universe/issues/8895>`_)
  * move resampleTrajectory() to separate file
  * add tests for resampleTrajectory()
  ---------
* test(lane_departure_checker): add tests for cutTrajectory (`#8887 <https://github.com/autowarefoundation/autoware_universe/issues/8887>`_)
  * move cutTrajectory() to separate file
  * add test for cutTrajectory()
  ---------
* fix(control): align the parameters with launcher (`#8789 <https://github.com/autowarefoundation/autoware_universe/issues/8789>`_)
  align the control parameters
* refactor(start_planner, lane_departure_checker): remove redundant calculation in fuseLaneletPolygon (`#8682 <https://github.com/autowarefoundation/autoware_universe/issues/8682>`_)
  * remove redundant fused lanelet calculation
  * remove unnecessary change
  * add new function
  * fix spelling mistake
  * fix spelling mistake
  * use std::move and lambda funcion for better code
  * add comment for better understanding
  * fix cppcheck
  ---------
* fix(autoware_lane_departure_checker): not to show error message "trajectory deviation is too large" during manual driving (`#8404 <https://github.com/autowarefoundation/autoware_universe/issues/8404>`_)
  * update: update not to show error message "trajectory deviation is too large" during manual driving
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(lane_departure_checker): fix uninitialized variables (`#8451 <https://github.com/autowarefoundation/autoware_universe/issues/8451>`_)
  fix(lane_departure_checker): fix uninitialized_variables
* feat(start_planner): add time_keeper (`#8254 <https://github.com/autowarefoundation/autoware_universe/issues/8254>`_)
  * feat(start_planner): add time_keeper
  * fix
  * fix
  * fix shadow variables
  ---------
* fix(autoware_lane_departure_checker): fix shadowVariable (`#7931 <https://github.com/autowarefoundation/autoware_universe/issues/7931>`_)
  fix:shadowVariable
* refactor(autoware_universe_utils): changed the API to be more intuitive and added documentation (`#7443 <https://github.com/autowarefoundation/autoware_universe/issues/7443>`_)
  * refactor(tier4_autoware_utils): Changed the API to be more intuitive and added documentation.
  * use raw shared ptr in PollingPolicy::NEWEST
  * update
  * fix
  * Update evaluator/autoware_control_evaluator/include/autoware/control_evaluator/control_evaluator_node.hpp
  Co-authored-by: danielsanchezaran <daniel.sanchez@tier4.jp>
  ---------
  Co-authored-by: danielsanchezaran <daniel.sanchez@tier4.jp>
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware_universe/issues/7640>`_)
* feat(motion_velocity_planner, lane_departure_checker): add processing time Float64 publishers (`#7683 <https://github.com/autowarefoundation/autoware_universe/issues/7683>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(control)!: refactor directory structures of the control checkers (`#7524 <https://github.com/autowarefoundation/autoware_universe/issues/7524>`_)
  * aeb
  * control_validator
  * lane_departure_checker
  * shift_decider
  * fix
  ---------
* refactor(autoware_lane_departure_checker)!: rename directory name  (`#7410 <https://github.com/autowarefoundation/autoware_universe/issues/7410>`_)
* Contributors: Go Sakayori, Kosuke Takeuchi, Kyoichi Sugahara, Maxime CLEMENT, Mitsuhiro Sakamoto, T-Kimura-MM, Takayuki Murooka, Yuki TAKAGI, Yukinari Hisaki, Yutaka Kondo, Zhe Shen, kobayu858

0.26.0 (2024-04-03)
-------------------
