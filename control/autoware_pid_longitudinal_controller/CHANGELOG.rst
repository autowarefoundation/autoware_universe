^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_pid_longitudinal_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* fix(pid): fix a bug that acceleration feedback does not go in the correct direction when reverse (`#10822 <https://github.com/autowarefoundation/autoware_universe/issues/10822>`_)
  * fix(pid): fix a bug that acceleration feedback does not go in the correct direction when reverse
  * fix CI
  ---------
* fix(pid_longitudinal_controller): fix reseting the prev value (`#10684 <https://github.com/autowarefoundation/autoware_universe/issues/10684>`_)
* Contributors: TaikiYamada4, Yuki TAKAGI, Yuxuan Liu

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
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* fix: add missing includes to autoware_universe_utils (`#10091 <https://github.com/autowarefoundation/autoware_universe/issues/10091>`_)
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
* fix: remove unnecessary parameters (`#9935 <https://github.com/autowarefoundation/autoware_universe/issues/9935>`_)
* feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in fil… (`#9848 <https://github.com/autowarefoundation/autoware_universe/issues/9848>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files control/autoware_pid_longitudinal_controller
* feat(pid_longitudinal_controller): add new slope compensation mode trajectory_goal_adaptive (`#9705 <https://github.com/autowarefoundation/autoware_universe/issues/9705>`_)
* feat(pid_longitudinal_controller): add virtual wall for dry steering and emergency (`#9685 <https://github.com/autowarefoundation/autoware_universe/issues/9685>`_)
  * feat(pid_longitudinal_controller): add virtual wall for dry steering and emergency
  * fix
  ---------
* feat(pid_longitudinal_controller): remove trans/rot deviation validation since the control_validator has the same feature (`#9675 <https://github.com/autowarefoundation/autoware_universe/issues/9675>`_)
  * feat(pid_longitudinal_controller): remove trans/rot deviation validation since the control_validator has the same feature
  * fix test
  ---------
* feat(pid_longitudinal_controller): add smooth_stop mode in debug_values (`#9681 <https://github.com/autowarefoundation/autoware_universe/issues/9681>`_)
* feat(pid_longitudinal_controller): update trajectory_adaptive; add debug_values, adopt rate limit fillter (`#9656 <https://github.com/autowarefoundation/autoware_universe/issues/9656>`_)
* fix(autoware_pid_longitudinal_controller): fix bugprone-branch-clone (`#9629 <https://github.com/autowarefoundation/autoware_universe/issues/9629>`_)
  fix: bugprone-branch-clone
* Contributors: Fumiya Watanabe, Takayuki Murooka, Vishal Chauhan, Yuki TAKAGI, kobayu858

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
* fix(cpplint): include what you use - control (`#9565 <https://github.com/autowarefoundation/autoware_universe/issues/9565>`_)
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(pid_longitudinal_controller): suppress rclcpp_warning/error (`#9384 <https://github.com/autowarefoundation/autoware_universe/issues/9384>`_)
  * feat(pid_longitudinal_controller): suppress rclcpp_warning/error
  * update codeowner
  ---------
* feat(trajectory_follower): publsih control horzion (`#8977 <https://github.com/autowarefoundation/autoware_universe/issues/8977>`_)
  * feat(trajectory_follower): publsih control horzion
  * fix typo
  * rename functions and minor refactor
  * add option to enable horizon pub
  * add tests for horizon
  * update docs
  * rename to ~/debug/control_cmd_horizon
  ---------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kosuke Takeuchi, M. Fatih Cırıt, Ryohsuke Mitsudome, Takayuki Murooka, Yutaka Kondo

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
* refactor(autoware_interpolation): prefix package and namespace with autoware (`#8088 <https://github.com/autowarefoundation/autoware_universe/issues/8088>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* fix(pid_longitudinal_controller): fix the same point error (`#8758 <https://github.com/autowarefoundation/autoware_universe/issues/8758>`_)
  * fix same point
* feat(pid_longitudinal_controller)!: add acceleration feedback block (`#8325 <https://github.com/autowarefoundation/autoware_universe/issues/8325>`_)
* refactor(control/pid_longitudinal_controller): rework parameters (`#6707 <https://github.com/autowarefoundation/autoware_universe/issues/6707>`_)
  * reset and re-apply refactoring
  * style(pre-commit): autofix
  * .
  * .
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(pid_longitudinal_controller): re-organize diff limit structure and fix state change condition (`#7718 <https://github.com/autowarefoundation/autoware_universe/issues/7718>`_)
  change diff limit structure
  change stopped condition
  define a new param
* fix(controller): revival of dry steering (`#7903 <https://github.com/autowarefoundation/autoware_universe/issues/7903>`_)
  * Revert "fix(autoware_mpc_lateral_controller): delete the zero speed constraint (`#7673 <https://github.com/autowarefoundation/autoware_universe/issues/7673>`_)"
  This reverts commit 69258bd92cb8a0ff8320df9b2302db72975e027f.
  * dry steering
  * add comments
  * add minor fix and modify unit test for dry steering
  ---------
* fix(autoware_pid_longitudinal_controller, autoware_trajectory_follower_node): unite diagnostic_updater\_ in PID and MPC. (`#7674 <https://github.com/autowarefoundation/autoware_universe/issues/7674>`_)
  * diag_updater\_ added in PID
  * correct the pointer form
  * pre-commit
  ---------
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(control)!: refactor directory structures of the trajectory followers (`#7521 <https://github.com/autowarefoundation/autoware_universe/issues/7521>`_)
  * control_traj
  * add follower_node
  * fix
  ---------
* ci(pre-commit): autoupdate (`#7499 <https://github.com/autowarefoundation/autoware_universe/issues/7499>`_)
  Co-authored-by: M. Fatih Cırıt <mfc@leodrive.ai>
* refactor(trajectory_follower_node): trajectory follower node add autoware prefix (`#7344 <https://github.com/autowarefoundation/autoware_universe/issues/7344>`_)
  * rename trajectory follower node package
  * update dependencies, launch files, and README files
  * fix formats
  * remove autoware\_ prefix from launch arg option
  ---------
* refactor(trajectory_follower_base): trajectory follower base add autoware prefix (`#7343 <https://github.com/autowarefoundation/autoware_universe/issues/7343>`_)
  * rename trajectory follower base package
  * update dependencies and includes
  * fix formats
  ---------
* refactor(vehicle_info_utils)!: prefix package and namespace with autoware (`#7353 <https://github.com/autowarefoundation/autoware_universe/issues/7353>`_)
  * chore(autoware_vehicle_info_utils): rename header
  * chore(bpp-common): vehicle info
  * chore(path_optimizer): vehicle info
  * chore(velocity_smoother): vehicle info
  * chore(bvp-common): vehicle info
  * chore(static_centerline_generator): vehicle info
  * chore(obstacle_cruise_planner): vehicle info
  * chore(obstacle_velocity_limiter): vehicle info
  * chore(mission_planner): vehicle info
  * chore(obstacle_stop_planner): vehicle info
  * chore(planning_validator): vehicle info
  * chore(surround_obstacle_checker): vehicle info
  * chore(goal_planner): vehicle info
  * chore(start_planner): vehicle info
  * chore(control_performance_analysis): vehicle info
  * chore(lane_departure_checker): vehicle info
  * chore(predicted_path_checker): vehicle info
  * chore(vehicle_cmd_gate): vehicle info
  * chore(obstacle_collision_checker): vehicle info
  * chore(operation_mode_transition_manager): vehicle info
  * chore(mpc): vehicle info
  * chore(control): vehicle info
  * chore(common): vehicle info
  * chore(perception): vehicle info
  * chore(evaluator): vehicle info
  * chore(freespace): vehicle info
  * chore(planning): vehicle info
  * chore(vehicle): vehicle info
  * chore(simulator): vehicle info
  * chore(launch): vehicle info
  * chore(system): vehicle info
  * chore(sensing): vehicle info
  * fix(autoware_joy_controller): remove unused deps
  ---------
* refactor(pid_longitudinal_controller)!: prefix package and namespace with autoware (`#7383 <https://github.com/autowarefoundation/autoware_universe/issues/7383>`_)
  * add prefix
  * fix
  * fix trajectory follower node param
  ---------
* Contributors: Esteve Fernandez, Kosuke Takeuchi, Satoshi OTA, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, Zhe Shen, awf-autoware-bot[bot], mkquda, oguzkaganozt

0.26.0 (2024-04-03)
-------------------
