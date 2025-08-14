^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_surround_obstacle_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat(surround_obstacle_checker): add safety factors to the published planning factor (`#11077 <https://github.com/autowarefoundation/autoware_universe/issues/11077>`_)
  * feat(surround_obstacle_checker): add safety factors to the published planning factor
  * distinguish between DynamicObject and PointCloud in planning factor, and add test
  * fix typo
  * use utils function
  ---------
  Co-authored-by: Kotakku <kotakkucu@gmail.com>
* chore(surround_obstacle_checker): update maintainer (`#11078 <https://github.com/autowarefoundation/autoware_universe/issues/11078>`_)
* Contributors: Kotakku, Satoshi OTA

0.46.0 (2025-06-20)
-------------------

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
* feat!: replace VelocityLimit messages with autoware_internal_planning_msgs (`#10273 <https://github.com/autowarefoundation/autoware_universe/issues/10273>`_)
* feat: adaption to ROS nodes guidelines about directory structure (`#10268 <https://github.com/autowarefoundation/autoware_universe/issues/10268>`_)
* feat(Autoware_planning_factor_interface): replace tier4_msgs with autoware_internal_msgs (`#10204 <https://github.com/autowarefoundation/autoware_universe/issues/10204>`_)
* Contributors: Hayato Mizushima, NorahXiong, Ryohsuke Mitsudome, Yutaka Kondo, 心刚

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* Contributors: Fumiya Watanabe, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
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
* feat(autoware_surround_obstacle_checker): tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_surround_obstacle_checker (`#9915 <https://github.com/autowarefoundation/autoware_universe/issues/9915>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files  planning/autoware_surround_obstacle_checker
* Contributors: Fumiya Watanabe, Mamoru Sobue, Satoshi OTA, Vishal Chauhan

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
* ci(pre-commit): autoupdate (`#8949 <https://github.com/autowarefoundation/autoware_universe/issues/8949>`_)
  Co-authored-by: M. Fatih Cırıt <mfc@autoware.org>
* fix(surround_obstacle_checker)!: remove stop reason (`#9450 <https://github.com/autowarefoundation/autoware_universe/issues/9450>`_)
  fix(surround_obstacle_checker): remove stop reason
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix(autoware_surround_obstacle_checker): fix clang-diagnostic-unused-private-field (`#9399 <https://github.com/autowarefoundation/autoware_universe/issues/9399>`_)
  * fix: clang-diagnostic-unused-private-field
  * refactor: fmt
  * refactor: fmt
  * refactor: fmt
  * fix: hpp
  ---------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Satoshi OTA, Yutaka Kondo, awf-autoware-bot[bot], kobayu858

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
* feat(costmap_generator, control_validator, scenario_selector, surround_obstacle_checker, vehicle_cmd_gate): add processing time pub. (`#9065 <https://github.com/autowarefoundation/autoware_universe/issues/9065>`_)
  * feat(costmap_generator, control_validator, scenario_selector, surround_obstacle_checker, vehicle_cmd_gate): Add: processing_time_pub
  * fix: pre-commit
  * feat(costmap_generator): fix: No output when not Active.
  * fix: clang-format
  * Re: fix: clang-format
  ---------
* test(surround_obstacle_checker): add unit tests (`#9039 <https://github.com/autowarefoundation/autoware_universe/issues/9039>`_)
  * refactor: isStopRequired
  * test: write test for isStopRequired
  * refactor: use universe utils
  * fix: shutdown
  ---------
* fix(other_planning_packages): align the parameters with launcher (`#8793 <https://github.com/autowarefoundation/autoware_universe/issues/8793>`_)
  * parameters in planning/others aligned
  * update json
  ---------
* fix(autoware_surround_obstacle_checker): fix unusedFunction (`#8774 <https://github.com/autowarefoundation/autoware_universe/issues/8774>`_)
  fix:unusedFunction
* feat(surround_obstacle_checker): integrate generate_parameter_library (`#8719 <https://github.com/autowarefoundation/autoware_universe/issues/8719>`_)
  * add generate_parameter_library to package
  * add parameter file generator script
  * use mapped parameters
  * integrate generate_parameter_library
  * style(pre-commit): autofix
  * check to use dynamic object
  * remove default values
  * fix variable shadowing
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_surround_obstacle_checker): fix passedByValue (`#8206 <https://github.com/autowarefoundation/autoware_universe/issues/8206>`_)
  fix:passedByValue
* fix(autoware_surround_obstacle_checker): fix constVariableReference (`#8059 <https://github.com/autowarefoundation/autoware_universe/issues/8059>`_)
  fix:constVariableReference
* fix(autoware_surround_obstacle_checker): fix funcArgNamesDifferent (`#8020 <https://github.com/autowarefoundation/autoware_universe/issues/8020>`_)
  fix:funcArgNamesDifferent
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(surround_obstacle_checker): remove include directory (`#7507 <https://github.com/autowarefoundation/autoware_universe/issues/7507>`_)
  * feat(surround_obstacle_checker): remove include directory
  * fix
  * fix
  ---------
* fix(planning): set single depth sensor data qos for pointlcoud polling subscribers (`#7490 <https://github.com/autowarefoundation/autoware_universe/issues/7490>`_)
  set single depth sensor data qos for pointlcoud polling subscribers
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
* refactor(surround_obstacle_checker)!: prefix package and namespace with autoware (`#7298 <https://github.com/autowarefoundation/autoware_universe/issues/7298>`_)
  * fix(autoware_surround_obstacle_checker): rename
  * fix(autoware_surround_obstacle_checker): rename header
  * fix(launch): update package name
  ---------
* Contributors: Kazunori-Nakajima, Kosuke Takeuchi, Mitsuhiro Sakamoto, Satoshi OTA, Takayuki Murooka, Yutaka Kondo, Zhe Shen, kobayu858, mkquda

0.26.0 (2024-04-03)
-------------------
