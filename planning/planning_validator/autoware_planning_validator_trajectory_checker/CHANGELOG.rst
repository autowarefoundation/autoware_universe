^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_validator_trajectory_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(planning_validator): apply `agnocast_wrapper::Node` to `planning_validator` (`#12964 <https://github.com/autowarefoundation/autoware_universe/issues/12964>`_)
  * apply agnocast_wrapper::node
  * style(pre-commit): autofix
  * fix(autoware_planning_validator): feed stop checker per odometry message
  Restore the original VehicleStopChecker behavior: subscribe to
  /localization/kinematic_state and call addTwist() on every odometry message,
  instead of feeding the latest polled odometry once per trajectory cycle.
  * refactor(autoware_planning_validator): drop odometry subscriber comment
  * refactor(planning_validator): migrate to polling:: API
  * fix(planning_validator): adopt PlanningFactorInterfaceT<Node>
  * fix(autoware_planning_validator): launch as a standalone node and fix the agnocast build wiring
  * fix(autoware_planning_validator): pass the underlying rclcpp node to the planning test manager
  * style(pre-commit): autofix
  * refactor(autoware_planning_validator): declare the utils packages it includes and drop the stale comment
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: kobayu858 <yutaro.kobayashi.2@tier4.jp>
* chore(planning): update package maintainers (`#13186 <https://github.com/autowarefoundation/autoware_universe/issues/13186>`_)
* fix(planning_validator, control_validator): fix include path for angles header (`#13026 <https://github.com/autowarefoundation/autoware_universe/issues/13026>`_)
  * fix include path for angles header
  * add missing package dependency
  ---------
* Contributors: Koichi Imai, Ryohsuke Mitsudome, Satoshi OTA, mkquda

0.52.0 (2026-06-30)
-------------------

0.51.0 (2026-05-01)
-------------------

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(planning_validator): reset last_valid_trajectory when receiving new route   (`#11836 <https://github.com/autowarefoundation/autoware_universe/issues/11836>`_)
  * fix(planning_validator): reset last_valid_trajectory when receiving new route
  * skip check_traject_shift when ego stops
  * fix unit test failure
  * use ivehicle_stop_checker
  ---------
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* Contributors: Kem (TiankuiXian), Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(planning_validator): update QoS settings for operational mode state subscriber (`#11401 <https://github.com/autowarefoundation/autoware_universe/issues/11401>`_)
* fix: use the correct jerk computation formula (`#11306 <https://github.com/autowarefoundation/autoware_universe/issues/11306>`_)
  * fix: use the correct jerk computation formula
  * also fix test script
  ---------
* feat(planning_validator): add operational mode state handling and validation filtering (`#11216 <https://github.com/autowarefoundation/autoware_universe/issues/11216>`_)
  * feat(planning_validator): add operational mode state handling and validation filtering
  * refactor(planning_validator):  unused validation_filtering method and initialize validation status with existing function
  * feat(planning_validator): add operational mode state handling and publisher for test
  ---------
* Contributors: Kyoichi Sugahara, Ryohsuke Mitsudome, Yuxuan Liu

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* refactor(planning_validator): refactor planning validator configuration and error handling (`#11081 <https://github.com/autowarefoundation/autoware_universe/issues/11081>`_)
  * refactor trajectory check error handling
  * define set_diag_status function for each module locally
  * update documentation
  ---------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(planning_validator): fix conflict between trajectory shift and distance deviation checks (`#10799 <https://github.com/autowarefoundation/autoware_universe/issues/10799>`_)
  * use global is_critical_error flag for trajectory diagnostics update
  * add warning to readme
  * Update planning/planning_validator/autoware_planning_validator/README.md
  ---------
* fix(planning_validator_trajectory_checker): set is_critical_error flag to false at start of validation (`#10912 <https://github.com/autowarefoundation/autoware_universe/issues/10912>`_)
  set is_critical_error flag to false at start of validation
* fix(planning_validator): check the yaw deviation of the initial trajectory (`#10878 <https://github.com/autowarefoundation/autoware_universe/issues/10878>`_)
* Contributors: Maxime CLEMENT, Mete Fatih Cırıt, mkquda

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(planning_validator): add condition to check the yaw deviation (`#10818 <https://github.com/autowarefoundation/autoware_universe/issues/10818>`_)
* fix(planning_validator): fix the lateral distance calculation (`#10801 <https://github.com/autowarefoundation/autoware_universe/issues/10801>`_)
* feat(planning_validator): subscribe additional topic for collision detection (`#10745 <https://github.com/autowarefoundation/autoware_universe/issues/10745>`_)
  * feat(planning_validator): subscribe pointcloud
  * feat(planning_validator): subscribe route and map
  * fix(planning_validator): load glog component
  * fix: unexpected test fail
  ---------
* refactor(planning_validator): implement plugin structure for planning validator node (`#10571 <https://github.com/autowarefoundation/autoware_universe/issues/10571>`_)
  * chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)
  not sync github-release
  * create planning latency validator plugin module
  * Revert "chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)"
  This reverts commit 871a8540ade845c7c9a193029d407b411a4d685b.
  * create planning trajectory validator plugin module
  * Update planning/planning_validator/autoware_planning_validator/src/manager.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/node.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * minor fix
  * refactor implementation
  * uncomment lines for adding pose markers
  * fix CMakeLists
  * add comment
  * update planning launch xml
  * Update planning/planning_validator/autoware_planning_latency_validator/include/autoware/planning_latency_validator/latency_validator.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/plugin_interface.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/types.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/src/node.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_latency_validator/src/latency_validator.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * apply pre-commit checks
  * rename plugins for consistency
  * rename directories and files to match package names
  * refactor planning validator tests
  * add packages maintainer
  * separate trajectory check parameters
  * add missing package dependencies
  * move trajectory diagnostics test to trajectory checker module
  * remove blank line
  * add launch args for validator modules
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  Co-authored-by: GitHub Action <action@github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* Contributors: Maxime CLEMENT, Satoshi OTA, TaikiYamada4, mkquda
