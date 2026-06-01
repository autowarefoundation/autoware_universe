^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory_follower_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(pid_longitudinal_controller): parameterize ff_scale limits (`#12415 <https://github.com/mitsudome-r/autoware_universe/issues/12415>`_)
  parameterize ff_scale limits
* feat(mpc_lateral_controller): add integrator friendly parameters to mpc lateral control (`#12301 <https://github.com/mitsudome-r/autoware_universe/issues/12301>`_)
  * implement max update threshold on mpc side
  * add new parameter to lateral_controller_defaults.param.yaml
  * modify the logic to only update offset when last target is reached
  ---------
* feat(autoware_trajectory_follower_node): adopt cie (`#12320 <https://github.com/mitsudome-r/autoware_universe/issues/12320>`_)
  * feat: adopt cie
  * style(pre-commit): autofix
  * fix: revert ros2 executor
  * Potential fix for pull request finding
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
  * fix
  * fix: remove unnecessary lines
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
* feat(steer_offset_estimator): implement calibration logic (`#12004 <https://github.com/mitsudome-r/autoware_universe/issues/12004>`_)
  * refactor(steer_offset_estimator): restructure node and estimator implementation
  - Updated the CMakeLists.txt to reflect new library and executable structure.
  - Removed outdated README and added a new detailed README.md for better documentation.
  - Introduced a new node class for steer offset estimation and refactored the estimator logic.
  - Added utility functions for pose and steering calculations.
  - Implemented tests for the estimator and utility functions to ensure reliability.
  - Updated parameters in the schema and configuration files for improved clarity and functionality.
  - Removed deprecated files and images to streamline the package.
  This commit enhances the overall architecture and usability of the steer offset estimator package.
  * refactor(steer_offset_estimator): update CMake configuration and remove deprecated files
  - Bump CMake minimum version to 3.14 and adjust project structure in CMakeLists.txt.
  - Refactor library and executable definitions for clarity and maintainability.
  - Remove the main.cpp file as the node is now defined in a separate header and source file.
  - Update parameter comments in the configuration file for better clarity.
  - Remove the glog dependency from package.xml to streamline dependencies.
  This commit enhances the organization and readability of the steer offset estimator package.
  * docs(steer_offset_estimator): enhance README formatting for mathematical equations
  - Improved the formatting of mathematical equations in the README.md to enhance readability by adding line breaks.
  - Removed the monitoring section to streamline the documentation.
  This update aims to provide clearer guidance on the steering offset estimation algorithm and its implementation details.
  * docs(steer_offset_estimator): add debug info output section to README
  * docs(steer_offset_estimator): improve formatting of algorithm steps in README
  * feat(steer_offset_estimator): enhance estimator parameters and update calculations
  - Added new parameters: measurement_noise, denominator_floor, and covariance_floor to improve estimation stability.
  - Refactored the update logic to incorporate Kalman gain and residual calculations, enhancing the accuracy of the steering offset estimation.
  - Updated debug output to reflect new calculation metrics, including kalman_gain and residual.
  This commit improves the robustness and performance of the steer offset estimator by refining its parameterization and calculation methods.
  * feat(steer_offset_estimator): add new parameters for enhanced estimation
  - Introduced measurement_noise, denominator_floor, and covariance_floor parameters to the SteerOffsetEstimatorParameters structure.
  - Updated the parameter loading function to accommodate the new parameters, improving the configurability of the estimator.
  This change aims to enhance the performance and stability of the steering offset estimation process by allowing for more precise parameter tuning.
  * fix(steer_offset_estimator): update debug output to use standard deviation
  - Modified the debug output format in the SteerOffsetEstimatorNode to replace covariance with standard deviation for clarity.
  - This change enhances the readability of the debug information by providing a more intuitive metric for uncertainty.
  * refactor(steer_offset_estimator): rename and restructure noise parameters for clarity
  - Renamed measurement_noise to measurement_noise_covariance and added process_noise_covariance to the SteerOffsetEstimatorParameters structure for better clarity.
  - Updated the parameter loading function to reflect these changes, enhancing the configurability of the estimator.
  - Refactored the update logic to utilize the new covariance parameters, improving the accuracy of the steering offset estimation.
  This commit aims to streamline the parameterization and enhance the performance of the steer offset estimator.
  * docs(steer_offset_estimator): update README and schema for Kalman Filter implementation
  - Revised the README to reflect the transition from Recursive Least Squares (RLS) to Kalman Filter for steering offset estimation, including detailed algorithm steps and parameter descriptions.
  - Updated the schema to replace the forgetting factor with process and measurement noise covariance parameters, enhancing clarity and configurability.
  - Adjusted default values for min_velocity and max_steer to improve operational thresholds.
  This commit aims to provide comprehensive documentation and parameterization for the new Kalman Filter approach, ensuring better understanding and usability of the estimator.
  * Tune parameter
  * fix unit test build issues
  * ensure steering report stamp is synced with pose stamp
  * set time stamp for steering status msg
  * refactor update function
  * ensure latest steering info is not too old
  * add docstring for functions
  * ensure previous pose is not too old when computing twist
  * fix accessing empty list
  * add gates got max steer rate and max angular velocity, refactor code.
  * add missing params to schema
  * fix unit tests
  * remove unused util function
  * use std::optional instead of throw error
  * use create_timer instead of create_wall_timer
  * don't use shared_ptrs in deque
  * Update vehicle/autoware_steer_offset_estimator/src/node.cpp
  Co-authored-by: Taiki Yamada <129915538+TaikiYamada4@users.noreply.github.com>
  * Update vehicle/autoware_steer_offset_estimator/src/node.hpp
  Co-authored-by: Taiki Yamada <129915538+TaikiYamada4@users.noreply.github.com>
  * Update vehicle/autoware_steer_offset_estimator/README.md
  Co-authored-by: Taiki Yamada <129915538+TaikiYamada4@users.noreply.github.com>
  * run precommit checks
  * publish steering offset error
  * Update vehicle/autoware_steer_offset_estimator/src/node.hpp
  Co-authored-by: Taiki Yamada <129915538+TaikiYamada4@users.noreply.github.com>
  * fix spelling
  * fix format
  * update readme and schema
  * add maintainer
  * Update vehicle/autoware_steer_offset_estimator/include/autoware/steer_offset_estimator/utils.hpp
  Co-authored-by: Yukinari Hisaki <42021302+yhisaki@users.noreply.github.com>
  * chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/mitsudome-r/autoware_universe/issues/1776>`_)
  not sync github-release
  * implement offset calibration parameters
  * implement service to manually trigger steering offset update
  * implement automated calibration logic
  * fix broken logic
  * update param value, make use of std::abs consistent
  * publish steer offset update when calibration is triggered
  * subscribe to steer offset update in controller node and update offset value
  * publish initial steering offset on startup
  * remove steering offset estimator within lateral controller, and use value from new estimator node
  * Revert "chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/mitsudome-r/autoware_universe/issues/1776>`_)"
  This reverts commit 957bc8e22b2f9b7995d536510932d9c2fe2cc312.
  * update parameter schema
  * update readme
  * add parameter to enable/disable parameter file overwrite
  * minor fix for unit tests
  * fix topic name, initialize member variable
  * fix spelling
  * add docstring for functions
  * add comment to yaml to indicate auto update
  * remove unused struct member
  * create the service server even if the CalibrationMode is OFF
  * implement low pass filter for steering offset on mpc side to prevent steep correction
  * update param schema
  * fix test
  * modify implementation such that estimated offset is considered to be a residual bias
  * separate offset update logic from callibration logic
  - check if offset update topic needs to be published for control
  - when offset update is published, right new published value to log file
  - separate calibration logic to right offset to calibration file when triggered
  - remove unnecessary output topic steer_offset_error (since its same as steering offset value itself)
  * Update control/autoware_mpc_lateral_controller/include/autoware/mpc_lateral_controller/mpc_lateral_controller.hpp
  Co-authored-by: Taiki Yamada <129915538+TaikiYamada4@users.noreply.github.com>
  * update readme, tune params, remove unused param
  * fix spelling, fix param schema
  ---------
  Co-authored-by: Y.Hisaki <yhisaki31@gmail.com>
  Co-authored-by: Taiki Yamada <129915538+TaikiYamada4@users.noreply.github.com>
  Co-authored-by: Yukinari Hisaki <42021302+yhisaki@users.noreply.github.com>
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* chore: organize maintainer (`#12147 <https://github.com/mitsudome-r/autoware_universe/issues/12147>`_)
* feat(trajectory_follower_node): add trajectory timeout monitoring with diagnostic updater (`#12082 <https://github.com/mitsudome-r/autoware_universe/issues/12082>`_)
  * feat(autoware_trajectory_follower_node): add trajectory timeout check via diagnostic_updater
  Add incoming trajectory message timeout detection using
  PollingSubscriber::latest_timestamp() and publish the result
  through diagnostic_updater.
  Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>
  * chore(trajectory_follower_node): change statement position.
  * style(pre-commit): autofix
  * chore(trajectory_follower_node): replace magic number by std::optional
  * fix missing update for README.md
  * rename
  * chore(trajectory_follower_node): rename the method name
  * chore(trajectory_follower_node): change the diagnostic name
  * fix(trajectory_follower_node): remove default value.
  ---------
  Co-authored-by: Claude Opus 4.6 <noreply@anthropic.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Go Sakayori, Satoshi OTA, Takayuki AKAMINE, Tetsuhiro Kawaguchi, github-actions, mkquda

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* refactor(autoware_trajectory_follower_node): remove redundant diagnostic updates from lateral and longitudinal controllers (`#11934 <https://github.com/autowarefoundation/autoware_universe/issues/11934>`_)
* feat: control : fix contol packages compile error when using ros2 jazzy (`#11556 <https://github.com/autowarefoundation/autoware_universe/issues/11556>`_)
* Contributors: Kyoichi Sugahara, Ryohsuke Mitsudome, 心刚

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(simple_trajectory_follower): update remapping (`#11645 <https://github.com/autowarefoundation/autoware_universe/issues/11645>`_)
  * move private headers
  * use component
  * remove empty line
  * update topic names
  ---------
  Co-authored-by: Junya Sasaki <junya.sasaki@tier4.jp>
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* fix(autoware_trajectory_follower_node): change plot label of data[25] from calculated into feedback (`#11266 <https://github.com/autowarefoundation/autoware_universe/issues/11266>`_)
* feat(pid_longitudinal_controller): don't switch to DRIVE if the state conditions are not met (`#11369 <https://github.com/autowarefoundation/autoware_universe/issues/11369>`_)
  * feat(pid_longitudinal_controller): don't switch to DRIVE if the state conditions are not met
  * add is_autoware_control_enabled field for tests
  ---------
* Contributors: Mert Çolak, Ryohsuke Mitsudome, Takagi, Isamu, Tim Clephas, Xiaoyu WANG

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat: change planning output topic name to /planning/trajectory (`#11135 <https://github.com/autowarefoundation/autoware_universe/issues/11135>`_)
  * change planning output topic name to /planning/trajectory
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_trajectory_follower, autoware_mission_planner_universe, autoware_scenario_selector): use transient_local for operation_mode_state (`#11101 <https://github.com/autowarefoundation/autoware_universe/issues/11101>`_)
  * subscribe operation-mode with transient_local
  * fix mistake
  * fix unit test code
  * pre-commit
  ---------
* Contributors: Kem (TiankuiXian), Yukihiro Saito

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
* fix(planning, control): reuse stamp of subscribed topic to measure component latency (`#10201 <https://github.com/autowarefoundation/autoware_universe/issues/10201>`_)
  * fix(behavior_velocity_planner): reuse timestamp of recieved path
  * fix(behavior_path_planner): check timestamp first in timer driven callback
  * fix(trajectory_follower_node): check timestamp first in timer driven callback
  * fix(vehicle_cmd_gate): reuse timestamp of recieved path
  ---------
* Contributors: Hayato Mizushima, Satoshi OTA, Yutaka Kondo

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
* feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in fil… (`#9853 <https://github.com/autowarefoundation/autoware_universe/issues/9853>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files control/autoware_trajectory_follower_node
* fix: remove unnecessary parameters (`#9935 <https://github.com/autowarefoundation/autoware_universe/issues/9935>`_)
* chore(trajectory_follower_node): fix typos (`#9707 <https://github.com/autowarefoundation/autoware_universe/issues/9707>`_)
* feat(pid_longitudinal_controller): update plotjuggler settings (`#9703 <https://github.com/autowarefoundation/autoware_universe/issues/9703>`_)
* feat(pid_longitudinal_controller): remove trans/rot deviation validation since the control_validator has the same feature (`#9675 <https://github.com/autowarefoundation/autoware_universe/issues/9675>`_)
  * feat(pid_longitudinal_controller): remove trans/rot deviation validation since the control_validator has the same feature
  * fix test
  ---------
* Contributors: Fumiya Watanabe, Kosuke Takeuchi, Takayuki Murooka, Vishal Chauhan, Yuki TAKAGI

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
* refactor: correct spelling (`#9528 <https://github.com/autowarefoundation/autoware_universe/issues/9528>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(mpc_lateral_controller): suppress rclcpp_warning/error (`#9382 <https://github.com/autowarefoundation/autoware_universe/issues/9382>`_)
  * feat(mpc_lateral_controller): suppress rclcpp_warning/error
  * fix
  * fix test
  ---------
* fix(autoware_trajectory_follower_node): fix clang-diagnostic-format-security (`#9378 <https://github.com/autowarefoundation/autoware_universe/issues/9378>`_)
* refactor(fake_test_node): prefix package and namespace with autoware (`#9249 <https://github.com/autowarefoundation/autoware_universe/issues/9249>`_)
* feat(trajectory_follower): publsih control horzion (`#8977 <https://github.com/autowarefoundation/autoware_universe/issues/8977>`_)
  * feat(trajectory_follower): publsih control horzion
  * fix typo
  * rename functions and minor refactor
  * add option to enable horizon pub
  * add tests for horizon
  * update docs
  * rename to ~/debug/control_cmd_horizon
  ---------
* fix(control): missing dependency in control components (`#9073 <https://github.com/autowarefoundation/autoware_universe/issues/9073>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kosuke Takeuchi, M. Fatih Cırıt, Ryohsuke Mitsudome, Ryuta Kambe, Takayuki Murooka, Yutaka Kondo, ぐるぐる

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix(control): missing dependency in control components (`#9073 <https://github.com/autowarefoundation/autoware_universe/issues/9073>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Yutaka Kondo, ぐるぐる

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(control): align the parameters with launcher (`#8789 <https://github.com/autowarefoundation/autoware_universe/issues/8789>`_)
  align the control parameters
* feat(autoware_mpc_lateral_controller): add predicted trajectory acconts for input delay (`#8436 <https://github.com/autowarefoundation/autoware_universe/issues/8436>`_)
  * feat: enable delayed initial state for predicted trajectory
  * feat: enable debug publishing of predicted and resampled reference trajectories
  ---------
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
* ci: disable failing tests undetected due to broken regex filter (`#7731 <https://github.com/autowarefoundation/autoware_universe/issues/7731>`_)
* fix(autoware_pid_longitudinal_controller, autoware_trajectory_follower_node): unite diagnostic_updater\_ in PID and MPC. (`#7674 <https://github.com/autowarefoundation/autoware_universe/issues/7674>`_)
  * diag_updater\_ added in PID
  * correct the pointer form
  * pre-commit
  ---------
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* feat(mpc_lateral_controller): signal a MRM when MPC fails. (`#7016 <https://github.com/autowarefoundation/autoware_universe/issues/7016>`_)
  * mpc fail checker diagnostic added
  * fix some scope issues
  * member attribute added.
  * shared pointer added.
  * member attribute (diag_updater\_) added
  * dependency added.
  * implementation of the MpcLateralController corrected!
  * typo in comment corrected!
  * member method argument corrected
  * delete unnecessary reference mark
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
  * rebase
  * correct the include
  * pre-commit
  ---------
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(control)!: refactor directory structures of the trajectory followers (`#7521 <https://github.com/autowarefoundation/autoware_universe/issues/7521>`_)
  * control_traj
  * add follower_node
  * fix
  ---------
* refactor(pure_pursuit): prefix package and namespace with autoware\_ (`#7301 <https://github.com/autowarefoundation/autoware_universe/issues/7301>`_)
  * RT1-6683 add autoware prefix to package and namepace
  * fix precommit
  ---------
* refactor(trajectory_follower_node): trajectory follower node add autoware prefix (`#7344 <https://github.com/autowarefoundation/autoware_universe/issues/7344>`_)
  * rename trajectory follower node package
  * update dependencies, launch files, and README files
  * fix formats
  * remove autoware\_ prefix from launch arg option
  ---------
* Contributors: Kosuke Takeuchi, Kyoichi Sugahara, M. Fatih Cırıt, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, Zhe Shen, Zulfaqar Azmi, mkquda, oguzkaganozt

0.26.0 (2024-04-03)
-------------------
