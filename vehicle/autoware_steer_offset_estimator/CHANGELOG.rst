^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_steer_offset_estimator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(steer_offset_estimator): implement new steer offset estimator using kalman filter (`#11911 <https://github.com/autowarefoundation/autoware_universe/issues/11911>`_)
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
  ---------
  Co-authored-by: Y.Hisaki <yhisaki31@gmail.com>
  Co-authored-by: Taiki Yamada <129915538+TaikiYamada4@users.noreply.github.com>
  Co-authored-by: Yukinari Hisaki <42021302+yhisaki@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, mkquda

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* chore(autoware_steer_offset_estimator): add new maintainer Taiki Yamada to package.xml (`#11797 <https://github.com/autowarefoundation/autoware_universe/issues/11797>`_)
  Add new maintainer Taiki Yamada to package.xml
* Contributors: Ryohsuke Mitsudome, Yukinari Hisaki

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(steer_offset_estimator): integrate twist estimation function to steer_offset_estimator node (`#11315 <https://github.com/autowarefoundation/autoware_universe/issues/11315>`_)
* chore(autoware_steer_offset_estimator): add maintainer (`#11313 <https://github.com/autowarefoundation/autoware_universe/issues/11313>`_)
  add maintainer
* feat(steer_offset_estimator): load initial steer offset from arbitrary file and publish steering_offset_error (`#11249 <https://github.com/autowarefoundation/autoware_universe/issues/11249>`_)
* Contributors: Ryohsuke Mitsudome, Yukinari Hisaki

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------

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
* Contributors: Hayato Mizushima, Yutaka Kondo

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
* feat(autoware_steer_offset_estimator)!: tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_steer_offset_estimator (`#9926 <https://github.com/autowarefoundation/autoware_universe/issues/9926>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files vehicle/autoware_steer_offset_estimator
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Vishal Chauhan

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
* refactor(global_parameter_loader): prefix package and namespace with autoware (`#9303 <https://github.com/autowarefoundation/autoware_universe/issues/9303>`_)
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, Ryohsuke Mitsudome, Yutaka Kondo

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
* refactor(pose2twist)!: prefix package and namespace with autoware (`#8347 <https://github.com/autowarefoundation/autoware_universe/issues/8347>`_)
  * add autoware\_ prefix
  * use target_include_directories instead
  ---------
  Co-authored-by: SakodaShintaro <shintaro.sakoda@tier4.jp>
* fix(steer_offset_estimator): fix link to json schema in README (`#7655 <https://github.com/autowarefoundation/autoware_universe/issues/7655>`_)
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
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
* chore(steer_offset_estimator): add prefix autoware\_ to steer_offset_estimator (`#7342 <https://github.com/autowarefoundation/autoware_universe/issues/7342>`_)
  * add perfix
  * fix directory structrue
  * fix include guard
  ---------
* Contributors: Go Sakayori, Masaki Baba, Maxime CLEMENT, Satoshi OTA, Takayuki Murooka, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
