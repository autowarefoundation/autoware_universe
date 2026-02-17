^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_raw_vehicle_cmd_converter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* feat: vehicle related packages support jazzy (`#11605 <https://github.com/autowarefoundation/autoware_universe/issues/11605>`_)
* Contributors: Ryohsuke Mitsudome, mkquda, 心刚

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat(raw_vehicle_cmd_converter): add control component latency measuremement and publisher (`#11044 <https://github.com/autowarefoundation/autoware_universe/issues/11044>`_)
  feat(raw_vehicle_cmd_converter): add control component latency measurement and publisher
* Contributors: Kyoichi Sugahara

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
* feat(autoware_raw_vehicle_cmd_converter)!: tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_raw_vehicle_cmd_converter (`#9924 <https://github.com/autowarefoundation/autoware_universe/issues/9924>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files vehicle/autoware_raw_vehicle_cmd_converter
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* fix(raw_veihicle_converter): fix too long line (`#9716 <https://github.com/autowarefoundation/autoware_universe/issues/9716>`_)
* feat(raw_vehicle_cmd_converter): add vehicle adaptor  (`#8782 <https://github.com/autowarefoundation/autoware_universe/issues/8782>`_)
  * feat(raw_vehicle_cmd_converter): add vehicle adaptor
  sub operation status
  * feat(raw_vehicle_cmd_converter): publish vehicle adaptor output
  * use control horizon
  * revert carla
  * update docs
  ---------
* Contributors: Fumiya Watanabe, Kosuke Takeuchi, Vishal Chauhan

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
* fix(cpplint): include what you use - vehicle (`#9575 <https://github.com/autowarefoundation/autoware_universe/issues/9575>`_)
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

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
* fix(simple_planning_simulator, raw_vehicle_cmd_converter): swap row index and column index for csv loader  (`#8963 <https://github.com/autowarefoundation/autoware_universe/issues/8963>`_)
  swap row and column
* test(raw_vehicle_cmd_converter): add tests (`#8951 <https://github.com/autowarefoundation/autoware_universe/issues/8951>`_)
  * remove header file according to clangd warning
  * add test
  * fix
  * add test for get function
  * apply clang tidy
  * fix test content
  ---------
* refactor(autoware_interpolation): prefix package and namespace with autoware (`#8088 <https://github.com/autowarefoundation/autoware_universe/issues/8088>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(start_planner,raw_vechile_cmd_converter): align parameter with autoware_launch's parameter (`#8913 <https://github.com/autowarefoundation/autoware_universe/issues/8913>`_)
  * align autoware_raw_vehicle_cmd_converter's parameter
  * align start_planner's parameter
  ---------
* fix(raw_vehicle_cmd_converter): fix convert_steer_cmd_method condition (`#8813 <https://github.com/autowarefoundation/autoware_universe/issues/8813>`_)
* fix(raw_vehicle_cmd_converter): fix null check (`#8677 <https://github.com/autowarefoundation/autoware_universe/issues/8677>`_)
* chore(raw_vehicle_cmd_converter): add maintainer (`#8671 <https://github.com/autowarefoundation/autoware_universe/issues/8671>`_)
* feat(raw_vehicle_cmd_converter): set convert_actuation_to_steering_status false by default (`#8668 <https://github.com/autowarefoundation/autoware_universe/issues/8668>`_)
* feat(raw_vehicle_cmd_converter): disable actuation to steering (`#8588 <https://github.com/autowarefoundation/autoware_universe/issues/8588>`_)
* feat(raw_vehicle_cmd_converter): add steer command conversion with VGR (`#8504 <https://github.com/autowarefoundation/autoware_universe/issues/8504>`_)
  * feat(raw_vehicle_cmd_converter): add steer command conversion with VGR
  * make class and add test
  * remove member vgr_coef from node
  * update readme
  * add svg
  * add plot scripts
  * Update vehicle/autoware_raw_vehicle_cmd_converter/README.md
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
  * not always subscribe actuation_status
  * add comment for using normal sub for steering status
  ---------
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(raw_vehicle_cmd_converter): use polling subscriber (`#7319 <https://github.com/autowarefoundation/autoware_universe/issues/7319>`_)
  * replace subscription
  * fix document
  * sum up functions
  * add maintainer
  ---------
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
* refactor(accel_brake_map_calibrator)!: add autoware\_ prefix (`#7351 <https://github.com/autowarefoundation/autoware_universe/issues/7351>`_)
  * add prefix to the codes
  change dir name
  update
  update
  * delete debug
  * fix format
  * fix format
  * restore
  * poi
  ---------
* refactor(raw_vehicle_cmd_converter)!: prefix package and namespace with autoware (`#7385 <https://github.com/autowarefoundation/autoware_universe/issues/7385>`_)
  * add prefix
  * fix other packages
  * fix cppcheck
  * pre-commit
  * fix
  ---------
* Contributors: Esteve Fernandez, Go Sakayori, Kosuke Takeuchi, Kyoichi Sugahara, Sho Iwasawa, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
