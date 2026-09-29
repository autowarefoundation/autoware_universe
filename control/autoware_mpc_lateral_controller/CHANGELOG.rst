^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_mpc_lateral_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* test(autoware_mpc_lateral_controller): add characterization tests (`#13356 <https://github.com/autowarefoundation/autoware_universe/issues/13356>`_)
  test(autoware_mpc_lateral_controller): add characterization tests for MpcLateralController (`#13356 <https://github.com/autowarefoundation/autoware_universe/issues/13356>`_)
  MpcLateralController has no test of its own, so the parameter declaration, the vehicle
  model built from it, the steering offset compensation, the state kept between cycles and
  the command reset are untested. Later work moves these responsibilities between classes,
  and pinning the behaviour first makes a change visible. This adds 37 tests that drive the
  controller through LateralControllerBase and assert on LateralOutput alone. Three of them
  record a value or pin a behaviour that a later change is meant to break, so they carry the
  DISABLED\_ prefix and stay out of the default run.
* refactor(mpc_lateral_controller): decouple mpc logic from rclcpp::Node (`#13090 <https://github.com/autowarefoundation/autoware_universe/issues/13090>`_)
  * refactor(autoware_mpc_lateral_controller): decouple QPSolverOSQP from rclcpp::Logger/Clock
  Change QPSolverInterface::solve() to return a QPSolverResult (success flag
  and warning message) instead of performing RCLCPP_WARN/RCLCPP_WARN_THROTTLE
  internally. Logging is now done by the caller in MPC::executeOptimization,
  allowing QPSolverOSQP to be constructed without a logger or clock.
  * refactor(autoware_mpc_lateral_controller): use tl::expected for MPC helper return values
  Replace std::pair<ResultWithReason, T> in getData, executeOptimization,
  resampleMPCTrajectoryByTime, and updateStateForDelayCompensation with
  tl::expected<T, std::string>, matching the Error-as-Value pattern already
  used elsewhere in autoware_universe and avoiding the need to construct
  dummy values on failure.
  * refactor(autoware_mpc_lateral_controller): return predicted trajectory via MpcResult
  Merge ResultWithReason into MpcResult and add a predicted_trajectory
  field, so MPC::calculateMPC() returns the predicted trajectory instead
  of writing it through an output parameter.
  * refactor(autoware_mpc_lateral_controller): return ctrl_cmd via MpcResult
  Add a ctrl_cmd field to MpcResult so MPC::calculateMPC() returns the
  computed control command instead of writing it through an output
  parameter.
  * refactor(autoware_mpc_lateral_controller): move diagnostic into MpcResult
  Add a Float32MultiArrayStamped field to MpcResult and remove the
  diagnostic output parameter from calculateMPC, so callers get the
  diagnostic data as part of the returned result instead of via a
  mutable reference argument.
  * refactor(autoware_mpc_lateral_controller): move ctrl_cmd_horizon into MpcResult
  Add a LateralHorizon field to MpcResult and remove the ctrl_cmd_horizon
  output parameter from calculateMPC, so callers get the control command
  horizon as part of the returned result instead of via a mutable
  reference argument.
  * refactor(autoware_mpc_lateral_controller): change fields order
  * refactor(autoware_mpc_lateral_controller): publish Frenet predicted trajectory from MpcLateralController::run()
  Move the Frenet-coordinate predicted trajectory out of MPC's internal
  debug publisher and expose it via MpcResult::debug_msgs instead, so
  MpcLateralController::run() is responsible for publishing it alongside
  the other MPC outputs.
  * refactor(autoware_mpc_lateral_controller): publish resampled reference trajectory from MpcLateralController::run()
  Move the resampled reference trajectory debug message creation out of
  MPC::resampleMPCTrajectoryByTime and into MPC::calculateMPC, exposing
  it via MpcResult::debug_msgs so MpcLateralController::run() publishes
  it alongside the other debug topics.
  * refactor(autoware_mpc_lateral_controller): publish nearest pose from MpcLateralController::run()
  Move the nearest pose debug message creation out of MPC::publishNearestDebug
  and into MPC::calculateMPC, exposing it via MpcResult::debug_msgs so
  MpcLateralController::run() publishes it alongside the other debug topics.
  * refactor(autoware_mpc_lateral_controller): publish nearest segment trajectory from MpcLateralController::run()
  Extract the ego-nearest segment index computation and trajectory
  construction out of MPC::publishNearestDebug and into MPC::getData,
  storing the result in MPCData/MpcResult::debug_msgs so
  MpcLateralController::run() publishes it alongside the other debug
  topics. MPC::publishNearestDebug keeps only the nearest_info publish,
  reusing the already-computed segment indices.
  * refactor(autoware_mpc_lateral_controller): make buildNearestInfoMessage a member function returning the debug message
  Turn MPC::publishNearestDebug into MPC::buildNearestInfoMessage, a
  private member function that builds and returns the Float32MultiArrayStamped
  instead of publishing it directly. Store the result in
  MPCData::nearest_info / MpcDebugTopicMessage::nearest_info so
  MpcLateralController::run() publishes it alongside the other debug
  topics, and drop the now-unused m_debug_nearest_info_pub from MPC.
  * refactor(autoware_mpc_lateral_controller): remove rclcpp::Node dependency from MPC
  MPC's constructor no longer does anything now that all debug
  publishers have moved to MpcLateralController, so drop the
  rclcpp::Node & parameter and switch construction to the default
  constructor at all call sites.
  * refactor(autoware_mpc_lateral_controller): make debug_msgs optional in MpcResult
  Only populate MpcResult::debug_msgs when publish_debug_trajectories is
  enabled, and drive publishing in MpcLateralController::run() off the
  optional's presence instead of separately checking m_mpc's flag.
  * refactor(autoware_mpc_lateral_controller): extract publishDebugMessages logic
  * fix(autoware_mpc_lateral_controller): stamp debug topics with MPC input timestamp
  Debug/diagnostic outputs (frenet predicted trajectory, resampled
  reference trajectory, nearest pose/segment/info, diagnostic) were
  stamped with the wall-clock time at publish, causing a small offset
  from the odometry data they were actually computed from. Stamp them
  inside calculateMPC() using the odometry header timestamp instead, so
  they align with the input data for debugging. The main predicted
  trajectory output is left unchanged.
  * fix(autoware_mpc_lateral_controller): use a single timestamp captured in run() for all published messages
  run() is invoked once per control cycle by a timer, so all messages
  produced during that cycle should share one timestamp. Previously,
  ctrl_cmd and ctrl_cmd_horizon were each stamped with a separate
  clock\_->now() call, and predicted_trajectory relied on
  publishPredictedTraj() re-stamping it with clock\_->now() at publish
  time, independently of the timestamp used for debug/diagnostic topics
  in calculateMPC().
  Capture the timestamp once at the top of run() and thread it through
  calculateMPC(), createCtrlCmdMsg(), and createCtrlCmdHorizonMsg() so
  every message published from the same control cycle carries the exact
  same timestamp.
  * refactor(autoware_mpc_lateral_controller): extract setTimestamp() to stamp all MpcResult messages
  calculateMPC() stamped ctrl_cmd, ctrl_cmd_horizon, predicted_trajectory,
  diagnostic, and each debug_msgs field individually inline. Extract this
  into a single setTimestamp() free function so MpcResult is stamped
  consistently in one place, regardless of which fields are populated.
  * fix(autoware_mpc_lateral_controller): stamp MpcResult on every early-return failure path
  When calculateMPC() returns early (getData, delay compensation,
  trajectory resampling, or optimization failure), the returned
  MpcResult was default-constructed, leaving diagnostic.stamp and
  predicted_trajectory.header.stamp at zero. Since publishDebugValues()
  and publishPredictedTraj() no longer re-stamp these messages
  themselves, a zero timestamp was published on every MPC failure,
  making these outputs unusable for time-based analysis.
  Apply setTimestamp() to each failure-path MpcResult so it carries the
  same per-cycle timestamp as a successful result.
  * refactor(autoware_mpc_lateral_controller): set predicted trajectory frame_id inside calculateMPC
  Move the header.frame_id assignment for predicted_trajectory from
  MpcLateralController::publishPredictedTraj into MPC::calculateMPC via a new
  setHeader helper, so the MpcResult returned by calculateMPC() is complete
  (header stamp and frame_id both set) rather than relying on the node to
  fill it in afterward. MPC now caches the reference trajectory's frame_id
  in setReferenceTrajectory().
  * refactor(autoware_mpc_lateral_controller): derive debug message frame_id from reference trajectory
  Replace hardcoded "map" frame_id assignments on debug messages
  (nearest_pose, nearest_segment_trajectory, resampled_reference_trajectory,
  predicted_trajectory_frenet) with the actual reference trajectory frame_id,
  set uniformly in setHeader alongside the timestamp.
  * fix(autoware_mpc_lateral_controller): publish debug topics on MPC failure paths
  Previously debug_msgs stayed nullopt whenever calculateMPC returned early
  on failure, so resampled_reference_trajectory, nearest_pose,
  nearest_segment_trajectory, and nearest_info stopped being published
  entirely while MPC was failing, breaking offline analysis of failure
  cases. Populate debug_msgs incrementally as each field becomes available
  and carry it into every failure branch.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* fix(pre-commit): update pre-commit-hooks-ros to v0.10.3 and adapt include guards (`#13083 <https://github.com/autowarefoundation/autoware_universe/issues/13083>`_)
  * chore: sync files
  * fix(pre-commit): adapt include guards to pre-commit-hooks-ros v0.10.3
  ros-include-guard v0.10.3 only recognises an include guard when #endif is the
  last non-empty line of the file, so that feature test macros are no longer
  mistaken for guards. 29 headers failed that check.
  25 headers wrap the guard in "// clang-format off" / "// clang-format on"
  because the #endif comment plus its // NOLINT exceeds the 100 column limit.
  Drop only the trailing "on" marker; the "off" marker then runs to end of file
  and still protects the line from being wrapped.
  3 CUDA headers ended with "/* *INDENT-ON* */". Move it above the #endif so it
  stays paired with the "/* *INDENT-OFF* */" near the top of the file.
  autoware_behavior_path_planner/test/input.hpp closed its guard immediately after
  opening it, leaving the entire body unguarded. Move the #endif to the end.
  Also hold clang-format at v21.1.8. clang-format 22 migrates
  "AlignAfterOpenBracket: AlwaysBreak" to "BreakAfterOpenBracketIf: true", which
  forces a break after every "if (" whose condition does not fit on one line and
  reformats 88 files.
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
* feat(trajectory_follower, pid, mpc): introduce temporal controller (`#12886 <https://github.com/autowarefoundation/autoware_universe/issues/12886>`_)
  * initial change
  remove resampling
  feat: add temporal trajectory mode to MPC follower (keep spatial default)
  add temporal target selection to PID and stabilize nearest-time in PID/MPC
  remove old parameter
  fix parameter load for MPC
  small fi
  ake temporal nearest-point selection time-driven for PID/MPC and add tests
  fix: align temporal nearest-point handling in MPC and PID, and use input yaw fallback for short segment
  stabilize temporal reference tracking in PID and MPC
  add temporal tracking debug signals for PID and MPC
  gate temporal yaw and curvature by ds dt and velocity
  move diag_updater initialization to constructor initializer list
  * update logic for isDrivingForward
  * change curvature calculation
  * feat: actively reset nearest time to zero if we receive new trajectory
  * fix(pid_longitudinal_controller): reset temporal phase on trajectory replan
  Mirror MPC `#3050 <https://github.com/autowarefoundation/autoware_universe/issues/3050>`_ by reinitializing m_prev_nearest_time from spatial nearest
  when header.stamp changes, and fall back to global spatial nearest when the
  temporal observation window has no candidates.
  Co-authored-by: Cursor <cursoragent@cursor.com>
  * precommit
  * test(pid_longitudinal_controller): use empty time window for spatial fallback case
  The fallback test used window [1.8, 2.2], which still contains t=2.0, so the
  bounded search ran instead of spatial fallback and returned t=1.0. Match the
  MPC fallback test by using a non-overlapping window [10.0, 11.0].
  Co-authored-by: Cursor <cursoragent@cursor.com>
  * fix spell
  * add maintainer
  * set mode flag to spatial
  * Update control/autoware_mpc_lateral_controller/param/lateral_controller_defaults.param.yaml
  Co-authored-by: Go Sakayori <go-sakayori@users.noreply.github.com>
  * fix comment
  * revert: PR `#3072 <https://github.com/autowarefoundation/autoware_universe/issues/3072>`_ and `#3050 <https://github.com/autowarefoundation/autoware_universe/issues/3050>`_ (reset nearest time on new trajectory)
  (cherry picked from commit 9f54a422c4f86465183e5f8a484f543429694b6b)
  * fix reference point selection
  (cherry picked from commit 4ead7da4c53d6902f4573246498a917423c59136)
  * refactors
  (cherry picked from commit e661dc7403fa54bbeee862772b34da97d88208b8)
  ---------
  Co-authored-by: Go Sakayori <gsakayori@gmail.com>
  Co-authored-by: Go Sakayori <go.sakayori@tier4.jp>
  Co-authored-by: YuxuanLiuTier4Desktop <619684051@qq.com>
  Co-authored-by: Cursor <cursoragent@cursor.com>
  Co-authored-by: Go Sakayori <go-sakayori@users.noreply.github.com>
* Contributors: Kazuki Komiya, Kotakku, Ryohsuke Mitsudome, Takahisa Ishikawa, awf-autoware-bot[bot]

0.52.0 (2026-06-30)
-------------------

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(mpc_lateral_controller): add integrator friendly parameters to mpc lateral control (`#12301 <https://github.com/mitsudome-r/autoware_universe/issues/12301>`_)
  * implement max update threshold on mpc side
  * add new parameter to lateral_controller_defaults.param.yaml
  * modify the logic to only update offset when last target is reached
  ---------
* refactor(autoware_mpc_lateral_controller): remove unused config params (`#12245 <https://github.com/mitsudome-r/autoware_universe/issues/12245>`_)
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
* chore: organize maintainer (`#12148 <https://github.com/mitsudome-r/autoware_universe/issues/12148>`_)
* Contributors: Autumn60, Satoshi OTA, github-actions, mkquda

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat!: remove ROS 2 Galactic codes (`#11905 <https://github.com/autowarefoundation/autoware_universe/issues/11905>`_)
* refactor(autoware_trajectory_follower_node): remove redundant diagnostic updates from lateral and longitudinal controllers (`#11934 <https://github.com/autowarefoundation/autoware_universe/issues/11934>`_)
* fix(mpc_lateral_controller): use terminal velocity to extend trajectory (`#11826 <https://github.com/autowarefoundation/autoware_universe/issues/11826>`_)
  * use terminal velocity when extending yaw
  * add early return
  * avoid 0.0
  ---------
* Contributors: Go Sakayori, Kyoichi Sugahara, Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* fix(mpc_lateral_controller):  correct variable used for yaw input (`#11809 <https://github.com/autowarefoundation/autoware_universe/issues/11809>`_)
  fix yaw input for lerp
* Contributors: Go Sakayori, Ryohsuke Mitsudome

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* feat(mpc_lateral_controller): publish the wheel angle in the predicted trajectory (`#11153 <https://github.com/autowarefoundation/autoware_universe/issues/11153>`_)
* Contributors: Maxime CLEMENT, Ryohsuke Mitsudome, Tim Clephas

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* fix(mpc): update time from start in the predicted trajectory (`#10753 <https://github.com/autowarefoundation/autoware_universe/issues/10753>`_)
* Contributors: Zulfaqar Azmi

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat: change info messages to debug, and debug_throttle method (`#10757 <https://github.com/autowarefoundation/autoware_universe/issues/10757>`_)
  change info messages to debug, and debug_throttle method
* feat: mpc info throttle msgs (`#10687 <https://github.com/autowarefoundation/autoware_universe/issues/10687>`_)
  add info msgs
* Contributors: TaikiYamada4, danielsanchezaran

0.45.0 (2025-05-22)
-------------------

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(control): apply THROTTLE to frequent log (`#10418 <https://github.com/autowarefoundation/autoware_universe/issues/10418>`_)
  * fix(control): apply THROTTLE to frequent log
  * fix
  * fix
  * fix
  * fix
  ---------
* refactor(mpc_lateral_controller): rework parameter (`#8935 <https://github.com/autowarefoundation/autoware_universe/issues/8935>`_)
* Contributors: Prakash Kannaiah, Ryohsuke Mitsudome, Takayuki Murooka

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* feat(path_optimizer): additional failure logging and failure mode handling (`#10276 <https://github.com/autowarefoundation/autoware_universe/issues/10276>`_)
  MRM when MPT fails
* fix(autoware_mpc_lateral_controller): replace Eigen::VectorXd with Eigen::Vector3d for state representation (`#10235 <https://github.com/autowarefoundation/autoware_universe/issues/10235>`_)
  * fix(autoware_mpc_lateral_controller): replace Eigen::VectorXd with Eigen::Vector3d for state representation
  * docs(autoware_mpc_lateral_controller): update comments for state representation and discretization considerations
  ---------
* chore(mpc_lateral_controller): add package maintainer (`#10239 <https://github.com/autowarefoundation/autoware_universe/issues/10239>`_)
  add package maintainer
* Contributors: Arjun Jagdish Ram, Hayato Mizushima, Kyoichi Sugahara, Yutaka Kondo, mkquda

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
* feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in fil… (`#9846 <https://github.com/autowarefoundation/autoware_universe/issues/9846>`_)
  * feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files ontrol/autoware_mpc_lateral_controller
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(autoware_mpc_lateral_controller): fix formula description in vehicle_model_bicycle_kinematics.hpp (`#8971 <https://github.com/autowarefoundation/autoware_universe/issues/8971>`_)
  fix formula description in vehicle_model_bicycle_kinematics.hpp
* fix(mpc_lateral_controller): prevent unstable steering command while stopped (`#9690 <https://github.com/autowarefoundation/autoware_universe/issues/9690>`_)
  * modify logic of function isStoppedState
  * use a constant distance margin instead of wheelbase length
  * add comment to implementation
  ---------
* feat(mpc_lateral_controller): remove trans/rot deviation validation since the control_validator has the same feature (`#9684 <https://github.com/autowarefoundation/autoware_universe/issues/9684>`_)
* docs: modified minor sign error (`#8140 <https://github.com/autowarefoundation/autoware_universe/issues/8140>`_)
* Contributors: Autumn60, Fumiya Watanabe, Takayuki Murooka, Vishal Chauhan, Yuki Kimura, mkquda

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
* fix(autoware_mpc_lateral_controller): fix clang-tidy errors (`#9436 <https://github.com/autowarefoundation/autoware_universe/issues/9436>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(mpc_lateral_controller): suppress rclcpp_warning/error (`#9382 <https://github.com/autowarefoundation/autoware_universe/issues/9382>`_)
  * feat(mpc_lateral_controller): suppress rclcpp_warning/error
  * fix
  * fix test
  ---------
* fix(autoware_mpc_lateral_controller): fix variableScope (`#9390 <https://github.com/autowarefoundation/autoware_universe/issues/9390>`_)
* feat: suppress warning/error of the empty predicted trajectory by MPC (`#9373 <https://github.com/autowarefoundation/autoware_universe/issues/9373>`_)
* chore(autoware_mpc_lateral_controller): add maintainer (`#9374 <https://github.com/autowarefoundation/autoware_universe/issues/9374>`_)
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
* fix(autoware_mpc_lateral_controller): fix bugprone-misplaced-widening-cast (`#9224 <https://github.com/autowarefoundation/autoware_universe/issues/9224>`_)
  * fix: bugprone-misplaced-widening-cast
  * fix: consider negative values
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(mpc_lateral_controller): correctly resample the MPC trajectory yaws (`#9199 <https://github.com/autowarefoundation/autoware_universe/issues/9199>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kosuke Takeuchi, Kyoichi Sugahara, M. Fatih Cırıt, Maxime CLEMENT, Ryohsuke Mitsudome, Ryuta Kambe, Takayuki Murooka, Yutaka Kondo, kobayu858

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix(autoware_mpc_lateral_controller): fix bugprone-misplaced-widening-cast (`#9224 <https://github.com/autowarefoundation/autoware_universe/issues/9224>`_)
  * fix: bugprone-misplaced-widening-cast
  * fix: consider negative values
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(mpc_lateral_controller): correctly resample the MPC trajectory yaws (`#9199 <https://github.com/autowarefoundation/autoware_universe/issues/9199>`_)
* Contributors: Esteve Fernandez, Maxime CLEMENT, Yutaka Kondo, kobayu858

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(osqp_interface): added autoware prefix to osqp_interface (`#8958 <https://github.com/autowarefoundation/autoware_universe/issues/8958>`_)
* fix(autoware_mpc_lateral_controller): fix calculation method of predicted trajectory (`#9048 <https://github.com/autowarefoundation/autoware_universe/issues/9048>`_)
  * fix(vehicle_model): fix calculation method of predicted trajectory
  ---------
* refactor(autoware_interpolation): prefix package and namespace with autoware (`#8088 <https://github.com/autowarefoundation/autoware_universe/issues/8088>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* chore(mpc_lateral_controller): consistent parameters with autoware_launch (`#8914 <https://github.com/autowarefoundation/autoware_universe/issues/8914>`_)
* chore: remove duplicate line in mpc_lateral_controller.cpp (`#8916 <https://github.com/autowarefoundation/autoware_universe/issues/8916>`_)
  remove duplicate line in mpc_lateral_controller.cpp
* feat(autoware_mpc_lateral_controller): add predicted trajectory acconts for input delay (`#8436 <https://github.com/autowarefoundation/autoware_universe/issues/8436>`_)
  * feat: enable delayed initial state for predicted trajectory
  * feat: enable debug publishing of predicted and resampled reference trajectories
  ---------
* fix(autoware_mpc_lateral_controller): fix cppcheck warnings (`#8149 <https://github.com/autowarefoundation/autoware_universe/issues/8149>`_)
  * fix(autoware_mpc_lateral_controller): fix cppcheck warnings
  * Update control/autoware_mpc_lateral_controller/src/lowpass_filter.cpp
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
  ---------
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
* fix(autoware_mpc_lateral_controller): add timestamp and frame ID to published trajectory (`#8164 <https://github.com/autowarefoundation/autoware_universe/issues/8164>`_)
  add timestamp and frame ID to published trajectory
* fix(controller): revival of dry steering (`#7903 <https://github.com/autowarefoundation/autoware_universe/issues/7903>`_)
  * Revert "fix(autoware_mpc_lateral_controller): delete the zero speed constraint (`#7673 <https://github.com/autowarefoundation/autoware_universe/issues/7673>`_)"
  This reverts commit 69258bd92cb8a0ff8320df9b2302db72975e027f.
  * dry steering
  * add comments
  * add minor fix and modify unit test for dry steering
  ---------
* fix(autoware_mpc_lateral_controller): delete the zero speed constraint (`#7673 <https://github.com/autowarefoundation/autoware_universe/issues/7673>`_)
  * delete steer rate limit when vel = 0
  * delete unnecessary variable
  * pre-commit
  ---------
* fix(autoware_mpc_lateral_controller): relax the steering rate constraint at zero speed (`#7581 <https://github.com/autowarefoundation/autoware_universe/issues/7581>`_)
  * constraint for zero velocity updated
  * correct the comment
  ---------
* fix(autoware_mpc_lateral_controller): fix duplicateExpression warning (`#7542 <https://github.com/autowarefoundation/autoware_universe/issues/7542>`_)
  * fix(autoware_mpc_lateral_controller): fix duplicateExpression warning
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_mpc_lateral_controller): fix duplicateAssignExpression warning (`#7572 <https://github.com/autowarefoundation/autoware_universe/issues/7572>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* fix(mpc_lateral_controller): align the MPC steering angle when the car is controlled manually. (`#7109 <https://github.com/autowarefoundation/autoware_universe/issues/7109>`_)
  * align the MPC steering angle when the car is controlled manually.
  * update the condition for is_driving_manually
  * STOP mode included
  * comment the is_driving_manually
  * align the steering outside (after) the solver.
  * use the flag input_data.current_operation_mode.is_autoware_control_enabled
  * correct a typo
  * correct the under control condition check
  * undo the space delete
  * unchange the unrelevant line
  * pre-commit
  ---------
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
* refactor(mpc_lateral_controller, trajectory_follower_node)!: prefix package and namespace with autoware (`#7306 <https://github.com/autowarefoundation/autoware_universe/issues/7306>`_)
  * add the prefix to the folder
  * named to autoware_mpc_lateral_controller
  * rename the folder in the include
  * correct the package name in xml and CMakeLists
  * correct the namespace and include
  * change namespace and include in src/
  * change namespace and include in test/
  * fix the trajectory_follower_node
  * undo rename to the namespace
  * change the trajectory_follower_node, Controller.drawio.svg, and README.md
  * fixed by pre-commit
  * suppress the unnecessary line length detect
  ---------
* Contributors: Autumn60, Esteve Fernandez, Kosuke Takeuchi, Kyoichi Sugahara, Ryuta Kambe, Satoshi OTA, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, Zhe Shen, mkquda

0.26.0 (2024-04-03)
-------------------
