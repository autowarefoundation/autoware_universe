^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_pid_longitudinal_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* fix(pid_long): fix slope sign (`#13247 <https://github.com/autowarefoundation/autoware_universe/issues/13247>`_)
* refactor(pid_logitudinal_controller): cleanup core logic (`#13071 <https://github.com/autowarefoundation/autoware_universe/issues/13071>`_)
  * refactor(autoware_pid_longitudinal_controller): remove unused m_current_time member
  m_current_time was assigned once per run() but never read anywhere in
  the class.
  * refactor(autoware_pid_longitudinal_controller): remove unused m_prev_nearest_time member
  m_prev_nearest_time was only ever written to (set or reset) and never
  read, so it had no effect on behavior.
  * refactor(autoware_pid_longitudinal_controller): pass is_steer_converged as parameter
  is_steer_converged was cached as a member for one control cycle only
  to be read inside updateControlState(), the sole function that used
  it. Pass it as a parameter to updateControlState() instead of holding
  it as class state.
  * refactor(autoware_pid_longitudinal_controller): remove unimplemented member function declarations
  getCurrentMotion() and calcFilteredAcc() were declared in the header
  but never defined or called anywhere in the class.
  * refactor(autoware_pid_longitudinal_controller): remove redundant m_received_invalid_trajectory member
  The value was only set and read within a single run() call, so it can
  be a local variable derived directly from is_valid_trajectory() instead
  of a persistent member.
  * refactor(autoware_pid_longitudinal_controller): extract debug/slope message creation into free helper functions
  Move the inline debug_message and slope_message construction in run()
  into createDebugMessage() and createSlopeMessage() free functions.
  * refactor(autoware_pid_longitudinal_controller): return emergency stop reason from updateControlState
  Have changeControlState() and updateControlState() return the emergency
  stop reason instead of writing it to the m_emergency_stop_reason member,
  removing the member variable and the reset-before-use pattern in run().
  * refactor(autoware_pid_longitudinal_controller): add const to side-effect-free member functions
  Moves the m_prev_ctrl_cmd update out of createCtrlCmdMsg (which should
  only format the output message) and into calcCtrlCmd where the state
  transition belongs, allowing createCtrlCmdMsg and getTimeUnderControl
  to be marked const.
  * refactor(autoware_pid_longitudinal_controller): make createCtrlCmdMsg a free helper function
  Converts createCtrlCmdMsg into a free function in the anonymous
  namespace, matching createDebugMessage/createSlopeMessage, since it
  does not need access to any member state. Argument order places
  current_time first to align with the other message-creation helpers.
  * fix(autoware_pid_longitudinal_controller): avoid clock-type mismatch in delay-compensation time arithmetic
  PidLongitudinalController compared current_time (whose clock type is
  caller-supplied) against timestamps implicitly converted from
  builtin_interfaces::msg::Time, which defaults to RCL_ROS_TIME. When
  current_time used a different clock type, rclcpp::Time subtraction threw
  "can't subtract times with different time sources".
  Replace the ROS-message-backed m_ctrl_cmd_vec with a dedicated
  TimestampedAcceleration struct that stores rclcpp::Time directly, so
  stored stamps always share current_time's clock type without needing an
  explicit conversion. The remaining message-derived timestamp (trajectory
  header stamp) is now converted using current_time.get_clock_type()
  instead of the implicit RCL_ROS_TIME default.
  * test(autoware_pid_longitudinal_controller): drop clock-type overrides now that current_time flows through unchanged
  make_time() no longer needs an rcl_clock_type_t parameter, and no test
  needs to force RCL_ROS_TIME, since PidLongitudinalController no longer
  implicitly assumes RCL_ROS_TIME for message-derived timestamps.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* test(pid_longitudinal_controller): add unit tests (`#13064 <https://github.com/autowarefoundation/autoware_universe/issues/13064>`_)
  * test(autoware_pid_longitudinal_controller): add unit tests for PidLongitudinalController
  * test(autoware_pid_longitudinal_controller): trim redundant PidLongitudinalController test cases
  Removes several overlapping/redundant test cases while keeping the
  temporal-trajectory coverage (TemporalTrajectoryProducesValidCommand
  and its make_temporal_trajectory helper), which exercises the
  use_temporal_trajectory branch in getControlData.
  * test(autoware_pid_longitudinal_controller): assert control_cmd output alongside control_state
  Most PidLongitudinalController tests only checked the internal control_state
  enum, leaving the actual control_cmd (velocity/acceleration) output that
  drives the vehicle unverified in most state-transition scenarios.
  * refactor(autoware_pid_longitudinal_controller): fix copy right
  * test(autoware_pid_longitudinal_controller): fix state-transition test timing and readability
  Several PidLongitudinalController tests advanced the clock by full
  seconds between run() calls while the ego stayed at velocity 0, which
  exceeded stopped_state_entry_duration_time (0.1s) and forced an
  unintended STOPPED transition before the transition under test could
  be observed. Tighten those gaps to stay under the threshold.
  Also add a make_time(seconds) helper so timestamps read as a point in
  time (e.g. make_time(0.03)) instead of an opaque (seconds, nanoseconds)
  pair, and rebase every test's starting time to 0.0 instead of 1.0.
  * test(pid_longitudinal_controller): drop unused spacing parameter from trajectory helpers
  spacing was always passed as 1.0 at every call site, so fix it as an
  internal constant in make_straight_trajectory/make_temporal_trajectory
  instead of threading it through every test.
  * refactor(autoware_pid_longitudinal_controller): remove redundant static_cast<float> in EXPECT_FLOAT_EQ
  gtest's EXPECT_FLOAT_EQ already converts both arguments to float via
  CmpHelperFloatingPointEQ<float>, so the explicit cast was unnecessary.
  * test(autoware_pid_longitudinal_controller): default steer-convergence guard to disabled
  Most tests need departure without also driving steer convergence, so flip
  make_default_config()'s default for enable_keep_stopped_until_steer_convergence
  to false and let only the two tests that verify the guard opt in explicitly,
  removing 13 repeated per-test overrides.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(pid_logitudinal_controller): extract core logic (`#13029 <https://github.com/autowarefoundation/autoware_universe/issues/13029>`_)
  * refactor(autoware_pid_longitudinal_controller): extract PidLongitudinalControllerConfig logic
  Collect the scattered parameter member variables of PidLongitudinalController
  into a single PidLongitudinalControllerConfig struct, in preparation for
  extracting the core control logic from ROS 2 node concerns.
  * refactor(autoware_pid_longitudinal_controller): include PID/smooth-stop/lpf gains in config
  Move the remaining declare_parameter-based fields (PID gains and limits,
  smooth stop params, and the vel/acc/pitch lowpass filter gains) into
  PidLongitudinalControllerConfig so every ROS parameter feeding the
  control algorithm lives in one struct, ahead of extracting the core
  logic from rclcpp::Node.
  * refactor(autoware_pid_longitudinal_controller): extract core logic from PidLongitudinalController
  Split the ROS 2 node concerns out of PidLongitudinalController into a new
  PidLongitudinalControllerNode wrapper (declared in
  pid_longitudinal_controller_node.hpp, defined alongside the core logic in
  pid_longitudinal_controller.cpp to keep the diff close to the original file).
  The core class no longer depends on rclcpp::Node/rclcpp::Logger/publishers/
  diagnostic_updater: it takes a PidLongitudinalControllerConfig, exposes
  run()/setConfig()/getDebugValues(), and returns the control state and error
  causes (control_state, received_invalid_trajectory, emergency_stop_reason)
  via PidLongitudinalControllerResult instead of a separate getControlState()
  getter or logging directly. The Node builds the config from parameters, owns
  the publishers/diagnostics/parameter callback, and turns the returned causes
  into actual RCLCPP\_* log calls.
  autoware_trajectory_follower_node/src/controller_node.cpp now instantiates
  PidLongitudinalControllerNode instead of the (now core-only)
  PidLongitudinalController.
  * refactor(autoware_pid_longitudinal_controller): move Node functions into pid_longitudinal_controller_node.cpp; drop unused includes
  Move create_config() and all PidLongitudinalControllerNode method
  implementations (ctor, paramCallback, isReady, run, emitLogs,
  publishDebugData, publishVirtualWallMarker, setupDiagnosticUpdater,
  checkControlState) out of pid_longitudinal_controller.cpp into a new
  pid_longitudinal_controller_node.cpp, matching the extract-core-logic
  naming convention of <package>.cpp for core logic and <package>_node.cpp
  for the ROS 2 node wrapper.
  Also remove autoware_utils/ros/marker_helper.hpp and autoware_utils/geometry/
  normalization.hpp includes that were unused (pre-existing dead includes,
  unrelated to the createStopVirtualWallMarker calls, which come from
  autoware/motion_utils/marker/marker_helper.hpp). marker_helper.hpp
  happened to pull in visualization_msgs/msg/marker_array.hpp as a side
  effect, so switch the marker.hpp include to marker_array.hpp directly
  since MarkerArray is the type actually used.
  * refactor(autoware_pid_longitudinal_controller): expose debug and slope data as ready-to-publish messages
  Have PidLongitudinalControllerResult carry debug_message and
  slope_message directly so the node publishes them as-is instead of
  reconstructing Float32MultiArrayStamped from getDebugValues() and
  slope_angle. Also merge publishDebugData() and
  publishVirtualWallMarker() into a single publishMessage().
  * refactor(autoware_pid_longitudinal_controller): seed m_last_running_time on first cycle
  The core class no longer holds a clock to read at construction, so
  m_last_running_time starts as nullptr instead of clock\_->now(). Seed it
  with the first control cycle's current_time so stopped_condition still
  engages after a long standstill even if the vehicle never actually ran,
  matching the previous construction-time-seeded behavior.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(autoware_pid_longitudinal_controller): remove dead code and extract clock now (`#13007 <https://github.com/autowarefoundation/autoware_universe/issues/13007>`_)
  * refactor(autoware_pid_longitudinal_controller): remove clock->now()
  * refactor(autoware_pid_longitudinal_controller): remove debug logs and dead code
  * refactor(autoware_pid_longitudinal_controller): remove unreachable slope source fallback
  m_slope_source is validated as one of the enum values in the constructor,
  so the fallback branch in getControlData() was dead code.
  * refactor(autoware_pid_longitudinal_controller): remove unreachable RCLCPP_FATAL branch in updateControlState
  * refactor(autoware_pid_longitudinal_controller): remove redundant error log in getControlData
  setTrajectory() already logs when the incoming trajectory is invalid or
  has fewer than 2 points, so the duplicate log in getControlData() is
  unnecessary.
  * refactor(autoware_pid_longitudinal_controller): extract IsValidTrajectory logic
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(autoware_pid_longitudinal_controller): prepare for decoupling from rclcpp::Node (`#12994 <https://github.com/autowarefoundation/autoware_universe/issues/12994>`_)
  * refactor(autoware_pid_longitudinal_controller): rename m_trajectory to m_last_valid_trajectory
  * refactor(autoware_pid_longitudinal_controller): fetch current state from ControlData
  Remove m_current_kinematic_state, m_current_accel, and m_current_operation_mode
  member variables. These values are now stored in ControlData and passed
  explicitly through the call chain instead of being cached as node-level state.
  * refactor(autoware_pid_longitudinal_controller): remove unused temporal fields from ControlData
  temporal_observed_time, temporal_window_min, temporal_window_max, and
  temporal_observation_used were never written anywhere, only read for
  debug publishing, so they always carried their default value. Publish
  those debug values as literal defaults instead.
  * refactor(autoware_pid_longitudinal_controller): remove unused m_vehicle_width member
  * refactor(autoware_pid_longitudinal_controller): move toStr to a free function
  * refactor(autoware_pid_longitudinal_controller): move virtual wall marker publish to run()
  Store the marker created in calcEmergencyCtrlCmd() / updateControlState()
  into m_virtual_wall_marker and publish it once from run() via
  publishVirtualWallMarker(), removing the publisher call from the control
  logic to prepare for decoupling from rclcpp::Node.
  * refactor(autoware_pid_longitudinal_controller): remove unused includes
  Remove Eigen/Core, Eigen/Geometry, tf2/utils.hpp, tf2_ros/buffer.h,
  tf2_ros/transform_listener.h, and tf2_msgs/msg/tf_message.hpp, none of
  which are referenced in this header or the corresponding source file.
  Replace geometry_msgs/msg/pose_stamped.hpp with geometry_msgs/msg/pose.hpp,
  since only geometry_msgs::msg::Pose is used, not PoseStamped.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* test(pid_longitudinal_controller): split smooth stop tests (`#12989 <https://github.com/autowarefoundation/autoware_universe/issues/12989>`_)
  * test(autoware_pid_longitudinal_controller): split SmoothStop test into independent scenarios
  Break the single monolithic test into per-scenario TESTs following the
  AAA pattern, and add coverage for the calcTimeToStop() prediction branch
  that was previously untested.
  * refactor(autoware_pid_longitudinal_controller): narrow rclcpp include in smooth stop test
  Only rclcpp::Time is used, so replace the umbrella rclcpp/rclcpp.hpp
  include with rclcpp/time.hpp.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(autoware_pid_longitudinal_controller): refactor smooth stop (`#12983 <https://github.com/autowarefoundation/autoware_universe/issues/12983>`_)
  * refactor(autoware_pid_longitudinal_controller): require SmoothStop::Params at construction
  SmoothStop previously allowed a default-constructed, unconfigured state
  guarded by a bool flag, throwing std::runtime_error from calculate() and
  calcTimeToStop() if setParams() had not been called yet. The Params
  struct is now public and required by the constructor, so an unconfigured
  SmoothStop can no longer be constructed and the runtime checks are
  removed. setParams() remains for the legitimate runtime reconfiguration
  path (ROS dynamic parameter updates), now taking a Params struct instead
  of 11 positional doubles. PidLongitudinalController holds the instance in
  a std::optional since its Params are only known partway through the
  node's parameter declaration.
  Also switch std::experimental::optional to std::optional in SmoothStop,
  since this package builds with C++17 via autoware_cmake's default
  CMAKE_CXX_STANDARD and no longer needs the pre-standardization Library
  Fundamentals TS version.
  * refactor(autoware_pid_longitudinal_controller): decouple SmoothStop from the ROS clock and encapsulate its velocity history
  SmoothStop called rclcpp::Clock::now() internally, coupling this pure
  deceleration-profile logic to the ROS runtime and forcing tests to sleep
  on the real wall clock to exercise its time-based branches. init() now
  takes the current time as an explicit argument instead of querying the
  clock, and calcTimeToStop() derives its reference time from the latest
  sample of the velocity history it is given instead of querying the
  clock separately.
  calcTimeToStop() was only ever called internally and did not use any
  SmoothStop member state, so per this project's convention of minimizing
  member functions in favor of free helper functions, it is moved into an
  anonymous namespace in smooth_stop.cpp.
  Ownership of the velocity history used to predict the time to stop also
  moves from PidLongitudinalController into SmoothStop itself via a new
  recordMotion() method, called once per control cycle. calculate() now
  reads the current velocity, acceleration and time from the latest
  recorded motion sample instead of taking them as separate, redundant
  arguments, and calcTimeToStop() derives its current_time the same way,
  since it was always equal to vel_hist.back().first.
  * refactor(autoware_pid_longitudinal_controller): simplify calculate() and return SmoothStop's mode instead of writing to DebugValues
  Move calcTimeToStop() and is_fast_vel next to where they are used, inside
  the is_running branch, so the stop_dist overshoot early returns no longer
  pay for this computation.
  SmoothStop::calculate() now returns a Result{acc, mode} instead of
  writing the selected mode into a DebugValues reference. This removes
  SmoothStop's dependency on the ROS-specific debug value mechanism; the
  caller writes the mode into DebugValues itself.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
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
* Contributors: Kotakku, Ryohsuke Mitsudome, Takahisa Ishikawa, Yuki TAKAGI

0.52.0 (2026-06-30)
-------------------

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* perf(control): use emplace/emplace_back to avoid temporary object creation (`#12236 <https://github.com/mitsudome-r/autoware_universe/issues/12236>`_)
* feat(pid_longitudinal_controller): parameterize ff_scale limits (`#12415 <https://github.com/mitsudome-r/autoware_universe/issues/12415>`_)
  parameterize ff_scale limits
* fix(autoware_pid_longitudinal_controller): fix test for ROS 2 Jazzy (`#12373 <https://github.com/mitsudome-r/autoware_universe/issues/12373>`_)
  fix(autoware_pid_longitudinal_controller): fix test
* refactor(autoware_pid_longitudinal_controller): remove unused config params (`#12246 <https://github.com/mitsudome-r/autoware_universe/issues/12246>`_)
  * refactor(autoware_pid_longitudinal_controller): remove unused config params
  * fix(README): correct formatting of core parameters table
  ---------
* feat(pid_long): apply slope compensation for stopped acc (`#12007 <https://github.com/mitsudome-r/autoware_universe/issues/12007>`_)
  add slope compensation for stopped acc
* chore: organize maintainer (`#12149 <https://github.com/mitsudome-r/autoware_universe/issues/12149>`_)
* Contributors: Autumn60, Go Sakayori, Ryohsuke Mitsudome, Satoshi OTA, Yuki TAKAGI, github-actions, nishikawa-masaki

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat!: remove ROS 2 Galactic codes (`#11905 <https://github.com/autowarefoundation/autoware_universe/issues/11905>`_)
* refactor(autoware_trajectory_follower_node): remove redundant diagnostic updates from lateral and longitudinal controllers (`#11934 <https://github.com/autowarefoundation/autoware_universe/issues/11934>`_)
* Contributors: Kyoichi Sugahara, Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* feat(pid_longitudinal_controller): don't switch to DRIVE if the state conditions are not met (`#11369 <https://github.com/autowarefoundation/autoware_universe/issues/11369>`_)
  * feat(pid_longitudinal_controller): don't switch to DRIVE if the state conditions are not met
  * add is_autoware_control_enabled field for tests
  ---------
* Contributors: Mert Çolak, Ryohsuke Mitsudome, Tim Clephas

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------

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
