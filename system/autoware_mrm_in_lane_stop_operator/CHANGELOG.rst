^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_mrm_in_lane_stop_operator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(autoware_mrm_in_lane_stop_operator): add node (`#13332 <https://github.com/autowarefoundation/autoware_universe/issues/13332>`_)
  * feat: add node
  * style(pre-commit): autofix
  * fix: refactor
  * fix: refactor
  * fix: spelling
  * feat(autoware_mrm_in_lane_stop_operator): switch trigger to InLaneStopTrigger with profile-based config
  Replace ConstantJerkDecelerationTrigger/target_acceleration/target_jerk
  with tier4_system_msgs/InLaneStopTrigger and a named deceleration
  profile (moderate/emergency) per mode.
  Extract ModeConfig into its own file with a profile_from_name() helper,
  and extract mode lookup/binding into a ModeTable class to keep the
  mode-name/id and profile-name/value associations in one place.
  * fix(autoware_mrm_in_lane_stop_operator): send mode's profile on cancel
  publish_trigger(false, ...) was passing PROFILE_UNKNOWN on cancel instead
  of the mode's own profile.
  * style(pre-commit): autofix
  * fix(autoware_mrm_in_lane_stop_operator): add missing includes for cpplint
  Add <utility> for std::move in mode_table.hpp and <string> in
  mode_config.cpp/mode_table.cpp, per cpplint's
  build/include_what_you_use.
  * feat(autoware_mrm_in_lane_stop_operator): use reliable + transient_local QoS for trigger topic
  Late-joining subscribers (e.g. the in-lane stop planner starting after
  this node) now receive the last published InLaneStopTrigger instead of
  missing it.
  * fix(autoware_mrm_in_lane_stop_operator): decouple skip_relay_call from trigger publish
  execute()/cancel() were only reached when skip_relay_call was false, due
  to short-circuit evaluation in on_request()'s condition and an early
  return in cancel_active_mode(). This meant InLaneStopTrigger was never
  published while skip_relay_call was true.
  skip_relay_call should only control whether the relay service is called;
  it must not affect whether or what gets published on the trigger topic.
  Move the flag check inside execute()/cancel() so the trigger is always
  published, and only the relay service call is skipped.
  * feat(autoware_mrm_in_lane_stop_operator): switch mode transitions on profile, not mode id
  Previously, requesting a different mode id always cancelled the current
  mode before activating the new one, even when both modes carried the
  same deceleration profile. Now:
  - If the newly requested mode resolves to the same profile as the
  currently active mode, do nothing, even if the mode id differs.
  - If the profile differs, switch straight to the new mode without an
  intermediate cancel.
  - Cancel only happens when the request is unhandled (not one of our
  configured modes) or when the requested mode's own profile is
  PROFILE_UNKNOWN.
  * feat(autoware_mrm_in_lane_stop_operator): log when mode changes but profile does not
  Makes the no-op path in on_request() observable when a different mode
  id is requested but resolves to the same profile as the currently
  active mode.
  * docs(autoware_mrm_in_lane_stop_operator): document profile-based mode transitions
  Explain on_request()'s no-op-on-same-profile and skip-cancel-on-profile-
  change behavior, and when cancellation actually happens.
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Tetsuhiro Kawaguchi
