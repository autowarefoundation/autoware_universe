^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_in_lane_mrm_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(autoware_in_lane_mrm_planner): keep publishing road border stop reason while latched (`#13432 <https://github.com/autowarefoundation/autoware_universe/issues/13432>`_)
  * feat(autoware_in_lane_mrm_planner): keep publishing road border stop reason while latched
  While the in-lane stop trigger is latched the candidates are not re-planned,
  so the road border stop planning factor and debug markers stopped being
  published at the trigger and the stop reason disappeared from rviz during
  the MRM. Re-publish the contact the latched trajectory was planned with
  (debug markers and the planning factor, whose distance is measured from the
  current ego pose) until the trigger is released. The stop pose is kept with
  the contact since the latched trajectory may be resampled afterwards.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
  * refactor(autoware_in_lane_mrm_planner): remove unused RoadBorderContact::stop_index
  The stop point is now referenced by stop_pose only.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* feat(autoware_in_lane_mrm_planner): add road border stop (`#13430 <https://github.com/autowarefoundation/autoware_universe/issues/13430>`_)
  * feat(autoware_in_lane_mrm_planner): add road border stop
  Insert a stop point before the vehicle footprint interferes with a map road
  border (REQ-003 of the in-lane stop planning design, Phase2). Ported from
  mkuri/autoware.universe feat/in-lane-mrm-planner-road-border-stop (69bc5ef3a,
  reviewed as `TetsuKawa/autoware_universe#13 <https://github.com/TetsuKawa/autoware_universe/issues/13>`_) onto the InLaneStopTrigger /
  per-profile planner.
  - Add MrmRoadBorderStopPlanner: sweeps the footprint along the candidate
  trajectory from the ego nearest point, queries lanelet2 linestring segments
  of the configured types (default road_border) through an R-tree rebuilt only
  when the map instance changes, refines the contact arc length by bisection
  and inserts a stop point stop_margin before the contact, never behind the
  ego. Segments outside ego z +/- vehicle height are ignored.
  - Apply it in the trajectory modifier right after the obstacle stop, once per
  cycle on the shared base trajectory; the stop point reaches
  MrmStopVelocityPlanner as a zero velocity constraint for every profile, so
  deceleration relaxation (per profile) stays in one place.
  - Publish PlanningFactor (STOP) under in_lane_mrm_road_border_stop and debug
  markers (contact footprint / segment / point, stop virtual wall).
  - Add road_border_stop parameters (yaml, config, schema) and README section.
  - Add 11 unit tests with a synthetic lanelet map.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
  * fix(autoware_in_lane_mrm_planner): start road border contact search at the ego pose
  The sweep started at the trajectory point returned by findNearestSegmentIndex,
  i.e. the start of the segment the ego is on, up to one point interval behind
  base_link. A border just behind the ego rear could therefore be reported as a
  contact at the ego (stop at ego), and the debug contact footprint was drawn
  behind the vehicle.
  - Evaluate the ego footprint first, then only trajectory points ahead of the
  ego; the bisection refines between the ego pose and the first interfering
  point.
  - Keep the refined contact pose in RoadBorderContact and draw the debug
  contact footprint there (the contact index shifts when the stop point is
  inserted).
  - Add tests for a border behind the ego footprint and a contact refined from
  an ego pose between trajectory points.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
  * docs(autoware_in_lane_mrm_planner): document road border stop output topics
  The debug marker topic `~/road_border_stop/debug/marker` is node-relative and
  resolves to `/in_lane_mrm_planner/road_border_stop/debug/marker` with the
  default launch, not `/planning/...` as assumed in verification procedures.
  List the planning factor and marker topics explicitly and describe the stop
  wall position and the ego-first footprint sweep.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
  * refactor(autoware_in_lane_mrm_planner): remove unused road border stop members
  - Remove the unused debug_footprints\_ member and RoadBorderContact::contact_index
  (the stop point is based on contact_arc_length).
  - Document why closest_point_on_segment is used instead of
  bg::closest_points (not available in Boost 1.74 on ROS 2 Humble).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* fix(autoware_in_lane_mrm_planner): clamp relaxation to the profile limits (`#13429 <https://github.com/autowarefoundation/autoware_universe/issues/13429>`_)
  fix(autoware_in_lane_mrm_planner): clamp deceleration/jerk relaxation to the profile limits
  The relaxation loop added the step and only then compared with
  max\_*_relaxation, so a step that does not divide the range evenly overshot
  the limit (e.g. target -1.5, step -1.0, max -3.0 -> -2.5 -> -3.5). Clamp each
  step with std::max(value + step, max) for both jerk and deceleration so the
  relaxed constraint never exceeds the profile's max\_*_relaxation.
  Behaviour change: a stop that was feasible only thanks to the overshoot now
  falls through to the existing max-relaxation fallback (unchanged here).
  Co-authored-by: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* fix(autoware_in_lane_mrm_planner): read storage_identifier from mcap bag metadata (`#13428 <https://github.com/autowarefoundation/autoware_universe/issues/13428>`_)
  * fix(autoware_in_lane_mrm_planner): read storage_identifier from mcap bag metadata
  rosbag2 (humble) writes `storage_identifier: mcap` in metadata.yaml, which the
  `storage_id:` regex did not match, so the analyzer failed to open mcap bags.
  Accept both keys.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
  * fix(autoware_in_lane_mrm_planner): fix spell-check warning in storage id regex
  Spell out storage_identifier and storage_id as alternatives instead of
  storage_id(?:entifier)? so that cspell does not flag "entifier".
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* feat(in_lane_mrm_planner): use in lane stop trigger msg (`#13427 <https://github.com/autowarefoundation/autoware_universe/issues/13427>`_)
  * feat(autoware_in_lane_mrm_planner): add per-profile MRM stop velocity constraints
  Replace the single mrm_velocity target/relaxation parameters with
  mrm_velocity.profiles.{moderate,emergency}.{target_deceleration,target_jerk,
  max_deceleration_relaxation,max_jerk_relaxation}, matching the
  tier4_system_msgs/msg/InLaneStopTrigger profiles. Relaxation steps and
  brake_delay_time stay shared.
  Defaults follow the L4 constraints: moderate -3.0 m/s^2 / -5.0 m/s^3
  (relaxation limits -4.0 / -10.0), emergency -6.0 / -20.0 (-8.0 / -30.0).
  MrmStopVelocityPlanner::apply() and select_profile_limits() take the
  profile to plan with (default: moderate).
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
  * feat(autoware_in_lane_mrm_planner): subscribe InLaneStopTrigger with per-profile constraints
  - Remove mrm_trigger_relay_node; the planner subscribes
  tier4_system_msgs/msg/InLaneStopTrigger directly on ~/input/trigger
  (launch default /system/in_lane_stop/trigger) with reliable +
  transient_local depth 1, since the operator publishes on change only.
  - Plan the path once per cycle and keep a candidate for every profile
  (moderate, emergency). The moderate candidate is the hot-standby output.
  - trigger=true latches the requested profile's candidate (retried every
  cycle if none exists yet, e.g. planner started after the trigger).
  - A profile change while triggered re-plans from the current state and
  re-latches the new profile; only a candidate planned in that cycle is
  latched, otherwise the current latch is kept and the re-latch retried.
  - trigger=false unlatches.
  - An unknown profile value (PROFILE_UNKNOWN) with trigger=true falls back
  to the moderate profile with a throttled error log (design decision to
  be reviewed).
  - planner_status gains requested_profile (12) and latched_profile (13).
  - rosbag scripts read /system/in_lane_stop/trigger.
  Depends on `tier4/tier4_autoware_msgs#240 <https://github.com/tier4/tier4_autoware_msgs/issues/240>`_.
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
  * fix(autoware_in_lane_mrm_planner): fix spell-check and cppcheck warnings
  - Rename LatchAction::RELATCH to RE_LATCH (cspell unknown word).
  - Deduce the size of kAllStopProfiles from its initializer and add
  kNumStopProfiles derived from it. cppcheck 2.13 cannot parse
  kAllStopProfiles.size() in a member array type, so it did not see
  latest_candidates\_ as a member and reported false functionStatic
  warnings on TrajectoryLatcher::update_candidate/has_candidate.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* feat(in_lane_mrm_planner): add packages (`#13349 <https://github.com/autowarefoundation/autoware_universe/issues/13349>`_)
  * feat: add packages
  * style(pre-commit): autofix
  * fix: move package
  * fix: move to planning component
  * fix(autoware_in_lane_mrm_planner): prevent duplicate trajectory points that crash the MPC (`#149 <https://github.com/autowarefoundation/autoware_universe/issues/149>`_)
  * fix(autoware_in_lane_mrm_planner): prevent duplicate trajectory points that crash the MPC
  Root cause of the 2026-06-30 MRM trajectory follower SIGABRT: the published
  trajectory contained consecutive points at the same position, violating the
  strictly increasing arc-length assumption of MPC spline resampling
  (guarded follower-side by autoware.universe `#3127 <https://github.com/autowarefoundation/autoware_universe/issues/3127>`_; this fixes the source).
  The duplicate was minted in densify_near_arc_length(): the accumulated
  resample grid and the exact terminal arc length differ by a few ulps, so
  the std::set<double> sampled both, emitting two points at the trajectory
  end at the same position (bitwise-identical in the incident rosbag).
  - densify_near_arc_length(): drop arc-length samples closer than half the
  resample interval to their predecessor; always keep the exact terminal
  (replacing a colliding grid sample) so the stop point is preserved.
  - Add remove_overlap_points() as a final publish net (1 mm, applied after
  the velocity planner and before the read-only validator): removes
  (near-)duplicate points while keeping the minimum velocity of an overlap
  run so a duplicated stop point does not lose v=0. Removal count is
  logged (5 s throttle) and exposed as planner_status data[11]
  (sanitized_points); analyzer scripts and README updated accordingly.
  - Regression tests include the verbatim 12-point trajectory published at
  11:44:25.408 that crashed the follower.
  - Record deferred follow-ups (shift-to-ego short-trajectory fallback,
  lateral-offset self-intersection at sharp kinks, diagnostics design) in
  README "Future tasks".
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
  * feat(in_lane_mrm_planner): compensate brake actuation delay in MRM stop profile (`#150 <https://github.com/autowarefoundation/autoware_universe/issues/150>`_)
  * feat: add mrm_velocity.brake_delay_time parameter
  * feat: delay deceleration ramp by brake_delay_time in MRM stop profile
  * docs: note drive-side delay modeling as future task
  * docs: record brake_delay_time review follow-ups as future tasks
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
  * fix: check ptr (`#152 <https://github.com/autowarefoundation/autoware_universe/issues/152>`_)
  * fix(in_lane_mrm_planner): reword terms flagged by spell-check
  Replace "SIGABRT" and "ulps" in comments/docs with plain wording so the
  autoware spell-check-differential job accepts the ported `#149 <https://github.com/autowarefoundation/autoware_universe/issues/149>`_ changes.
  Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
  * fix(autoware_in_lane_mrm_planner): use tier4_system_msgs::msg::InLaneStopTrigger for the trigger relay
  mrm_trigger_relay_node subscribed to
  tier4_control_msgs::msg::ConstantJerkDecelerationTrigger and only ever
  read its `trigger` field. Switch to
  tier4_system_msgs::msg::InLaneStopTrigger, which also has a `trigger`
  field, so the relay logic is unchanged.
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
  * fix(autoware_in_lane_mrm_planner): resolve cppcheck findings
  - take_data(): drop the InputData field assignments that ran before
  each subscriber's take_data() check; they were always immediately
  overwritten by the assignment right after that check, so cppcheck
  correctly flagged them as redundant.
  - Mark TrajectorySelectorStub::select, InLaneMrmTrajectoryValidator::
  has_finite_values, MrmStopVelocityPlanner::fill_ego_prefix/
  fill_zero_velocity_profile/apply_zero_stop_profile, and PathPlanner::
  shift_trajectory_to_ego/convert_path_to_trajectory static: none of
  them touch instance state, confirmed by inspection and by none of
  their classes having a base class to override. apply_zero_stop_profile
  only needed this after fill_zero_velocity_profile became static.
  - path_planner.hpp: rename shift_trajectory_to_ego's declared
  `shift_params` parameter to `params` to match the definition
  (funcArgNamesDifferent).
  Verified with `cppcheck --enable=performance,style --inconclusive`
  locally: none of these findings remain.
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
  * ci(cspell): add project words for in_lane_mrm_planner
  latcher/Latcher/LATCHER (TrajectoryLatcher, PredictedObjectsLatcher)
  and the matplotlib sharex/axvspan identifiers used by the rosbag
  analyzer script are all legitimate, just not in cspell's dictionary.
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
  * fix(autoware_in_lane_mrm_planner): resolve pre-commit-lite findings
  flake8-ros (flake8-docstrings, flake8-comprehensions):
  - in_lane_mrm_planner_rosbag_analyzer.py, in_lane_mrm_planner_rosbag_record.py:
  mark the module docstrings raw (r\"\"\") since they contain backslash
  line continuations in the Usage examples (D301).
  - in_lane_mrm_planner_rosbag_analyzer.py: rewrite two
  sorted(set(... for ...)) generators as set comprehensions (C401), and
  a dict comprehension that just copies an existing (key, value)
  sequence as dict(...) (C416).
  - in_lane_mrm_rosbag_common.py: reword find_rising_edges' docstring to
  imperative mood (D401); reformat detect_emergency_episodes'
  docstring so the summary is a single line ending in a period,
  followed by a blank line before the extended description (D205,
  D400).
  cpplint:
  - test_param_validation.cpp: add the missing <string> include for
  std::string (build/include_what_you_use).
  Verified locally with flake8 (using this repo's setup.cfg) and
  cpplint --quiet: no findings remain in the touched files.
  Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Makoto Kurihara <mkuri8m@gmail.com>
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
  Co-authored-by: Makoto Kurihara <cld-makoto.kurihara@tier4.jp>
* Contributors: Makoto Kurihara, Ryohsuke Mitsudome, Tetsuhiro Kawaguchi
