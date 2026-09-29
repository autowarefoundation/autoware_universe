^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_traffic_light_compliance_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(traffic_light_compliance_checker): improve compliance checker stability and amber handling (`#13371 <https://github.com/autowarefoundation/autoware_universe/issues/13371>`_)
  * modify tl compliance checker to detect stop attempts at amber light and (optionally) add tl id to force rejection buffer
  - update traffic_light_stop params in modifier
  - update traffic_light filter params in validator
  - update modifier and validator parameter structs
  - implement logic to detect stops at ember light and optionally add to force rejection buffer
  - update & refactor traffic_light filter tests
  * refactor tl compliance checker
  - update amber rejection hystory while checking for violations instead of post process
  - apply allow_if_cannot stop check while checking for violations instead of post process
  * improve crossig time limit logic
  compute dynamic time limit for amber light based on tracked tl duration instead of using fixed value param
  * check the flag reject_if_stop_detected before adding tl to amber_rejection_history\_
  * add test cases to traffic_light_stop and traffic_light_filter
  * minor refactor
  * fix(traffic_light_stop): floor scan length with min_lookahead_distance and wire stop time step (`#3232 <https://github.com/autowarefoundation/autoware_universe/issues/3232>`_)
  * fix(traffic_light_stop): check full traj horizon and wire stop time step
  At low ego speed the compliance checker capped the scan by comfortable
  stopping distance (~0.5 m), missing red stop lines ahead. Also assign
  trajectory_time_step\_ so the existing 3-point stop fallback uses the
  configured step.
  Co-authored-by: Cursor <cursoragent@cursor.com>
  * fix(traffic_light): floor scan length with min_lookahead_distance
  Restore the comfortable-stop scan cap but floor it at 20 m so creeping
  ego still sees nearby stop lines, without rejecting far lights in the
  shared traffic_light_filter path.
  Co-authored-by: Cursor <cursoragent@cursor.com>
  ---------
  Co-authored-by: Cursor <cursoragent@cursor.com>
  * fix cherry-pick errors
  * implement arrow aware amber tl compliance check
  - Track YellowState (kNotYellow / kFromGreen / kFromNonGreen) in TrafficLightStatusTracker from raw Green Circle → Amber transitions
  - Skip stop-line collection for arrow-aware amber when enable_arrow_aware_yellow_passing, turn lane, mapped static arrow, and kFromGreen all hold
  - Keep Red→Amber / unknown-origin amber as stop (no override)
  - Add shared utils (is_equal, has\_*_circle, has_static_arrow, is_arrow_aware_amber_pass) used by tracker and checker
  - Add enable_arrow_aware_yellow_passing (default true) and wire it through trajectory_modifier traffic_light_stop and trajectory_validator traffic_light_filter (params, schema, config)
  - Document arrow-aware amber behavior in the compliance checker README
  - Add unit tests for yellow-transition tracking and end-to-end arrow-aware amber cases
  * hold last stable TL status while gated states settle
  - Track current candidate and last stable signal separately in TrafficLightStatusTracker
  - Emit the last stable status while red/amber/unknown are below their stable-duration thresholds instead of clearing elements
  - Update YellowState only when the accepted stable status changes, avoiding single-frame false detections
  - Pass through raw signals when ego is stopped for responsiveness, while still updating stable history
  - Accept candidate states with duration >= threshold (including immediate green)
  * replace yellow usage by amber
  * fix test
  * rename test file and extend test cases for compliance checker
  * improve TL compliance checker to prevent chattering behavior
  - refactor TrafficLightStatusTracker::filter_signals to emit all stable statuses in the history, not only the ones in this frame
  - keep a history of violation arc lengths per id
  - use previous violation arc lengths to floor current frame lookahead distance
  - use current frame collected stop lines to cleanup the history of violation arc lengths
  * Update common/autoware_traffic_light_compliance_checker/include/autoware/traffic_light_compliance_checker/utils.hpp
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * feat(traffic_light_stop, traffic_light_compliance_checker): fix inconsistent amber rejection behavior (`#3380 <https://github.com/autowarefoundation/autoware_universe/issues/3380>`_)
  * update modifier traffic_light_stop parameters
  - modify traffic_light_stop config to be consistent with traffic_light_filter
  - update parameter_struct.yaml and schema
  - update integration tests
  - add delay_response_time param
  * ensure distance to stop line and crossing time are measured with respect to ego front
  * update default trajectory_processor config
  ---------
  * fix format
  * remove duplicated set of params
  * fix test
  ---------
  Co-authored-by: Yuxuan Liu <619684051@qq.com>
  Co-authored-by: Cursor <cursoragent@cursor.com>
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* feat(trajectory_modifier, minimum_rule_based_planner): improve obstacle stop feature (`#13255 <https://github.com/autowarefoundation/autoware_universe/issues/13255>`_)
  * fix(trajectory_modifier): fix obstacle stop unstable stop wall (`#3196 <https://github.com/autowarefoundation/autoware_universe/issues/3196>`_)
  * fix stop wall appears behind ego
  * always publish modifier debug markers
  * fix extend_trajectory() function
  - use path curvature at end instead of relying on trajecory point orientations
  - for low end speed trajectory, default to straight extension
  * fix insert_stop_point logic to prevent wrong orientation stop pose
  * introduce minimum stop margin below which duplicate_check_threshold is ignored
  * handle zero stop_point_arc_length in insert_stop_point function
  ---------
  * refactor(obstacle_stop): optimize collision check logic (`#3024 <https://github.com/autowarefoundation/autoware_universe/issues/3024>`_)
  refactor obstacle stop utility function get_nearest_object_collision, update default param values
  * feat(trajectory_modifier): remove max object velocity threshold for obstacle stop (`#3234 <https://github.com/autowarefoundation/autoware_universe/issues/3234>`_)
  * remove max_velocity_th param, fix collision check logic
  * tune rss params
  * apply pre-commit checks
  ---------
  * feat(trajectory_modifier, backup_planner): enable filtering objects by type and shape (`#3255 <https://github.com/autowarefoundation/autoware_universe/issues/3255>`_)
  * enable filtering objects by type and shape
  - update obstacle_stop params in modifier and backup planner to specify enabled types per shape
  - update surround_obstacle_stop params in modifier and backup planner to specify enabled types per shape
  - modify ObjectFilter code in obstacle_stop_utils to support filtering by type and shape
  - modify ObstacleProximityChecker class to support filtering by type and shape
  - fix ObstacleTracker logic for ignoring orientation change
  * Update planning/autoware_trajectory_modifier/include/autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * apply pre-commit checks
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * fix cherry-pick errors
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* fix(autoware_traffic_light_compliance_checker): avoid reading uninitialized Parameters (`#13042 <https://github.com/autowarefoundation/autoware_universe/issues/13042>`_)
* feat(traffic_light_compliance_checker): allow when close and cannot stop (`#13028 <https://github.com/autowarefoundation/autoware_universe/issues/13028>`_)
* fix(traffic_light_compliance_checker): improve stop point detection (`#13003 <https://github.com/autowarefoundation/autoware_universe/issues/13003>`_)
* feat(traffic_light_compliance_checker): param for stable UNKNOWN signal (`#12936 <https://github.com/autowarefoundation/autoware_universe/issues/12936>`_)
* feat(trajectory_modifier): add traffic light stop to trajectory modifier (`#12875 <https://github.com/autowarefoundation/autoware_universe/issues/12875>`_)
  * feat(trajectory_modifier): implement traffic light stop plugin in modifier (`#3017 <https://github.com/autowarefoundation/autoware_universe/issues/3017>`_)
  * add traffic_light_stop plugin framework
  * refactor traffic_light_compliance_checker, add crossing point and arc length to Violation struct
  * add required inputs to trajectory_modifer
  - add subscribers for lanelet_map, route, and signals
  - add lanelet_map, route, and signal ptrs to trajectory_modifier plugin InputData struct
  - update and refactory trajectory_modifier node to process new input data
  * implement is is_trajectory_modification_required function for traffic_light_stop plugin
  * refactor setting stop point logic and add utility functions to commonize logic
  * implement traffic light stopping logic
  * publish debug string
  * set default param values, update schema
  * check for empty trajectory
  ---------
  * feat(traffic_light_compliance_checker): add documentation for traffic light compliance checker  (`#3085 <https://github.com/autowarefoundation/autoware_universe/issues/3085>`_)
  * write the readme for traffic_light_compliance_checker package
  * add unit tests for traffic light stop
  * update traffic_light_filter documentation, add docs file for traffic_light_stop module
  ---------
  * feat(traffic_light_stop): add missing parameter treat_unknown_light_as_red (`#3091 <https://github.com/autowarefoundation/autoware_universe/issues/3091>`_)
  add missing parameter treat_unknown_light_as_red
  * fix spelling
  ---------
* Contributors: Koichi Imai, Maxime CLEMENT, Ryohsuke Mitsudome, mkquda

0.52.0 (2026-06-30)
-------------------
* chore: align package versions to 0.51.0 and reset changelogs
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(traffic_light_filter): sync with E2E development branch (`#12813 <https://github.com/autowarefoundation/autoware_universe/issues/12813>`_)
* Contributors: Maxime CLEMENT, github-actions
