^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory_validator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* feat(crosswalk_filter): port crosswalk filter improvements (`#13374 <https://github.com/autowarefoundation/autoware_universe/issues/13374>`_)
  * Apply PR `#3318 <https://github.com/autowarefoundation/autoware_universe/issues/3318>`_ changes
  * Apply PR `#3319 <https://github.com/autowarefoundation/autoware_universe/issues/3319>`_ changes
* fix(trajectory_validator): port fixes to trajectory validator (`#13366 <https://github.com/autowarefoundation/autoware_universe/issues/13366>`_)
  * feat(trajectory validator): replace DANGER with HIGH_CAUTION in vehicle constraint filter (`#3188 <https://github.com/autowarefoundation/autoware_universe/issues/3188>`_)
  * fix(trajectory_validator): keep shadow filter metrics in validation report (`#3279 <https://github.com/autowarefoundation/autoware_universe/issues/3279>`_)
  refactor final trajectory risk level calculation
  ---------
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
* feat(boundary_departure): report NEAR_BOUNDARY as a low caution risk (`#13345 <https://github.com/autowarefoundation/autoware_universe/issues/13345>`_)
* feat(trajectory_ranker): implement and integrate ranker into selector node (`#13353 <https://github.com/autowarefoundation/autoware_universe/issues/13353>`_)
  * feat(trajectory_ranker): implement new ranker module and integrate into selector component (`#3208 <https://github.com/autowarefoundation/autoware_universe/issues/3208>`_)
  * add trajectory_ranker_wrapper framework
  * refactor trajectory ranker parameter handling
  * implement trajectory_ranker class framework
  * implement core ranker logic
  - add logic to evaluate trajectories based on risk level
  - add logic to evaluate trajectories based on source
  - use existing metrics based evaluation to evaluate trajectory quality
  * integrate new ranker into trajectory_selector_node
  * refactor code
  * remove obsolete ranker node
  * refactor for debugging
  * add flag to enable/disable ranker within selectory node
  * fix parameter update logic, cleanup code
  * update launch files
  * disable quality evaluation by default
  * fix topic name
  * minor refactor
  * output debug to console when best trajectory has low score
  * support new backup planner dual go/stop trajectories
  * populate generator info of ScoredCandidateTrajectories
  * filter out shadow mode metrics before assigning combined trajectory risk level
  * update source penalties
  * add integration tests for trajectory ranker
  * add ranker parameters schema, update readme
  * remove simple_trajectory_ranker_node
  * remove launch prefix
  * pass active_filter_names to validator from wrapper
  * add missing includes
  ---------
  * replace rclcpp::Node usage by agnocast_wrapper::Node
  * fix selector node tests
  * add missing selector config file
  ---------
* fix(trajectory_selector, trajectory_validator): sync changes to the (`#13344 <https://github.com/autowarefoundation/autoware_universe/issues/13344>`_)
  * fix(trajectory_selector): pass route to validator context and expose validation report
  * feat(trajectory_validator): publish planning factors from validator filters
  ---------
* feat(trajectory_selector): apply `agnocast_wrapper::Node` to `autoware_trajectory_selector` (`#12920 <https://github.com/autowarefoundation/autoware_universe/issues/12920>`_)
  * apply agnocast_wrapper::Node
  * apply agnocast_wrapper::Node
  * fix trajectory_concatenator_wrapper
  * style(pre-commit): autofix
  * fix to not use template
  * style(pre-commit): autofix
  * fix: move bug
  * fix: use {} for agnocast message_ptr null
  * fix: wrap test context assignments in agnocast msg_ptr
  * fix: executor
  * fix: polling
  * refactor: subscriber
  * refactor: skip test in cmake
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: kobayu858 <yutaro.kobayashi.2@tier4.jp>
* feat(trajectory_validator): evaluate the risk of obstructing pedestrian crossing (`#13123 <https://github.com/autowarefoundation/autoware_universe/issues/13123>`_)
  * feat(trajectory_validator): evaluate the risk of obstructing pedestrian crossing (`#3209 <https://github.com/autowarefoundation/autoware_universe/issues/3209>`_)
  * add crosswalk_filter framework
  * implement logic to get target crosswalks
  * visualize target crosswalk
  * implement object filtering logic
  * fix target object processing logic
  * implement feasibility assessment
  * fix is_crossing condition
  * fix logic
  * revise logic for determining target objects
  - implement function to generate detection areas for crosswalk
  - check if detected object is within the detection area or not
  * use parameter instead of hardcoded const
  * add CrosswalkFilter.md to docs
  * add unit tests for crosswalk_filter
  * get highest probability classification
  * add more debug markers
  * add planning factor to crosswalk_filter validation result
  * add flag to enable/disable using trejectory time info for evaluating stop duration
  * visualize arrival distance threshold from crosswalk stop line
  ---------
  * add missing dependency
  * fix cherry-pick related issues
  * minor fixes
  ---------
* fix(boundary_departure): separate hysteresis state (`#12986 <https://github.com/autowarefoundation/autoware_universe/issues/12986>`_)
  fix(boundary_departure): separate hysteresis state (`#3116 <https://github.com/autowarefoundation/autoware_universe/issues/3116>`_)
  * fix(boundary_departure): separate hysteresis state
  * fix: separate hash
  * fix: precommit
  ---------
  Co-authored-by: Yuxuan Liu <619684051@qq.com>
* feat(traffic_light_compliance_checker): allow when close and cannot stop (`#13028 <https://github.com/autowarefoundation/autoware_universe/issues/13028>`_)
* fix(trajectory_validator): replace is feasible input from trajectory points to candidate trajectory (`#12985 <https://github.com/autowarefoundation/autoware_universe/issues/12985>`_)
  fix(trajectory_validator): replace is feasible input from trajectory points to candidate trajectory (`#3101 <https://github.com/autowarefoundation/autoware_universe/issues/3101>`_)
  * fix(trajectory_validator): replace is feasible input from trajectory points to candidate trajectory
  * fix: update trajectory selector test
  ---------
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
* Contributors: Koichi Imai, Maxime CLEMENT, Ryohsuke Mitsudome, Zulfaqar Azmi, mkquda

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(trajectory_validator): add another severity level for metric and validation reports (`#12841 <https://github.com/autowarefoundation/autoware_universe/issues/12841>`_)
  add another severity level for metric and validation report
  - create separate msg RiskLevel.msg
  - move severity levels from Metric/ValidationReport.msg to RiskLevel.msg
  - add new level FATAL
* feat(traffic_light_filter): sync with E2E development branch (`#12813 <https://github.com/autowarefoundation/autoware_universe/issues/12813>`_)
* chore(trajectory_validator): modify validation report msg enums (`#12729 <https://github.com/autowarefoundation/autoware_universe/issues/12729>`_)
  modify msg enums in ValidationReport.msg and MetricReport.msg, and update relevant code
* feat(trajectory_selector): combine validator and concatenator (`#12532 <https://github.com/autowarefoundation/autoware_universe/issues/12532>`_)
  * feat(concatenator): add concatenator
  * feat: combine concatenator with validator
  * fix: remove explicit find package, and pre-commit
  * fix: failing test
  * fix: create public interface for concatenator, and move concatenator to detail folder
  * feat: separate validator to validator interface and initialize selector
  * fix loading parameters
  * fix(node): publish validated trajectories; remove dead member and unjustified mutable
  on_timer() computed the validated result but never called publish(), making
  the node a no-op at the output. Both integration tests were silently timing
  out because of this.
  Also removed sub_trajectories\_ which was declared but never assigned in
  subscribers(), and dropped the unjustified `mutable` qualifier from
  time_keeper\_ (no const method ever writes to it).
  Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
  * fix(validator_interface): use validator_ptr\_ in validate_trajectories
  validate_trajectories() was constructing a new TrajectoryValidator on
  every call (copying the plugins\_ vector each time) instead of using the
  validator_ptr\_ member that is initialized in the constructor for exactly
  this purpose. validator_ptr\_ was live memory that was never called.
  Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
  * fix(validator_interface): remove redundant diagnostics clear; merge duplicate DebugPublisher
  Two cleanups in validate_trajectories / publishers():
  1. The first diagnostics_interface_ptr\_->clear() was dead work: the
  diagnostics are cleared again five lines later, just before the
  add_key_value loop, so the first call never had observable effect.
  2. pub_validation_reports\_ and pub_debug\_ were both initialized to a
  DebugPublisher with the identical prefix "~/debug". A single
  DebugPublisher handles multiple sub-topics; the duplicate object
  added confusion without benefit. Removed pub_validation_reports\_ and
  routed its one call-site through pub_debug\_.
  Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
  * fix: add test
  * fix: rename context
  * style(pre-commit): autofix
  * fix: return if concatenated is empty
  * doc: docstring
  * fix: remove failed spellcheck
  * fix initial processing time value
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
  * fix: rename interface to wrapper
  * remove processing time and add unit test
  * style(pre-commit): autofix
  * separate trajectory selector
  * fix: precommit
  * readme
  * fix: addresses copilot comments
  * fix: address minor copilot comment
  ---------
  Co-authored-by: Claude Sonnet 4.6 <noreply@anthropic.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
* fix(trajectory_validator): remove out of lane filter (`#12653 <https://github.com/autowarefoundation/autoware_universe/issues/12653>`_)
* feat(trajectory_validator): uncrossable boundary departure filter (`#12587 <https://github.com/autowarefoundation/autoware_universe/issues/12587>`_)
  * feat(trajectory_validator): uncrossable boundary departure filter
  * fix: reviewer's comment
  ---------
* refactor(trajectory_validator): separate implementation from node (`#12531 <https://github.com/autowarefoundation/autoware_universe/issues/12531>`_)
  * feat: refactoring validator to prepare for merging nodes
  * fix: move trajectory pipeline
  * rename to stage
  ---------
* Contributors: Maxime CLEMENT, Zulfaqar Azmi, github-actions, mkquda

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore(trajectory_validator): remove a trajectory_validator plugin (`#12522 <https://github.com/mitsudome-r/autoware_universe/issues/12522>`_)
  * delete collision filter
  ---------
* feat(autoware_traffic_light_utils): rewrite hasTrafficLightCircleColor and hasTrafficLightShape into three functions to handle overseas color arrow traffic light (`#12481 <https://github.com/mitsudome-r/autoware_universe/issues/12481>`_)
  * feat(autoware_traffic_light_utils): merge hasTrafficLightCirleColor and hasTrafficLightShape into a general function hasTrafficLightShapeColor to handle oversea color arrow traffic light
  * feat(autoware_traffic_light_utils): merge hasTrafficLightCirleColor and hasTrafficLightShape into a general function hasTrafficLightShapeColor to handle oversea color arrow traffic light
  * fix: modify default parameter for hasTrafficLightShapeColor
  * fix: separate hasTrafficLightShapeColor into three functions
  * chore(miscs): remove unused lanelet2 extension header (`#12081 <https://github.com/mitsudome-r/autoware_universe/issues/12081>`_)
  chore(miscs): remove unused header include for lanelet2_extension
  * fix: revert modification
  * fix: revert modification
  * style(pre-commit): autofix
  * fix: change TrafficLightElement msg belonging
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_deprecated_boundary_departure_checker): replace autoware_universe_utils with autoware_utils_geometry (`#12416 <https://github.com/mitsudome-r/autoware_universe/issues/12416>`_)
* feat(trajectory_validator): publish plugins' processing time and debug markers (`#12483 <https://github.com/mitsudome-r/autoware_universe/issues/12483>`_)
  * feat: trajectory validator markers
  * docs: docstring and time keeper
  * fix: rename time keeper publisher
  ---------
* feat(trajectory_validator): add evaluation tables to propagate filtering results (`#12445 <https://github.com/mitsudome-r/autoware_universe/issues/12445>`_)
  * feat: add evaluation tables to propagate filtering results
  * move shadow mode
  * feat(trajectory_validator): update diagnostic level logic (`#2842 <https://github.com/mitsudome-r/autoware_universe/issues/2842>`_)
  * feat: update diag level logic
  * chore: add comments
  ---------
  * feat(trajectory_validator): add support of publishing validation report (`#2855 <https://github.com/mitsudome-r/autoware_universe/issues/2855>`_)
  * feat: add validator report message
  * feat: replace return value of is_feasible function with ValidationResult
  * feat: add publisher for validation reports
  * feat: update metric name
  * test: update test
  * feat: update metrics
  * refactor: update evaluation result handling
  * fix: resolve build error
  * fix: update cmake and msg level
  ---------
  * fix: remove some artifact
  * fix: remove collision check filter unit test
  * fix: compilation and run error
  * fix: traffic light module inconsistency
  * fix: precommit
  * fix: traffic light filter test
  * fix: unit test
  ---------
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
* feat(planning): replace autoware_universe_utils with specific autoware_utils sub-packagesr (`#12443 <https://github.com/mitsudome-r/autoware_universe/issues/12443>`_)
* feat(trajectory_validator): support shadow mode (`#12478 <https://github.com/mitsudome-r/autoware_universe/issues/12478>`_)
  * feat(trajectory_validator): support shadow mode
  * fix: get shadow mode as param
  * fix: add guard for empty names
  ---------
* refactor(boundary_departure_checker): deprecate legacy rule-based boundary departure checker (`#12420 <https://github.com/mitsudome-r/autoware_universe/issues/12420>`_)
  refactor: separate bdp
* fix(trajectory_validator): diagnostic compares input and output trajectory (`#12444 <https://github.com/mitsudome-r/autoware_universe/issues/12444>`_)
  fix: diagnostic compares input and output trajectory
* refactor(autoware_trajectory_validator): simplify traffic light stop line lookup (`#12428 <https://github.com/mitsudome-r/autoware_universe/issues/12428>`_)
  * refactor(planning): simplify traffic light stop line lookup
  Use traffic light group ids to collect stop lines directly from the
  lanelet map instead of scanning lanelet regulatory elements.
  This removes lanelet-dependent matching, aligns the helper interface
  with the actual inputs, and drops an unused bounding box include.
  * cast traffic light regulatory elements as const
  ---------
* refactor(trajectory_validator): use parameter generated by generate_parameter_library (`#12352 <https://github.com/mitsudome-r/autoware_universe/issues/12352>`_)
  * refactor: use generate parameter library-generated parameters
  * fix: traffic rule filter parameters
  ---------
* feat(trajectory_validator): delete traffic_rule_filter node and move traffic_light_filter (`#12369 <https://github.com/mitsudome-r/autoware_universe/issues/12369>`_)
* feat(trajectory_validator): rename autoware_trajectory_safety_filter to autoware_trajectory_validator (`#12312 <https://github.com/mitsudome-r/autoware_universe/issues/12312>`_)
  * fix: rename safety filter to validator
  * Renaming additional artifacts
  * fix: build error
  * fix: rename config
  * fix: parameter namespace
  * fix: use safety namespace
  ---------
* Contributors: Maxime CLEMENT, Vishal Chauhan, Xiaoyu WANG, Yuki TAKAGI, Yukinari Hisaki, Zulfaqar Azmi, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore(trajectory_safety_filter): fix maintainer (`#12095 <https://github.com/autowarefoundation/autoware_universe/issues/12095>`_)
  put back saito-san
* chore(trajectory_safety_filter): add maintainer (`#12087 <https://github.com/autowarefoundation/autoware_universe/issues/12087>`_)
  * chore(trajectory_safety_filter): add maintainer
  * chore: rearrange maintainer order alphabetically
  * chore: remove Sakoda-san and Saito-san
  ---------
* feat(safety_filter): subscribe to acceleration (`#12030 <https://github.com/autowarefoundation/autoware_universe/issues/12030>`_)
  feat: subscribe to acceleration
* Contributors: Go Sakayori, Ryohsuke Mitsudome, Zulfaqar Azmi

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* feat(autoware_lanelet2_utils): replace from/toBinMsg (Planning and Control Component) (`#11784 <https://github.com/autowarefoundation/autoware_universe/issues/11784>`_)
  * planning component toBinMsg replacement
  * control component fromBinMsg replacement
  * planning component fromBinMsg replacement
  ---------
* Contributors: Ryohsuke Mitsudome, Sarun MUKDAPITAK

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: add safety gate for generator-selector framework (`#11404 <https://github.com/autowarefoundation/autoware_universe/issues/11404>`_)
  * copy packages from new planning framework
  * introduce plugin
  * add/remove plugins
  * fix precommit
  * fix readme
  * fix include guard
  * fix precommit
  * remove test section in CMakeList
  * change definition in README
  * add comment for ttc calculation
  * use lambda function for check_collison function
  * calculate obstacle position only once
  * use boundary departure checker
  * add future work section to README
  * small fix for if condition
  ---------
* Contributors: Go Sakayori, Ryohsuke Mitsudome
