^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory_selector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* fix(planning): fix ENABLE_AGNOCAST=1 build and startup of trajectory_selector (`#13364 <https://github.com/autowarefoundation/autoware_universe/issues/13364>`_)
  * fix(trajectory_selector): fix ENABLE_AGNOCAST=1 build
  * build(trajectory_ranker): add autoware_agnocast_wrapper_setup to the library target
  ---------
* refactor(trajectory_adapter): integrate trajectory adapter into selector node (`#13355 <https://github.com/autowarefoundation/autoware_universe/issues/13355>`_)
  * feat(trajectory_adapter): integrate trajectory adapter into selector node (`#3312 <https://github.com/autowarefoundation/autoware_universe/issues/3312>`_)
  integrate trajectory adapter into selector node
  - Extract TrajectoryAdapter and TrajectoryAdapterWrapper from the standalone adapter node
  - Integrate adapter into the selector pipeline after trajectory ranking
  - Publish planning trajectory and turn indicators from trajectory_selector_node
  - Keep latency debug publishing in TrajectoryAdapterWrapper
  - Convert autoware_trajectory_adapter from a standalone node into a shared library
  - Remove the standalone trajectory adapter node and its launch file
  - Add autoware_trajectory_adapter, autoware_planning_msgs, and autoware_vehicle_msgs dependencies to the selector package
  - Update trajectory_selector.launch.xml with trajectory and turn-indicator output remaps
  * apply pre-commit checks
  * add missing include
  ---------
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
* refactor(planning): move node design files into each package (`#13102 <https://github.com/autowarefoundation/autoware_universe/issues/13102>`_)
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
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
* fix(trajectory_validator): replace is feasible input from trajectory points to candidate trajectory (`#12985 <https://github.com/autowarefoundation/autoware_universe/issues/12985>`_)
  fix(trajectory_validator): replace is feasible input from trajectory points to candidate trajectory (`#3101 <https://github.com/autowarefoundation/autoware_universe/issues/3101>`_)
  * fix(trajectory_validator): replace is feasible input from trajectory points to candidate trajectory
  * fix: update trajectory selector test
  ---------
* Contributors: Koichi Imai, Ryohsuke Mitsudome, Taekjin LEE, Yutaro Kobayashi, Zulfaqar Azmi, mkquda

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(trajectory_selector): anchor the concatenation on the main input (`#12722 <https://github.com/autowarefoundation/autoware_universe/issues/12722>`_)
* feat(planning): apply autoware_agnocast_wrapper to diffussion planner trajectory pipeline nodes for CIE (`#12779 <https://github.com/autowarefoundation/autoware_universe/issues/12779>`_)
  * feat(autoware_diffusion_planner): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_optimizer): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_adapter): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_ranker): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_selector): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_modifier): apply autoware_agnocast_wrapper for CIE
  ---------
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
* Contributors: Maxime CLEMENT, Zulfaqar Azmi, atsushi yano, github-actions
