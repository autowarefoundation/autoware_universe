^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory_validator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.52.0 (2026-06-26)
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
