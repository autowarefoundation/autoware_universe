^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory_concatenator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.52.0 (2026-06-26)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(trajectory_selector): anchor the concatenation on the main input (`#12722 <https://github.com/autowarefoundation/autoware_universe/issues/12722>`_)
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
* Contributors: Maxime CLEMENT, Zulfaqar Azmi, github-actions

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(planning): replace autoware_universe_utils with specific autoware_utils sub-packagesr (`#12443 <https://github.com/mitsudome-r/autoware_universe/issues/12443>`_)
* Contributors: Vishal Chauhan, github-actions

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: trajectory concatenator (`#11321 <https://github.com/autowarefoundation/autoware_universe/issues/11321>`_)
  * port autoware_trajectory_concatenator to autoware universe
  * fix launch file
  * remove unused part
  * remove misleading line
  * copilot suggestions
  * review changes
  * remove temp file
  * change hyphen in README
  * remove unnecessary login library
  * remove repeated headers
  * change include to autoware_utils_rclcpp/polling_subscriber.hpp
  * remove unused Odometry using declaration
  * remove unused motion utils
  * add schema
  ---------
* Contributors: Ryohsuke Mitsudome, danielsanchezaran
