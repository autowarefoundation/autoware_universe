^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_path_direction_change_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.52.0 (2026-06-26)
-------------------
* chore: align package versions to 0.51.0 and reset changelogs
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(behavior_path_planner): add direction change module. Initial Commit (`#12638 <https://github.com/autowarefoundation/autoware_universe/issues/12638>`_)
  * feat: add direction change module. Initial Commit
  * style(pre-commit): autofix
  * refactor: direction_change module
  * style(pre-commit): autofix
  * feat: dont flip bounds for reverse manuever for direction_change tagged lanelets
  the reason is, the nature of reversing in direction_change module and start_planner is different in terms of ego orientation, lane direction, direction of ego motion.
  direction_change lanelet bounds doesn't need reversing
  * style(pre-commit): autofix
  * fix: address issues of path snapping with multiple cusps; build path from dc tagged lanelets
  * style(pre-commit): autofix
  * refactor: remove unused methods and helpers
  * style(pre-commit): autofix
  * fix: pre-commit cpp lint errors
  * fix: lint errors
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: emmeyteja, github-actions
