^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_boundary_departure_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(boundary_departure): report NEAR_BOUNDARY as a low caution risk (`#13345 <https://github.com/autowarefoundation/autoware_universe/issues/13345>`_)
* chore(pre-commit): update clang-format to v22.1.5 (`#13126 <https://github.com/autowarefoundation/autoware_universe/issues/13126>`_)
  * chore(pre-commit): update clang-format to v22.1.5
  * style(pre-commit): autofix
  ---------
* fix(boundary_departure): separate hysteresis state (`#12986 <https://github.com/autowarefoundation/autoware_universe/issues/12986>`_)
  fix(boundary_departure): separate hysteresis state (`#3116 <https://github.com/autowarefoundation/autoware_universe/issues/3116>`_)
  * fix(boundary_departure): separate hysteresis state
  * fix: separate hash
  * fix: precommit
  ---------
  Co-authored-by: Yuxuan Liu <619684051@qq.com>
* feat(boundary_departure): visualize segments where departure occurs (`#12984 <https://github.com/autowarefoundation/autoware_universe/issues/12984>`_)
  * feat(boundary_departure): visualize segments where departure occurs (`#3094 <https://github.com/autowarefoundation/autoware_universe/issues/3094>`_)
  * fix: add wall marker to visualize boundary departure
  * remove unused header edit
  * fix: make text marker white
  ---------
  * Update common/autoware_boundary_departure_checker/src/detail/debug.cpp
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * Update common/autoware_boundary_departure_checker/src/detail/debug.cpp
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* Contributors: Mete Fatih Cırıt, Ryohsuke Mitsudome, Zulfaqar Azmi

0.52.0 (2026-06-30)
-------------------
* chore: align package versions to 0.51.0 and reset changelogs
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(boundary_departure): boundary departure checker minimal implementation (`#12421 <https://github.com/autowarefoundation/autoware_universe/issues/12421>`_)
  * start a copy of boundary departure checker
  * docs
  * fix: precommit and some debug marker related changes.
  * Revert "fix: precommit and some debug marker related changes."
  This reverts commit 532a781255b57a3576c4b184e09a147c07969d03.
  * Revert "docs"
  This reverts commit 82eda0dca26a84ffabb17438763789ce4af0a9db.
  * addtional changes
  * feat: docs
  * fix: precommit
  * fix: rework initializer
  * fix: remove pyplotter
  * fix: remove rclcpp from test
  * fix: sync debug marker with e2e
  * remove changelog
  * fix: major changes
  * fix: remove dependencies
  * fix: minor refactoring
  * fix(boundary_departure): fix false negative when ego overlap with boundary on front or rear segment. (`#2892 <https://github.com/autowarefoundation/autoware_universe/issues/2892>`_)
  fix(boundary_departure): fix false negative
  * fix: failing test, and bypass ON time buffer if departure point is less than ON time buffer
  * fix: unit test, remove unused includes and added nodiscard
  * lanelet_map_ptr\_ as rtree member
  * refactor: move private imp to detail
  * docs: add readme and parameter schema
  * fix: documents
  * fix: schema
  * fix: schema naming
  * Revert "fix: schema naming"
  This reverts commit e0ca6b04103098619d03f97df057eca0557c11e6.
  * fix: changed json schema
  * revert: wrong changes
  * fix: incorrect "
  * fix: remove unused cmakelist lines and add scenario to explain approaching departure
  ---------
* Contributors: Zulfaqar Azmi, github-actions
