^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_dummy_infrastructure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------

0.46.0 (2025-06-20)
-------------------

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
* feat(Autoware_planning_factor_interface): replace tier4_msgs with autoware_internal_msgs (`#10204 <https://github.com/autowarefoundation/autoware_universe/issues/10204>`_)
* feat(dummy_infrastructure): auto approval when ego stops at stop line (`#10223 <https://github.com/autowarefoundation/autoware_universe/issues/10223>`_)
  feat(dummy_infrastructur): auto approval when ego stops at stop line
* chore(dummy_infrastructure): add maintainers kosuke55, 1222-takeshi, asa-naki (`#10228 <https://github.com/autowarefoundation/autoware_universe/issues/10228>`_)
* Contributors: Hayato Mizushima, Kosuke Takeuchi, Yutaka Kondo, 心刚

0.42.0 (2025-03-03)
-------------------

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat: apply `autoware\_` prefix for `dummy_infrastructure` (`#9969 <https://github.com/autowarefoundation/autoware_universe/issues/9969>`_)
  * feat(dummy_infrastructure): apply `autoware\_` prefix (see below):
  Note:
  * In this commit, I did not organize a folder structure.
  The folder structure will be organized in the next some commits.
  * The changes will follow the Autoware's guideline as below:
  - https://autowarefoundation.github.io/autoware-documentation/main/contributing/coding-guidelines/ros-nodes/directory-structure/#package-folder
  * rename(dummy_infrastructure): move a header under `include/autoware`
  * Fixes due to this changes for .hpp/.cpp files will be applied in the next commit
  * fix(dummy_infrastructure): fix include header path
  * To follow the previous commit
  * rename: `dummy_infrastructure` => `autoware_dummy_infrastructure`
  * bug(autoware_dummy_infrastructure): revert wrongly updated copyrights
  * update(autoware_dummy_infrastructure): `README.md`
  * update: `CODEOWNERS`
  * fix(autoware_dummy_infrastructure): fix package name in CHANGELOG.rst
  * docs(autoware_dummy_infrastructure): fix package name in README and package description
  ---------
  Co-authored-by: Ryohsuke Mitsudome <ryohsuke.mitsudome@tier4.jp>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Junya Sasaki

0.40.0 (2024-12-12)
-------------------
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
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, Ryohsuke Mitsudome, Yutaka Kondo

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
* refactor(dummy_infrastructure): rework parameters (`#5275 <https://github.com/autowarefoundation/autoware_universe/issues/5275>`_)
* Contributors: Yuntianyi Chen, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
* chore: update maintainer (`#4140 <https://github.com/autowarefoundation/autoware_universe/issues/4140>`_)
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* build: mark autoware_cmake as <buildtool_depend> (`#3616 <https://github.com/autowarefoundation/autoware_universe/issues/3616>`_)
  * build: mark autoware_cmake as <buildtool_depend>
  with <build_depend>, autoware_cmake is automatically exported with ament_target_dependencies() (unecessary)
  * style(pre-commit): autofix
  * chore: fix pre-commit errors
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
* build(dummy_infrastructure): add build dependency (`#2739 <https://github.com/autowarefoundation/autoware_universe/issues/2739>`_)
* feat(dummy_infrastructure): change to multiple virtual signal state outputs (`#1717 <https://github.com/autowarefoundation/autoware_universe/issues/1717>`_)
  * Added parameter to dummy_infrastructure.param.yaml
  * Modified dummy infrastructure
  * Modified dummy infrastructure for multiple commands
  * update dummy_infrastructure
* chore: upgrade cmake_minimum_required to 3.14 (`#856 <https://github.com/autowarefoundation/autoware_universe/issues/856>`_)
* refactor: use autoware cmake (`#849 <https://github.com/autowarefoundation/autoware_universe/issues/849>`_)
  * remove autoware_auto_cmake
  * add build_depend of autoware_cmake
  * use autoware_cmake in CMakeLists.txt
  * fix bugs
  * fix cmake lint errors
* ci(pre-commit): update pre-commit-hooks-ros (`#625 <https://github.com/autowarefoundation/autoware_universe/issues/625>`_)
  * ci(pre-commit): update pre-commit-hooks-ros
  * ci(pre-commit): autofix
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* ci: check include guard (`#438 <https://github.com/autowarefoundation/autoware_universe/issues/438>`_)
  * ci: check include guard
  * apply pre-commit
  * Update .pre-commit-config.yaml
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  * fix: pre-commit
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
* feat: change pachage name: autoware_msgs -> tier4_msgs (`#150 <https://github.com/autowarefoundation/autoware_universe/issues/150>`_)
  * change pkg name: autoware\_*_msgs -> tier\_*_msgs
  * ci(pre-commit): autofix
  * autoware_external_api_msgs -> tier4_external_api_msgs
  * ci(pre-commit): autofix
  * fix description
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takeshi Miura <57553950+1222-takeshi@users.noreply.github.com>
* feat: add dummy_infrastructure package (`#19 <https://github.com/autowarefoundation/autoware_universe/issues/19>`_)
  * Feature/add virtual traffic light planner (`#1588 <https://github.com/autowarefoundation/autoware_universe/issues/1588>`_)
  * Change formatter to clang-format and black (`#2332 <https://github.com/autowarefoundation/autoware_universe/issues/2332>`_)
  * Revert "Temporarily comment out pre-commit hooks"
  This reverts commit 748e9cdb145ce12f8b520bcbd97f5ff899fc28a3.
  * Replace ament_lint_common with autoware_lint_common
  * Remove ament_cmake_uncrustify and ament_clang_format
  * Apply Black
  * Apply clang-format
  * Fix build errors
  * Fix for cpplint
  * Fix include double quotes to angle brackets
  * Apply clang-format
  * Fix build errors
  * Add COLCON_IGNORE (`#500 <https://github.com/autowarefoundation/autoware_universe/issues/500>`_)
  * delete COLCON_IGNORE (`#540 <https://github.com/autowarefoundation/autoware_universe/issues/540>`_)
  * add readme [dummy infrastructure] (`#693 <https://github.com/autowarefoundation/autoware_universe/issues/693>`_)
  * add readme dummy infra
  * fix lint
  * update readme
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: taikitanaka3 <65527974+taikitanaka3@users.noreply.github.com>
  Co-authored-by: taikitanaka3 <taiki.tanaka@tier4.jp>
* Contributors: Esteve Fernandez, Kenji Miyake, Takagi, Isamu, Tomoya Kimura, Vincent Richard, Yohei Mishina
