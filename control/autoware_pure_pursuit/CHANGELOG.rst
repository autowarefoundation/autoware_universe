^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_pure_pursuit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat: change planning output topic name to /planning/trajectory (`#11135 <https://github.com/autowarefoundation/autoware_universe/issues/11135>`_)
  * change planning output topic name to /planning/trajectory
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Yukihiro Saito

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* refactor(pure_pursuit): rework parameters (`#8196 <https://github.com/autowarefoundation/autoware_universe/issues/8196>`_)
* Contributors: Batuhan Beytekin, TaikiYamada4

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
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* fix: add missing includes to autoware_universe_utils (`#10091 <https://github.com/autowarefoundation/autoware_universe/issues/10091>`_)
* Contributors: Fumiya Watanabe, Ryohsuke Mitsudome, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_pure_pursuit)!: tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_pure_pursuit (`#9849 <https://github.com/autowarefoundation/autoware_universe/issues/9849>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files control/autoware_pure_pursuit
* Contributors: Fumiya Watanabe, Vishal Chauhan

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
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
* ci(pre-commit): update cpplint to 2.0.0 (`#9557 <https://github.com/autowarefoundation/autoware_universe/issues/9557>`_)
* fix(cpplint): include what you use - control (`#9565 <https://github.com/autowarefoundation/autoware_universe/issues/9565>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_pure_pursuit): fix cppcheck unusedFunction (`#9276 <https://github.com/autowarefoundation/autoware_universe/issues/9276>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Ryuta Kambe, Yutaka Kondo, awf-autoware-bot[bot]

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_pure_pursuit): fix cppcheck unusedFunction (`#9276 <https://github.com/autowarefoundation/autoware_universe/issues/9276>`_)
* Contributors: Esteve Fernandez, Ryuta Kambe, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(pure_pursuit): add autoware\_ prefix in launch file (`#8687 <https://github.com/autowarefoundation/autoware_universe/issues/8687>`_)
* fix(autoware_pure_pursuit): fix unusedFunction (`#8552 <https://github.com/autowarefoundation/autoware_universe/issues/8552>`_)
  fix:unusedFunction
* fix(autoware_pure_pursuit): fix redundantInitialization redundantInitialization (`#8225 <https://github.com/autowarefoundation/autoware_universe/issues/8225>`_)
  * fix(autoware_pure_pursuit): fix redundantInitialization redundantInitialization
  * style(pre-commit): autofix
  * Update control/autoware_pure_pursuit/src/autoware_pure_pursuit_core/interpolate.cpp
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
* fix(autoware_pure_pursuit): fix shadowVariable (`#7932 <https://github.com/autowarefoundation/autoware_universe/issues/7932>`_)
  * fix:shadowVariable
  * refactor: using range-based for loop
  ---------
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(control)!: refactor directory structures of the trajectory followers (`#7521 <https://github.com/autowarefoundation/autoware_universe/issues/7521>`_)
  * control_traj
  * add follower_node
  * fix
  ---------
* refactor(pure_pursuit): prefix package and namespace with autoware\_ (`#7301 <https://github.com/autowarefoundation/autoware_universe/issues/7301>`_)
  * RT1-6683 add autoware prefix to package and namepace
  * fix precommit
  ---------
* Contributors: Kosuke Takeuchi, Ryohsuke Mitsudome, Ryuta Kambe, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, Zulfaqar Azmi, kobayu858

0.26.0 (2024-04-03)
-------------------
