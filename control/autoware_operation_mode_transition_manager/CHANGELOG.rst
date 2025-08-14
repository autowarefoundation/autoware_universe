^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_operation_mode_transition_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat: change planning output topic name to /planning/trajectory (`#11135 <https://github.com/autowarefoundation/autoware_universe/issues/11135>`_)
  * change planning output topic name to /planning/trajectory
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(operation_mode_transition_manager): modify the engage condition (`#11097 <https://github.com/autowarefoundation/autoware_universe/issues/11097>`_)
  * check engage on driving before check engage condition
  * fix
  ---------
* Contributors: Yukihiro Saito, Zhi Wang

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat!: replace tier4_system_msgs with autoware_system_msgs for services (`#10842 <https://github.com/autowarefoundation/autoware_universe/issues/10842>`_)
  * feat!: replace tier4_system_msgs with autoware_system_msgs for services
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(operaton_mode_transition_manager): add flag node (`#10828 <https://github.com/autowarefoundation/autoware_universe/issues/10828>`_)
  * feat(operaton_mode_transition_manager): add flag node
  * rename node
  ---------
* fix(operation_mode_transition_manager): fix mode change without route (`#10749 <https://github.com/autowarefoundation/autoware_universe/issues/10749>`_)
* Contributors: Ryohsuke Mitsudome, TaikiYamada4, Takagi, Isamu

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* feat(operation_mode_transition_manager): add timeout of the input (`#10457 <https://github.com/autowarefoundation/autoware_universe/issues/10457>`_)
  fix(operation_mode_transition_manager): add timeout for the input
* refactor(operation_mode_transition_manager): less member variable (`#10453 <https://github.com/autowarefoundation/autoware_universe/issues/10453>`_)
  * refactor(operatoin_mode_transition_manager): remove unnecessary member variable
  * implement InputData
  * fix
  * fix
  * resolve the reviews
  * add maintainer
  ---------
* Contributors: TaikiYamada4, Takayuki Murooka

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(control): apply THROTTLE to frequent log (`#10418 <https://github.com/autowarefoundation/autoware_universe/issues/10418>`_)
  * fix(control): apply THROTTLE to frequent log
  * fix
  * fix
  * fix
  * fix
  ---------
* Contributors: Ryohsuke Mitsudome, Takayuki Murooka

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* Contributors: Fumiya Watanabe, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_component_interface_specs_universe!): rename package (`#9753 <https://github.com/autowarefoundation/autoware_universe/issues/9753>`_)
* Contributors: Fumiya Watanabe, Ryohsuke Mitsudome

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
* refactor(component_interface_utils): prefix package and namespace with autoware (`#9092 <https://github.com/autowarefoundation/autoware_universe/issues/9092>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

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
* refactor(component_interface_utils): prefix package and namespace with autoware (`#9092 <https://github.com/autowarefoundation/autoware_universe/issues/9092>`_)
* Contributors: Esteve Fernandez, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(component_interface_specs): prefix package and namespace with autoware (`#9094 <https://github.com/autowarefoundation/autoware_universe/issues/9094>`_)
* fix(autoware_operation_mode_transition_manager): fix funcArgNamesDifferent (`#7997 <https://github.com/autowarefoundation/autoware_universe/issues/7997>`_)
  fix: funcArgNamesDifferent_con-1
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* fix(operation mode manager): add nullptr check in onTimer (`#7446 <https://github.com/autowarefoundation/autoware_universe/issues/7446>`_)
* refactor(operation_mode_transition_manager): prefix package and namespace with autoware\_ (`#7291 <https://github.com/autowarefoundation/autoware_universe/issues/7291>`_)
  * RT1-6682 add prefix package and namespace with autoware\_
  * RT1-6682 fix package's description
  ---------
* Contributors: Esteve Fernandez, Kosuke Takeuchi, Mamoru Sobue, Takayuki Murooka, Yutaka Kondo, Zulfaqar Azmi, kobayu858

0.26.0 (2024-04-03)
-------------------
