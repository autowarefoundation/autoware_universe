^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_diagnostic_graph_aggregator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat(diagnostic_graph_aggregator): diagnostic graph test interface (`#10455 <https://github.com/autowarefoundation/autoware_universe/issues/10455>`_)
* feat(diagnostic_graph_aggregator): add initializing flag (`#10893 <https://github.com/autowarefoundation/autoware_universe/issues/10893>`_)
* Contributors: Takagi, Isamu

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(diagnostic_graph_aggregator): support latch and dependent error (`#10829 <https://github.com/autowarefoundation/autoware_universe/issues/10829>`_)
  * update aggregator
  * update utils
  * update hazard converter
  * update adapi
  * fix build error
  * ignore spell check for yamls
  * fix copyright year
  * ignore spell check for timeline
  * change link index to fix out of bounds access
  * reflect link index change to utils
  * fix for cppcheck
  * fix for cppcheck
  ---------
* feat(autoware_diagnostic_graph_aggregator): support regex for edits in configuration (`#10671 <https://github.com/autowarefoundation/autoware_universe/issues/10671>`_)
  * feat(autoware_diagnostic_graph_aggregator): support regex for edits in configuration
  * style(pre-commit): autofix
  * MR fixes
  ---------
  Co-authored-by: Marek Piechula <mpiechula@autonomous-systems.pl>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Marek Piechula, TaikiYamada4, Takagi, Isamu

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
* docs(diagnostic_graph_aggregator): update document (`#10199 <https://github.com/autowarefoundation/autoware_universe/issues/10199>`_)
* Contributors: Hayato Mizushima, Mamoru Sobue, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(diagnostic_graph_aggregator): remove edit feature (`#10062 <https://github.com/autowarefoundation/autoware_universe/issues/10062>`_)
  Co-authored-by: Junya Sasaki <junya.sasaki@tier4.jp>
* Contributors: Fumiya Watanabe, Takagi, Isamu

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat: apply `autoware` prefix for `component_state_monitor` and its dependencies (`#9961 <https://github.com/autowarefoundation/autoware_universe/issues/9961>`_)
* Contributors: Fumiya Watanabe, Junya Sasaki

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
* fix(cpplint): include what you use - system (`#9573 <https://github.com/autowarefoundation/autoware_universe/issues/9573>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(diagnostic_graph_aggregator): implement diagnostic graph dump functionality (`#9261 <https://github.com/autowarefoundation/autoware_universe/issues/9261>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(diagnostic_graph_aggregator): implement diagnostic graph dump functionality (`#9261 <https://github.com/autowarefoundation/autoware_universe/issues/9261>`_)
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
* fix(diagnostic_graph_aggregator): fix unusedFunction (`#8580 <https://github.com/autowarefoundation/autoware_universe/issues/8580>`_)
  fix: unusedFunction
  Co-authored-by: kobayu858 <129580202+kobayu858@users.noreply.github.com>
* fix(diagnostic_graph_aggregator): fix noConstructor (`#8508 <https://github.com/autowarefoundation/autoware_universe/issues/8508>`_)
  fix:noConstructor
* fix(diagnostic_graph_aggregator): fix cppcheck warning of functionStatic (`#8266 <https://github.com/autowarefoundation/autoware_universe/issues/8266>`_)
  * fix: deal with functionStatic warning
  * suppress warning by comment
  ---------
* fix(diagnostic_graph_aggregator): fix uninitMemberVar (`#8313 <https://github.com/autowarefoundation/autoware_universe/issues/8313>`_)
  * fix:funinitMemberVar
  * fix:funinitMemberVar
  * fix:uninitMemberVar
  * fix:clang format
  ---------
* fix(diagnostic_graph_aggregator): fix functionConst (`#8279 <https://github.com/autowarefoundation/autoware_universe/issues/8279>`_)
  * fix:functionConst
  * fix:functionConst
  * fix:clang format
  ---------
* fix(diagnostic_graph_aggregator): fix constParameterReference (`#8054 <https://github.com/autowarefoundation/autoware_universe/issues/8054>`_)
  fix:constParameterReference
* fix(diagnostic_graph_aggregator): fix constVariableReference (`#8062 <https://github.com/autowarefoundation/autoware_universe/issues/8062>`_)
  * fix:constVariableReference
  * fix:constVariableReference
  * fix:constVariableReference
  * fix:constVariableReference
  ---------
* fix(diagnostic_graph_aggregator): fix shadowFunction (`#7838 <https://github.com/autowarefoundation/autoware_universe/issues/7838>`_)
  * fix(diagnostic_graph_aggregator): fix shadowFunction
  * feat: modify variable name
  ---------
  Co-authored-by: Takagi, Isamu <isamu.takagi@tier4.jp>
* fix(diagnostic_graph_aggregator): fix uselessOverride warning (`#7768 <https://github.com/autowarefoundation/autoware_universe/issues/7768>`_)
  * fix(diagnostic_graph_aggregator): fix uselessOverride warning
  * restore and suppress inline
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(diagnostic_graph_aggregator): fix shadowArgument warning in create_unit_config (`#7664 <https://github.com/autowarefoundation/autoware_universe/issues/7664>`_)
* feat(diagnostic_graph_aggregator): componentize node (`#7025 <https://github.com/autowarefoundation/autoware_universe/issues/7025>`_)
* fix(diagnostic_graph_aggregator): fix a bug where unit links were incorrectly updated (`#6932 <https://github.com/autowarefoundation/autoware_universe/issues/6932>`_)
  fix(diagnostic_graph_aggregator): fix unit link filter
* feat: remake diagnostic graph packages (`#6715 <https://github.com/autowarefoundation/autoware_universe/issues/6715>`_)
* Contributors: Hayate TOBA, Koichi98, Ryuta Kambe, Takagi, Isamu, Yutaka Kondo, kobayu858, taisa1

0.26.0 (2024-04-03)
-------------------
* feat(diagnostic_graph_aggregator): update tools (`#6614 <https://github.com/autowarefoundation/autoware_universe/issues/6614>`_)
* docs(diagnostic_graph_aggregator): update documents (`#6613 <https://github.com/autowarefoundation/autoware_universe/issues/6613>`_)
* feat(diagnostic_graph_aggregator): add dump tool (`#6427 <https://github.com/autowarefoundation/autoware_universe/issues/6427>`_)
* feat(diagnostic_graph_aggregator): change default publish rate (`#5872 <https://github.com/autowarefoundation/autoware_universe/issues/5872>`_)
* feat(diagnostic_graph_aggregator): rename system_diagnostic_graph package (`#5827 <https://github.com/autowarefoundation/autoware_universe/issues/5827>`_)
* Contributors: Takagi, Isamu
