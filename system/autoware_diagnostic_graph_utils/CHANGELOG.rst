^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_diagnostic_graph_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* feat(autoware_deignostic_graph_utils): throttle warning message (`#10758 <https://github.com/autowarefoundation/autoware_universe/issues/10758>`_)
* docs(system): fix system documentation link (`#10707 <https://github.com/autowarefoundation/autoware_universe/issues/10707>`_)
* feat(diagnostic_graph_utils): show the error graph on the terminal only when it changes (`#10665 <https://github.com/autowarefoundation/autoware_universe/issues/10665>`_)
  * feat(diagnostic_graph_utils): show the error graph on the terminal only when it changes
  * update
  ---------
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, TaikiYamada4, Takagi, Isamu, Takayuki Murooka, Yuxuan Liu

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

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat: apply `autoware\_` prefix for `diagnostic_graph_utils` (`#9968 <https://github.com/autowarefoundation/autoware_universe/issues/9968>`_)
* Contributors: Fumiya Watanabe, Junya Sasaki

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* feat(diagnostic_graph_utils): publish error graph instead of the terminal log (`#9421 <https://github.com/autowarefoundation/autoware_universe/issues/9421>`_)
  * feat(diagnostic_graph_utils): publish error graph instead of the terminal log
  * update
  * fix
  * Update system/diagnostic_graph_utils/src/node/logging.cpp
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  * error_graph -> error_graph_text
  ---------
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* fix(cpplint): include what you use - system (`#9573 <https://github.com/autowarefoundation/autoware_universe/issues/9573>`_)
* fix(diagnostic_graph_utils): fix clang-diagnostic-delete-abstract-non-virtual-dtor (`#9431 <https://github.com/autowarefoundation/autoware_universe/issues/9431>`_)
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
* fix(diagnostic_graph_utils): reset graph when new one is received (`#9208 <https://github.com/autowarefoundation/autoware_universe/issues/9208>`_)
  fix(diagnostic_graph_utils): reset graph when new one is reveived
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Ryuta Kambe, Takagi, Isamu, Takayuki Murooka, Yutaka Kondo

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
* fix(diagnostic_graph_utils): reset graph when new one is received (`#9208 <https://github.com/autowarefoundation/autoware_universe/issues/9208>`_)
  fix(diagnostic_graph_utils): reset graph when new one is reveived
* Contributors: Esteve Fernandez, Takagi, Isamu, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* feat(static_centerline_generator): organize AUTO/GUI/VMB modes (`#7432 <https://github.com/autowarefoundation/autoware_universe/issues/7432>`_)
* feat(diagnostic_graph_utils): componentize node (`#7189 <https://github.com/autowarefoundation/autoware_universe/issues/7189>`_)
* feat(default_ad_api): add diagnostics api (`#7052 <https://github.com/autowarefoundation/autoware_universe/issues/7052>`_)
* feat: remake diagnostic graph packages (`#6715 <https://github.com/autowarefoundation/autoware_universe/issues/6715>`_)
* Contributors: Takagi, Isamu, Takayuki Murooka, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
