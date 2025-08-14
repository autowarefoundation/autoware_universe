^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_radar_fusion_to_detected_object
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Mete Fatih Cırıt

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* chore(radar): delete maintainer for radar packages (`#10815 <https://github.com/autowarefoundation/autoware_universe/issues/10815>`_)
  * chore(radar): delete maintainer name
  * fix typo
  ---------
* Contributors: Satoshi Tanaka, TaikiYamada4

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
* chore: refine maintainer list (`#10110 <https://github.com/autowarefoundation/autoware_universe/issues/10110>`_)
  * chore: remove Miura from maintainer
  * chore: add Taekjin-san to perception_utils package maintainer
  ---------
* Contributors: Fumiya Watanabe, Shunsuke Miura, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_radar_fusion_to_detected_object): fix bugprone-errors (`#9654 <https://github.com/autowarefoundation/autoware_universe/issues/9654>`_)
  fix: bugprone-error
* Contributors: Fumiya Watanabe, kobayu858

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
* fix(autoware_radar_fusion_to_detected_object): fix variableScope (`#8429 <https://github.com/autowarefoundation/autoware_universe/issues/8429>`_)
  fix:variableScope
  Co-authored-by: kobayu858 <129580202+kobayu858@users.noreply.github.com>
* fix(autoware_radar_fusion_to_detected_object): fix functionConst (`#8287 <https://github.com/autowarefoundation/autoware_universe/issues/8287>`_)
  fix:functionConst
* fix(autoware_radar_fusion_to_detected_object): fix warnings of functionStatic (`#8052 <https://github.com/autowarefoundation/autoware_universe/issues/8052>`_)
  fix: deal with functionStatic warning
* fix(autoware_radar_fusion_to_detected_object): fix constParameterReference (`#8040 <https://github.com/autowarefoundation/autoware_universe/issues/8040>`_)
  fix:constParameterReference
* refactor(radar)!: add package name prefix of autoware\_ (`#7892 <https://github.com/autowarefoundation/autoware_universe/issues/7892>`_)
  * refactor: rename radar_object_tracker
  * refactor: rename package from radar_object_tracker to autoware_radar_object_tracker
  * refactor: rename package from radar_object_clustering to autoware_radar_object_clustering
  * refactor: rename package from radar_fusion_to_detected_object to autoware_radar_fusion_to_detected_object
  * refactor: rename radar_crossing_objects_noise_filter to autoware_radar_crossing_objects_noise_filter
  * refactor: rename object_velocity_splitter to autoware_object_velocity_splitter
  * refactor: rename object_range_splitter to autoware_object_range_splitter
  * refactor: update readme
  ---------
* Contributors: Nagi70, Taekjin LEE, Yutaka Kondo, kobayu858, taisa1

0.26.0 (2024-04-03)
-------------------
