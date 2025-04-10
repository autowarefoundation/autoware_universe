^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_pose_initializer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* feat: apply `autoware\_` prefix for `default_ad_api_helpers` (`#9965 <https://github.com/autowarefoundation/autoware_universe/issues/9965>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <isamu.takagi@tier4.jp>
* feat(autoware_component_interface_specs_universe!): rename package (`#9753 <https://github.com/autowarefoundation/autoware_universe/issues/9753>`_)
* refactor(autoware_universe_utils): add missing 's' in the class of diagnostics_interface (`#9777 <https://github.com/autowarefoundation/autoware_universe/issues/9777>`_)
* feat!: move diagnostics_module from localization_util to unverse_utils (`#9714 <https://github.com/autowarefoundation/autoware_universe/issues/9714>`_)
  * feat!: move diagnostics_module from localization_util to unverse_utils
  * remove diagnostics module from localization_util
  * style(pre-commit): autofix
  * minor fix in pose_initializer
  * add test
  * style(pre-commit): autofix
  * remove unnecessary declaration
  * module -> interface
  * remove unnecessary equal expression
  * revert the remove of template function
  * style(pre-commit): autofix
  * use overload instead
  * include what you use -- test_diagnostics_interface.cpp
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Junya Sasaki, Ryohsuke Mitsudome, kminoda

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
* fix(cpplint): include what you use - localization (`#9567 <https://github.com/autowarefoundation/autoware_universe/issues/9567>`_)
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
* feat(pose_initializer): check error initial pose and gnss pose, output diagnostics (`#8947 <https://github.com/autowarefoundation/autoware_universe/issues/8947>`_)
  * check initial pose error use GNSS pose
  * add pose_error_check_enabled parameter
  * fixed key value name
  * rename diagnostics key value
  * update README
  * fixed schema json
  * fixed type and default in schema json
  * rename key value
  ---------
* refactor(localization_util)!: prefix package and namespace with autoware (`#8922 <https://github.com/autowarefoundation/autoware_universe/issues/8922>`_)
  add autoware prefix to localization_util
* refactor(pose_initializer)!: prefix package and namespace with autoware (`#8701 <https://github.com/autowarefoundation/autoware_universe/issues/8701>`_)
  * add autoware\_ prefix
  * fix link
  ---------
  Co-authored-by: SakodaShintaro <shintaro.sakoda@tier4.jp>
* Contributors: Esteve Fernandez, Masaki Baba, RyuYamamoto, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
