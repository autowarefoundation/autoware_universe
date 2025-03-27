^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_system_diagnostic_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* feat(path_optimizer): additional failure logging and failure mode handling (`#10276 <https://github.com/autowarefoundation/autoware_universe/issues/10276>`_)
  MRM when MPT fails
* Contributors: Arjun Jagdish Ram, Hayato Mizushima, Yutaka Kondo

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
* feat: apply `autoware` prefix for `component_state_monitor` and its dependencies (`#9961 <https://github.com/autowarefoundation/autoware_universe/issues/9961>`_)
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
* fix(localization_error_monitor, system diag): fix to use diagnostics_module in localization_util (`#8543 <https://github.com/autowarefoundation/autoware_universe/issues/8543>`_)
  * fix(localization_error_monitor): fix to use diagnostics_module in localization_util
  * fix: update media
  * fix: update component name
  * fix: rename include file
  ---------
* feat(autonomous_emergency_braking): enable AEB emergency stop (`#8255 <https://github.com/autowarefoundation/autoware_universe/issues/8255>`_)
  enable AEB emergency stop
* fix(system_diagnostic_monitor): fix local mode config (`#7532 <https://github.com/autowarefoundation/autoware_universe/issues/7532>`_)
* feat(tier4_system_launch): modify diagnostic_graph_aggregator_graph argument (`#7133 <https://github.com/autowarefoundation/autoware_universe/issues/7133>`_)
* feat(default_ad_api): use diagnostic graph (`#7043 <https://github.com/autowarefoundation/autoware_universe/issues/7043>`_)
* feat(system diags): rename diag of ndt scan matcher (`#6889 <https://github.com/autowarefoundation/autoware_universe/issues/6889>`_)
  rename ndt diag
* feat: remake diagnostic graph packages (`#6715 <https://github.com/autowarefoundation/autoware_universe/issues/6715>`_)
* Contributors: RyuYamamoto, Takagi, Isamu, Yamato Ando, Yutaka Kondo, danielsanchezaran

0.26.0 (2024-04-03)
-------------------
* feat(system_diagnostic_graph): change config file format (`#5340 <https://github.com/autowarefoundation/autoware_universe/issues/5340>`_)
* feat: system diagnostic monitor (`#4722 <https://github.com/autowarefoundation/autoware_universe/issues/4722>`_)
* Contributors: Takagi, Isamu
