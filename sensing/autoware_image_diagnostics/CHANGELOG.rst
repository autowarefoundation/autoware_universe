^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_image_diagnostics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(image_blockage_diagnostics): refactor codes for detailed diagnostic (`#10670 <https://github.com/autowarefoundation/autoware_universe/issues/10670>`_)
  * chore: remove default parameter and clean code
  * chore: refactor for diagnostic interface
  * refactor: seperate long functions
  * chore: change variable naming
  * chore: clean code
  * chore: clean functions
  * chore: fix diagnostic message
  * chore: fix color mapping
  * chore: fix spell error
  * chore: fix schema order
  * chore: clean code
  * chore: clean code
  * chore: check parameters
  * chore: add twist
  * chore: remove the warning state for each block
  * chore: remove warning state
  * chore: add timestamp
  * chore: add hardware id
  * chore: add hardware_id
  * chore: remove twist implementation
  * chore: add hysteresis check
  * chore: clean function logic
  * chore: fix schema
  * chore: clean code
  * chore: remove twist related parameters from file
  * chore: add default value in schema
  * chore: changes license
  * chore: fix spell error
  * chore: fix dft image
  * chore: add debug mode
  * chore: only establish publisher when debug is on
  * chore: set default to false
  * chore: unify diagnostics namespace usage for consistency
  * chore: remove updater dependency
  ---------
* Contributors: TaikiYamada4, Yi-Hsiang Fang (Vivid)

0.45.0 (2025-05-22)
-------------------

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore(perception): code owner revision (`#10358 <https://github.com/autowarefoundation/autoware_universe/issues/10358>`_)
  * feat: add Masato Saeki and Taekjin Lee as maintainer to multiple package.xml files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Taekjin LEE

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
* feat(autoware_image_diagnostics): tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_image_diagnostics (`#9918 <https://github.com/autowarefoundation/autoware_universe/issues/9918>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files sensing/autoware_image_diagnostics
* fix(autoware_image_diagnostics): fix bugprone-branch-clone (`#9723 <https://github.com/autowarefoundation/autoware_universe/issues/9723>`_)
  fix: bugprone-error
* Contributors: Fumiya Watanabe, Vishal Chauhan, kobayu858

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
* fix(cpplint): include what you use - sensing (`#9571 <https://github.com/autowarefoundation/autoware_universe/issues/9571>`_)
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

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
* fix(autoware_image_diagnostics): fix cppcheck warnings (`#8228 <https://github.com/autowarefoundation/autoware_universe/issues/8228>`_)
  * fix(autoware_image_diagnostics): fix cppcheck warnings
  * fix
  ---------
* refactor(image_diagnostics): add package name prefix of autoware\_ (`#8130 <https://github.com/autowarefoundation/autoware_universe/issues/8130>`_)
  * refactor: rename image_diagnostics to autoware_image_diagnostics
  * refactor: rename sensing/image_diagnostics to sensing/autoware_image_diagnostics
  ---------
* Contributors: Ryuta Kambe, Taekjin LEE, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
