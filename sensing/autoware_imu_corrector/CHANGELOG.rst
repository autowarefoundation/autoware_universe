^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_imu_corrector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* docs: fix link to deviation_estimator in README (`#11695 <https://github.com/autowarefoundation/autoware_universe/issues/11695>`_)
  Updated the link for deviation_estimator in README.
* feat: implement bias correction controls and update README (`#11474 <https://github.com/autowarefoundation/autoware_universe/issues/11474>`_)
  * chore: modified readme description for clarification on imu correction
  * style(pre-commit): autofix
  * new: flag for static or dynamic bias correction
  * style(pre-commit): autofix
  * chore: throw exception when dual bias correction is selected
  * style(pre-commit): autofix
  * chore: correction configuration loged and moved mutualexclusion
  * style(pre-commit): autofix
  * chore: disabled static bias correctio when static and dynamic are enabled
  * fix: schema updated to pass review
  * chore: enabled static bias by default
  * chore: enhanced readme explanation and fixed default variables state
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Taiki Yamada <129915538+TaikiYamada4@users.noreply.github.com>
* Contributors: Masahiro Kubota, Ryohsuke Mitsudome, SergioReyesSan

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: on off scale and bias correction (`#11314 <https://github.com/autowarefoundation/autoware_universe/issues/11314>`_)
  * feat: adding parameter to turn on or off scale and bias correction
  * style(pre-commit): autofix
  * chore: add parameter to turn on or off scale and bias correction in launcher
  * chore: updating schema
  * chore: add override parameters from file to enable disable scal bias
  * chore: renamed params to enable disble scale bias correction
  * chore: modified schema files
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat: scale gyro estimation (`#10724 <https://github.com/autowarefoundation/autoware_universe/issues/10724>`_)
  * feat: scale estimation for gyro on z axis added
  * after precommit local
  * wip: imu correction with scale and bias
  * feat: added diagnostic messages when scale change
  * monitoring bias and scale gyro status in diagnostics message
  * style(pre-commit): autofix
  * added scale and bias  on purpose for testing
  * style(pre-commit): autofix
  * added flipped gyro scaled bas signal
  * style(pre-commit): autofix
  * fixed bias sign, removed flipped mod signal
  * style(pre-commit): autofix
  * added scale estimation  update function
  * style(pre-commit): autofix
  * Added threshold to avoid estimation when vehicle is stopped
  * style(pre-commit): autofix
  * scale changes real time test
  * changed parameters scale file
  * style(pre-commit): autofix
  * feat: new ekf scale estimation using angle
  * changed offset used for testing
  * bias rotated and not rotated used for angle scale estimation
  * style(pre-commit): autofix
  * added bias to publish and bias not rotated for scale estimation
  * deleting commented lines and renaming bias variable
  * style(pre-commit): autofix
  * rewrites gyro correction to avoid confusion, adds filter to scale estimation
  * style(pre-commit): autofix
  * Adding limits for covariance ekf angle
  * style(pre-commit): autofix
  * removing unused functions
  * new params to detect large scale changes
  * style(pre-commit): autofix
  * renaming node name and changed parameters boundaries
  * style(pre-commit): autofix
  * refactor: main function to estimate scale
  * style(pre-commit): autofix
  * refactoring variables
  * style(pre-commit): autofix
  * new: Added json file for parameters and refactoring
  * style(pre-commit): autofix
  * refactor variables
  * Refactor variables
  * style(pre-commit): autofix
  * new: unittest for scale angle estimation
  * style(pre-commit): autofix
  * added dependencies to package xml
  * style(pre-commit): autofix
  * header files inside include directory refactored , tested functions now public
  * style(pre-commit): autofix
  * moved include files and  code refactored
  * style(pre-commit): autofix
  * refactor add has value checking for variable
  * refactor: removed debug vectors
  * style(pre-commit): autofix
  * refactor: published scale for angle must be divided
  * refactor: added  missing header file
  * style(pre-commit): autofix
  * refactor: added zero checking
  * style(pre-commit): autofix
  * refactor: use abs to perform comparison
  * chore: added scale estimation description to readme
  * style(pre-commit): autofix
  * chore: readme grammar correction
  * refactor: fix to pass CI
  * fix: documentation changed
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, SergioReyesSan

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
* fix(imu_corrector): remove non-periodic publish to /diagnostics topic (`#9951 <https://github.com/autowarefoundation/autoware_universe/issues/9951>`_)
  fix(imu_corrector): remove force_update() in timer callback
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* Contributors: Fumiya Watanabe, interimadd

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
* chore(autoware_imu_corrector): refactored the imu corrector into the autoware namespace (`#8222 <https://github.com/autowarefoundation/autoware_universe/issues/8222>`_)
  * chore: refactored the imu corrector into the autoware namespace
  * chore: reverted to non-exported includes
  ---------
  Co-authored-by: Yamato Ando <yamato.ando@gmail.com>
* Contributors: Kenzo Lobos Tsunekawa, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
