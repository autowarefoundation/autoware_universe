^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_pose_covariance_modifier
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(autoware_ndt_scan_matcher)!: port the package to Autoware Core (`#10688 <https://github.com/autowarefoundation/autoware_universe/issues/10688>`_)
* Contributors: Ryohsuke Mitsudome, TaikiYamada4

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
* feat(autoware_ekf_localizer)!: porting from universe to core 2nd (`#10067 <https://github.com/autowarefoundation/autoware_universe/issues/10067>`_)
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Motz

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* revert: revert "feat(autoware_ekf_localizer)!: porting from universe to core (`#9978 <https://github.com/autowarefoundation/autoware_universe/issues/9978>`_)" (`#10004 <https://github.com/autowarefoundation/autoware_universe/issues/10004>`_)
  This reverts commit 037c315fbee69bb5923ec10bb8e8e70f890725ea.
* feat(autoware_ekf_localizer)!: porting from universe to core (`#9978 <https://github.com/autowarefoundation/autoware_universe/issues/9978>`_)
  * feat: delete ekf_localizer files
  * doc: Modify ekf_localizer directory links
  * ci: remove ekf_localizer from the codecov target list
  ---------
* Contributors: Motz, Ryohsuke Mitsudome

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
* docs(autoware_pose_covariance_modifier): fix gt symbol (`#9082 <https://github.com/autowarefoundation/autoware_universe/issues/9082>`_)
* docs(autoware_pose_cov_modifier): fix line breaks and dead links (`#8991 <https://github.com/autowarefoundation/autoware_universe/issues/8991>`_)
  * fix(autoware_pose_cov_modifier): fix line breaks
  * fix dead links
  ---------
* refactor(autoware_interpolation): prefix package and namespace with autoware (`#8088 <https://github.com/autowarefoundation/autoware_universe/issues/8088>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* fix(autoware_pose_covariance_modifier): fix funcArgNamesDifferent (`#8007 <https://github.com/autowarefoundation/autoware_universe/issues/8007>`_)
  fix:funcArgNamesDifferent
* fix(pose_covariance_modifier): fix json schema (`#7323 <https://github.com/autowarefoundation/autoware_universe/issues/7323>`_)
  fix json schema
  Co-authored-by: Kotaro Yoshimoto <pythagora.yoshimoto@gmail.com>
* fix(autoware_pose_covariance_modifier): change log output from screen to both (`#7198 <https://github.com/autowarefoundation/autoware_universe/issues/7198>`_)
  change log output from screen to both
  Co-authored-by: SakodaShintaro <shintaro.sakoda@tier4.jp>
* feat(autoware_pose_covariance_modifier): add new node to early fuse gnss and ndt poses (`#6570 <https://github.com/autowarefoundation/autoware_universe/issues/6570>`_)
  Co-authored-by: M. Fatih Cırıt <mfc@leodrive.ai>
* Contributors: Esteve Fernandez, Masaki Baba, Yamato Ando, Yutaka Kondo, kobayu858, melike tanrikulu

0.26.0 (2024-04-03)
-------------------
