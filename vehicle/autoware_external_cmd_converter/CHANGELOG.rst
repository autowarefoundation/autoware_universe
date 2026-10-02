^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_external_cmd_converter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(external_cmd_converter): apply agnocast_wrapper::Node to external_cmd_converter (`#13014 <https://github.com/autowarefoundation/autoware_universe/issues/13014>`_)
  * feat(external_cmd_converter): apply agnocast_wrapper::Node to external_cmd_converter
  Co-authored-by: Koichi Imai <koichi.imai.2@tier4.jp>
  * fix: cppcheck
  * refactor: revert comment
  * refactor: revert comment
  * refactor: remove qos
  * refactor: migrate to polling AIP
  * style(pre-commit): autofix
  * fix: clang format
  * refactor: subscriber
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Koichi Imai <koichi.imai.2@tier4.jp>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* fix(vehicle): declare the dependencies these packages use (`#13208 <https://github.com/autowarefoundation/autoware_universe/issues/13208>`_)
  * fix(vehicle): declare the dependencies these packages use
  Four packages use headers or symbols of packages that they never declare. Add the 10 missing entries: 8 <depend> and 2 <test_depend>.
  * fix(vehicle): declare the script runtime dependencies and tf2
  The review of the first commit found nine more missing entries.
  autoware_steer_offset_estimator uses tf2::Quaternion and tf2::Vector3 in an installed header and in the library code, so it gets <depend>tf2</depend>.
  The installed Python scripts of autoware_accel_brake_map_calibrator import rclpy, ament_index_python, numpy, and yaml, and its launch file starts rviz2. The installed plot script of autoware_raw_vehicle_cmd_converter imports ament_index_python, matplotlib, and numpy. These get <exec_depend> entries.
  ---------
* refactor(vehicle): move node design files into each package (`#13098 <https://github.com/autowarefoundation/autoware_universe/issues/13098>`_)
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* Contributors: Mete Fatih Cırıt, Ryohsuke Mitsudome, Taekjin LEE, Yutaro Kobayashi

0.52.0 (2026-06-30)
-------------------

0.51.0 (2026-05-01)
-------------------

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------

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
* Merge remote-tracking branch 'origin/main' into humble
* feat: manual control (`#10354 <https://github.com/autowarefoundation/autoware_universe/issues/10354>`_)
  * feat(default_adapi): add manual control
  * add conversion
  * update selector
  * update selector depends
  * update converter
  * modify heartbeat name
  * update launch
  * update api
  * fix pedal callback
  * done todo
  * apply message rename
  * fix test
  * fix message type and qos
  * fix steering_tire_velocity
  * fix for clang-tidy
  ---------
* Contributors: Ryohsuke Mitsudome, Takagi, Isamu

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
* fix(cpplint): include what you use - vehicle (`#9575 <https://github.com/autowarefoundation/autoware_universe/issues/9575>`_)
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
* feat(autoware_external_cmd_converter): add ext cmd converter tests (`#9118 <https://github.com/autowarefoundation/autoware_universe/issues/9118>`_)
  * add ext cmd converter tests
  * add briefs to describe the tests
  * update tests and add nan and inf check
  ---------
* refactor(autoware_external_cmd_converter): add explanation about external control commands (`#8224 <https://github.com/autowarefoundation/autoware_universe/issues/8224>`_)
  * refactor(autoware_external_cmd_converter): add explanation about external control commands
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Tomoya Kimura <tomoya.kimura@tier4.jp>
* fix(autoware_external_cmd_converter): fix check_topic_state (`#7921 <https://github.com/autowarefoundation/autoware_universe/issues/7921>`_)
  * fix(autoware_external_cmd_converter): fix check_topic_state
  * style(pre-commit): autofix
  * Update vehicle/autoware_external_cmd_converter/src/node.cpp
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: shtokuda <shumpei.tokuda@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(external cmd converter)!: add autoware\_ prefix (`#7361 <https://github.com/autowarefoundation/autoware_universe/issues/7361>`_)
  * add prefix to the code
  * rename
  * fix
  * fix
  * fix
  * Update .github/CODEOWNERS
  ---------
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
* Contributors: Kosuke Takeuchi, SHtokuda, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, danielsanchezaran

0.26.0 (2024-04-03)
-------------------
