^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_landmark_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(lidar_marker_localizer): extend lidar-marker localization to enhance flexibility and debugging capabilities (`#11574 <https://github.com/mitsudome-r/autoware_universe/issues/11574>`_)
  * feat: parse landmarks only in target id list
  * feat: Add a y-axis condition to marker selection around the vehicle
  * feat: correct z axis of self pose
  * feat: reference a certain ring
  * feat: make detect_landmarks function templated
  for using PointType in both PointXYZIRC and PointXYZIRADRT
  * feat: make detect_landmarks ring loop robust to empty pointclouds
  - Add check to skip empty ring pointclouds in detect_landmarks
  - Prevent unnecessary processing and possible errors when a ring has no points
  * feat: make save_intensity function for csv output
  - Enable generic processing for multiple point cloud types
  - Prepare for future multi-LiDAR marker localization support
  * refactor: fix member function declarations and add const qualifiers
  * feat: add debug outputs for marker detection process
  * feat: make queue sizes for output pose configurable
  * doc: Add a y-axis condition to marker selection around the vehicle
  * doc: add marker_height_from_ground to lidar_marker_localizer.schema.json
  * feat: add "lidar-marker" pose estimator to pose_estimator_airbiter
  # Conflicts:
  #	launch/tier4_localization_launch/launch/localization.launch.xml
  #	launch/tier4_localization_launch/launch/pose_twist_estimator/lidar_marker_localizer.launch.xml
  #	launch/tier4_localization_launch/launch/pose_twist_estimator/pose_twist_estimator.launch.xml
  * feat: make initial ring id range  configurable
  * doc: update for ndt_lidar-maker mode
  * doc: update for new features in lidar-marker localizer
  * doc: fix indent
  To indicate the contributor's affiliation
  * refactor: update autoware_utils dependency
  * style(pre-commit): autofix
  * doc: update schema.json
  * style(pre-commit): autofix
  * fix: debug topics table and wrong condition to output
  * style(pre-commit): autofix
  * fix: diagnostics_interface dependency
  * chore: remove codes commented out
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* chore(localization, evaluator): remove unused lanelet2_extension header (`#12297 <https://github.com/mitsudome-r/autoware_universe/issues/12297>`_)
  * unused lanelet2_extension in localization component
  * unused lanelet2_extension in evaluator component
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* Contributors: Motz, Sarun MUKDAPITAK, github-actions

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(autoware_landmark_manager): fix deprecated autoware_utils header (`#10515 <https://github.com/autowarefoundation/autoware_universe/issues/10515>`_)
  * fix autoware_utils header
  * fix: remove '\'
  * fix geometry includes
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Takagi, Isamu <43976882+isamu-takagi@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: update maintainer (`#11373 <https://github.com/autowarefoundation/autoware_universe/issues/11373>`_)
  * chore: update maintainer
  remove Ryu Yamamoto
  * chore: update maintainer
  remove Kento Yabuuchi
  * chore: update maintainer
  remove Shintaro Sakoda
  ---------
* Contributors: Kazusa Hashimoto, Motz, Ryohsuke Mitsudome

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
* refactor(localization_util)!: prefix package and namespace with autoware (`#8922 <https://github.com/autowarefoundation/autoware_universe/issues/8922>`_)
  add autoware prefix to localization_util
* refactor(localization): remove unnecessary dependency in localization packages (`#8202 <https://github.com/autowarefoundation/autoware_universe/issues/8202>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Yamato Ando <yamato.ando@gmail.com>
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware_universe/issues/7640>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(ar_tag_based_localizer): add prefix "autoware\_" to ar_tag_based_localizer (`#7483 <https://github.com/autowarefoundation/autoware_universe/issues/7483>`_)
  * Added prefix "autoware\_" to ar_tag_based_localizer
  * style(pre-commit): autofix
  * Fixed localization_launch
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Kento Yabuuchi, Kosuke Takeuchi, Masaki Baba, SakodaShintaro, Takayuki Murooka, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
