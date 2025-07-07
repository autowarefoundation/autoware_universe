^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_lidar_marker_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* fix(lidar_marker_localizer): fix typo in launch file (`#10405 <https://github.com/autowarefoundation/autoware_universe/issues/10405>`_)
  fix typo
* Contributors: Ryohsuke Mitsudome, Yamato Ando

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
* fix(autoware_lidar_marker_localization): fix segmentation fault (`#8943 <https://github.com/autowarefoundation/autoware_universe/issues/8943>`_)
  * fix segmentation fault
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
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
* Contributors: Fumiya Watanabe, Yamato Ando, kminoda

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
* refactor(autoware_point_types): prefix namespace with autoware::point_types (`#9169 <https://github.com/autowarefoundation/autoware_universe/issues/9169>`_)
* refactor(localization_util)!: prefix package and namespace with autoware (`#8922 <https://github.com/autowarefoundation/autoware_universe/issues/8922>`_)
  add autoware prefix to localization_util
* feat(localization): add `lidar_marker_localizer` (`#5573 <https://github.com/autowarefoundation/autoware_universe/issues/5573>`_)
  * Added lidar_marker_localizer
  * style(pre-commit): autofix
  * fix launch file
  * style(pre-commit): autofix
  * Removed subscriber_member_function.cpp
  * Renamed the package and the node
  * style(pre-commit): autofix
  * Removed pose_array_interpolator
  * Removed unused files
  * Removed include dir
  * style(pre-commit): autofix
  * Renamed wrong names
  * fix magic number
  * style(pre-commit): autofix
  * fix bug
  * parameterized
  * style(pre-commit): autofix
  * add base_covariance
  * style(pre-commit): autofix
  * Removed std::cerr
  * Removed unused code
  * Removed unnecessary publishers
  * Changed to use alias
  * Fixed result_base_link_on_map
  * Changed to use "using std::placeholders"
  * Refactored points_callback
  * Fixed as pointed out by linter
  * Refactored lidar_marker_localizer
  * Fixed const reference
  * Refactor point variables
  * Added detect_landmarks
  * rework filering params
  * fix marker position
  * style(pre-commit): autofix
  * fix build error
  * fix marker position
  * style(pre-commit): autofix
  * update readme
  * style(pre-commit): autofix
  * Added calculate_diff_pose
  * Fixed to pass linter
  * update package.xml
  * Fixed to use SmartPoseBuffer
  * Fixed function calculate_diff_pose to calculate_new_self_pose
  * Compatible with the latest landmark_manager
  * Fixed pub_marker
  * Fixed launch
  * Removed unused arg
  * Removed limit_distance_from_self_pose_to_marker_from_lanelet2
  * Fixed parse_landmarks
  * Fixed parameter type
  * Fixed typo
  * rework diagnostics
  * style(pre-commit): autofix
  * rotate covariance
  * style(pre-commit): autofix
  * add json schema
  * style(pre-commit): autofix
  * parameterize marker name
  * python to xml
  * update launch files
  * style(pre-commit): autofix
  * add debug/pose_with_covariance
  * style(pre-commit): autofix
  * update readme
  * update readme
  * add depend
  * add sample dataset
  * add param marker_height_from_ground
  * style(pre-commit): autofix
  * fix typo
  * add includes
  * add name to TODO comment
  * style(pre-commit): autofix
  * rename lidar-marker
  * modify sample dataset url
  * add flowchat to readme
  * fix callbackgroup
  * add TODO comment
  * fix throttle timer
  * delete unused valriable
  * delete unused line
  * style(pre-commit): autofix
  * fix the duplicated code
  * style(pre-commit): autofix
  * avoid division by zero
  * fix TODO comment
  * fix uncrustify failed
  * style(pre-commit): autofix
  * Update localization/landmark_based_localizer/lidar_marker_localizer/src/lidar_marker_localizer.cpp
  Co-authored-by: SakodaShintaro <rgbygscrsedppbwg@gmail.com>
  * change lint_common
  * update CMakeLists
  * save intensity func
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * fix build error
  * style(pre-commit): autofix
  * apply PointXYZIRC
  * add autoware prefix
  * componentize
  * move directory
  * use localization_util's diagnostics lib
  * style(pre-commit): autofix
  * applay linter
  * style(pre-commit): autofix
  * to pass spell-check
  * remove _ex
  * refactor
  * style(pre-commit): autofix
  * remove unused depend
  * update readme
  * fix typo
  * fix json
  * fix autoware prefix
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: yamato-ando <Yamato ANDO>
  Co-authored-by: Yamato Ando <yamato.ando@gmail.com>
  Co-authored-by: yamato-ando <yamato.ando@tier4.jp>
* Contributors: Esteve Fernandez, Masaki Baba, SakodaShintaro, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
