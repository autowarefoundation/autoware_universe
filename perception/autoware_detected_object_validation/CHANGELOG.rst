^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_detected_object_validation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* fix(autoware_detected_object_validation): templatize lanelet filter (`#10883 <https://github.com/autowarefoundation/autoware_universe/issues/10883>`_)
  * templatize node and add filter for tracked objects
  * add test for tracked objects
  * Update README.md
  ---------
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
* Contributors: Masaki Baba

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(obstacle_pointcloud_validator): add processing time ms for track… (`#10738 <https://github.com/autowarefoundation/autoware_universe/issues/10738>`_)
  feat(obstacle_pointcloud_validator): add processing time ms for tracking perception performance
  Co-authored-by: Jian Kang <jian.kang@tier4.jp>
* feat(autoware_detected_object_validation): add lanelet object elevation filter (`#10649 <https://github.com/autowarefoundation/autoware_universe/issues/10649>`_)
  * add 3d lanelet filter
  * style(pre-commit): autofix
  * add missing include
  * refactor
  * style(pre-commit): autofix
  * remove unused
  * fix and update
  * fix typo
  * update content
  * make it const
  * remove old comment
  * fix and change the var of args to match
  * remove height filter test and add elevation filter test
  * apply some of clang-tidy suggestions
  * apply of clang-tidy suggestion
  * fix log message
  * fix typo
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* Contributors: Kang, Masaki Baba, TaikiYamada4

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* chore: perception code owner update (`#10645 <https://github.com/autowarefoundation/autoware_universe/issues/10645>`_)
  * chore: update maintainers in multiple perception packages
  * Revert "chore: update maintainers in multiple perception packages"
  This reverts commit f2838c33d6cd82bd032039e2a12b9cb8ba6eb584.
  * chore: update maintainers in multiple perception packages
  * chore: add Kok Seang Tan as maintainer in multiple perception packages
  ---------
* Contributors: Taekjin LEE, TaikiYamada4

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: reduce warn log frequency (`#10414 <https://github.com/autowarefoundation/autoware_universe/issues/10414>`_)
  * feat: limit warning message frequency to 3.0 sec
  * refactor: use THROTTLE ERROR macro
  fix: reduce code complexity
  ---------
  Co-authored-by: Jian Kang <jian.kang@tier4.jp>
* Contributors: Kang, Ryohsuke Mitsudome

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* fix(tier4_perception_launch): add missing exec_depend (`#10130 <https://github.com/autowarefoundation/autoware_universe/issues/10130>`_)
  * fix(tier4_perception_launch): add missing exec_depend
  * add exec_depend for perception
  ---------
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* chore: refine maintainer list (`#10110 <https://github.com/autowarefoundation/autoware_universe/issues/10110>`_)
  * chore: remove Miura from maintainer
  * chore: add Taekjin-san to perception_utils package maintainer
  ---------
* Contributors: Fumiya Watanabe, Shunsuke Miura, Takagi, Isamu, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_detected_object_validation): add height filter in lanelet filtering (`#10003 <https://github.com/autowarefoundation/autoware_universe/issues/10003>`_)
  * feat: add height filter param
  * feat: use ego base height
  * fix: build error
  * feat: add lanelet filter test
  * feat: add height filter test
  * docs: update README and lanelet filter
  * fix: do not getTransform when flag is off
  ---------
* feat(autoware_detected_object_validation): tier4_debug_msgs changed to autoware_internal_debug_msgs in fil… (`#9871 <https://github.com/autowarefoundation/autoware_universe/issues/9871>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files perception/autoware_detected_object_validation
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* feat(autoware_detected_object_validation): set validate distance in the obstacle pointcloud based validator  (`#9663 <https://github.com/autowarefoundation/autoware_universe/issues/9663>`_)
  * chore: add validate_max_distance_m parameter for obstacle_pointcloud_based_validator
  * chore: optimize object distance validation in obstacle_pointcloud_validator
  * chore: add validate_max_distance_m parameter for obstacle_pointcloud_based_validator
  ---------
* Contributors: Fumiya Watanabe, Taekjin LEE, Vishal Chauhan, Yoshi Ri

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
* fix(cpplint): include what you use - perception (`#9569 <https://github.com/autowarefoundation/autoware_universe/issues/9569>`_)
* feat(object_lanelet_filter): add configurable margin for object lanel… (`#9240 <https://github.com/autowarefoundation/autoware_universe/issues/9240>`_)
  * feat(object_lanelet_filter): add configurable margin for object lanelet filter
  modified:   perception/autoware_detected_object_validation/src/lanelet_filter/lanelet_filter.cpp
  * feat(object_lanelet_filter): add condition to check wheter polygon is empty in debug mode
  * feat(object_lanelet_filter): fix cppcheck
  * fix: brig back missing type definition
  * feat: add stop watch for processing time in object lanelet filter
  * feat(object_lanelet_filter): remove extra comment line
  * feat(_object_lanelet_filter): udpate readme
  * style(pre-commit): autofix
  * Update perception/autoware_detected_object_validation/src/lanelet_filter/debug.cpp
  Co-authored-by: Shintaro Tomie <58775300+Shin-kyoto@users.noreply.github.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Sebastian Zęderowski <szederowski@autonomous-systems.pl>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shintaro Tomie <58775300+Shin-kyoto@users.noreply.github.com>
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix(autoware_detected_object_validation): fix bugprone-incorrect-roundings (`#9220 <https://github.com/autowarefoundation/autoware_universe/issues/9220>`_)
  fix: bugprone-incorrect-roundings
* fix(autoware_detected_object_validation): fix clang-diagnostic-error (`#9215 <https://github.com/autowarefoundation/autoware_universe/issues/9215>`_)
  fix: clang-c-error
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Sebastian Zęderowski, Yutaka Kondo, kobayu858

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix(autoware_detected_object_validation): fix bugprone-incorrect-roundings (`#9220 <https://github.com/autowarefoundation/autoware_universe/issues/9220>`_)
  fix: bugprone-incorrect-roundings
* fix(autoware_detected_object_validation): fix clang-diagnostic-error (`#9215 <https://github.com/autowarefoundation/autoware_universe/issues/9215>`_)
  fix: clang-c-error
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Yutaka Kondo, kobayu858

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(object_recognition_utils): add autoware prefix to object_recognition_utils (`#8946 <https://github.com/autowarefoundation/autoware_universe/issues/8946>`_)
* refactor(detected_object_validation): rework parameters (`#7750 <https://github.com/autowarefoundation/autoware_universe/issues/7750>`_)
  * refactor(detected_object_validation): rework parameters
  * style(pre-commit): autofix
  * Update perception/detected_object_validation/schema/object_lanelet_filter.schema.json
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * Update perception/detected_object_validation/schema/object_position_filter.schema.json
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * refactor(detected_object_validation): rework parameters
  * refactor(detected_object_validation): rework parameters
  * refactor(detected_object_validation): rework parameters
  * style(pre-commit): autofix
  * refactor(detected_object_validation): rework parameters
  * refactor(detected_object_validation): rework parameters
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_detected_object_validation): fix functionStatic (`#8482 <https://github.com/autowarefoundation/autoware_universe/issues/8482>`_)
  fix:functionStatic
* fix(autoware_detected_object_validation): fix cppcheck warnings of functionStatic (`#8256 <https://github.com/autowarefoundation/autoware_universe/issues/8256>`_)
  fix: deal with functionStatic warnings
* fix(autoware_detected_object_validation): fix functionConst (`#8285 <https://github.com/autowarefoundation/autoware_universe/issues/8285>`_)
  fix: functionConst
* perf(autoware_detected_object_validation): reduce lanelet_filter processing time  (`#8240 <https://github.com/autowarefoundation/autoware_universe/issues/8240>`_)
  * add local r-tree for fast searching
  change to _func\_\_
  add more debug
  use local rtree
  fix
  tmp update
  fix bug
  clean unused
  clean up
  * clean up
  * style(pre-commit): autofix
  * chore: Optimize object filtering and improve performance
  The code changes in `lanelet_filter.cpp` optimize the object filtering process by using the `empty()` function instead of checking the size of the `transformed_objects.objects` vector. This change improves performance and simplifies the code logic.
  Refactor the code to use `empty()` instead of `size()` for checking if the `transformed_objects.objects` vector is empty. This change improves readability and performance.
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* refactor(probabilistic_occupancy_grid_map, occupancy_grid_map_outlier_filter): add autoware\_ prefix to package name (`#8183 <https://github.com/autowarefoundation/autoware_universe/issues/8183>`_)
  * chore: fix package name probabilistic occupancy grid map
  * fix: solve launch error
  * chore: update occupancy_grid_map_outlier_filter
  * style(pre-commit): autofix
  * refactor: update package name to autoware_probabilistic_occupancy_grid_map on a test
  * refactor: rename folder of occupancy_grid_map_outlier_filter
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* refactor(detected_object_validation)!: add package name prefix of autoware\_ (`#8122 <https://github.com/autowarefoundation/autoware_universe/issues/8122>`_)
  refactor: rename detected_object_validation to autoware_detected_object_validation
* Contributors: Batuhan Beytekin, Esteve Fernandez, Hayate TOBA, Masaki Baba, Taekjin LEE, Yoshi Ri, Yutaka Kondo, kobayu858, taisa1

0.26.0 (2024-04-03)
-------------------
