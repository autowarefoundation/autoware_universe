^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_compare_map_segmentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(autoware_pointcloud_preprocessor): add diagnostic message (`#10579 <https://github.com/autowarefoundation/autoware_universe/issues/10579>`_)
  * feat: add diag msg
  * chore: fix code
  * chore: remove outlier count in ring
  * chore: move format timestamp to utility
  * chore: add paramter to schema
  * chore: add parameter for cluster
  * chore: clean code
  * chore: fix schema
  * chore: move diagnostic updater to filter base class
  * chore: fix schema
  * chore: fix spell error
  * chore: set up diagnostic updater
  * refactor: utilize autoware_utils diagnostic message
  * chore: add publish
  * chore: add detail message
  * chore: const for time difference
  * refactor: structure diagnostics to class
  * chore: const reference
  * chore: clean logic
  * chore: modify function name
  * chore: update parameter
  * chore: move evaluate status into diagnostic
  * chore: fix description for concatenated pointcloud
  * chore: timestamp mismatch threshold
  * chore: fix diagnostic key
  * chore: change function naming
  ---------
* Contributors: TaikiYamada4, Yi-Hsiang Fang (Vivid)

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
* chore(elevation_map): add @asa-naki to maitainer (`#10621 <https://github.com/autowarefoundation/autoware_universe/issues/10621>`_)
* fix(autoware_compare_map_segmentation): handle empty output point cloud  (`#10572 <https://github.com/autowarefoundation/autoware_universe/issues/10572>`_)
  * fix(voxel_based_compare_map_filter): handle empty output point cloud and set appropriate frame_id
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Kosuke Takeuchi, Taekjin LEE, TaikiYamada4

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(compare_map_segmentation): publish warning when pcl::voxelgrid failed to filter with large leaf size (`#10381 <https://github.com/autowarefoundation/autoware_universe/issues/10381>`_)
  * feat: add warn in pcl overflow with compare_map
  chore: fix message length
  feat: add sanity check into voxel based compare map
  feat: add feasibility check function
  * fix: update include paths for voxel_grid_map_loader in segmentation filters
  * feat: add diagnostic status handling in VoxelGridMapLoader and related components
  * feat: add diagnostic updater for voxel-based compare map filter and improve status reporting
  * refactor: move isFeasibleWithPCLVoxelGrid function to VoxelGridMapLoader and streamline diagnostics handling
  * feat: update diagnostic status handling in VoxelGridMapLoader and related components
  * feat: enhance diagnostic status handling in voxel-based compare map filters
  * style(pre-commit): autofix
  * refactor: add comments for clarity on voxel number calculations and overflow checks
  * Update perception/autoware_compare_map_segmentation/lib/voxel_grid_map_loader.cpp
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: yoshiri <yoshiyoshidetteiu@gmail.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* fix(autoware_compare_map_segmentation): missing includes (`#10413 <https://github.com/autowarefoundation/autoware_universe/issues/10413>`_)
  fix: missing include
* fix(compare_map_segmentation): last map update logic (`#10351 <https://github.com/autowarefoundation/autoware_universe/issues/10351>`_)
  fix(compare_map): logic
* fix(voxel_based_compare_map): temporary fix pointcloud transform lookup  (`#10299 <https://github.com/autowarefoundation/autoware_universe/issues/10299>`_)
  * fix(voxel_based_compare_map): temporary fix pointcloud transform lookup_time
  * pre-commit
  * chore: reduce timeout
  * fix: misalignment when tranform back output
  * fix: typo
  ---------
* Contributors: Kenzo Lobos Tsunekawa, Ryohsuke Mitsudome, Shumpei Wakabayashi, Taekjin LEE, badai nguyen

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* fix(compare_map_filter): deadlock bug fix (`#10222 <https://github.com/autowarefoundation/autoware_universe/issues/10222>`_)
  * fix(compare_map_filter): deadlock bug fix
  * fix: change to lock_guard
  * fix: CI error
  * reduce scope of mutex
  * refactor
  * chore: refactor
  * fix: add missing mutex for map_grid_size_x
  ---------
* Contributors: Hayato Mizushima, Yutaka Kondo, badai nguyen

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
* feat(autoware_compare_map_segmentation): tier4_debug_msgs changed to autoware_internal_debug_msgs in fil… (`#9869 <https://github.com/autowarefoundation/autoware_universe/issues/9869>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files perception/autoware_compare_map_segmentation
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Vishal Chauhan

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
* chore(compare_map_segmentation): rename defined type (`#9181 <https://github.com/autowarefoundation/autoware_universe/issues/9181>`_)
* fix(cpplint): include what you use - perception (`#9569 <https://github.com/autowarefoundation/autoware_universe/issues/9569>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(compare_map_segmentation): add maintainer (`#9371 <https://github.com/autowarefoundation/autoware_universe/issues/9371>`_)
* fix(compare_map_segmentation): timer period mismatched with parameter (`#9259 <https://github.com/autowarefoundation/autoware_universe/issues/9259>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_compare_map_segmentation): fix cppcheck constVariableReference (`#9196 <https://github.com/autowarefoundation/autoware_universe/issues/9196>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Ryuta Kambe, Yutaka Kondo, badai nguyen

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
* fix(autoware_compare_map_segmentation): fix cppcheck constVariableReference (`#9196 <https://github.com/autowarefoundation/autoware_universe/issues/9196>`_)
* Contributors: Esteve Fernandez, Ryuta Kambe, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(autoware_point_types): prefix namespace with autoware::point_types (`#9169 <https://github.com/autowarefoundation/autoware_universe/issues/9169>`_)
* refactor(autoware_compare_map_segmentation): resolve clang-tidy error in autoware_compare_map_segmentation (`#9162 <https://github.com/autowarefoundation/autoware_universe/issues/9162>`_)
  * refactor(autoware_compare_map_segmentation): resolve clang-tidy error in autoware_compare_map_segmentation
  * style(pre-commit): autofix
  * include message_filters as SYSTEM
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(compare_map_segmentation): add missing mutex lock (`#9097 <https://github.com/autowarefoundation/autoware_universe/issues/9097>`_)
  * fix(compare_map_segmentation): missing mutux
  * chore: rename mutex\_
  * fix: remove unnecessary mutex
  * fix: typos
  * chore: minimize mutex scope
  * chore: change to lock_guard
  * fix: check tree initialization
  * fix: memory ordering
  * fix: replace all static_map_loader_mutex\_
  ---------
* fix(compare_map_segmentation): throw runtime error when using non-split map pointcloud for DynamicMapLoader (`#9024 <https://github.com/autowarefoundation/autoware_universe/issues/9024>`_)
  * fix(compare_map_segmentation): throw runtime error when using non-split map pointcloud for DynamicMapLoader
  * chore: typo
  * fix: launch
  * Update perception/autoware_compare_map_segmentation/schema/voxel_distance_based_compare_map_filter.schema.json
  Co-authored-by: Yoshi Ri <yoshiyoshidetteiu@gmail.com>
  * fix: change to RCLCPP_ERROR
  ---------
  Co-authored-by: Yoshi Ri <yoshiyoshidetteiu@gmail.com>
* chore(compare_map_segmentation): add node tests (`#8907 <https://github.com/autowarefoundation/autoware_universe/issues/8907>`_)
  * chore(compare_map_segmentation): add test for voxel_based_compare_map_filter
  * feat: add test for other compare map filter
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_compare_map_segmentation): typo bug fix (`#8939 <https://github.com/autowarefoundation/autoware_universe/issues/8939>`_)
  fix(compare_map_filter): typo bug fix
* fix(autoware_compare_map_segmentation): fix unusedFunction (`#8725 <https://github.com/autowarefoundation/autoware_universe/issues/8725>`_)
  fix:unusedFunction
* fix(compare_map_segmentation): use squared distance to compare threshold (`#8744 <https://github.com/autowarefoundation/autoware_universe/issues/8744>`_)
  fix: use square distance to compare threshold
* fix(autoware_compare_map_segmentation): fix unusedFunction (`#8565 <https://github.com/autowarefoundation/autoware_universe/issues/8565>`_)
  fix:unusedFunction
* fix(autoware_compare_map_segmentation): fix cppcheck warnings of functionStatic (`#8263 <https://github.com/autowarefoundation/autoware_universe/issues/8263>`_)
  * fix: deal with functionStatic warnings
  * fix: deal with functionStatic warnings
  * fix: remove unnecessary const
  * fix: build error
  ---------
* fix(autoware_compare_map_segmentation): fix uninitMemberVar (`#8338 <https://github.com/autowarefoundation/autoware_universe/issues/8338>`_)
  fix:uninitMemberVar
* fix(autoware_compare_map_segmentation): fix passedByValue (`#8233 <https://github.com/autowarefoundation/autoware_universe/issues/8233>`_)
  fix:passedByValue
* fix(autoware_compare_map_segmentation): fix redundantInitialization warning (`#8226 <https://github.com/autowarefoundation/autoware_universe/issues/8226>`_)
* revert: revert "refactor(autoware_map_msgs): modify pcd metadata msg (`#7852 <https://github.com/autowarefoundation/autoware_universe/issues/7852>`_)" (`#8180 <https://github.com/autowarefoundation/autoware_universe/issues/8180>`_)
* refactor(autoware_map_msgs): modify pcd metadata msg (`#7852 <https://github.com/autowarefoundation/autoware_universe/issues/7852>`_)
* refactor(compare_map_segmentation): add package name prefix of autoware\_ (`#8005 <https://github.com/autowarefoundation/autoware_universe/issues/8005>`_)
  * refactor(compare_map_segmentation): add package name prefix of autoware\_
  * docs: update Readme
  ---------
* Contributors: Esteve Fernandez, Ryohsuke Mitsudome, Ryuta Kambe, Yamato Ando, Yoshi Ri, Yukinari Hisaki, Yutaka Kondo, badai nguyen, kobayu858, taisa1

0.26.0 (2024-04-03)
-------------------
