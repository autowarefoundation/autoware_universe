^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_euclidean_cluster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.0 (2025-08-11)
-------------------

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
* feat(autoware_euclidean_cluster): enhance VoxelGridBasedEuclideanCluster with Large Cluster Filtering Parameters (`#10618 <https://github.com/autowarefoundation/autoware_universe/issues/10618>`_)
  * Squashed commit of the following:
  commit cf3035909ccad94003b2b06f8608b6cb887b221a
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Tue May 13 11:34:32 2025 +0900
  debugging impl removed
  commit 17ee5fc61053e1ff816294a962d9f61dc73cd164
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Tue May 13 11:24:04 2025 +0900
  parameters reading finished
  commit 6731b5150344515fce11bf5c0128a20145a0b6a8
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Tue May 13 09:50:47 2025 +0900
  euclidean cluster filter
  commit 4a65dafec7728209dc4015c513920215d259ddae
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Fri May 9 15:46:07 2025 +0900
  Squashed commit of the following:
  commit 699e657c3997e0c3457d9c1f5fffe1081c4433cc
  Merge: 4833afd811 e876ece2f8
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Fri May 9 11:14:48 2025 +0900
  Merge branch 'main' into feat/autoware_perception_rviz_plugin/detected_objects_with_feature_display
  commit 4833afd8114364625a4a9a82b237e72a09c737be
  Author: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Date:   Fri May 9 02:13:09 2025 +0000
  style(pre-commit): autofix
  commit d7bf97d85c1c97300adf52b7ace62a7c08b78402
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Fri May 9 10:53:33 2025 +0900
  fix all problems of rviz
  commit 91ec2882a505df6996d49a2395f977eae1841314
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Thu May 8 19:22:56 2025 +0900
  format fix
  commit fa1e680ab138253831398c51c415dd3861ea298b
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Thu May 8 17:56:16 2025 +0900
  helper to better structure
  commit 2e4ba008e8c3351fc12f33b79f2fe41c492b1f3c
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Thu May 8 16:26:16 2025 +0900
  colorbar visualization optimized
  commit 25e4b9f4131cf38ce89c9b8b28dee0ed6562a4c3
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Thu May 8 15:42:48 2025 +0900
  basic functions all implemented
  commit 3e3db86a1f3ff266cb00b8b83fa57231ee8e2fb8
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Thu May 8 10:31:02 2025 +0900
  colorbar
  commit a6be3ce4a2a3fc48b54ba798af7875d6b071d88b
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Wed May 7 18:05:45 2025 +0900
  colormap fully implemented
  commit 46762b344541580d3411f61ea78828e5f35d9cfb
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Wed May 7 17:49:07 2025 +0900
  colormap implemented
  commit e3024f1d2865ca76c0b8e338fa5c2d6bd282dd22
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Thu Apr 17 10:30:24 2025 +0900
  feat(euclidean_cluster): add markers for clusters
  remove filter
  profiling
  rviz detected_objects_with_feature
  detected objects
  stage all commits
  Revert non-visualization changes to state of 43480ef7
  commit daef21efb35bc0c4dc2fe9009906199d2b3cf9b1
  Author: lei.gu <lei.gu@tier4.jp>
  Date:   Fri May 9 15:36:16 2025 +0900
  colorbar
  * style(pre-commit): autofix
  * corresponding part of VoxelGridBasedEuclideanCluster used in detection by tracker
  * style(pre-commit): autofix
  * cluster point number diag removed
  * add max_num_points_per_cluster
  * euclidean cluster diag impl restored
  * add comments for the parameters
  * readme updated
  * diag removed
  * random point and exclude extreme large cluster
  * fix index problem
  * Refactor voxel grid parameters to improve clarity and functionality
  - Renamed `max_num_points_per_cluster` to `max_voxel_cluster_for_output` for better understanding of its purpose.
  - Updated related code and configuration files to reflect this change.
  - Adjusted logic in the clustering algorithm to utilize the new parameter name.
  This change enhances the readability of the code and aligns parameter naming with their intended use.
  * Add uniform point cloud generation function for voxel clustering tests
  - Introduced `generateClusterWithinVoxelUniform` to create a uniform point cloud for testing.
  - Updated test case to utilize the new function, adjusting the number of generated points.
  - Modified `max_voxel_cluster_for_output` to reflect the new clustering logic.
  This enhances the testing framework by providing a more controlled point cloud generation method, improving test reliability.
  * Update voxel grid parameters for euclidean clustering
  - Reduced `min_voxel_cluster_size_for_filtering` from 150 to 65 to better accommodate medium-sized trucks, considering LiDAR occlusion.
  - Added comments to clarify the rationale behind the new threshold.
  This change aims to improve the filtering process in the voxel grid-based euclidean clustering algorithm.
  * max_num_points_per_cluster removed
  This change aims to enhance code readability and maintainability in the voxel grid-based euclidean clustering implementation.
  * Update README.md to clarify clustering parameters
  Revised descriptions for `min_cluster_size` and `max_cluster_size` to specify that they refer to the number of voxels instead of points. This change enhances the clarity of the documentation for the euclidean clustering methods.
  * Update README.md to correct clustering parameter descriptions
  Modified the descriptions for `min_cluster_size` and `max_cluster_size` in the README to clarify that they refer to the number of points instead of voxels. This change improves the accuracy of the documentation for the euclidean clustering methods.
  * Add max_voxel_cluster_for_output parameter to README.md
  Introduced a new parameter `max_voxel_cluster_for_output` to the documentation, specifying the maximum number of voxel clusters to output. This addition enhances the clarity of the clustering configuration options available in the euclidean clustering methods.
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* Contributors: TaikiYamada4, Yi-Hsiang Fang (Vivid), lei.gu

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
* feat(euclidean_cluster): add diagnostics warning when cluster skipped (`#10278 <https://github.com/autowarefoundation/autoware_universe/issues/10278>`_)
  * feat(euclidean_cluster): add diagnostics warning when cluster skipped due to excessive points from large objects
  * remove temporary code
  * style(pre-commit): autofix
  * feat(euclidean_cluster): diagnostics modified to remove redundant info
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* Contributors: Ryohsuke Mitsudome, lei.gu

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
* feat(autoware_euclidean_cluster)!: tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_euclidean_cluster (`#9873 <https://github.com/autowarefoundation/autoware_universe/issues/9873>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files perception/autoware_euclidean_cluster
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
* fix(cpplint): include what you use - perception (`#9569 <https://github.com/autowarefoundation/autoware_universe/issues/9569>`_)
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
* fix(autoware_euclidean_cluster): fix bugprone-misplaced-widening-cast (`#9227 <https://github.com/autowarefoundation/autoware_universe/issues/9227>`_)
  fix: bugprone-misplaced-widening-cast
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo, kobayu858

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
* fix(autoware_euclidean_cluster): fix bugprone-misplaced-widening-cast (`#9227 <https://github.com/autowarefoundation/autoware_universe/issues/9227>`_)
  fix: bugprone-misplaced-widening-cast
* Contributors: Esteve Fernandez, Yutaka Kondo, kobayu858

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(autoware_point_types): prefix namespace with autoware::point_types (`#9169 <https://github.com/autowarefoundation/autoware_universe/issues/9169>`_)
* refactor(autoware_pointcloud_preprocessor): rework crop box parameters (`#8466 <https://github.com/autowarefoundation/autoware_universe/issues/8466>`_)
  * feat: add parameter schema for crop box
  * chore: fix readme
  * chore: remove filter.param.yaml file
  * chore: add negative parameter for voxel grid based euclidean cluster
  * chore: fix schema description
  * chore: fix description of negative param
  ---------
* refactor(pointcloud_preprocessor): prefix package and namespace with autoware (`#7983 <https://github.com/autowarefoundation/autoware_universe/issues/7983>`_)
  * refactor(pointcloud_preprocessor)!: prefix package and namespace with autoware
  * style(pre-commit): autofix
  * style(pointcloud_preprocessor): suppress line length check for macros
  * fix(pointcloud_preprocessor): missing prefix
  * fix(pointcloud_preprocessor): missing prefix
  * fix(pointcloud_preprocessor): missing prefix
  * fix(pointcloud_preprocessor): missing prefix
  * fix(pointcloud_preprocessor): missing prefix
  * refactor(pointcloud_preprocessor): directory structure (soft)
  * refactor(pointcloud_preprocessor): directory structure (hard)
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* refactor(euclidean_cluster): add package name prefix of autoware\_ (`#8003 <https://github.com/autowarefoundation/autoware_universe/issues/8003>`_)
  * refactor(euclidean_cluster): add package name prefix of autoware\_
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Amadeusz Szymko, Esteve Fernandez, Yi-Hsiang Fang (Vivid), Yutaka Kondo, badai nguyen

0.26.0 (2024-04-03)
-------------------
