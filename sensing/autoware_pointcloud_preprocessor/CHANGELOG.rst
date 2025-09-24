^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_pointcloud_preprocessor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.0 (2025-08-11)
-------------------
* refactor(pointcloud_preprocessor): extract downsample logic from pickup_based_voxel_downsample_filter (`#11098 <https://github.com/autowarefoundation/autoware_universe/issues/11098>`_)
  * feat(pointcloud_preprocessor): add voxel size struct and downsampling function to pickup based filter
  * refactor(pointcloud_preprocessor): use point_cloud2_iterator to handle pointcloud
  * refactor(pointcloud_preprocessor): pass VoxelSize by const reference to improve performance
  * feat(pointcloud_preprocessor): enhance voxel grid downsampling tests with additional scenarios
  * feat(pointcloud_preprocessor): refactor downsampling logic to extract unique voxel point indices and copy filtered points
  * fix(pointcloud_preprocessor): optimize voxel point index extraction and memory copying in downsampling
  * refactor(pointcloud_preprocessor): rename voxel_map to index_map for clarity in downsampling functions
  * refactor(pointcloud_preprocessor): remove unused includes
  * chore(pointcloud_preprocessor): apply clang-format and cpplint
  * chore(pointcloud_preprocessor): fix linter error
  * style(pre-commit): autofix
  * style(poincloud_preprocessor): adjust clang-format directives for consistency
  * fix(pointcloud_preprocessor): correct function name from copy_filtered_point to copy_filtered_points
  * fix(pointcloud_preprocessor): update parameter type from ConstSharedPtr to reference
  * refactor(pointcloud_preprocessor): consolidate voxel size parameters into a single struct
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_pointcloud_preprocessor): add publisher for concatenated pointcloud meta info (`#10851 <https://github.com/autowarefoundation/autoware_universe/issues/10851>`_)
  * feat(autoware_pointcloud_preprocessor): add publisher for concatenated pointcloud meta info
  * style(pre-commit): autofix
  * feat(autoware_cuda_pointcloud_preprocessor): handle concatenated pointcloud meta info
  * feat(autoware_pointcloud_preprocessor): serialized config of matching strategy
  * feat(autoware_pointcloud_preprocessor): update msg
  * feat(autoware_pointcloud_preprocessor): update msg (2)
  * docs(autoware_pointcloud_preprocessor): add cloud info topic description
  * feat(autoware_pointcloud_preprocessor): add unit tests for cloud info
  * fix(autoware_pointcloud_preprocessor): pre-commit
  * fix(autoware_pointcloud_preprocessor): remove *_struct headers inclusion
  * fix(autoware_pointcloud_preprocessor): check if the matching strategy cannot be enumerated
  * test(autoware_pointcloud_preprocessor): full cloud repr
  * feat(autoware_pointcloud_preprocessor): auto success set & more unit tests
  * feat(autoware_pointcloud_preprocessor): publish info regardless cloud content
  * style(autoware_pointcloud_preprocessor): typo
  * feat(autoware_pointcloud_preprocessor): make update_concatenated_point_cloud_config static for easier integration
  * docs(autoware_pointcloud_preprocessor): typo
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * fix(autoware_pointcloud_preprocessor): publish cloud info out of condition block
  * fix(autoware_pointcloud_preprocessor): container access with safe bound checking
  * style(autoware_pointcloud_preprocessor): unify naming convention (part 1 - content)
  * style(autoware_pointcloud_preprocessor): unify naming convention (part 2 - files name)
  * style(autoware_pointcloud_preprocessor): naming convention for main API
  * doc(autoware_pointcloud_preprocessor): add docstring
  * feat(autoware_pointcloud_preprocessor): add remap to launch files
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
* fix(pointcloud_preprocessor): handle empty pointclouds in pickup_based_downsample_filter (`#11003 <https://github.com/autowarefoundation/autoware_universe/issues/11003>`_)
  * feat(pointcloud_preprocessor): add integration test  for pickup based downsamplie filter node
  * feat(pointcloud_preprocessor): add test for pickup based downsample filter with zero length pointcloud
  that test will fail for now.
  * refactor(pointcloud_preprocessor): simplify test for pickup based downsample filter
  * fix(pointcloud_preprocessor): enable to output zero length pointcloud
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* docs(autoware_pointcloud_preprocessor): point cloud concatenation strategies (`#10994 <https://github.com/autowarefoundation/autoware_universe/issues/10994>`_)
  * docs(autoware_pointcloud_preprocessor): point cloud concatenation strategies
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Amadeusz Szymko, Mete Fatih Cırıt, Takahisa Ishikawa

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* refactor(blockage_diag): split up `filter` function, add doc comments (`#10708 <https://github.com/autowarefoundation/autoware_universe/issues/10708>`_)
  * chore(blockage_diag): explain parameters in the code
  * chore(blockage_diag): make method naming and ovrerrides conformant
  * refactor(blockage_diag): size/index calculation functions
  * refactor(blockage_diag): split up filter function
  * style(pre-commit): autofix
  * chore: make cppcheck happy
  * chore: downscale by 256 to fit uint8_max exactly
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_cuda_pointcloud_preprocessor): diagnostic for cuda pointcloud preprocessor (`#10793 <https://github.com/autowarefoundation/autoware_universe/issues/10793>`_)
  * feat: add ring, crop box diag
  * feat: add distortion correction
  * feat: add concat diagnostics
  * chore: remove if debug publisher
  * chore: clean code
  * chore: clean code
  * chore: utilize mask instead of atomicadd
  * chore: count nan points and numbers of point after crop box filter
  * chore: fix schema
  * chore: move to structure
  * chore: update library
  * chore: prefix output
  * chore: chagne shared pointer to const reference
  * chore: use device vector for thrust count
  * chore: fix output pointcloud name
  * chore: add comment
  * chore: reuse function
  * chore: fix merging issue
  * chore: prefix output
  * chore: fix layout
  * chore: add doc comment
  * chore(autoware_cuda_pointcloud_preprocessor): disable uncrustify
  ---------
  Co-authored-by: Max SCHMELLER <max.schmeller@tier4.jp>
* fix(autoware_pointcloud_preprocessor): unify diagnostic interface namespace (`#10807 <https://github.com/autowarefoundation/autoware_universe/issues/10807>`_)
  fix: unify diagnostic interface namespace
* fix: -Werror=maybe-uninitialized (`#10791 <https://github.com/autowarefoundation/autoware_universe/issues/10791>`_)
  fix: -Werror=maybe-uninitialized
  `#10591 <https://github.com/autowarefoundation/autoware_universe/issues/10591>`_
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
* Contributors: Max Schmeller, TaikiYamada4, Tim Clephas, Yi-Hsiang Fang (Vivid)

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* fix(autoware_pointcloud_preprocessor): combine_cloud_handler always set XYZIRC (`#10617 <https://github.com/autowarefoundation/autoware_universe/issues/10617>`_)
  always set XYZIRC
* feat(ring_outlier_filter): update filtering parameter and process (`#10537 <https://github.com/autowarefoundation/autoware_universe/issues/10537>`_)
* feat(autoware_pointcloud_preprocessor): templated version of the pointcloud concatenation (`#10298 <https://github.com/autowarefoundation/autoware_universe/issues/10298>`_)
  * feat: refactored the concat into a templated design to allow cuda implementations and extend it to radars
  * fix: moved the concat cpp for consistency and component loading
  * chore: removed unused dep
  * fix: missing virtual destructor
  * fix: fixed missing dep
  * chore: removed unused var
  * chore: refactored the cloud handler
  * chore: updated documentation
  * fix: fixed rebase error
  * chore: removed commented include
  * chore: removed another rebase error induced print
  * fix: and yet another rebase induced error
  * chore: changed method name
  * chore: removing key from dict for peace of mind
  * chore: reimplemented latest changes in the base branch
  * chore: missed dep
  * chore: spell
  * chore: removed explicit template instantiation since clang tidy reported it was being done implicitly and thus redundant
  * chore: added documentation regarding why allocation is done right after publishing
  * chore: replaced at for extract+mapped
  * chore: moved format_timestamp into its own file
  ---------
* Contributors: Kento Yabuuchi, Kenzo Lobos Tsunekawa, Kotaro Uetake, TaikiYamada4

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* perf(autoware_pointcloud_preprocessor): introduce managed transform buffer with implicitly defined listener type (`#9197 <https://github.com/autowarefoundation/autoware_universe/issues/9197>`_)
  * feat(autoware_universe_utils): rework managed transform buffer
  * feat(autoware_pointcloud_preprocessor): integrate Managed TF Buffer into pointcloud densifier
  * chore: update repos
  * chore(managed_transform_buffer): fix version
  ---------
  Co-authored-by: Kenzo Lobos-Tsunekawa <kenzo.lobos@tier4.jp>
* fix(pointcloud_preprocessor): added missing includes (`#10412 <https://github.com/autowarefoundation/autoware_universe/issues/10412>`_)
  fix: added missing includes
* fix: missing dependency on tf2_sensor_msgs (`#10400 <https://github.com/autowarefoundation/autoware_universe/issues/10400>`_)
* feat(autoware_pointcloud_preprocessor): add pointcloud_densifier package (`#10226 <https://github.com/autowarefoundation/autoware_universe/issues/10226>`_)
  * feat(autoware_pointcloud_preprocessor): add pointcloud_densifier package
  * style(pre-commit): autofix
  * fix(autoware_pointcloud_preprocessor): add header
  * fix(autoware_pointcloud_preprocessor): add schema and fix debugger
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(voxel_based_compare_map): temporary fix pointcloud transform lookup  (`#10299 <https://github.com/autowarefoundation/autoware_universe/issues/10299>`_)
  * fix(voxel_based_compare_map): temporary fix pointcloud transform lookup_time
  * pre-commit
  * chore: reduce timeout
  * fix: misalignment when tranform back output
  * fix: typo
  ---------
* Contributors: Amadeusz Szymko, Kaan Çolak, Kenzo Lobos Tsunekawa, Ryohsuke Mitsudome, Tim Clephas, badai nguyen

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* feat(autoware_pointcloud_preprocessor): add missing vehicle msg depency (`#10313 <https://github.com/autowarefoundation/autoware_universe/issues/10313>`_)
  feat(auotawre_pointcloud_preprocessor): add missing vehicle msg depency
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* chore(autoware_pointcloud_preprocessor): fix variable naming in distortion corrector (`#10185 <https://github.com/autowarefoundation/autoware_universe/issues/10185>`_)
  chore: fix naming
* feat(autoware_image_based_projection_fusion): redesign image based projection fusion node (`#10016 <https://github.com/autowarefoundation/autoware_universe/issues/10016>`_)
* Contributors: Hayato Mizushima, Maxime CLEMENT, Yi-Hsiang Fang (Vivid), Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_pointcloud_preprocessor): fix potential double unlock in concatenate node (`#10082 <https://github.com/autowarefoundation/autoware_universe/issues/10082>`_)
  * feat: reuse collectors
  * fix: potential double unlock
  * style(pre-commit): autofix
  * chore: remove mutex
  * chore: reset the processing cloud only if needed
  * chore: fix grammar
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: SakodaShintaro <shintaro.sakoda@tier4.jp>
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* chore: refine maintainer list (`#10110 <https://github.com/autowarefoundation/autoware_universe/issues/10110>`_)
  * chore: remove Miura from maintainer
  * chore: add Taekjin-san to perception_utils package maintainer
  ---------
* feat(autoware_pointcloud_preprocessor): reuse collectors to reduce creation of collector and timer (`#10074 <https://github.com/autowarefoundation/autoware_universe/issues/10074>`_)
  * feat: reuse collectors
  * chore: remove for-loop to find_if
  * chore: remove set period
  * chore: remove oldest timestamp
  * chore: fix managing collector list logic
  * chore: fix logging
  * feat: change to THROTTLE
  * feat: initialize required number of collectors when the node start
  * chore: fix init collector
  * chore: fix grammar
  ---------
* fix(autoware_pointcloud_preprocessor): empty input validation (`#10115 <https://github.com/autowarefoundation/autoware_universe/issues/10115>`_)
  * fix(autoware_pointcloud_preprocessor): fix 0 division
  * style(pre-commit): autofix
  * fix float and error throttle
  * style(pre-commit): autofix
  * fix
  * fix param validation
  * fix unused var
  * feat add input validatoin
  * fix too cautious floating
  * fix error msg
  * fix
  plural
  * fix: set exclusiveMinimum 0.0
  * fix: reomve unnecessary validatoin
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(distortion_corrector_node): replace imu and twist callback with polling subscriber (`#10057 <https://github.com/autowarefoundation/autoware_universe/issues/10057>`_)
  * fix(distortion_corrector_node): replace imu and twist callback with polling subscriber
  Changed to read data in bulk using take to reduce subscription callback overhead.
  Especially effective when the frequency of imu or twist is high, such as 100Hz.
  * fix(distortion_corrector_node): include vector header for cpplint check
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: Yi-Hsiang Fang (Vivid) <146902905+vividf@users.noreply.github.com>
* chore(pointcloud_preprocessor): add Max to codeowners (`#10083 <https://github.com/autowarefoundation/autoware_universe/issues/10083>`_)
  chore(pointcloud_preprocessor): add Max to maintainers
* Contributors: Fumiya Watanabe, Max Schmeller, Shumpei Wakabayashi, Shunsuke Miura, Takahisa Ishikawa, Yi-Hsiang Fang (Vivid), 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_pointcloud_preprocessor): redesign concatenate and time sync node (`#8300 <https://github.com/autowarefoundation/autoware_universe/issues/8300>`_)
  * chore: rebase main
  * chore: solve conflicts
  * chore: fix cpp check
  * chore: add diagnostics readme
  * chore: update figure
  * chore: upload jitter.png and add old design link
  * chore: add the link to the tool for analyzing timestamp
  * fix: fix bug that timer didn't cancel
  * chore: fix logic for logging
  * Update sensing/autoware_pointcloud_preprocessor/docs/concatenate-data.md
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_pointcloud_preprocessor/src/concatenate_data/combine_cloud_handler.cpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_pointcloud_preprocessor/schema/cocatenate_and_time_sync_node.schema.json
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_pointcloud_preprocessor/schema/cocatenate_and_time_sync_node.schema.json
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_pointcloud_preprocessor/src/concatenate_data/combine_cloud_handler.cpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_pointcloud_preprocessor/src/concatenate_data/combine_cloud_handler.cpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * chore: remove distortion corrector related changes
  * feat: add managed tf buffer
  * chore: fix filename
  * chore: add explanataion for maximum queue size
  * chore: add explanation for timeout_sec
  * chore: fix schema's explanation
  * chore: fix description for twist and odom
  * chore: remove license that are not used
  * chore: change guard to prama once
  * chore: default value change to string
  * Update sensing/autoware_pointcloud_preprocessor/test/test_concatenate_node_unit.cpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_pointcloud_preprocessor/test/test_concatenate_node_unit.cpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_pointcloud_preprocessor/test/test_concatenate_node_unit.cpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_pointcloud_preprocessor/test/test_concatenate_node_unit.cpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * style(pre-commit): autofix
  * chore: clang-tidy style for static constexpr
  * chore: remove unused vector header
  * chore: fix naming concatenated_cloud_publisher
  * chore: fix namimg diagnostic_updater\_
  * chore: specify parameter in comment
  * chore: change RCLCPP_WARN to RCLCPP_WARN_STREAM_THROTTLE
  * chore: add comment for cancelling timer
  * chore: Simplify loop structure for topic-to-cloud mapping
  * chore: fix spell errors
  * chore: fix more spell error
  * chore: rename mutex and lock
  * chore: const reference for string parameter
  * chore: add explaination for RclcppTimeHash\_
  * chore: change the concatenate node to parent node
  * chore: clean processOdometry and processTwist
  * chore: change twist shared pointer queue to twist queue
  * chore: refactor compensate pointcloud to function
  * chore: reallocate memory for concatenate_cloud_ptr
  * chore: remove new to make shared
  * chore: dis to distance
  * chore: refacotr poitncloud_sub
  * chore: return early return but throw runtime error
  * chore: replace #define DEFAULT_SYNC_TOPIC_POSTFIX with member variable
  * chore: fix spell error
  * chore: remove redundant function call
  * chore: replace conplex tuple to structure
  * chore: use reference instead of a pointer to conveys node
  * chore: fix camel to snake case
  * chore: fix logic of publish synchronized pointcloud
  * chore: fix cpp check
  * chore: remove logging and throw error directly
  * chore: fix clangd warnings
  * chore: fix json schema
  * chore: fix clangd warning
  * chore: remove unused variable
  * chore: fix launcher
  * chore: fix clangd warning
  * chore: ensure thread safety
  * style(pre-commit): autofix
  * chore: clean code
  * chore: add parameters for handling rosbag replay in loops
  * chore: fix diagonistic
  * chore: reduce copy operation
  * chore: reserve space for concatenated pointcloud
  * chore: fix clangd error
  * chore: fix pipeline latency
  * chore: add debug mode
  * chore: refactor convert_to_xyzirc_cloud function
  * chore: fix json schema
  * chore: fix logging output
  * chore: fix the output order of the debug mode
  * chore: fix pipeline latency output
  * chore: clean code
  * chore: set some parameters to false in testing
  * chore: fix default value for schema
  * chore: fix diagnostic msgs
  * chore: fix parameter for sample ros bag
  * chore: update readme
  * chore: fix empty pointcloud
  * chore: remove duplicated logic
  * chore: fix logic for handling empty pointcloud
  * chore: clean code
  * chore: remove rosbag_replay parameter
  * chore: remove nodelet cpp
  * chore: clang tidy warning
  * feat: add naive approach for unsynchronized pointclouds
  * chore: add more explanations in docs for naive approach
  * feat: refactor naive method and fix the multithreading issue
  * chore: set parameter to naive
  * chore: fix parameter
  * chore: fix readme
  * Update sensing/autoware_pointcloud_preprocessor/docs/concatenate-data.md
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_pointcloud_preprocessor/docs/concatenate-data.md
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * style(pre-commit): autofix
  * feat: remove mutually exclusive approaches
  * chore: fix spell error
  * chore: remove unused variable
  * refactor: refactor collectorInfo to polymorphic
  * chore: fix variable name
  * chore: fix figure and diagnostic msg in readme
  * chroe: refactor collectorinfo structure
  * chore: revert wrong file changes
  * chore: improve message
  * chore: remove unused input topics
  * chore: change to explicit check
  * chore: tier4 debug msgs to autoware internal debug msgs
  * chore: update documentation
  ---------
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_pointcloud_preprocessor): tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_pointcloud_preprocessor (`#9920 <https://github.com/autowarefoundation/autoware_universe/issues/9920>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files sensing/autoware_pointcloud_preprocessor
* fix(autoware_pointcloud_preprocessor): fix autoware pointcloud preprocessor docs (`#9765 <https://github.com/autowarefoundation/autoware_universe/issues/9765>`_)
  * fix downsample and passthrough
  * fix: fix blockage-diag docs that page is not shown
  ---------
* fix(autoware_pointcloud_preprocessor): fix image display in distortion corrector (`#9761 <https://github.com/autowarefoundation/autoware_universe/issues/9761>`_)
  fix: fix image display
* fix(autoware_pointcloud_preprocessor): remove unused function mask() (`#9751 <https://github.com/autowarefoundation/autoware_universe/issues/9751>`_)
* fix: enable to copy all information in pickup based pointcloud downsampler (`#9686 <https://github.com/autowarefoundation/autoware_universe/issues/9686>`_)
  enable to copy all information in downsampler
* Contributors: Fumiya Watanabe, Ryuta Kambe, Vishal Chauhan, Yi-Hsiang Fang (Vivid), Yoshi Ri

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
* fix(autoware_pointcloud_preprocessor): remove unused arg and unavailable param file. (`#9525 <https://github.com/autowarefoundation/autoware_universe/issues/9525>`_)
  Remove unused arg and unavailable param file.
* fix(autoware_pointcloud_preprocessor): fix clang-diagnostic-inconsistent-missing-override (`#9445 <https://github.com/autowarefoundation/autoware_universe/issues/9445>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore: update license of pointcloud preprocessor (`#9397 <https://github.com/autowarefoundation/autoware_universe/issues/9397>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_pointcloud_preprocessor): clang-tidy error in distortion corrector (`#9412 <https://github.com/autowarefoundation/autoware_universe/issues/9412>`_)
  fix: clang-tidy
* fix(autoware_pointcloud_preprocessor): clang-tidy for overrides (`#9414 <https://github.com/autowarefoundation/autoware_universe/issues/9414>`_)
  fix: clang-tidy for overrides
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_pointcloud_preprocessor): fix the wrong naming of crop box parameter file  (`#9258 <https://github.com/autowarefoundation/autoware_universe/issues/9258>`_)
  fix: fix the wrong file name
* fix(autoware_pointcloud_preprocessor): launch file load parameter from yaml (`#8129 <https://github.com/autowarefoundation/autoware_universe/issues/8129>`_)
  * feat: fix launch file
  * chore: fix spell error
  * chore: fix parameters file name
  * chore: remove filter base
  ---------
* Contributors: Daisuke Nishimatsu, Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Mukunda Bharatheesha, Ryohsuke Mitsudome, Ryuta Kambe, Yi-Hsiang Fang (Vivid), Yutaka Kondo

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
* fix(autoware_pointcloud_preprocessor): fix the wrong naming of crop box parameter file  (`#9258 <https://github.com/autowarefoundation/autoware_universe/issues/9258>`_)
  fix: fix the wrong file name
* fix(autoware_pointcloud_preprocessor): launch file load parameter from yaml (`#8129 <https://github.com/autowarefoundation/autoware_universe/issues/8129>`_)
  * feat: fix launch file
  * chore: fix spell error
  * chore: fix parameters file name
  * chore: remove filter base
  ---------
* Contributors: Esteve Fernandez, Yi-Hsiang Fang (Vivid), Yutaka Kondo

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
* feat(autoware_pointcloud_preprocessor): distortion corrector node update azimuth and distance (`#8380 <https://github.com/autowarefoundation/autoware_universe/issues/8380>`_)
  * feat: add option for updating distance and azimuth value
  * chore: clean code
  * chore: remove space
  * chore: add documentation
  * chore: fix docs
  * feat: conversion formula implementation for degree, still need to change to rad
  * chore: fix tests for AzimuthConversionExists function
  * feat: add fastatan to utils
  * feat: remove seperate sin, cos and use sin_and_cos function
  * chore: fix readme
  * chore: fix some grammar errors
  * chore: fix spell error
  * chore: set debug mode to false
  * chore: set update_azimuth_and_distance default value to false
  * chore: update readme
  * chore: remove cout
  * chore: add opencv license
  * chore: fix grammar error
  * style(pre-commit): autofix
  * chore: add runtime error when azimuth conversion failed
  * chore: change default pointcloud
  * chore: change function name
  * chore: move variables to structure
  * chore: add random seed
  * chore: rewrite get conversion function
  * chore: fix opencv fast atan2 function
  * chore: fix schema description
  * Update sensing/autoware_pointcloud_preprocessor/test/test_distortion_corrector_node.cpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_pointcloud_preprocessor/test/test_distortion_corrector_node.cpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * chore: move code to function for readability
  * chore: simplify code
  * chore: fix sentence, angle conversion
  * chore: add more invalid condition
  * chore: fix the string name to enum
  * chore: remove runtime error
  * chore: use optional for AngleConversion structure
  * chore: fix bug and clean code
  * chore: refactor the logic of calculating conversion
  * chore: refactor function in unit test
  * chore: RCLCPP_WARN_STREAM logging when failed to get angle conversion
  * chore: improve normalize angle algorithm
  * chore: improve multiple_of_90_degrees logic
  * chore: add opencv license
  * style(pre-commit): autofix
  * chore: clean code
  * chore: fix sentence
  * style(pre-commit): autofix
  * chore: add 0 0 0 points in test case
  * chore: fix spell error
  * Update common/autoware_universe_utils/NOTICE
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_pointcloud_preprocessor/src/distortion_corrector/distortion_corrector_node.cpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_pointcloud_preprocessor/src/distortion_corrector/distortion_corrector.cpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * chore: use constexpr for threshold
  * chore: fix the path of license
  * chore: explanation for failures
  * chore: use throttle
  * chore: fix empty pointcloud function
  * refactor: change camel to snake case
  * Update sensing/autoware_pointcloud_preprocessor/include/autoware/pointcloud_preprocessor/distortion_corrector/distortion_corrector_node.hpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_pointcloud_preprocessor/include/autoware/pointcloud_preprocessor/distortion_corrector/distortion_corrector_node.hpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * style(pre-commit): autofix
  * Update sensing/autoware_pointcloud_preprocessor/test/test_distortion_corrector_node.cpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * refactor: refactor virtual function in base class
  * chore: fix test naming error
  * chore: fix clang error
  * chore: fix error
  * chore: fix clangd
  * chore: add runtime error if the setting is wrong
  * chore: clean code
  * Update sensing/autoware_pointcloud_preprocessor/src/distortion_corrector/distortion_corrector.cpp
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * style(pre-commit): autofix
  * chore: fix unit test for runtime error
  * Update sensing/autoware_pointcloud_preprocessor/docs/distortion-corrector.md
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * chore: fix offset_rad_threshold
  * chore: change pointer to reference
  * chore: snake_case for unit test
  * chore: fix refactor process twist and imu
  * chore: fix abs and return type of matrix to tf2
  * chore: fix grammar error
  * chore: fix readme description
  * chore: remove runtime error
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* refactor(autoware_pointcloud_preprocessor): rework crop box parameters (`#8466 <https://github.com/autowarefoundation/autoware_universe/issues/8466>`_)
  * feat: add parameter schema for crop box
  * chore: fix readme
  * chore: remove filter.param.yaml file
  * chore: add negative parameter for voxel grid based euclidean cluster
  * chore: fix schema description
  * chore: fix description of negative param
  ---------
* refactor(autoware_pointcloud_preprocessor): rework approximate downsample filter parameters (`#8480 <https://github.com/autowarefoundation/autoware_universe/issues/8480>`_)
  * feat: rework approximate downsample parameters
  * chore: add boundary
  * chore: change double to float
  * feat: rework approximate downsample parameters
  * chore: add boundary
  * chore: change double to float
  * chore: fix grammatical error
  * chore: fix variables from double to float in header
  * chore: change minimum to float
  * chore: fix CMakeLists
  ---------
* refactor(autoware_pointcloud_preprocessor): rework dual return outlier filter parameters (`#8475 <https://github.com/autowarefoundation/autoware_universe/issues/8475>`_)
  * feat: rework dual return outlier filter parameters
  * chore: fix readme
  * chore: change launch file name
  * chore: fix type
  * chore: add boundary
  * chore: change boundary
  * chore: fix boundary
  * chore: fix json schema
  * Update sensing/autoware_pointcloud_preprocessor/schema/dual_return_outlier_filter_node.schema.json
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * chore: fix grammar error
  * chore: fix description for weak_first_local_noise_threshold
  * chore: change minimum and maximum to float
  ---------
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* refactor(autoware_pointcloud_preprocessor): rework ring outlier filter parameters (`#8468 <https://github.com/autowarefoundation/autoware_universe/issues/8468>`_)
  * feat: rework ring outlier parameters
  * chore: add explicit cast
  * chore: add boundary
  * chore: remove filter.param
  * chore: set default frame
  * chore: add maximum boundary
  * chore: boundary to float type
  ---------
* refactor(autoware_pointcloud_preprocessor): rework pickup based voxel grid downsample filter parameters (`#8481 <https://github.com/autowarefoundation/autoware_universe/issues/8481>`_)
  * feat: rework pickup based voxel grid downsample filter parameter
  * chore: update date
  * chore: fix spell error
  * chore: add boundary
  * chore: fix grammatical error
  ---------
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* ci(pre-commit): autoupdate (`#7630 <https://github.com/autowarefoundation/autoware_universe/issues/7630>`_)
  * ci(pre-commit): autoupdate
  * style(pre-commit): autofix
  * fix: remove the outer call to dict()
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
* refactor(autoware_pointcloud_preprocessor): rework random downsample filter parameters (`#8485 <https://github.com/autowarefoundation/autoware_universe/issues/8485>`_)
  * feat: rework random downsample filter parameter
  * chore: change name
  * chore: add explicit cast
  ---------
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* refactor(autoware_pointcloud_preprocessor): rework pointcloud accumulator parameters  (`#8487 <https://github.com/autowarefoundation/autoware_universe/issues/8487>`_)
  * feat: rework pointcloud accumulator parameters
  * chore: add explicit cast
  * chore: add boundary
  ---------
* refactor(autoware_pointcloud_preprocessor): rework radius search 2d outlier filter parameters (`#8474 <https://github.com/autowarefoundation/autoware_universe/issues/8474>`_)
  * feat: rework radius search 2d outlier filter parameters
  * chore: fix schema
  * chore: explicit cast
  * chore: add boundary in schema
  ---------
* refactor(autoware_pointcloud_preprocessor): rework ring passthrough filter parameters (`#8472 <https://github.com/autowarefoundation/autoware_universe/issues/8472>`_)
  * feat: rework ring passthrough parameters
  * chore: fix cmake
  * feat: add schema
  * chore: fix readme
  * chore: fix parameter file name
  * chore: add boundary
  * chore: fix default parameter
  * chore: fix default parameter in schema
  ---------
* fix(autoware_pointcloud_preprocessor): static TF listener as Filter option (`#8678 <https://github.com/autowarefoundation/autoware_universe/issues/8678>`_)
* fix(pointcloud_preprocessor): fix typo (`#8762 <https://github.com/autowarefoundation/autoware_universe/issues/8762>`_)
* fix(autoware_pointcloud_preprocessor): instantiate templates so that the symbols exist when linking (`#8743 <https://github.com/autowarefoundation/autoware_universe/issues/8743>`_)
* fix(autoware_pointcloud_preprocessor): fix unusedFunction (`#8673 <https://github.com/autowarefoundation/autoware_universe/issues/8673>`_)
  fix:unusedFunction
* fix(autoware_pointcloud_preprocessor): resolve issue with FLT_MAX not declared on Jazzy (`#8586 <https://github.com/autowarefoundation/autoware_universe/issues/8586>`_)
  fix(pointcloud-preprocessor): FLT_MAX not declared
  Fixes compilation error on Jazzy:
  error: ‘FLT_MAX’ was not declared in this scope
* fix(autoware_pointcloud_preprocessor): blockage diag node add runtime error when the parameter is wrong (`#8564 <https://github.com/autowarefoundation/autoware_universe/issues/8564>`_)
  * fix: add runtime error
  * Update blockage_diag_node.cpp
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * fix: add RCLCPP error logging
  * chore: remove unused variable
  ---------
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* chore(autoware_pointcloud_preprocessor): change unnecessary warning message to debug (`#8525 <https://github.com/autowarefoundation/autoware_universe/issues/8525>`_)
* refactor(autoware_pointcloud_preprocessor): rework voxel grid outlier filter  parameters (`#8476 <https://github.com/autowarefoundation/autoware_universe/issues/8476>`_)
  * feat: rework voxel grid outlier filter parameters
  * chore: add boundary
  ---------
* refactor(autoware_pointcloud_preprocessor): rework lanelet2 map filter parameters (`#8491 <https://github.com/autowarefoundation/autoware_universe/issues/8491>`_)
  * feat: rework lanelet2 map filter parameters
  * chore: remove unrelated files
  * fix: fix node name in launch
  * chore: fix launcher
  * chore: fix spell error
  * chore: add boundary
  ---------
* refactor(autoware_pointcloud_preprocessor): rework vector map inside area filter parameters  (`#8493 <https://github.com/autowarefoundation/autoware_universe/issues/8493>`_)
  * feat: rework vector map inside area filter parameter
  * chore: fix launcher
  * chore: fix launcher input and output
  ---------
* refactor(autoware_pointcloud_preprocessor): rework concatenate_pointcloud and time_synchronizer_node parameters (`#8509 <https://github.com/autowarefoundation/autoware_universe/issues/8509>`_)
  * feat: rewort concatenate pointclouds and time synchronizer parameter
  * chore: fix launch files
  * chore: fix schema
  * chore: fix schema
  * chore: fix integer and number default value in schema
  * chore: add boundary
  ---------
* refactor(autoware_pointcloud_preprocessor): rework voxel grid downsample filter parameters (`#8486 <https://github.com/autowarefoundation/autoware_universe/issues/8486>`_)
  * feat:rework voxel grid downsample parameters
  * chore: add boundary
  ---------
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* refactor(autoware_pointcloud_preprocessor): rework blockage diag parameters  (`#8488 <https://github.com/autowarefoundation/autoware_universe/issues/8488>`_)
  * feat: rework blockage diag parameters
  * chore: fix readme
  * chore: fix schema description
  * chore: add boundary for schema
  ---------
* chore(autoware_pcl_extensions): refactored the pcl_extensions (`#8220 <https://github.com/autowarefoundation/autoware_universe/issues/8220>`_)
  chore: refactored the pcl_extensions according to the new rules
* feat(pointcloud_preprocessor)!: revert "fix: added temporary retrocompatibility to old perception data (`#7929 <https://github.com/autowarefoundation/autoware_universe/issues/7929>`_)" (`#8397 <https://github.com/autowarefoundation/autoware_universe/issues/8397>`_)
  * feat!(pointcloud_preprocessor): Revert "fix: added temporary retrocompatibility to old perception data (`#7929 <https://github.com/autowarefoundation/autoware_universe/issues/7929>`_)"
  This reverts commit 6b9f164b123e2f6a6fedf7330e507d4b68e45a09.
  * feat(pointcloud_preprocessor): minor grammar fix
  Co-authored-by: David Wong <33114676+drwnz@users.noreply.github.com>
  ---------
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  Co-authored-by: David Wong <33114676+drwnz@users.noreply.github.com>
* fix(autoware_pointcloud_preprocessor): fix variableScope (`#8447 <https://github.com/autowarefoundation/autoware_universe/issues/8447>`_)
  * fix:variableScope
  * refactor:use const
  ---------
* fix(autoware_pointcloud_preprocessor): fix unreadVariable (`#8370 <https://github.com/autowarefoundation/autoware_universe/issues/8370>`_)
  fix:unreadVariable
* fix(ring_outlier_filter): remove unnecessary resize to prevent zero points (`#8402 <https://github.com/autowarefoundation/autoware_universe/issues/8402>`_)
  fix: remove unnecessary resize
* fix(autoware_pointcloud_preprocessor): fix cppcheck warnings of functionStatic (`#8163 <https://github.com/autowarefoundation/autoware_universe/issues/8163>`_)
  fix: deal with functionStatic warnings
  Co-authored-by: Yi-Hsiang Fang (Vivid) <146902905+vividf@users.noreply.github.com>
* perf(autoware_pointcloud_preprocessor): lazy & managed TF listeners (`#8174 <https://github.com/autowarefoundation/autoware_universe/issues/8174>`_)
  * perf(autoware_pointcloud_preprocessor): lazy & managed TF listeners
  * fix(autoware_pointcloud_preprocessor): param names & reverse frames transform logic
  * fix(autoware_ground_segmentation): add missing TF listener
  * feat(autoware_ground_segmentation): change to static TF buffer
  * refactor(autoware_pointcloud_preprocessor): move StaticTransformListener to universe utils
  * perf(autoware_universe_utils): skip redundant transform
  * fix(autoware_universe_utils): change checks order
  * doc(autoware_universe_utils): add docstring
  ---------
* fix(autoware_pointcloud_preprocessor): fix functionConst (`#8280 <https://github.com/autowarefoundation/autoware_universe/issues/8280>`_)
  fix:functionConst
* fix(autoware_pointcloud_preprocessor): fix passedByValue (`#8242 <https://github.com/autowarefoundation/autoware_universe/issues/8242>`_)
  fix:passedByValue
* fix(autoware_pointcloud_preprocessor): fix redundantInitialization (`#8229 <https://github.com/autowarefoundation/autoware_universe/issues/8229>`_)
* fix(autoware_pointcloud_preprocessor): revert increase_size() in robin_hood (`#8151 <https://github.com/autowarefoundation/autoware_universe/issues/8151>`_)
* fix(autoware_pointcloud_preprocessor): fix knownConditionTrueFalse warning (`#8139 <https://github.com/autowarefoundation/autoware_universe/issues/8139>`_)
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
* Contributors: Amadeusz Szymko, Esteve Fernandez, Fumiya Watanabe, Kenzo Lobos Tsunekawa, Rein Appeldoorn, Ryuta Kambe, Shintaro Tomie, Yi-Hsiang Fang (Vivid), Yoshi Ri, Yukinari Hisaki, Yutaka Kondo, awf-autoware-bot[bot], kobayu858, taisa1

0.26.0 (2024-04-03)
-------------------
