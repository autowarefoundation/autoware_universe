^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_lidar_centerpoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat(autoware_lidar_centerpoint): add class-wise confidence thresholds to CenterPoint (`#10881 <https://github.com/autowarefoundation/autoware_universe/issues/10881>`_)
  * Add PreprocessCuda to CenterPoint
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * Add intensity preprocessing
  * style(pre-commit): autofix
  * Fix config\_.point_feature_size\_ typo
  * style(pre-commit): autofix
  * Fix point typo
  * style(pre-commit): autofix
  * Change score_threshold to score_thresholds
  * Use <autoware/cuda_utils/cuda_utils.hpp> for clear_async
  * Rename pre_ptr\_ to pre_proc_ptr\_
  * Remove unused getCacheSize() and getIdx
  * Use template in generateVoxels_random_kernel instead
  * style(pre-commit): autofix
  * Remove references in generateVoxels_random_kernel
  * Remove references in generateVoxels_random_kernel
  * style(pre-commit): autofix
  * Remove generateIntensityFeatures_kernel and add the case of 11 to ENCODER_IN_FEATURE_SIZE for generateFeatures_kernel
  * style(pre-commit): autofix
  * Add class-wise confidence thresholds to CenterPoint
  * style(pre-commit): autofix
  * Remov empty line changes
  * Update score_threshold to score_thresholds in REAMME
  * style(pre-commit): autofix
  * Change score_thresholds from pass by value to pass by reference
  * style(pre-commit): autofix
  * Add information about class names in scehema
  * Change vector<double> to vector<float>
  * Remove thrust and add stream\_ to PostProcessCUDA
  * style(pre-commit): autofix
  * Fix incorrect initialization of score_thresholds\_ vector
  * Fix postprocess CudaMemCpy error
  * Fix postprocess score_thresholds_d_ptr\_ typing error
  * Fix score_thresholds typing in node.cpp
  * Static casting params.score_thresholds vector
  * style(pre-commit): autofix
  * Update perception/autoware_lidar_centerpoint/src/node.cpp
  * Update perception/autoware_lidar_centerpoint/include/autoware/lidar_centerpoint/centerpoint_config.hpp
  * Update centerpoint_config.hpp
  * Update node.cpp
  * Update score_thresholds\_ to double since ros2 supports only double instead of float
  * style(pre-commit): autofix
  * Fix cuda memory and revert double score_thresholds\_ to float score_thresholds\_
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
* feat(autoware_lidar_centerpoint): add Intensity support to CenterPoint (`#10854 <https://github.com/autowarefoundation/autoware_universe/issues/10854>`_)
  * Add PreprocessCuda to CenterPoint
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * Add intensity preprocessing
  * style(pre-commit): autofix
  * Fix config\_.point_feature_size\_ typo
  * style(pre-commit): autofix
  * Fix point typo
  * style(pre-commit): autofix
  * Use <autoware/cuda_utils/cuda_utils.hpp> for clear_async
  * Rename pre_ptr\_ to pre_proc_ptr\_
  * Remove unused getCacheSize() and getIdx
  * Use template in generateVoxels_random_kernel instead
  * style(pre-commit): autofix
  * Remove references in generateVoxels_random_kernel
  * Remove references in generateVoxels_random_kernel
  * style(pre-commit): autofix
  * Remove generateIntensityFeatures_kernel and add the case of 11 to ENCODER_IN_FEATURE_SIZE for generateFeatures_kernel
  * style(pre-commit): autofix
  * Remov empty line changes
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Kok Seang Tan

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* chore(perception): delete maintainer name (`#10816 <https://github.com/autowarefoundation/autoware_universe/issues/10816>`_)
* feat(autoware_lidar_centerpoint): add ShortRange centerpoint as another new node base on autoware_lidar_centerpoint (`#10727 <https://github.com/autowarefoundation/autoware_universe/issues/10727>`_)
  * Add shortrange_centerpoint launcher to autoware_lidar_centerpoint
  * style(pre-commit): autofix
  * Update README.md to include launching of shortrange centerpoint
  * style(pre-commit): autofix
  * Add option to launch shortrange centerpoint in tier4_perception_launch
  * Add option to launch shortrange centerpoint in tier4_perception_launch
  * style(pre-commit): autofix
  * Update correct launcher name in lidar_dnn_detection.launch.xml
  * Fix Update typo in lidar_dnn_detector launcher for shortrange centerpoint
  * Add use_short_range_detection to launcher for switching shortrange detection
  * Add use_short_range_detection to launcher for switching shortrange detection
  * Format spaces
  * style(pre-commit): autofix
  * Rename shortrange to short_range
  * Change shor_range centerpoint to one of the centerpoint variants
  * Fix shortrange spelling warning
  * Fix default value for use_short_range_centerpoint and fix missing short_range_centerpoint_common.param.yaml
  * Fix default value for use_short_range_centerpoint
  * Rename short_range_centerpoint to centerpoint_short_range for model name and type
  * Rename short_range_centerpoint to centerpoint_short_range for model name and type
  * Update value to default
  * Update short range centerpoint namespace
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Kok Seang Tan, Satoshi Tanaka, TaikiYamada4

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
* feat(autoware_lidar_centerpoint): add distance of z to pillar center as a new lidar encoder feature in CenterPoint (`#10592 <https://github.com/autowarefoundation/autoware_universe/issues/10592>`_)
  * Add support of distance of z to pillar center in centerpoint
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Amadeusz Szymko <amadeusz.szymko.2@tier4.jp>
* feat(autoware_lidar_centerpoint): added target architectures for centerpoint (`#10616 <https://github.com/autowarefoundation/autoware_universe/issues/10616>`_)
  * feat: added target architectures for centerpoint
  * chore: mistook the capabilities of edge devices
  ---------
* Contributors: Kenzo Lobos Tsunekawa, Kok Seang Tan, Taekjin LEE, TaikiYamada4

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(lidar_centerpoint): add diagnostics for processing time (`#10284 <https://github.com/autowarefoundation/autoware_universe/issues/10284>`_)
  * add diagnostics for processing time
  * style(pre-commit): autofix
  * change to timer callback to check the error
  * style(pre-commit): autofix
  * add ms in name
  * fix cppcheck error
  * change parameter name
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * change structure
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * change var name
  * change config related part
  * fix typo
  * change launch arg name
  * add schema file
  * refactor(autoware_lidar_centerpoint): remove unused is_processing_delayed\_ variable
  * add processing time in diag.
  * style(pre-commit): autofix
  * refactor
  * style(pre-commit): autofix
  * use diagnostic_updater instead of timer callback
  * style(pre-commit): autofix
  * change validation_callback_interval_ms type to number
  * Update perception/autoware_lidar_centerpoint/src/node.cpp
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* chore(perception): code owner revision (`#10358 <https://github.com/autowarefoundation/autoware_universe/issues/10358>`_)
  * feat: add Masato Saeki and Taekjin Lee as maintainer to multiple package.xml files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_lidar_centerpoint): added the cuda_blackboard to centerpoint (`#9453 <https://github.com/autowarefoundation/autoware_universe/issues/9453>`_)
  * feat: introduced the cuda transport layer (cuda blackboard) to centerpoint
  * chore: fixed compilation issue on pointpainting
  * fix: fixed compile errors in the ml models
  * fix: fixed standalone non-composed launcher
  * chore: ci/cd
  * chore: clang tidy related fix
  * chore: removed non applicable override (point painting does not support the blackboard yet)
  * chore: temporarily ignoring warning until pointpainting also supports the blackboard
  * chore: ignoring spell
  * feat: removed the deprecated compatible subs option in the constructor
  * chore: bump the cuda blackboard version in the build depends
  * Update perception/autoware_image_projection_based_fusion/src/pointpainting_fusion/pointpainting_trt.cpp
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * Update perception/autoware_image_projection_based_fusion/src/pointpainting_fusion/pointpainting_trt.cpp
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  ---------
  Co-authored-by: Amadeusz Szymko <amadeusz.szymko.2@tier4.jp>
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* chore(localization, perception): remove koji minoda as maintainer from multiple packages (`#10359 <https://github.com/autowarefoundation/autoware_universe/issues/10359>`_)
  fix: remove Koji Minoda as maintainer from multiple package.xml files
* Contributors: Kenzo Lobos Tsunekawa, Masaki Baba, Ryohsuke Mitsudome, Taekjin LEE

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* refactor: add autoware_cuda_dependency_meta (`#10073 <https://github.com/autowarefoundation/autoware_universe/issues/10073>`_)
* Contributors: Esteve Fernandez, Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* chore(autoware_lidar_centerpoint): add maintainer (`#10076 <https://github.com/autowarefoundation/autoware_universe/issues/10076>`_)
* Contributors: Amadeusz Szymko, Fumiya Watanabe, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_tensorrt_common): multi-TensorRT compatibility & tensorrt_common as unified lib for all perception components (`#9762 <https://github.com/autowarefoundation/autoware_universe/issues/9762>`_)
  * refactor(autoware_tensorrt_common): multi-TensorRT compatibility & tensorrt_common as unified lib for all perception components
  * style(pre-commit): autofix
  * style(autoware_tensorrt_common): linting
  * style(autoware_lidar_centerpoint): typo
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * docs(autoware_tensorrt_common): grammar
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * fix(autoware_lidar_transfusion): reuse cast variable
  * fix(autoware_tensorrt_common): remove deprecated inference API
  * style(autoware_tensorrt_common): grammar
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * style(autoware_tensorrt_common): grammar
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  * fix(autoware_tensorrt_common): const pointer
  * fix(autoware_tensorrt_common): remove unused method declaration
  * style(pre-commit): autofix
  * refactor(autoware_tensorrt_common): readability
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * fix(autoware_tensorrt_common): return if layer not registered
  * refactor(autoware_tensorrt_common): readability
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * fix(autoware_tensorrt_common): rename struct
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
* feat(lidar_centerpoint, pointpainting): add diag publisher for max voxel size (`#9720 <https://github.com/autowarefoundation/autoware_universe/issues/9720>`_)
* fix(autoware_lidar_centerpoint): fixed rounding errors that caused illegal memory access (`#9795 <https://github.com/autowarefoundation/autoware_universe/issues/9795>`_)
  fix: fixed rounding errors that caused illegal memory address
* feat(autoware_lidar_centerpoint): process front voxels first (`#9608 <https://github.com/autowarefoundation/autoware_universe/issues/9608>`_)
  * feat: optimize voxel indexing in preprocess_kernel.cu
  * fix: remove redundant index check
  * fix: modify voxel index for better memory access
  ---------
* Contributors: Amadeusz Szymko, Fumiya Watanabe, Kenzo Lobos Tsunekawa, Taekjin LEE, kminoda

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* fix(lidar_centerpoint): non-maximum suppression target decision logic (`#9595 <https://github.com/autowarefoundation/autoware_universe/issues/9595>`_)
  * refactor(lidar_centerpoint): optimize non-maximum suppression search distance calculation
  * feat(lidar_centerpoint): do not suppress if one side of the object is pedestrian
  * style(pre-commit): autofix
  * refactor(lidar_centerpoint): remove unused variables
  * refactor: remove unused variables
  fix: implement non-maximum suppression logic to the transfusion
  refactor: remove unused parameter iou_nms_target_class_names
  Revert "fix: implement non-maximum suppression logic to the transfusion"
  This reverts commit b8017fc366ec7d67234445ef5869f8beca9b6f45.
  fix: revert transfusion modification
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* fix(cpplint): include what you use - perception (`#9569 <https://github.com/autowarefoundation/autoware_universe/issues/9569>`_)
* fix(autoware_lidar_centerpoint): fix clang-diagnostic-delete-abstract-non-virtual-dtor (`#9515 <https://github.com/autowarefoundation/autoware_universe/issues/9515>`_)
* feat(autoware_lidar_centerpoint): added a check to notify if we are dropping pillars (`#9488 <https://github.com/autowarefoundation/autoware_universe/issues/9488>`_)
  * feat: added a check to notify if we are dropping pillars
  * chore: updated text
  * chore: throttled the message
  ---------
* fix(autoware_lidar_centerpoint): fix clang-diagnostic-unused-private-field (`#9471 <https://github.com/autowarefoundation/autoware_universe/issues/9471>`_)
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kenzo Lobos Tsunekawa, M. Fatih Cırıt, Ryohsuke Mitsudome, Taekjin LEE, Yutaka Kondo, kobayu858

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
* refactor(tensorrt_common)!: fix namespace, directory structure & move to perception namespace (`#9099 <https://github.com/autowarefoundation/autoware_universe/issues/9099>`_)
  * refactor(tensorrt_common)!: fix namespace, directory structure & move to perception namespace
  * refactor(tensorrt_common): directory structure
  * style(pre-commit): autofix
  * fix(tensorrt_common): correct package name for logging
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* refactor(object_recognition_utils): add autoware prefix to object_recognition_utils (`#8946 <https://github.com/autowarefoundation/autoware_universe/issues/8946>`_)
* fix(autoware_lidar_centerpoint): fix twist covariance orientation (`#8996 <https://github.com/autowarefoundation/autoware_universe/issues/8996>`_)
  * fix(autoware_lidar_centerpoint): fix covariance converter considering the twist covariance matrix is based on the object coordinate
  fix style
  * fix: update test of box3DToDetectedObject function
  ---------
* fix(autoware_lidar_centerpoint): convert object's velocity to follow its definition (`#8980 <https://github.com/autowarefoundation/autoware_universe/issues/8980>`_)
  * fix: convert object's velocity to follow its definition in box3DToDetectedObject function
  * Update perception/autoware_lidar_centerpoint/lib/ros_utils.cpp
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
  ---------
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* feat(autoware_lidar_centerpoint): shuffled points before feeding them to the model (`#8814 <https://github.com/autowarefoundation/autoware_universe/issues/8814>`_)
  * feat: shuffling points before feeding them into the model to achieve uniform sampling into the voxels
  * Update perception/autoware_lidar_centerpoint/src/node.cpp
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  * Update perception/autoware_lidar_centerpoint/src/node.cpp
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  * Update perception/autoware_lidar_centerpoint/lib/centerpoint_trt.cpp
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  * Update perception/autoware_lidar_centerpoint/include/autoware/lidar_centerpoint/centerpoint_config.hpp
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  ---------
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
* refactor(autoware_lidar_centerpoint): use std::size_t instead of size_t (`#8820 <https://github.com/autowarefoundation/autoware_universe/issues/8820>`_)
  * refactor(autoware_lidar_centerpoint): use std::size_t instead of size_t
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(autoware_lidar_centerpoint): add centerpoint sigma parameter (`#8731 <https://github.com/autowarefoundation/autoware_universe/issues/8731>`_)
  add centerpoint sigma parameter
* fix(autoware_lidar_centerpoint): fix unusedFunction (`#8572 <https://github.com/autowarefoundation/autoware_universe/issues/8572>`_)
  fix:unusedFunction
* fix(autoware_lidar_centerpoint): place device vector in CUDA device system (`#8272 <https://github.com/autowarefoundation/autoware_universe/issues/8272>`_)
* docs(centerpoint): add description for ml package params (`#8187 <https://github.com/autowarefoundation/autoware_universe/issues/8187>`_)
* chore(autoware_lidar_centerpoint): updated tests (`#8158 <https://github.com/autowarefoundation/autoware_universe/issues/8158>`_)
  chore: updated centerpoin tests. they are currently commented out but they were not compiling (forgot to update them when I added the new cloud capacity parameter)
* refactor(lidar_centerpoint)!: fix namespace and directory structure (`#8049 <https://github.com/autowarefoundation/autoware_universe/issues/8049>`_)
  * add prefix in lidar_centerpoint
  * add .gitignore
  * change include package name in image_projection_based fusion
  * fix
  * change in codeowner
  * delete package
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * solve conflict too
  * fix include file
  * fix typo in launch file
  * add prefix in README
  * fix bugs by conflict
  * style(pre-commit): autofix
  * change namespace from  to
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* Contributors: Amadeusz Szymko, Esteve Fernandez, Kenzo Lobos Tsunekawa, Masato Saeki, Taekjin LEE, Yoshi Ri, Yutaka Kondo, kminoda, kobayu858

0.26.0 (2024-04-03)
-------------------
