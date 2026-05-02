^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_tensorrt_yolox
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat: default artifact paths to ~/autoware_data/ml_models (`#12523 <https://github.com/mitsudome-r/autoware_universe/issues/12523>`_)
  feat(launches,configs): default artifact paths to ~/autoware_data/ml_models
  Roll every per-package `data_path` / `model_path` launch-arg default
  from `$(env HOME)/autoware_data[/...]` to
  `$(env HOME)/autoware_data/ml_models[/...]` so standalone universe
  launches resolve artifacts under the new `~/autoware_data/ml_models/`
  layout (`autowarefoundation/autoware#7068 <https://github.com/autowarefoundation/autoware/issues/7068>`_).
  When invoked through autoware_launch the parent overrides cascade and
  already pin the new root (`autowarefoundation/autoware_launch#1835 <https://github.com/autowarefoundation/autoware_launch/issues/1835>`_); this
  commit closes the gap for users who launch a perception / localization /
  sensing / planning component directly with `ros2 launch <pkg>`.
  22 launch files updated (one-line default change each):
  - e2e/autoware_tensorrt_vad/launch/vad_carla_tiny.launch.xml
  - localization/yabloc/yabloc_pose_initializer/launch/yabloc_pose_initializer.launch.xml
  - perception/autoware_bevfusion/launch/bevfusion.launch.xml
  - perception/autoware_camera_streampetr/launch/streampetr.launch.xml
  - perception/autoware_image_projection_based_fusion/launch/pointpainting_fusion.launch.xml
  - perception/autoware_lidar_apollo_instance_segmentation/launch/lidar_apollo_instance_segmentation.launch.xml
  - perception/autoware_lidar_centerpoint/launch/lidar_centerpoint.launch.xml
  - perception/autoware_lidar_frnet/launch/lidar_frnet.launch.xml
  - perception/autoware_lidar_transfusion/launch/lidar_transfusion.launch.xml
  - perception/autoware_ptv3/launch/ptv3.launch.xml
  - perception/autoware_shape_estimation/launch/shape_estimation.launch.xml
  - perception/autoware_simpl_prediction/launch/simpl.launch.xml
  - perception/autoware_tensorrt_bevdet/launch/tensorrt_bevdet.launch.xml
  - perception/autoware_tensorrt_bevformer/launch/bevformer.launch.xml
  - perception/autoware_tensorrt_yolox/launch/{yolox_traffic_light_detector,yolox_tiny,yolox_s_plus_opt}.launch.xml
  - perception/autoware_traffic_light_classifier/launch/{car,pedestrian}_traffic_light_classifier.launch.xml
  - perception/autoware_traffic_light_fine_detector/launch/traffic_light_fine_detector.launch.xml
  - planning/autoware_diffusion_planner/launch/diffusion_planner.launch.xml
  - sensing/autoware_calibration_status_classifier/launch/calibration_status_classifier.launch.xml
  Drive-by README and test fixes:
  - e2e/autoware_tensorrt_vad/{README.md,docs/design.md}: also migrate the
  `$HOME/autoware_map/Town01` examples to `$HOME/autoware_data/maps/Town01`.
  - localization/yabloc/{README.md,yabloc_pose_initializer/README.md}: also
  migrate `$HOME/autoware_map/sample-map-rosbag` to
  `$HOME/autoware_data/maps/demos/sample-map-rosbag`.
  - control/autoware_smart_mpc_trajectory_follower/README.md: migrate the
  `map_path:=$HOME/autoware_map/sample-map-planning` example to
  `$HOME/autoware_data/maps/demos/sample-map-planning`.
  - simulator/autoware_carla_interface/README.md: migrate every
  `$HOME/autoware_map/Town01/...` reference to
  `$HOME/autoware_data/maps/Town01/...`.
  - perception/{autoware_bevfusion,autoware_image_projection_based_fusion,autoware_lidar_centerpoint,autoware_tensorrt_bevformer}/README.md: copy-paste examples updated to `~/autoware_data/ml_models/<pkg>`.
  - perception/autoware_camera_streampetr/config/ml_package_camera_streampetr.param.yaml: header comment updated.
  - planning/autoware_diffusion_planner/README.md: prerequisites snippet updated.
  - sensing/autoware_calibration_status_classifier/test/{test_model_inference,test_calibration_status_classifier}.cpp: hardcoded fallback ONNX path updated.
  Users on the legacy layout can pin the old root with
  `data_path:=$HOME/autoware_data` (or the per-package equivalent) on the
  command line.
  Refs: https://github.com/autowarefoundation/autoware/issues/7068
* chore(perception): move perception node configuration file to each package (`#12440 <https://github.com/mitsudome-r/autoware_universe/issues/12440>`_)
  move perception node configuration file to each package
* fix(autoware_tensorrt_yolox): add missing cstdint include and ament_index_cpp test dependency (`#12400 <https://github.com/mitsudome-r/autoware_universe/issues/12400>`_)
  label.hpp uses uint32_t without including <cstdint>, causing build
  failure on Ubuntu 24.04 / ROS 2 Jazzy. test_label.cpp uses
  ament_index_cpp but it was not declared as a test dependency.
* refactor(autoware_tensorrt_yolox): parameterize label remapping (`#12204 <https://github.com/mitsudome-r/autoware_universe/issues/12204>`_)
  * fix hard-coded label remapping issue by introducing file-based loading for flexibility and maintainability
  * add remap files
  * style(pre-commit): autofix
  * fix to camel case
  * remove commented out lines
  * trim spcaces in label name
  * remove unused functions
  * add test for label processing
  * update schema
  * add cspell ignore
  * reduce scope
  * fix uselessCallsSubstr
  * style(pre-commit): autofix
  * add missing header
  * fix typo
  * fix comment
  * fix header
  * rename parameter and file
  * fix comment spacing
  * add roi to semseg remap param
  * style(pre-commit): autofix
  * rename variable from roi_id_to_name_map to roi_class_name_list
  * add comment
  * fix to parameter name
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * change initialization to method chaining
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * use local variable
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
* perf(perception): use emplace_back and emplace to avoid temporary object creation (`#12201 <https://github.com/mitsudome-r/autoware_universe/issues/12201>`_)
  * perf(perception): use emplace_back to avoid temporary object creation
  * style(pre-commit): autofix
  * perf(perception): use emplace/emplace_back for most containers
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* feat(autoware_tensorrt_yolox): restore Turing arch compatibility (`#12212 <https://github.com/mitsudome-r/autoware_universe/issues/12212>`_)
* feat(autoware_tensorrt_yolox): cuda 12.0 build compatibility (`#12192 <https://github.com/mitsudome-r/autoware_universe/issues/12192>`_)
  feat(autoware_tensorrt_yolox): CUDA 12.0+ build compatibility
* Contributors: Amadeusz Szymko, Masaki Baba, Mete Fatih Cırıt, Taekjin LEE, github-actions, nishikawa-masaki

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* refactor(tensorrt_yolox): split utility functions (`#12042 <https://github.com/autowarefoundation/autoware_universe/issues/12042>`_)
  * move util functions to utils
  * style(pre-commit): autofix
  * change name utils to label
  * style(pre-commit): autofix
  * move unnamed namespace under tensorrt_yolox namespace
  * remove static
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_tensorrt_yolox): update nvcc flags (`#12057 <https://github.com/autowarefoundation/autoware_universe/issues/12057>`_)
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
* chore(autoware_tensorrt_yolox): remove cudnn dependency (`#11898 <https://github.com/autowarefoundation/autoware_universe/issues/11898>`_)
* feat(autoware_tensorrt_yolox): add schema for autoware_tensorrt_yolox (`#10047 <https://github.com/autowarefoundation/autoware_universe/issues/10047>`_)
  * feat(autoware_tensorrt_yolox):Add schema for autoware_tensorrt_yolox
  * Update yolox_s_plus_opt.schema.json
  * Update yolox_s_plus_opt.schema.json
  * Update yolox_tiny.schema.json
  update min and max value in read file for "score_threshold"
  * Update yolox_tiny.schema.json
  update min and max value in read file for "nms_threshold"
  * fix: apply pre-commit
  * fix: add required model_path, label_path, and color_map_path to YOLOX parameter files for schema compliance
  * style(pre-commit): autofix
  * feat(tensorrt_yolox): Update schemas to include new parameters
  * style(pre-commit): autofix
  * chore
  ---------
  Co-authored-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: MasatoSaeki <masato.saeki@tier4.jp>
  Co-authored-by: Masato Saeki <78376491+MasatoSaeki@users.noreply.github.com>
* Contributors: Amadeusz Szymko, Masaki Baba, Ryohsuke Mitsudome, Vishal Chauhan

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
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* chore: perception code owner update (`#10645 <https://github.com/autowarefoundation/autoware_universe/issues/10645>`_)
  * chore: update maintainers in multiple perception packages
  * Revert "chore: update maintainers in multiple perception packages"
  This reverts commit f2838c33d6cd82bd032039e2a12b9cb8ba6eb584.
  * chore: update maintainers in multiple perception packages
  * chore: add Kok Seang Tan as maintainer in multiple perception packages
  ---------
* feat(autoware_tensorrt_yolox): added target architectures for yolox (`#10611 <https://github.com/autowarefoundation/autoware_universe/issues/10611>`_)
  * chore: added target architectures for yolox
  * chore: mistook the compute capabilities of edge devices
  * chore: cspell
  ---------
* Contributors: Kenzo Lobos Tsunekawa, Taekjin LEE, TaikiYamada4

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(tensorrt_yolox): add autoware_utils packages (`#10460 <https://github.com/autowarefoundation/autoware_universe/issues/10460>`_)
  Co-authored-by: t4-adc <grp-rd-1-adc-admin@tier4.jp>
* fix(autoware_tensorrt_yolox): explicitly install shared library (`#10454 <https://github.com/autowarefoundation/autoware_universe/issues/10454>`_)
* Contributors: Kazunori-Nakajima, Manato Hirabayashi, Ryohsuke Mitsudome

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* chore(perception): refactor perception launch (`#10186 <https://github.com/autowarefoundation/autoware_universe/issues/10186>`_)
  * fundamental change
  * style(pre-commit): autofix
  * fix typo
  * fix params and modify some packages
  * pre-commit
  * fix
  * fix spell check
  * fix typo
  * integrate model and label path
  * style(pre-commit): autofix
  * for pre-commit
  * run pre-commit
  * for awsim
  * for simulatior
  * style(pre-commit): autofix
  * fix grammer in launcher
  * add schema for yolox_tlr
  * style(pre-commit): autofix
  * fix file name
  * fix
  * rename
  * modify arg name  to
  * fix typo
  * change param name
  * style(pre-commit): autofix
  * chore
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Shintaro Tomie <58775300+Shin-kyoto@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* refactor: add autoware_cuda_dependency_meta (`#10073 <https://github.com/autowarefoundation/autoware_universe/issues/10073>`_)
* Contributors: Esteve Fernandez, Hayato Mizushima, Masato Saeki, Yutaka Kondo

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
* feat(autoware_tensorrt_yolox)!: tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_tensorrt_yolox (`#9898 <https://github.com/autowarefoundation/autoware_universe/issues/9898>`_)
* feat(tensorrt_yolox): add launch for tlr model (`#9828 <https://github.com/autowarefoundation/autoware_universe/issues/9828>`_)
  * feat(tensorrt_yolox): add launch for tlr model
  * chore: typo
  * docs: update readme and description
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_tensorrt_yolox): modify tensorrt_yolox_node name (`#9156 <https://github.com/autowarefoundation/autoware_universe/issues/9156>`_)
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
* fix(autoware_tensorrt_yolox): fix bugprone-exception-escape (`#9734 <https://github.com/autowarefoundation/autoware_universe/issues/9734>`_)
  * fix: bugprone-error
  * fix: fmt
  * fix: fmt
  ---------
* Contributors: Amadeusz Szymko, Fumiya Watanabe, Vishal Chauhan, badai nguyen, cyn-liu, kobayu858

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
* fix(autoware_tensorrt_yolox): fix clang-diagnostic-inconsistent-missing-override (`#9512 <https://github.com/autowarefoundation/autoware_universe/issues/9512>`_)
  fix: clang-diagnostic-inconsistent-missing-override
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
* refactor(cuda_utils): prefix package and namespace with autoware (`#9171 <https://github.com/autowarefoundation/autoware_universe/issues/9171>`_)
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
* refactor(cuda_utils): prefix package and namespace with autoware (`#9171 <https://github.com/autowarefoundation/autoware_universe/issues/9171>`_)
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
* feat(autoware_tensorrt_yolox): add GPU - CUDA device option (`#8245 <https://github.com/autowarefoundation/autoware_universe/issues/8245>`_)
  * init CUDA device option
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(autoware_tensorrt_yolox): add Kotaro Uetake as maintainer (`#8595 <https://github.com/autowarefoundation/autoware_universe/issues/8595>`_)
  chore: add Kotaro Uetake as maintainer
* fix: cpp17 namespaces (`#8526 <https://github.com/autowarefoundation/autoware_universe/issues/8526>`_)
  Use traditional-style nameplace nesting for nvcc
  Co-authored-by: Yuri Guimaraes <yuri.kgpps@gmail.com>
* fix(docs): fix docs for tensorrt yolox (`#8304 <https://github.com/autowarefoundation/autoware_universe/issues/8304>`_)
  fix docs for tensorrt yolox
* refactor(tensorrt_yolox): move utils into perception_utils (`#8435 <https://github.com/autowarefoundation/autoware_universe/issues/8435>`_)
  * chore(tensorrt_yolo): refactor utils
  * style(pre-commit): autofix
  * fix: tensorrt_yolox
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_tensorrt_yolox): fix variableScope (`#8430 <https://github.com/autowarefoundation/autoware_universe/issues/8430>`_)
  fix: variableScope
  Co-authored-by: kobayu858 <129580202+kobayu858@users.noreply.github.com>
* fix(tensorrt_yolox): add run length encoding for sematic segmentation mask (`#7905 <https://github.com/autowarefoundation/autoware_universe/issues/7905>`_)
  * fix: add rle compress
  * fix: rle compress
  * fix: move rle into utils
  * chore: pre-commit
  * Update perception/autoware_tensorrt_yolox/src/utils.cpp
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
  * fix: remove unused variable
  * Update perception/autoware_tensorrt_yolox/src/utils.cpp
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * style(pre-commit): autofix
  * feat: add unit test for utils
  * style(pre-commit): autofix
  * fix: unit test
  * chore: change to explicit index
  * style(pre-commit): autofix
  * fix: cuda cmake
  * fix: separate unit test into different PR
  ---------
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_tensorrt_yolox): fix unreadVariable (`#8356 <https://github.com/autowarefoundation/autoware_universe/issues/8356>`_)
  * fix:unreadVariable
  * fix:unreadVariable
  ---------
* refactor: image transport decompressor/autoware prefix (`#8197 <https://github.com/autowarefoundation/autoware_universe/issues/8197>`_)
  * refactor: add `autoware` namespace prefix to image_transport_decompressor
  * refactor(image_transport_decompressor): add `autoware` prefix to the package code
  * refactor: update package name in CODEOWNER
  * fix: merge main into the branch
  * refactor: update packages which depend on image_transport_decompressor
  * refactor(image_transport_decompressor): update README
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* refactor(tensorrt_yolox)!: fix namespace and directory structure (`#7992 <https://github.com/autowarefoundation/autoware_universe/issues/7992>`_)
  * refactor: add autoware namespace prefix to `tensorrt_yolox`
  * refactor: apply `autoware` namespace to tensorrt_yolox
  * chore: update CODEOWNERS
  * fix: resolve `yolox_tiny` to work
  ---------
* Contributors: Abraham Monrroy Cano, Amadeusz Szymko, Esteve Fernandez, Ismet Atabay, Kotaro Uetake, Manato Hirabayashi, Nagi70, Yutaka Kondo, Yuxuan Liu, badai nguyen, kobayu858

0.26.0 (2024-04-03)
-------------------
