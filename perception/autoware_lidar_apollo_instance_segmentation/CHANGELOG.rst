^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_lidar_apollo_instance_segmentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Contributors: Mete Fatih Cırıt, Taekjin LEE, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat!: remove ROS 2 Galactic codes (`#11905 <https://github.com/autowarefoundation/autoware_universe/issues/11905>`_)
* fix(lidar_apollo_instance_segmentation): add missing dep, clean up CMakeLists (`#11876 <https://github.com/autowarefoundation/autoware_universe/issues/11876>`_)
* fix: add missing ament_index_cpp dependency (`#11875 <https://github.com/autowarefoundation/autoware_universe/issues/11875>`_)
* Contributors: Mete Fatih Cırıt, Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* fix(lidar_apollo_instance_segmentation): fixed preprocessing (`#8172 <https://github.com/autowarefoundation/autoware_universe/issues/8172>`_)
  Co-authored-by: Mete Fatih Cırıt <mfc@autoware.org>
* fix(autoware_lidar_apollo_instance_segmentation): add empty point cloud guard (`#11745 <https://github.com/autowarefoundation/autoware_universe/issues/11745>`_)
  * fix(autoware_lidar_apollo_instance_segmentation): add empty point cloud guard
  Add validation to check for empty point clouds before processing to prevent
  undefined behavior and potential crashes.
  * Apply suggestion from @Copilot
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  * Update perception/autoware_lidar_apollo_instance_segmentation/src/node.cpp
  * Update perception/autoware_lidar_apollo_instance_segmentation/src/node.cpp
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Kenzo Lobos Tsunekawa, Ryohsuke Mitsudome, Yutaka Kondo

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* refactor(autoware_lidar_apollo_instance_segmentation): refactor launch and parameter (`#11405 <https://github.com/autowarefoundation/autoware_universe/issues/11405>`_)
  refactor params
* fix(autoware_lidar_apollo_instance_segmentation): remove invalid key and change variable int to double (`#11396 <https://github.com/autowarefoundation/autoware_universe/issues/11396>`_)
  * remove invalid key and change variable int to double
  * change type to int
  * update schema
  ---------
* fix(lidar_apollo_instance_segmentation): revert apollo parameter pr 10097 (`#11357 <https://github.com/autowarefoundation/autoware_universe/issues/11357>`_)
  * Revert "feat(autoware_lidar_apollo_instance_segmentation): created the schema file, updated the readme file and deleted the default parameter in node files (`#10097 <https://github.com/autowarefoundation/autoware_universe/issues/10097>`_)"
  This reverts commit dcb9739b5c684afaede1f164f259b7364a7c32cf.
  * feat(autoware_lidar_apollo_instance_segmentation): created the schema file, updated the readme file and deleted the default parameter in node files (`#10097 <https://github.com/autowarefoundation/autoware_universe/issues/10097>`_)
  * feat(autoware_lidar_apollo_instance_segmentation): Created the schema file, updated the readme file and deleted the default parameter in node files
  * style(pre-commit): autofix
  * Update hdl-64.schema.json
  * Update vlp-16.schema.json
  * Update vls-128.schema.json
  * Update hdl-64.param.yaml
  * Update vlp-16.param.yaml
  * Update vls-128.param.yaml
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * fix(apollo_instance_segmentation): nest params under lidar_instance_segmentation; remove stray pipe
  * fix(apollo_instance_segmentation): nest params under lidar_instance_segmentation for schema compliance
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_lidar_apollo_instance_segmentation): created the schema file, updated the readme file and deleted the default parameter in node files (`#10097 <https://github.com/autowarefoundation/autoware_universe/issues/10097>`_)
  * feat(autoware_lidar_apollo_instance_segmentation): Created the schema file, updated the readme file and deleted the default parameter in node files
  * style(pre-commit): autofix
  * Update hdl-64.schema.json
  * Update vlp-16.schema.json
  * Update vls-128.schema.json
  * Update hdl-64.param.yaml
  * Update vlp-16.param.yaml
  * Update vls-128.param.yaml
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * fix(apollo_instance_segmentation): nest params under lidar_instance_segmentation; remove stray pipe
  * fix(apollo_instance_segmentation): nest params under lidar_instance_segmentation for schema compliance
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* Contributors: Masato Saeki, Ryohsuke Mitsudome, Taekjin LEE, Vishal Chauhan

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
* Contributors: Taekjin LEE, TaikiYamada4

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
* refactor: add autoware_cuda_dependency_meta (`#10073 <https://github.com/autowarefoundation/autoware_universe/issues/10073>`_)
* Contributors: Esteve Fernandez, Hayato Mizushima, Yutaka Kondo

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
* feat(autoware_lidar_apollo_instance_segmentation): tier4_debug_msgs to autoware_internal_debug_msgs in files perce… (`#9876 <https://github.com/autowarefoundation/autoware_universe/issues/9876>`_)
  feat: tier4_debug_msgs to autoware_internal_debug_msgs in files perception/autoware_lidar_apollo_instance_segmentation
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
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
* Contributors: Amadeusz Szymko, Fumiya Watanabe, Vishal Chauhan

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
* fix(autoware_lidar_apollo_instance_segmentation): fix cppcheck suspiciousFloatingPointCast (`#9195 <https://github.com/autowarefoundation/autoware_universe/issues/9195>`_)
* refactor(cuda_utils): prefix package and namespace with autoware (`#9171 <https://github.com/autowarefoundation/autoware_universe/issues/9171>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Ryuta Kambe, Yutaka Kondo

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
* fix(autoware_lidar_apollo_instance_segmentation): fix cppcheck suspiciousFloatingPointCast (`#9195 <https://github.com/autowarefoundation/autoware_universe/issues/9195>`_)
* refactor(cuda_utils): prefix package and namespace with autoware (`#9171 <https://github.com/autowarefoundation/autoware_universe/issues/9171>`_)
* Contributors: Esteve Fernandez, Ryuta Kambe, Yutaka Kondo

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
* fix(autoware_lidar_apollo_instance_segmentation): added existence probability (`#8862 <https://github.com/autowarefoundation/autoware_universe/issues/8862>`_)
  * added existence probability
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(lidar_apollo_instance_segmentation): fix critical bug (`#8444 <https://github.com/autowarefoundation/autoware_universe/issues/8444>`_)
  Co-authored-by: Shintaro Tomie <58775300+Shin-kyoto@users.noreply.github.com>
* refactor(lidar_apollo_instance_segmentation)!: fix namespace and directory structure (`#7995 <https://github.com/autowarefoundation/autoware_universe/issues/7995>`_)
  * refactor: add autoware namespace prefix
  * chore: update CODEOWNERS
  * refactor: add `autoware` prefix
  ---------
* Contributors: Amadeusz Szymko, Kotaro Uetake, Samrat Thapa, Yutaka Kondo, kminoda

0.26.0 (2024-04-03)
-------------------
