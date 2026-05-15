^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_shape_estimation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* refactor(autoware_universe): use autoware_ament_auto_package in perception utility packages (`#12281 <https://github.com/mitsudome-r/autoware_universe/issues/12281>`_)
  Co-authored-by: github-actions <github-actions@github.com>
* feat(shape_estimation): use CallbackIsolatedAgnocastExecutor for shape_estimation (`#12357 <https://github.com/mitsudome-r/autoware_universe/issues/12357>`_)
  * apply cie to shape_estimation
  * fix to use AgnocastOnlyExecutor
  ---------
* feat(autoware_shape_estimation): replace to agnocast wrapper (`#12308 <https://github.com/mitsudome-r/autoware_universe/issues/12308>`_)
  * feat: replace agnocast_wrapper_register_node
  * feat: replace node
  * fix: executor name
  * feat: replace pub sub
  * style(pre-commit): autofix
  * fix: remove agnocastlib dependency
  * fix: revert autoware_utils
  * style(pre-commit): autofix
  * fix: add inlude
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Koichi Imai <45482193+Koichi98@users.noreply.github.com>
* perf(perception): use emplace_back and emplace to avoid temporary object creation (`#12201 <https://github.com/mitsudome-r/autoware_universe/issues/12201>`_)
  * perf(perception): use emplace_back to avoid temporary object creation
  * style(pre-commit): autofix
  * perf(perception): use emplace/emplace_back for most containers
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* fix(autoware_shape_estimation): cuda and tensorrt condition checks (`#12200 <https://github.com/mitsudome-r/autoware_universe/issues/12200>`_)
  fix(CMakeLists): CUDA and TensorRT condition checks
* Contributors: Koichi Imai, Mete Fatih Cırıt, Taekjin LEE, Tetsuhiro Kawaguchi, Vishal Chauhan, github-actions, nishikawa-masaki

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat!: remove ROS 2 Galactic codes (`#11905 <https://github.com/autowarefoundation/autoware_universe/issues/11905>`_)
* chore(autoware_shape_estimation): remove cudnn dependency (`#11892 <https://github.com/autowarefoundation/autoware_universe/issues/11892>`_)
* feat: simulation related packages support jazzy (`#11424 <https://github.com/autowarefoundation/autoware_universe/issues/11424>`_)
* Contributors: Amadeusz Szymko, Ryohsuke Mitsudome, 心刚

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* feat(shape_estimation): correcting yaw (`#11309 <https://github.com/autowarefoundation/autoware_universe/issues/11309>`_) (`#11634 <https://github.com/autowarefoundation/autoware_universe/issues/11634>`_)
  * feat(shape_estimation): correcting yaw
  * fix test
  * fix: test and pre-commit
  ---------
* feat: refactor shape estimation and documentations; from autowarefoundation `#11353 <https://github.com/autowarefoundation/autoware_universe/issues/11353>`_ (`#11631 <https://github.com/autowarefoundation/autoware_universe/issues/11631>`_)
  * feat: refactor shape estimation and documentations; from autowarefoundation `#11353 <https://github.com/autowarefoundation/autoware_universe/issues/11353>`_
  * fix CI
  * fix: CI last line
  ---------
* Contributors: Ryohsuke Mitsudome, Yuxuan Liu

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* chore(perception): add maintainer (`#11458 <https://github.com/autowarefoundation/autoware_universe/issues/11458>`_)
  add maintainer
* Contributors: Masaki Baba, Ryohsuke Mitsudome, Tim Clephas

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): autofix (`#10982 <https://github.com/autowarefoundation/autoware_universe/issues/10982>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome

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
* Merge remote-tracking branch 'origin/main' into humble
* chore(perception): code owner revision (`#10358 <https://github.com/autowarefoundation/autoware_universe/issues/10358>`_)
  * feat: add Masato Saeki and Taekjin Lee as maintainer to multiple package.xml files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Taekjin LEE

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
* feat(autoware_shape_estimation): tier4_debug_msgs chnaged to autoware_internal_debug_msgs in autoware_shape_estimation (`#9897 <https://github.com/autowarefoundation/autoware_universe/issues/9897>`_)
  feat: tier4_debug_msgs chnaged to autoware_internal_debug_msgs in files  perception/autoware_shape_estimation
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
* fix(autoware_shape_estimation): fix bugprone-branch-clone (`#9659 <https://github.com/autowarefoundation/autoware_universe/issues/9659>`_)
  * fix: bugprone-error
  * fix: fmt
  * fix: pre-commit
  * fix: pre-commit
  ---------
* Contributors: Amadeusz Szymko, Fumiya Watanabe, Vishal Chauhan, kobayu858

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
* refactor: correct spelling (`#9528 <https://github.com/autowarefoundation/autoware_universe/issues/9528>`_)
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

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
* feat(autoware_shape_estimation): add reference object based corrector (`#9148 <https://github.com/autowarefoundation/autoware_universe/issues/9148>`_)
  * add object based corrector
  * apply cppcheck suggestion
  * fix typo
  ---------
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* refactor(tensorrt_common)!: fix namespace, directory structure & move to perception namespace (`#9099 <https://github.com/autowarefoundation/autoware_universe/issues/9099>`_)
  * refactor(tensorrt_common)!: fix namespace, directory structure & move to perception namespace
  * refactor(tensorrt_common): directory structure
  * style(pre-commit): autofix
  * fix(tensorrt_common): correct package name for logging
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* test(autoware_shape_estimation): add unit testing for `ShapeEstimationNode` (`#8740 <https://github.com/autowarefoundation/autoware_universe/issues/8740>`_)
  test: add unit testing for `ShapeEstimationNode`
* fix(autoware_shape_estimation): fix unusedFunction (`#8575 <https://github.com/autowarefoundation/autoware_universe/issues/8575>`_)
  * fix:unusedFunction
  * fix:unusedFunction
  * fix:end of file issues
  * fix:copyright
  ---------
* fix(autoware_shape_estimation): resolve undefined reference to `~TrtShapeEstimator()` (`#8738 <https://github.com/autowarefoundation/autoware_universe/issues/8738>`_)
  fix: resolve undefined reference to `~TrtShapeEstimator()`
* feat(shape_estimation): add ml shape estimation (`#7860 <https://github.com/autowarefoundation/autoware_universe/issues/7860>`_)
  * feat(shape_estimation): add ml shape estimation
  * style(pre-commit): autofix
  * feat(shape_estimation): fix exceed objects
  * style(pre-commit): autofix
  * feat(shape_estimation): fix indent
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(shape_estimation): add package name prefix of autoware\_ (`#7999 <https://github.com/autowarefoundation/autoware_universe/issues/7999>`_)
  * refactor(shape_estimation): add package name prefix of autoware\_
  * style(pre-commit): autofix
  * fix: mising prefix
  * fix: cmake
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Amadeusz Szymko, Kaan Çolak, Kotaro Uetake, Masaki Baba, Yutaka Kondo, badai nguyen, kobayu858

0.26.0 (2024-04-03)
-------------------
