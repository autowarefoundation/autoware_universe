^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_calibration_status_classifier
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* feat(autoware_calibration_status_classifier): add angular velocity filter and refine filters design (`#12286 <https://github.com/mitsudome-r/autoware_universe/issues/12286>`_)
  * feat(autoware_calibration_status_classifier): add angular velocity filter and refine filters design
  * feat(autoware_calibration_status_classifier): add per-pair threshold
  * refactor(autoware_calibration_status_classifier): reverse naming convention logic
  * fix(autoware_calibration_status_classifier): adjust mutex
  ---------
* feat(autoware_calibration_status_classifier): add vehicle's ego raycasting filter (`#12272 <https://github.com/mitsudome-r/autoware_universe/issues/12272>`_)
  * feat(autoware_calibration_status_classifier): add vehicle's ego raycasting filter
  * refactor(autoware_calibration_status_classifier): move parameter
  * style(pre-commit): autofix
  * refactor(autoware_calibration_status_classifier): ego_box parameter placement
  * feat(autoware_calibration_status_classifier): detailed mask key
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_calibration_status_classifier): cuda 12.0 build compatibility (`#12193 <https://github.com/mitsudome-r/autoware_universe/issues/12193>`_)
  * feat(autoware_calibration_status_classifier): CUDA 12.0+ build compatibility
  * feat: restore Turing arch
  ---------
* Contributors: Amadeusz Szymko, Mete Fatih Cırıt, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_calibration_status_classifier): update nvcc flags (`#12058 <https://github.com/autowarefoundation/autoware_universe/issues/12058>`_)
* chore: update `autoware_calibration_status_classifier` maintainer (`#12035 <https://github.com/autowarefoundation/autoware_universe/issues/12035>`_)
* chore(autoware_calibration_status_classifier): remove cudnn dependency (`#11902 <https://github.com/autowarefoundation/autoware_universe/issues/11902>`_)
* chore: add maintainer of PTv3, FRNet, and CalibrationStatusClassifier (`#11945 <https://github.com/autowarefoundation/autoware_universe/issues/11945>`_)
  * chore: update `autoware_ptv3` maintainer
  * chore: update `autoware_lidar_frnet` maintainer
  * chore: update `autoware_calibration_status_classifier` maintainer
  ---------
* fix: add missing ament_index_cpp dependency (`#11875 <https://github.com/autowarefoundation/autoware_universe/issues/11875>`_)
* Contributors: Amadeusz Szymko, Manato Hirabayashi, Mete Fatih Cırıt, Patiphon Narksri, Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* feat(autoware_calibration_status_classifier): add ML-based miscalibration detection module (`#11222 <https://github.com/autowarefoundation/autoware_universe/issues/11222>`_)
  * feat(autoware_calibration_status): add ML-based miscalibration detection module
  * feat(autoware_calibration_status): extended configuration and diagnostics
  * fix(autoware_calibration_status): model's input array format
  * test(autoware_calibration_status): inference only test
  * style(pre-commit): autofix
  * refactor(autoware_calibration_status): rename lidar_range to max_depth
  * fix(autoware_calibration_status): add missing header
  * feat(autoware_calibration_status): add naive number of objects filter
  * feat(autoware_calibration_status): add periodic and manual mode
  * refactor(autoware_calibration_status): improve image handling and optimize calibration pipeline
  Refactors the calibration status module to handle both distorted and rectified images,
  reorganizes data structures, and optimizes the processing pipeline. Adds new utility
  classes for better camera/LiDAR information management.
  * style(pre-commit): autofix
  * style(autoware_calibration_status): pre-commit
  * test(autoware_calibration_status): make that CI skip unit tests
  * style(autoware_calibration_status): cspell
  * test(autoware_calibration_status): skip test before loading data
  * test(autoware_calibration_status): another yet attempt to fix CI
  * style(autoware_calibration_status): cspell
  * fix(autoware_calibration_status): correct types
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * fix(autoware_calibration_status): correct types
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * style(pre-commit): autofix
  * fix(autoware_calibration_status): user desired function for cuda memory allocation
  * style(autoware_calibration_status): early return instead of scoped implementaiton
  * feat(autoware_calibration_status): use of  __restrict_\_ keyword
  * docs(autoware_calibration_status): update future work
  * fix(autoware_calibration_status): include missing directory
  * fix(autoware_calibration_status): use preallocated class member
  * style(pre-commit): autofix
  * style(autoware_calibration_status): use lambda for adding diagnostic
  * style(autoware_calibration_status): split function
  * style(pre-commit): autofix
  * refactor(autoware_calibration_status): change atomic operation logic and extras
  * refactor(autoware_calibration_status): use autoware diagnostic interface
  * fix(autoware_calibration_status): cspell
  * feat(autoware_calibration_status_classifier): rename autoware_calibration_status to autoware_calibration_status_classifier
  * style(pre-commit): autofix
  * fix(autoware_calibration_status_classifier): prevent potential race condition
  * fix(autoware_calibration_status_classifier): add mutex for input msgs data access
  * fix(autoware_calibration_status_classifier): pre-commit
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
* Contributors: Amadeusz Szymko, Ryohsuke Mitsudome, Tim Clephas
