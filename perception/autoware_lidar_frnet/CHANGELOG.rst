^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_lidar_frnet
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* feat(autoware_lidar_frnet): add multi-format input, ego crop box, sensor-specific config, docs (`#12000 <https://github.com/mitsudome-r/autoware_universe/issues/12000>`_)
  * feat(autoware_lidar_frnet): add multi-format input, ego crop box, sensor-specific config, docs
  * feat(autoware_lidar_frnet): remove polygon debug msg in favor of marker debug msg
  * style(autoware_lidar_frnet): rename var
  * fix(autoware_lidar_frnet): redundant var
  * feat(autoware_lidar_frnet): redundant sync
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * fix(autoware_lidar_frnet): pre-commit
  * fix(autoware_lidar_frnet): async copy
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * fix(autoware_lidar_frnet): pre-commit
  * fix(autoware_lidar_frnet): remove redundant sync
  * feat(autoware_lidar_frnet): customizable output format && refine naming convention
  * feat(autoware_lidar_frnet): update profile
  ---------
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
* feat(autoware_lidar_frnet): cuda 12.0 build compatibility (`#12185 <https://github.com/mitsudome-r/autoware_universe/issues/12185>`_)
  * feat(autoware_lidar_frnet): CUDA 12.0+ build compatibility
  * feat: restore Turing arch
  ---------
* Contributors: Amadeusz Szymko, Mete Fatih Cırıt, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_lidar_frnet): update nvcc flags (`#12050 <https://github.com/autowarefoundation/autoware_universe/issues/12050>`_)
* chore(autoware_lidar_frnet): remove cudnn dependency (`#11888 <https://github.com/autowarefoundation/autoware_universe/issues/11888>`_)
* chore: add maintainer of PTv3, FRNet, and CalibrationStatusClassifier (`#11945 <https://github.com/autowarefoundation/autoware_universe/issues/11945>`_)
  * chore: update `autoware_ptv3` maintainer
  * chore: update `autoware_lidar_frnet` maintainer
  * chore: update `autoware_calibration_status_classifier` maintainer
  ---------
* Contributors: Amadeusz Szymko, Manato Hirabayashi, Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* feat(autoware_lidar_frnet): integrate cuda blackboard into point clooud process (`#11677 <https://github.com/autowarefoundation/autoware_universe/issues/11677>`_)
  * feat(lidar_frnet): integrate cuda_blackboard for enhanced point cloud processing
  * feat(lidar_frnet): integrate CudaBlackboard for point cloud publishing
  * feat(lidar_frnet): add CUDA remappings for point cloud outputs in launch configuration
  Co-authored-by: Amadeusz Szymko <amadeuszszymko@gmail.com>
  * fix(lidar_frnet): make point cloud layout members const for safety
  ---------
  Co-authored-by: Amadeusz Szymko <amadeusz.szymko.2@tier4.jp>
* Contributors: Kyoichi Sugahara, Ryohsuke Mitsudome

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_lidar_frnet): add FRNet for LiDAR semantic segmentation (`#10503 <https://github.com/autowarefoundation/autoware_universe/issues/10503>`_)
  * feat(autoware_lidar_frnet): add FRNet for LiDAR semantic segmentation
  * docs(autoware_lidar_frnet): style
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@gmail.com>
  * feat(autoware_lidar_frnet): store cloud layouts in minimal type
  * feat(autoware_lidar_frnet): use clear async
  * fix(autoware_lidar_frnet): correct value for cuda mem set & use clear async
  * fix(autoware_lidar_frnet): remove redundant stream sync
  * fix(autoware_lidar_frnet): avoid cuda memory allocation
  * fix(autoware_lidar_frnet): avoid cuda memory allocation (2)
  * fix(autoware_lidar_frnet): precess only output clouds with active subscribers
  * fix(autoware_lidar_frnet): atomic operation for fp precision point (x, y, z, intensity)
  * fix(autoware_lidar_frnet): explicit device stream sync for thrust
  * feat(autoware_lidar_frnet): use cub::DeviceRadixSort
  * feat(autoware_lidar_frnet): avoid host vectors
  * feat(autoware_lidar_frnet): update cuda flags
  * fix(autoware_lidar_frnet): final adjustment
  ---------
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@gmail.com>
* Contributors: Amadeusz Szymko, Ryohsuke Mitsudome
