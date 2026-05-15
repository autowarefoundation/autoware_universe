^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_ptv3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_ptv3): bad cmake variable syntax (`#12513 <https://github.com/mitsudome-r/autoware_universe/issues/12513>`_)
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
* feat(autoware_ptv3): add multi-type input and output & update model (`#12362 <https://github.com/mitsudome-r/autoware_universe/issues/12362>`_)
  * feat(autoware_ptv3): add multi-type input and output & update model
  * fix(autoware_ptv3): missing reference
  Co-authored-by: Kyoichi Sugahara <32741405+kyoichi-sugahara@users.noreply.github.com>
  * docs(autoware_ptv3): add troubleshooting
  ---------
  Co-authored-by: Kyoichi Sugahara <32741405+kyoichi-sugahara@users.noreply.github.com>
* feat(autoware_ptv3): cuda 12.0 build compatibility (`#12188 <https://github.com/mitsudome-r/autoware_universe/issues/12188>`_)
  * feat(autoware_ptv3): CUDA 12.0+ build compatibility
  * feat: restore Turing arch
  ---------
* Contributors: Amadeusz Szymko, Mete Fatih Cırıt, Vincent Richard, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_ptv3): update nvcc flags (`#12053 <https://github.com/autowarefoundation/autoware_universe/issues/12053>`_)
* chore(autoware_ptv3): remove cudnn dependency (`#11894 <https://github.com/autowarefoundation/autoware_universe/issues/11894>`_)
* chore: add maintainer of PTv3, FRNet, and CalibrationStatusClassifier (`#11945 <https://github.com/autowarefoundation/autoware_universe/issues/11945>`_)
  * chore: update `autoware_ptv3` maintainer
  * chore: update `autoware_lidar_frnet` maintainer
  * chore: update `autoware_calibration_status_classifier` maintainer
  ---------
* Contributors: Amadeusz Szymko, Manato Hirabayashi, Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_ptv3): implemented an inference node for ptv3 using tensorrt (`#10600 <https://github.com/autowarefoundation/autoware_universe/issues/10600>`_)
  * feat: implemented an inference node for ptv3 using tensorrt
  * chore: cspells
  * chore: schemas
  * chore: lint (line was too long)
  * chore: more schemas
  * fix: mistook the compute capabilities of edge devices
  * chore: replaced incorrect bevfusion -> ptv3
  * chore: forgot to remove unused schema
  * chore: duplicated variable
  * chore: changed package dep name
  * chore: fixed schema comment
  * chore: removed unused headers in the post process kernels
  * chore: replaced in favor of auto
  * chore: removed unused headers
  * chore: changed initialization order
  * chore: replaced 0 by nullptr
  * chore: replaced type in favor of auto
  * chore: removed redundant message
  * chore: fixed compilation due to review changes
  * fix: replaced int64 by uint64
  * chore: added more descriptive comment in the schema
  * style(autoware_ptv3): cleanup
  ---------
  Co-authored-by: Amadeusz Szymko <amadeusz.szymko.2@tier4.jp>
* Contributors: Kenzo Lobos Tsunekawa, Ryohsuke Mitsudome
