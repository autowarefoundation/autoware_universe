^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_camera_streampetr
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* feat(autoware_camera_streampetr): cuda 12.0 build compatibility (`#12181 <https://github.com/mitsudome-r/autoware_universe/issues/12181>`_)
  * feat(autoware_camera_streampetr): CUDA 12.0+ build compatibility
  * feat: restore Turing arch
  ---------
* chore(streampetr): update configs and launch file to reflect autoware artifacts (`#12134 <https://github.com/mitsudome-r/autoware_universe/issues/12134>`_)
  * update model params
  * separated model and node config files
  * style(pre-commit): autofix
  * merged launch file
  * include all params in node config file
  * style(pre-commit): autofix
  * remove default params
  * fixed typo
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_camera_streampetr): update nvcc flags (`#12046 <https://github.com/mitsudome-r/autoware_universe/issues/12046>`_)
* chore(autoware_camera_streampetr): update maintainer names (`#12106 <https://github.com/mitsudome-r/autoware_universe/issues/12106>`_)
  * Update maintaner name in streampetr
  * Update maintaner name in streampetr
  * Update maintaner name in streampetr
  ---------
* Contributors: Amadeusz Szymko, Kok Seang Tan, Mete Fatih Cırıt, Samrat Thapa, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore(stream_petr): remove invalid thrust stream policy for copy operations (`#12064 <https://github.com/autowarefoundation/autoware_universe/issues/12064>`_)
  Remove unnecessary thrust::device specification
* chore(autoware_camera_streampetr): remove cudnn dependency (`#11890 <https://github.com/autowarefoundation/autoware_universe/issues/11890>`_)
* Contributors: Amadeusz Szymko, Ryohsuke Mitsudome, Samrat Thapa

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* chore(streampetr): removed thrust stream policy (`#11800 <https://github.com/autowarefoundation/autoware_universe/issues/11800>`_)
  * removed thrust stream
  * syncronize stream before thrust
  ---------
* fix: prevent possible dangling pointer from .str().c_str() pattern (`#11609 <https://github.com/autowarefoundation/autoware_universe/issues/11609>`_)
  * Fix dangling pointer caused by the .str().c_str() pattern.
  std::stringstream::str() returns a temporary std::string,
  and taking its c_str() leads to a dangling pointer when the temporary is destroyed.
  This patch replaces such usage with a const reference of std::string variable to ensure pointer validity.
  * Revert the changes made to the functions. They should only be applied to the macros.
  ---------
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  Co-authored-by: Junya Sasaki <junya.sasaki@tier4.jp>
* feat(streampetr): class wise confidence threshold (`#11756 <https://github.com/autowarefoundation/autoware_universe/issues/11756>`_)
  * class wise threshold
  * style(pre-commit): autofix
  * add checks
  * style(pre-commit): autofix
  * prevent cuda reallocatoin
  * updated conf values
  * style(pre-commit): autofix
  * removed unused import
  * add policy for thrust
  * style(pre-commit): autofix
  * use cuda_utils helpers
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(stream_petr): use dynamic triangle filter for image downsampling   (`#11724 <https://github.com/autowarefoundation/autoware_universe/issues/11724>`_)
  * downsample with anti-aliasing
  * synchronize streams
  * remove unused changes
  * style(pre-commit): autofix
  * fixed cuda sync location
  * remove unused code
  * added optimize TODO
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Samrat Thapa, Takatoshi Kondo

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* feat(autoware_camera_streampetr): cuda based undistortion with rectification   (`#11420 <https://github.com/autowarefoundation/autoware_universe/issues/11420>`_)
  * working inference with distortion
  * removed unnecessary code
  * remove unused parameter
  * style(pre-commit): autofix
  * fixed based on comments
  * style(pre-commit): autofix
  * added unroll
  * style(pre-commit): autofix
  * comment for clarity
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_camera_streampetr): implementation of StreamPETR using tensorrt (`#11139 <https://github.com/autowarefoundation/autoware_universe/issues/11139>`_)
  * added streampetr
  * use trt_common for build and forward pass
  * style(pre-commit): autofix
  * use optional parameters
  * remove unused methods
  * style(pre-commit): autofix
  * fix lint errors
  * ament
  * style(pre-commit): autofix
  * refactor complex code
  * simplified functions
  * style(pre-commit): autofix
  * removed uncrustify
  * fix clang errors
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Samrat Thapa, Tim Clephas
