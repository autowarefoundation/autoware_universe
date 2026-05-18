^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_tensorrt_vad
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* refactor(diffusion_planner,bevformer,vad): use launch arg paths (`#12521 <https://github.com/mitsudome-r/autoware_universe/issues/12521>`_)
  refactor(autoware_diffusion_planner,autoware_tensorrt_bevformer,autoware_tensorrt_vad): use launch substitutions for artifact paths
  Move the absolute $(env HOME)/autoware_data/<model> strings out of the
  param YAMLs and have each YAML reference a launch substitution. The
  launch arg defaults still resolve to the same paths, so behaviour is
  unchanged; downstream consumers (autoware_launch) can now flip a single
  data_path default to point at the upcoming ~/autoware_data/ml_models
  layout.
  - autoware_diffusion_planner: add a data_path arg in
  diffusion_planner.launch.xml defaulting to
  $(env HOME)/autoware_data/diffusion_planner; switch the param YAML to
  $(var data_path). Drop the unused artifact_dir from the schema (left
  over after the corresponding launch arg was removed) and align the
  schema's onnx_model_path and args_path defaults with the YAML.
  - autoware_tensorrt_bevformer: rewrite model_params.engine_file and
  model_params.onnx_file defaults to use $(var data_path); the launch
  already declares the arg.
  - autoware_tensorrt_vad: rewrite model_param_path to use
  $(var model_path); the launch already declares the arg.
  Also fixes a stale v3.1 reference in the diffusion_planner README to
  match the YAML default of v4.0.
  Refs: https://github.com/autowarefoundation/autoware/issues/7068
* feat(autoware_tensorrt_vad): restore Turing arch compatibility (`#12207 <https://github.com/mitsudome-r/autoware_universe/issues/12207>`_)
* feat(autoware_tensorrt_vad): cuda 12.0 build compatibility (`#12179 <https://github.com/mitsudome-r/autoware_universe/issues/12179>`_)
  feat(autoware_tensorrt_vad): CUDA 12.0+ build compatibility
  Co-authored-by: Max-Bin <vborisw@gmail.com>
* Contributors: Amadeusz Szymko, Mete Fatih Cırıt, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_tensorrt_vad): update nvcc flags (`#12044 <https://github.com/autowarefoundation/autoware_universe/issues/12044>`_)
  Co-authored-by: Max-Bin <vborisw@gmail.com>
* fix(autoware_tensorrt_vad): remove unnecessary USE_SCOPED_HEADER_INST… (`#11988 <https://github.com/autowarefoundation/autoware_universe/issues/11988>`_)
  fix(autoware_tensorrt_vad): remove unnecessary USE_SCOPED_HEADER_INSTALL_DIR
  The USE_SCOPED_HEADER_INSTALL_DIR flag is unnecessary for this package
  because:
  - The package has no include/ directory (no public headers)
  - All headers are private in src/ directory
  - This flag only affects public header installation paths
  - It can cause build failures in certain configurations
  This flag was the only usage in autoware_universe and provided no benefit.
* chore: remove autoware_launch exec_depend from e2e/autoware_tensorrt_vad (`#11879 <https://github.com/autowarefoundation/autoware_universe/issues/11879>`_)
  Co-authored-by: Max-Bin <vborisw@gmail.com>
* fix(autoware_tensorrt_vad): remove unused function `load_object_configuration` (`#11909 <https://github.com/autowarefoundation/autoware_universe/issues/11909>`_)
* fix(autoware_tensorrt_vad): remove unused function `load_map_configuration` (`#11908 <https://github.com/autowarefoundation/autoware_universe/issues/11908>`_)
* fix(autoware_tensorrt_vad): remove unused function `load_image_normalization` (`#11910 <https://github.com/autowarefoundation/autoware_universe/issues/11910>`_)
* fix: add cv_bridge.hpp support (`#11873 <https://github.com/autowarefoundation/autoware_universe/issues/11873>`_)
* Contributors: Amadeusz Szymko, Max-Bin, Mete Fatih Cırıt, Ryohsuke Mitsudome, Ryuta Kambe, Taeseung Sohn

0.49.0 (2025-12-30)
-------------------
* chore: align version number
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* refactor: migrate autoware_tensorrt_vad to e2e directory (`#11730 <https://github.com/autowarefoundation/autoware_universe/issues/11730>`_)
  Move autoware_tensorrt_vad package from planning/ to e2e/ directory
  to align with AWF's organization of end-to-end learning-based
  components.
  Changes:
  - Created e2e/ directory for end-to-end components
  - Moved all autoware_tensorrt_vad files from planning/ to e2e/
  - Updated documentation references to reflect new location
  - Verified package builds successfully in new location
  This follows the pattern established in the AWF main repository:
  https://github.com/autowarefoundation/autoware_universe/tree/main/e2e
* Contributors: Max-Bin, Ryohsuke Mitsudome
