^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_camera_streampetr
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* fix(design): align the perception node designs with the packages they describe (`#13339 <https://github.com/autowarefoundation/autoware_universe/issues/13339>`_)
  The lanelet filter components register under the lanelet_filter:: namespace,
  and the apollo instance segmentation and transfusion nodes are built with the
  autoware\_ prefix on the executable name.
  BEVFusion, StreamPetr and LidarFRNet name param file defaults that resolve to
  nothing: the first two are missing the config/ prefix, and the three files
  LidarFRNet names are called frnet.param.yaml, ml_package_frnet_ot128.param.yaml
  and diagnostics_frnet.param.yaml.
  ElevationMapLoader pins its map_hash input with global:, which keeps the port
  out of the design graph: link_manager skips any connection whose target is a
  global input port and the exporter emits no remap. remap_target: keeps the same
  fixed topic name while letting the port take part in the graph.
* refactor(camera-streampetr): replace NMS with perception_utils::IouBevNms (`#13179 <https://github.com/autowarefoundation/autoware_universe/issues/13179>`_)
  refactor: replace NMS with perception_utils::IouBevNms
* fix(autoware_camera_streampetr): report the preprocess time with sub-millisecond precision (`#13212 <https://github.com/autowarefoundation/autoware_universe/issues/13212>`_)
  * fix(autoware_camera_streampetr): report the preprocess time with sub-millisecond precision
  The per-image preprocessing takes about 2 ms, but the data store truncated
  it through duration_cast<milliseconds> to whole milliseconds, quantizing
  away most of the signal before it reached the latency/preprocess debug
  topic and the diagnostics. Measure it as a double in milliseconds instead.
  * chore: clean up comment
  * feat(autoware_camera_streampetr): report per-camera input status diagnostics
  The node consumes five cameras but nothing outside its own log reported
  whether they are actually usable: a camera that never arrives, publishes
  an encoding the preprocessing rejects, or silently stops publishing
  mid-run leaves /diagnostics empty while the node stops inferring. The
  motivating incident was a subscription that failed to be created at all:
  every camera published normally and the sensing-side diagnostics stayed
  green, but the node received nothing -- only a consumer-side status can
  report that class of failure.
  Publish a timer-driven camera_status (period
  diagnostics.validation_callback_interval_ms) so the reporting survives
  the cameras dying. Each camera collapses into one state,
  most-specific-cause first: rejected (ERROR, dropped by input validation),
  waiting_camera_info / waiting_image (WARN, normal during start-up),
  stale (ERROR, newest frame older than diagnostics.max_image_age_ms) and
  active. Per camera it also carries the resolved input topic (the model
  index and the physical camera differ per deployment) and the image age;
  summary keys (num_waiting/num_stale/num_rejected, stalest camera,
  inter-camera stamp spread) make the worst offender readable without
  per-camera digging. Ages are clamped at zero so a rosbag loop does not
  read as a stall, and printed as fixed-point strings so epoch-sized values
  do not degrade to scientific notation.
  * feat(autoware_camera_streampetr): add a processing-time watchdog with per-stage breakdown
  Add a processing_time_status task to the diagnostics updater, mirroring
  the lidar detectors: WARN once a cycle exceeds
  diagnostics.max_allowed_processing_time_ms, escalating to ERROR when it
  stays over budget for longer than
  diagnostics.max_acceptable_consecutive_delay_ms. Timer driven, so a node
  that has stopped inferring altogether keeps reporting -- exactly when the
  per-cycle path stops running -- and 'waiting' is reported until the first
  inference completes. The stopwatch becomes a plain always-on member: the
  watchdog needs the per-cycle total even with the debug topics disabled,
  which makes every null check on it dead code.
  Beyond the thresholds shared with the other detectors, the status carries:
  - preprocess/inference/postprocess_time_ms: the same cycle's per-stage
  breakdown, latched together with the total, so an over-budget cycle can
  be localized without enabling debug_mode. Not-yet-inferred cycles
  report 'n/a' rather than 0.0, which would read as a 0 ms cycle.
  - last_frame_timestamp / last_published_timestamp: the newest published
  detection under both clocks (sensing instant vs node clock), plus their
  difference output_latency_ms (end-to-end latency including camera
  transport and decode queueing, which processing_time_ms cannot see) and
  time_since_last_publish_ms (how long since the node last produced
  objects). Observational only; the level is decided by the thresholds.
  Timestamps are fixed-point strings: the epoch-sized values degrade to
  scientific notation through the double overload.
  * refactor(autoware_camera_streampetr): split postprocessing out of the inference call
  inference_detector() ran the model and then decoded the detections inside
  the same call, so the postprocess stage was timed inside the inference
  window: the reported stage times invited computing
  total - preprocess - inference - postprocess, which goes negative. The
  postprocess Duration was also fragile -- its CUDA events are recorded on
  the stream, but bbox conversion and NMS are host work, so the number was
  only correct while the stream happened to be idle there.
  Split the network API into inference_detector() (model only, stream
  synchronized before returning) and postprocess() (bbox decode + NMS).
  The node times each with its own stopwatch window, so
  latency/inference and the new latency/postprocess topic -- previously
  the nested latency/inference/postprocess -- are disjoint, and the
  per-cycle results travel as a named InferenceResult struct instead of a
  tuple with two adjacent doubles. Since postprocess() only reads the
  head's output bindings, the camera store is unfrozen before it: the
  cameras resume as soon as the forward pass ends instead of staying
  blocked through the decode.
  Note for plots and analysis scripts: the debug topic
  latency/inference/postprocess is renamed to latency/postprocess.
  * refactor(autoware_camera_streampetr): replace the forward-time vector with named subnetwork timings
  * refactor(autoware_camera_streampetr): rename functions to snake_case
  The package was ported with Google-style camelCase and PascalCase
  function names mixed into otherwise snake_case code. The Autoware C++
  guidelines follow the ROS 2 developer guide: CamelCase for types,
  snake_case for functions, methods and variables. Rename every function
  this package owns accordingly (network helpers, Duration and Memory
  methods, ego-mask helpers, CUDA kernels and launchers, NMS and utility
  functions), plus the camelCase local variables in cuda_utils and the
  constant kMaxCameraMaskId, which becomes ALL_CAPS per the same guide.
  Purely mechanical; no behavior change. Left as-is deliberately:
  SubNetwork methods that forward 1:1 to same-named TensorRT APIs
  (enqueueV3, getNbIOTensors, getTensorShape, setTensorAddress, ...) so
  they stay greppable against the TensorRT documentation, and
  Profiler::reportLayerTime, which overrides nvinfer1::IProfiler.
  * style(pre-commit): autofix
  * refactor(autoware_camera_streampetr): address camera-status diagnostics review comments
  Rename the stalest\_* diagnostics to oldest_image\_* (clearer wording, same
  meaning: the camera that has gone longest without a new frame) and hoist
  the per-camera state strings into named constants, since external
  monitors match on them.
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* fix(autoware_camera_streampetr): fix online offline difference (`#13167 <https://github.com/autowarefoundation/autoware_universe/issues/13167>`_)
  * fix(autoware_camera_streampetr): feed the model RGB and validate image input
  The preprocessing kernel wrote the source channels straight to the model
  input with BGR mean/std, so an rgb8 camera was fed BGR. The kernel now
  takes swap_rb and maps source channel i to model channel (swap_rb ? 2-i : i)
  while writing the planar output, which is free because that write is
  scattered per channel anyway. mean/std are stored in RGB order and indexed
  by the model channel.
  update_camera_image() resolves the channel order from the message encoding
  and drops frames it cannot consume: encodings other than rgb8/bgr8, padded
  row strides, and truncated buffers all misread the densely packed upload.
  The errors are throttled since a misconfigured camera hits them every frame.
  Compressed input no longer goes through image_transport. Its compressed
  plugin calls substr(node_namespace.size()) on the unresolved base topic, so
  "~/input/cameraN/image" (21 chars) under a longer node namespace such as
  "/perception/object_recognition/detection" (40 chars) throws
  std::out_of_range and kills the node at construction (reproduced against
  ros-humble-compressed-image-transport 2.5.5). Both branches subscribe
  plainly instead; the compressed path decodes to bgr8, which is what
  cv::imdecode already produces, and lets the GPU do the R/B swap. The
  subscribed topics are unchanged, so the launch remaps still apply.
  Also report which cameras are missing in the sync warning, instead of only
  saying that something is.
  * chore: clean code
  * chore: fix
  * chore: update maintainer list
  * chore: change func naming
  * chore: fix image transport
  * chore: gpu color conversion
  * chore: clean code
  ---------
* docs: replace retired model hosting URLs with Hugging Face links (`#13204 <https://github.com/autowarefoundation/autoware_universe/issues/13204>`_)
  * docs: replace retired model hosting URLs with Hugging Face links
  The model artifacts moved from awf.ml.dev.web.auto and the
  autoware-files S3 bucket to Hugging Face repositories under the
  AutowareFoundation org (`autowarefoundation/autoware#7223 <https://github.com/autowarefoundation/autoware/issues/7223>`_). The READMEs
  still pointed manual downloads at the old hosts, and the yabloc README
  still gave wget instructions for the retired archive.
  The two dataset links on the S3 bucket stay: the bucket keeps serving
  datasets, maps and rosbags. Only the model objects are retired.
  The centerpoint v0 and v1 files were never migrated and stop being
  distributed, so their changelog rows lose the download links.
  * docs: name the ML package configs in the model download notes
  The launch files read transfusion_ml_package.param.yaml and
  ml_package_camera_streampetr.param.yaml from the model directory. The
  download notes did not name them, so a manual download missed two
  required files.
  ---------
* refactor(autoware_universe): use autoware_ament_auto_package in perception DNN packages (`#12277 <https://github.com/autowarefoundation/autoware_universe/issues/12277>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* refactor(perception): move node design files into each package (`#13104 <https://github.com/autowarefoundation/autoware_universe/issues/13104>`_)
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* Contributors: Kotaro Uetake, Mete Fatih Cırıt, Ryohsuke Mitsudome, Taekjin LEE, Vishal Chauhan, Yi-Hsiang Fang (Vivid)

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(streampetr): apply mask to input image (`#12656 <https://github.com/autowarefoundation/autoware_universe/issues/12656>`_)
  * feat(autoware_vehicle_cmd_gate): apply to CIE (`#12654 <https://github.com/autowarefoundation/autoware_universe/issues/12654>`_)
  * feat: apply to CIE
  * chore: re-trigger CI
  * chore: re-trigger CI
  ---------
  * feat(camera_streampetr): add CUDA ego mask in GPU preprocess
  Apply polygon ego masking on distorted BGR in GPU before undistortion,
  replacing separate ego_mask image topics. Includes per-ROI YAML configs
  for X2 camera9/camera10 and ego_mask node parameters.
  Co-authored-by: Cursor <cursoragent@cursor.com>
  * fix: move masking after undistortion
  * fix: polygon conig
  * fix: documents
  * fix: precommit
  * style(pre-commit): autofix
  * fix: refactor
  * fix: structure
  * fix: move polygon config to camera_streampetr.para.yaml
  ---------
  Co-authored-by: Yutaro Kobayashi <129580202+kobayu858@users.noreply.github.com>
  Co-authored-by: Cursor <cursoragent@cursor.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_camera_streampetr): refine config to separate ros2 node param and ml param (`#12826 <https://github.com/autowarefoundation/autoware_universe/issues/12826>`_)
  * feat: separate parameter from node and ml models
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(autoware_camera_streampetr): add TRAFFIC_CONE and BARRIER classes (`#12784 <https://github.com/autowarefoundation/autoware_universe/issues/12784>`_)
  * add animal and hazard classes
  * fix parameter names to TRAFFIC_CONE and BARRIER
  * relax parameter validation to allow threshold arrays longer than num_classes
  * update initialization value
  ---------
* Contributors: Masaki Baba, Tao Zhong, Yoshi Ri, github-actions

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
