^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_tensorrt_yolox
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_tensorrt_yolox): remove roi_overlay_segmentation_label parameter (`#12895 <https://github.com/autowarefoundation/autoware_universe/issues/12895>`_)
  The per-class roi_overlay_segmentation_label boolean flags duplicated the
  gating already provided by the ROI-to-segmentation remap: a class whose
  semseg_id is -1 (g_unmapped_label_id) is never overlaid. Drop the redundant
  RoiOverlaySemsegLabel struct, its config member, the node parameters and the
  JSON schema entries, and inline mapRoiLabel2SegLabel into overlapSegmentByRoi.
  Overlay enable/disable is now expressed solely in roi_to_semseg_label_remap.csv;
  CAR/TRUCK/BUS are remapped to -1 to preserve the previous behavior where their
  flags were false. This also removes the fragile assumption in isOverlay() that
  the model output class ordering matched the tier4 Semantic enum.
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(autoware_tensorrt_yolox): load label data in the node and pass it to the detector (`#12866 <https://github.com/autowarefoundation/autoware_universe/issues/12866>`_)
  * refactor(autoware_tensorrt_yolox): throw on GPU init failure in constructor
  Remove `TrtYoloXDetector::isGPUInitialized()` from the public API and
  instead throw `std::runtime_error` in the constructor when the GPU is
  not available, so callers do not need to check initialisation state
  after construction.
  * refactor(autoware_tensorrt_yolox): pass parsed label data instead of file paths to detector
  Replace the four label/remap/color-map file-path fields of
  TrtYoloXDetectorConfig with a single parsed LabelMaps, so the detector is
  decoupled from file I/O and constructible from in-memory data.
  - Add LabelMaps plus load_label_maps() (reads files and resolves remaps
  into ready-to-use lookup tables) and the pure build_roi_id_to_target_id_map()
  - Move file reading to the node layer; the detector receives resolved tables
  - Drop the duplicated label members and setupLabel() from the detector
  * refactor(autoware_tensorrt_yolox): hide internal label helpers from the public header
  Only load_label_maps, load_image_list and build_roi_id_to_target_id_map are
  used outside label.cpp, so move the remaining parsing helpers into an
  anonymous namespace.
  - Move trim/read_csv/file_exists/load_list_from_text_file and the
  read_label_file/load_segmentation_colormap/load_label_id_remap_file parsers
  into an anonymous namespace; simplify them to return values now that the
  name-to-id outputs are unused
  - Drop the now-unneeded <cstdint>/<optional> includes from the header
  - Rewrite test_label to exercise the public load_label_maps and the pure
  build_roi_id_to_target_id_map instead of the internal parsers
  * refactor(autoware_tensorrt_yolox): drop the skip_header_lines option
  Production CSV files always have a header, so read_csv always skipped exactly
  one line. Hard-code skipping the header line and remove the skip_header_lines
  parameter from read_csv, load_segmentation_colormap and load_label_id_remap_file.
  Also remove the now-unused test_label_remap_without_header.csv test fixture.
  * refactor(autoware_tensorrt_yolox): inline label test data as temp files
  Remove the test_label_data/ directory and write file contents directly
  in each test's Arrange section using write_temp_file(), eliminating the
  ament_index_cpp dependency and the install step for test data.
  * refactor(autoware_tensorrt_yolox): make build_roi_id_to_target_id_map a file-local helper
  Move build_roi_id_to_target_id_map into the anonymous namespace in label.cpp
  since it is only called within load_label_maps. Replace the three direct-call
  tests with a load_label_maps-level test that covers the missing-label throw path.
  * refactor(autoware_tensorrt_yolox): restore load_image_list position to minimize diff
  * refactor(autoware_tensorrt_yolox): simplify load_image_list by removing unused prefix arg
  The prefix parameter was never exercised — the sole call site passed "".
  Remove it, rename the parameter to filepath for clarity, and add a doxygen
  comment matching the style of load_label_maps. Tests are updated accordingly.
  * refactor(autoware_tensorrt_yolox): replace LabelMaps with RoiLabel and split load functions
  Replace the struct-of-arrays LabelMaps with a per-class RoiLabel struct
  (array-of-structs), remove LabelMaps entirely by inlining roi_labels and
  semseg_color_map directly into TrtYoloXDetectorConfig, and split the
  monolithic load_label_maps into two focused functions: load_label_maps
  returning std::vector<RoiLabel> and load_semseg_colormap returning
  std::vector<Colormap>.
  * refactor(autoware_tensorrt_yolox): merge load_semseg_colormap into load_segmentation_colormap
  * feat(tenrorrt_yolox): handle case for contents without valid data
  * fix(tensorrt_yolox): add bounds check in getColorizedMask to prevent out-of-bounds access
  * fix(tensorrt_yolox): reindex segmentation colormap by ID to fix out-of-order and non-contiguous entries
  Previously load_segmentation_colormap() pushed rows in CSV order, so
  out-of-order or non-contiguous IDs caused getColorizedMask() to map
  pixel values to wrong colors. Now the parsed entries are placed into a
  vector sized max_id+1 and indexed by their ID field.
  Tests added for both the out-of-order and non-contiguous cases.
  * refactor(autoware_tensorrt_yolox): rename unmapped_label_id to g_unmapped_label_id
  Rename the global constant from unmapped_label_id to g_unmapped_label_id to follow naming conventions for global variables. Update all references in header, source, and test files.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: Junya Sasaki <junya.sasaki@tier4.jp>
* refactor(autoware_tensorrt_yolox): remove unused public API (`#12835 <https://github.com/autowarefoundation/autoware_universe/issues/12835>`_)
  Remove public API symbols that have no caller in the autowarefoundation
  and tier4 organizations (verified via GitHub code search):
  - preprocess.{hpp,cu}: 6 unused exported GPU functions and their kernels
  (resize_bilinear_gpu, letterbox_gpu, nchw_to_nhwc_gpu, to_float_gpu,
  resize_bilinear_letterbox_gpu, resize_bilinear_letterbox_nhwc_to_nchw32_gpu);
  the batch/multi-scale/argmax variants that are actually used are kept
  - TrtYoloX::printProfiling and TrtYoloX::initPreprocessBuffer
  - the unused cache_dir constructor parameter (documented as "unused
  variable"); update the call sites in tensorrt_yolox_detector and
  autoware_traffic_light_fine_detector accordingly
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(tensorrt_yolox): remove dead code (`#12825 <https://github.com/autowarefoundation/autoware_universe/issues/12825>`_)
  * refactor(autoware_tensorrt_yolox): remove unused color_masks parameter
  The color_masks argument was threaded through doInference and
  feedforwardAndDecode but never read or written (both were marked
  [[maybe_unused]]). The published color mask is generated separately
  via getColorizedMask from the segmentation mask, so this parameter
  was dead. Remove it from the signatures and all call sites.
  * refactor(autoware_tensorrt_yolox): remove dead code (commented-out blocks, unused symbols)
  Remove internally-dead code that has no caller anywhere in the workspace:
  - nhwc_to_nchw_gpu and its kernel (not even declared in preprocess.hpp)
  - load_label_remap_file declaration (no definition, no caller; distinct from
  the live load_label_id_remap_file)
  - unused using-aliases NetworkIOPtr / ProfileDimsPtr / Profiler / TrtCommon
  - commented-out code blocks in the constructor and the batch preprocess fn
  * refactor(autoware_tensorrt_yolox): remove unused includes in preprocess.cu
  <stdio.h> and <stdlib.h> are no longer used (no printf/malloc/exit etc.).
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(tensorrt_yolox): unify preprocessing on GPU and remove CPU path (`#12803 <https://github.com/autowarefoundation/autoware_universe/issues/12803>`_)
  Remove the use_gpu_preprocess option and the CPU preprocessing code path so
  that preprocessing is always performed on the GPU.
  - Drop the use_gpu_preprocess constructor argument and use_gpu_preprocess\_ flag
  - Delete the CPU-only preprocess(), multiScalePreprocess() and getMaskImage()
  - Remove now-dead members input_h\_ and segmentation_out_prob_h\_
  - Drop the preprocess_on_gpu ROS parameter, config struct member, param files,
  JSON schema entries and the test override
  - Remove unused includes (<fstream>, <optional>) from the touched files
  - Update autoware_traffic_light_fine_detector for the new constructor signature
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(autoware_tensorrt_yolox): remove unused roi inference (`#12800 <https://github.com/autowarefoundation/autoware_universe/issues/12800>`_)
  * refactor(autoware_tensorrt_yolox): remove unused doInferenceWithRoi inference path
  doInferenceWithRoi, its preprocessWithRoi / preprocessWithRoiGpu helpers and the
  orphaned crop_resize_bilinear_letterbox_nhwc_to_nchw32_batch_gpu CUDA kernel were
  introduced together in `#4125 <https://github.com/autowarefoundation/autoware_universe/issues/4125>`_ (2023-07-08) but doInferenceWithRoi has never been
  called from anywhere in the repository. The consumer (traffic_light_fine_detector)
  uses doMultiScaleInference instead, so this ROI/crop inference path has been dead on
  arrival for ~3 years.
  Remove the dead path. The shared Roi host/device buffers and the multi-scale path
  (doMultiScaleInference / multiScalePreprocessGpu / multi_scale\_*_gpu kernel) are kept
  intact as they are still used.
  Verified: the package builds and all tests pass (including the GPU characterization
  test, all 3 cases run), and autoware_traffic_light_fine_detector still builds against
  the modified library.
  * chore(autoware_tensorrt_yolox): remove unused includes
  Drop includes with no symbol usage in the reviewed files:
  - preprocess.hpp: <cublas_v2.h>, <curand.h> (no BLAS/RNG is used anywhere in the
  package; only the CUDA runtime headers are required for cudaStream_t).
  - tensorrt_yolox.cpp: <assert.h>, <iomanip> (no assert() or stream manipulators are
  used).
  Build verified after removal.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(autoware_tensorrt_yolox): extract detector (`#12794 <https://github.com/autowarefoundation/autoware_universe/issues/12794>`_)
  * refactor(autoware_tensorrt_yolox): aggregate node parameters into TrtYoloXDetectorConfig
  Gather all node parameters into a single TrtYoloXDetectorConfig struct (with a
  namespace-scope RoiOverlaySemsegLabel) and store it as a config\_ member, instead
  of scattering them across constructor locals and individual member variables.
  Behavior is unchanged: the node still owns the engine and runs inference itself.
  This isolates the parameter plumbing from the later detector extraction so that
  each diff stays focused.
  * refactor(autoware_tensorrt_yolox): extract TrtYoloXDetector class within node file
  Introduce a rclcpp::Node-independent TrtYoloXDetector class (with a
  TrtYoloXDetectorResult output type) that owns the TrtYoloX engine and the label
  remapping, and have TrtYoloXNode delegate per-frame detection to it. The
  detector is defined inline in tensorrt_yolox_node.hpp/.cpp for now; a follow-up
  commit moves it to its own files without changing behavior.
  - Move setupLabel/overlapSegmentByRoi/mapRoiLabel2SegLabel/getColorizedMask and
  the inference data members out of TrtYoloXNode into TrtYoloXDetector. The node
  builds the TrtYoloXDetectorConfig locally and passes it to the detector.
  - The new definitions are placed next to the node code they were extracted from
  (detector constructor right after the node constructor, detect() right after
  onImage), and detect() keeps the body of the original onImage as-is, only
  replacing the publishes with TrtYoloXDetectorResult fields, so the diff stays
  close to a pure relocation.
  - detect() takes a sensor_msgs::msg::Image and returns an
  std::optional<TrtYoloXDetectorResult> whose fields are ROS messages with
  headers already stamped; inference failure is reported as std::nullopt and
  cv_bridge::Exception propagates to the node, keeping the original ERROR/WARN
  logging in the node.
  - setupLabel keeps its try/catch but, since the detector must not depend on
  rclcpp::Logger, rethrows the failure as a std::runtime_error annotated with
  "Label initialization failed" instead of logging via the node logger.
  * refactor(autoware_tensorrt_yolox): move TrtYoloXDetector to its own files
  Move the TrtYoloXDetector class (and TrtYoloXDetectorConfig /
  TrtYoloXDetectorResult) out of tensorrt_yolox_node.hpp/.cpp into dedicated
  tensorrt_yolox_detector.hpp/.cpp, and register the new source in CMakeLists.
  This is a pure relocation with no behavior change; tensorrt_yolox_node.hpp now
  includes tensorrt_yolox_detector.hpp and the node-only includes are trimmed.
  * refactor(autoware_tensorrt_yolox): return tl::expected from detect
  Change TrtYoloXDetector::detect to return tl::expected<TrtYoloXDetectorResult,
  std::string> instead of std::optional. This unifies the two failure channels
  (cv_bridge conversion exception and inference failure) into a single
  Error-as-Value return, so the node handles failures through one path and logs
  the cause at ERROR level.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* test(autoware_tensorrt_yolox): set HAZARD overlay parameter in node characterization test (`#12792 <https://github.com/autowarefoundation/autoware_universe/issues/12792>`_)
  The node now declares the statically-typed parameter
  roi_overlay_segmentation_label.HAZARD (added on main in `#12730 <https://github.com/autowarefoundation/autoware_universe/issues/12730>`_) without a
  default, so the integration test must provide it; otherwise node
  construction throws "must be initialized". Mirror the *.param.yaml values
  (false for the traffic-light detector, true for the segmentation model).
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* feat(autoware_tensorrt_yolox): add HAZARD label and update remap files (`#12730 <https://github.com/autowarefoundation/autoware_universe/issues/12730>`_)
  * add HAZARD label and update remapping files
  * fix remap class
  ---------
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* fix(autoware_tensorrt_yolox): correct std::clamp argument order (`#12775 <https://github.com/autowarefoundation/autoware_universe/issues/12775>`_)
  fix(autoware_tensorrt_yolox): correct std::clamp argument order in bbox clamping
  std::clamp expects (value, low, high), but the bounding-box offset
  clamping passed (0, x1/y1, cols/rows), i.e. the value and lower bound
  were swapped. As a result the top-left offset was not clamped to the
  image bounds when the detection box exceeded the right/bottom edge, and
  the low > high case is undefined behavior.
  Fix the argument order to clamp the coordinate into [0, cols/rows].
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* test(tensorrt_yolox): add integration test (`#12774 <https://github.com/autowarefoundation/autoware_universe/issues/12774>`_)
  * test(tensorrt_yolox): add integration test
  * test(tensorrt_yolox): consolidate node test assertions into helpers
  Refine the integration test added in the previous commit:
  - extract expect_objects_detected / expect_segmentation_mask_published /
  expect_color_mask_published assertion helpers
  - rename local constants to snake_case
  - drop the tautological score-threshold assertion (the node already filters
  detections by the threshold before publishing)
  * test(tensorrt_yolox): clarify integration test wait and image loading helpers
  - split publish_until into publish_until_detected_objects_received and
  wait_for_mask_messages so each test's Act reads as its intent
  - move the load-failure check into load_test_image (throws on empty)
  * test(tensorrt_yolox): ignore semseg word in spell check
  Add an inline `cspell: ignore semseg` directive to the integration test,
  matching the rest of the package, so the spell-check CI does not flag the
  "semseg" substring in the remap / color-map file names.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* Contributors: Masaki Baba, Takahisa Ishikawa, github-actions

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
