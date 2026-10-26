^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_traffic_light_classifier
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* build(autoware_traffic_light_classifier): move classifier headers to public include directory (`#13361 <https://github.com/autowarefoundation/autoware_universe/issues/13361>`_)
  refactor(autoware_traffic_light_classifier): publicize CNNClassifier headers
  Move classifier_interface.hpp and cnn_classifier.hpp from src/classifier/ to
  include/autoware/traffic_light_classifier/classifier/ so that other packages can
  construct a CNNClassifier against the extracted classification core, the way they
  already can against TrafficLightClassifier.
  The remaining backends (cnn_lamp_recognizer, color_classifier) stay private; only
  their include of the now-public ClassifierInterface changes. No behavior change.
* build(traffic_light_perception): split core/node CMake targets and expose public core headers (`#13325 <https://github.com/autowarefoundation/autoware_universe/issues/13325>`_)
  * refactor(autoware_traffic_light_category_merger): expose core header, split core/node CMake targets
  Move traffic_light_category_merger.hpp into
  include/autoware/traffic_light_category_merger/, and split the CMake
  target into ${PROJECT_NAME} (core) and ${PROJECT_NAME}_node (Node
  adapter linking the core). The Node header stays in src/.
  No behavior change; colcon build + colcon test pass (7 tests, 0
  failures).
  * refactor(autoware_traffic_light_selector): expose core header, split core/node CMake targets
  Move traffic_light_selector.hpp into
  include/autoware/traffic_light_selector/, keeping
  traffic_light_selector_utils.hpp private in src/ (not part of the
  public API). Split the CMake target into ${PROJECT_NAME} (core:
  selector + utils) and ${PROJECT_NAME}_node (Node adapter linking the
  core).
  No behavior change; colcon build + colcon test pass (7 tests, 0
  failures).
  * refactor(autoware_traffic_light_map_based_detector): expose core header, split core/node CMake targets
  Move traffic_light_map_based_detector.hpp into
  include/autoware/traffic_light_map_based_detector/, keeping
  traffic_light_map_based_detector_process.hpp private in src/ (not
  part of the public API). Since the public header declares functions
  taking image_geometry::PinholeCameraModel, add that include directly
  to the header instead of pulling it in transitively via the private
  process header. Split the CMake target into ${PROJECT_NAME} (core:
  detector + process) and ${PROJECT_NAME}_node (Node adapter linking
  the core).
  No behavior change; colcon build + colcon test pass (7 tests, 0
  failures).
  * refactor(autoware_tensorrt_yolox): build TrtYoloXDetector as part of the core library
  tensorrt_yolox_detector.cpp implements core detection logic and was
  being compiled into ${PROJECT_NAME}_node (the Node adapter target)
  instead of ${PROJECT_NAME} (the core library). Move it to the core
  target so the core library does not depend on the node target for
  its own logic.
  * refactor(autoware_traffic_light_classifier): expose TrafficLightClassifier as the sole public header
  - Move traffic_light_classifier.hpp to include/autoware/traffic_light_classifier/
  and forward-declare ClassifierInterface instead of including
  classifier/classifier_interface.hpp, so the classifier/*.hpp backends and
  classifier_params.hpp / traffic_light_classifier_node.hpp stay private under src/.
  - Split the CMake library target into ${PROJECT_NAME} (ROS-free classification
  core) and ${PROJECT_NAME}_node (rclcpp::Node adapter layer), mirroring the
  core/node separation already present in src/.
  - single_image_debug_inference_node and the node-level integration test now
  link against both libraries instead of recompiling every source file.
  * refactor(autoware_traffic_light_multi_camera_fusion): move headers under include/, split core/node CMake targets
  * refactor(autoware_traffic_light_arbiter): split core/node CMake targets
  * refactor(autoware_image_transport_decompressor): move core logic header to include, node header to src
  * refactor(autoware_crosswalk_traffic_light_estimator): move core logic header to include, node header to src, split core/node CMake targets
  ---------
* refactor(autoware_traffic_light_classifier): pass ROS 2 message types through classify() and make_debug_image() (`#13240 <https://github.com/autowarefoundation/autoware_universe/issues/13240>`_)
  * refactor(autoware_traffic_light_classifier): pass raw image message into classify()
  Move the sensor_msgs::msg::Image decode (cv_bridge) into
  TrafficLightClassifier::classify(), so the node passes the raw ROS
  message instead of a pre-decoded cv::Mat. classify() also now
  short-circuits on an empty ROI array without decoding, and stamps the
  result header from the input image's header, so the node no longer
  special-cases the empty-ROI response or restamps the header itself.
  * refactor(autoware_traffic_light_classifier): return debug image message from make_debug_image
  make_debug_image() now returns a sensor_msgs::msg::Image::ConstSharedPtr
  (nullptr when there is nothing to render) instead of a bare cv::Mat, taking
  the classify() Result directly so the header (previously left default-
  constructed/empty) is stamped from the classified signals. The node no
  longer needs to depend on cv_bridge to assemble the message itself.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
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
* refactor(perception): move node design files into each package (`#13104 <https://github.com/autowarefoundation/autoware_universe/issues/13104>`_)
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* refactor(autoware_traffic_light_classifier): simplify the node layer (always-on subscribe, drop the Nodelet suffix) (`#13117 <https://github.com/autowarefoundation/autoware_universe/issues/13117>`_)
  Now that the classification logic lives in the ROS-free cores/wrapper, the node is a thin I/O
  shell; this tidies it up:
  - Subscribe to image / ROI unconditionally in the constructor, removing the 100 ms timer and its
  connectCb. connectCb was a lazy-subscribe optimization that unsubscribed the inputs (and thus
  stopped inference) whenever ~/output/traffic_signals had no subscribers; always-on is simpler,
  and the callback already no-ops until the classifier is constructed. **Behavior change:** the
  node now runs the sync + classify on every synchronized input pair regardless of whether the
  output has subscribers. In steady operation a downstream always subscribes, so this matters only
  for partial / bag / development runs. (The debug image stays gated on its own subscriber count.)
  - Rename the class TrafficLightClassifierNodelet -> TrafficLightClassifierNode: it is a rclcpp
  component, not a (long-removed) nodelet. Fully in-package -- the executable name and launch files
  are unchanged (they reference the executable, not the class), and no other package references the
  plugin string.
  - snake_case the remaining camelCase names (ROS 2 style):
  - node callback: imageRoiCallback -> image_roi_callback
  - utils helpers (used by the CNN / lamp cores; casing normalized): convertColorStringtoT4 /
  convertShapeStringtoT4 / convertColorT4toString / convertShapeT4toString / isColorLabel ->
  convert_color_string_to_t4 / convert_shape_string_to_t4 / convert_color_t4_to_string /
  convert_shape_t4_to_string / is_color_label
  - single-image debug tool: onMouse / inferWithCrop -> on_mouse / infer_with_crop, and the
  toString label helper -> state_to_string (renamed rather than to_string to avoid confusion
  with std::to_string used alongside it).
  No topic / parameter / output changes.
* refactor(autoware_traffic_light_classifier): return-based classify and drop the Core suffix (`#13081 <https://github.com/autowarefoundation/autoware_universe/issues/13081>`_)
  ClassifierInterface::classify now returns std::optional<TrafficLightArray> (one signal per
  image, with traffic_light_id / type left unset) instead of filling a caller-owned array. The
  TrafficLightClassifier wrapper zips traffic_light_id / type onto the result afterwards. This
  removes the per-backend images/signals size guard; the wrapper instead enforces the one-signal-
  per-image contract once, at the single place that relies on it (the id/type association loop).
  The three cores reclaim the plain names ColorClassifier / CNNClassifier / CnnLampRecognizer
  freed when the adapters were deleted in `#13070 <https://github.com/autowarefoundation/autoware_universe/issues/13070>`_; their raw per-image method stays infer(). The
  lamp recognizer takes its traffic_light_type at construction and passes it to
  update_traffic_signals (now a parameter, no longer read off the signal), so classify leaves
  id/type unset like the other backends -- no dead write, one source of truth per layer.
  Also fixes the single-image debug tool, which passed an empty signals array and so always
  failed the old size guard. No external behavior change (topics, params, and outputs unchanged).
* refactor(autoware_traffic_light_classifier): collapse classifier adapters into the cores (`#13070 <https://github.com/autowarefoundation/autoware_universe/issues/13070>`_)
  The three ROS adapters (CNNClassifier / ColorClassifier / CnnLampRecognizer) are removed. Their
  Cores now implement ClassifierInterface directly: the interface entry point is renamed
  getTrafficSignals -> classify, and each Core's raw per-image method is renamed classify -> infer
  so classify(images, signals) can wrap infer() with the caller-signal mapping. make_debug_image is
  consolidated onto the Core (the composite and the per-image renderer are now overloads on one
  class), removing the same-named split between core and adapter.
  The node and single-image debug node construct the Cores directly; the color reconfigure handle is
  now a ColorClassifierCore (its get_config / set_config are used natively, so the adapter
  pass-throughs are gone).
  The three ROS-isolated adapter tests are retired. Their images/signals size-guard assertions are
  refolded against each core's classify(): ROS-free for color, and into the GPU-gated core fixtures
  for CNN and lamp (the lamp per-image scatter was already covered by the core's infer() test).
* refactor(autoware_traffic_light_classifier): move logging and dynamic reconfigure to the node (`#13056 <https://github.com/autowarefoundation/autoware_universe/issues/13056>`_)
  The three classifier adapters are now Node-free: their constructors take only their plain
  Config (no rclcpp::Node), and the size-guard / inference-failure paths return false without
  logging. The node's existing generic error on a null classify result covers the failure path.
* refactor(autoware_traffic_light_classifier): move debug-image publishing to the node (`#13047 <https://github.com/autowarefoundation/autoware_universe/issues/13047>`_)
  The three classifier adapters no longer own an image_transport publisher. Instead the
  ClassifierInterface gains make_debug_image(images), which returns one composite RGB debug
  frame built from the most recent getTrafficSignals call; the node owns a single publisher on
  ~/output/debug/image and publishes only when a consumer is attached. TrafficLightClassifier
  exposes the classified crops via Result::roi_images and forwards make_debug_image.
* refactor(autoware_traffic_light_classifier): declare classifier params in the node (`#13041 <https://github.com/autowarefoundation/autoware_universe/issues/13041>`_)
  Move ROS parameter declaration for the three classifiers out of their adapter implementations into a new node-side classifier_params translation unit.
  Each adapter constructor now takes its plain Config, built by the node via  declare_hsv/cnn/lamp_config, so the classifier files nolonger call declare_parameter or read the label file.
* refactor(autoware_traffic_light_classifier): remove dead code and clean up naming in CnnLampRecognizer (`#13019 <https://github.com/autowarefoundation/autoware_universe/issues/13019>`_)
  Naming cleanup and dead-code removal for CnnLampRecognizer. No behavior change.
  - Remove unreachable `case Shape::PED` in `update_traffic_signals` (the
  pedestrian guard always `continue`s for PED; PED→CIRCLE mapping is unchanged).
  - Rename file-local constants and anonymous-namespace helpers to ROS 2
  snake_case (e.g. `debug_image_width`, `angle_to_arrow_direction`, `get_2d_iou`,
  `run_nms`, `convert_bbox_info_to_lamp_element`, `overlap_1d`).
  - Localize the `pi` constant into its only user, `angle_to_arrow_direction`.
  - Rename `CnnLampRecognizerCore` methods to snake_case (`update_traffic_signals`,
  `do_inference`, `decode_tlr_output`). The core is interface-free, so it owns
  its naming; the adapter's `getTrafficSignals` override stays camelCase to match
  `ClassifierInterface`.
  - Rename the debug helper to `make_debug_image` and make it non-destructive
  (`const cv::Mat &` in, returns a new image), matching the ColorClassifierCore /
  CNNClassifierCore family; drop the caller's redundant `clone()`.
  - Replace the cryptic `MsgTE` alias with a `using` for `TrafficLightElement`.
  - Rename `BBoxInfo` fields to `class_id` / `sub_class_id`.
* test(autoware_traffic_light_classifier): add CnnLampRecognizer adapter test and retire characterization (`#13018 <https://github.com/autowarefoundation/autoware_universe/issues/13018>`_)
  Mirror the CNN classifier test layout for the lamp recognizer now that the
  Node-free CnnLampRecognizerCore split has landed:
  - Add test_cnn_lamp_recognizer_adapter.cpp (GPU-gated, ROS-isolated) covering
  the adapter-only concerns that cannot move to the core: the images/signals
  size guard and the per-image -> per-signal-slot scatter of getTrafficSignals.
  Mirrors test_cnn_classifier_adapter.cpp.
  - Retire the real-model characterization test: the ROS-free CnnLampRecognizerCore
  unit test (`#13001 <https://github.com/autowarefoundation/autoware_universe/issues/13001>`_) plus this adapter test now cover its responsibilities, and
  dropping it removes the brittle live-model output pins (AMBER/GREEN).
  - Rename the core test to the canonical test_cnn_lamp_recognizer.cpp (drop the
  _core suffix), matching the CNN naming (unsuffixed = core, _adapter = adapter),
  and switch it to ament_auto_add_gtest since it is ROS-free.
  Production code is unchanged.
* test(autoware_traffic_light_classifier): add CnnLampRecognizerCore unit tests (`#13001 <https://github.com/autowarefoundation/autoware_universe/issues/13001>`_)
  Cover the Node-free core's static helpers model-free (updateTrafficSignals
  output-list contracts and per-element mapping, outputDebugImage geometry)
  and add a GPU-gated classify() fixture that self-skips when the model or a
  usable GPU is missing.
* refactor(autoware_traffic_light_classifier): extract a Node-free CnnLampRecognizerCore (`#12993 <https://github.com/autowarefoundation/autoware_universe/issues/12993>`_)
  Split CnnLampRecognizer into a Node-free CnnLampRecognizerCore plus a thin
  CnnLampRecognizer : ClassifierInterface ROS adapter, mirroring the CNNClassifier
  core/adapter shape (`#12967 <https://github.com/autowarefoundation/autoware_universe/issues/12967>`_). Both classes stay in cnn_lamp_recognizer.{hpp,cpp}.
  - CnnLampRecognizerCore owns the TensorRT engine, preprocess / doInference /
  decodeTlrOutput, NMS and dedup. classify(images) returns
  DetectionResult{lamps_per_image, success}; updateTrafficSignals and
  outputDebugImage become static core helpers. It references no rclcpp / node /
  image_transport.
  - CnnLampRecognizer keeps only node_ptr\_, image_pub\_, and core\_. declare_lamp_config
  reads the ROS parameters and narrows anchors double->float; the anchors-size
  validation and bbox_offset derivation move into the core ctor so a
  directly-constructed core is correct without the ROS parameter path.
  - On inference failure the caller's traffic_signals is left untouched
  (all-or-nothing) instead of partially mutated. This mirrors the CNN core
  extraction (`#12967 <https://github.com/autowarefoundation/autoware_universe/issues/12967>`_); doInference failures are infrastructure-level
  (engine / binding / launch / CUDA), not data-dependent, so no partial-success
  state exists.
  - doInference no longer logs setInputShape failures directly; the single
  adapter-level "inference failed" error covers it.
  The public API (CnnLampRecognizer(rclcpp::Node*) + getTrafficSignals) is unchanged,
  so the existing test_cnn_lamp_recognizer characterization test (`#12974 <https://github.com/autowarefoundation/autoware_universe/issues/12974>`_) guards
  behavior unchanged.
* test(autoware_traffic_light_classifier): add CnnLampRecognizer characterization test (`#12974 <https://github.com/autowarefoundation/autoware_universe/issues/12974>`_)
  * test(autoware_traffic_light_classifier): add CnnLampRecognizer characterization test
  Pin the current end-to-end behavior of CnnLampRecognizer::getTrafficSignals
  against the real lamp-recognizer TensorRT model, as a safety net before the
  planned Node-free core/adapter split. Follows the sibling test_cnn_classifier.cpp:
  real-model characterization, self-skip when the GPU or model is unavailable, and
  coarse assertions (argmax color/shape pinned exactly, confidence only bounded).
  The green test crops do not all decode as green -- the assertions pin the
  observed per-exposure output (normal -> amber circle, weak/medium -> green circle,
  strong -> no detection -> single UNKNOWN element), locking in the pipeline so the
  upcoming split can be shown to preserve it.
  * test(autoware_traffic_light_classifier): silence cspell warning for comlops model name
  Add an inline `cspell:ignore comlops` directive, matching the existing
  convention in the package's launch XML files, to avoid the
  spell-check-differential warning on the lamp recognizer model filename.
  Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
  ---------
  Co-authored-by: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
* test(autoware_traffic_light_classifier): add CNN adapter size-guard test (`#12976 <https://github.com/autowarefoundation/autoware_universe/issues/12976>`_)
  Follow-up to `#12975 <https://github.com/autowarefoundation/autoware_universe/issues/12975>`_. Add the ROS adapter test for CNNClassifier,
  restoring the getTrafficSignals size-guard coverage left out of scope
  when `#12975 <https://github.com/autowarefoundation/autoware_universe/issues/12975>`_ made the CNN core test Node-free. GPU-gated (TensorRT engine
  build); self-skips via GTEST_SKIP without a GPU/model.
* test(autoware_traffic_light_classifier): make CNN classifier core test ROS-free (`#12975 <https://github.com/autowarefoundation/autoware_universe/issues/12975>`_)
  Rewrite CNN classifier core tests to remove the ROS layer, mirroring the color classifier's test structure to eliminate RMW/DDS flakiness.
  - Direct Core Testing: Modified `test_cnn_classifier.cpp` to test `CNNClassifierCore::classify()` directly via `CNNConfig` without `rclcpp::Node`. (Retained GPU-gated self-skipping).
  - File Consolidation: Merged static-helper tests from `test_cnn_classifier_core.cpp` into `test_cnn_classifier.cpp` for complete core coverage in a single file.
  - Streamlined Classification: Consolidated tests into two GPU-gated checks (single-image and batch), removing redundant dimming-level variants.
  - Robust Assertions (`#12940 <https://github.com/autowarefoundation/autoware_universe/issues/12940>`_): Shifted assertions to verify only the model's stable output contract (counts, confidence range, consistency). Dropped strict color/shape label pinning to prevent test fragility during model updates (label mapping remains covered by model-free `decode_label` tests).
  - Deferred: The node-dependent `getTrafficSignals` size guard test is left for a follow-up ROS adapter test.
* refactor(autoware_traffic_light_classifier): extract a Node-free CNNClassifierCore (`#12967 <https://github.com/autowarefoundation/autoware_universe/issues/12967>`_)
  Split CNNClassifier into a Node-free CNNClassifierCore and a thin ROS
  adapter, mirroring the ColorClassifierCore split. The core depends only on
  TensorRT + OpenCV + tier4_perception_msgs (no rclcpp, image_transport,
  cv_bridge, or logging) and is constructed from a plain CNNConfig, so its
  label-decode and debug-image helpers can be exercised without a node.
  The adapter keeps the ROS-facing concerns (parameter declaration, label-file
  reading, ~/output/debug/image publishing, logging, the images/signals size
  guard, and element merging that preserves upstream traffic-light id/type) and
  keeps CNNClassifier's public API unchanged.
  Construction now fails fast: a missing label file and a wrong-size mean/std
  throw instead of leaving a half-constructed classifier that would fault at
  inference time (both were latent silent-failure paths before).
  Add ROS-free unit tests for the static core helpers (decode_label branch
  coverage + make_debug_image geometry); the GPU characterization test is kept
  unchanged and still passes, confirming behavior is preserved.
* test(autoware_traffic_light_classifier): add CNN classifier characterization test (`#12940 <https://github.com/autowarefoundation/autoware_universe/issues/12940>`_)
  Pin the current end-to-end behavior of CNNClassifier as a safety net before splitting cnn_classifier.{hpp,cpp} into a Node-free core and a ROS adapter.
  It drives the real MobileNet-v2 model (downloaded under autoware_data) through getTrafficSignals over the green ROI crops in test/test_data, self-skips (GTEST_SKIP) when the GPU or model is unavailable, and is gated in CMake behind TRT_AVAIL AND CUDA_AVAIL. No production code is changed.
* refactor(autoware_traffic_light_classifier): clean up ColorClassifier config handling and dead code (`#12941 <https://github.com/autowarefoundation/autoware_universe/issues/12941>`_)
  * Remove dead code: Drop the unused HSV {Hue, Sat, Val} enum (a leftover from before the Node-free core split) and the unused <opencv2/highgui/highgui.hpp> include, which stays available transitively via classifier_interface.hpp for downstream consumers.
  * Build HSV thresholds once at construction: Add a declare_hsv_config() helper and initialize ColorClassifierCore directly in the constructor initializer list, so the HSV bands are built in a single pass from the declared parameters. This avoids the previous default-construct-then-set_config() sequence, which ran update_thresholds() twice.
  * Single source of HSV config: Add ColorClassifierCore::get_config() and remove the adapter's shadow HSVConfig copy. On dynamic reconfigure, the adapter reads the current config back from the core (read-modify-write) instead of maintaining a duplicate. get_config() exists solely for the adapter's parametersCallback and can be removed along with the callback once dynamic reconfigure moves to the Node.
  * Merge private sections: Combine the two separate private: sections (parametersCallback declaration and data members) into one, now that the split no longer serves a purpose.
* test(autoware_traffic_light_classifier): drive ColorClassifierCore tests ROS-free and extend coverage (`#12933 <https://github.com/autowarefoundation/autoware_universe/issues/12933>`_)
  Rework ColorClassifierCore tests to construct the core directly from an HSVConfig and assert on classify() results, with no node, parameters, or RMW involved; move the images/signals size-mismatch guard (adapter-only) into a small ROS-isolated adapter test. Drop the trivial fixture in favor of plain TEST cases with a local core per Arrange. Reword the adapter's size-mismatch comment and drop ROS framing from the core test header so it's described purely in the library's own terms. No production change.
* refactor(autoware_traffic_light_classifier): extract a Node-free ColorClassifierCore (`#12916 <https://github.com/autowarefoundation/autoware_universe/issues/12916>`_)
  Extracts a Node-free ColorClassifierCore (OpenCV + message types only) from ColorClassifier, splitting it into run_pipeline (HSV filter → binarize → denoise), classify_element (pure decision), and make_debug_image (cold path, only runs when a debug consumer is attached), with results returned via a ClassifierResult struct.
* Contributors: Mete Fatih Cırıt, Ryohsuke Mitsudome, Taekjin LEE, Takahisa Ishikawa, Takayuki AKAMINE

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_traffic_light_classifier): add ColorClassifier unit tests (`#12890 <https://github.com/autowarefoundation/autoware_universe/issues/12890>`_)
  Add unit tests for ColorClassifier covering HSV-band classification,
  out-of-band -> UNKNOWN, confidence bounds, threshold-driven classification,
  size mismatch, and the empty-batch no-op. Written in Arrange-Act-Assert.
  The classifier currently takes an rclcpp::Node *, so the tests host a node
  and prime the HSV thresholds via set_parameter(); a later commit removes
  that ROS coupling and only the construction/priming (Arrange) changes.
  Co-authored-by: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
* test(autoware_traffic_light_classifier): remove superseded node characterization tests (`#12889 <https://github.com/autowarefoundation/autoware_universe/issues/12889>`_)
  The node-level characterization tests (test_traffic_light_classifier_characteristics)
  were added as a transitional safety net while the classification logic was being
  separated from the ROS node. That separation is complete and the behavior is now
  covered by the core unit tests and the node integration tests, so the
  characterization tests are redundant and are removed.
  Co-authored-by: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
* test(autoware_traffic_light_classifier): add core unit and node integration tests (`#12882 <https://github.com/autowarefoundation/autoware_universe/issues/12882>`_)
  Add test coverage for the traffic light classifier at two layers:
  * `test_traffic_light_classifier.cpp`: ROS-free unit tests driving `TrafficLightClassifier::classify()` directly with a `FakeClassifier` backend — pinning per-ROI orchestration (type filtering, zero-sized→UNKNOWN append, output ordering, crop geometry, exposure overwrite, backend-failure early return).
  * `test_traffic_light_classifier_integration.cpp`: node-level tests for the topic-driven pub/sub path and diagnostics.
  Both are registered in `CMakeLists.txt` (ROS-free gtest for the core, ros-isolated gtest for the node). The integration test includes cv_bridge/OpenCV directly rather than via transitive node-header includes.
* refactor(autoware_traffic_light_classifier): extract classification logic from ROS node (`#12851 <https://github.com/autowarefoundation/autoware_universe/issues/12851>`_)
  Move the per-ROI orchestration (type filtering, exposure detection, crop and
  classify, UNKNOWN handling) out of TrafficLightClassifierNodelet::imageRoiCallback
  into a ROS-free TrafficLightClassifier class. The node becomes a thin adapter that
  handles I/O (params, pub/sub, cv_bridge, diagnostics) and delegates classification.
  Behavior is preserved; the existing characterization test pins it as the safety net.
* test(autoware_traffic_light_classifier): add characterization test for decoupling the node and the logic (`#12840 <https://github.com/autowarefoundation/autoware_universe/issues/12840>`_)
  Introduces a node-level characterization test suite for `TrafficLightClassifierNodelet::imageRoiCallback` to serve as a safety net ahead of the node/logic decoupling refactor.
  **Key Changes:**
  * **Behavior Pinned:** Covers ROI filtering, zero-size handling, exposure overwrites, ID propagation, and diagnostics.
  * **Hardware & CI Robustness:** Uses a CPU-only HSV backend (no CUDA/TensorRT required) and runs via `ament_add_ros_isolated_gtest` for stable execution in CI.
  * **Improved Design:** Replaces PNG fixtures with declarative synthetic images and explicitly documents uncharacterized scopes.
* Contributors: Takayuki AKAMINE, github-actions

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
* feat(traffic_light_classifier): add classifier_type parameter to TrafficLightClassifierCar and TrafficLightClassifierPedestrian nodes (`#12490 <https://github.com/mitsudome-r/autoware_universe/issues/12490>`_)
  - Introduced a new parameter `classifier_type` to both TrafficLightClassifierCar and TrafficLightClassifierPedestrian configurations.
  - The parameter allows selection between different classifier types: 0 for HSVFilter, 1 for CNN, and 2 for LampRecognizer.
* feat(traffic_light_classifier): add regression architecture based classifier (`#12302 <https://github.com/mitsudome-r/autoware_universe/issues/12302>`_)
  * comlops model option adding
  remove git file
  * fix: skipping swap RB channel
  * chore: launch param
  * fix: add angle calc
  * fix angle
  * fix: launch
  * fix: NMS
  * draw detected element into debug image
  * fix: postprocess
  * docs
  * fix: remaping
  * refactor
  * fix: ped classifier
  * revert launch
  * fix: empty bug
  * fix: ped
  * fix: ped mode
  * revert unintended chanage tensorrt_commom
  * change to genIoU, check center inside
  * fix: remap
  * fix: angchors values
  * add roi expand
  * Revert "fix: angchors values"
  This reverts commit fde570c4ecb912d3625133f806dd6c00ea4aa132.
  * refix anchors
  * refactor: classifier_type param
  * refactor
  refactor
  * style(pre-commit): autofix
  * refactor
  * typo
  * fix: debug image
  * style(pre-commit): autofix
  * refactor
  * refactor debug image
  * fix: launch for new model
  * spelling
  * fix docs
  * refactor
  * refactor: arch parameters
  * refactor: mv model arch param to config file
  * rename ml param
  * refactor: remove unneccesary param
  * fix rename func and args
  * rename func
  * replace pi
  * refactor
  * refactor
  * rename func
  * typo
  * ci fix
  * ix: lamp override bug
  * fix: add fail-safe
  * add sanity check
  * fix: ci
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* chore(perception): move perception node configuration file to each package (`#12440 <https://github.com/mitsudome-r/autoware_universe/issues/12440>`_)
  move perception node configuration file to each package
* chore(traffic_light_recognition): add maintainer (`#12221 <https://github.com/mitsudome-r/autoware_universe/issues/12221>`_)
  add maintainer
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* Contributors: Masaki Baba, Mete Fatih Cırıt, Taekjin LEE, badai nguyen, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(traffic_light_classifier): add under exposure detection (`#11818 <https://github.com/autowarefoundation/autoware_universe/issues/11818>`_)
  * add under exposure detection
  * update parameter
  * add test for under exposure
  * style(pre-commit): autofix
  * change diagnostics to distinguish over and under exposure
  * change parameter
  * style(pre-commit): autofix
  * change default value
  * fix required
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* chore(autoware_traffic_light_classifier): remove cudnn dependency (`#11899 <https://github.com/autowarefoundation/autoware_universe/issues/11899>`_)
  * chore(autoware_traffic_light_classifier): remove cudnn dependency
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* fix: add missing ament_index_cpp dependency (`#11875 <https://github.com/autowarefoundation/autoware_universe/issues/11875>`_)
* Contributors: Amadeusz Szymko, Masaki Baba, Mete Fatih Cırıt, Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* refactor(autoware_traffic_light_classifier): split utils and add test (`#10633 <https://github.com/autowarefoundation/autoware_universe/issues/10633>`_)
  * first commit
  * split data convert
  * chore
  * style(pre-commit): autofix
  * move function
  * add const
  * add const
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Masato Saeki, Ryohsuke Mitsudome

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat(autoware_traffic_light_classifier): move `rclcpp::shutdown();` from child to parent to avoid `rclcpp::exceptions::RCLError` (`#11048 <https://github.com/autowarefoundation/autoware_universe/issues/11048>`_)
  move child to parent
* Contributors: Masato Saeki

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* chore: update traffic light packages code owner (`#10644 <https://github.com/autowarefoundation/autoware_universe/issues/10644>`_)
  chore: add Taekjin Lee as maintainer to multiple perception packages
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
* feat(traffic_light_classifier): update diagnostics when harsh backlight is detected (`#10218 <https://github.com/autowarefoundation/autoware_universe/issues/10218>`_)
  feat: update diagnostics when harsh backlight is detected
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
* Contributors: Esteve Fernandez, Hayato Mizushima, Kotaro Uetake, Masato Saeki, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore: refine maintainer list (`#10110 <https://github.com/autowarefoundation/autoware_universe/issues/10110>`_)
  * chore: remove Miura from maintainer
  * chore: add Taekjin-san to perception_utils package maintainer
  ---------
* feat(autoware_traffic_light_classifier): add traffic light classifier schema, README and car and ped launcher (`#10048 <https://github.com/autowarefoundation/autoware_universe/issues/10048>`_)
  * feat(autoware_traffic_light_classifier):Add traffic light classifier schema and README
  * add individual launcher
  * style(pre-commit): autofix
  * fix description
  * fix README and source code
  * separate schema in README
  * fix README
  * fix launcher
  * style(pre-commit): autofix
  * fix typo
  ---------
  Co-authored-by: MasatoSaeki <masato.saeki@tier4.jp>
  Co-authored-by: Masato Saeki <78376491+MasatoSaeki@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Shunsuke Miura, Vishal Chauhan

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore(autoware_traffic_light_classifier): modify docs (`#9819 <https://github.com/autowarefoundation/autoware_universe/issues/9819>`_)
  * modify docs
  * style(pre-commit): autofix
  * fix docs
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
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
* Contributors: Amadeusz Szymko, Fumiya Watanabe, Masato Saeki

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
* fix(autoware_traffic_light_classifier): fix clang-diagnostic-delete-abstract-non-virtual-dtor (`#9497 <https://github.com/autowarefoundation/autoware_universe/issues/9497>`_)
  fix: clang-diagnostic-delete-abstract-non-virtual-dtor
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(autoware_traffic_light*): add maintainer (`#9280 <https://github.com/autowarefoundation/autoware_universe/issues/9280>`_)
  * add fundamental commit
  * add forgot package
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* refactor(cuda_utils): prefix package and namespace with autoware (`#9171 <https://github.com/autowarefoundation/autoware_universe/issues/9171>`_)
* Contributors: Esteve Fernandez, Fumiya Watanabe, Masato Saeki, Ryohsuke Mitsudome, Yutaka Kondo, kobayu858

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(autoware_traffic_light*): add maintainer (`#9280 <https://github.com/autowarefoundation/autoware_universe/issues/9280>`_)
  * add fundamental commit
  * add forgot package
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* refactor(cuda_utils): prefix package and namespace with autoware (`#9171 <https://github.com/autowarefoundation/autoware_universe/issues/9171>`_)
* Contributors: Esteve Fernandez, Masato Saeki, Yutaka Kondo

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
* fix(traffic_light_classifier): fix traffic light monitor warning (`#8412 <https://github.com/autowarefoundation/autoware_universe/issues/8412>`_)
  fix traffic light monitor warning
* fix(autoware_traffic_light_classifier): fix passedByValue (`#8392 <https://github.com/autowarefoundation/autoware_universe/issues/8392>`_)
  fix:passedByValue
* fix(traffic_light_classifier): fix zero size roi bug (`#7608 <https://github.com/autowarefoundation/autoware_universe/issues/7608>`_)
  * fix: continue to process when input roi size is zero
  * fix: consider when roi size is zero, rois is empty
  fix
  * fix: use emplace_back instead of push_back for adding images and backlight indices
  The code changes in `traffic_light_classifier_node.cpp` modify the way images and backlight indices are added to the respective vectors. Instead of using `push_back`, the code now uses `emplace_back`. This change improves performance and ensures proper object construction.
  * refactor: bring back for loop skim and output_msg filling
  * chore: refactor code to handle empty input ROIs in traffic_light_classifier_node.cpp
  * refactor: using index instead of vector length
  ---------
* fix(traffic_light_classifier): fix funcArgNamesDifferent (`#8153 <https://github.com/autowarefoundation/autoware_universe/issues/8153>`_)
  * fix:funcArgNamesDifferent
  * fix:clang format
  ---------
* refactor(traffic_light\_*)!: add package name prefix of autoware\_ (`#8159 <https://github.com/autowarefoundation/autoware_universe/issues/8159>`_)
  * chore: rename traffic_light_fine_detector to autoware_traffic_light_fine_detector
  * chore: rename traffic_light_multi_camera_fusion to autoware_traffic_light_multi_camera_fusion
  * chore: rename traffic_light_occlusion_predictor to autoware_traffic_light_occlusion_predictor
  * chore: rename traffic_light_classifier to autoware_traffic_light_classifier
  * chore: rename traffic_light_map_based_detector to autoware_traffic_light_map_based_detector
  * chore: rename traffic_light_visualization to autoware_traffic_light_visualization
  ---------
* Contributors: Amadeusz Szymko, Sho Iwasawa, Taekjin LEE, Yutaka Kondo, kobayu858

0.26.0 (2024-04-03)
-------------------
