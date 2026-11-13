^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_traffic_light_map_based_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(autoware_traffic_light_pipeline): add traffic_light_recognition node (`#13367 <https://github.com/autowarefoundation/autoware_universe/issues/13367>`_)
  * feat(autoware_traffic_light_pipeline): add traffic_light_recognition node
  Compose, for one camera, the ROS-free core logic already extracted into
  autoware_tensorrt_yolox, autoware_traffic_light_map_based_detector,
  autoware_traffic_light_selector, autoware_traffic_light_classifier and
  autoware_traffic_light_category_merger into a single rclcpp::Node, in place of the
  5-Node graph production currently launches per camera:
  map_based_detector -> whole_image_detector(yolox) -> selector
  -> car_classifier / pedestrian_classifier -> category_merger
  The fine_detection path (traffic_light_fine_detector +
  traffic_light_occlusion_predictor) is out of scope.
  The package ships a ROS-free core (TrafficLightRecognition) plus a Node adapter
  (TrafficLightRecognitionNode, registered as an rclcpp_components plugin). The core's
  constructor needs the vector map, so the Node builds it on ~/input/vector_map, which
  is also where the three TensorRT engines are built; build_only builds those engines
  without a map and exits.
  The ML artifacts are named in config/traffic_light_recognition.param.yaml relative to
  a single ml_model_path parameter -- the same split autoware_lidar_centerpoint's
  ml_package.param.yaml uses -- so the config file names no user-specific path and needs
  no launch substitution.
  * docs(autoware_traffic_light_pipeline): add launch and test instructions to README
  * fix(autoware_traffic_light_pipeline): clean up unused includes and stale test comment
  * fix(autoware_traffic_light_pipeline): skip build when autoware_tensorrt_yolox is unavailable
  autoware_tensorrt_yolox returns early and installs no headers when
  autoware_tensorrt_common is missing, so this package fails to compile on
  environments without CUDA / TensorRT such as the non-CUDA CI runners.
  Guard the package with the same early return instead.
  * feat(autoware_traffic_light_pipeline): sync image and camera_info with bounded ApproximateTime
  Image and camera_info come from the same camera driver and are expected to
  share a stamp, so ExactTime worked. ApproximateTime tolerates drivers that
  stamp the two slightly differently; with equal stamps it behaves identically,
  publishing as soon as the second message arrives.
  Its default max interval is effectively unbounded, which would silently pair
  the current image with a camera_info from up to a full queue ago whenever one
  is dropped -- and both are subscribed with best-effort SensorDataQoS, so drops
  are expected under load. A stale camera_info makes the map-based detector
  project its ROIs from the ego pose of a different instant than the image was
  captured at, misplacing every ROI. Bound the interval to 50 ms, well below one
  frame period, so such pairs are rejected.
  * fix(autoware_traffic_light_map_based_detector): validate timestamp offset range
  An inverted min/max timestamp offset silently degraded detection: the tf
  sample loop never ran, leaving only the single stamp sample, which shrank
  the rough ROI vibration margin without any error.
  * fix(autoware_traffic_light_pipeline): wait for tf before running recognition
  TrafficLightRecognition::run() looks up transforms through tf2::BufferCore,
  which cannot wait. Without the wait the map based detector returned no ROIs,
  so every detection was discarded and empty signals were published.
  * fix(autoware_traffic_light_pipeline): align min_timestamp_offset with original package
  * fix(autoware_traffic_light_pipeline): rebuild only the map based detector on vector map update
  TrafficLightRecognition was rebuilt from scratch in the vector map callback,
  which reloaded the YOLOX and the two classifier TensorRT engines even though
  none of them depends on the map.
  Construct TrafficLightRecognition once in the node constructor and feed the map
  through set_map(), which recreates only the map based detector. The construction
  failure is now reported instead of escaping the subscription callback, and the
  missing map is reported through the return value of set_route() and run().
  * refactor(autoware_traffic_light_pipeline): return tl::expected from set_route
  * feat(autoware_traffic_light_pipeline): publish exposure diagnostics
  Report the classifiers' over/under exposure flags on /diagnostics. The
  core builds the DiagnosticArray so the node adapter publishes it the same
  way as the other outputs, and a status is emitted on every processed
  frame -- including frames with no selected ROI, which is the normal case
  whenever no traffic light is in view.
  * perf(autoware_traffic_light_pipeline): skip whole image detection when no ROI is expected
  * feat(autoware_traffic_light_pipeline): make model precision and normalization configurable
  The classifiers' mean / std must match the preprocessing their model was
  trained with, and model_path is already a parameter, so these were exposed
  as parameters too. precision is a property of the deployed TensorRT engine
  rather than of the ONNX file, so it is exposed for the whole image detector
  and both classifiers as well.
  car_classifier and pedestrian_classifier now share a ClassifierModelConfig,
  which lets make_car_classifier / make_pedestrian_classifier collapse into a
  single make_classifier.
  * refactor(autoware_traffic_light_map_based_detector): remove duplicated timestamp offset validation from node
  The same min/max_timestamp_offset validation is already performed in the
  TrafficLightMapBasedDetector constructor, so the node-side check is
  redundant.
  * fix(autoware_traffic_light_pipeline): disable intra-process comm for transient_local subscriptions
  The vector map and route subscriptions need transient_local durability, which rclcpp rejects
  when intra-process communication is enabled. Loading the node into a component container with
  use_intra_process_comms:=true therefore threw from the constructor and the component failed to
  load. Disable intra-process communication for these two subscriptions explicitly.
  * chore(autoware_traffic_light_pipeline): add maintainers
  ---------
  Co-authored-by: Masaki Baba <masaki.baba.2@tier4.jp>
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
* refactor(autoware_traffic_light_map_based_detector): resolve tf transforms inside detect() via tf2::BufferCore (`#13270 <https://github.com/autowarefoundation/autoware_universe/issues/13270>`_)
  * refactor(autoware_traffic_light_map_based_detector): extract fetch_tf_map2camera_samples logic
  Extract the tf lookup and sampling logic (previously duplicated across
  node.cpp) into a single tf2::BufferCore-only helper called from
  TrafficLightMapBasedDetector::detect(). This lets detect() accept a
  tf buffer directly (tf2_ros::Buffer publicly inherits from
  tf2::BufferCore, so the node can pass its own buffer as-is) instead of
  a pre-resolved vector of transform samples, while keeping the core
  class itself free of any tf2_ros/rclcpp::Node dependency.
  - Move transform sampling window fields (min/max_timestamp_offset)
  into TrafficLightMapBasedDetectorConfig.
  - TrafficLightMapBasedDetector::detect() now takes tf2::BufferCore &
  and resolves samples internally.
  - MapBasedDetector::camera_info_callback() waits up to 0.2s for the
  transform via tf2_ros::Buffer::canTransform() before delegating to
  detect(), since tf2::BufferCore has no wait capability of its own.
  - Update unit tests to build a tf2::BufferCore (via setTransform)
  instead of constructing StampedTransform vectors directly.
  * refactor(autoware_traffic_light_map_based_detector): add const to tf_buffer parameter in detect
  * refactor(autoware_traffic_light_map_based_detector): wait for tf at latest required stamp including timestamp offset
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* Contributors: Ryohsuke Mitsudome, Takahisa Ishikawa

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_traffic_light_map_based_detector): simplify detect() and unify ROI computation (`#12565 <https://github.com/autowarefoundation/autoware_universe/issues/12565>`_)
  * refactor(autoware_traffic_light_map_based_detector): simplify detect() with helper functions
  Extract traffic light set selection and expect ROI config construction
  into small helpers so that detect() reads as orchestration logic.
  * refactor(autoware_traffic_light_map_based_detector): return visible traffic lights by value
  Replace the output parameter of get_visible_traffic_lights() with a
  return value so the function signature reflects what is computed.
  * refactor(autoware_traffic_light_map_based_detector): unify ROI computation and use std::optional
  Treat the single-transform ROI calculation as the one-element case of
  the multi-transform bounding ROI calculation, exposing a single
  get_traffic_light_roi() returning std::optional<TrafficLightRoi>.
  Promote the per-transform projection to project_traffic_light_to_roi()
  returning std::optional.
  * refactor(autoware_traffic_light_map_based_detector): replace unused visualization include with utility/query
  Drop the autoware_lanelet2_extension visualization header that is no
  longer used directly and include utility/query.hpp explicitly for the
  lanelet::utils::query helpers we actually call.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(autoware_traffic_light_map_based_detector): rename functions to snake_case (`#12551 <https://github.com/autowarefoundation/autoware_universe/issues/12551>`_)
  * refactor(autoware_traffic_light_map_based_detector): rename functions to snake_case
  Rename camelCase function names to snake_case per Autoware coding
  conventions. Test suite/case names in test_utils.cpp are aligned to
  PascalCase to match the other test suites in the package and comply
  with Google Test naming guidance.
  * style(autoware_traffic_light_map_based_detector): wrap long lines after rename
  Wrap lines that exceeded the 100-character limit after renaming
  functions to snake_case, fixing cpplint failures in CI.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* Contributors: Takahisa Ishikawa, github-actions

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_traffic_light_map_based_detector): add unit tests (`#12524 <https://github.com/mitsudome-r/autoware_universe/issues/12524>`_)
  * test(autoware_traffic_light_map_based_detector): add unit tests for TrafficLightMapBasedDetector
  Add a node-independent unit test suite that exercises the public API
  (constructor, setRoute, detect) of TrafficLightMapBasedDetector via
  plain helper functions. Covers config validation, route-less detect
  fallback, subtype/distance/angle filters, and setRoute error paths.
  Line coverage of traffic_light_map_based_detector.cpp improves from
  85.9% to 88.5%.
  * style(autoware_traffic_light_map_based_detector): rename test helper functions to snake_case
  * test(autoware_traffic_light_map_based_detector): trim redundant assertions and comments
  * refactor(autoware_traffic_light_map_based_detector): extract IDs from LaneletMapBin via query helpers
  Replace the TestMap struct with plain LaneletMapBin and add
  get_road_lanelet_ids() / get_traffic_light_ids() helpers that derive
  IDs by querying the map directly. Map creation and ID extraction are
  now independent and reusable for any LaneletMapBin source.
  * test(autoware_traffic_light_map_based_detector): add ROI pixel coordinate test mirroring node-level integration test
  Add a unit test that asserts numerical ROI pixel coordinates (rough and
  expect) using the same geometry and camera setup as the integration
  test, with the derivation kept inline for traceability. Also reorder
  existing tests and add expect_rois empty assertions in the filter-out
  cases for symmetry with rough_rois.
  * test(autoware_traffic_light_map_based_detector): simplify node test to a smoke test
  Reduce the integration test to verify only that the node publishes one
  ROI per output topic when given the full input pipeline (TF + map +
  route + camera_info). Pixel-coordinate correctness is now covered by
  the unit test, so the node test focuses on rclcpp wiring, TF lookup,
  and multi-topic coordination only.
  * refactor(autoware_traffic_light_map_based_detector): replace fixture with free helpers in node smoke test
  Drop the test fixture in favor of free helper functions for data
  construction and a templated spin_until helper for the wait loop.
  Also drop the route publish (detect() falls back to all map traffic
  lights), shorten the post-map spin to 100 ms, and inline rclcpp
  init/shutdown. The test is now a single ~50-line TEST that reads
  top-to-bottom.
  * docs(autoware_traffic_light_map_based_detector): add ASCII art of the test lanelet layout
  Add a top-down (X-Y plane) ASCII diagram above make_test_map() in both
  the unit test and the node-level integration test so readers can see
  the road and traffic-light geometry without re-deriving it from the
  point coordinates.
  * style(pre-commit): autofix
  * refactor(autoware_traffic_light_map_based_detector): parameterize camera pose helper with rotation angle
  Replace make_default_camera_pose() with make_camera_pose(rotation_angle_deg)
  so the angle-range test can construct the rotated pose directly via
  make_camera_pose(90.0) instead of inlining quaternion math.
  * refactor(autoware_traffic_light_map_based_detector): inline tf samples vector at each test call site
  Drop the make_tf_samples() helper and construct the
  std::vector<StampedTransform> directly with brace-init at each test, so
  each test arranges its inputs without relying on a one-line wrapper.
  * refactor(autoware_traffic_light_map_based_detector): arange variable declarations
  * test(autoware_traffic_light_map_based_detector): add test for wider rough ROI with yaw-varied transform samples
  * test(autoware_traffic_light_map_based_detector): document rough ROI width calculation in yaw-varied test
  Add a comment that explains how the merged rough ROI width (72) is derived
  from the per-sample ROIs at 0° and 5°, so future readers can verify the
  expected values without re-deriving them.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* refactor(traffic_light_map_based_detector): unify detect() transform args into StampedTransform vector (`#12488 <https://github.com/mitsudome-r/autoware_universe/issues/12488>`_)
  * refactor(traffic_light_map_based_detector): unify detect() transform args into StampedTransform vector
  Replace the two separate transform arguments (vector + single) in
  detect() with a single std::vector<StampedTransform> that carries
  timestamp information. This makes the interface self-descriptive
  and eliminates ambiguity about what each transform argument represents.
  * fix(traffic_light_map_based_detector): add empty check for tf_map2camera_samples in detect()
  * refactor(traffic_light_map_based_detector): use rclcpp::Time in StampedTransform
  * fix(traffic_light_map_based_detector): avoid dangling-reference false positive on GCC13
  Receive findClosestTransform() result by value instead of const reference to
  silence -Werror=dangling-reference. tf2::Transform copy cost is negligible.
  * style(traffic_light_map_based_detector): organize space
  Co-authored-by: Masaki Baba <maumaumaumaumaumaumaumaumaumau@gmail.com>
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: Masaki Baba <maumaumaumaumaumaumaumaumaumau@gmail.com>
* refactor(traffic_light_map_based_detector): require LaneletMapBin in constructor and simplify SetRouteResult (`#12449 <https://github.com/mitsudome-r/autoware_universe/issues/12449>`_)
  * refactor(traffic_light_map_based_detector): require LaneletMapBin in TrafficLightMapBasedDetector constructor
  Move map initialization from a separate setMap() call into the constructor,
  strengthening the class invariant so that map-related data is always valid
  after construction. This eliminates null checks inside setRoute() and detect(),
  and moves the "map received?" concern to the Node layer where it belongs.
  * refactor(traffic_light_map_based_detector): simplify SetRouteResult to std::optional<SetRouteError>
  Replace LogLevel, LogMessage, and SetRouteResult with a single
  SetRouteError struct returned via std::optional. This removes the
  unused Warn log level and the logMessages() helper in the Node,
  making the error path simpler and more direct.
  * fix(traffic_light_map_based_detector): improve log message wording
  Rewrite warning log messages to use the "failed to ..." phrasing and
  fix ungrammatical wording in the route callback message.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* feat(traffic_light_map_based_detector): remove unused parameter and replace silent fallback with exeption (`#12434 <https://github.com/mitsudome-r/autoware_universe/issues/12434>`_)
  * refactor(autoware_traffic_light_map_based_detector): remove unused timestamp_sample_len parameter
  * refactor(autoware_traffic_light_map_based_detector): throw on invalid parameters instead of silent fallback
  * refactor(autoware_traffic_light_map_based_detector): move max_detection_range validation into TrafficLightMapBasedDetector constructor
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* chore(perception): move perception node configuration file to each package (`#12440 <https://github.com/mitsudome-r/autoware_universe/issues/12440>`_)
  move perception node configuration file to each package
* refactor(traffic_light_map_based_detector): extract core logic (`#12412 <https://github.com/mitsudome-r/autoware_universe/issues/12412>`_)
  * refactor(autoware_traffic_light_map_based_detector): add TrafficLightMapBasedDetector core logic class
  * refactor(autoware_traffic_light_map_based_detector): delegate node logic to TrafficLightMapBasedDetector core class
  * refactor(autoware_traffic_light_map_based_detector): construct detector and config in constructor body
  Move parameter declaration and config validation from the member
  initializer list into the constructor body so that max_detection_range
  validation (which was lost during the core logic extraction) is
  restored. Use std::unique_ptr for the detector to enable construction
  after validation.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* test(traffic_light_map_based_detector): add integration test (`#12396 <https://github.com/mitsudome-r/autoware_universe/issues/12396>`_)
  * test(traffic_light_map_based_detector): add integration test
  * test(traffic_light_map_based_detector): add geometry and ROI calculation comments to integration test
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* chore(perception): remove unused lanelet2_extension header (`#12295 <https://github.com/mitsudome-r/autoware_universe/issues/12295>`_)
  unused lanelet2_extension in perception component
* chore(traffic_light_recognition): add maintainer (`#12221 <https://github.com/mitsudome-r/autoware_universe/issues/12221>`_)
  add maintainer
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* Contributors: Masaki Baba, Sarun MUKDAPITAK, Taekjin LEE, Takahisa Ishikawa, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat!: remove ROS 2 Galactic codes (`#11905 <https://github.com/autowarefoundation/autoware_universe/issues/11905>`_)
* Contributors: Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* feat(autoware_lanelet2_utils): replace from/toBinMsg (Sensing, Visualization and Perception Component) (`#11785 <https://github.com/autowarefoundation/autoware_universe/issues/11785>`_)
  * perception component toBinMsg replacement
  * visualization component fromBinMsg replacement
  * sensing component fromBinMsg replacement
  * perception component fromBinMsg replacement
  ---------
* Contributors: Ryohsuke Mitsudome, Sarun MUKDAPITAK

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* Contributors: Ryohsuke Mitsudome, Tim Clephas

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* refactor(autoware_traffic_light_map_based_detector): split utils and add test (`#10353 <https://github.com/autowarefoundation/autoware_universe/issues/10353>`_)
  * split utils and add test
  * style(pre-commit): autofix
  * chore
  * fix pre-commit
  * change name for include guard
  * fix cmake
  * fix
  * refactor
  * fix name
  * merge similar function
  * change file name from utils to process
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: update traffic light packages code owner (`#10644 <https://github.com/autowarefoundation/autoware_universe/issues/10644>`_)
  chore: add Taekjin Lee as maintainer to multiple perception packages
* Contributors: Masato Saeki, Taekjin LEE, TaikiYamada4

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
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* chore: refine maintainer list (`#10110 <https://github.com/autowarefoundation/autoware_universe/issues/10110>`_)
  * chore: remove Miura from maintainer
  * chore: add Taekjin-san to perception_utils package maintainer
  ---------
* feat(autoware_traffic_light_map_based_detector): created the schema file,updated the readme file and deleted the default parameter in node files code (`#10107 <https://github.com/autowarefoundation/autoware_universe/issues/10107>`_)
  * feat(autoware_traffic_light_map_based_detector): Created the schema file,updated the readme file and deleted the default parameter in node files code
  * style(pre-commit): autofix
  * move params from launch to param
  * chore
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: MasatoSaeki <masato.saeki@tier4.jp>
* Contributors: Fumiya Watanabe, Shunsuke Miura, Vishal Chauhan, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore(autoware_traffic_light_map_based_detector): modify docs (`#9817 <https://github.com/autowarefoundation/autoware_universe/issues/9817>`_)
  * modify docs
  * fix title
  * fix docs
  * fix word
  * add comment about debug markers
  * fix docs
  ---------
* Contributors: Fumiya Watanabe, Masato Saeki

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
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Masato Saeki, Ryohsuke Mitsudome, Yutaka Kondo

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
* Contributors: Esteve Fernandez, Masato Saeki, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* fix(autoware_traffic_light_map_based_detector): output from screen to both (`#8411 <https://github.com/autowarefoundation/autoware_universe/issues/8411>`_)
* fix(traffic_light_map_based_detector): fix funcArgNamesDifferent (`#8155 <https://github.com/autowarefoundation/autoware_universe/issues/8155>`_)
  fix:funcArgNamesDifferent
* refactor(traffic_light\_*)!: add package name prefix of autoware\_ (`#8159 <https://github.com/autowarefoundation/autoware_universe/issues/8159>`_)
  * chore: rename traffic_light_fine_detector to autoware_traffic_light_fine_detector
  * chore: rename traffic_light_multi_camera_fusion to autoware_traffic_light_multi_camera_fusion
  * chore: rename traffic_light_occlusion_predictor to autoware_traffic_light_occlusion_predictor
  * chore: rename traffic_light_classifier to autoware_traffic_light_classifier
  * chore: rename traffic_light_map_based_detector to autoware_traffic_light_map_based_detector
  * chore: rename traffic_light_visualization to autoware_traffic_light_visualization
  ---------
* Contributors: Taekjin LEE, Yutaka Kondo, kminoda, kobayu858

0.26.0 (2024-04-03)
-------------------
