^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_traffic_light_selector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
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
* refactor(autoware_traffic_light_selector): shrink node characterization test (`#12928 <https://github.com/autowarefoundation/autoware_universe/issues/12928>`_)
  refactor(autoware_traffic_light_selector): shrink node characterization test to simple integration test
  The selection logic is now fully covered by TrafficLightSelector unit
  tests, so the node-level test only needs to verify that the node's
  subscribers, synchronizer and publisher are wired correctly. Replaced
  the four IoU-selection scenarios with a single empty-inputs case and
  removed the now-unused ROI-building helpers.
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refacotr(traffic_light_selector): extract core logic (`#12924 <https://github.com/autowarefoundation/autoware_universe/issues/12924>`_)
  * refactor(autoware_traffic_light_selector): extract TrafficLightSelector logic
  * refactor(autoware_traffic_light_selector): pass CameraInfo directly to TrafficLightSelector::select
  * test(traffic_light_selector): add unit test
  * refactor(autoware_traffic_light_selector): convert TrafficLightSelector to free function
  select() has no member state, so the class wrapper was unnecessary. Convert
  it to a free function in the autoware::traffic_light namespace and add a
  docstring, updating the node and unit test call sites accordingly.
  * test(traffic_light_selector): wire up selector unit test and simplify assertions
  Register test_traffic_light_selector.cpp as a gtest target so it actually
  runs, and assert against a full TrafficLightRoiArray with expect_same
  instead of checking individual fields.
  * test(autoware_traffic_light_selector): simplify test setup with select_helper
  Introduce a select_helper that wraps the detected/rough/expected ROI
  array construction and the select() call, and pre-build TrafficLightRoi
  fixtures so each test case reads as plain data instead of nested
  make\_* calls.
  * test(autoware_traffic_light_selector): compare rois vectors directly
  Have select_helper() return the plain rois vector and expect_same()
  take vectors, dropping the TrafficLightRoiArray wrapping that was only
  needed to satisfy the message type at comparison call sites.
  * test(autoware_traffic_light_selector): use more descriptive variable names
  * test(autoware_traffic_light_selector): split make_traffic_light_roi into make_car_traffic_light_roi and make_pedestrian_traffic_light_roi
  * test(traffic_light_selector): reduce test case
  * test(traffic_light_selector): use distinct detected/expected ROIs in MultipleExpectedRoisEachAssigned
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* test(traffic_light_selector): add integration test (`#12892 <https://github.com/autowarefoundation/autoware_universe/issues/12892>`_)
  * test(traffic_light_selector): add integration test
  * style(pre-commit): autofix
  * test(autoware_traffic_light_selector): add multi-candidate highest-IoU selection case
  * test(autoware_traffic_light_selector): extract single-output-roi assertion helper
  * refactor(autoware_traffic_light_selector): simplify publish_inputs to accept element vectors
  Accept std::vector<RegionOfInterest> and std::vector<TrafficLightRoi> directly so callers
  can pass brace-enclosed lists instead of constructing intermediate wrapper objects.
  * test(traffic_light_selector): improve readability
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, Takahisa Ishikawa

0.52.0 (2026-06-30)
-------------------

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore(perception): move perception node configuration file to each package (`#12440 <https://github.com/mitsudome-r/autoware_universe/issues/12440>`_)
  move perception node configuration file to each package
* chore(traffic_light_recognition): add maintainer (`#12221 <https://github.com/mitsudome-r/autoware_universe/issues/12221>`_)
  add maintainer
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* fix(autoware_traffic_light_selector): change to genIoU, check inside by center (`#12220 <https://github.com/mitsudome-r/autoware_universe/issues/12220>`_)
  * change to genIoU, check center inside
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Masaki Baba, Taekjin LEE, badai nguyen, github-actions

0.50.0 (2026-02-14)
-------------------

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
* chore: update traffic light packages code owner (`#10644 <https://github.com/autowarefoundation/autoware_universe/issues/10644>`_)
  chore: add Taekjin Lee as maintainer to multiple perception packages
* feat(autoware_traffic_light_selector): new matching algorithm and unit test (`#10352 <https://github.com/autowarefoundation/autoware_universe/issues/10352>`_)
  * refactor and test
  * style(pre-commit): autofix
  * add dependency
  * unnecessary dependency
  * chore
  * fix typo
  * remove change in category_merger
  * chnage var name
  * add validation shiftRoi
  * add new matching algo
  * modify unittest
  * remove unnecessary file
  * change type from uint8_t to int64_t
  * change  variable name in looping
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * use move instead of copy
  Co-authored-by: badai nguyen  <94814556+badai-nguyen@users.noreply.github.com>
  * change variable name in utils
  * apply  header file
  * to pass cppcheck
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
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
* fix: fix version
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* build(autoware_traffic_light_selector): fix missing sophus dependency (`#10141 <https://github.com/autowarefoundation/autoware_universe/issues/10141>`_)
  * build(autoware_traffic_light_selector): fix missing sophus dependency
  * fix missing cgal dependency
  ---------
* fix(autoware_traffic_light_selector): add camera_info into message_filter (`#10089 <https://github.com/autowarefoundation/autoware_universe/issues/10089>`_)
  * add mutex
  * change message filter
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(traffic_light_selector): add new node for traffic light selection (`#9721 <https://github.com/autowarefoundation/autoware_universe/issues/9721>`_)
  * feat: add traffic light selector node
  feat: add traffic ligth selector node
  * fix: add check expect roi iou
  * fix: tl selector
  * fix: launch file
  * fix: update matching score
  * fix: calc sum IOU for whole shifted image
  * fix: check inside rough roi
  * fix: check inside function
  * feat: add max_iou_threshold
  * chore: pre-commit
  * docs: add readme
  * refactor: launch file
  * docs: pre-commit
  * docs
  * chore: typo
  * refactor
  * fix: add unknown in selector
  * fix: change to GenIOU
  * feat: add debug topic
  * fix: add maintainer
  * chore: pre-commit
  * fix:cmake
  * fix: move param to yaml file
  * fix: typo
  * fix: add schema
  * fix
  * style(pre-commit): autofix
  * fix typo
  ---------
  Co-authored-by: Masato Saeki <78376491+MasatoSaeki@users.noreply.github.com>
  Co-authored-by: MasatoSaeki <masato.saeki@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Esteve Fernandez, Fumiya Watanabe, Masato Saeki, badai nguyen, 心刚

* fix: fix version
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* build(autoware_traffic_light_selector): fix missing sophus dependency (`#10141 <https://github.com/autowarefoundation/autoware_universe/issues/10141>`_)
  * build(autoware_traffic_light_selector): fix missing sophus dependency
  * fix missing cgal dependency
  ---------
* fix(autoware_traffic_light_selector): add camera_info into message_filter (`#10089 <https://github.com/autowarefoundation/autoware_universe/issues/10089>`_)
  * add mutex
  * change message filter
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(traffic_light_selector): add new node for traffic light selection (`#9721 <https://github.com/autowarefoundation/autoware_universe/issues/9721>`_)
  * feat: add traffic light selector node
  feat: add traffic ligth selector node
  * fix: add check expect roi iou
  * fix: tl selector
  * fix: launch file
  * fix: update matching score
  * fix: calc sum IOU for whole shifted image
  * fix: check inside rough roi
  * fix: check inside function
  * feat: add max_iou_threshold
  * chore: pre-commit
  * docs: add readme
  * refactor: launch file
  * docs: pre-commit
  * docs
  * chore: typo
  * refactor
  * fix: add unknown in selector
  * fix: change to GenIOU
  * feat: add debug topic
  * fix: add maintainer
  * chore: pre-commit
  * fix:cmake
  * fix: move param to yaml file
  * fix: typo
  * fix: add schema
  * fix
  * style(pre-commit): autofix
  * fix typo
  ---------
  Co-authored-by: Masato Saeki <78376491+MasatoSaeki@users.noreply.github.com>
  Co-authored-by: MasatoSaeki <masato.saeki@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Esteve Fernandez, Fumiya Watanabe, Masato Saeki, badai nguyen, 心刚
