^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_image_transport_decompressor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* test(image_transport_decompressor): move the matrix to a logic test and document the limits (`#13246 <https://github.com/autowarefoundation/autoware_universe/issues/13246>`_)
  * test(image_transport_decompressor): move the matrix to a logic test
  * move the 40-case encoding/format matrix out of the node test into a
  new test_image_transport_decompressor.cpp that calls decompress()
  directly, so it needs no ROS context
  * the node test keeps only the 3 wiring cases: publish, "encoding"
  parameter forwarding, and drop-without-publish on a decode failure
  * register the new logic test in CMakeLists.txt
  The expected values are carried over unchanged, so the move itself
  proves the relocation is faithful.
  * docs(image_transport_decompressor): document what the encoding parameter publishes
  * fill the empty Assumptions / Known limits section with a table of what
  each camera encoding ends up as, per `encoding` value
  * say in the schema what `encoding` does, instead of "The image encoding
  to use for the decompressed image"
  * test(image_transport_decompressor): cover the empty payload
  cv::imdecode reports a payload it cannot make sense of by returning an
  empty image, but throws when there is no payload at all. Only the first
  was exercised, leaving the rethrow uncovered.
  ---------
* refactor(image_transport_decompressor): extract logic part as a function (`#13235 <https://github.com/autowarefoundation/autoware_universe/issues/13235>`_)
  Move decompress() out of ImageTransportDecompressor so the decode logic
  builds and runs without a rclcpp::Node.
  * split the CMake target into ${PROJECT_NAME} for the decode logic and
  ${PROJECT_NAME}_ros for the node, which links against it. The logic
  library links nothing from rclcpp/rmw/rosidl
  * decompress() takes the compressed image and the requested encoding as
  parameters, returns the image by value, and no longer logs: a decode
  failure propagates as std::runtime_error and the node reports it with
  RCLCPP_ERROR, so the caller owns the policy for a corrupted payload
  * wrap the cv::Exception cv::imdecode throws on an empty payload, so a
  caller has one exception type to catch, with the decoder's own message
  carried over
  * drop the unreachable channel-count branches: cv::IMREAD_COLOR always
  yields three channels, so only the bgr8 case was ever taken
  * the twelve encoding-to-conversion cases become two lookup tables in
  revert_color_transformation() instead of six if blocks nested in an
  if/else
  * cv::COLOR\_* instead of the legacy CV\_* macros, which only resolve via a
  deprecated C header that cv_bridge happens to include
  The 40-case node characterization test added in `#13202 <https://github.com/autowarefoundation/autoware_universe/issues/13202>`_ is untouched and
  still passes, which is the check that the published output did not move.
  Fixing the KNOWN DEFECT rows it pins is out of scope here.
  ---------
* refactor(image_transport_decompressor): added node characterization test (`#13202 <https://github.com/autowarefoundation/autoware_universe/issues/13202>`_)
  * test(image_transport_decompressor): add node characterization test
  * renamed the sources to *_node.* ahead of the node/logic split
  * pinned the published image for every source encoding x encoding parameter,
  using the format strings compressed_image_transport actually emits. The
  compressed input is derived from the source encoding so that each row of the
  matrix stays one line
  * rows that hand the consumer something other than what the camera produced are
  marked KNOWN DEFECT: a fabricated alpha channel, a grayscale or Bayer image
  inflated to three channels, a 16-bit image cut to its upper 8 bits, and every
  malformed message
  ---------
* refactor(sensing): move node design files into each package (`#13105 <https://github.com/autowarefoundation/autoware_universe/issues/13105>`_)
* Contributors: Kazuki Komiya, Ryohsuke Mitsudome, Taekjin LEE, Takahisa Ishikawa

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(image_transport_decompressor): replace anonymous node name with static name using camera_id (`#12726 <https://github.com/autowarefoundation/autoware_universe/issues/12726>`_)
  refactor: fixed node name
* Contributors: Yutaro Kobayashi, github-actions

0.51.0 (2026-05-01)
-------------------

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

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_image_transport_decompressor): fix bugprone-branch-clone (`#9724 <https://github.com/autowarefoundation/autoware_universe/issues/9724>`_)
  fix: bugprone-error
* Contributors: Fumiya Watanabe, kobayu858

0.40.0 (2024-12-12)
-------------------
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
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, Ryohsuke Mitsudome, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
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
* Contributors: Manato Hirabayashi, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
