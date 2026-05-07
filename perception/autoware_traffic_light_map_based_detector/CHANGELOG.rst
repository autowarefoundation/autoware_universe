^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_traffic_light_map_based_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
