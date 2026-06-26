^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_traffic_light_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.52.0 (2026-06-26)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_traffic_light_visualization): retire characterization test and rename core unit test (`#12612 <https://github.com/autowarefoundation/autoware_universe/issues/12612>`_)
  * test(autoware_traffic_light_visualization): retire characterization test for TrafficLightMapVisualizerNode
  The characterization test introduced in `#12487 <https://github.com/autowarefoundation/autoware_universe/issues/12487>`_ has served its purpose
  as the safety net during the refactor series (`#12511 <https://github.com/autowarefoundation/autoware_universe/issues/12511>`_ / `#12553 <https://github.com/autowarefoundation/autoware_universe/issues/12553>`_ / `#12591 <https://github.com/autowarefoundation/autoware_universe/issues/12591>`_
  / `#12596 <https://github.com/autowarefoundation/autoware_universe/issues/12596>`_). Its scope is now fully covered by the finer-grained layers:
  - extract_bulbs unit tests: lanelet -> Bulb conversion
  - TrafficLightVisualizer core unit tests: bulb -> marker generation
  - Node smoke tests: pub/sub plumbing and map-not-received guard
  Remove the characterization test and its CMakeLists.txt entry. The
  remaining test executables run an order of magnitude faster, with no
  loss of behavioural coverage.
  * test(autoware_traffic_light_visualization): rename TrafficLightVisualizer unit test to take the package's main test slot
  With the characterization test retired, the unit test file for
  TrafficLightVisualizer is the most natural fit for the package's main
  test name. Rename it from test_traffic_light_map_visualizer_core.cpp
  to test_traffic_light_map_visualizer.cpp and update the matching
  ament_add_gtest target name in CMakeLists.txt accordingly.
  No content changes; this is a pure rename + CMake target rename.
* test(autoware_traffic_light_visualization): add extract_bulbs unit tests and node smoke test (`#12596 <https://github.com/autowarefoundation/autoware_universe/issues/12596>`_)
  test(autoware_traffic_light_visualization): add extract_bulbs unit tests and node smoke test
  - 9 unit tests for extract_bulbs (attribute filters, color resolution, multi-bulb / multi-traffic-light cases).
  - 2 Node smoke tests (happy path + detection-before-map guard).
  - Move test files into test/traffic_light_map_visualizer/ to mirror src/.
  No production code change.
  Co-authored-by: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
* test(autoware_traffic_light_visualization): add unit tests for TrafficLightVisualizer (`#12591 <https://github.com/autowarefoundation/autoware_universe/issues/12591>`_)
  * test(autoware_traffic_light_visualization): add unit tests for TrafficLightVisualizer
  After Option B refactor, TrafficLightVisualizer is independent of
  lanelet primitives, so its marker generation logic can be exercised
  directly with synthetic Bulb structures. The new test file covers
  8 cases (color mapping, silent skip, partial matches, multi-group
  aggregation, and the empty-input boundaries) without ROS spin-up.
  Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
  * test(autoware_traffic_light_visualization): inline single-use test helpers
  make_point and make_element each had only one caller (make_bulb and
  make_group respectively). Folding them into their callers keeps the
  test helpers complete in one place and removes a layer of indirection.
  Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
  * test(autoware_traffic_light_visualization): replace magic group ids with named constants
  Promotes 100, 200, 999 to test_group_id, another_test_group_id, and
  non_existent_group_id, matching the existing arbitrary_id /
  non_existent_id convention used in the characterization tests.
  Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
  * test(autoware_traffic_light_visualization): align EmptyDetection test with AAA pattern
  Extract make_detection({}) into the Arrange section so the test follows
  the same structure as the other seven cases.
  Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
  * test(autoware_traffic_light_visualization): use non_existent_group_id constant in Test 5
  The constant was already defined but unused; the test referenced the
  literal 999 directly. Switch to the named constant for consistency
  with test_group_id and another_test_group_id.
  Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
  * test(autoware_traffic_light_visualization): zero out unused bulb positions
  The position values 1.0, 2.0, 3.0 in single-bulb tests and the
  positions 1.0/2.0/3.0 keyed to the bulb id in OnlyDetectedColorsAreShown
  were never asserted. Use 0, 0, 0 uniformly to remove setup noise.
  Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
  * test(autoware_traffic_light_visualization): promote arbitrary bulb id 42 to named constant
  Match the existing arbitrary_id / non_existent_id naming convention.
  Bulb ids 1-4 in OnlyDetectedColorsAreShown and TwoGroupsBothMatched
  remain numeric since those values are intentionally distinct.
  Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
  * test(autoware_traffic_light_visualization): assert marker inherits bulb id and position
  Position drives where rviz draws the marker, so it is part of the
  observable output contract. Add a focused test that verifies both the
  id and the x/y/z coordinates flow from Bulb to Marker.
  Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
  * test(autoware_traffic_light_visualization): strengthen multi-group attribution checks
  # Conflicts:
  #	perception/autoware_traffic_light_visualization/test/test_extract_bulbs.cpp
  * test(autoware_traffic_light_visualization): silence position noise via make_bulb overload
  * test(autoware_traffic_light_visualization): cover marker header, group mix, and per-bulb position
  * test(autoware_traffic_light_visualization): drop trivial MarkerHeaderCarriesStampAndMapFrame test
  * test(autoware_traffic_light_visualization): reorder test_traffic_light_map_visualizer_core by scenario
  * test(autoware_traffic_light_visualization): drop trivial id/position assignment test
  MarkerInheritsBulbIdAndPosition only verifies that bulb.id and
  bulb.position propagate through create_bulb_marker as direct
  assignments, which is low value to test in isolation. The
  TwoGroupsBothMatchedProduceFourMarkers test already covers per-bulb id
  and position propagation, and the characterization test exercises the
  end-to-end path. Remove the redundant trivial test.
  * test(autoware_traffic_light_visualization): inline single-use helpers and document id-to-x setup in TwoGroupsBothMatchedProduceFourMarkers
  - Inline collect_marker_ids and index_markers_by_id into the only test
  that uses them. The two map/set lookups are now built in a single pass
  over the markers, matching the pattern from b631187c0 of inlining
  single-use test helpers.
  - Add a setup comment explaining the bulb id N -> x = N.0 convention so
  the position assertions are self-explanatory and the aliasing-guard
  intent is visible.
  ---------
  Co-authored-by: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
* refactor(autoware_traffic_light_visualization): introduce Bulb struct and tidy internal naming (`#12553 <https://github.com/autowarefoundation/autoware_universe/issues/12553>`_)
  Second of a planned series of incremental refactors for
  traffic_light_map_visualizer (follow-up to `#12511 <https://github.com/autowarefoundation/autoware_universe/issues/12511>`_).
  - Introduce Bulb { id, position, color } and BulbsByGroupId, and
  extract `extract_bulbs(map_traffic_lights) -> BulbsByGroupId` as a
  free function. TrafficLightVisualizer no longer holds
  lanelet::ConstPoint3d; lanelet API knowledge is confined to the
  conversion seam.
  - Extract `parse_bulb_color` helper.
  - Tidy helper signatures (pass Time by value, hoist `using` to
  namespace scope) and scope TrafficLightGroupArray / LaneletMapBin
  aliases to the node class private section.
  - Internal renames for clarity: viz_lanelet_map -> lanelet_map,
  regulatory_element -> traffic_light, traffic_lights ->
  map_traffic_lights, current_time -> stamp, plus subscription /
  publisher / callback member updates.
  - Drop noisy "Map is loaded" debug log.
  - Update copyright year to 2020-2026.
  Behavior is preserved end-to-end; the characterization test from
  `#12487 <https://github.com/autowarefoundation/autoware_universe/issues/12487>`_ acts as the safety net.
* refactor(autoware_traffic_light_visualization): split TrafficLightVisualizer class from Node and tidy internals (`#12511 <https://github.com/autowarefoundation/autoware_universe/issues/12511>`_)
  * refactor(autoware_traffic_light_visualization): extract TrafficLightVisualizer from node
  * refactor(autoware_traffic_light_visualization): improve readability of TrafficLightVisualizer
  * refactor(autoware_traffic_light_visualization): inline marker namespace literal
  * docs(autoware_traffic_light_visualization): annotate marker_lifetime_ns with 200 ms
* Contributors: Takayuki AKAMINE, github-actions

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_traffic_light_visualization): add characterization test for TrafficLightMapVisualizerNode (`#12487 <https://github.com/mitsudome-r/autoware_universe/issues/12487>`_)
  * test(autoware_traffic_light_visualization): add characterization tests for TrafficLightMapVisualizerNode
  Add characterization tests to capture the current behavior of
  TrafficLightMapVisualizerNode before refactoring. These tests serve as
  a safety net to ensure behavior is preserved during upcoming logic
  separation and code cleanup.
  * test(autoware_traffic_light_visualization): clarify map layout diagram with XZ side view
  ---------
* chore(traffic_light_recognition): add maintainer (`#12221 <https://github.com/mitsudome-r/autoware_universe/issues/12221>`_)
  add maintainer
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* Contributors: Masaki Baba, Takayuki AKAMINE, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: add missing ament_index_cpp dependency (`#11875 <https://github.com/autowarefoundation/autoware_universe/issues/11875>`_)
* fix: add cv_bridge.hpp support (`#11873 <https://github.com/autowarefoundation/autoware_universe/issues/11873>`_)
* Contributors: Mete Fatih Cırıt, Ryohsuke Mitsudome

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

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Mete Fatih Cırıt

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* chore(autoware_traffic_light_visualization): modify docs (`#10345 <https://github.com/autowarefoundation/autoware_universe/issues/10345>`_)
  * fix docs
  * style(pre-commit): autofix
  * add images of docs
  * fix path in json
  * fix typo
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
* Contributors: Hayato Mizushima, Masato Saeki, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(tier4_perception_launch): add option for new TL detector model (`#9731 <https://github.com/autowarefoundation/autoware_universe/issues/9731>`_)
  * feat: add traffic_light_detector launch
  fix: tier4 perception launch
  fix: add multi tlr detector launch
  fix: tier4 launch
  fix: tl detector launch
  fix: data director
  fix: precision int8
  chore: revert to fp16
  feat: remove occlusion and add car ped classification merger
  fix: launch for multi camera
  chore: pre-commit
  fix: update matching score
  feat: add max_iou_threshold
  feat: add occlusion unknown classifier
  * fix: tl detector launch
  * refactor: traffic_light_launch.xml
  * fix: remove tl fine detector
  * fix: refactor
  * chore: pre-commit
  * fix: cspelling check
  * fix: error after rename package
  * fix: default tl model name
  * fix: new tlr for multi cameras
  * modify args
  * style(pre-commit): autofix
  * refactor
  * add category_merger to container
  * fix args
  * run pre-commit
  ---------
  Co-authored-by: Masato Saeki <78376491+MasatoSaeki@users.noreply.github.com>
  Co-authored-by: MasatoSaeki <masato.saeki@tier4.jp>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Fumiya Watanabe, badai nguyen

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_traffic_light_visualization): fix bugprone-branch-clone (`#9668 <https://github.com/autowarefoundation/autoware_universe/issues/9668>`_)
  fix: bugprone-error
* Contributors: Fumiya Watanabe, kobayu858

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
* fix(traffic_light_roi_visualizer): show unknown results correctly (`#9467 <https://github.com/autowarefoundation/autoware_universe/issues/9467>`_)
  fix: show unknown results correctly
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(autoware_traffic_light_visualization): include opencv as system (`#9331 <https://github.com/autowarefoundation/autoware_universe/issues/9331>`_)
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Masato Saeki, Ryohsuke Mitsudome, Tao Zhong, Yukinari Hisaki, Yutaka Kondo

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
* fix(docs): fix documentation for traffic light visualization (`#8303 <https://github.com/autowarefoundation/autoware_universe/issues/8303>`_)
  fix docs traffic light visualization
* fix(autoware_traffic_light_visualization): fix to visualize correct color and shapes (`#8428 <https://github.com/autowarefoundation/autoware_universe/issues/8428>`_)
  fix(autoware_traffic_light_visualization): fix vialization to draw correct shapes
  Co-authored-by: Yi-Hsiang Fang (Vivid) <146902905+vividf@users.noreply.github.com>
* fix(traffic_light_visualization): fix funcArgNamesDifferent (`#8156 <https://github.com/autowarefoundation/autoware_universe/issues/8156>`_)
  fix:funcArgNamesDifferent
* fix(traffic_light_visualizer): remove cerr temporarily to avoid flooding logs (`#8294 <https://github.com/autowarefoundation/autoware_universe/issues/8294>`_)
  * fix(traffic_light_visualizer): remove cerr temporarily to avoid flooding logs
  * fix precommit
  * fix
  ---------
* fix(autoware_traffic_light_visualization): fix passedByValue (`#8241 <https://github.com/autowarefoundation/autoware_universe/issues/8241>`_)
  fix:passedByValue
* feat(traffic_light_roi_visualizer): add an option to use normal publisher instead of image tranport in traffic light roi visualizer (`#8157 <https://github.com/autowarefoundation/autoware_universe/issues/8157>`_)
  * apply new parameter schemes, set default parameters
  add an option to use normal publisher instead of image tranport in traffic light roi visualizer
  * small fix on default value
  ---------
* refactor(traffic_light\_*)!: add package name prefix of autoware\_ (`#8159 <https://github.com/autowarefoundation/autoware_universe/issues/8159>`_)
  * chore: rename traffic_light_fine_detector to autoware_traffic_light_fine_detector
  * chore: rename traffic_light_multi_camera_fusion to autoware_traffic_light_multi_camera_fusion
  * chore: rename traffic_light_occlusion_predictor to autoware_traffic_light_occlusion_predictor
  * chore: rename traffic_light_classifier to autoware_traffic_light_classifier
  * chore: rename traffic_light_map_based_detector to autoware_traffic_light_map_based_detector
  * chore: rename traffic_light_visualization to autoware_traffic_light_visualization
  ---------
* Contributors: Kotaro Uetake, Taekjin LEE, Yutaka Kondo, Yuxuan Liu, kminoda, kobayu858

0.26.0 (2024-04-03)
-------------------
