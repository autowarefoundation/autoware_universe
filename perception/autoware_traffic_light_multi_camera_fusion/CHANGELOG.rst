^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_traffic_light_multi_camera_fusion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(traffic_light_multi_camera_fusion): restore fuse() return-based output (`#12777 <https://github.com/autowarefoundation/autoware_universe/issues/12777>`_)
  Revert the fuse() input/output change introduced in `#12710 <https://github.com/autowarefoundation/autoware_universe/issues/12710>`_ so that
  MultiCameraFusion::fuse() returns the fused TrafficLightGroupArray via
  MultiCameraFusionResult again, instead of writing into an output
  reference parameter. The agnocast publisher adoption in the node is
  kept; the node now publishes through the agnocast publisher's
  publish(const &) overload.
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* fix(traffic_light_multi_camera_fusion): use `CallbackIsolatedAgnocastExecutor` (`#12776 <https://github.com/autowarefoundation/autoware_universe/issues/12776>`_)
  * fix to use CallbackIsolatedAgnocastExecutor
  * add launch for agnocast
  ---------
* test(traffic_light_multi_camera_fusion): shrink node integration test (`#12752 <https://github.com/autowarefoundation/autoware_universe/issues/12752>`_)
  * test(autoware_traffic_light_multi_camera_fusion): shrink node integration test to single case
  * test(autoware_traffic_light_multi_camera_fusion): rename test constants to snake_case
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(autoware_traffic_light_multi_camera_fusion): simplify determine_best_group_state (`#12737 <https://github.com/autowarefoundation/autoware_universe/issues/12737>`_)
  Extract helper functions to improve readability of determine_best_group_state:
  - get_best_record: consolidate the repeated highest-log-odds record lookup
  - detect_group_conflict: isolate pairwise conflict detection (detection only)
  - build_partial_matched_record: rebuild a record from matched signals
  The publish policy branching now lives entirely in the caller, and the
  single-use get_best_state_key helper is inlined into get_best_record.
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* feat(traffic_light_multi_camera_fusion): apply agnocast to publisher in `traffic_light_multi_camera_fusion` (`#12710 <https://github.com/autowarefoundation/autoware_universe/issues/12710>`_)
  * apply agnocast to publisher
  * fix cpplint
  * fix to use SingleThreadedExecutor
  ---------
* feat(traffic_light_multi_camera_fusion): apply autoware_agnocast_wrapper for CIE (`#12712 <https://github.com/autowarefoundation/autoware_universe/issues/12712>`_)
* refactor(traffic_light_multi_camera_fusion): improve readability (`#12686 <https://github.com/autowarefoundation/autoware_universe/issues/12686>`_)
  * refactor(traffic_light_multi_camera_fusion): return conflict status by value instead of member
  Replace the MultiCameraFusion::conflicted_regulatory_element_status\_ member,
  which acted as a hidden output channel written by determine_best_group_state
  and read back in fuse, with an explicit return value. determine_best_group_state
  now builds and returns a local std::vector<ConflictInfo> and is const;
  group_fusion forwards it through an output argument like unmapped_traffic_light_ids,
  and fuse writes it straight into the result. This removes the implicit
  call-ordering dependency on member state and keeps the fusion logic pure.
  * refactor(traffic_light_multi_camera_fusion): detect unmapped ids in a dedicated pass
  Stop threading the unmapped_traffic_light_ids output argument through
  group_fusion -> accumulate_group_evidence -> process_fused_record. Detecting
  unmapped ids only needs the fused_record_map and the id-to-regulatory-element
  mapping, which is independent from the log-odds evidence accumulation. Extract
  it into a free function find_unmapped_traffic_light_ids and call it once in
  fuse(), so the group-fusion call chain no longer carries an unrelated output
  argument. process_fused_record now just skips unregistered ids.
  * refactor(traffic_light_multi_camera_fusion): make multi_camera_fusion to free function
  * refactor(traffic_light_multi_camera_fusion): make log-odds update helpers free functions
  * refactor(traffic_light_multi_camera_fusion): return group fusion result by value
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(traffic_light_multi_camera_fusion): improve readability and maintainability (`#12657 <https://github.com/autowarefoundation/autoware_universe/issues/12657>`_)
  * refactor(autoware_traffic_light_multi_camera_fusion): replace SignalValidator class with free function
  SignalValidator held only a static check_conflict and an unimplemented
  merge_partial_match (dead code). Demote check_conflict to a free function
  in namespace signal_validator, drop the merge_partial_match declaration,
  and remove the conditionally-created signal_validator\_ unique_ptr member
  (which left a hidden segfault precondition when the consistency check was
  disabled). Update the unit test to call the free function and drop the now
  empty test fixture.
  * refactor(autoware_traffic_light_multi_camera_fusion): move file-local helpers out of the header
  is_unknown(StateKey) and compare_state_key_log_odds were inline functions
  in the header but are only used inside the .cpp. convert_output_msg and
  update_best_record were static member functions that touch no member state.
  Demote all four into the existing anonymous namespace in the .cpp so the
  public header only exposes the types and the class API.
  * refactor(autoware_traffic_light_multi_camera_fusion): rename ambiguous is_unknown overloads
  Two same-named is_unknown functions checked different things: one a StateKey,
  the other a TrafficLight signal. Rename them to is_state_key_unknown and
  is_signal_unknown so the check target is clear from the name alone.
  * test(traffic_light_multi_camra_fusion): refactor test for util
  * refactor(autoware_traffic_light_multi_camera_fusion): replace compare_record with has_higher_or_equal_priority
  Redesign the three-valued compare_record (-1/0/1), whose only caller used
  just its sign via '>= 0', into a boolean predicate
  has_higher_or_equal_priority(candidate, incumbent) that directly answers
  whether a candidate record should replace the current best one. The fixed
  priority order (timestamp, recognized-over-unknown, visibility, confidence)
  and tie handling are preserved.
  Also refactor test_utils.cpp for readability: extract make_signal/make_record
  helpers, split multi-assertion tests into single-scenario cases, and replace
  magic numbers with named constants.
  * refactor(autoware_traffic_light_multi_camera_fusion): replace cal_visible_score with is_fully_visible
  cal_visible_score returned int 0/1, which is effectively a boolean. Its only
  caller compared two scores to decide which record is less truncated. Replace it
  with a boolean predicate is_fully_visible(record) (true = not truncated by the
  image boundary) and simplify the visibility check in has_higher_or_equal_priority.
  The unit tests become EXPECT_TRUE/EXPECT_FALSE and the now-unused VisibleScore
  enum is removed; stale references in the integration test comments are updated.
  * refactor(autoware_traffic_light_multi_camera_fusion): clean up unnecessary includes
  - traffic_light_multi_camera_fusion_process.cpp / test_utils.cpp: include
  <rclcpp/time.hpp> instead of <rclcpp/rclcpp.hpp> (only rclcpp::Time is used)
  - signal_validator.hpp: drop <tier4_perception_msgs/msg/traffic_light.hpp>
  (no longer references TrafficLight; StateKey comes from types.hpp) and include
  the precise <traffic_light_element.hpp> in signal_validator.cpp where used
  - multi_camera_fusion.hpp: drop unused <utility>
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* test(autoware_traffic_light_multi_camera_fusion): add unit tests for core logic (`#12630 <https://github.com/autowarefoundation/autoware_universe/issues/12630>`_)
  * test(autoware_traffic_light_multi_camera_fusion): add unit tests for MultiCameraFusion
  Add unit tests covering construction, single/multi camera fusion, unmapped
  traffic light handling, message lifespan filtering, and signal consistency
  check behavior with conflict diagnostics.
  * test(autoware_traffic_light_multi_camera_fusion): remove redundant constructor tests
  MultiCameraFusionConstruction's three EXPECT_NO_THROW tests are
  implicitly covered by every MultiCameraFusionFuse test which constructs
  the same object in their Arrange step.
  * test(autoware_traffic_light_multi_camera_fusion): bundle fuse inputs via make_fusion_input
  Introduce a `FusionInput` struct and `make_fusion_input(stamp, frame_id, signal)`
  helper to remove the stamp/frame_id/traffic_light_id triple-repetition that
  appeared in every happy-path fuse call. The two edge-case tests (empty arrays
  and mismatched ROI/signal IDs) keep using the lower-level helpers directly
  since they intentionally build inputs that diverge from the bundled shape.
  * test(autoware_traffic_light_multi_camera_fusion): cover compare_record and partial-conflict paths
  Add six MultiCameraFusionFuse tests that route two observations through
  the same traffic_light_id so Stage 1 compare_record is actually invoked
  (existing tests used distinct ids and only exercised Stage 2 log-odds):
  visibility-score tiebreaker, unknown vs valid signal, same-frame_id
  timestamp tiebreaker, and confidence tiebreaker. Also cover Stage 2
  PARTIAL_CONFLICT with publish_partial_matched_signal both enabled
  (common state published) and disabled (fail-safe UNKNOWN).
  * test(autoware_traffic_light_multi_camera_fusion): drop null lanelet map case and shorten input names
  Remove the NullLaneletMap unit test (covered by node-level checks) and rename
  input_cameraN to inputN to reduce noise in multi-camera fuse tests.
  * test(autoware_traffic_light_multi_camera_fusion): extract helpers for single fused color assertions
  Add expect_single_fused_color and expect_single_fused_color_and_shape helpers
  to consolidate the repeated "size=1, elements.size=1, color (and shape)"
  assertions in the fuse tests.
  * test(autoware_traffic_light_multi_camera_fusion): extract helper for single conflict status assertion
  Add expect_single_conflict_status to consolidate the repeated
  "conflicted_regulatory_element_status size=1 + conflict_type" checks across
  the consistency/partial-conflict tests.
  * test(autoware_traffic_light_multi_camera_fusion): default stamp in make_fusion_input
  Reorder make_fusion_input so that frame_id and signal come first and stamp
  takes a default rclcpp::Time(0, 0). Tests that do not depend on timestamp
  no longer need to construct or pass a stamp; the three tests where timing
  is significant (message lifespan, newer-timestamp wins) pass stamp explicitly
  as the last argument.
  * test(autoware_traffic_light_multi_camera_fusion): extract compound signal helper
  Add make_signal_with_left_arrow to build a CIRCLE+LEFT_ARROW two-element
  signal and replace the duplicated compound signal construction in the
  partial-conflict tests.
  * test(autoware_traffic_light_multi_camera_fusion): remove comment
  * test(autoware_traffic_light_multi_camera_fusion): cover min confidence aggregation across elements
  Add a test that two cameras observe the same traffic_light_id with
  CIRCLE + LEFT_ARROW elements whose per-element confidences differ.
  The winner is decided by utils::get_min_confidence, so min-aggregation
  picks the camera whose weakest element confidence is higher; a
  max-aggregation implementation would pick the other camera and the
  output element confidences would differ.
  Extend make_signal_with_left_arrow to accept a separate confidence per
  element so the test can construct uneven signals, and introduce an
  expect_element_confidence helper to keep the assertion concise.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(traffic_light_multi_camera_fusion): extract core logic from node (`#12610 <https://github.com/autowarefoundation/autoware_universe/issues/12610>`_)
  * test(autoware_traffic_light_multi_camera_fusion): use PascalCase for test names
  Rename gtest TestSuite and TestName arguments in test_utils.cpp to
  PascalCase as required by Google Test naming conventions.
  * refactor(autoware_traffic_light_multi_camera_fusion): rename node files to multi_camera_fusion
  Pure rename in preparation for extracting the core fusion logic into a
  separate class. The Node class still lives in the renamed files; only
  include paths, include guards, and CMakeLists/test references are
  updated to keep the build green. The logical separation between Node
  and core class happens in the follow-up commit.
  * refactor(autoware_traffic_light_multi_camera_fusion): extract MultiCameraFusion logic
  Move the Node-specific code (rclcpp subscribers/publisher, diagnostics,
  parameter declarations) out of multi_camera_fusion.{cpp,hpp} into a new
  MultiCameraFusionNode in traffic_light_multi_camera_fusion_node.{cpp,hpp}.
  What remains in multi_camera_fusion.{cpp,hpp} is a pure-logic class with
  a fuse() entry point that takes ROS2 messages and returns a Result,
  without depending on rclcpp::Node or rclcpp::Logger. Warnings are
  returned via Result fields so the Node logs them (Error-as-Value).
  * fix(autoware_traffic_light_multi_camera_fusion): satisfy cppcheck functionStatic/functionConst
  Mark MultiCameraFusion::convert_output_msg and update_best_record as
  static, and update_log_odds / update_group_info_for_element as const,
  so that cppcheck --enable=all --inconclusive passes. These functions
  do not modify the object state; the qualifiers reflect existing
  behavior.
  * refactor(autoware_traffic_light_multi_camera_fusion): return unmapped traffic light IDs as data
  Previously the core MultiCameraFusion class built warning log strings
  ("Found Traffic Light Id = X which is not defined in Map") and returned
  them via MultiCameraFusionResult::warnings. Replace this with a raw
  vector of traffic light IDs (unmapped_traffic_light_ids) and move the
  string formatting to MultiCameraFusionNode, so that the core class only
  deals with data and presentation concerns stay in the Node.
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(autoware_traffic_light_multi_camera_fusion): rename functions to snake_case (`#12603 <https://github.com/autowarefoundation/autoware_universe/issues/12603>`_)
  Rename camelCase function names (member, free, and anonymous-namespace
  functions) to snake_case to follow the project naming convention.
  No behavior change.
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* test(autoware_traffic_light_multi_camera_fusion): add integration test for node (`#12581 <https://github.com/autowarefoundation/autoware_universe/issues/12581>`_)
  * test(autoware_traffic_light_multi_camera_fusion): add integration test for node
  * test(autoware_traffic_light_multi_camera_fusion): add test for confidence-based color selection
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* fix(traffic_light_multi_camera_fusion): use minimum confidence across signal elements (`#12566 <https://github.com/autowarefoundation/autoware_universe/issues/12566>`_)
  use minimum confidence across signal elements
* Contributors: Koichi Imai, Masaki Baba, Takahisa Ishikawa, atsushi yano, github-actions

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(traffic_light_multi_camera_fusion): add signal consistency check (`#12492 <https://github.com/mitsudome-r/autoware_universe/issues/12492>`_)
  * add signal validator
  * add test for signal validator
  * style(pre-commit): autofix
  * fix cppcheck issue
  * add missing header
  * fix typo
  * style(pre-commit): autofix
  * add missing const
  * fix typo
  * update readme
  * make member function as static
  * add comments
  * fix typo
  * change parameter name
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* fix(traffic-light):  fix traffic light nodes message types for system design format files (`#12446 <https://github.com/mitsudome-r/autoware_universe/issues/12446>`_)
  fix(perception): update message types for traffic light nodes to use autoware_perception_msgs
* chore(perception): move perception node configuration file to each package (`#12440 <https://github.com/mitsudome-r/autoware_universe/issues/12440>`_)
  move perception node configuration file to each package
* chore(traffic_light_recognition): add maintainer (`#12221 <https://github.com/mitsudome-r/autoware_universe/issues/12221>`_)
  add maintainer
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* Contributors: Masaki Baba, Taekjin LEE, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_traffic_light_multi_camera_fusion): use valid color and use whole elements as state (`#11989 <https://github.com/autowarefoundation/autoware_universe/issues/11989>`_)
  * feat(autoware_traffic_light_multi_camera_fusion): remove unknown and use whole elements as state
  * refactor the compareStateKeyLogOdds
  ---------
* Contributors: Masato Saeki, Ryohsuke Mitsudome

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
* fix(traffic_light_camera_fusion): change group fusion algorithm (`#11297 <https://github.com/autowarefoundation/autoware_universe/issues/11297>`_)
  * fix(traffic_light_camera_fusion): change group fusion algorithm
  * style(pre-commit): autofix
  * fix: potential array access violation
  * fix: validate func
  * feat: bayesian update
  * doc(traffic_light_camera_fusion): add bayesian method
  * chore: adding comments to variables and functions
  * doc: make simple, add figure
  * doc: fix github style
  * doc: fix mermaid error
  * style(pre-commit): autofix
  * chore: add param prior_log_odds
  * fix: modified summation function
  * feat: support color and shape
  * style(pre-commit): autofix
  * doc: update param schema
  * fix: bayesian estimation
  * style(pre-commit): autofix
  * fix: build error
  * fix: code health
  * fix: code complex
  * fix: complex branch
  * style(pre-commit): autofix
  * modify docs
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Shumpei Wakabayashi <42209144+shmpwk@users.noreply.github.com>
  Co-authored-by: Yuxuan Liu <619684051@qq.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
  Co-authored-by: Masato Saeki <78376491+MasatoSaeki@users.noreply.github.com>
  Co-authored-by: MasatoSaeki <masato.saeki@tier4.jp>
* refactor(autoware_traffic_light_multi_camera_fusion): split utils and add test  (`#10360 <https://github.com/autowarefoundation/autoware_universe/issues/10360>`_)
  * init
  * chore
  * style(pre-commit): autofix
  * add remained test
  * add include file
  * refactor
  * move variable from cpp to hpp
  * chore
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Masato Saeki, Ryohsuke Mitsudome, toki-1441

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
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore: refine maintainer list (`#10110 <https://github.com/autowarefoundation/autoware_universe/issues/10110>`_)
  * chore: remove Miura from maintainer
  * chore: add Taekjin-san to perception_utils package maintainer
  ---------
* chore(traffic_light_multi_camera_fusion): read parameters from yaml file (`#10144 <https://github.com/autowarefoundation/autoware_universe/issues/10144>`_)
  * chore(traffic_light_multi_camera_fusion): read parameters from yaml file
  * style(pre-commit): autofix
  * add all_traffic_light_camera param to launch.xml
  * fix json schema
  * remove camera namespace parameter from config file
  * revert unnecessary change
  * remove camera_namespaces from required
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore(autoware_traffic_light_multi_camera_fusion): created Schema file and updated ReadME file for parameters setting (`#9994 <https://github.com/autowarefoundation/autoware_universe/issues/9994>`_)
  * feat(autoware_traffic_light_multi_camera_fusion): Created Schema file and updated ReadME file for parameters setting
  * style(pre-commit): autofix
  * fix: updated param file , schema and node.cpp file  for traffic_light_multi_camera_fusion as per review comments
  * style(pre-commit): autofix
  * Update traffic_light_multi_camera_fusion_node.cpp
  updated code as per suggestion
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Shunsuke Miura, Tomohito ANDO, Vishal Chauhan

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore(autoware_traffic_light_multi_camera_fusion): modify docs (`#9821 <https://github.com/autowarefoundation/autoware_universe/issues/9821>`_)
  * fix docs
  * add condition
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
* feat(autoware_traffic_light_multi_camera_fusion): resolve clang-tidy error (`#9336 <https://github.com/autowarefoundation/autoware_universe/issues/9336>`_)
  * feat(autoware_traffic_light_multi_camera_fusion): resolve clang-tidy error
  * add const to argument
  ---------
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Masato Saeki, Ryohsuke Mitsudome, Yukinari Hisaki, Yutaka Kondo

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
* refactor(traffic_light\_*)!: add package name prefix of autoware\_ (`#8159 <https://github.com/autowarefoundation/autoware_universe/issues/8159>`_)
  * chore: rename traffic_light_fine_detector to autoware_traffic_light_fine_detector
  * chore: rename traffic_light_multi_camera_fusion to autoware_traffic_light_multi_camera_fusion
  * chore: rename traffic_light_occlusion_predictor to autoware_traffic_light_occlusion_predictor
  * chore: rename traffic_light_classifier to autoware_traffic_light_classifier
  * chore: rename traffic_light_map_based_detector to autoware_traffic_light_map_based_detector
  * chore: rename traffic_light_visualization to autoware_traffic_light_visualization
  ---------
* Contributors: Taekjin LEE, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
