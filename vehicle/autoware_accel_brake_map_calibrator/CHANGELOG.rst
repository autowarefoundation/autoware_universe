^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_accel_brake_map_calibrator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
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
* Contributors: Takahisa Ishikawa, github-actions

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* style: update pre-commit (black 26.1.0, pre-commit-hooks-ros 0.10.2) (`#12195 <https://github.com/mitsudome-r/autoware_universe/issues/12195>`_)
* Contributors: Taeseung Sohn, github-actions

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* chore(accel_brake_map_calibrator): change output topic name of accel_brake_map_calibrator (`#11251 <https://github.com/autowarefoundation/autoware_universe/issues/11251>`_)
* Contributors: Ryohsuke Mitsudome, Tim Clephas, Yukinari Hisaki

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
* feat(autoware_accel_brake_map_calibrator)!: tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_accel_brake_map_calibrator (`#9923 <https://github.com/autowarefoundation/autoware_universe/issues/9923>`_)
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Vishal Chauhan

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
* fix(cpplint): include what you use - vehicle (`#9575 <https://github.com/autowarefoundation/autoware_universe/issues/9575>`_)
* ci(pre-commit): autoupdate (`#8949 <https://github.com/autowarefoundation/autoware_universe/issues/8949>`_)
  Co-authored-by: M. Fatih Cırıt <mfc@autoware.org>
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo, awf-autoware-bot[bot]

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
* feat(autoware_accel_brake_map_calibrator): conditional actuation data processing based on source (`#8593 <https://github.com/autowarefoundation/autoware_universe/issues/8593>`_)
  * fix: Conditional Actuation Data Processing Based on Source
  * style(pre-commit): autofix
  * delete extra comentout, indent
  * add take validation
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(autowre_accel_brake_map_calibrator): fix for flake-ros v0.9.0 (`#8529 <https://github.com/autowarefoundation/autoware_universe/issues/8529>`_)
* fix(autoware_accel_brake_map_calibrator): fix redundantInitialization (`#8230 <https://github.com/autowarefoundation/autoware_universe/issues/8230>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(accel_brake_map_calibrator): replace polling takeData function with the callback function (`#7429 <https://github.com/autowarefoundation/autoware_universe/issues/7429>`_)
  * fix : repush to solve conflict
  * style(pre-commit): autofix
  * delete duplicated int cast
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(accel_brake_map_calibrator)!: add autoware\_ prefix (`#7351 <https://github.com/autowarefoundation/autoware_universe/issues/7351>`_)
  * add prefix to the codes
  change dir name
  update
  update
  * delete debug
  * fix format
  * fix format
  * restore
  * poi
  ---------
* Contributors: Kosuke Takeuchi, Ryuta Kambe, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, eiki

0.26.0 (2024-04-03)
-------------------
