^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_crosswalk_traffic_light_estimator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(traffic-light):  fix traffic light nodes message types for system design format files (`#12446 <https://github.com/mitsudome-r/autoware_universe/issues/12446>`_)
  fix(perception): update message types for traffic light nodes to use autoware_perception_msgs
* chore(perception): move perception node configuration file to each package (`#12440 <https://github.com/mitsudome-r/autoware_universe/issues/12440>`_)
  move perception node configuration file to each package
* refactor(autoware_crosswalk_traffic_light_estimator): improve readability of get_non_red_lanelets (`#12380 <https://github.com/mitsudome-r/autoware_universe/issues/12380>`_)
  * refactor(autoware_crosswalk_traffic_light_estimator): extract is_lanelet_non_red free function
  * refactor(autoware_crosswalk_traffic_light_estimator): replace boost::optional with std::optional
  * refactor(autoware_crosswalk_traffic_light_estimator): extract is_green_or_amber and is_unknown_or_absent helpers
  * refactor(autoware_crosswalk_traffic_light_estimator): improve readability of is_lanelet_non_red with named variables
  * style(pre-commit): autofix
  * refactor(autoware_crosswalk_traffic_light_estimator): add decision table comment to is_lanelet_non_red
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* refactor(autoware_universe): use autoware_ament_auto_package in perception utility packages (`#12281 <https://github.com/mitsudome-r/autoware_universe/issues/12281>`_)
  Co-authored-by: github-actions <github-actions@github.com>
* fix(crosswalk_traffic_light_estimator): skip lanelets with no signal info in get_non_red_lanelets (`#12370 <https://github.com/mitsudome-r/autoware_universe/issues/12370>`_)
  * test(crosswalk_traffic_light_estimator): add test case for right turn arrow scenario
  * test(crosswalk_traffic_light_estimator): unify test map
  * fix(crosswalk_traffic_light_estimator): prevent unwatched traffic lights from being treated as non-red
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* feat(crosswalk_traffic_light_estimator): simplify CrosswalkTrafficLightEstimator behavior and add unit tests (`#12288 <https://github.com/mitsudome-r/autoware_universe/issues/12288>`_)
  * refactor(crosswalk_traffic_light_estimator): simplify update_map method by removing routing graph parameter
  * feat(crosswalk_traffic_light_estimator): simplify get_non_red_lanelets logic by removing unnecessary signal history check
  * test(crosswalk_traffic_light_estimator): add unit tests for traffic light estimation logic
  * test(crosswalk_traffic_light_estimator): add unit tests for signal estimation rules
  * test(crosswalk_traffic_light_estimator): simplify integration test to single empty-input case
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(crosswalk_traffic_light_estimator): extract core estimation logic (`#12273 <https://github.com/mitsudome-r/autoware_universe/issues/12273>`_)
  * refactor(autoware_crosswalk_traffic_light_estimator): rename node.cpp to crosswalk_traffic_light_estimator.cpp
  * refactor(autoware_crosswalk_traffic_light_estimator): extract CrosswalkTrafficLightEstimator logic
  * fix(autoware_crosswalk_traffic_light_estimator): value-initialize config\_ to fix cppcheck warning
  * fix(autoware_crosswalk_traffic_light_estimator): move static member functions to anonymous namespace
  * fix(autoware_crosswalk_traffic_light_estimator): add const qualifier to match declaration and definition
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
* refactor(crosswalk_traffic_light_estimator): rename function names to snake case and move header file to src (`#12262 <https://github.com/mitsudome-r/autoware_universe/issues/12262>`_)
  * refactor(autoware_crosswalk_traffic_light_estimator): rename functions and variables to snake_case
  Rename camelCase function names and UPPER_CASE constexpr variables to
  snake_case to comply with the Autoware coding conventions defined in
  .clang-tidy (readability-identifier-naming).
  * refactor(autoware_crosswalk_traffic_light_estimator): move headers from include/ to src/
  Move node.hpp and flashing_detection.hpp to src/ since they are
  internal headers not needed by external packages. Update all include
  paths accordingly.
  * refactor(autoware_crosswalk_traffic_light_estimator): restore const value name
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* refactor(crosswalk_traffic_light_estimator): extract flashing signal detector and unit tests (`#12252 <https://github.com/mitsudome-r/autoware_universe/issues/12252>`_)
  * feat(crosswalk_traffic_light_estimator): extract flashing signal detection logic into separate class and add tests
  * refactor(crosswalk_traffic_light_estimator): update estimate_stable_color to include current_time and remove update_signal_history
  * refactor(crosswalk_traffic_light_estimator): replace direct detector usage with DetectorTimeline for improved test clarity
  * refactor(flash_detection): extract signal history update logic into separate method for clarity
  * refactor(crosswalk_traffic_light_estimator): rename DetectorTimeline to FlashingDetectorDriver for clarity and consistency
  * refactor(flash_detection): remove redundant FlashingTransition_RedToGreen test and update assertions for clarity
  * refactor(autoware_crosswalk_traffic_light_estimator): reduce cyclomatic complexity of update_signal_history
  * refactor(autoware_crosswalk_traffic_light_estimator): reduce cyclomatic complexity of update_flashing_state
  * refactor(autoware_crosswalk_traffic_light_estimator): reduce cyclomatic complexity of update_and_get_color_state
  * refactor(autoware_crosswalk_traffic_light_estimator): extract remove_expired_entries and improve readability
  * refactor(crosswalk_traffic_light_estimator): reorganize signal handling functions for improved clarity and maintainability
  * refactor(crosswalk_traffic_light_estimator): use unordered_map for flashing state maps
  Key ordering is not needed for is_flashing\_ and current_color_state\_,
  so use unordered_map for consistency with other maps in the same file.
  * fix(flashing_detection): correct confidence value for occluded signals in skippable signal check
  * fix(crosswalk_traffic_light_estimator): reset flashing state when history contains only UNKNOWN entries
  * test(crosswalk_traffic_light_estimator): update PrunesOldEntries test to verify UNKNOWN-only history reset
  * test(crosswalk_traffic_light_estimator): use default confidence for UNKNOWN signals in flashing detection tests
  * test(crosswalk_traffic_light_estimator): clarify test condition
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: Masaki Baba <masaki.baba.2@tier4.jp>
* perf(perception): use emplace_back and emplace to avoid temporary object creation (`#12201 <https://github.com/mitsudome-r/autoware_universe/issues/12201>`_)
  * perf(perception): use emplace_back to avoid temporary object creation
  * style(pre-commit): autofix
  * perf(perception): use emplace/emplace_back for most containers
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* chore(traffic_light_recognition): add maintainer (`#12221 <https://github.com/mitsudome-r/autoware_universe/issues/12221>`_)
  add maintainer
  Co-authored-by: badai nguyen <94814556+badai-nguyen@users.noreply.github.com>
* feat(crosswalk_traffic_light_estimator): remove /route topic to simplify estimation logic (`#12116 <https://github.com/mitsudome-r/autoware_universe/issues/12116>`_)
  * feat(autoware_crosswalk_traffic_light_estimator): remove route dependency and use signal-driven crosswalk estimation
  Remove ~/input/route subscription and instead precompute traffic light to
  crosswalk mappings at map load time. The estimation now processes only
  crosswalks related to received traffic signals, eliminating per-callback
  routing graph queries and improving performance.
  * test(autoware_crosswalk_traffic_light_estimator): add integration tests for crosswalk signal estimation
  * doc(autoware_crosswalk_traffic_light_estimator): remove route dependency from README and update related descriptions
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Takahisa.Ishikawa <takahisa.ishikawa@tier4.jp>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
  Co-authored-by: Masato Saeki <78376491+MasatoSaeki@users.noreply.github.com>
* Contributors: Masaki Baba, Taekjin LEE, Takahisa Ishikawa, Vishal Chauhan, github-actions, nishikawa-masaki

0.50.0 (2026-02-14)
-------------------

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
* fix(autoware_crosswalk_traffic_light_estimator): remove old traffic light groups (`#11536 <https://github.com/autowarefoundation/autoware_universe/issues/11536>`_)
  * remove old traffic light groups
  * refactor set CrosswalkTrafficSignals
  * style(pre-commit): autofix
  * add const &
  * chore
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(crosswalk_traffic_light_estimator): estimation rules from lanelet map (`#11397 <https://github.com/autowarefoundation/autoware_universe/issues/11397>`_)
* Contributors: Masato Saeki, Maxime CLEMENT, Ryohsuke Mitsudome

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(autoware_crosswalk_traffic_light_estimator): add parameters to use pedestrian traffic signal result estimated in perception pipeline (`#10763 <https://github.com/autowarefoundation/autoware_universe/issues/10763>`_)
  * add flag to use pedestrian signals result
  * style(pre-commit): autofix
  * change param name
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Masato Saeki, TaikiYamada4

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
* Merge remote-tracking branch 'origin/main' into humble
* fix(autoware_crosswalk_traffic_light_estimator): add process that guard access to empty elements (`#10281 <https://github.com/autowarefoundation/autoware_universe/issues/10281>`_)
  * fix(autoware_crosswalk_traffic_light_estimator) : add process that guard access to empty elements.
  * fix for linter
  ---------
* Contributors: Ryohsuke Mitsudome, k-hazama-esol

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
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* chore: refine maintainer list (`#10110 <https://github.com/autowarefoundation/autoware_universe/issues/10110>`_)
  * chore: remove Miura from maintainer
  * chore: add Taekjin-san to perception_utils package maintainer
  ---------
* Contributors: Fumiya Watanabe, Shunsuke Miura, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_crosswalk_traffic_light_estimator)!: tier4_debug_msgs changes to autoware_internal_debug_msgs in autoware_crosswalk_traffic_light_estimator (`#9870 <https://github.com/autowarefoundation/autoware_universe/issues/9870>`_)
* chore(autoware_crosswalk_traffic_light_estimator): fix docs (`#9822 <https://github.com/autowarefoundation/autoware_universe/issues/9822>`_)
  * fix docs
  * fix docs
  * add tlr output image
  * modify sentense
  * modify sentense
  * refactor readme
  * fix docs
  * fix
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_crosswalk_traffic_light_estimator): overwrite invalid detection result (`#9667 <https://github.com/autowarefoundation/autoware_universe/issues/9667>`_)
  * add code in order to check invalid detection
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Masato Saeki, Vishal Chauhan

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
* fix(autoware_crosswalk_traffic_light_estimator): fix constVariableReference (`#8055 <https://github.com/autowarefoundation/autoware_universe/issues/8055>`_)
  fix:constVariableReference
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware_universe/issues/7640>`_)
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* fix(tier4_control_launch, crosswalk_traffic_light_estimator): fix a mistake when adding prefixes (`#7423 <https://github.com/autowarefoundation/autoware_universe/issues/7423>`_)
  Fixed a mistake when adding prefixes
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
* refactor(crosswalk_traffic_light_estimator)!: add autoware\_ prefix (`#7365 <https://github.com/autowarefoundation/autoware_universe/issues/7365>`_)
  * add prefix
* Contributors: Kosuke Takeuchi, SakodaShintaro, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, kobayu858

0.26.0 (2024-04-03)
-------------------
