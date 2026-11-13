^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory_ranker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* fix(planning): fix ENABLE_AGNOCAST=1 build and startup of trajectory_selector (`#13364 <https://github.com/autowarefoundation/autoware_universe/issues/13364>`_)
  * fix(trajectory_selector): fix ENABLE_AGNOCAST=1 build
  * build(trajectory_ranker): add autoware_agnocast_wrapper_setup to the library target
  ---------
* feat(trajectory_ranker): implement and integrate ranker into selector node (`#13353 <https://github.com/autowarefoundation/autoware_universe/issues/13353>`_)
  * feat(trajectory_ranker): implement new ranker module and integrate into selector component (`#3208 <https://github.com/autowarefoundation/autoware_universe/issues/3208>`_)
  * add trajectory_ranker_wrapper framework
  * refactor trajectory ranker parameter handling
  * implement trajectory_ranker class framework
  * implement core ranker logic
  - add logic to evaluate trajectories based on risk level
  - add logic to evaluate trajectories based on source
  - use existing metrics based evaluation to evaluate trajectory quality
  * integrate new ranker into trajectory_selector_node
  * refactor code
  * remove obsolete ranker node
  * refactor for debugging
  * add flag to enable/disable ranker within selectory node
  * fix parameter update logic, cleanup code
  * update launch files
  * disable quality evaluation by default
  * fix topic name
  * minor refactor
  * output debug to console when best trajectory has low score
  * support new backup planner dual go/stop trajectories
  * populate generator info of ScoredCandidateTrajectories
  * filter out shadow mode metrics before assigning combined trajectory risk level
  * update source penalties
  * add integration tests for trajectory ranker
  * add ranker parameters schema, update readme
  * remove simple_trajectory_ranker_node
  * remove launch prefix
  * pass active_filter_names to validator from wrapper
  * add missing includes
  ---------
  * replace rclcpp::Node usage by agnocast_wrapper::Node
  * fix selector node tests
  * add missing selector config file
  ---------
* refactor(planning): move node design files into each package (`#13102 <https://github.com/autowarefoundation/autoware_universe/issues/13102>`_)
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* feat(simple_trajectory_ranker): apply agnocast_wrapper::Node to simple_trajectory_ranker (`#13065 <https://github.com/autowarefoundation/autoware_universe/issues/13065>`_)
  * feat: apply agnocast
  * fix; test
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* feat(diffusion_planner,trajectory_ranker,trajectory_adapter): populate turn_indicator field on CandidateTrajectory (`#12922 <https://github.com/autowarefoundation/autoware_universe/issues/12922>`_)
  * add turn_indicator into cantidate trajectory
  * rename field
  * fix rebase misstake
  * changes for turn_indicator topic publisher from adapter
  * fix turn_indicators_command timestamp
  ---------
* fix(diffusion_planner, trajectory ranker): remove builder pattern (`#12944 <https://github.com/autowarefoundation/autoware_universe/issues/12944>`_)
  remove builder pattern
* Contributors: Kotakku, Ryohsuke Mitsudome, Taekjin LEE, Yutaro Kobayashi, mkquda

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(planning): apply autoware_agnocast_wrapper to diffussion planner trajectory pipeline nodes for CIE (`#12779 <https://github.com/autowarefoundation/autoware_universe/issues/12779>`_)
  * feat(autoware_diffusion_planner): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_optimizer): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_adapter): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_ranker): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_selector): apply autoware_agnocast_wrapper for CIE
  * feat(autoware_trajectory_modifier): apply autoware_agnocast_wrapper for CIE
  ---------
* Contributors: atsushi yano, github-actions

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(trajectory_ranker): add simple ranker only based on generator name (`#11963 <https://github.com/mitsudome-r/autoware_universe/issues/11963>`_)
* feat(autoware_lanelet2_extension): replace remaining lanelet2_extension utilities functions - planning component (`#12083 <https://github.com/mitsudome-r/autoware_universe/issues/12083>`_)
  * replace getArcCoordinates in planning component
  * replace getCenterlineWithOffset in planning component
  * replace getRight/LeftBoundWithOffset in planning component
  * replace getExpandedLanelet(s) in planning component
  * replace combineLaneletsShape in planning component
  * remove log for empty combine_lanelet_opt
  * bind reference to optional value
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* Contributors: Maxime CLEMENT, Sarun MUKDAPITAK, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore(trajectory_ranker,trajectory_traffic_rule_filter): add maintainers (`#12020 <https://github.com/autowarefoundation/autoware_universe/issues/12020>`_)
  chore(trajectory_ranker,traffic_rule_filter): add maintainers
* fix(autoware_trajectory_ranker): prevent node crash when performing inner product operation (`#12009 <https://github.com/autowarefoundation/autoware_universe/issues/12009>`_)
  * fix: prevent not crash when performing inner product operation
  * fix: missing code
  * fix: wrong variable
  ---------
* Contributors: Maxime CLEMENT, Ryohsuke Mitsudome, Zulfaqar Azmi

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* feat(trajectory_ranker): add trajectory consistency score (`#11762 <https://github.com/autowarefoundation/autoware_universe/issues/11762>`_)
  * add trajectory consistency score
  * add test
  * metric with configurable parameters
  * fix pre-commit
  * fix comments
  * fix calculation of total variance
  * extract common ego frame transformation logic
  * updated test
  ---------
* Contributors: Go Sakayori, Ryohsuke Mitsudome

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* feat(trajectory_ranker): add trajectory ranker (`#11318 <https://github.com/autowarefoundation/autoware_universe/issues/11318>`_)
  * add basic implementation of trajectory ranker
  * fix repeating call when calculation jerk
  * use range based for loop
  * remove unnecessary static_cast<std::ptrdiff_t>
  * use move semantic
  * change file name and include directory structure
  * change type double to float
  * fix package.xml
  * fix CMakeList
  * add ndoe suffix
  * use early return
  * avoid zero division for metrics calculation
  * use parameter for ttc calcultion in metrics
  * fix metric calculation
  ---------
* Contributors: Go Sakayori, Ryohsuke Mitsudome, Tim Clephas
