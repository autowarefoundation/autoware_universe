^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* fix: missing dependencies for tl_expected and libexpected-dev (`#13438 <https://github.com/autowarefoundation/autoware_universe/issues/13438>`_)
* fix(planning): fix ENABLE_AGNOCAST=1 build and startup of trajectory_selector (`#13364 <https://github.com/autowarefoundation/autoware_universe/issues/13364>`_)
  * fix(trajectory_selector): fix ENABLE_AGNOCAST=1 build
  * build(trajectory_ranker): add autoware_agnocast_wrapper_setup to the library target
  ---------
* refactor(trajectory_adapter): integrate trajectory adapter into selector node (`#13355 <https://github.com/autowarefoundation/autoware_universe/issues/13355>`_)
  * feat(trajectory_adapter): integrate trajectory adapter into selector node (`#3312 <https://github.com/autowarefoundation/autoware_universe/issues/3312>`_)
  integrate trajectory adapter into selector node
  - Extract TrajectoryAdapter and TrajectoryAdapterWrapper from the standalone adapter node
  - Integrate adapter into the selector pipeline after trajectory ranking
  - Publish planning trajectory and turn indicators from trajectory_selector_node
  - Keep latency debug publishing in TrajectoryAdapterWrapper
  - Convert autoware_trajectory_adapter from a standalone node into a shared library
  - Remove the standalone trajectory adapter node and its launch file
  - Add autoware_trajectory_adapter, autoware_planning_msgs, and autoware_vehicle_msgs dependencies to the selector package
  - Update trajectory_selector.launch.xml with trajectory and turn-indicator output remaps
  * apply pre-commit checks
  * add missing include
  ---------
* feat(trajectory_adapter): apply `agnocast_wrapper::Node` to trajectory_adapter (`#12842 <https://github.com/autowarefoundation/autoware_universe/issues/12842>`_)
  * apply agnocast_wrapper::Node to trajectory adapter
  * fix cpplint
  * fix copilot review
  ---------
  Co-authored-by: kobayu858 <yutaro.kobayashi.2@tier4.jp>
* refactor(planning): move node design files into each package (`#13102 <https://github.com/autowarefoundation/autoware_universe/issues/13102>`_)
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* feat(diffusion_planner,trajectory_ranker,trajectory_adapter): populate turn_indicator field on CandidateTrajectory (`#12922 <https://github.com/autowarefoundation/autoware_universe/issues/12922>`_)
  * add turn_indicator into cantidate trajectory
  * rename field
  * fix rebase misstake
  * changes for turn_indicator topic publisher from adapter
  * fix turn_indicators_command timestamp
  ---------
* Contributors: Koichi Imai, Kotakku, Ryohsuke Mitsudome, Taekjin LEE, Yutaro Kobayashi, mkquda

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

0.50.0 (2026-02-14)
-------------------

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(trajectory_adapter): add trajectory adapter (`#11324 <https://github.com/autowarefoundation/autoware_universe/issues/11324>`_)
  * add trajectory adaptor
  * change spelling from adaptor to adapter
  * fix spelling
  * disable spelling check for commnet
  * fix header include files
  * remove function removeOverlapPoints
  * remove motion_utils
  * remove end() check
  * add node suffix
  ---------
* Contributors: Go Sakayori, Ryohsuke Mitsudome
