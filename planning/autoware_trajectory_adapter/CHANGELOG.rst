^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
