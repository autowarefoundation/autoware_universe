^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_run_out_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* fix(run_out): fix  numerical stability in run_out interpolation (`#10808 <https://github.com/autowarefoundation/autoware_universe/issues/10808>`_)
  * fix(run_out): fix  numerical stability in run_out interpolation
  * fix build
  ---------
* feat(run_out): option to preserve parts of ignored predicted paths (`#10754 <https://github.com/autowarefoundation/autoware_universe/issues/10754>`_)
* chore(run_out): add Alqudah Mohammad as maintainer (`#10762 <https://github.com/autowarefoundation/autoware_universe/issues/10762>`_)
* fix(run_out): guard against decreasing ego trajectory times (`#10746 <https://github.com/autowarefoundation/autoware_universe/issues/10746>`_)
* feat(autoware_motion_velocity_planner): only wait for required subscriptions (`#10732 <https://github.com/autowarefoundation/autoware_universe/issues/10732>`_)
* Contributors: Maxime CLEMENT, Ryohsuke Mitsudome, TaikiYamada4, Yuxuan Liu

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* fix(motion_velocity_planner): add missing header (`#10560 <https://github.com/autowarefoundation/autoware_universe/issues/10560>`_)
* fix(motion_velocity_planner): remove unused functions (`#10563 <https://github.com/autowarefoundation/autoware_universe/issues/10563>`_)
* fix(motion_velocity_planner): remove unused function (`#10564 <https://github.com/autowarefoundation/autoware_universe/issues/10564>`_)
* fix(motion_velocity_planner/run_out): fix tf2 include (.hpp->.h) (`#10548 <https://github.com/autowarefoundation/autoware_universe/issues/10548>`_)
* chore(motion_velocity_run_out): add diagnostic_updater for dependency resolve (`#10535 <https://github.com/autowarefoundation/autoware_universe/issues/10535>`_)
* feat(motion_velocity_planner): add new run_out module (`#10388 <https://github.com/autowarefoundation/autoware_universe/issues/10388>`_)
* Contributors: Mamoru Sobue, Masaki Baba, Maxime CLEMENT, Ryuta Kambe, TaikiYamada4
