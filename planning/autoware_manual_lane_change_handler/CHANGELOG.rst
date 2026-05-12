^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_manual_lane_change_handler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.51.0 (2026-05-01)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* fix(autoware_manual_lane_change_handler): use RouteHandler directly (`#12389 <https://github.com/mitsudome-r/autoware_universe/issues/12389>`_)
  * fix
  * removed unused params
  ---------
* chore(planning): remove unused lanelet2_extension header (`#12294 <https://github.com/mitsudome-r/autoware_universe/issues/12294>`_)
  * unused lanelet2_extension in planning component
  * unused lanelet2_extension in planning component (2)
  * unused lanelet2_extension in planning component (3)
  ---------
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* fix(manual_lane_change_handler): remove unreadVariable (`#11973 <https://github.com/mitsudome-r/autoware_universe/issues/11973>`_)
* Contributors: Arjun Jagdish Ram, Ryuta Kambe, Sarun MUKDAPITAK, github-actions

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(manual_lane_change_handler): remove unusedVariable (`#11972 <https://github.com/autowarefoundation/autoware_universe/issues/11972>`_)
* feat(manual_lane_change_handler): check reroute_availability when set_preferred_lanes is called (`#11819 <https://github.com/autowarefoundation/autoware_universe/issues/11819>`_)
  add feature to check the reroute_availability when the set_preferred_lane is called
* Contributors: Ryohsuke Mitsudome, Ryuta Kambe, Taiki Yamada

0.49.0 (2025-12-30)
-------------------

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(manual_lane_change_handler): publishing shift-number (`#11641 <https://github.com/autowarefoundation/autoware_universe/issues/11641>`_)
  * publishing shift-number
  * changed warn to info
  ---------
  Co-authored-by: Taiki Yamada <129915538+TaikiYamada4@users.noreply.github.com>
* feat(mission_planner): manual lane selection (`#11169 <https://github.com/autowarefoundation/autoware_universe/issues/11169>`_)
  * manual lane change
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Arjun Jagdish Ram, Ryohsuke Mitsudome
