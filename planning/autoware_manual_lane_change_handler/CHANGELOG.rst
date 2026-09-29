^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_manual_lane_change_handler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* fix(autoware_manual_lane_change_handler): added a new maintainer (`#13273 <https://github.com/autowarefoundation/autoware_universe/issues/13273>`_)
  Added new maintainer
* fix(planning, control): point design param_files at the packages that install them (`#13219 <https://github.com/autowarefoundation/autoware_universe/issues/13219>`_)
  BehaviorPathPlanner listed its plugin modules' param files as relative
  paths, resolving against the host node package; each module package
  installs its own config. PlanningValidator referenced its checker
  plugins' param files under the host package with a planning_validator\_
  filename prefix the plugins do not use. TrajectoryFollower referenced
  config/ where the package installs param/, and controller files that
  exist per controller type (mpc, pid). ManualLaneChangeHandler declared
  a param file that does not exist; the node declares no parameters.
  The run_out module ships a config directory that ament_auto_package()
  did not install.
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
* refactor(planning): move node design files into each package (`#13102 <https://github.com/autowarefoundation/autoware_universe/issues/13102>`_)
  Co-authored-by: Claude Opus 5 (1M context) <noreply@anthropic.com>
* Contributors: Arjun Jagdish Ram, Ryohsuke Mitsudome, Taekjin LEE

0.52.0 (2026-06-30)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat: add area support for route planning and fix DCO signoff (`#12572 <https://github.com/autowarefoundation/autoware_universe/issues/12572>`_)
  * feat: add support for area for route planning
  * feat(mission_planner): publish lane+area route segments and goal height
  * fix(manual_lane_change_handler): guard lane-only APIs when route has areas
  * feat(mission_planner): visualize route area segments as LINE_STRIP in RViz
  * fix (remaining_distance_calculator): derive lane list from route msg when areas present in the route
  * fix(mission_planner): drop redundant goal_height init in refine_goal_height
  Removes cppcheck redundantInitialization warning; goal_height is always
  set in the area vs lane branches before use.
  * style(pre-commit): autofix
  * fix: resolve merge conflicts
  * style(pre-commit): autofix
  ---------
  Co-authored-by: Ryohsuke Mitsudome <ryohsuke.mitsudome@tier4.jp>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: emmeyteja, github-actions

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
