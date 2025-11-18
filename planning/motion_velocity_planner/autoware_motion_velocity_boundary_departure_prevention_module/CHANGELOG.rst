^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_boundary_departure_prevention_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* docs(boundary_departure): update documentation (`#11229 <https://github.com/autowarefoundation/autoware_universe/issues/11229>`_)
  * docs(boundary_departure): update documentation
  * add additional explanation
  * rearrange subsection
  * explanation on reducing false positive
  * fix equation not rendered
  * Braking distance calculatation
  * All explained
  * fix equation not rendered correctly and image size too small
  * fix table
  * fix spell check
  ---------
* refactor(boundary_departure): move visualization marker functions to … (`#11331 <https://github.com/autowarefoundation/autoware_universe/issues/11331>`_)
  refactor(boundary_departure): move visualization marker functions to a new function
* docs(boundary_departure): update README for new steering abnormalities (`#11212 <https://github.com/autowarefoundation/autoware_universe/issues/11212>`_)
* refactor(boundary_departure): make plan single-exit to eliminate repeated debug logs and redundant update_ptr\_ calls (`#11245 <https://github.com/autowarefoundation/autoware_universe/issues/11245>`_)
  * refactor(boundary_departure): move plan to other function so that updater_ptr is called only once
  * remove warn
  * docstring
  * rename function and remove unreacheable
  ---------
* refactor(boundary_departure): simplify diagnostic pointer call (`#11243 <https://github.com/autowarefoundation/autoware_universe/issues/11243>`_)
  * refactor(boundary_departure): simplify diagnostic pointer call
  * use default throttle duration
  ---------
* fix(autoware_motion_velocity_boundary_departure_prevention_module): remove unused function (`#11215 <https://github.com/autowarefoundation/autoware_universe/issues/11215>`_)
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* fix(boundary_departure): explicitly delete diagnotics ptr after module's destruction (`#11217 <https://github.com/autowarefoundation/autoware_universe/issues/11217>`_)
  fix(boundary_departure): explicitly delete diagnotics ptr after module destruction
* chore(boundary_departure): add maintainer (`#11214 <https://github.com/autowarefoundation/autoware_universe/issues/11214>`_)
* fix(boundary_departure_checker): update poses for the new trajectory (`#11198 <https://github.com/autowarefoundation/autoware_universe/issues/11198>`_)
* feat(boundary_departure): support addition parameters for dynamic reconfiguration  (`#11180 <https://github.com/autowarefoundation/autoware_universe/issues/11180>`_)
  * feat(boundary_departure): rqt reconfigure
  * fix node dying
  * fix precommit
  ---------
* fix(motion_velocity_planner): remove unused function (`#11177 <https://github.com/autowarefoundation/autoware_universe/issues/11177>`_)
* fix(motion_velocity_planner): remove unused function (`#11178 <https://github.com/autowarefoundation/autoware_universe/issues/11178>`_)
* fix(motion_velocity_planner): remove unused function (`#11175 <https://github.com/autowarefoundation/autoware_universe/issues/11175>`_)
* fix(boundary_departure): critical departure not cleared (`#11173 <https://github.com/autowarefoundation/autoware_universe/issues/11173>`_)
* fix(boundary_departure): fix chattering critical departure MRM (`#11171 <https://github.com/autowarefoundation/autoware_universe/issues/11171>`_)
* perf(boundary_departure): more efficient arc length calculations for ego footprint points (`#11136 <https://github.com/autowarefoundation/autoware_universe/issues/11136>`_)
  * directly calculate lon_dist_on_ref_traj from the fp's trajectory point
  * remove unused vehicle longitudinal offset argument
  * remove unused trajectory argument
  * substract the longitudinal offset of the projected ego point
  * rename lon_dist_on_ref_traj to lon_dist_on_pred_traj
  * revise and optimize the dist_on_traj to be the ego_dist_on_ref_traj
  * remove unused find_new_critical_departure_points function
  * remove offsets by the ego vehicle longitudinal offset
  * refactor to use motion_utils instead of trajectory::closest (perf)
  * remove last trajectory::closest (perf)
  * fix braking_dist calculation
  ---------
* feat(boundary_departure_checker): improve steering abnormality (`#11130 <https://github.com/autowarefoundation/autoware_universe/issues/11130>`_)
* refactor(boundary_departure): check output exist and matches type for diagnostic level (`#11150 <https://github.com/autowarefoundation/autoware_universe/issues/11150>`_)
  fix(boundary_departure): check output exist and matches type for diag level
* fix(boundary_departure): gradual slow down with feasible profile (`#11131 <https://github.com/autowarefoundation/autoware_universe/issues/11131>`_)
  * fix(boundary_departure): gradual slow down when comfort slow not possible
  * fix calculation and add feasible velocity
  * fix CI
  ---------
* feat(boundary_departure): on time buffer for critical departure (`#11126 <https://github.com/autowarefoundation/autoware_universe/issues/11126>`_)
  * feat(boundary_departure): on time buffer for critical departure
  * refactoring
  * replace critical dpt time to double
  * rename other variable for consistency
  ---------
* fix(boundary_departure): refactor merge_departure_intervals function to avoid crashes (`#11129 <https://github.com/autowarefoundation/autoware_universe/issues/11129>`_)
* Contributors: Maxime CLEMENT, Ryohsuke Mitsudome, Ryuta Kambe, Zulfaqar Azmi

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat(boundary_departure): slow down computation (`#11085 <https://github.com/autowarefoundation/autoware_universe/issues/11085>`_)
  * feat(boundary_departure): slow down computation
  * fixed some logic
  * refactor function get_interp_to_point()
  * Update planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/slow_down_interpolator.cpp
  * fix pre-commit diff error
  * use function
  ---------
  Co-authored-by: mohammad alqudah <alqudah.mohammad@tier4.jp>
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
* feat(boundary_departure): configurable departure points and type based on time (`#11073 <https://github.com/autowarefoundation/autoware_universe/issues/11073>`_)
  * feat(boundary_departure): configurable departure points and type based on time
  * Update planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module/src/utils.hpp
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * move out is_departure_persist
  * cutoff time based on decel
  * refactoring
  * refactoring
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* fix(boundary_departure): merging multiple close by departure points (`#11083 <https://github.com/autowarefoundation/autoware_universe/issues/11083>`_)
  * fix(boundary_departure): merging multiple closeby departure points
  * removed nested loops and fix critical departure being merged together
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  * refactoring
  * refactor merge function
  ---------
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
* fix(boundary_departure): configurable diagnostic level (`#11064 <https://github.com/autowarefoundation/autoware_universe/issues/11064>`_)
  * RT-10807 fix diagnostic level cant be configured properly.
  * RT1-10807 fix parameter configuration
  * fix MRM chatters when not in autonomous mode
  ---------
* fix(boundary_departure): subscribe to route to determine goal changes (`#11004 <https://github.com/autowarefoundation/autoware_universe/issues/11004>`_)
  * fix(boundary_departure): use route msg to check for goal changes
  * changed goal ptr to route ptr
  ---------
* refactor(boundary_departure): reduce autonomous mode logging level (`#10986 <https://github.com/autowarefoundation/autoware_universe/issues/10986>`_)
  refactor(boundary_departure): reduce logging level
* chore(boundary_departure): add logging for monitoring (`#10969 <https://github.com/autowarefoundation/autoware_universe/issues/10969>`_)
* fix(boundary_departure): disable when autoware disengaged (`#10962 <https://github.com/autowarefoundation/autoware_universe/issues/10962>`_)
  fix(boundary_departure): disable when ego is not in autonomous mode
* fix(boundary_departure): exception and log handling (`#10914 <https://github.com/autowarefoundation/autoware_universe/issues/10914>`_)
  fix(boundary_departure): exception handling
* refactor(boundary_departure): use polling subscriber (`#10901 <https://github.com/autowarefoundation/autoware_universe/issues/10901>`_)
* Contributors: Zulfaqar Azmi

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(motion_velocity_planner): boundary departure prevention module (`#10817 <https://github.com/autowarefoundation/autoware_universe/issues/10817>`_)
  * feat(motion_velocity_planner): boundary departure prevention module
  * add maintainers
  * Add initial readme
  * Adds abnormalities explanation
  * fix infinite loop error
  * fix goal getter
  * add flowchart
  * add initial image folder
  * Add remaining abnormality footprints images
  * docstring on the boundary departure checker
  * Revert motion_planning.launch.xml to state before 14323161e3
  * Additional docstrings and separating slow down interpolator file
  * Update closest projection
  * jsonschema for parameters
  * fix json schema error
  * fix jsonschema
  * Departure type explanation
  * fix cppcheck failure
  * remove author tag
  * Fix misleading explanation
  * removed unused member variable
  * move boundary departure to experimental namespace
  * update maintainer's email address
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * grammar fix
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * refactoring
  * refactor slow down tuple
  * fix build failure due to clang-tidy
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* Contributors: TaikiYamada4, Zulfaqar Azmi
