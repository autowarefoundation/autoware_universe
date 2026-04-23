^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_boundary_departure_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* refactor(boundary_departure): rename Abnormalities type (`#12071 <https://github.com/autowarefoundation/autoware_universe/issues/12071>`_)
  * refactor(boundary_departure): rename Abnormalities type
  * refactor: rename variable name
  * refactor: replace order or member variable
  ---------
* chore: reflect the move of the description packages (`#11969 <https://github.com/autowarefoundation/autoware_universe/issues/11969>`_)
* refactor(boundary_departure): footprint generator and renaming abnormality type to footprint type (`#11992 <https://github.com/autowarefoundation/autoware_universe/issues/11992>`_)
  * refactor(boundary_departure): abnormality generator
  * refactor: rename generator to footprint generator
  * rename folder
  * renaming type to Footprint type
  * refactor: precommit
  * fix: remove unused variables
  * refactor: rename abnormality_to_check variable
  * refactor: rename (build failure)
  * refactoring: variable name change
  * Update common/autoware_boundary_departure_checker/src/footprint_generator/steering_footprint.cpp
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  * Update common/autoware_boundary_departure_checker/src/footprint_generator/steering_footprint.cpp
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
  * chore: pre-commit
  ---------
  Co-authored-by: mkquda <168697710+mkquda@users.noreply.github.com>
* refactor(boundary_departure): reduce number of defined structs and alias (`#11985 <https://github.com/autowarefoundation/autoware_universe/issues/11985>`_)
  * refactor(boundary_departure): replace hardcoded abnormality to map
  * refactor(boundary_departure): remove CriticalDeparturePoint struct
  * Reduce using alias
  * refactor(boundary_departure): setting up critical departure
  * refactor(boundary_departure): remove unnecessary structs
  * fix optional access
  ---------
* refactor(boundary_departure): move debug marker related functions (`#11952 <https://github.com/autowarefoundation/autoware_universe/issues/11952>`_)
  * refactor(boundary_departure): move debug marker related functions
  * Update common/autoware_boundary_departure_checker/src/debug.cpp
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * Update common/autoware_boundary_departure_checker/include/autoware/boundary_departure_checker/debug.hpp
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* refactor(boundary_departure): move function from boundary_departure_prevention to uncrossable_boundary_departure_checker (`#11944 <https://github.com/autowarefoundation/autoware_universe/issues/11944>`_)
  * refactor(boundary_departure): move function from bdp to bdc
  * minor refactoring
  * move clock, change to static
  ---------
* refactor(boundary_departure): reduce call to boundary_departure_checker_ptr and class renaming (`#11864 <https://github.com/autowarefoundation/autoware_universe/issues/11864>`_)
  * refactor: move function so that only one call is needed to find abnormality data
  * fix precommit
  * removed unnecessary member variable
  ---------
* Contributors: Ryohsuke Mitsudome, Takagi, Isamu, Zulfaqar Azmi

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* refactor(boundary_departure_checker): replace autoware_utils (`#11733 <https://github.com/autowarefoundation/autoware_universe/issues/11733>`_)
* Contributors: Mete Fatih Cırıt, Ryohsuke Mitsudome

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* feat(turn_signal_decider): add threshold based on distance to lane bound for turning off blinker (`#11519 <https://github.com/autowarefoundation/autoware_universe/issues/11519>`_)
  * feat(turn_signal_decider): add threshold based on distance to lane bound for turning off blinker
  * fix default value in readme
  * Update readme
  * update parameter description
  ---------
* fix(boundary_departure): false activation for underpass/overpass areas (`#11323 <https://github.com/autowarefoundation/autoware_universe/issues/11323>`_)
  * fix(boundary_departure): false activation for underpass/overpass areas
  * fix precommit
  ---------
* fix(autoware_boundary_departure_checker): remove unused function (`#11189 <https://github.com/autowarefoundation/autoware_universe/issues/11189>`_)
* fix(autoware_boundary_departure_checker): add missing dependency to magic_enum (`#11200 <https://github.com/autowarefoundation/autoware_universe/issues/11200>`_)
* fix(autoware_boundary_departure_checker): remove unused function (`#11190 <https://github.com/autowarefoundation/autoware_universe/issues/11190>`_)
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
* fix(boundary_departure): refactor merge_departure_intervals function to avoid crashes (`#11129 <https://github.com/autowarefoundation/autoware_universe/issues/11129>`_)
* Contributors: Maxime CLEMENT, Ryohsuke Mitsudome, Ryuta Kambe, Tim Clephas, Zulfaqar Azmi

0.47.1 (2025-08-14)
-------------------
* fix(autoware_boundary_departure_checker): add missing dependency to magic_enum (`#11201 <https://github.com/autowarefoundation/autoware_universe/issues/11201>`_)
* Contributors: Mete Fatih Cırıt

0.47.0 (2025-08-11)
-------------------
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
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(start_planner,boundary_departure_checker): remove micro inner rings after bg::union\_ (`#10971 <https://github.com/autowarefoundation/autoware_universe/issues/10971>`_)
* feat(control_evaluator): boundary departure check in control evaluator (`#10859 <https://github.com/autowarefoundation/autoware_universe/issues/10859>`_)
  * feat(control_evaluator): add boundary departure check
  * slight refactoring
  * fix node dying due to invalid linestring reference
  * docstring fix
  * Revise  metric's messages and move function call
  * add comments to the extra margin
  ---------
* Contributors: Mehmet Dogru, Mete Fatih Cırıt, Zulfaqar Azmi

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
* chore(boundary_departure_checker): add maintainer (`#10805 <https://github.com/autowarefoundation/autoware_universe/issues/10805>`_)
  * chore(boundary_departure_checker): add maintainer
  * Remove some maintainer
  ---------
* Contributors: TaikiYamada4, Zulfaqar Azmi

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* refactor(departure_checker): move lane departure checker class to departure_checker  (`#10337 <https://github.com/autowarefoundation/autoware_universe/issues/10337>`_)
  * RT1-9640: separate lane departure checker library
  * move back parameter
  * separating parameters
  * renamed to boundary departure checker
  * pre-commit
  * remove trajectory deviation
  * rename namespace
  * move boundary departure checker to common folder
  * rename class name
  ---------
* Contributors: TaikiYamada4, Zulfaqar Azmi
