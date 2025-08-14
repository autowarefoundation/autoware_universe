^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_boundary_departure_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
