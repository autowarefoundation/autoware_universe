^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_boundary_departure_prevention_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
