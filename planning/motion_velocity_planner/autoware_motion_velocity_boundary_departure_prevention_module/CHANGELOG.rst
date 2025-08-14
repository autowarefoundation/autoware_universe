^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_boundary_departure_prevention_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
