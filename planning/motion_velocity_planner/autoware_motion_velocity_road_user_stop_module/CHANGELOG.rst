^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_road_user_stop_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* fix(road_user_stop): remove generate_parameter_library's default_value parameters (`#11117 <https://github.com/autowarefoundation/autoware_universe/issues/11117>`_)
  * delete generate_parameter_library's_default_value parameter
  * rerun ci
  ---------
* feat(road_user_stop): add road_user_stop module (`#11041 <https://github.com/autowarefoundation/autoware_universe/issues/11041>`_)
  * initial commit of road_user_stop module
  * add stop condition for wrong user and masked adjacent lanelet
  * update detection and stop planning method
  * change stop plan algorithm
  * use generate_parameter_library
  * chattering prevention and on/off parameterize of intersection and crosswalk
  * change detection method of intersection, and tidy up
  * enable gtest
  * fix default parameter to match obstacle_stop
  * code tidy up
  * update readme
  * fix based on PR review
  * remove dead code
  * align function name convention
  * change calculation method of ego lanelets and stop distance.
  * fix typo
  * remove dead code
  * change stop planning for opposing traffic
  * fix typo
  * add ScopedTimeTrack
  * use obstacle_stop based logic
  * performance improvement
  * update README
  * tidyup
  * add scoped time track
  * add virtual wall
  * (test) empty debug marker
  * minimum debug marker
  * remove unnecessary maybe_unused
  * rename scoped time track variables
  ---------
* Contributors: Kotakku
