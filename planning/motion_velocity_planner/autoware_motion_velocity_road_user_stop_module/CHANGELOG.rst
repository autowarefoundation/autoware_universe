^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_velocity_road_user_stop_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* refactor: fix leftover dependent autoware_utils from updating vehicle_info_utils (`#11734 <https://github.com/autowarefoundation/autoware_universe/issues/11734>`_)
* fix(road_user_stop): fix topic name string (`#11678 <https://github.com/autowarefoundation/autoware_universe/issues/11678>`_)
* feat(motion_velocity_planner): refactor time publisher (`#11646 <https://github.com/autowarefoundation/autoware_universe/issues/11646>`_)
  * delete bdp time publish
  * delete dynamic_obstacle_stop time publish
  * delete obstacle_cruise time publish
  * delete obstacle_slow_down time publish
  * delete obstacle_velocity_limiter time publish
  * delete out of lane time publish
  * delete run out time publish
  * add short module name function
  ---------
* Contributors: Mete Fatih Cırıt, Ryohsuke Mitsudome, Yuki TAKAGI

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(road_user_stop): delete path_length_buffer (`#11653 <https://github.com/autowarefoundation/autoware_universe/issues/11653>`_)
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* docs(road_user_stop): add parameter docs (`#11539 <https://github.com/autowarefoundation/autoware_universe/issues/11539>`_)
  add parameters docs
* feat(road_user_stop): exclude ego vehicle's sides from detection area (`#11496 <https://github.com/autowarefoundation/autoware_universe/issues/11496>`_)
  * exclude ego side polygon
  * fix typo
  * delete polygon_utils copy function
  ---------
* feat(road_user_stop): pass for stop vru (`#11491 <https://github.com/autowarefoundation/autoware_universe/issues/11491>`_)
  * pass for stop vru
  * fix cppcheck
  ---------
* Contributors: Kotakku, Ryohsuke Mitsudome, Tim Clephas, Yuki TAKAGI

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
