^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_validator_rear_collision_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(planning_validator): implement a collision detection feature for rearward objects of the vehicle (`#10800 <https://github.com/autowarefoundation/autoware_universe/issues/10800>`_)
  * feat(planning_validator): add new flag
  * feat(rear_collision_checker): add new validator plugin
  * fix: current lane extraction range
  * fix: keep previous data when he estimated velocity may be an outlier
  * fix: check reachable distance
  * fix: remove unused param
  * fix: integrate new interface
  * fix: use common marker publisher
  * fix: rename diag
  * fix: parameterize
  * fix: keep checking
  * fix: author
  * fix: remove unused variable
  * chore: vru -> vulnerable_road_user
  * fix: early return
  * fix: cppcheck
  * fix: cppcheck
  * fix: cppcheck
  * fix: clang tidy
  ---------
* Contributors: Satoshi OTA, TaikiYamada4
