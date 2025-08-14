^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_validator_rear_collision_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* feat(rear_collision_checker): support selecting safety metric from TTC or RSS (`#11072 <https://github.com/autowarefoundation/autoware_universe/issues/11072>`_)
  * fix: return distance to predicted collision point
  * refactor: move to utils
  * refactor: move to utils
  * refactor: generalize function
  * feat: selectable metric
  * chore: rename variable for readability
  * refactor: not use lambda
  ---------
* refactor(planning_validator): refactor planning validator configuration and error handling (`#11081 <https://github.com/autowarefoundation/autoware_universe/issues/11081>`_)
  * refactor trajectory check error handling
  * define set_diag_status function for each module locally
  * update documentation
  ---------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* refactor(rear_collision_checker): update parameter structure (`#11067 <https://github.com/autowarefoundation/autoware_universe/issues/11067>`_)
  * refactor(rear_collision_checker): update parameter structure
  * chore: remove unused msg type
  ---------
* chore(rear_collision_checker): add maintainer (`#11069 <https://github.com/autowarefoundation/autoware_universe/issues/11069>`_)
* feat(rear_collision_checker): improve collision detection logic (`#10992 <https://github.com/autowarefoundation/autoware_universe/issues/10992>`_)
  * feat(rear_collision_checker): added a parameter to configure how many seconds ahead to predict collisions
  * feat(rear_collision_checker): add parameter to enable collision check for forward obstacles
  ---------
* feat(intersection_collision_checker): improve icc debug markers (`#10967 <https://github.com/autowarefoundation/autoware_universe/issues/10967>`_)
  * add DebugData struct
  * refactor and publish debug info and markers
  * always publish lanelet debug markers
  * refactor debug markers code
  * remove unused functions
  * pass string by reference
  * set is_safe flag for rear_collision_checker debug data
  * fix for cpp check
  * add maintainer
  ---------
* feat(intersection_collision_checker): improve feature to reduce false positive occurrence (`#10899 <https://github.com/autowarefoundation/autoware_universe/issues/10899>`_)
  * keep a map of already detected target lanelets
  * fix on/off time buffers logic
  * add debug marker to visualize colliding object
  * use resampled trajectory instead of raw trajectory
  * fix overlap index computation
  * fix on/off time buffers logic for rear collision checker
  * fix planning .pages file, fix format
  * update readme
  * ignore not moving pcd object
  * handle case when object is very close to overlap point
  ---------
* feat(planning_validator): improve intersection collision checker implementation (`#10839 <https://github.com/autowarefoundation/autoware_universe/issues/10839>`_)
  * use parameter generator library
  * add pointcloud latency compensation
  * change msg field name
  * add readme file
  * add parameters dection to readme
  * publish planning factor for intersection_collision_checker
  * refactor lanelet selection and filtering
  * update readme
  * set safety factor array in planning factor
  * clean up includes
  * publish planning factor for rear collision checker
  * fix spelling
  * rename variables to avoid shadowing
  * Update planning/planning_validator/autoware_planning_validator/src/node.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * fix planning factor initialization
  * fix format
  * add on time buffer
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* Contributors: Mete Fatih Cırıt, Satoshi OTA, mkquda

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
