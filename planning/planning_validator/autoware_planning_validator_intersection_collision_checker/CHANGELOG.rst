^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_validator_intersection_collision_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* refactor(planning_validator): refactor planning validator configuration and error handling (`#11081 <https://github.com/autowarefoundation/autoware_universe/issues/11081>`_)
  * refactor trajectory check error handling
  * define set_diag_status function for each module locally
  * update documentation
  ---------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* docs(intersection_collision_checker): update documentation (`#11070 <https://github.com/autowarefoundation/autoware_universe/issues/11070>`_)
  * update implementation description in readme
  * update parameters description in readme
  * fix spelling mistakes
  ---------
* fix(intersection_collision_checker): improve logic for objects near overlap (`#11058 <https://github.com/autowarefoundation/autoware_universe/issues/11058>`_)
  * parameterize max history time
  * improve logic for object near overlap point
  ---------
* fix(intersection_collision_checker): use parameter instead of static constexpr (`#11034 <https://github.com/autowarefoundation/autoware_universe/issues/11034>`_)
  use parameter instead of static constexpr
* feat(intersection_collision_checker): improve logic to address false negatives (`#11030 <https://github.com/autowarefoundation/autoware_universe/issues/11030>`_)
  * revise and improve velocity estimation and tracking logic
  * ensure consistent collision lane ids
  * small refactoring
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
* feat(intersection_collision_checker): improve target lane selection and velocity estimation to reduce false positives (`#10928 <https://github.com/autowarefoundation/autoware_universe/issues/10928>`_)
  * reset velocity and track time for too high acceleration. apply max velocity threshold
  * refine target lanes selection
  * add package maintainers
  * fix behavior for objects close to overlap
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
* Contributors: Mete Fatih Cırıt, mkquda

0.46.0 (2025-06-20)
-------------------
* Merge remote-tracking branch 'upstream/main' into tmp/TaikiYamada/bump_version_base
* feat(planning_validator): implement redundant collision prevention feature when ego makes a turn at intersection (`#10750 <https://github.com/autowarefoundation/autoware_universe/issues/10750>`_)
  * create planning latency validator plugin module
  * Revert "chore(sync-files.yaml): not synchronize `github-release.yaml` (`#1776 <https://github.com/autowarefoundation/autoware_universe/issues/1776>`_)"
  This reverts commit 871a8540ade845c7c9a193029d407b411a4d685b.
  * create planning trajectory validator plugin module
  * Update planning/planning_validator/autoware_planning_validator/src/manager.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/node.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * minor fix
  * refactor implementation
  * uncomment lines for adding pose markers
  * fix CMakeLists
  * add comment
  * update planning launch xml
  * Update planning/planning_validator/autoware_planning_latency_validator/include/autoware/planning_latency_validator/latency_validator.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/plugin_interface.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/include/autoware/planning_validator/types.hpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_validator/src/node.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * Update planning/planning_validator/autoware_planning_latency_validator/src/latency_validator.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * apply pre-commit checks
  * rename plugins for consistency
  * rename directories and files to match package names
  * refactor planning validator tests
  * add packages maintainer
  * separate trajectory check parameters
  * add missing package dependencies
  * move trajectory diagnostics test to trajectory checker module
  * remove blank line
  * add new planning validator plugin collision_checker
  * add launch args for validator modules
  * logic for setting route handler
  * add poincloud filtering function
  * check for turn direction lane
  * compute target lanelets for collision check
  * fix format
  * refactor lanelets filtering
  * update launch files
  * move lanelet selection functions to utils file
  * check for collisions
  * check observation time of tracked object
  * add more config params, fix pcd object function
  * extend target lanes
  * add off timeout after collision is detected
  * define const value
  * improve overlap time estimation
  * Update planning/planning_validator/autoware_planning_validator_collision_checker/src/collision_checker.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * rename planning_validator collision_checker module
  * move pointcloud filtering to just before collision check
  * use proper name for module param
  * fix logic for extending target lanelets
  * change logging level
  ---------
  Co-authored-by: GitHub Action <action@github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* Contributors: TaikiYamada4, mkquda
