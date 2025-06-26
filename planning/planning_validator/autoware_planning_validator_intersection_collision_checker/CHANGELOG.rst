^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_validator_intersection_collision_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
