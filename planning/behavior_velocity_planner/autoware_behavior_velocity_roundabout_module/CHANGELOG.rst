^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_behavior_velocity_roundabout_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(autoware_lanelet2_utils): replace ported functions from autoware_lanelet2_extension (`#11593 <https://github.com/autowarefoundation/autoware_universe/issues/11593>`_)
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* fix: tf2 uses hpp headers in rolling (and is backported) (`#11620 <https://github.com/autowarefoundation/autoware_universe/issues/11620>`_)
* feat(behavior_velocity_rtc_interface, behavior_velocity\_*_module): replace PathWithLaneId with Trajectory<> class (`#11555 <https://github.com/autowarefoundation/autoware_universe/issues/11555>`_)
* feat(autoware_lanelet2_extension): remove redundant autoware_lanelet2_extension depend from packages (`#11492 <https://github.com/autowarefoundation/autoware_universe/issues/11492>`_)
* feat(autoware_lanelet2_utils): porting functions from lanelet2_extension to autoware_lanelet2_utils package (replacing usage) in planning component (`#11374 <https://github.com/autowarefoundation/autoware_universe/issues/11374>`_)
  Co-authored-by: Mamoru Sobue <hilo.soblin@gmail.com>
* feat(roundabout): add autoware_behavior_velocity_roundabout_module (`#11142 <https://github.com/autowarefoundation/autoware_universe/issues/11142>`_)
  * feat(behavior_velocity_roundabout_module): add roundabout behavior velocity planner module
  * feat(behavior_planning): add roundabout module to behavior planning launch and RTC interface
  * Add functions for behavior velocity planner in roundabout module
  * Refactor roundabout module
  * feat(roundabout_module): integrate Roundabout regulatory element
  * refactor(roundabout_module): remove associative_ids from attention lane
  * move header files to include
  * refactor(roundabout_module): remove util files and update includes for intersection module
  * fix(roundabout_module): update include path for interpolated_path_info
  * refactor(roundabout_module): update include paths for consistency
  * fix(scene_roundabout): update include path for result.hpp
  * refactor(roundabout_module): clean up unused structures and methods
  * refactor(roundabout_module): clean up and optimize code structure and comments
  * refactor(object_manager): remove unused stopline handling from ObjectInfo initialization
  * refactor(roundabout_module): remove unused variables and comments related to second pass judge line
  * refactor(roundabout_module): improve comments and remove unnecessary code in decision result and collision handling
  * feat(ttc_visualizer): add TTC visualizer script for time-to-collision analysis
  * refactor(roundabout_module): remove consider_wrong_direction_vehicle parameter and related code
  * refactor(roundabout_module): simplify object management and improve code clarity
  * style(pre-commit): autofix
  * fix(manager): include <set> header for improved functionality
  * fix(roundabout): update parameter name
  * docs(roundabout): update README to clarify module role and functionality
  * fix(readme): fix README
  * fix(readme): fix README
  * style(pre-commit): autofix
  * refactor(roundabout): fix attention lanelet  logic
  * style(pre-commit): autofix
  * review roundabout
  * update utils
  * refactor(roundabout): fix attention lanelet logic
  * feat(cmake): add installation for ttc.py script
  * refactor(detectOcclusion):  fix cppcheck error
  * style(pre-commit): autofix
  * docs(README): fix unkown word ci error
  * style(pre-commit): autofix
  * fix: update copyright year to 2025 in multiple files
  * docs(README): add flowchart for roundabout behavior velocity module logic
  * style(pre-commit): autofix
  * fix: correct capitalization in copyright notice
  * refactor(roundabout-module): streamline roundabout module initialization and remove unused conflicting area logic
  * refactor(roundabout-module): remove unnecessary comments and streamline lanelet handling
  * refactor(roundabout-module): rename variables for clarity and improve lanelet handling
  * style(pre-commit): autofix
  * refactor(roundabout-module): remove attention_area_length parameter and update related logic
  * style(pre-commit): autofix
  * build(build_depends_humble.repos): bump autoware_lanelet2_extension to 0.9.0
  * fix(package): add dependency on tf2_geometry_msgs
  * feat(planning): add roundabout handling to planning factors and conversion map
  * Revert "feat(planning): add roundabout handling to planning factors and conversion map"
  This reverts commit e79eab23e89969f88bc7485c4ed337e843d33a7f.
  * refactor(roundabout):  Move the parameter definition
  * style(pre-commit): autofix
  * fix(package): add  maintainer
  * refactor(debug): clean up includes and remove unused color definitions
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Y.Hisaki <yhisaki31@gmail.com>
  Co-authored-by: pre-commit-ci-lite[bot] <117423508+pre-commit-ci-lite[bot]@users.noreply.github.com>
* Contributors: Mitsuhiro Sakamoto, Ryohsuke Mitsudome, Sarun MUKDAPITAK, Sho Iwasawa, Tim Clephas
