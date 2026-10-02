^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_obstacle_proximity_checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.53.0 (2026-09-29)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.53.0-changelog
* feat(trajectory_modifier, minimum_rule_based_planner): improve obstacle stop feature (`#13255 <https://github.com/autowarefoundation/autoware_universe/issues/13255>`_)
  * fix(trajectory_modifier): fix obstacle stop unstable stop wall (`#3196 <https://github.com/autowarefoundation/autoware_universe/issues/3196>`_)
  * fix stop wall appears behind ego
  * always publish modifier debug markers
  * fix extend_trajectory() function
  - use path curvature at end instead of relying on trajecory point orientations
  - for low end speed trajectory, default to straight extension
  * fix insert_stop_point logic to prevent wrong orientation stop pose
  * introduce minimum stop margin below which duplicate_check_threshold is ignored
  * handle zero stop_point_arc_length in insert_stop_point function
  ---------
  * refactor(obstacle_stop): optimize collision check logic (`#3024 <https://github.com/autowarefoundation/autoware_universe/issues/3024>`_)
  refactor obstacle stop utility function get_nearest_object_collision, update default param values
  * feat(trajectory_modifier): remove max object velocity threshold for obstacle stop (`#3234 <https://github.com/autowarefoundation/autoware_universe/issues/3234>`_)
  * remove max_velocity_th param, fix collision check logic
  * tune rss params
  * apply pre-commit checks
  ---------
  * feat(trajectory_modifier, backup_planner): enable filtering objects by type and shape (`#3255 <https://github.com/autowarefoundation/autoware_universe/issues/3255>`_)
  * enable filtering objects by type and shape
  - update obstacle_stop params in modifier and backup planner to specify enabled types per shape
  - update surround_obstacle_stop params in modifier and backup planner to specify enabled types per shape
  - modify ObjectFilter code in obstacle_stop_utils to support filtering by type and shape
  - modify ObstacleProximityChecker class to support filtering by type and shape
  - fix ObstacleTracker logic for ignoring orientation change
  * Update planning/autoware_trajectory_modifier/include/autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * apply pre-commit checks
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
  * fix cherry-pick errors
  ---------
  Co-authored-by: Maxime CLEMENT <78338830+maxime-clem@users.noreply.github.com>
* chore(common): update package maintainers (`#13182 <https://github.com/autowarefoundation/autoware_universe/issues/13182>`_)
* feat: support new object classification labels (`#12956 <https://github.com/autowarefoundation/autoware_universe/issues/12956>`_)
  * fix(obstacle_proximity_checker): support new object classification labels
  * feat(collision_detector): support new object classification labels
  * feat(surround_obstacle_checker): support new object classification labels
  * fix: apply pre-commit
  ---------
* feat(trajectory_modifier): support new object classes in trajectory_modifier (`#12910 <https://github.com/autowarefoundation/autoware_universe/issues/12910>`_)
  * support HAZARD & ANIMAL object classes in modifier obstacle stop
  * add missing object types for surround_obstacle_stop
  ---------
* feat(trajectory_modifier): add surround obstacle stop plugin to trajectory modifier (`#12894 <https://github.com/autowarefoundation/autoware_universe/issues/12894>`_)
  * extract core logic from surround_obstacle_checker to new common package obstacle_proximit_checker
  * add new modifier plugin surround_obstacle_stop which uses common package obstacle_proximity_checker
  * apply pre-commit checks
  * refactor implementation, add integration test for surround_obstacle_stop
  * run proximity checker only once per planning cycle
  * add maintainers for new package
  * update default param values
  * update and use utility function replace_trajectory_with_stop_point
  * add readme for autoware_obstacle_proximity_checker
  * fix unit tests
  ---------
* Contributors: Ryohsuke Mitsudome, Satoshi OTA, mkquda
