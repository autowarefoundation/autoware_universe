^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory_optimizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* revert(autoware_trajectory_optimizer): "fix(trajectory_optimizer): set_max_velocity (`#11642 <https://github.com/autowarefoundation/autoware_universe/issues/11642>`_)" (`#11657 <https://github.com/autowarefoundation/autoware_universe/issues/11657>`_)
  Revert "fix(trajectory_optimizer): set_max_velocity (`#11642 <https://github.com/autowarefoundation/autoware_universe/issues/11642>`_)"
  This reverts commit 3519f7e65deec25f0a178ac406884836f05008ad.
* fix(autoware_trajectory_smoother): reset eb smoother after each iter (`#11643 <https://github.com/autowarefoundation/autoware_universe/issues/11643>`_)
* fix(trajectory_optimizer): set_max_velocity (`#11642 <https://github.com/autowarefoundation/autoware_universe/issues/11642>`_)
  Fixed set_max_velocity
* fix(trajectory_optimizer): correct kinematic enforcer indexing and plugin order (`#11633 <https://github.com/autowarefoundation/autoware_universe/issues/11633>`_)
  Fixed off-by-one error in kinematic feasibility enforcer segment distance calculation.
  The constraint evaluation needs distance from anchor to current point, while point
  placement needs distance from current to next point. Added explicit anchor-to-first-point
  distance and use segment_distances[i+1] for placement.
  Reordered plugins to remove final TrajectoryPointFixer which was overriding heading
  alignments via resample_close_proximity_points, causing orientations mismatched with
  geometric heading direction.
* feat(autoware_trajectory_optimizer): optimizer kinematic feasibility plugin (`#11616 <https://github.com/autowarefoundation/autoware_universe/issues/11616>`_)
  * WIP add Ackerman-bicycle model-based kinematic feasibility enforcer
  * WIP close point merging
  * add close proximity point resampling
  * remove prints
  * add cluster resampling
  * move resampling of close proximity points to utils lib
  * comment
  * use autoware_utils_math::normalize_radian to simplify code
  * docs
  * schema and default value mention
  * remove unused param from schema
  * review comments and fix yaw addition bug
  * improve description
  ---------
* refactor(autoware_trajectory_optimizer): reuse function to copy original orientation (`#11594 <https://github.com/autowarefoundation/autoware_universe/issues/11594>`_)
* revert(autoware_trajectory_optimizer): akima spline to old implementation (`#11573 <https://github.com/autowarefoundation/autoware_universe/issues/11573>`_)
  * revert(trajectory_optimizer): restore original Akima spline implementation
  Revert apply_spline() to the working implementation from commit 93ace66f96
  (Oct 7, 2025) to restore good Akima spline performance observed on Sep 30.
  The implementation broken by commit 4de1a6df6e used a complex 4-stage
  resampling approach (2x linear → Akima → crop → linear) that degraded
  interpolation quality. This revert restores the simple single-pass Akima
  spline using the experimental trajectory interpolator.
  Changes:
  - Restore single-pass AkimaSpline interpolation using Builder pattern
  - Keep orientation copying feature (added in commit 55ad6dd9c6)
  - Remove unused max_yaw_discrepancy_deg parameter from signature and configs
  - Add autoware_trajectory dependency to package.xml
  - Update tests to match new signature
  - Add calculate_time_from_start() after Akima spline and EB smoother to fix
  acceleration recalculation in velocity optimizer
  * fix orientation check threshold
  * make a common function for accel recalculation
  * add acceleration recalcualtion after velocity-changing functions
  * remove magic numbers for constexpr
  * add const to func definition
  * remove recalc acc as it is not needed in clamp velocities
  ---------
* feat(autoware_trajectory_optimizer): dynamic plugin ordering (`#11566 <https://github.com/autowarefoundation/autoware_universe/issues/11566>`_)
  * refactor(trajectory_optimizer): convert plugins to default constructors
  - Add default constructor and initialize() method to plugin base class
  - Update all 6 plugins to use default constructors
  - Fix plugin inheritance to public for accessibility
  - Move plugin initialization to node constructor for proper parameter loading
  - Prepare foundation for pluginlib-based dynamic plugin loading
  This change maintains backward compatibility while enabling future dynamic
  plugin loading. All plugins must now be initialized via the initialize()
  method which automatically calls set_up_params().
  * feat(trajectory_optimizer): add pluginlib infrastructure for plugins
  - Create plugins.xml to declare all 6 plugins for pluginlib
  - Add PLUGINLIB_EXPORT_CLASS macros to all plugin implementations
  - Create separate plugin library for dynamic loading
  - Add pluginlib dependency to package.xml
  Plugins are now discoverable by pluginlib but still instantiated
  directly. Next step is to refactor main node to use ClassLoader.
  * feat(trajectory_optimizer): implement dynamic plugin loading via pluginlib
  - Replace hardcoded plugin pointers with pluginlib ClassLoader
  - Load plugins dynamically from plugin_names parameter
  - Plugins execute in order specified by configuration
  - Add load_plugin() method with error handling
  - Forward parameter updates to all loaded plugins
  - Add plugin_names to trajectory_optimizer.param.yaml with default order
  Plugin order is now fully configurable at startup via plugin_names parameter.
  Runtime enable/disable still supported via existing boolean activation flags.
  * fix(trajectory_optimizer): separate plugin library to avoid namespace collision
  - Move plugins to separate library not linked to main component
  - Prevents pluginlib namespace collision warnings
  - Allows proper runtime plugin loading via pluginlib ClassLoader
  This fixes the class_loader collision warnings that occurred when plugins
  were compiled into the main component library.
  * feat(trajectory_optimizer): add plugin_names parameter to schema
  - Add plugin_names array parameter to JSON schema
  - Default order matches current hardcoded execution sequence
  - Schema validates plugin class names are strings
  - Update CMakeLists formatting from pre-commit
  Users can now configure plugin execution order via plugin_names parameter
  while maintaining runtime enable/disable via activation flags.
  * docs(trajectory_optimizer): update README for pluginlib architecture
  - Document pluginlib-based plugin loading system
  - Add plugin_names parameter configuration examples
  - Update plugin development pattern for new architecture
  - Clarify plugin order configuration vs runtime activation
  - Update key files list with plugins.xml
  Documentation now reflects the dynamic plugin loading system while
  maintaining information about runtime enable/disable via activation flags.
  * remove wrong set up for smoother, update comment in param file
  * add orientation preservation to qp smoother
  * remove unused function change to private
  * add const to input variables
  ---------
* feat(autoware_trajectory_optimizer): customization of point constraining (`#11550 <https://github.com/autowarefoundation/autoware_universe/issues/11550>`_)
  * preserve first 3 points for better adherance to original path
  * parameter-based point constraining
  * remove unneeded check
  * fix qp smoother doc
  * add variable name instead of magic number
  * fix default in schema
  * remove redundant checks
  ---------
* fix(autoware_trajectory_optimizer): add set up params to be able to reconfigure velocity params (`#11529 <https://github.com/autowarefoundation/autoware_universe/issues/11529>`_)
  * add set up params to be able to reconfigure velocity params
  * add set up params for eb smoother
  ---------
* feat(autoware_trajectory_optimizer): smoother qp solver and param refactoring (`#11507 <https://github.com/autowarefoundation/autoware_universe/issues/11507>`_)
  * WIP add QP solver for path smoothing
  * wip improve convergence and weights
  * param tuning and input validation
  * remove unused params
  * add check for yaw deviations
  * rename param and functions to fix orientation
  * refactor: separate runtime data from config parameters
  Add TrajectoryOptimizerData struct to separate runtime vehicle state
  (odometry, acceleration) from configuration parameters. Update plugin
  base class and all plugins to accept data separately from params.
  Update utils functions to take explicit Odometry parameters.
  * refactor(extender): move parameters to plugin-local ownership
  Add TrajectoryExtenderParams struct with plugin-specific parameters.
  Plugin now declares and updates its own parameters independently.
  Update add_ego_state_to_trajectory() to take explicit parameters.
  * refactor(spline_smoother): move parameters to plugin-local ownership
  Add TrajectorySplineSmootherParams struct with plugin-specific parameters.
  Plugin now declares and updates its own parameters independently.
  Update apply_spline() to take explicit parameters instead of params struct.
  Update tests and callers to use new signature.
  * refactor(velocity_optimizer): move parameters to plugin-local ownership
  Add TrajectoryVelocityOptimizerParams struct with 8 plugin-specific parameters.
  Plugin now declares and updates its own parameters independently.
  Move velocity-related parameters from main node to plugin ownership.
  Parameters moved to plugin:
  - target_pull_out_speed_mps, target_pull_out_acc_mps2
  - max_speed_mps, max_lateral_accel_mps2
  - set_engage_speed, limit_speed, limit_lateral_acceleration
  - smooth_velocities
  * refactor: move QP smoother config to plugins directory
  Move qp_smoother.param.yaml from config/trajectory_smoothing/ to
  config/plugins/ for consistency with other plugin configurations.
  * refactor: update launch file to load plugin config files
  Update trajectory_optimizer.launch.xml to load all plugin-specific
  config files from the plugins/ directory:
  - trajectory_extender.param.yaml
  - trajectory_spline_smoother.param.yaml
  - trajectory_qp_smoother.param.yaml (moved from trajectory_smoothing/)
  - trajectory_velocity_optimizer.param.yaml
  * refactor: rename smooth_trajectories to use_eb_smoother and remove duplicates
  Rename parameter smooth_trajectories to use_eb_smoother for consistency
  with other plugin activation flags (use_qp_smoother, use_akima_spline).
  Remove duplicate parameter handling:
  - Remove threshold params from main node (now only in plugins)
  - Remove past_ego_state_trajectory\_ from main node (Extender owns it)
  - Remove duplicate add_ego_state_to_trajectory call (Extender handles it)
  - Remove unused last_time\_ member variable
  - Remove unused includes (utils.hpp, algorithm, etc.)
  Main node now only declares 5 activation flags. Plugin-specific parameters
  are handled exclusively by their respective plugins.
  * refactor: complete plugin parameter decoupling with namespacing
  Reduce TrajectoryOptimizerParams to 5 activation flags only.
  All plugin-specific parameters moved to plugin ownership with namespacing.
  Changes:
  - Add plugin namespaces to all parameters (trajectory_extender.*, etc.)
  - Remove duplicate parameter declarations from main node
  - Remove past_ego_state_trajectory\_ from main node (Extender owns it)
  - Remove duplicate ego state update logic
  - Add threshold params to VelocityOptimizer for filter_velocity()
  - Update filter_velocity() signature to take explicit params
  - Remove dead code: interpolate_trajectory(), copy_trajectory_orientation()
  - Remove unused member: last_time\_
  - Remove unused includes
  TrajectoryOptimizerParams reduced from 28 items to 5 flags (82% reduction).
  Each plugin now has complete parameter independence with clear namespacing.
  * refactor: update JSON schema for plugin-namespaced parameters
  Update schema to reflect new plugin-local parameter ownership with namespacing.
  Main node now defines:
  - 5 activation flags (use_akima_spline_interpolation, use_eb_smoother, etc.)
  - 4 nested plugin parameter objects with namespaces:
  - trajectory_extender (3 params)
  - trajectory_spline_smoother (4 params)
  - trajectory_qp_smoother (9 params)
  - trajectory_velocity_optimizer (10 params)
  Remove all flat plugin-specific parameters from top level.
  Rename smooth_trajectories to use_eb_smoother in schema.
  Keep external plugin configs (jerk_filter_params, elastic_band_params).
  Schema now provides validation for all namespaced plugin parameters.
  * refactor: remove unused code from main optimizer node
  Remove unused variables, functions, and includes from main node:
  - Remove empty functions: initialize_planners(), reset_previous_data()
  - Remove unused member: prev_optimized_traj_points_ptr\_
  - Remove unused includes: elastic_band.hpp, replan_checker.hpp, etc.
  - Remove unused using declarations for plugin types
  - Remove duplicate publisher creation
  - Clean up unnecessary comments
  Main optimizer node is now minimal and focused on orchestration only.
  * set akima spline as default
  * fix schema
  * Revert "fix schema"
  This reverts commit ac583c485fc6965a9cdb51dce051b99de9b4f540.
  * fix variable shadowing
  * remove useless comments
  * more comment fixing
  * update readme
  * add geometric vs stored yaw discrepancy remover
  * fix last point, change default params
  * add velocity-based fidelity weight calculation
  * disable fix orientation
  * reduce min lenght for qp solver, disable backward path extension
  * velocity threshold set to 0.25 m/s
  * remove unnecessary comments
  * update param to use degrees
  * comment fix
  * update README with plugin order information
  * remove unused function
  * param tuning
  * remove comment
  * param tuning
  * formatting
  * docs: update QP smoother default parameter values
  Update documentation and schema to match actual defaults in param file:
  - velocity_threshold_mps: 0.2 -> 0.3 m/s
  - sigmoid_sharpness: 40.0 -> 50.0
  - min_fidelity_weight: 0.1 -> 0.01
  - constrain_last_point: true -> false
  - use_velocity_based_fidelity: false -> true
  - orientation_correction_threshold_deg: 15.0 -> 5.0
  - weight_smoothness (schema): 100.0 -> 10.0
  Add missing velocity-based fidelity parameters to schema:
  - use_velocity_based_fidelity
  - velocity_threshold_mps
  - sigmoid_sharpness
  - min_fidelity_weight
  - max_fidelity_weight
  - constrain_last_point
  * pre-commit...
  ---------
* feat(autoware_trajectory_optimizer): copy nearest point orientation from original trajectory (`#11483 <https://github.com/autowarefoundation/autoware_universe/issues/11483>`_)
  * copy nearest point orientation from original trajectory
  * fix schema default, add header definition, use parameter to get closest point
  ---------
* fix(autoware_trajectory_optimizer): optimizer spline and orientation (`#11469 <https://github.com/autowarefoundation/autoware_universe/issues/11469>`_)
  * fix issue with point fixer changing the heading of the trajectory
  * replace spline with core tool
  * set eb smoothing to false
  * resample first to get better spline performance, remove outlier points
  * move velocity smoothing to before the path smoothing
  * move back the velocity smoothing step
  * turn off lat acceleration limiter
  * update schema with new params
  * rename variables to be more explicit
  * add variable names and notes for more clarity
  * fix includes
  * simplify logic with lambda
  ---------
* chore: topic migration (`#11452 <https://github.com/autowarefoundation/autoware_universe/issues/11452>`_)
  fix
* fix(trajectory_optimizer): add acceleration recalculation (`#11441 <https://github.com/autowarefoundation/autoware_universe/issues/11441>`_)
  Added recalculation
* fix(autoware_trajectory_optimizer): reduce excessive optimizer logging (`#11290 <https://github.com/autowarefoundation/autoware_universe/issues/11290>`_)
  * change warns and errors to be throttle messages
  * solve clock issue
  ---------
* feat: add `autoware_trajectory_optimizer` (`#11110 <https://github.com/autowarefoundation/autoware_universe/issues/11110>`_)
  * Added `autoware_trajectory_optimizer`
  * Added destructors
  * Added `cspell:ignore jerkfiltered`
  * Added json schema
  * style(pre-commit): autofix
  * Fixed includes
  * Fixed cpps
  * style(pre-commit): autofix
  * Added NOLINTNEXTLINE
  * Fixed test_trajectory_optimize.cpp
  * Removed duplicated sentence
  * Fix the version of cmake_minimum_required
  * Use autoware_utils_rclcpp
  * Use `autoware_utils_debug`
  * Use `autoware_utils_rclcpp`
  * Added unit to `over_stop_velocity_warn_thr`
  * Removed sentences about LICENSE
  * Removed duplicated params
  * remove previous trajectory publishing and single trajectory publishing, as they are not needed
  * undo remove of single trajectory publishing
  * Removed from "required"
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Daniel Sanchez <danielsanchezaran@gmail.com>
* Contributors: Arjun Jagdish Ram, Ryohsuke Mitsudome, SakodaShintaro, danielsanchezaran
