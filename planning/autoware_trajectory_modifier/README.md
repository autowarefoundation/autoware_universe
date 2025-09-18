# Autoware Trajectory Modifier

The `autoware_trajectory_modifier` package provides a plugin-based architecture for post-processing trajectory points to improve trajectory quality and ensure vehicle safety. It takes candidate trajectories and applies various modification algorithms to enhance their feasibility and safety characteristics.

## Features

- Plugin-based architecture for extensible trajectory modifications
- Stop point fixing to prevent trajectory issues near stationary conditions
- Configurable parameters to adjust modification behavior

## Architecture

The trajectory modifier uses a plugin-based system where different modification algorithms can be implemented as plugins. Each plugin inherits from the `TrajectoryModifierPluginBase` class and implements the required interface.

### Plugin Interface

All modifier plugins must inherit from `TrajectoryModifierPluginBase` and implement:

- `modify_trajectory()` - Main method to modify trajectory points
- `set_up_params()` - Initialize plugin parameters
- `on_parameter()` - Handle parameter updates
- `is_trajectory_modification_required()` - Determine if modification is needed

### Current Plugins

#### Stop Point Fixer

The Stop Point Fixer plugin addresses trajectory issues when the ego vehicle is stationary or moving at very low speeds. It prevents problematic trajectory points that could cause planning issues by:

- Monitoring ego vehicle velocity and trajectory distance
- Replacing the trajectory with a single stop point when conditions are met
- Ensuring smooth operation during stationary periods

## Dependencies

This package depends on the following packages:

- `autoware_internal_planning_msgs`: For candidate trajectory message types
- `autoware_planning_msgs`: For output trajectory message types
- `autoware_motion_utils`: Motion-related utility functions
- `autoware_trajectory`: Trajectory data structures and utilities
- `autoware_utils`: Common utility functions

## Input/Output

- **Input**: `autoware_internal_planning_msgs::msg::CandidateTrajectories`
- **Output**: Modified `autoware_internal_planning_msgs::msg::CandidateTrajectories` and selected `autoware_planning_msgs::msg::Trajectory`

## Parameters

- `use_stop_point_fixer`: Enable the stop point fixer modifier plugin (default: true)
- `stop_point_fixer.velocity_threshold_mps`: Velocity threshold below which ego vehicle is considered stationary (default: 0.1 m/s)
- `stop_point_fixer.min_distance_threshold_m`: Minimum distance threshold to trigger trajectory replacement (default: 1.0 m)

Parameters can be set via YAML configuration files in the `config/` directory.

## Adding New Modifier Plugins

To add a new modifier plugin:

1. Create header and source files in `trajectory_modifier_plugins/`
2. Inherit from `TrajectoryModifierPluginBase`
3. Implement the required virtual methods
4. Register the plugin in the main node's `initialize_modifiers()` method
5. Add plugin-specific parameters to the schema and config files
