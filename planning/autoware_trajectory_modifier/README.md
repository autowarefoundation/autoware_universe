# Autoware Trajectory Modifier

The `autoware_trajectory_modifier` package provides a plugin-based architecture for modifying input trajectories to ensure compliance with traffic rules and regulations. This package follows a similar architecture to the trajectory optimizer but focuses on trajectory modification tasks that enforce traffic law adherence, safety constraints, and regulatory compliance.

## Architecture

The trajectory modifier uses a plugin-based system where different modification algorithms can be implemented as plugins. Each plugin inherits from the `TrajectoryModifierPluginBase` class and implements the required interface.

### Plugin Interface

All modifier plugins must inherit from `TrajectoryModifierPluginBase` and implement:

- `modify_trajectory()` - Main method to modify trajectory points
- `set_up_params()` - Initialize plugin parameters
- `on_parameter()` - Handle parameter updates

### Package Structure

```text
autoware_trajectory_modifier/
├── include/autoware/trajectory_modifier/
│   ├── trajectory_modifier.hpp
│   ├── trajectory_modifier_structs.hpp
│   ├── utils.hpp
│   └── trajectory_modifier_plugins/
│       └── trajectory_modifier_plugin_base.hpp
├── src/
│   ├── trajectory_modifier.cpp
│   ├── utils.cpp
│   └── trajectory_modifier_plugins/          # Future plugin implementations
├── config/
│   └── trajectory_modifier.param.yaml
├── launch/
│   └── trajectory_modifier.launch.xml
├── schema/
│   └── trajectory_modifier.schema.json
└── tests/
    └── test_trajectory_modifier.cpp
```

## Purpose

The primary goal of the trajectory modifier is to ensure that candidate trajectories comply with traffic rules, safety regulations, and legal requirements. This includes:

- **Traffic Rule Compliance**: Enforcing speed limits, lane restrictions, and traffic signal adherence
- **Safety Constraints**: Ensuring minimum safety distances, collision avoidance, and emergency constraints
- **Regulatory Compliance**: Meeting local traffic laws, vehicle regulations, and operational standards
- **Legal Requirements**: Adhering to jurisdiction-specific driving rules and vehicle operation laws

## Usage

The trajectory modifier node subscribes to candidate trajectories and applies configured modifier plugins sequentially. Each plugin enforces specific traffic rules or safety constraints by modifying trajectory points as needed to ensure compliance.

### Input/Output

- **Input**: `autoware_internal_planning_msgs::msg::CandidateTrajectories`
- **Output**: Modified `autoware_internal_planning_msgs::msg::CandidateTrajectories` and selected `autoware_planning_msgs::msg::Trajectory`

### Parameters

- `nearest_dist_threshold_m`: Distance threshold for nearest point search
- `nearest_yaw_threshold_rad`: Yaw threshold for nearest point search

## Adding New Modifier Plugins

To add a new modifier plugin for traffic rule compliance:

1. Create header and source files in `trajectory_modifier_plugins/`
2. Inherit from `TrajectoryModifierPluginBase`
3. Implement required virtual methods focusing on specific traffic rule enforcement
4. Register the plugin in the main node's `initialize_modifiers()` method
5. Add plugin-specific parameters to the schema and config files

## Future Extensions

This skeleton provides the foundation for implementing specific traffic rule compliance plugins such as:
- **Speed Limit Enforcement**: Modify velocities to comply with speed limits
- **Traffic Signal Compliance**: Ensure trajectories respect traffic light states
- **Lane Rule Enforcement**: Modify trajectories to follow lane restrictions and markings
- **Stop Sign Compliance**: Enforce complete stops at stop signs
- **Yield Rule Enforcement**: Modify trajectories to yield right-of-way appropriately
- **School Zone Restrictions**: Apply special speed and behavior rules in school zones
- **Construction Zone Compliance**: Adapt trajectories for construction zone rules
