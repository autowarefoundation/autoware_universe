# Direction Change Module

## Overview

This module enables the behavior path planner to handle paths with cusp points (direction changes) by detecting cusp points and reversing path point orientations (yaw angles) to indicate reverse direction segments. This enables bidirectional path following through orientation-based direction indication.

## Features

- **Cusp Detection**: Automatically detects points in the path where direction changes occur (typically 180-degree turns) using configurable angle thresholds
- **Orientation Reversal**: Reverses path point yaw angles at cusp points to indicate reverse direction segments
- **Direction Change Handling**: Enables the vehicle to follow paths with direction changes (forward and reverse segments)
- **Map-Based Activation**: Activates only when `direction_change_lane` is set to `"yes"` in lanelet map

## Parameters

See `config/direction_change.param.yaml` for detailed parameter descriptions.

## Integration

This module is automatically registered as a plugin in the behavior path planner. To enable it, add it to the module list in the behavior path planner configuration.

## Usage

The module activates automatically when:

- **A `direction_change_lane` attribute is set to `"yes"` in the lanelet map** (required)
- The path is within lanelets containing this tag

### Map Requirements

**Important**: This module only activates when lanelets in the path have the `direction_change_lane` attribute set to `"yes"` in the Lanelet2 map. Without this, the module will not activate.

To enable the module:

1. Add the `direction_change_lane` attribute to relevant lanelets in your Lanelet2 map
2. Set the attribute value to `"yes"` (e.g., `direction_change_lane: "yes"`)

The module modifies the path by:

- Checking for `direction_change_lane: "yes"` in lanelets along the path
- Detecting cusp points based on configurable angle thresholds
- Reversing path point orientations (yaw angles) at cusp points to indicate reverse segments
- Processing the entire path in batch mode (all at once)
