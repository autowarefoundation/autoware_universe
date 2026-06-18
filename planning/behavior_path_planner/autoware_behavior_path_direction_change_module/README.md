# Direction Change Module

## Overview

This module enables the behavior path planner to handle paths with cusp points (direction changes) by detecting cusp points and reversing path point orientations (yaw angles) to indicate reverse direction segments. This enables bidirectional path following through orientation-based direction indication.

## Features

- **Cusp Detection**: Automatically detects points in the path where direction changes occur (typically 180-degree turns) using configurable angle thresholds
- **Orientation Reversal**: Reverses path point yaw angles at cusp points to indicate reverse direction segments
- **Direction Change Handling**: Enables the vehicle to follow paths with direction changes (forward and reverse segments)
- **Map-Based Activation**: Activates only when `direction_change` is set to `"yes"` in lanelet map and ego is away from the goal pose.

## Parameters

See `config/direction_change.param.yaml` for detailed parameter descriptions.

## Integration

This module is automatically registered as a plugin in the behavior path planner. To enable it, add it to the module list in the behavior path planner configuration.

## Usage

The module activates automatically when:

- **A `direction_change` attribute is set to `"yes"` in the lanelet map** (required)
- The path is within lanelets containing this tag

## Map Requirements

See the [document](autoware_lanelet2_extension/autoware_lanelet2_extension/docs/lanelet2_format_extension.md)