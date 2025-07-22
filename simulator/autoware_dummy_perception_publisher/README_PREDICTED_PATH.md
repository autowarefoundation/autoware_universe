# Dummy Object Predicted Path Feature - Quick Reference

This feature enables dummy objects in the simulation to follow predicted trajectories from the perception system, providing more realistic object movement.

## What's New

### RViz Tools

- Added "Predicted" checkbox to all dummy object placement tools (car, pedestrian, unknown)
- Press 'P' to toggle prediction mode for objects
- Objects created with prediction enabled use `action: PREDICT`

### Core Functionality

- **Euclidean Distance Mapping**: Automatically assigns nearest predicted objects to dummy objects
- **Trajectory Interpolation**: Objects follow curved predicted paths instead of straight lines
- **Smart Remapping**: Handles disappeared predictions by finding new best matches
- **Validation System**: Ensures realistic movement with speed, position, and trajectory checks

## Key Algorithm: `updateDummyToPredictedMapping`

Located in `src/node.cpp:1056`, this function:

1. Collects all available predicted objects and their positions
2. Identifies dummy objects that need new predictions
3. Finds best matches using Euclidean distance
4. Validates matches for realistic movement
5. Updates the mapping maintained in `dummy_to_predicted_uuid_map_`

### Distance-Based Matching

- Vehicles: Max 10m initial matching distance
- Pedestrians: Max 5m initial matching distance
- Considers speed compatibility and trajectory similarity

### Position Calculation

When a dummy object has `action: PREDICT`:

- Uses `calculateTrajectoryBasedPosition` to interpolate along predicted path
- Falls back to extrapolation if beyond predicted trajectory
- Maintains smooth motion even during remapping

## Configuration

Key parameters in `config/dummy_perception_publisher.param.yaml`:

- `predicted_path_delay`: 2.0 (seconds before using predictions)
- `min_keep_duration`: 3.0 (minimum seconds to keep same prediction)
- Vehicle thresholds: 2.0m distance, 15° yaw, 5% speed tolerance
- Pedestrian thresholds: 3.0m distance, 45° yaw, 30% speed tolerance

See [docs/PARAMETER_EXPLANATION.md](docs/PARAMETER_EXPLANATION.md) for detailed parameter usage

## Usage Example

```bash
# 1. Launch the dummy perception publisher
ros2 launch autoware_dummy_perception_publisher dummy_perception_publisher.launch.xml

# 2. In RViz, place objects with "Predicted" checkbox enabled

# 3. Ensure predicted objects are being published
ros2 topic echo /perception/object_recognition/objects
```

## Files Modified

- `src/node.cpp`: Core mapping logic and trajectory interpolation
- `include/.../node.hpp`: New member functions and data structures
- `src/tools/interactive_object.cpp`: Added predicted property to RViz tools
- `config/dummy_perception_publisher.param.yaml`: Configuration parameters
- `schema/dummy_perception_publisher.schema.json`: Parameter validation

For detailed documentation, see `docs/PREDICTED_PATH_FEATURE.md`
