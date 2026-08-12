# RefinePenetrationByStaticObjects

## Overview

The `RefinePenetrationByStaticObjects` processor is designed to refine the predicted path by considering the penetration of the vehicle into static objects. This processor helps to improve the accuracy of the predicted path by taking into account the static objects in the environment.

## Purpose

## Algorithm

### Pseudo Code

```pseudocode
ALGORITHM: RefinePenetrationByStaticObjects

INPUT: (mutable)target, context, speed_threshold

BEGIN
  FOR EACH mode_idx, mode IN ENUMERATE(target.predicted_paths) DO
    // Find collision with slow-moving obstacles using OBB
    hit = FIND_COLLISION(target, mode_idx, context.objects, speed_threshold)
    IF NOT hit THEN
      CONTINUE
    END IF

    // Compute cumulative distances along the original path
    base_keys = CUMULATIVE_DISTANCES(mode.path)
    // Compute expected travel distances based on target speed
    query_keys = CUMULATIVE_TRAVEL_DISTANCES(target.speed, mode.time_step, LENGTH(mode.path))

    // Clamp query distances to collision point
    s_max = MIN(hit.distance, base_keys.LAST)
    query_keys = [CLAMP(s, 0, s_max) FOR s IN query_keys]

    // Interpolate and update waypoints
    FOR i = 1 TO LENGTH(mode.path) - 1 DO
      mode.path[i].position = INTERPOLATE(base_keys, original_positions, query_keys[i])
      mode.path[i].orientation = AZIMUTH_BETWEEN(mode.path[i-1], mode.path[i])
    END FOR
  END FOR
END
```

## Parameters

| Parameter         | Type   | Default  | Unit | Description                                                                                                |
| ----------------- | ------ | -------- | ---- | ---------------------------------------------------------------------------------------------------------- |
| `speed_threshold` | double | 1.0      | m/s  | Refine penetration if the path collides with objects slower than this value.                               |
| `interpolation`   | string | "linear" | -    | Interpolation method to use when refining penetration. Options: "linear", "spline", and "spline_by_akima". |

## Configuration Example

```yaml
/**:
  ros__parameters:
    processors: [refine_penetration_by_static_objects]
    refine_penetration_by_static_objects:
      speed_threshold: 1.0 # Refine penetration only when the path collides with objects slower than this value.
      interpolation: "linear" # Interpolation method. Options: "linear", "spline", and "spline_by_akima".
```
