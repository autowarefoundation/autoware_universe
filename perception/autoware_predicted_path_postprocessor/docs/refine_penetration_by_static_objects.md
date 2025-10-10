# RefinePenetrationByStaticObjects

## Overview

The `RefinePenetrationByStaticObjects` processor is designed to refine the predicted path by considering the penetration of the vehicle into static objects. This processor helps to improve the accuracy of the predicted path by taking into account the static objects in the environment.

## Purpose

## Algorithm

### Pseudo Code

```pseudocode
ALGORITHM: RefinePenetrationByStaticObjects

INPUT: (mutable)object, context, speed_threshold

BEGIN
  FOR EACH mode IN object.predicted_paths DO
    hit = FIND_COLLISION(mode, context.objects, speed_threshold)
    // Skip if the path does not collide with any object slower than the speed threshold
    IF NOT hit THEN
      CONTINUE
    END IF

    // Refine penetration by adjusting the path based on the collision information
    original_distances = [0, d1, d2, d3, ...]  // cumulative distances
    original_positions = [p0, p1, p2, p3, ...]  // original waypoints
    FOR EACH i, waypoint IN ENUMERATE(waypoints:=mode.path, i:=1) DO
      new_distance = CLAMP(original_distances[i], 0, hit.distance)
      // LERP to find position along origin path shape
      new_position = LERP(original_distances, original_positions, new_distance)
      waypoints[i].position = new_position
      waypoints[i].orientation = AZIMUTH_BETWEEN(waypoints[i-1], waypoints[i])
    END FOR
  END FOR
END
```

## Parameters

| Parameter         | Type   | Default | Unit | Description                                                                  |
| ----------------- | ------ | ------- | ---- | ---------------------------------------------------------------------------- |
| `speed_threshold` | double | 1.0     | m/s  | Refine penetration if the path collides with objects slower than this value. |

## Configuration Example

```yaml
/**:
  ros__parameters:
    processors: [refine_penetration_by_static_objects]
    refine_penetration_by_static_objects:
      speed_threshold: 1.0 # Refine penetration only when the path collides with objects slower than this value.
```
