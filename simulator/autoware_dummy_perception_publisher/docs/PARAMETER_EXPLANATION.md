# Predicted Path Configuration Parameters

This document provides detailed explanations of all configuration parameters used in the dummy object predicted path feature.

## Overview

The parameters control how dummy objects are matched to predicted objects and how their movements are validated. The system uses separate thresholds for vehicles and pedestrians, reflecting their different movement characteristics.

## General Prediction Parameters

### predicted_path_delay

- **Type**: `double`
- **Unit**: seconds
- **Default**: `2.0`
- **Description**: Delay before using predicted paths for newly created dummy objects. During this initial period, objects use straight-line motion to allow the perception system to stabilize before relying on predictions.

### min_keep_duration

- **Type**: `double`
- **Unit**: seconds
- **Default**: `3.0`
- **Description**: Minimum time to keep using the same predicted object before allowing remapping. Once a dummy object is mapped to a predicted object, it must maintain that association for at least this duration. This prevents frequent switching between predictions which could cause erratic movement and provides stability in object tracking.

### max_yaw_change

- **Type**: `double`
- **Unit**: radians
- **Default**: `1.57` (90 degrees)
- **Description**: Maximum acceptable yaw angle change between consecutive predictions. When validating trajectory updates, the system compares the yaw angle at the end of the current prediction with the start of the new prediction. Updates where the angle change exceeds this threshold are rejected to prevent unrealistic sudden turns.

### max_path_length_change_ratio

- **Type**: `double`
- **Unit**: dimensionless ratio
- **Default**: `3.0`
- **Description**: Maximum acceptable ratio between old and new path lengths. When validating trajectory updates, the system calculates the ratio of new path length to old path length. Ratios greater than 3.0 or less than 1/3.0 are rejected to prevent drastic changes in predicted trajectory length which could indicate tracking errors.

## Vehicle-Specific Parameters

### vehicle.max_remapping_distance

- **Type**: `double`
- **Unit**: meters
- **Default**: `2.0`
- **Description**: Maximum distance for initial mapping or remapping a vehicle dummy object to a predicted object. When searching for a predicted object to map to, only candidates within this distance are considered. This applies during initial mapping and when the previous predicted object disappears.

### vehicle.max_remapping_yaw_diff

- **Type**: `double`
- **Unit**: radians
- **Default**: `0.26` (15 degrees)
- **Description**: Maximum yaw difference allowed when remapping vehicles. The system compares the dummy object's current yaw with the candidate predicted object's yaw and rejects candidates with differences exceeding this threshold. Vehicles typically maintain consistent heading, so large yaw differences indicate different objects.

### vehicle.max_speed_difference_ratio

- **Type**: `double`
- **Unit**: dimensionless ratio
- **Default**: `1.05` (5% tolerance)
- **Description**: Maximum speed difference ratio when comparing dummy and predicted speeds. Calculated as abs(dummy_speed - predicted_speed) / dummy_speed. Candidates with ratios exceeding this threshold are rejected to ensure speed compatibility for smooth transitions.

### vehicle.min_speed_ratio / vehicle.max_speed_ratio

- **Type**: `double`
- **Unit**: dimensionless ratio
- **Default**: `0.5` / `1.5`
- **Description**: Acceptable range for predicted_speed / dummy_speed ratio. The predicted object's speed must be between 50% and 150% of the dummy object's speed. This check is only performed when the dummy speed exceeds the speed_check_threshold to allow some speed variation while preventing unrealistic matches.

### vehicle.speed_check_threshold

- **Type**: `double`
- **Unit**: m/s
- **Default**: `1.0`
- **Description**: Minimum speed for applying speed ratio checks. Speed validation is only performed when the dummy object's speed exceeds this threshold. This prevents issues with near-zero speeds where speed ratios become unreliable.

### vehicle.max_position_difference

- **Type**: `double`
- **Unit**: meters
- **Default**: `1.5`
- **Description**: Maximum position difference for path similarity validation. When checking if two predicted paths are similar, the system compares positions at the same time points along the paths. If any position difference exceeds this threshold, the paths are considered dissimilar.

### vehicle.max_path_length_ratio

- **Type**: `double`
- **Unit**: dimensionless ratio
- **Default**: `1.1` (10% difference)
- **Description**: Maximum path length ratio for similarity checks. When comparing total path lengths of two predictions, the ratio must be between 0.9 and 1.1. This ensures that similar paths have comparable lengths.

### vehicle.max_overall_direction_diff

- **Type**: `double`
- **Unit**: radians
- **Default**: `0.52` (30 degrees)
- **Description**: Maximum overall direction difference between paths. The system calculates the angle between vectors from path start to end points. Paths with angular differences exceeding this threshold are considered dissimilar.

## Pedestrian-Specific Parameters

All pedestrian parameters follow the same patterns as vehicle parameters but with more lenient thresholds to account for the unpredictable nature of pedestrian movement:

### pedestrian.max_remapping_distance

- **Type**: `double`
- **Unit**: meters
- **Default**: `3.0`
- **Description**: Same as vehicle parameter but with a larger threshold since pedestrians can appear/disappear more suddenly.

### pedestrian.max_remapping_yaw_diff

- **Type**: `double`
- **Unit**: radians
- **Default**: `0.78` (45 degrees)
- **Description**: Same as vehicle parameter but with a larger threshold since pedestrians can change direction quickly.

### pedestrian.max_speed_difference_ratio

- **Type**: `double`
- **Unit**: dimensionless ratio
- **Default**: `1.3` (30% tolerance)
- **Description**: Same as vehicle parameter but with more tolerance for speed variation.

### pedestrian.min_speed_ratio / pedestrian.max_speed_ratio

- **Type**: `double`
- **Unit**: dimensionless ratio
- **Default**: `0.3` / `2.0`
- **Description**: Same as vehicle parameters but with a wider acceptable range to accommodate variable pedestrian speeds.

### pedestrian.speed_check_threshold

- **Type**: `double`
- **Unit**: m/s
- **Default**: `0.5`
- **Description**: Same as vehicle parameter but with a lower threshold suitable for walking speeds.

### pedestrian.max_position_difference

- **Type**: `double`
- **Unit**: meters
- **Default**: `2.5`
- **Description**: Same as vehicle parameter but allowing more variation for less predictable pedestrian paths.

### pedestrian.max_path_length_ratio

- **Type**: `double`
- **Unit**: dimensionless ratio
- **Default**: `1.5` (50% difference)
- **Description**: Same as vehicle parameter but allowing more variation in path lengths.

### pedestrian.max_overall_direction_diff

- **Type**: `double`
- **Unit**: radians
- **Default**: `1.04` (60 degrees)
- **Description**: Same as vehicle parameter but allowing more direction change for pedestrian movement.

## Usage Flow

1. **Initial Mapping**: When a dummy object with `action: PREDICT` is created, the system finds the nearest predicted object within `max_remapping_distance` these two objects are then mapped to keep track of the predicted path of the dummy object.

2. **Validation**: The candidate must pass all applicable checks:

   - Distance threshold
   - Yaw difference
   - Speed compatibility (if speed exceeds threshold)
   - Path similarity (if remapping from an existing prediction)

3. **Stability**: Once mapped, the relationship is maintained for at least `min_keep_duration` seconds.

4. **Remapping**: If the predicted object disappears, the system searches for a new match using the same validation criteria.

5. **Continuous Validation**: During prediction updates, trajectory changes are validated using `max_yaw_change` and `max_path_length_change_ratio`.

## Parameter Tuning Guidelines

- **Increase thresholds** if objects are failing to find matches or losing tracking too often
- **Decrease thresholds** if incorrect matches are occurring or movement appears unrealistic
- **Vehicle parameters** should remain strict for realistic traffic simulation
- **Pedestrian parameters** can be more lenient due to unpredictable human movement
- Monitor logs for validation failures to identify which parameters need adjustment
