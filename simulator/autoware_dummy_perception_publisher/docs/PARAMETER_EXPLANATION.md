# Predicted Path Configuration Parameters Explained

This document provides detailed explanations of all configuration parameters used in the dummy object predicted path feature.

## Overview

The parameters control how dummy objects are matched to predicted objects and how their movements are validated. The system uses separate thresholds for vehicles and pedestrians, reflecting their different movement characteristics.

## General Prediction Parameters

### `predicted_path_delay` (2.0 seconds)
**Status**: Declared but not actively used (TODO in code)
**Purpose**: Intended to add a delay before using predicted paths for newly created dummy objects
**Rationale**: Allows the perception system to stabilize before relying on predictions

### `min_keep_duration` (3.0 seconds)
**Used in**: `findMatchingPredictedObject()` at line 786
**Purpose**: Minimum time to keep using the same predicted object before allowing remapping
**How it works**:
- Once a dummy object is mapped to a predicted object, it must use that prediction for at least 3 seconds
- Prevents frequent switching between predictions which could cause erratic movement
**Rationale**: Provides stability in object tracking and prevents jittery behavior

### `max_yaw_change` (1.57 radians = 90°)
**Used in**: `isTrajectoryValid()` at line 1181
**Purpose**: Maximum acceptable yaw angle change between consecutive predictions
**How it works**:
- Compares the yaw angle at the end of the current prediction with the start of the new prediction
- Rejects updates where the angle change exceeds 90°
**Rationale**: Prevents unrealistic sudden turns (e.g., a car instantly reversing direction)

### `max_path_length_change_ratio` (3.0)
**Used in**: `isTrajectoryValid()` at line 1214
**Purpose**: Maximum acceptable ratio between old and new path lengths
**How it works**:
- Calculates the ratio of new path length to old path length
- Rejects if ratio > 3.0 or < 1/3.0
**Rationale**: Prevents drastic changes in predicted trajectory length which could indicate tracking errors

## Vehicle-Specific Parameters

### `vehicle.max_remapping_distance` (2.0 meters)
**Used in**: `isValidRemappingCandidate()` at line 1251
**Purpose**: Maximum distance for initial mapping or remapping a vehicle dummy to a predicted object
**How it works**:
- When searching for a predicted object to map to, only considers those within 2 meters
- Applied during initial mapping and when the previous predicted object disappears
**Rationale**: Vehicles are more predictable and should be matched to nearby predictions only

### `vehicle.max_remapping_yaw_diff` (0.26 radians = 15°)
**Used in**: `isValidRemappingCandidate()` at line 1253
**Purpose**: Maximum yaw difference allowed when remapping vehicles
**How it works**:
- Compares the dummy object's current yaw with the candidate predicted object's yaw
- Rejects candidates with > 15° difference
**Rationale**: Vehicles typically maintain consistent heading; large yaw differences indicate different objects

### `vehicle.max_speed_difference_ratio` (1.05 = 5% tolerance)
**Used in**: `isValidRemappingCandidate()` at line 1255
**Purpose**: Maximum speed difference ratio when comparing dummy and predicted speeds
**How it works**:
- Calculates abs(dummy_speed - predicted_speed) / dummy_speed
- Rejects if ratio > 0.05 (5%)
**Rationale**: Ensures speed compatibility for smooth transitions

### `vehicle.min_speed_ratio` / `vehicle.max_speed_ratio` (0.5 - 1.5)
**Used in**: `isValidRemappingCandidate()` at lines 1276-1278
**Purpose**: Acceptable range for predicted_speed / dummy_speed ratio
**How it works**:
- Predicted speed must be between 50% and 150% of dummy speed
- Only checked when dummy speed > `speed_check_threshold`
**Rationale**: Allows some speed variation while preventing unrealistic matches

### `vehicle.speed_check_threshold` (1.0 m/s)
**Used in**: `isValidRemappingCandidate()` at line 1280
**Purpose**: Minimum speed for applying speed ratio checks
**How it works**:
- Speed validation is only performed when dummy speed > 1.0 m/s
- Prevents issues with near-zero speeds
**Rationale**: Speed ratios become unreliable at very low speeds

### `vehicle.max_position_difference` (1.5 meters)
**Used in**: `arePathsSimilar()` at line 1436
**Purpose**: Maximum position difference for path similarity validation
**How it works**:
- When checking if two predicted paths are similar, compares positions at same time points
- Rejects if any position difference > 1.5m
**Rationale**: Ensures spatial consistency between consecutive predictions

### `vehicle.max_path_length_ratio` (1.1 = 10% difference)
**Used in**: `arePathsSimilar()` at line 1529
**Purpose**: Maximum path length ratio for similarity checks
**How it works**:
- Compares total path lengths of two predictions
- Rejects if ratio > 1.1 or < 0.9
**Rationale**: Similar paths should have similar lengths

### `vehicle.max_overall_direction_diff` (0.52 radians = 30°)
**Used in**: `arePathsSimilar()` at line 1572
**Purpose**: Maximum overall direction difference between paths
**How it works**:
- Calculates angle between vectors from path start to end
- Rejects if angle > 30°
**Rationale**: Ensures paths have similar overall direction

## Pedestrian-Specific Parameters

All pedestrian parameters follow the same patterns as vehicle parameters but with more lenient thresholds:

### Key Differences:
- **max_remapping_distance**: 3.0m (vs 2.0m) - Pedestrians can appear/disappear more suddenly
- **max_remapping_yaw_diff**: 45° (vs 15°) - Pedestrians can change direction quickly
- **max_speed_difference_ratio**: 1.3 (vs 1.05) - More speed variation allowed
- **speed_ratio range**: 0.3-2.0 (vs 0.5-1.5) - Pedestrians have more variable speeds
- **speed_check_threshold**: 0.5 m/s (vs 1.0 m/s) - Lower threshold for walking speeds
- **max_position_difference**: 2.5m (vs 1.5m) - Less predictable paths
- **max_path_length_ratio**: 1.5 (vs 1.1) - More variation in path lengths
- **max_overall_direction_diff**: 60° (vs 30°) - Pedestrians can change direction more

## Usage Flow

1. **Initial Mapping**: When a dummy object with `action: PREDICT` is created, the system finds the nearest predicted object within `max_remapping_distance`

2. **Validation**: The candidate must pass all checks:
   - Distance threshold
   - Yaw difference
   - Speed compatibility
   - Path similarity (if remapping)

3. **Stability**: Once mapped, the relationship is maintained for at least `min_keep_duration`

4. **Remapping**: If the predicted object disappears, the system searches for a new match using the same validation criteria

5. **Continuous Validation**: During updates, trajectory changes are validated using `max_yaw_change` and `max_path_length_change_ratio`

## Tuning Guidelines

- **Increase thresholds** if objects are failing to find matches or losing tracking too often
- **Decrease thresholds** if incorrect matches are occurring or movement appears unrealistic
- **Vehicle parameters** should remain strict for realistic traffic simulation
- **Pedestrian parameters** can be more lenient due to unpredictable human movement
- Monitor logs for validation failures to identify which parameters need adjustment