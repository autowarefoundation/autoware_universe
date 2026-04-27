# Boundary Departure Checker

## 1. Introduction

The Boundary Departure Checker is a trajectory validation module designed to prevent the autonomous vehicle from crossing uncrossable road boundaries. It continuously evaluates the ego vehicle's predicted path and outputs a departure severity status to the planning system to ensure safe lane following.

## 2. Input and Output

### Input

- **Vehicle Information (`vehicle_info`):** Used to generate geometric footprints and filter boundaries based on the vehicle's elevation.
- **Kinematic State (`/localization/kinematic_state`):** Provides ego vehicle velocity, used to calculate the minimum physical braking distance.
- **Acceleration (`/localization/acceleration`):** Used alongside velocity to calculate the minimum braking distance.
- **Candidate Trajectories (`/planning/generator/concatenated/candidate_trajectories`):** The predicted paths to be evaluated, requiring accurate time-from-start values.
- **Vector Map (`/map/vector_map`):** The Lanelet2 map used to extract uncrossable boundary locations, such as `road_border`.

### Output

- **Status:** The departure severity classification (`NONE`, `APPROACHING_DEPARTURE`, or `CRITICAL_DEPARTURE`).
- **Score:** A numerical value used for trajectory cost evaluation.
- **Processing Time:** Execution time in milliseconds for system monitoring.
- **Debug Markers:** Visualizations for the ego footprint, boundaries, and geometric projections.

## 3. What the Module Does

The module evaluates whether the ego vehicle's predicted trajectory will safely stay within the road boundaries. It splits the vehicle's footprint into distinct left and right sides to evaluate the environment asymmetrically. By computing the geometric intersection between the vehicle's predicted footprint and mapped uncrossable boundaries, it classifies the trajectory's safety based on physical braking limits and predefined time thresholds. It also applies time-based hysteresis to prevent status flickering caused by noisy trajectory predictions.

## 4. Parameters

The module uses a structured parameter configuration to define thresholds, footprint margins, and buffer times. Below is a high-level representation of the parameter schema:

{{ json_to_markdown("common/autoware_boundary_departure_checker/schema/boundary_departure_checker.schema.json") }}

## 5. Process Overview

1. **Footprint Generation:** The module generates geometric footprints for the ego vehicle at every point along the candidate trajectory. It expands the footprint size using margins to account for localization uncertainty.
2. **Boundary Extraction and Filtering:** Uncrossable boundaries (e.g., `road_border`) are extracted from the Lanelet2 map and stored in a spatial R-tree. Boundaries significantly above or below the vehicle's Z-axis height are filtered out to prevent false positives from overpasses.
3. **Distance Calculation:** The system calculates the shortest lateral distance from the left and right footprint segments to the nearest filtered boundary.
4. **Severity Evaluation:** A minimum physical braking distance is dynamically calculated using current speed, acceleration, maximum allowed deceleration, jerk, and brake delay. The departure severity is assigned as follows:
   - **NONE:** The footprint lateral distance is greater than the critical lateral margin.
   - **APPROACHING:** A departure is detected, but it is farther than the braking distance AND the time to departure is greater than the cutoff threshold.
   - **CRITICAL:** A departure is detected within the braking distance OR before the cutoff time expires.
5. **Hysteresis Logic:** To ensure stability, an **ON-Time buffer** suppresses the `CRITICAL` state until the departure is continuously detected for a set duration. If a collision is imminent, this buffer is bypassed. An **OFF-Time buffer** ensures the status does not revert to `NONE` until the trajectory is continuously evaluated as safe.

## 6. Possible Scenarios and Expected Behavior

- **Scenario 1:** The ego vehicle's footprint overlaps with a map boundary that does not have the `road_border` tag (or other defined uncrossable tags).
  - **Expected Behavior:** Evaluated as safe (`NONE`).
- **Scenario 2:** The ego vehicle's footprint does not overlap the defined lateral gap to the `road_border`.
  - **Expected Behavior:** Evaluated as safe (`NONE`).
- **Scenario 3:** The footprint overlaps the lateral gap to the `road_border`, AND the arc length to the overlap is less than the minimum braking distance, AND the time to reach it is less than the cutoff time.
  - **Expected Behavior:** Evaluated as a departure (`CRITICAL_DEPARTURE`).
- **Scenario 4:** The footprint overlaps the lateral gap, and the time to reach the overlap is less than the cutoff time, even if the arc length is greater than the minimum braking distance.
  - **Expected Behavior:** Evaluated as a departure (`CRITICAL_DEPARTURE`).
- **Scenario 5:** The footprint overlaps the lateral gap, and the arc length to the overlap is less than the minimum braking distance, even if the time to reach the overlap exceeds the cutoff time.
  - **Expected Behavior:** Evaluated as a departure (`CRITICAL_DEPARTURE`).
