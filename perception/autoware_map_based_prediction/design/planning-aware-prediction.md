# Planning-Aware Prediction Fidelity Allocation

## Motivation

`map_based_prediction` processes every tracked object uniformly: each vehicle goes through
lanelet search, maneuver detection, and multi-mode reference path generation, regardless of
whether the object can ever interact with the ego vehicle's plan. In dense traffic this
uniform treatment dominates the node's processing time — the package already ships a
processing-time diagnostic (`processing_time_tolerance`) precisely because this is a known
operational concern.

Recent end-to-end driving research reached state-of-the-art results by inverting the
conventional "perception → planning" data flow. VeteranAD ("Perception in Plan: Coupled
Perception and Planning for End-to-End Autonomous Driving", AAAI 2026) uses planning priors
(anchored trajectories) to guide perception, concentrating computation on the traffic
elements along the intended path instead of processing the whole scene uniformly.

This design generalizes that idea to Autoware's modular architecture:

> Do not spend equal computation on information irrelevant to driving. Let the planning
> intent decide where perception fidelity is needed.

In a modular stack the "planning intent" is readily available: the previous planning cycle's
trajectory. This feature feeds it back into prediction and grades the prediction fidelity per
object.

## Architecture

```text
                       +--------------------------------------+
   /localization/      |        map_based_prediction          |
   kinematic_state --->|  +--------------------------------+  |
                       |  |      RelevanceClassifier       |  |
   /planning/          |  |  corridor distance + approach  |  |
   scenario_planning/ ->|  |  test + per-object hysteresis  |  |
   trajectory          |  +---------------+----------------+  |
                       |                  |                   |
   tracking/objects -->|   HIGH relevance | LOW relevance     |
                       |        v         v                   |
                       |  full lanelet-   constant-velocity   |
                       |  based multi-    single-mode path,   |
                       |  mode prediction short horizon       |
                       +--------------------------------------+
```

- `RelevanceClassifier` (`relevance_classifier.hpp` / `lib/relevance_classifier.cpp`) is a
  pure-logic component with no ROS dependencies beyond message types, so it is unit-testable
  and reusable by other nodes.
- The classification is evaluated only for vehicle classes (car, bus, trailer, motorcycle,
  truck) inside `ObjectsCallback::objectsCallback()`.

### Relevance rules

An object is **HIGH** relevance if any of the following holds:

1. It is within `always_relevant_radius` of the ego vehicle.
2. Its lateral distance to the planned trajectory polyline is smaller than
   `lateral_margin_base + lateral_margin_rate * s`, where `s` is the arc length of the
   nearest trajectory point. The widening corridor reflects the growing uncertainty of the
   plan further ahead, analogous to how VeteranAD focuses precisely on near-future guide
   points and more loosely on far-future ones.
3. Its velocity vector points toward the corridor and the time to reach it is below
   `time_to_corridor_threshold` (cut-in / crossing precaution).

Everything else is **LOW** relevance and receives `PathGenerator::generatePathForNonVehicleObject()`
(a constant-velocity straight path — the same generator already used for unknown-class
objects) with the shorter `low_fidelity_time_horizon`.

## Safety design

This feature allocates fidelity; it never drops objects.

- **Default off.** `planning_aware_prediction.enable` defaults to `false`; the default
  behavior of the node is bit-identical to the previous implementation.
- **Fail-safe.** If the ego trajectory has not been received, has fewer than 2 points, or is
  older than `ego_trajectory_timeout`, every object is classified HIGH — identical to the
  feature being disabled. A planning outage therefore degrades gracefully to the previous
  behavior.
- **Asymmetric hysteresis.** Promotion LOW → HIGH is immediate. Demotion HIGH → LOW requires
  `demote_frame_count` consecutive LOW classifications. This prevents mode flapping and errs
  on the side of full fidelity.
- **VRUs are exempt.** Pedestrians and bicycles always receive their regular prediction:
  they are safety-critical, and skipping `PredictorVru::predict()` would interfere with the
  crosswalk-user history bookkeeping (`retrieveUndetectedObjects()` re-emits history entries
  that were not predicted in the current frame).
- **Downstream compatibility.** LOW relevance vehicles still appear in the output with a
  valid single-mode path (`confidence = 1.0`), so downstream planning modules keep seeing
  every object.

## Debugging

With `publish_debug_markers: true` and the feature enabled, a sphere marker is published per
vehicle in the `relevance` namespace: green = HIGH, gray = LOW.

For benchmarking, use `publish_processing_time_detail: true` and compare
`~/debug/processing_time_detail_ms` with the feature enabled and disabled on the same rosbag.

## Future work (generalization roadmap)

The relevance interface deliberately depends only on the planned trajectory and object
kinematics, so the same classification can guide other perception stages:

- **Prediction (this package):** graded maneuver-mode count and horizon instead of the
  current binary fidelity split; multi-candidate corridors once planners publish candidate
  trajectories.
- **Detection:** prioritized ROI selection for camera detectors along the projected
  corridor; higher-rate processing of corridor-relevant point cloud regions.
- **Tracking:** measurement-update rate allocation by relevance.
- **Occupancy grid:** finer resolution or higher update rates inside the corridor.

Promoting `RelevanceClassifier` to a shared package is intentionally deferred until a second
consumer exists.

## References

1. B. Zhang, J. Li, N. Song, L. Zhang, "Perception in Plan: Coupled Perception and Planning
   for End-to-End Autonomous Driving," AAAI 2026.
