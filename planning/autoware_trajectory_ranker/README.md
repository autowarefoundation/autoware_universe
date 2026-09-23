# Trajectory Ranker

## Purpose/Role

This package provides a C++ library that scores candidate trajectories and selects the best one. Ranking combines three independently configurable terms:

1. **Safety** — penalty from the validator risk level (`RiskLevel`)
2. **Source** — penalty from the trajectory generator (diffusion vs backup planners)
3. **Quality** — optional metric-plugin evaluation (comfort / progress / consistency)

The library can be embedded into a calling ros node (such as `autoware_trajectory_selector`) via `TrajectoryRankerWrapper`.

## Architecture

| Component                 | Role                                                                                          |
| ------------------------- | --------------------------------------------------------------------------------------------- |
| `TrajectoryRankerWrapper` | Owns parameters, metric `Evaluator`, and `TrajectoryRanker`; entry point used by the selector |
| `TrajectoryRanker`        | Applies safety / source / quality penalties and computes the final scalar score               |
| `Evaluator`               | Loads and runs quality metric plugins (only when `evaluation.enable` is true)                 |

## Algorithm Overview

For each call to `TrajectoryRanker::process()`:

1. **Safety evaluation** (`safety.enable`): map each trajectory's `RiskLevel` to a penalty via `safety.levels` / `safety.penalty`.
2. **Source evaluation** (`source.enable`): map each trajectory's `TrajectorySource` to a penalty via `source.levels` / `source.penalty`.
3. **Quality evaluation** (`evaluation.enable`, default off): resample each trajectory relative to ego odometry, run the configured metric plugins, and set `quality_penalty = 1 - quality_score`. Skipped if route handler or odometry is unavailable.
4. **Score aggregation**:
   \[
   \mathrm{score} = 1 - \frac{
   s*{\mathrm{safety}}\,p*{\mathrm{safety}}
   - s*{\mathrm{source}}\,p*{\mathrm{source}}
   - s*{\mathrm{eval}}\,p*{\mathrm{quality}}
     }{
     s*{\mathrm{safety}} + s*{\mathrm{source}} + s*{\mathrm{eval}}
     }
     \]
     where \(s*\_\) are the configured scales and \(p\_\_\) are the per-trajectory penalties (zero when that term is disabled).
5. **Best selection**: the trajectory with the highest score is retained as `best_trajectory_info` and used to update trajectory history / previous points for subsequent quality metrics.

### Trajectory sources

| `TrajectorySource`    | Typical generator name prefix  | Default source level string |
| --------------------- | ------------------------------ | --------------------------- |
| `DIFFUSION_PLANNER`   | `DiffusionPlanner_`            | `diffusion_planner`         |
| `BACKUP_PLANNER_GO`   | `MinimumRuleBasedPlanner_Go`   | `backup_planner_go`         |
| `BACKUP_PLANNER_STOP` | `MinimumRuleBasedPlanner_Stop` | `backup_planner_stop`       |

Source mapping from generator names is performed in the calling node (e.g `trajectory_selector_node`) before calling the ranker.

### Default penalties (from `config/trajectory_ranker.param.yaml`)

| Term    | Levels / order                                                  | Penalties                         | Scale  | Enabled by default |
| ------- | --------------------------------------------------------------- | --------------------------------- | ------ | ------------------ |
| Safety  | `safe`, `low_caution`, `high_caution`, `danger`, `fatal`        | `0.0`, `0.1`, `0.4`, `1.0`, `1.0` | `20.0` | yes                |
| Source  | `diffusion_planner`, `backup_planner_go`, `backup_planner_stop` | `0.0`, `0.45`, `0.5`              | `5.0`  | yes                |
| Quality | metric plugins under `evaluation.plugin_names`                  | derived from metric score         | `1.0`  | **no**             |

## Quality Metrics

Used only when `evaluation.enable` is true. Plugins are loaded by `Evaluator` from `evaluation.plugin_names`.

| Metric                    | Type         | Description                              | Class name                                                    |
| ------------------------- | ------------ | ---------------------------------------- | ------------------------------------------------------------- |
| **TravelDistance**        | Maximization | Progress along the trajectory            | `autoware::trajectory_ranker::metrics::TravelDistance`        |
| **LateralAcceleration**   | Deviation    | Lateral acceleration comfort             | `autoware::trajectory_ranker::metrics::LateralAcceleration`   |
| **LongitudinalJerk**      | Deviation    | Longitudinal jerk smoothness             | `autoware::trajectory_ranker::metrics::LongitudinalJerk`      |
| **LateralDeviation**      | Deviation    | Deviation from preferred lane centerline | `autoware::trajectory_ranker::metrics::LateralDeviation`      |
| **SteeringConsistency**   | Deviation    | Consistency vs previous steering command | `autoware::trajectory_ranker::metrics::SteeringConsistency`   |
| **TrajectoryConsistency** | Deviation    | Consistency vs recent best trajectories  | `autoware::trajectory_ranker::metrics::TrajectoryConsistency` |

- **Maximization**: higher raw values are better
- **Deviation**: lower raw values are better

When quality evaluation is enabled, trajectories are resampled to `evaluation.sampling_number` points at `evaluation.sampling_resolution` seconds relative to the current ego pose.

## Interface

This package is primarily a C++ library consumed by `trajectory_selector_node`. The selector owns the ROS I/O and passes:

- `RankerInputTrajectories` — each entry has a `CandidateTrajectory`, validator `risk_level`, and `TrajectorySource`
- `RankerContext` — odometry, route handler, and generator info (needed for quality metrics and output generator metadata)

### Selector-side topics

| Direction | Topic name                     | Message Type                                                      | Description                                  |
| --------- | ------------------------------ | ----------------------------------------------------------------- | -------------------------------------------- |
| Publisher | `~/output/scored_trajectories` | `autoware_internal_planning_msgs/msg/ScoredCandidateTrajectories` | Scored candidates from the integrated ranker |

## CAMP Fixed-Weight Scoring

The package also provides the scoring core for CAMP (Conic Atom Meta-Policy), a
decision-layer ranker for an ordered pool produced by a frozen trajectory
generator. CAMP does not modify or regenerate trajectories. It applies learned
nonnegative weights to 16 deployment-observable trajectory atoms and selects
the lowest-cost original candidate.

The bundled `config/camp_v26_k8_50k.json` model contains the K=8 fixed-weight
CAMP parameters, atom scales, and endpoint-status patterns. Raw observed atoms
are normalized as `clip(raw / scale, 0, 10)`. `not_applicable` and
`typed_missing` atoms remain inactive rather than being replaced with zero.
Candidate ties retain the original ordering.

```cpp
const auto model = load_camp_fixed_weight_model(model_path);
const auto ranking = rank_camp_candidates(model, status_pattern, raw_candidate_atoms);
const auto selected_row = ranking.selected_index;
```

The scoring API consumes materialized atom vectors separately from ROS
messages. `autoware_diffusion_planner` provides the integrated adapter: it
constructs the atoms from the original K=8 batch output and current map/planner
context, invokes this scoring core, and publishes the selected original row.
The training code and original deployment bundle are available in the
[CAMP repository](https://github.com/Fake-fate11/camp-core).

This adapter uses the version-1 Fixed export and keeps its K=8 generation
contract. The separate CAMP library also offers raw-affine Scene scoring and
explicit exports for other pool sizes. Those capabilities are not enabled by
this ROS adapter: Scene requires the matching frozen encoder embedding, and a
different K requires an explicitly matched generation/export configuration.
Do not reinterpret the bundled K=8 weights as a new training condition or
substitute a zero embedding for the required Scene input.

## Parameters

{{ json_to_markdown("planning/autoware_trajectory_ranker/schema/trajectory_ranker.schema.json") }}
