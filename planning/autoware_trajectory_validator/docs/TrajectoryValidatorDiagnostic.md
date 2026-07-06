# TrajectoryValidatorDiagnostic

`TrajectoryValidatorDiagnostic` reads the `ValidationReport` array produced by the validator node every cycle, decides how severe the situation is, and publishes a set of named `DiagnosticStatus` values that a downstream diagnostic graph can read to choose a safety action.

---

## Concepts

### RiskLevel

Each filter plugin rates every candidate trajectory and writes one `MetricReport` per (candidate, filter) pair.
The `MetricReport.risk.level` field carries a `RiskLevel` value:

| `MetricReport.risk.level` | Meaning                      |
| ------------------------- | ---------------------------- |
| `SAFE`                    | no concern                   |
| `LOW_CAUTION`             | minor concern, treated as OK |
| `HIGH_CAUTION`            | notable concern              |
| `DANGER`                  | trajectory at risk           |
| `FATAL`                   | critical failure             |

### Action enum

Internally, `TrajectoryValidatorDiagnostic` works with a four-step `Action` enum.
`convert_risk_level_to_action()` translates a `RiskLevel` into an `Action`:

| `MetricReport.risk.level` | → `Action`    |
| ------------------------- | ------------- |
| `SAFE`                    | `NONE`        |
| `LOW_CAUTION`             | `NONE`        |
| `HIGH_CAUTION`            | `COMFORTABLE` |
| `DANGER`                  | `MODERATE`    |
| `FATAL`                   | `EMERGENCY`   |

**`LOW_CAUTION` maps to `NONE`, the same as `SAFE`.** A candidate whose worst filter level is `LOW_CAUTION` is treated identically to a fully-clean candidate: it beats any candidate that has a `DANGER` or higher level.

`convert_risk_level_to_action()` in `risk_action.hpp` is the **only** place these constants are named, so updating the scale means editing one function.

### DiagnosticStatus published level

Each `Action` maps to a `DiagnosticStatus` level:

| `Action`      | published level |
| ------------- | --------------- |
| `NONE`        | `OK`            |
| `COMFORTABLE` | `ERROR`         |
| `MODERATE`    | `ERROR`         |
| `EMERGENCY`   | `ERROR`         |

---

## How a cycle works

The class is **stateless across cycles** — every cycle it recomputes from scratch and re-publishes everything.

```text
ValidationReport[]  (one entry per candidate trajectory)
        │
        ▼
1. For each active filter, find its best (minimum) Action across all candidates
        │
        ▼
2. For each filter whose best Action > NONE:
   fire the configured_action whose threshold exactly matches the filter's best Action
   (a filter with both MODERATE and EMERGENCY configured_actions fires only MODERATE at MODERATE,
   and only EMERGENCY at EMERGENCY — each level triggers an independent diagnostic response)
        │
        ▼
3. Publish ALL preset statuses every cycle:
   - active ones at ERROR
   - inactive and shadow-mode ones at OK
```

**Why publish everything every cycle?** A `DiagnosticStatus` that stops being sent goes stale and the diagnostic graph may interpret the silence as a hardware fault. Sending inactive statuses at `OK` prevents false alarms.

**Why per-filter minimum?** Each filter is evaluated independently. If a filter passes on at least one candidate, it does not fire — there is a safe trajectory available for that check. Only filters that fail on every candidate raise a diagnostic.

**LOW_CAUTION vs DANGER example.** Two candidates — A has one filter at `LOW_CAUTION`, B has one filter at `DANGER`:

| Candidate | filter_x level | Action                            |
| --------- | -------------- | --------------------------------- |
| A         | `LOW_CAUTION`  | `NONE` (LOW_CAUTION maps to NONE) |
| B         | `DANGER`       | `MODERATE`                        |

filter_x best action = `NONE` (minimum of `NONE` and `MODERATE`). No status fires — the filter passes on at least one candidate.

### Pattern reference

**Within one candidate — `std::max` (harshest metric governs)**

| Pattern      | Sequence                                               | Result    |
| ------------ | ------------------------------------------------------ | --------- |
| min only     | NONE                                                   | NONE      |
| max only     | EMERGENCY                                              | EMERGENCY |
| min then max | emplace NONE → `max(NONE, EMERGENCY)` = EMERGENCY      | EMERGENCY |
| max then min | emplace EMERGENCY → `max(EMERGENCY, NONE)` = EMERGENCY | EMERGENCY |

Order does not matter — `std::max` is commutative.

**Across candidates — `std::min` (safest outcome governs)**

| Pattern              | Sequence                                          | Result    |
| -------------------- | ------------------------------------------------- | --------- |
| min only             | NONE                                              | NONE      |
| max only             | EMERGENCY                                         | EMERGENCY |
| min first, max later | emplace NONE → `min(NONE, EMERGENCY)` = NONE      | NONE      |
| max first, min later | emplace EMERGENCY → `min(EMERGENCY, NONE)` = NONE | NONE      |

Order does not matter — `std::min` is commutative.

#### Combined two-stage example

```text
Candidate A: metric at DANGER       →  candidate_action = MODERATE  (max within A)
Candidate B: metric at LOW_CAUTION  →  candidate_action = NONE      (max within B, LOW_CAUTION maps to NONE)

filter_current_action after A: MODERATE
filter_current_action after B: min(MODERATE, NONE) = NONE
```

No configured_action fires — there is a safe candidate available (B).

### No candidate trajectories

When the `ValidationReport` array is empty (the generator produced no candidates), the class publishes the `no_candidates_diag_status_name` status at `ERROR`. This name is fixed as `"trajectory_validator_no_candidate_trajectory"` in `TrajectoryValidatorWrapper`.

### Shadow-mode filters

Filters loaded via `shadow_mode_filter_names` are excluded from the Action aggregation entirely. Their `MetricReport` entries still appear in the published `ValidationReportArray` topic (for debug), but they do not influence which trajectory is chosen as best. Their configured_action status names are published at `OK` every cycle.

The wrapper identifies shadow filters by calling `plugin->get_name()` on plugins with `is_shadow_mode() == true` and passing the resulting names as `active_filter_names` to the class. Any filter name **not** in `active_filter_names` is treated as shadow.

---

## Configured actions parameter

The mapping from `(filter_name, Action)` to a status name is configured in:

```text
config/trajectory_validator_diagnostic.param.yaml
```

```yaml
trajectory_validator_diagnostic:
  configured_actions:
    - uncrossable_boundary_departure_filter:moderate:trajectory_validator_uncrossable_boundary_departure_danger
```

Each entry is a colon-separated triple:

```text
<filter_name>:<action>:<diagnostic_status_name>
```

| Field                    | Description                                                                                                  |
| ------------------------ | ------------------------------------------------------------------------------------------------------------ |
| `filter_name`            | the name returned by `plugin->get_name()` — lowercase snake_case                                             |
| `action`                 | one of `none`, `comfortable`, `moderate`, `emergency`                                                        |
| `diagnostic_status_name` | the name published in `DiagnosticStatus.name` (without the node-name prefix added by `DiagnosticsInterface`) |

The class creates one `DiagnosticsInterface` publisher for each distinct `diagnostic_status_name` found in the configured_actions, plus one for `no_candidates_diag_status_name`. This preset is built once at startup and never changes at runtime.

### Finding the filter name

The filter name is what `plugin->get_name()` returns, **not** the plugin class name used in `filter_names` / `shadow_mode_filter_names`. The two differ in naming convention:

| Parameter value                                | `get_name()` value                        |
| ---------------------------------------------- | ----------------------------------------- |
| `"safety::UncrossableBoundaryDepartureFilter"` | `"uncrossable_boundary_departure_filter"` |

To find the filter name for any plugin, either check its `get_name()` implementation or run the node and look at the `ValidationReportArray` topic — the `validator_name` field in each `MetricReport` is the filter name.

---

## How to add a new diagnostic

**Step 1 — identify the filter name and the action level.**

Run the node (or check the plugin source) to find the filter name. Decide which `Action` level should fire the new status. Today only `moderate` is in use.

**Step 2 — choose a diagnostic status name.**

Pick a descriptive name that the diagnostic graph will reference. Convention: `trajectory_validator_<filter>_<tier>`, e.g. `trajectory_validator_collision_check_danger`.

**Step 3 — add a line to `trajectory_validator_diagnostic.param.yaml`.**

```yaml
trajectory_validator_diagnostic:
  configured_actions:
    - uncrossable_boundary_departure_filter:moderate:trajectory_validator_uncrossable_boundary_departure_danger
    - collision_check_filter:moderate:trajectory_validator_collision_check_danger
```

No code change is needed. The class reads `configured_actions` at startup and creates the publisher automatically.

**Step 4 — wire the new name in the diagnostic graph.**

The full status name seen on `/diagnostics` is `"<node_name>: <diagnostic_status_name>"`. Add a node in the diagnostic graph that listens to this full name and maps it to the intended safety action.

---

## Data-flow diagram

```text
filter plugins
    │  MetricReport { validator_name, risk.level }
    ▼
ValidationReport[]  ──────────────────────────────────────────────┐
    │                                                              │
    │  TrajectoryValidatorDiagnostic.update_and_publish()          │  (published as
    │                                                              │   ValidationReportArray
    │  1. compute per-candidate worst Action per filter            │   topic — unaffected
    │     (skip shadow filters)                                    │   by this class)
    │  2. accumulate per-filter minimum Action across candidates   │
    │  3. for each filter with Action != NONE,                     │
    │     fire the configured_action that exactly matches          │
    │  4. publish all statuses (triggered at ERROR, rest at OK)    │
    │                                                              │
    ▼                                                              │
/diagnostics  ◄────────────────────────────────────────────────────┘
    │
    ▼
diagnostic graph → chooses safety action (comfortable / moderate / emergency stop)
```

The validator report published on the ROS topic is **not modified** by this class — it always reflects what every plugin (including shadow-mode ones) produced. The diagnostic only looks at the report; it does not filter it.
