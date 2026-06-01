# autoware_redundancy_adapi_switcher

## Overview

This package gates adapi messages (`MrmState`, `DiagGraphStatus`, `DiagGraphStruct`) output by each ECU in a Main ECU / Sub ECU redundant configuration, based on the content of `active_control_unit`.

Rather than "switching" outputs, the node **passes through messages from the currently active ECU and discards messages from the inactive ECU**. One instance runs on each ECU, and each instance independently determines whether it is the active output master.

This package also manages the launch file for the Sub ECU adapi container (`HeartbeatNode`, `DiagnosticsNode`, `FailSafeNode`).

---

## Architecture

```
               active_control_unit
                      │
        ┌─────────────┴─────────────┐
        ▼                           ▼
[Main ECU]                      [Sub ECU]
RedundancyAdapiSwitcher         RedundancyAdapiSwitcher
  (is_main_ecu=true)              (is_main_ecu=false)
        │                           │
  is_active? ──Yes──▶ publish   is_active? ──Yes──▶ publish
             ──No──▶ discard              ──No──▶ discard
        │                           │
        ▼                           ▼
   MrmState                     MrmState
   DiagStatus        →→→        DiagStatus     (downstream)
   DiagStruct                   DiagStruct
```

An instance becomes the output master when its own ECU ID is present in `active_control_unit.ids`.

---

## Gating Logic

| `active_control_unit.ids` content | Behavior |
|---|---|
| Main ECU ID only | Main ECU instance publishes. Sub is silenced. |
| Sub ECU ID only | Sub ECU instance publishes. Main is silenced. |
| Both ECU IDs | Current master is kept (treated as a transitional state). |
| Empty list | All instances block their output (`output_blocked`). |
| Neither ECU ID | This ECU is assumed faulty; the other ECU becomes the output master. |

### DiagGraphStruct handling

`DiagGraphStruct` uses a `transient_local` QoS. To ensure late-joining subscribers receive the latest structure after a master switch, the most recently received struct is buffered internally and re-published together with the next `DiagGraphStatus`.

---

## Node: `redundancy_adapi_switcher`

### Parameters

| Parameter | Type | Description |
|---|---|---|
| `is_main_ecu` | `bool` | Whether this instance runs on the Main ECU |
| `main_ecu_id` | `uint8` | Main ECU ID (matched against `active_control_unit.ids`) |
| `sub_ecu_id` | `uint8` | Sub ECU ID (matched against `active_control_unit.ids`) |

### Subscriptions

| Topic | Type | QoS | Description |
|---|---|---|---|
| `~/input/mrm_state` | `autoware_adapi_v1_msgs/MrmState` | Reliable, depth 5 | MRM state from this ECU |
| `~/input/diag/status` | `tier4_system_msgs/DiagGraphStatus` | Reliable, depth 5 | Diagnostic status from this ECU |
| `~/input/diag/struct` | `tier4_system_msgs/DiagGraphStruct` | Reliable, depth 1, transient_local | Diagnostic graph structure from this ECU |
| `~/input/active_control_unit` | `tier4_system_msgs/ActiveControlUnit` | Reliable, depth 1, transient_local | Active ECU information |

### Publications

| Topic | Type | QoS | Description |
|---|---|---|---|
| `~/output/mrm_state` | `autoware_adapi_v1_msgs/MrmState` | Reliable, depth 5 | Gated MRM state |
| `~/output/diag/status` | `tier4_system_msgs/DiagGraphStatus` | Reliable, depth 5 | Gated diagnostic status |
| `~/output/diag/struct` | `tier4_system_msgs/DiagGraphStruct` | Reliable, depth 1, transient_local | Gated diagnostic graph structure |

---

## Launch Files

### `redundancy_adapi_switcher.launch.xml`

Launches the `RedundancyAdapiSwitcher` node.

| Argument | Default | Description |
|---|---|---|
| `is_main_ecu` | `true` | Identifies this instance as Main or Sub ECU |
| `input_mrm_state` | `/ecu/system/fail_safe/mrm_state` | Input MRM state topic |
| `input_diag_status` | `/diagnostics_graph/status/merge` | Input diagnostic status topic |
| `input_diag_struct` | `/diagnostics_graph/struct/merge` | Input diagnostic graph structure topic |
| `input_active_control_unit` | `/system/redundancy/active_control_unit` | Active control unit topic |
| `output_mrm_state` | `/system/fail_safe/mrm_state` | Output MRM state topic |
| `output_diag_status` | `/diagnostics_graph/status` | Output diagnostic status topic |
| `output_diag_struct` | `/diagnostics_graph/struct` | Output diagnostic graph structure topic |

### `sub_default_adapi.launch.py`

Launches the Sub ECU adapi container. Loads `HeartbeatNode`, `DiagnosticsNode`, and `FailSafeNode` into a `component_container_mt`.

| Argument | Default | Description |
|---|---|---|
| `config` | `autoware_default_adapi_universe/config/default_adapi.param.yaml` | Path to the adapi parameter file |
