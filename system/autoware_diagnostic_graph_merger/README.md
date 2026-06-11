# autoware_diagnostic_graph_merger

## Purpose

This package merges two `DiagGraphStruct` / `DiagGraphStatus` streams into a single unified graph.
It is intended for redundant ECU configurations where two independent ECUs each publish their own diagnostic graph, and a downstream node (e.g. MRM decision maker) requires a single merged view of the entire system health.

## Design

### Struct merge

`DiagGraphStruct` describes the static topology of the diagnostic graph (nodes, diagnostics, links).
When both inputs have published their struct, the node concatenates them into one struct.
Node indices from the second input are shifted by the size of the first graph so that every index in the merged struct remains unique.

```
merged.nodes = struct1.nodes + struct2.nodes
merged.diags = struct1.diags + shift(struct2.diags, offset)
merged.links = struct1.links + shift(struct2.links, offset)
merged.id    = struct1.id + "+" + struct2.id
```

The merged struct is re-published whenever either input struct is updated (transient-local QoS, so late subscribers receive the latest).

### Status merge

`DiagGraphStatus` carries the runtime health values (one entry per node / diagnostic leaf in the struct).
On every timer tick the node checks:

1. Both structs have been received.
2. Both statuses have been received.
3. Each status ID matches the corresponding struct ID (guards against stale data from a restart).

If all conditions are met the statuses are concatenated in the same order as the structs and published.

## Inputs / Outputs

### Inputs

| Topic               | Type                                    | QoS             | Description                              |
| ------------------- | --------------------------------------- | --------------- | ---------------------------------------- |
| `~/input1/struct`   | `tier4_system_msgs/msg/DiagGraphStruct` | Transient-local | Graph topology from ECU 1                |
| `~/input1/status`   | `tier4_system_msgs/msg/DiagGraphStatus` | Reliable        | Runtime health status from ECU 1         |
| `~/input2/struct`   | `tier4_system_msgs/msg/DiagGraphStruct` | Transient-local | Graph topology from ECU 2                |
| `~/input2/status`   | `tier4_system_msgs/msg/DiagGraphStatus` | Reliable        | Runtime health status from ECU 2         |

### Outputs

| Topic               | Type                                    | QoS             | Description                              |
| ------------------- | --------------------------------------- | --------------- | ---------------------------------------- |
| `~/output/struct`   | `tier4_system_msgs/msg/DiagGraphStruct` | Transient-local | Merged graph topology                    |
| `~/output/status`   | `tier4_system_msgs/msg/DiagGraphStatus` | Reliable        | Merged runtime health status             |

## Parameters

| Parameter | Type   | Default | Description                              |
| --------- | ------ | ------- | ---------------------------------------- |
| `rate`    | double | `10.0`  | Status publish rate [Hz]                 |

## Launch

```xml
<include file="$(find-pkg-share autoware_diagnostic_graph_merger)/launch/merge.launch.xml">
  <arg name="~/input1/struct" value="/main/diagnostics_graph/struct"/>
  <arg name="~/input1/status" value="/main/diagnostics_graph/status"/>
  <arg name="~/input2/struct" value="/sub/diagnostics_graph/struct"/>
  <arg name="~/input2/status" value="/sub/diagnostics_graph/status"/>
  <arg name="~/output/struct" value="/diagnostics_graph/struct"/>
  <arg name="~/output/status" value="/diagnostics_graph/status"/>
</include>
```

