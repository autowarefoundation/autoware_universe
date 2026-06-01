# autoware_redundancy_switcher_interface_plugins

`IAdapterPlugin` implementations for [`autoware_redundancy_switcher_interface`](../autoware_redundancy_switcher_interface/).

This package provides three adapters that can be loaded at runtime via pluginlib:

| Adapter | Use case |
|---|---|
| `RedundancySwitcherAdapter` | Production: connects to [redundancy_switcher](https://github.com/tier4/redundancy_switcher) via UDS |
| `SimpleSwitcherAdapter` | Development / CI: topic-based mock switcher, no hardware required |
| `NonRedundantSwitcherAdapter` | Single-ECU systems with no hardware switcher |

---

## RedundancySwitcherAdapter

UDS-based adapter for [redundancy_switcher](https://github.com/tier4/redundancy_switcher).
Translates between the `ElectionRequest` / `ElectionStatus` protocol and the
`SwitcherSignals` / `ActiveControlUnit` abstraction used by the framework.

Assumes a **Main ECU / Sub ECU / Main VCU / Sub VCU** topology with an election-based
fault-detection algorithm.

```xml
<!-- Main ECU -->
<include file="$(find-pkg-share autoware_redundancy_switcher_interface)/launch/redundancy_switcher_interface.launch.xml">
  <arg name="plugin_config"
    value="$(find-pkg-share autoware_redundancy_switcher_interface_plugins)/config/redundancy_switcher.main.param.yaml"/>
</include>
```

---

## SimpleSwitcherAdapter

Desktop mock that replaces the hardware switcher and its UDS protocol with ROS topics,
allowing redundancy switching scenarios to be tested on a single machine without any
special hardware.

```xml
<!-- Main ECU with simple (mock) switcher -->
<include file="$(find-pkg-share autoware_redundancy_switcher_interface)/launch/redundancy_switcher_interface.launch.xml">
  <arg name="is_main_ecu" value="true"/>
  <arg name="plugin_config"
    value="$(find-pkg-share autoware_redundancy_switcher_interface_plugins)/config/default.param.yaml"/>
</include>

<!-- Launch simple_switcher_node (single shared instance) -->
<include file="$(find-pkg-share autoware_redundancy_switcher_interface_plugins)/launch/simple_switcher_node.launch.xml"/>
```

Manual state injection:
```bash
# Self-interruption from Main ECU
ros2 topic pub --once /system/simple_switcher/request/self_interruption/main_ecu std_msgs/msg/Empty {}
# Reset
ros2 topic pub --once /system/simple_switcher/request/reset std_msgs/msg/Empty {}
```

---

## NonRedundantSwitcherAdapter

For single-ECU deployments where no hardware switcher exists.
Sets switcher signals to permanently stable and publishes `active_control_unit [0, 2]`
(Main ECU + Main VCU) at startup. All fault-related commands are ignored.

```xml
<include file="$(find-pkg-share autoware_redundancy_switcher_interface)/launch/redundancy_switcher_interface.launch.xml">
  <arg name="plugin_config"
    value="$(find-pkg-share autoware_redundancy_switcher_interface_plugins)/config/non_redundant.param.yaml"/>
</include>
```

---

## Documentation

- [docs/DESIGN.md](docs/DESIGN.md) — SimpleSwitcherAdapter: system diagram, state machine, topic/service list
