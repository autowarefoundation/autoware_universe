# autoware_redundancy_switcher_interface

Framework for managing redundancy switching in Autoware-based vehicle systems.

## Summary

This package provides:

- A pure C++ `Processor` that evaluates switching conditions and produces effects
- An `EventGateway` for thread-safe event submission
- A `CommandBus` for broadcasting effects to all registered adapters
- Three built-in adapters: `LogAdapter`, `DiagAdapter`, `SubSystemAdapter`
- An `IAdapterPlugin` interface for Switcher-side plugins loaded via pluginlib

The Switcher-side adapter (hardware-specific UDS or topic-based) is loaded at runtime as a pluginlib plugin. See sibling packages:

- `autoware_redundancy_switcher_interface_plugins` — default topic-based plugin
- Other project-specific plugins can be implemented separately following [docs/PLUGIN_DEVELOPMENT_GUIDE.md](docs/PLUGIN_DEVELOPMENT_GUIDE.md)

## Documentation

| Document | Description |
|---|---|
| [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md) | Functional requirements derived from the implementation |
| [docs/DESIGN.md](docs/DESIGN.md) | Detailed design: file structure, Mermaid diagrams, event/command lists, interfaces |
| [docs/PLUGIN_DEVELOPMENT_GUIDE.md](docs/PLUGIN_DEVELOPMENT_GUIDE.md) | How to implement a custom SwitcherAdapter plugin |
| [docs/TERMINOLOGY.md](docs/TERMINOLOGY.md) | Data type definitions and data flow |
| [docs/ADAPTER_THREAD_SAFETY.md](docs/ADAPTER_THREAD_SAFETY.md) | Thread safety guide for adapter plugin authors |

## Quick Start

```bash
colcon build --packages-select autoware_redundancy_switcher_interface
```

Launch (redundant mode with simple switcher):

```xml
<include file="$(find-pkg-share autoware_redundancy_switcher_interface)/launch/redundancy_switcher_interface.launch.xml">
  <arg name="is_main_ecu" value="true"/>
  <arg name="is_redundant" value="true"/>
  <arg name="plugin_config"
    value="$(find-pkg-share autoware_redundancy_switcher_interface_plugins)/config/default.param.yaml"/>
</include>
```

Launch (non-redundant mode):

```xml
<include file="$(find-pkg-share autoware_redundancy_switcher_interface)/launch/redundancy_switcher_interface.launch.xml">
  <arg name="is_redundant" value="false"/>
</include>
```

## Key Concepts

- **InputEvent**: "What occurred" — submitted by adapters to the Processor via EventGateway
- **OutputCommand**: "What should be done" — returned by the Processor and broadcast via CommandBus
- **DomainSnapshot**: Read-only view of Processor state, used by DiagAdapter and logging
- **SwitcherSignals**: Three orthogonal signals (`is_stable`, `is_self_interrupted`, `is_faulted`) from the SwitcherAdapter
