# tier4_control_mode_rviz_plugin

## Purpose

This plugin displays the current control mode status of Autoware.
The background color changes according to the mode, enabling intuitive status recognition.

## Inputs / Outputs

### Input

| Name                           | Type                                            | Description                                 |
| ------------------------------ | ----------------------------------------------- | ------------------------------------------- |
| `/vehicle/status/control_mode` | `autoware_vehicle_msgs::msg::ControlModeReport` | Topic representing the current control mode |

## Control Mode Types

| Mode                     | Value | Color     | Description                      |
| ------------------------ | ----- | --------- | -------------------------------- |
| NO_COMMAND               | 0     | Dark Gray | No command state                 |
| AUTONOMOUS               | 1     | Green     | Autonomous driving mode          |
| AUTONOMOUS_STEER_ONLY    | 2     | Dark Gray | Autonomous steering control only |
| AUTONOMOUS_VELOCITY_ONLY | 3     | Dark Gray | Autonomous velocity control only |
| MANUAL                   | 4     | Red       | Manual driving mode              |
| DISENGAGED               | 5     | Orange    | Control disengaged state         |
| NOT_READY                | 6     | Dark Gray | System not ready                 |

## How to Use

1. Launch RViz
2. Select `Panels` → `Add New Panel` from the menu
3. Choose `rviz_plugins/ControlModeDisplay`
4. The panel will display the current control mode

## RViz Configuration Example

```yaml
Panels:
  - Class: rviz_plugins/ControlModeDisplay
    Name: ControlModeDisplay
```
