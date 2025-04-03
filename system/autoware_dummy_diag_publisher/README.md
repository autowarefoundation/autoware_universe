# dummy_diag_publisher

## Purpose

This package outputs a dummy diagnostic data for debugging and developing.

## Inputs / Outputs

### Outputs

| Name           | Type                                     | Description         |
| -------------- | ---------------------------------------- | ------------------- |
| `/diagnostics` | `diagnostic_msgs::msgs::DiagnosticArray` | Diagnostics outputs |

## Parameters

{{ json_to_markdown("system/dummy_diag_publisher/schema/_empty.schema.json") }}
{{ json_to_markdown("system/dummy_diag_publisher/schema/dummy_diag_publisher.schema.json") }}
{{ json_to_markdown("system/dummy_diag_publisher/schema/extra.schema.json") }}

## Assumptions / Known limits

TBD.

## Usage

### launch

```sh
ros2 launch autoware_dummy_diag_publisher dummy_diag_publisher.launch.xml
```

### reconfigure

```sh
ros2 param set /dummy_diag_publisher velodyne_connection.status "Warn"
ros2 param set /dummy_diag_publisher velodyne_connection.is_active true
```
