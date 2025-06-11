# autoware_process_alive_monitor

This package provides a monitoring node that detects ROS 2 node crashes by analyzing `launch.log` files, rather than subscribing to `/rosout` logs.

---

## Overview

- **Node name**: `autoware_process_alive_monitor`
- **Monitored file**: `launch.log`
- **Detected event**: Looks for lines containing the substring `"process has died"` and extracts the node name and exit code.

When a crash or unexpected shutdown occurs, `ros2 launch` typically outputs a line in `launch.log` such as:

```bash
[ERROR] [node_name-1]: process has died [pid 12345, exit code 139, cmd '...']
```

The `autoware_process_alive_monitor` node continuously reads the latest `launch.log` file, detects these messages, and logs a warning or marks the node as "dead."

---

## How it Works

1. **Find `launch.log`**:
   - First, checks the `ROS_LOG_DIR` environment variable.
   - If not set, falls back to `~/.ros/log`.
   - Identifies the latest log directory based on modification time.
2. **Monitor `launch.log`**:
   - Reads the file from the last known position to detect new log entries.
   - Looks for lines containing `"process has died"`.
   - Extracts the node name and exit code.
3. **Filtering**:
   - **Ignored node names**: Nodes matching patterns in `ignore_node_names` are skipped.
   - **Ignored exit codes**: Logs with ignored exit codes are not flagged as errors.
4. **Regular Updates**:
   - A timer periodically reads new entries from `launch.log`.
   - Dead nodes are reported in the logs. (will be changed to publish diagnostics)

---

## Parameters

| Parameter Name      | Type       | Default           | Description                                                |
| ------------------- | ---------- | ----------------- | ---------------------------------------------------------- |
| `ignore_node_names` | `string[]` | `[]` (empty list) | Node name patterns to ignore. E.g., `['rviz2']`.           |
| `ignore_exit_codes` | `int[]`    | `[]` (empty list) | Exit codes to ignore (e.g., `0` or `130` for normal exit). |
| `check_interval`    | `double`   | `1.0`             | Timer interval (seconds) for scanning the log file.        |
| `enable_debug`      | `bool`     | `false`           | Enables debug logging for detailed output.                 |

Example **`autoware_process_alive_monitor.param.yaml`**:

```yaml
autoware_process_alive_monitor:
  ros__parameters:
    ignore_node_names:
      - rviz2
      - teleop_twist_joy
    ignore_exit_codes:
      - 0
      - 130
    check_interval: 1.0
    enable_debug: false
```

---

## Unimplemented Features

1. **Heartbeat Monitoring**:

   - Will publish a heartbeat topic that can be monitored by `topic_state_monitor`.
   - The `topic_state_monitor` will check the topic's publishing frequency to confirm the node is operational.

2. **Diagnostic Information**:
   - When a process death is detected, the node will publish to the `/diagnostics` topic.
   - This feature is planned to be implemented but not yet implemented.

---

## Limitations

- **後で書く**: TBD.

---
