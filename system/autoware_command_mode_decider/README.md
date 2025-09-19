# autoware_command_mode_decider

## Overview

This package determines the priorities of vehicle behavior.
The determined behavior is published to the switcher node as command mode.
The command mode is the unified concept of operation mode and MRM.
Typically, the system will select a specified operation mode when normal, and if it is not available, select an appropriate MRM.
Since the command mode selection algorithm is system-dependent, developers can implement any logic they want through plugins.

![peripheral-ros-graph](./doc/peripheral-ros-graph.drawio.svg)

## Debug topic

The debug topic type is Int32MultiArrayStamped. Each item represents the following.
Here, manual mode means the state where the autoware control is false.
Note that operation mode and command mode can be divided into narrow and broad senses depending on whether it contains manual mode.

| Index | Description                                                       | Mode candidates                     |
| ----- | ----------------------------------------------------------------- | ----------------------------------- |
| 0     | This is the current command mode.                                 | operation mode, manual control, MRM |
| 1     | This is the current operation mode.                               | operation mode                      |
| 2     | This is the last operation mode where the transition is complete. | operation mode, manual control      |
| 3     | This is the requested autoware control.                           | boolean value (0 or 1)              |
| 4     | This is the requested operation mode.                             | operation mode                      |
| 5+    | These are the decided command modes.                              | operation mode, MRM                 |
