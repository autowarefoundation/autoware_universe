# autoware_command_mode_switcher

## Overview

This package activates the target command mode from the decider node.
The activation process for each command mode is implementation dependent, so extensions using plugins are supported.
By implementing the required interface, each plugin will be able to operate under the appropriate state transitions.

## Status

The switcher node will output the following status depending on the plugin status.

| Status                   | Description                                            |
| ------------------------ | ------------------------------------------------------ |
| request                  | This command mode is requested                         |
| transition               | This command mode transition is in progress            |
| vehicle_gate_selected    | The vehicle interface is using this command mode       |
| network_gate_selected    | The ECU is selected (if there are multiple ECUs)       |
| command_gate_selected    | The command source of this command mode is selected    |
| command_source_exclusive | This is the only command mode using the command source |
| command_source_enabled   | This command mode is enabled                           |
| command_source_disabled  | This command mode is disabled                          |
