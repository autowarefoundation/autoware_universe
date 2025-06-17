# autoware_command_mode_types

## Overview

This package defines constants for command modes and command sources that are commonly used across several packages.
Command modes represents a particular behavior in Autoware, and in implementation it is a collective term for operation mode and MRM.
Developers can add their command modes and define the decision logic for them.
However, since the API currently does not support extending the operation mode, the focus will be on extending MRM.

## Architecture

The following diagram shows the architecture of the relevant modules.
The command mode decider and command mode switcher nodes allow their behavior to be customized using plugins.
The decider node determines the target command mode based on the command mode availability from diagnostic graph aggregator, and the switcher node receives it and actually enables the target mode.
Since a single module may support multiple command modes by switching between modes, the commands that are actually output are managed as a command source.
Finally, the control command gate node selects one of these command sources and sends it to the vehicle.

![command-mode-architecture](./doc/command-mode-architecture.drawio.svg)

## Adding modes and sources

If you want to add a new command mode or command source, It is recommended to create a package that defines the mode and source ID like this package.
Then, use the defined ID to create plugins and change the settings of each node. For information on assigning IDs, refer to the following sections.

## Command modes

The following table lists the mode IDs defined by this package. To add a new ID, follow the rules below.

- The value is a 16-bit unsigned integer.
- Do not use 0 through 9.
- It is recommended to assign a value that is not confused with the source IDs.

| ID   | Name             | Description                              |
| ---- | ---------------- | ---------------------------------------- |
| 0    | unknown          | unknown                                  |
| 1000 | manual           | operation mode autoware control disabled |
| 1001 | stop             | operation mode stop                      |
| 1002 | autonomous       | operation mode autonomous                |
| 1003 | local            | operation mode local                     |
| 1004 | remote           | operation mode remote                    |
| 2001 | emergency_stop   | MRM emergency stop                       |
| 2002 | comfortable_stop | MRM comfortable stop                     |
| 2003 | pull_over        | MRM pull over (not yet supported)        |

## Command sources

The following table lists the source IDs defined by this package. To add a new ID, follow the rules below.

- The value is a 16-bit unsigned integer.
- Do not use 0 through 9.
- It is recommended to assign a value that is not confused with the mode IDs.

| ID  | Name           | Description                                  |
| --- | -------------- | -------------------------------------------- |
| 0   | unknown        | unknown                                      |
| 1   | builtin        | builtin stop command of control command gate |
| 11  | stop           | operation mode stop                          |
| 12  | main           | operation mode stop, MRM comfortable stop    |
| 13  | local          | operation mode local                         |
| 14  | remote         | operation mode remote                        |
| 21  | emergency_stop | MRM emergency stop                           |
