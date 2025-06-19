# autoware_control_command_gate

## Overview

This package subscribes to multiple commands, selects one and publish it.
Here, unless otherwise specified, command refers to a set of four commands: control, gear, turn_indicators, and hazard_lights.
Each command input is identified as a command source, an arbitrary string.
The node also applies a nominal filter to the selected commands to correct for obvious abnormal values.

![dataflow](./doc/dataflow.drawio.svg)

## Requirements

- Functional
  - Subscribe to multiple commands as command source.
  - Generate a builtin stop command as command source.
  - Select one from the command sources and publish it.
  - Filter abnormal values in the command selected.
  - Filter the command selected smoothly on mode transition when requested.
  - Support parking mode for humans to explicitly stop the vehicle (T.B.D.).
- Safety
  - Detect timeouts for each command source.
  - Select builtin stop when selected command source is not available.
