# autoware_command_mode_switcher

## Overview

This package activates the selected command mode from the decider node.
The activation process for each command mode is implementation dependent, so extensions using plugins are supported.
By providing a specified interface, each plugin can operate under the appropriate state transitions.
