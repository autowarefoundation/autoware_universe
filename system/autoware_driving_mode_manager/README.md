# autoware_driving_mode_manager

## Overview

Autoware performs different behaviors depending on the API and the diagnostics.
Each behavior outputs either a trajectory or a command, and they are selected by the trajectory gate and command gate as shown below.

![architecture](./doc/architecture.drawio.svg)

## Driving Mode

Here, we distinguish Autoware behaviors as driving modes and call the inputs to each gate the trajectory source and command source.
The vehicle interface has a control mode as its state, which determines whether outputs from Autoware are applied.
In that case, each driving mode corresponds to a combination of trajectory source, command source, and control mode as shown below.

In practice, control mode may not be controlled by Autoware or may change due to overrides, so it is handled separately.
Driving modes under Autoware control are called autoware modes, and the rest are called platform modes.

<table>
<tr><th rowspan="2">Driving Mode</th><th colspan="2">Autoware Mode</th><th>Platform Mode</th></tr>
<tr><th>Trajectory Source</th><th>Command Source</th><th>Control Mode</th></tr>
<tr><td>AutoMode1</td> <td>T1</td><td>C1</td><td>A</td></tr>
<tr><td>AutoMode2</td> <td>T2</td><td>C1</td><td>A</td></tr>
<tr><td>MRM1</td>      <td>T3</td><td>C1</td><td>A</td></tr>
<tr><td>MRM2</td>      <td>any</td><td>C2</td><td>A</td></tr>
<tr><td>RemoteMode</td><td>any</td><td>C3</td><td>A</td></tr>
<tr><td>ManualMode</td><td>any</td><td>any</td><td>M</td></tr>
</table>

## Operation Mode and Fail-safe API

The operation mode API and Fail-safe API define their own IDs for modes.
Since driving mode integrates these modes, conversion from API IDs is required.

| API            | ID  | Driving Mode ID | Description     |
| -------------- | --- | --------------- | --------------- |
| Operation Mode | 1   | 1001            | StopMode        |
| Operation Mode | 2   | 1002            | AutonomousMode  |
| Operation Mode | 3   | 1003            | LocalMode       |
| Operation Mode | 4   | 1004            | RemoteMode      |
| Fail-safe      | 2   | 2001            | EmergencyStop   |
| Fail-safe      | 3   | 2002            | ComfortableStop |

## Implementation Overview

![data-flow](./doc/data-flow.drawio.svg)

## Drive Mode Status

| Flags       | Description                                                                                             |
| ----------- | ------------------------------------------------------------------------------------------------------- |
| available   | Whether it is possible to switch to the mode. This does not guarantee that output is actually produced. |
| ready       | Whether the mode output is actually being produced.                                                     |
| stable      | Whether the mode operation is stable and the transition can be completed.                               |
| continuable | Whether the vehicle is currently driving in the mode and can continue to do so.                         |

## Autoware Mode Transition

When the autoware mode changes, the following steps are used to switch modes.

1. Enable the command filter.
2. Wait for the current mode to become ready.
3. Switch the trajectory source.
4. Switch the command source.
5. Wait for the current mode to become stable.
6. Disable the command filter.

## Platform Mode Transition

Switching to a state where Autoware control is not applied is performed immediately.
Switching to a state where Autoware control is applied uses the following steps.
If additional conditions are required while driving, include them in available status.

1. Enable the command filter.
2. Wait for the current mode to become ready.
3. Switch the control mode.
4. Wait for the current mode to become stable.
5. Disable the command filter.
