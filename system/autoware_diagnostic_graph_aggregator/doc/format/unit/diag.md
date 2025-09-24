# Diag

The `diag` object is a unit that refers to a specific status within the source diagnostics.

## Format

| Name         | Type     | Required | Description                                       |
| ------------ | -------- | -------- | ------------------------------------------------- |
| `type`       | `string` | yes      | Specify `diag` when using this object.            |
| `node`       | `string` | yes      | Node name that outputs the diagnostic status.     |
| `name`       | `string` | yes      | The name of the diagnostic status.                |
| `timeout`    | `float`  | yes      | The timeout seconds for the diagnostic status.    |
| `hysteresis` | `float`  | yes      | The hysteresis seconds for the diagnostic status. |

## Hysteresis

If the `hysteresis` field is set to a number of seconds, level changes less than this time will be ignored.
If the field is not specified, it is equivalent to specifying 0.0 seconds, in which case the input and output will always match.
This is when the latch is set to 0.0 seconds.

![hysteresis-0sec](../images/hysteresis-0sec.drawio.svg)

This is when the hysteresis is set to 2.0 seconds.

![hysteresis-2sec](../images/hysteresis-2sec.drawio.svg)

Error level is also considered warning level when the current level is OK.
OK level is also considered warning level when the current level is error.
This is when the hysteresis is set to 2.0 seconds.

![hysteresis-warn](../images/hysteresis-warn.drawio.svg)
