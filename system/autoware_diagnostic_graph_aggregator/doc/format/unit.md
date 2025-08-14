# Unit

The `unit` is a base object that makes up the diagnostic graph.
Any derived object can be used where a unit object is required.

## Format

| Name                | Type     | Required | Description                                                              |
| ------------------- | -------- | -------- | ------------------------------------------------------------------------ |
| `path`              | `string` | no       | Any string to reference from other units.                                |
| `type`              | `string` | yes      | The string indicating the type of derived object.                        |
| `dependent`         | `string` | no       | The path of the dependent unit.                                          |
| `latch`             | `float`  | no       | The latch seconds for this unit.                                         |
| `$derived argument` | `any`    | no       | Additional arguments for constructing the derived object of given `type` |

## Derived object types

`$derived_arguments` denotes additional arguments to be passed for constructing the object which is specified by `type` parameter. For example, if "and" is provided as `type`, `list` parameter is also required.

- [diag](./unit/diag.md)
- [link](./unit/link.md)
- [and](./unit/and.md)
- [or](./unit/or.md)
- [remapping](./unit/remap.md)
  - `warn-to-ok`
  - `warn-to-error`
- [constant](./unit/const.md)
  - `ok`
  - `warn`
  - `error`
  - `stale`

## Dependent failure

The `dependent` field defines dependencies between units that do not have a parent-child relationship in the tree.
If the unit specified in this field is not OK, the `is_dependent` field in the node status will be true.
This allows the user to easily rule out units other than the root cause.

## Latch

If the `latch` field is set to a number of seconds, the worst level will be latched after the specified time has elapsed.
If the field is not specified, the latch is disabled and the input and output levels always match.

![latch-none](./images/latch-none.drawio.svg)

This is when the latch is set to 0.0 seconds.

![latch-0sec](./images/latch-0sec.drawio.svg)

This is when the latch is set to 2.0 seconds.

![latch-2sec](./images/latch-2sec.drawio.svg)

Error level is also considered warning level. This is when the latch is set to 2.0 seconds.

![latch-warn](./images/latch-warn.drawio.svg)

When the latch is cleared, the decision starts again from that moment, independent of the previous error duration. This is when the latch is set to 2.0 seconds.

![latch-reset](./images/latch-reset.drawio.svg)
