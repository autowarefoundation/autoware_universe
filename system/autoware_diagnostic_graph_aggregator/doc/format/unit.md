# Unit

The `unit` is a base object that makes up the diagnostic graph.
Any derived object can be used where a unit object is required.

## Format

| Name                | Type     | Required | Description                                                              |
| ------------------- | -------- | -------- | ------------------------------------------------------------------------ |
| `path`              | `string` | no       | Any string to reference from other units.                                |
| `type`              | `string` | yes      | The string indicating the type of derived object.                        |
| `$derived argument` | `any`    | no       | Additional arguments for constructing the derived object of given `type` |

`$derived_arguments` denotes additional arguments to be passed for constructing the object which is specified by `type` parameter. For example, if "and" is provided as `type`, `list` parameter is also required.

## Derived object types

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
