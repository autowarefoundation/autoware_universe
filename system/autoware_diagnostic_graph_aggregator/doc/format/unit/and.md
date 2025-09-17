# And

The `and` object is a unit that is evaluated as the maximum error level of the input units.
Note that error level `stale` is treated as `error`.

## Format

| Name   | Type                                   | Required | Description                           |
| ------ | -------------------------------------- | -------- | ------------------------------------- |
| `type` | <code>string</code>                    | yes      | Specify `and` when using this object. |
| `list` | <code>list\[[unit](../unit.md)]</code> | yes      | List of input unit objects.           |

If `list` is undeclared or empty, this object is evaluated as `ok`.

## Short-circuit evaluation

The type `short-circuit-and` is deprecated. This currently does exactly the same thing as `and`.
To ignore errors in dependencies, use the `dependent` field of the [unit](../unit.md) object.
