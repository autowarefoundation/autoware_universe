# Path

The path object specifies the file path of the subgraph to be imported.
The structure of the subgraph file should be [graph object](./graph.md).

## Format

| Name   | Type     | Required | Description                    |
| ------ | -------- | -------- | ------------------------------ |
| `path` | `string` | yes      | The file path of the subgraph. |

## Substitutions

File paths can contain substitutions like ROS 2 launch. The supported substitutions are as follows.

| Substitution                  | Description                                     |
| ----------------------------- | ----------------------------------------------- |
| `$(dirname)`                  | The path of this file directory.                |
| `$(find-pkg-share <package>)` | The path of the package.                        |
| `$(var <name>)`               | The value of the variable passed from the node. |

### Using `$(var <name>)`

The `$(var <name>)` substitution allows you to use variables in the graph file.
Variables are passed via the `graph_vars` parameter as a YAML map string.

Note: In launch XML, the value must be wrapped in single quotes to prevent YAML parsing.

```xml
<arg name="graph_vars" value="'{vehicle_id: vehicle1, config_dir: /path/to/config}'"/>
```

Then in your graph file:

```yaml
files:
  - { path: $(var config_dir)/$(var vehicle_id)/module.yaml }
```
