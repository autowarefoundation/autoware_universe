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

The `$(var <name>)` substitution allows you to use variables passed from the node.
Variables can be set using the `graph_vars` parameter in the node configuration.

The parameter accepts a list of strings in "key=value" format:

```xml
<node pkg="autoware_diagnostic_graph_aggregator" exec="aggregator" name="aggregator">
  <param name="graph_file" value="$(find-pkg-share my_package)/config/graph.yaml"/>
  <param name="graph_vars" value="['vehicle_id=vehicle1', 'config_dir=/path/to/config']"/>
</node>
```

Then in your YAML file:

```yaml
files:
  - { path: $(var config_dir)/$(var vehicle_id)/module.yaml }
```
