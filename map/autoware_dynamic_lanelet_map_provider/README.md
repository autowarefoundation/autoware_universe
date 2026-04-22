# autoware_dynamic_lanelet_map_provider

## Purpose

`autoware_dynamic_lanelet_map_provider` publishes a dynamic lanelet2 submap around the ego vehicle.

The node subscribes to lanelet2 map metadata and ego odometry, selects map cells that overlap a radius around the ego position, requests those cells through `GetSelectedLanelet2Map`, and publishes the resulting lanelet2 submap.

## Inner-workings / Algorithms

1. Subscribes to:
   - `input/odometry` (`nav_msgs/msg/Odometry`)
   - `input/lanelet2_map_metadata` (`autoware_map_msgs/msg/LaneletMapMetaData`, transient local)
2. Every 1 second, checks whether all prerequisites are available:
   - latest odometry
   - map metadata
   - no in-flight request
3. Computes ego movement from the last update point and skips update if movement is below `update_distance_threshold`.
4. Selects candidate map cells with an AABB(Axis-Aligned Bounding Box)-vs-circle overlap test:
   - each metadata cell has `[min_x, min_y, max_x, max_y]`
   - a cell is selected when its rectangle overlaps a circle centered at ego with radius `map_radius`
5. Calls `service/get_selected_lanelet2_map` with selected `cell_ids`.
6. Publishes non-empty response map on `output/lanelet2_submap` (transient local).

## Inputs / Outputs

### Input

| Name                                | Type                                             | Description                                                |
| ----------------------------------- | ------------------------------------------------ | ---------------------------------------------------------- |
| `input/odometry`                    | `nav_msgs::msg::Odometry`                        | Ego odometry used as the submap center.                    |
| `input/lanelet2_map_metadata`       | `autoware_map_msgs::msg::LaneletMapMetaData`     | Metadata of available lanelet2 map cells (AABB list).      |
| `service/get_selected_lanelet2_map` | `autoware_map_msgs::srv::GetSelectedLanelet2Map` | Service used to fetch lanelet2 map cells for selected IDs. |

### Output

| Name                     | Type                                    | Description                                     |
| ------------------------ | --------------------------------------- | ----------------------------------------------- |
| `output/lanelet2_submap` | `autoware_map_msgs::msg::LaneletMapBin` | Dynamic lanelet2 submap around the ego vehicle. |

## Parameters

### Node Parameters

None.

### Core Parameters

{{ json_to_markdown("map/autoware_dynamic_lanelet_map_provider/schema/lanelet2_map_provider.schema.json") }}

## Launch Arguments

The launch file `launch/lanelet2_map_provider.launch.xml` supports the following arguments:

| Name                                | Default                                                                                           | Description                                           |
| ----------------------------------- | ------------------------------------------------------------------------------------------------- | ----------------------------------------------------- |
| `param_path`                        | `$(find-pkg-share autoware_dynamic_lanelet_map_provider)/config/lanelet2_map_provider.param.yaml` | Parameter file path.                                  |
| `input_odometry`                    | `/localization/kinematic_state`                                                                   | Remap target for `input/odometry`.                    |
| `input_lanelet2_map_metadata`       | `/map/output/lanelet2_map_metadata`                                                               | Remap target for `input/lanelet2_map_metadata`.       |
| `output_lanelet2_submap`            | `output/lanelet2_submap`                                                                          | Remap target for `output/lanelet2_submap`.            |
| `service_get_selected_lanelet2_map` | `/map/service/get_selected_lanelet2_map`                                                          | Remap target for `service/get_selected_lanelet2_map`. |

## Assumptions / Known limits

- The node requires both odometry and lanelet2 metadata before the first update.
- Submap refresh is distance-triggered and evaluated by a 1 Hz timer, so updates are not continuous.
- If the map service is unavailable, the node only warns and retries on subsequent timer ticks.
- If the service returns an empty map payload, the node reports an error and does not publish.
