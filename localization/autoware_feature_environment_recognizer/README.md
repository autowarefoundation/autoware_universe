# autoware_feature_environment_recognizer

## Abstract

This package determines which area (polygon) in a Lanelet2 map the current position belongs to and identifies the environment ID based on the area's subtype. It can be used to identify environments with rich/poor features for map matching.

By using area-based determination instead of lanelet IDs, information can be directly embedded in the map, avoiding issues where lanelet IDs change when the map is regenerated.

## Environment ID Definitions

- **Environment ID -1**: Invalid (area not found or map not ready)
- **Environment ID 0**: Normal environment (default, when not matching any area)
- **Environment ID 1**: Uniform road (e.g., tunnel straight sections, uniform shape roads)
- **Environment ID 2**: Feature-poor road (roads with few features for map matching)

## Features

- Reads areas from the polygon layer of Lanelet2 maps
- Determines which area the current position is within (point-in-polygon check)
- Determines environment ID based on the area's subtype
- Publishes environment ID (integer)

## Node

### feature_environment_recognizer_node

Receives the current position pose and lanelet map, and publishes the environment ID.

#### Subscriptions

- `~/input/lanelet2_map` (`autoware_map_msgs::msg::LaneletMapBin`)
  - Lanelet2 map data

- `~/input/pose` (`geometry_msgs::msg::PoseWithCovarianceStamped`)
  - Current position pose

#### Publications

- `~/output/environment` (`autoware_feature_environment_recognizer::msg::FeatureEnvironment`)
  - Feature environment recognition result
  - `header`: Header containing the timestamp of the input position
  - `environment_id`: Identified environment ID
  - `confidence`: Confidence of the recognition result (currently unused, reserved for future use, default value: 1.0)

#### Parameters

- `default_environment_id` (int, default: 0)
  - Default environment ID when not matching any area (normal environment)

- `area_subtype_<subtype_name>.environment_id` (int, optional)
  - Mapping between area subtype and environment_id
  - `area_subtype_uniform_road`: Subtype for uniform roads (e.g., tunnel straight sections)
  - `area_subtype_feature_poor_road`: Subtype for feature-poor roads
  - Example: `area_subtype_uniform_road.environment_id: 1`

## Usage

### Launch

```bash
ros2 launch autoware_feature_environment_recognizer feature_environment_recognizer.launch.xml
```

### Parameter Configuration Example

Configure `config/feature_environment_recognizer.param.yaml` as follows:

```yaml
/**:
  ros__parameters:
    default_environment_id: 0

    # Environment ID 1: Uniform road (e.g., tunnel straight sections)
    area_subtype_uniform_road:
      environment_id: 1

    # Environment ID 2: Feature-poor road
    area_subtype_feature_poor_road:
      environment_id: 2
```

### Area Definition in Map

Define areas in the Lanelet2 map in the following format:

```xml
  <node id="1" lat="35.8xxxxx" lon="139.6xxxxx">
    <tag k="mgrs_code" v="54SUE000000"/>
    <tag k="local_x" v="10.0"/>
    <tag k="local_y" v="10.0"/>
    <tag k="ele" v="1.0"/>
  </node>
  <node id="2" lat="35.8xxxxx" lon="139.6xxxxx">
    <tag k="mgrs_code" v="54SUE000000"/>
    <tag k="local_x" v="10.0"/>
    <tag k="local_y" v="20.0"/>
    <tag k="ele" v="1.0"/>
  </node>
  <!-- Other nodes... -->

  <way id="5">
    <nd ref="1"/>
    <nd ref="2"/>
    <!-- Other nodes... -->
    <tag k="type" v="feature_environment_specify"/>
    <tag k="subtype" v="uniform_road"/>
    <tag k="area" v="yes"/>
  </way>
```

Important points:

- Set `type` to `"feature_environment_specify"`
- Set `subtype` to the subtype name defined in parameters (e.g., `"uniform_road"`, `"feature_poor_road"`)
- Set `area` to `"yes"`

## Implementation Details

This package uses the following features:

- Reads areas from the polygon layer of Lanelet2 maps
- Point-in-polygon determination using `boost::geometry::within()`
- Adopts a similar approach to `PoseEstimatorArea` in `pose_estimator_arbiter`

## Dependencies

- `autoware_lanelet2_extension`
- `autoware_lanelet2_utils`
- `autoware_map_msgs`
- `geometry_msgs`
- `rclcpp`
- `std_msgs`
- `Boost::geometry` (for point-in-polygon determination)
