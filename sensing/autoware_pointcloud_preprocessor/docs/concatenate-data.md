# concatenate_and_time_synchronize_node

## Purpose

The `concatenate_and_time_synchronize_node` is a node designed to combine and synchronize multiple point clouds into a single, unified point cloud. By integrating data from multiple LiDARs, this node significantly enhances the sensing range and coverage of autonomous vehicles, enabling more accurate perception of the surrounding environment. Synchronization ensures that point clouds are aligned temporally, reducing errors caused by mismatched timestamps.

For example, consider a vehicle equipped with three LiDAR sensors mounted on the left, right, and top positions. Each LiDAR captures data from its respective field of view, as shown below:

|                       Left                        |                       Top                       |                        Right                        |
| :-----------------------------------------------: | :---------------------------------------------: | :-------------------------------------------------: |
| ![Concatenate Left](./image/concatenate_left.png) | ![Concatenate Top](./image/concatenate_top.png) | ![Concatenate Right](./image/concatenate_right.png) |

After processing the data through the `concatenate_and_time_synchronize_node`, the outputs from all LiDARs are combined into a single comprehensive point cloud that provides a complete view of the environment:

![Full Scene View](./image/concatenate_all.png)

This resulting point cloud allows autonomous systems to detect obstacles, map the environment, and navigate more effectively, leveraging the complementary fields of view from multiple LiDAR sensors.

## Inner Workings / Algorithms

![concatenate_algorithm](./image/concatenate_algorithm.drawio.svg)

### Step 1: Match and Create Collector

When a point cloud arrives, its timestamp is checked, and an offset is subtracted to get the reference timestamp. The node then checks if there is an existing collector with the same reference timestamp. If such a collector exists, the point cloud is added to it. If no such collector exists, a new collector is created with the reference timestamp.

### Step 2: Trigger the Timer

Once a collector is created, a timer for that collector starts counting down (this value is defined by `timeout_sec`). The collector begins to concatenate the point clouds either when all point clouds defined in `input_topics` have been collected or when the timer counts down to zero.

### Step 3: Concatenate the Point Clouds

The concatenation process involves merging multiple point clouds into a single, concatenated point cloud. The timestamp of the concatenated point cloud will be the earliest timestamp from the input point clouds. By setting the parameter `is_motion_compensated` to `true`, the node will consider the timestamps of the input point clouds and utilize the `twist` information from `geometry_msgs::msg::TwistWithCovarianceStamped` to compensate for motion, aligning the point cloud to the selected (earliest) timestamp.

### Step 4: Publish the Point Cloud

After concatenation, the concatenated point cloud is published, and the collector is deleted to free up resources.

## Inputs / Outputs

### Input

| Name            | Type                                             | Description                                                                                                                                              |
| --------------- | ------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `~/input/twist` | `geometry_msgs::msg::TwistWithCovarianceStamped` | Twist information adjusts the point cloud scans based on vehicle motion, allowing LiDARs with different timestamps to be synchronized for concatenation. |
| `~/input/odom`  | `nav_msgs::msg::Odometry`                        | Vehicle odometry adjusts the point cloud scans based on vehicle motion, allowing LiDARs with different timestamps to be synchronized for concatenation.  |

By setting the `input_twist_topic_type` parameter to `twist` or `odom`, the subscriber will subscribe to either `~/input/twist` or `~/input/odom`. If the user doesn't want to use the twist information or vehicle odometry to compensate for motion, set `is_motion_compensated` to `false`.

### Output

| Name              | Type                            | Description               |
| ----------------- | ------------------------------- | ------------------------- |
| `~/output/points` | `sensor_msgs::msg::Pointcloud2` | Concatenated point clouds |

### Core Parameters

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/concatenate_and_time_sync_node.schema.json") }}

## Concatenation Strategies

The `concatenate_and_time_synchronize_node` supports different concatenation strategies through the `matching_strategy.type` parameter, designed to handle different LiDAR synchronization scenarios:

### Naive Strategy (`matching_strategy.type: "naive"`)

The naive strategy is designed for scenarios where LiDAR sensors are **not synchronized** or when precise timestamp alignment is not critical. This strategy:

- **Direct concatenation**: Point clouds are concatenated directly without complex timestamp matching
- **Simple collection**: Point clouds are collected and merged as they arrive within the timeout window
- **No offset compensation**: Does not use `lidar_timestamp_offsets` or `lidar_timestamp_noise_window` parameters
- **Faster processing**: Reduced computational overhead due to simplified logic

**When to use naive strategy:**

- LiDAR sensors are not hardware-synchronized
- Timestamp differences between sensors are negligible for your application
- You prioritize processing speed over precise temporal alignment
- Simple concatenation without complex matching logic is sufficient

#### Parameter Settings

Set the `type` parameter of `matching_strategy` to `"naive"` to concatenate the point clouds directly.

### Advanced Strategy (`matching_strategy.type: "advanced"`)

The advanced strategy is designed for scenarios where LiDAR sensors are **synchronized** and precise timestamp alignment is crucial. This strategy:

- **Precise timestamp matching**: Uses reference timestamps with offset compensation
- **Noise tolerance**: Accounts for timestamp jitter using `lidar_timestamp_noise_window`
- **Offset correction**: Applies `lidar_timestamp_offsets` to align different LiDAR timing
- **Robust collection**: Handles temporal variations and ensures proper point cloud grouping

**When to use advanced strategy:**

- LiDAR sensors are hardware-synchronized
- Precise temporal alignment is critical for your application
- You have measured timestamp offsets between your LiDAR sensors
- You want to handle timestamp noise and jitter effectively

**Required parameters for advanced strategy:**

- `lidar_timestamp_offsets`: Array of time offsets for each LiDAR sensor
- `lidar_timestamp_noise_window`: Time window to handle timestamp jitter
- `timeout_sec`: Maximum wait time for point cloud collection

#### Parameter Settings

Set the `type` parameter of `matching_strategy` to `"advanced"` and configure the `timeout_sec`, `lidar_timestamp_offsets` and `lidar_timestamp_noise_window` parameters accordingly.

##### timeout_sec

When network issues occur or when point clouds experience delays in the previous processing pipeline, some point clouds may be delayed or dropped. To address this, the `timeout_sec` parameter is used. Once the timer is created, it will start counting down from `timeout_sec`. If the timer reaches zero, the collector will not wait for delayed or dropped point clouds but will concatenate the remaining point clouds in the collector directly. The figure below demonstrates how `timeout_sec` works with `concatenate_and_time_sync_node` when `timeout_sec` is set to `0.12` (120 ms).

![concatenate_edge_case](./image/concatenate_edge_case.drawio.svg)

##### lidar_timestamp_offsets

Since different vehicles have varied designs for LiDAR scanning, the timestamps of each LiDAR may differ. Users need to know the offsets between each LiDAR and set the values in `lidar_timestamp_offsets`.

To monitor the timestamps of each LiDAR, run the following command:

```bash
ros2 topic echo "pointcloud_topic" --field header
```

The timestamps should increase steadily by approximately 100 ms, as per the Autoware default. You should see output like this:

```bash
nanosec: 156260951
nanosec: 257009560
nanosec: 355444581
```

This pattern indicates a LiDAR timestamp of 0.05.

If there are three LiDARs (left, right, top), and the timestamps for the left, right, and top point clouds are `0.01`, `0.05`, and `0.09` seconds respectively, the parameters should be set as [0.0, 0.04, 0.08]. This reflects the timestamp differences between the current point cloud and the point cloud with the earliest timestamp. Note that the order of the `lidar_timestamp_offsets` corresponds to the order of the `input_topics`.

The figure below demonstrates how `lidar_timestamp_offsets` works with `concatenate_and_time_sync_node`.

![ideal_timestamp_offset](./image/ideal_timestamp_offset.drawio.svg)

##### lidar_timestamp_noise_window

Additionally, due to the mechanical design of LiDARs, there may be some jitter in the timestamps of each scan, as shown in the image below. For example, if the scan frequency is set to 10 Hz (scanning every 100 ms), the timestamps between each scan might not be exactly 100 ms apart. To handle this noise, the `lidar_timestamp_noise_window` parameter is provided.

Users can use [this tool](https://github.com/tier4/timestamp_analyzer) to visualize the noise between each scan.

![jitter](./image/jitter.png)

From the example above, the noise ranges from 0 to 8 ms, so the user should set `lidar_timestamp_noise_window` to `0.008`.

The figure below demonstrates how `lidar_timestamp_noise_window` works with the `concatenate_and_time_sync_node`. If the green `X` is within the range of the red triangles, it indicates that the point cloud matches the reference timestamp of the collector.

![noise_timestamp_offset](./image/noise_timestamp_offset.drawio.svg)

## Launch

```bash
# The launch file will read the parameters from the concatenate_and_time_sync_node.param.yaml
ros2 launch autoware_pointcloud_preprocessor concatenate_and_time_sync_node.launch.xml
```

## Test

```bash
# build autoware_pointcloud_preprocessor
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to autoware_pointcloud_preprocessor

# test autoware_pointcloud_preprocessor
colcon test --packages-select autoware_pointcloud_preprocessor --event-handlers console_cohesion+
```

## Debug and Diagnostics

To verify whether the node has successfully concatenated the point clouds, the user can examine rqt or the `/diagnostics` topic using the following command:

```bash
ros2 topic echo /diagnostics
```

Below is an example output when the point clouds are concatenated successfully:

- Each point cloud has a value of `True`.
- The `Pointcloud concatenation succeeded` is `True`.
- The `level` value is `\0`. (diagnostic_msgs::msg::DiagnosticStatus::OK)

```bash
header:
  stamp:
    sec: 1722492015
    nanosec: 848508777
  frame_id: ''
status:
- level: "\0"
  name: 'concatenate_and_time_sync_node: concat_status'
  message: Concatenated pointcloud is published and include all topics
  hardware_id: concatenate_data_checker
  values:
  - key: Concatenated pointcloud timestamp
    value: '1718260240.159229994'
  - key: Minimum reference timestamp
    value: '1718260240.149230003'
  - key: Maximum reference timestamp
    value: '1718260240.169229984'
  - key: Timestamp: /sensing/lidar/left/pointcloud_before_sync
    value: '1718260240.159229994'
  - key: Concatenated: /sensing/lidar/left/pointcloud_before_sync
    value: 'True'
  - key: Timestamp: /sensing/lidar/right/pointcloud_before_sync
    value: '1718260240.194104910'
  - key: Concatenated: /sensing/lidar/right/pointcloud_before_sync
    value: 'True'
  - key: Timestamp: /sensing/lidar/top/pointcloud_before_sync
    value: '1718260240.234578133'
  - key: Concatenated: /sensing/lidar/top/pointcloud_before_sync
    value: 'True'
  - key: Pointcloud concatenation succeeded
    value: 'True'
```

Below is an example when point clouds fail to concatenate successfully.

- Some point clouds might have values of `False`.
- The `Pointcloud concatenation succeeded` is `False`.
- The `level` value is `\x02`. (diagnostic_msgs::msg::DiagnosticStatus::ERROR)

```bash
header:
  stamp:
    sec: 1722492663
    nanosec: 344942959
  frame_id: ''
status:
- level: "\x02"
  name: 'concatenate_and_time_sync_node: concat_status'
  message: Concatenated pointcloud is published but miss some topics
  hardware_id: concatenate_data_checker
  values:
  - key: Concatenated pointcloud timestamp
    value: '1718260240.859827995'
  - key: Minimum reference timestamp
    value: '1718260240.849828005'
  - key: Maximum reference timestamp
    value: '1718260240.869827986'
  - key: Timestamp: /sensing/lidar/left/pointcloud_before_sync/timestamp
    value: '1718260240.859827995'
  - key: Concatenated: /sensing/lidar/left/pointcloud_before_sync
    value: 'True'
  - key: Timestamp: /sensing/lidar/right/pointcloud_before_sync/timestamp
    value: '1718260240.895193815'
  - key: Concatenated: /sensing/lidar/right/pointcloud_before_sync
    value: 'True'
  - key: Concatenated: /sensing/lidar/top/pointcloud_before_sync
    value: 'False'
  - key: Pointcloud concatenation succeeded
    value: 'False'
```

## Node separation options

There is also an option to separate the concatenate_and_time_sync_node into two nodes: one for `time synchronization` and another for `concatenate pointclouds` ([See this PR](https://github.com/autowarefoundation/autoware_universe/pull/3312)).

Note that the `concatenate_pointclouds` and `time_synchronizer_nodelet` are using the [old design](https://github.com/autowarefoundation/autoware_universe/blob/9bb228fe5b7fa4c6edb47e4713c73489a02366e1/sensing/autoware_pointcloud_preprocessor/docs/concatenate-data.md) of the concatenate node.

## Assumptions / Known Limits

- If `is_motion_compensated` is set to `false`, the `concatenate_and_time_sync_node` will directly concatenate the point clouds without applying for motion compensation. This can save several milliseconds depending on the number of LiDARs being concatenated. Therefore, if the timestamp differences between point clouds are negligible, the user can set `is_motion_compensated` to `false` and omit the need for twist or odometry input for the node.
- As mentioned above, the user should clearly understand how their LiDAR's point cloud timestamps are managed to set the parameters correctly. If the user does not synchronize the point clouds, please set `matching_strategy.type` to `naive`.
