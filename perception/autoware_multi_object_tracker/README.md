# multi_object_tracker

## Purpose

The results of the detection are processed by a time series. The main purpose is to give ID and estimate velocity.

## Inner-workings / Algorithms

This multi object tracker consists of data association and EKF.

![multi_object_tracker_overview](image/multi_object_tracker_overview.svg)

### Data association

The data association performs maximum score matching, called min cost max flow problem.
In this package, mussp[1] is used as solver.
In addition, when associating observations to tracers, data association have gates such as the area of the object from the BEV, Mahalanobis distance, and maximum distance, depending on the class label.

### EKF Tracker

Models for pedestrians, bicycles (motorcycles), cars and unknown are available.
The pedestrian or bicycle tracker is running at the same time as the respective EKF model in order to enable the transition between pedestrian and bicycle tracking.
For big vehicles such as trucks and buses, we have separate models for passenger cars and large vehicles because they are difficult to distinguish from passenger cars and are not stable. Therefore, separate models are prepared for passenger cars and big vehicles, and these models are run at the same time as the respective EKF models to ensure stability.

## Inputs / Outputs

### Input

Multiple inputs are pre-defined in the input channel parameters (described below) and the inputs can be configured

| Name                        | Type          | Description                 |
| --------------------------- | ------------- | --------------------------- |
| `input/detection**/objects` | `std::string` | input topic                 |
| `input/detection**/channel` | `std::string` | input channel configuration |

rule of the channel configuration

- 'none' or empty : Indicates that this detection input channel is not used/disabled
- Any other string : Specifies a custom channel name to be used for the detection input, configured in `schema/input_channels.schema.json`

Up to 12 detection inputs can be configured (detection01 through detection12). Each input consists of an objects topic and its corresponding channel configuration.

Example configurations:

- Single detection input:

```yaml
input/detection01/objects: /perception/object_recognition/detection/objects
input/detection01/channel: detected_objects # general input channel type
input/detection02/objects: input/objects02
input/detection02/channel: none # Disabled
```

- Multiple detection inputs:

```yaml
# lidar centerpoint
input/detection01/objects: /perception/object_recognition/detection/lidar_centerpoint/objects
input/detection01/channel: lidar_centerpoint

# lidar short_range centerpoint
input/detection02/channel: /perception/object_recognition/detection/centerpoint_short_range/objects
input/detection02/objects: lidar_centerpoint_short_range

# camera lidar fusion
input/detection03/objects: /perception/object_recognition/detection/clustering/camera_lidar_fusion/objects
input/detection03/channel: camera_lidar_fusion

# camera lidar fusion based irregular object detection
input/detection04/objects: /perception/object_recognition/detection/irregular_object/objects
input/detection04/channel: camera_lidar_fusion_irregular

# detection by tracker
input/detection05/objects: /perception/object_recognition/detection/detection_by_tracker/objects
input/detection05/channel: detection_by_tracker

# radar
input/detection06/objects: /perception/object_recognition/detection/radar/objects
input/detection06/channel: radar

# disable
input/detection07/objects: input/objects07
input/detection07/channel: none # Disabled
```

Up to 12 detection inputs can be configured (detection01 through detection12). Each input consists of an objects topic and its corresponding channel configuration.

### Output

| Name       | Type                                            | Description     |
| ---------- | ----------------------------------------------- | --------------- |
| `~/output` | `autoware_perception_msgs::msg::TrackedObjects` | tracked objects |

## Parameters

### Input Channel parameters

{{ json_to_markdown("perception/autoware_multi_object_tracker/schema/input_channels.schema.json") }}

### Core Parameters

- Node

{{ json_to_markdown("perception/autoware_multi_object_tracker/schema/multi_object_tracker_node.schema.json") }}

- Association

{{ json_to_markdown("perception/autoware_multi_object_tracker/schema/data_association_matrix.schema.json") }}

#### Simulation parameters

{{ json_to_markdown("perception/autoware_multi_object_tracker/schema/simulation_tracker.schema.json") }}

## Assumptions / Known limits

See the [model explanations](models.md).

## Performance Benchmark & Unit Testing

### Overview

Unit tests and benchmarks are included to evaluate tracker performance under varying detection loads and object types.

### How to Run Locally

#### 1. Build with Tests

```bash
colcon build --packages-select autoware_multi_object_tracker \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

#### 2. Run the Performance Benchmark

```bash
source install/setup.bash
./build/autoware_multi_object_tracker/test_multi_object_tracker
```

This runs the default test (`runPerformanceTest`) and outputs timing data.

#### 3. Enable Optional Profiling Modes (Manual)

To evaluate scalability with object count:

- Manually enable the following functions in the test source:
  - `profilePerformanceVsCarCount()`
  - `profilePerformanceVsPedestrianCount()`

These run additional profiling scenarios.

### Rosbag Replay & Visualization

#### Simulated Rosbag Output

To record benchmark results for visualization in RViz:

1. Enable `write_bag = true` in `runIterations()`
2. Run the test; the output `.db3` path is printed
3. Visualize:

```bash
ros2 bag play <output_file>.db3
rviz2 -d <your_rviz_config>.rviz
```

#### Real Rosbag Input

1. Set the path in `runPerformanceTestWithRosbag()` to a real `.db3` file
2. Run the test
3. Visualize the tracking result in RViz

## (Optional) Error detection and handling

<!-- Write how to detect errors and how to recover from them.

Example:
  This package can handle up to 20 obstacles. If more obstacles found, this node will give up and raise diagnostic errors.
-->

## (Optional) Performance characterization

### Evaluation of muSSP

According to our evaluation, muSSP is faster than normal [SSP](lib/association/successive_shortest_path) when the matrix size is more than 100.

Execution time for varying matrix size at 95% sparsity. In real data, the sparsity was often around 95%.
![mussp_evaluation1](image/mussp_evaluation1.png)

Execution time for varying the sparsity with matrix size 100.
![mussp_evaluation2](image/mussp_evaluation2.png)

## (Optional) References/External links

This package makes use of external code.

| Name                                                 | License                                                   | Original Repository                  |
| ---------------------------------------------------- | --------------------------------------------------------- | ------------------------------------ |
| [muSSP](lib/association/mu_successive_shortest_path) | [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0) | <https://github.com/yu-lab-vt/muSSP> |

[1] C. Wang, Y. Wang, Y. Wang, C.-t. Wu, and G. Yu, "muSSP: Efficient
Min-cost Flow Algorithm for Multi-object Tracking," NeurIPS, 2019

## (Optional) Future extensions / Unimplemented parts

<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->
