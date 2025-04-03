# PointCloud Densifier

## Purpose

The `pointcloud_densifier` enhances sparse point cloud data by leveraging information from previous LiDAR frames,
creating a denser representation especially for long-range points. This is particularly useful for improving perception
of distant objects where LiDAR data tends to be sparse.

## Inner-workings / Algorithm

The algorithm works as follows:

1. **ROI Filtering**: First filters the input point cloud to only keep points in a specific region of interest (ROI),
   typically focused on the distant area in front of the vehicle.

2. **Occupancy Grid Creation**: Creates a 2D occupancy grid from the filtered points to track which areas contain valid
   points in the current frame.

3. **Previous Frame Integration**: Transforms points from previous frames into the current frame's coordinate system
   using TF transformations.

4. **Selective Point Addition**: Adds points from previous frames only if they fall into grid cells that are occupied
   in the current frame. This ensures that only relevant points are added, avoiding ghost points from dynamic objects.

5. **Combined Output**: Returns a combined point cloud that includes both the current frame's points and selected
   points from previous frames.

## Inputs / Outputs

### Input

| Name    | Type                            | Description       |
| ------- | ------------------------------- | ----------------- |
| `input` | `sensor_msgs::msg::PointCloud2` | Input point cloud |

### Output

| Name     | Type                            | Description                  |
| -------- | ------------------------------- | ---------------------------- |
| `output` | `sensor_msgs::msg::PointCloud2` | Densified point cloud output |

## Parameters

| Name                  | Type   | Default Value | Description                            |
| --------------------- | ------ | ------------- | -------------------------------------- |
| `num_previous_frames` | int    | 1             | Number of previous frames to consider  |
| `x_min`               | double | 80.0          | Minimum x coordinate of ROI in meters  |
| `x_max`               | double | 200.0         | Maximum x coordinate of ROI in meters  |
| `y_min`               | double | -20.0         | Minimum y coordinate of ROI in meters  |
| `y_max`               | double | 20.0          | Maximum y coordinate of ROI in meters  |
| `grid_resolution`     | double | 0.3           | Resolution of occupancy grid in meters |

## Assumptions / Known limits

- The filter assumes that the TF tree contains valid transformations between coordinate frames from previous point clouds to the current frame.
- Performance depends on the number of previous frames used - more frames increase density but also processing time.
- The filter performs best on static elements in the scene, as dynamic objects may create artifacts if they move between frames.
- The accuracy of the densification depends on the quality of the TF transformations and ego-vehicle motion estimation.

## Usage

The pointcloud_densifier can be launched using:
