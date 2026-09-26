# bag2bag runner

This tool reads a recorded rosbag, runs the ROS-free core of this package on every frame, and writes the results to a new rosbag. The tool never calls `rclcpp::init`, and it uses no executor and no DDS. Therefore the tool does not have to follow the recording rate of the bag, and the same input bag always gives the same output bag.

| Executable                              | Cores it runs                                                                | Output                                                                                |
| --------------------------------------- | ---------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| `traffic_light_pipeline_bag2bag_runner` | First stage (`TrafficLightRecognition`) + second stage (`MultiCameraFusion`) | `merged_signals` / `selected_rois` per camera, and the fused `TrafficLightGroupArray` |

## How to use

```bash
ros2 run autoware_traffic_light_pipeline traffic_light_pipeline_bag2bag_runner \
  --input-bag <path to the input bag> \
  --map <path to the map directory> \
  --output-bag result/output_bag \
  --camera 4,5
```

| Argument          | Meaning                                                                                                                                                                                          |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `--input-bag`     | Directory of the input bag                                                                                                                                                                       |
| `--map`           | Map directory. The directory must hold exactly one `.osm` file, which is a lanelet2 map in the MGRS frame. The directory must also hold exactly one `.yaml` file with the projector information. |
| `--output-bag`    | Directory of the output bag. The tool overwrites an existing bag.                                                                                                                                |
| `--camera`        | The camera numbers recorded in the bag. `4,5` means camera4 and camera5. The tool creates one `TrafficLightRecognition` instance for each entry                                                  |
| `--ml-model-path` | Directory that holds the machine learning artifacts. The default is `$HOME/autoware_data`                                                                                                        |
| `--config`        | Path to the recognition configuration file. The default is this package's installed `config/traffic_light_recognition.param.yaml`                                                                |

## Input and output

|              | Content                                                                                                                   |
| ------------ | ------------------------------------------------------------------------------------------------------------------------- |
| Input        | `camera_info`, `image_raw/compressed`, `/tf` and `/tf_static` from `--input-bag`                                          |
| First stage  | `merged_signals` (`TrafficLightArray`) and `selected_rois` (`TrafficLightRoiArray`) from `TrafficLightRecognition::run()` |
| Second stage | `traffic_light_groups` (`TrafficLightGroupArray`) from `MultiCameraFusion::fuse()`                                        |

|                               | Topic                                                                            |
| ----------------------------- | -------------------------------------------------------------------------------- |
| Input (image)                 | `/sensing/camera/camera<N>/image_raw/compressed`                                 |
| Input (camera_info)           | `/sensing/camera/camera<N>/camera_info`                                          |
| First stage (traffic_signals) | `/perception/traffic_light_recognition/camera<N>/classification/traffic_signals` |
| First stage (rois)            | `/perception/traffic_light_recognition/camera<N>/detection/rois`                 |
| Second stage                  | `/perception/traffic_light_recognition/internal/traffic_signals`                 |
