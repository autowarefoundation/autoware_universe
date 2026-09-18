# bag2bag runner

This tool reads a recorded rosbag, runs the ROS-free core of this package on every frame, and writes the results to a new rosbag. It never calls `rclcpp::init` and uses no executor and no DDS. So it does not have to follow the recording rate of the bag, and the same input always gives the same output bag.

| Executable                              | Cores it runs                                                                | Output                                                                                |
| --------------------------------------- | ---------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| `traffic_light_pipeline_bag2bag_runner` | First stage (`TrafficLightRecognition`) + second stage (`MultiCameraFusion`) | `merged_signals` / `selected_rois` per camera, and the fused `TrafficLightGroupArray` |

The second stage stops after `MultiCameraFusion::fuse()`. In production the data then goes to the arbiter and to crosswalk_traffic_light_estimator, but this package has no fusion Node yet, so the tool stops here. Its output matches the production topic `/perception/traffic_light_recognition/internal/traffic_signals`.

## How to use

```bash
ros2 run autoware_traffic_light_pipeline traffic_light_pipeline_bag2bag_runner \
  --input-bag <path to the input bag> \
  --map <path to the map directory> \
  --output-bag result/output_bag \
  --camera 4,5
```

| Argument          | Meaning                                                                                                                                                                                                                                                                                                                                                    |
| ----------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--input-bag`     | Directory of the input bag                                                                                                                                                                                                                                                                                                                                 |
| `--map`           | Map directory. It must contain exactly one `.osm` file (a lanelet2 map; only the MGRS frame is supported) and exactly one `.yaml` file (projector info). The file names do not matter. If either one is missing, or there is more than one, the tool stops with an error, so that it never runs silently on the wrong map                                  |
| `--output-bag`    | Directory of the output bag. An existing bag is overwritten. If the directory does not look like a rosbag2 bag, the tool refuses to delete it                                                                                                                                                                                                              |
| `--camera`        | The camera numbers recorded in the bag. `4,5` means camera4 and camera5. One `TrafficLightRecognition` instance is created for each entry                                                                                                                                                                                                                  |
| `--ml-model-path` | Directory that holds the ML artifacts. The default is `$HOME/autoware_data`, the same as the `data_path` argument of the launch file                                                                                                                                                                                                                       |
| `--config`        | Path to the recognition config. The default is this package's installed `config/traffic_light_recognition.param.yaml`, the same file the launch file gives to the Node. Give a copy of that file here when you want to try other values without editing the installed one. The copy must keep the same layout, because it is read as a Node parameter file |

Both `--flag value` and `--flag=value` are accepted.

## Input and output

|              | Content                                                                                                                   |
| ------------ | ------------------------------------------------------------------------------------------------------------------------- |
| Input        | `camera_info`, `image_raw/compressed`, `/tf` and `/tf_static` from `--input-bag`                                          |
| First stage  | `merged_signals` (`TrafficLightArray`) and `selected_rois` (`TrafficLightRoiArray`) from `TrafficLightRecognition::run()` |
| Second stage | `traffic_light_groups` (`TrafficLightGroupArray`) from `MultiCameraFusion::fuse()`                                        |

The topic names come from the camera names given by `--camera`, and all of them are the production names. They are derived in the same way as the `input/*` and `output/*` defaults in the launch file of the first stage; only the output topic of the second stage is a constant.

|                               | Topic                                                                            |
| ----------------------------- | -------------------------------------------------------------------------------- |
| Input (image)                 | `/sensing/camera/camera<N>/image_raw/compressed`                                 |
| Input (camera_info)           | `/sensing/camera/camera<N>/camera_info`                                          |
| First stage (traffic_signals) | `/perception/traffic_light_recognition/camera<N>/classification/traffic_signals` |
| First stage (rois)            | `/perception/traffic_light_recognition/camera<N>/detection/rois`                 |
| Second stage                  | `/perception/traffic_light_recognition/internal/traffic_signals`                 |

- An image and a camera_info are paired only when their header stamps are exactly equal, the same rule as `message_filters::ExactTime` in production. A message with no partner is dropped.
- Each message is written at the time of its header stamp, not at the time the tool runs.
- The output bag uses the same storage format as the input bag.
- If `run()` fails for a frame, or an image cannot be decoded, the tool prints a line to stderr and skips that frame. This is the same behavior as the Node.

## Processing order

The tool runs the first stage for all frames first, and only then runs the second stage for all of them. This is a different order from the Node graph, which calls the first stage and the second stage one after another for each frame. `MultiCameraFusion` keeps state, but it never reads a clock (it only looks at the header stamp of the trigger itself), so the same input sequence in the same order always produces the same output.

The first stage handles one camera at a time and frees that camera's frames before moving to the next camera. So memory use is proportional to the number of frames of one camera, not of all cameras together. The first stage keeps no state between cameras, so each result is the same either way. But the results come out grouped by camera, so they are sorted again into ascending `(stamp, camera_index)` order before they are passed to the second stage.

This order depends only on the input bag, so it is always deterministic. **However, it may not match a full-system run.** In production the two cameras run on different Jetsons, so which triple reaches multi_camera_fusion first inside one cycle depends on the latency of the first stage and changes every cycle. If `message_lifespan` is larger than the camera period, both orders still fuse two cameras, so the order only matters for frames where the two cameras disagree. In those frames the order decides whether cycle-N or cycle-N-1 of the other camera is mixed in, and the fused color can change. If you ever need an exact comparison, replay the execution order with a throwaway verification script instead of adding a flag to this tool.

## Where the settings come from

**This tool has no config file of its own.** It reads the config of the Node, so there is no second place where a threshold is written down. The implementation is also a single file, `traffic_light_pipeline_bag2bag_runner.cpp`.

- **All tuning values of the first stage** are read from this package's `config/traffic_light_recognition.param.yaml`. This is the same file the launch file gives to the Node, so each threshold is defined in one place only, and a default run always measures the production values. (`map_based_detector.min/max_timestamp_offset` works the same way: `CameraConfig` holds a value per camera, but every camera gets the value from the config.) You cannot change single values on the command line. To try other values, copy that file, edit the copy, and pass it with `--config`.
- **Topic names** are derived from the camera names, as shown in the table above.
- **The file names of the ML artifacts** are also in the package config. Only their location (`--ml-model-path`) is a command line argument, because it is machine specific.
- **The settings of the second stage** (`message_lifespan: 0.12`, `prior_log_odds: 0.0`, and three disable flags) are constants in the source. This package has no fusion Node, and no `config/*.param.yaml` that a launch file would pass to one. The tool also does not read the package config of `autoware_traffic_light_multi_camera_fusion` itself, because `message_lifespan: 0.09` there is not the production value: autoware_launch overrides it with 0.12. See the comment in the source for why a value below the camera period of 0.100005 s makes every trigger fuse only one camera.

Be careful when you use `--ml-model-path` to point to the same `.onnx` file in a different place. Even if the `.onnx` is identical, the TensorRT `.engine` cached next to it may have been built with a different version, and then the fp16 results change slightly. (Measured example: a confidence becomes exactly 1.0 instead of 0.99998.) `has_higher_or_equal_priority()` in `multi_camera_fusion` uses a strict comparison that keeps the later record when the confidence is equal, so this small difference alone can change the fused color. If you want the numbers to match production, the safe way is to share the engine too, that is, to point to the same path as production.
