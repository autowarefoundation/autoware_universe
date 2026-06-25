# Offline point cloud concatenation (`offline_concat.py`)

`offline_concat.py` runs the **same C++ concatenation core that the ROS node uses**, but offline and
without any running ROS graph. It reads the lidar clouds, the twist, and `/tf_static` straight out of
a rosbag2 SQLite store and feeds them to the pybind `Concatenator`
(`autoware_pointcloud_preprocessor.concatenate_pointclouds`) in the bag's record (arrival) order. This
makes the offline run reproduce the online node's grouping and concatenation, frame for frame, so you
can debug / regression-test the concat behavior from a recorded bag.

## What it does, step by step

1. **Reads the bag directly via SQLite** (`_open_bag` / `_iter_messages`) — no `rosbag2_py` or storage
   plugins required. Messages are pulled `order by timestamp`, i.e. in **record (arrival) order**.
2. **Discovers the input topics**: every `sensor_msgs/msg/PointCloud2` topic under `--lidar-prefix`
   (default `/sensing/lidar/`), excluding the node's own output
   (`/sensing/lidar/concatenated/pointcloud`).
3. **Composes the static TF chain** (`build_static_transforms`): the bag stores `/tf_static` as a chain
   (`base_link -> sensor_kit_base_link -> ... -> lidar`); the online node would resolve this with
   `lookupTransform`. Offline, the script walks child→parent edges up to `--output-frame` and multiplies
   the homogeneous matrices itself, producing one `output_frame <- sensor_frame` extrinsic per lidar.
4. **Feeds the stream to the `Concatenator`**: each cloud is passed with `arrival_time = messages.timestamp * 1e-9`
   — the offline analog of the wall-clock arrival the node sees — which drives the per-collector
   timeout exactly as the node's wall-clock timer would. Twist messages are fed via `process_twist`.
5. **Reports each emitted group** (`COMPLETE` / `TIMEOUT`) with point count, point step, output frame,
   and whether twist / extrinsics were missing.

## How I ran it

> **Use the ROS python (python3.10), NOT conda.** The pybind extension is built against the ROS python;
> a conda interpreter will fail to import the bindings. Drop miniconda from `PATH` first if you are in a
> conda env.

```bash
# 0. (if in a conda env) drop conda from PATH, then source ROS + the workspace
source /opt/ros/humble/setup.bash
source install/setup.bash   # after building autoware_pointcloud_preprocessor

cd src/autoware/universe/sensing/autoware_pointcloud_preprocessor

# 1. naive, quick look (no param file; topics taken alphabetically, CLI flags only):
python3 offline_concat.py /path/to/rosbag2_before_concat --strategy naive --timeout 0.1

# 2. advanced, mirroring the node (RECOMMENDED): the param file carries the input-topic order the
#    positional offsets are aligned to, so you cannot mis-order them.
python3 offline_concat.py /path/to/rosbag2_before_concat \
    --param-file src/autoware/launcher/aip_launcher/aip_x2_gen2_launch/config/concatenate_and_time_sync_node.param.yaml
```

The script accepts either a rosbag2 directory (it picks the first `*.db3`) or a single `.db3` file.

### Why `--param-file` for advanced

The advanced strategy's `lidar_timestamp_offsets` and `lidar_timestamp_noise_window` are **positional** —
aligned to the `input_topics` order. Passing them on the CLI (`--offsets` / `--noise`) means matching the
order by hand against the printed topic list, which is easy to get wrong. `--param-file` reads the topic
order and the offsets/noise from the **same file** the node uses, so they cannot be mis-ordered. CLI flags
(`--strategy`, `--timeout`, `--no-motion-compensation`) still override the param file where given.

### Key flags

| Flag                          | Meaning                                                                                                          |
| ----------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| `bag`                         | rosbag2 directory or a `.db3` file (positional)                                                                  |
| `--param-file`                | `concatenate_and_time_sync_node.param.yaml`; mirrors the node (topic order + strategy + offsets/noise + timeout) |
| `--strategy {naive,advanced}` | override matching strategy (default: from param file, else `naive`)                                              |
| `--timeout SEC`               | per-group timeout (default: from param file, else `0.1`)                                                         |
| `--output-frame`              | target frame for the concatenated cloud (default `base_link`)                                                    |
| `--lidar-prefix`              | topics under this prefix are treated as concat inputs (default `/sensing/lidar/`)                                |
| `--twist-topic`               | twist topic for motion compensation                                                                              |
| `--no-motion-compensation`    | disable twist motion compensation                                                                                |
| `--offsets` / `--noise`       | advanced offsets/noise on the CLI (use `--param-file` instead to avoid mis-ordering)                             |
| `--limit N`                   | stop after N complete groups (0 = all)                                                                           |

### A note on the bag

The arrival timestamps must be **real** — record with `ros2 bag record` (rate 1) on the vehicle / in real
time. A slow re-transcode or `bag play` at reduced rate stretches `messages.timestamp` into large gaps,
which times out every group (0 complete). That is a bag artifact, not a script bug.

## The Python API this script is built on

The script is a thin driver over the `Concatenator` class in
[`python/autoware_pointcloud_preprocessor/concatenate_pointclouds.py`](python/autoware_pointcloud_preprocessor/concatenate_pointclouds.py).
The minimal usage, stripped of the bag-reading and TF-composition plumbing, is:

```python
from autoware_pointcloud_preprocessor.concatenate_pointclouds import (
    CollectorStatus,
    Concatenator,
)

# tf_static: {sensor_frame -> geometry_msgs/TransformStamped}, each with
#   header.frame_id = output_frame, child_frame_id = sensor_frame
# (offline_concat.py builds these by composing the /tf_static chain; see build_static_transforms)

concat = Concatenator(
    input_topics=lidar_topics,           # list[str], order matters for advanced offsets/noise
    output_frame="base_link",
    tf_static=tf_static,                 # dict[str, TransformStamped]
    timeout_sec=0.2,                     # per-group timeout, in arrival time
    is_motion_compensated=True,
    matching_strategy="advanced",        # or "naive"
    lidar_timestamp_offsets=offsets,     # advanced only: one per input topic (seconds), same order
    lidar_timestamp_noise_window=noise,  # advanced only: one per input topic (seconds), same order
)

# Feed messages in ARRIVAL order. arrival_time is in seconds (e.g. bag record ts * 1e-9).
for topic, arrival_time_sec, msg in stream:        # your iterator, in record order
    if topic == twist_topic:
        concat.process_twist(twist_msg)            # TwistWithCovarianceStamped
        continue
    # msg is a sensor_msgs/PointCloud2 (rclpy object)
    for frame in concat.process_cloud(topic, msg, arrival_time=arrival_time_sec):
        cloud = frame.result.concatenated_cloud    # sensor_msgs/PointCloud2 (or None)
        if frame.status == CollectorStatus.COMPLETE:
            ...  # all input topics contributed
        elif frame.status == CollectorStatus.TIMEOUT:
            ...  # group closed before all topics arrived
        # frame.result also carries: concatenation_info, no_twist_available,
        # twist_time_gap_too_large, topic_to_original_stamp, dropped_frames_missing_transform

# Once the stream is exhausted, flush any still-open groups (emitted as TIMEOUT):
for frame in concat.flush():
    ...
```

### API surface

- **`Concatenator(...)`** — stateful, arrival-driven collector on top of the C++ core. Construct once
  with the input topics, output frame, the injected extrinsics, and the matching config. For
  `advanced`, `lidar_timestamp_offsets` and `lidar_timestamp_noise_window` are **required** (one value
  per input topic, in seconds, in the same order as `input_topics`).
- **`process_cloud(topic, cloud, arrival_time)`** → `list[ConcatenatedFrame]` — add one cloud and get
  back any groups that just resolved. Usually empty (still collecting) or one entry; can be several if
  more than one group resolves at once. **`arrival_time` (seconds) is required** — it drives both the
  naive matching window and the per-collector timeout. **Feed clouds in arrival order.**
- **`process_twist(twist)`** / **`process_odometry(odometry)`** — feed velocity in timestamp order, for
  motion compensation.
- **`flush()`** → `list[ConcatenatedFrame]` — emit every still-open group as a `TIMEOUT`; call once when
  the stream is exhausted.
- **`ConcatenatedFrame`** — `(status, result)`. `status` is `CollectorStatus.COMPLETE` or
  `CollectorStatus.TIMEOUT`.
- **`ConcatenationResult`** — the per-group output: `concatenated_cloud` (`PointCloud2`),
  `concatenation_info`, `transformed_clouds` (per-topic, only when `publish_synchronized_pointcloud` is
  enabled), `no_twist_available`, `twist_time_gap_too_large`, `topic_to_original_stamp`,
  `dropped_frames_missing_transform` (sensor frames dropped because no extrinsic was provided).

A lower-level **`CombineCloudHandler`** is also available: it combines an **already-collected** group
(`{topic -> PointCloud2}`) in a single `combine_pointclouds(...)` call, doing the transform, motion
compensation, and concatenation, but without the grouping/timeout logic. Use it when you have already
grouped the clouds yourself and only want the combine step. See the module docstring for details.

### Helpers in `offline_concat.py` you can reuse

- **`build_static_transforms(tf_messages, output_frame, sensor_frames)`** — composes a `/tf_static`
  chain into one `output_frame <- sensor_frame` `TransformStamped` per sensor (the offline replacement
  for `lookupTransform`).
- **`_iter_messages` / `_open_bag` / `_find_db3`** — read a rosbag2 SQLite store directly, in record
  order, without `rosbag2_py`.

## Related

- Unit tests: `test/test_concatenate_pointclouds_py.py` (Python collector behavior) and
  `test/test_matching_policy.cpp` (the shared matching core).
