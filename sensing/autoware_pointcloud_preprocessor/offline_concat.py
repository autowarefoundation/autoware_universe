#!/usr/bin/env python3
# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run the point cloud concatenation core (pybind) over a recorded rosbag, offline.

This drives the *same* C++ concatenation core the ROS node uses, via the
``autoware_pointcloud_preprocessor.concatenate_pointclouds`` Python bindings -- but without any
running ROS graph. It reads the lidar clouds, twist, and ``/tf_static`` straight out of the bag's
SQLite store, composes the static TF chain down to a single ``output_frame <- sensor_frame``
extrinsic per lidar (the online node gets this from its TF buffer's lookupTransform; offline we do
the chain multiplication ourselves), then feeds everything to the offline ``Concatenator`` in the
bag's record (arrival) order, passing each message's record timestamp as ``arrival_time`` so the
per-collector timeout matches the online node's wall-clock timer.

Run it with the ROS python (NOT conda), after building+sourcing the package:

    # drop conda from PATH first if you are in a conda env
    source /opt/ros/humble/setup.bash
    source install/setup.bash

    # naive, quick look:
    python3 offline_concat.py rosbag2_before_concat --strategy naive --timeout 0.1

    # advanced, mirroring the node (recommended -- the param file carries the input-topic order the
    # positional offsets are aligned to, so you cannot mis-order them):
    python3 offline_concat.py rosbag2_before_concat --param-file \
        .../config/concatenate_and_time_sync_node.param.yaml
"""

import argparse
import glob
import os
import sqlite3
import sys
from typing import Dict
from typing import List
from typing import Tuple

import numpy as np


def _err(msg: str) -> None:
    print(f"error: {msg}", file=sys.stderr)
    sys.exit(1)


def _check_env() -> None:
    try:
        from autoware_pointcloud_preprocessor import concatenate_pointclouds  # noqa: F401
        import rclpy.serialization  # noqa: F401
    except Exception as exc:  # pragma: no cover - environment guard
        _err(
            "cannot import the bindings: "
            f"{exc}\n"
            "Are you using the ROS python (python3.10, NOT conda) and did you "
            "`source install/setup.bash` after building autoware_pointcloud_preprocessor?"
        )


# --------------------------------------------------------------------------------------------------
# rosbag2 (sqlite3) reading -- done directly so this needs no rosbag2_py / storage plugins.
# --------------------------------------------------------------------------------------------------
def _find_db3(bag_path: str) -> str:
    if os.path.isfile(bag_path) and bag_path.endswith(".db3"):
        return bag_path
    matches = sorted(glob.glob(os.path.join(bag_path, "*.db3")))
    if not matches:
        _err(f"no .db3 file found under {bag_path!r}")
    return matches[0]


def _open_bag(db3: str):
    con = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
    cur = con.cursor()
    topics = {}  # name -> (id, type)
    for tid, name, typ in cur.execute("select id, name, type from topics"):
        topics[name] = (tid, typ)
    return con, cur, topics


def _iter_messages(cur, topics: Dict[str, Tuple[int, str]], names: List[str]):
    """Yield (topic_name, bag_timestamp_ns, raw_cdr_bytes) for the given topics, in time order."""
    ids = {topics[n][0]: n for n in names if n in topics}
    if not ids:
        return
    placeholders = ",".join("?" * len(ids))
    query = (
        f"select topic_id, timestamp, data from messages "
        f"where topic_id in ({placeholders}) order by timestamp"
    )
    for tid, ts, data in cur.execute(query, tuple(ids.keys())):
        yield ids[tid], ts, bytes(data)


# --------------------------------------------------------------------------------------------------
# Static TF: compose the chain into a single output_frame <- sensor_frame transform per lidar.
# --------------------------------------------------------------------------------------------------
def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array(
        [
            [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
            [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
            [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
        ]
    )


def _matrix_to_quat(r: np.ndarray) -> Tuple[float, float, float, float]:
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def _transform_matrix(t) -> np.ndarray:
    tr = t.transform.translation
    q = t.transform.rotation
    m = np.eye(4)
    m[:3, :3] = _quat_to_matrix(q.x, q.y, q.z, q.w)
    m[:3, 3] = (tr.x, tr.y, tr.z)
    return m


def build_static_transforms(tf_messages, output_frame: str, sensor_frames: List[str]):
    """Compose the static TF tree into one ``output_frame <- sensor_frame`` TransformStamped each.

    The bag stores the TF as a chain (e.g. base_link -> sensor_kit_base_link -> .../lidar_base_link
    -> .../lidar); the node would resolve this via lookupTransform. We replicate that by walking
    child->parent edges up to ``output_frame`` and multiplying the homogeneous matrices.
    """
    from geometry_msgs.msg import TransformStamped

    # child_frame -> (parent_frame, M) where p_parent = M @ p_child
    edge: Dict[str, Tuple[str, np.ndarray]] = {}
    for tf in tf_messages:
        for t in tf.transforms:
            edge[t.child_frame_id] = (t.header.frame_id, _transform_matrix(t))

    resolved: Dict[str, TransformStamped] = {}
    for sensor in sensor_frames:
        node = sensor
        total = np.eye(4)
        ok = True
        while node != output_frame:
            if node not in edge:
                ok = False
                break
            parent, m = edge[node]
            total = m @ total  # p_parent = m @ (p_node), accumulate up the chain
            node = parent
        if not ok:
            print(f"  WARN: no static TF chain {output_frame} <- {sensor}; will be dropped")
            continue
        out = TransformStamped()
        out.header.frame_id = output_frame
        out.child_frame_id = sensor
        out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = (
            float(total[0, 3]),
            float(total[1, 3]),
            float(total[2, 3]),
        )
        qx, qy, qz, qw = _matrix_to_quat(total[:3, :3])
        (
            out.transform.rotation.x,
            out.transform.rotation.y,
            out.transform.rotation.z,
            out.transform.rotation.w,
        ) = (qx, qy, qz, qw)
        resolved[sensor] = out
    return resolved


def _load_param_file(path: str) -> Dict:
    """Pull the concat settings out of a ``concatenate_and_time_sync_node.param.yaml``.

    The advanced ``lidar_timestamp_offsets`` are **positional** (aligned to ``input_topics``), so the
    only reliable way to mirror the node is to take the topic order and the offsets/noise from the
    same file. Returns input_topics, strategy, offsets, noise, timeout_sec, is_motion_compensated.
    """
    import yaml

    with open(path) as fh:
        doc = yaml.safe_load(fh)
    try:
        params = doc["/**"]["ros__parameters"]
    except (KeyError, TypeError):
        _err(f"{path!r} is not a /**:ros__parameters param file")
    ms = params.get("matching_strategy", {})
    return {
        "input_topics": list(params.get("input_topics", [])),
        "strategy": ms.get("type", "naive"),
        "offsets": list(ms.get("lidar_timestamp_offsets", []) or []),
        "noise": list(ms.get("lidar_timestamp_noise_window", []) or []),
        "timeout_sec": params.get("timeout_sec"),
        "is_motion_compensated": params.get("is_motion_compensated", True),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="rosbag2 directory or a .db3 file")
    parser.add_argument("--output-frame", default="base_link")
    parser.add_argument(
        "--lidar-prefix",
        default="/sensing/lidar/",
        help="PointCloud2 topics under this prefix are treated as concat inputs",
    )
    parser.add_argument(
        "--twist-topic",
        default="/sensing/vehicle_velocity_converter/twist_with_covariance",
    )
    parser.add_argument(
        "--exclude-topic",
        default="/sensing/lidar/concatenated/pointcloud",
        help="PointCloud2 topic to exclude from inputs (the node's own concat output)",
    )
    parser.add_argument(
        "--param-file",
        help="concatenate_and_time_sync_node.param.yaml; mirrors the node (input_topics order + "
        "strategy + offsets/noise + timeout). Recommended for advanced. CLI flags override it.",
    )
    parser.add_argument(
        "--strategy",
        choices=["naive", "advanced"],
        default=None,
        help="override the matching strategy (default: from --param-file, else naive)",
    )
    parser.add_argument(
        "--timeout", type=float, default=None, help="group timeout [s] (default: param, else 0.1)"
    )
    parser.add_argument(
        "--no-motion-compensation", action="store_true", help="disable twist motion compensation"
    )
    parser.add_argument(
        "--offsets",
        type=float,
        nargs="+",
        help="advanced strategy: per-topic lidar_timestamp_offsets, aligned to the printed "
        "input-topic order (use --param-file instead to avoid mis-ordering)",
    )
    parser.add_argument(
        "--noise",
        type=float,
        nargs="+",
        help="advanced strategy: per-topic lidar_timestamp_noise_window (same order as --offsets)",
    )
    parser.add_argument("--limit", type=int, default=0, help="stop after N complete groups (0=all)")
    args = parser.parse_args()

    _check_env()
    from autoware_pointcloud_preprocessor.concatenate_pointclouds import CollectorStatus
    from autoware_pointcloud_preprocessor.concatenate_pointclouds import Concatenator
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    db3 = _find_db3(args.bag)
    con, cur, topics = _open_bag(db3)
    print(f"bag: {db3}")

    # Which PointCloud2 topics under the prefix are present (the node's own output excluded).
    present = {
        n
        for n, (_tid, typ) in topics.items()
        if typ == "sensor_msgs/msg/PointCloud2"
        and n.startswith(args.lidar_prefix)
        and n != args.exclude_topic
    }
    if not present:
        _err(f"no PointCloud2 topics under {args.lidar_prefix!r}")

    # Resolve input-topic order + matching config. A --param-file mirrors the node exactly (the
    # advanced offsets are POSITIONAL, so the input_topics order MUST match them); without it we fall
    # back to alphabetical topics + the CLI flags. CLI flags override the param file where given.
    if args.param_file:
        pf = _load_param_file(args.param_file)
        strategy = args.strategy or pf["strategy"]
        # Keep only param topics present in the bag, preserving order AND their paired offset/noise.
        n_param = len(pf["input_topics"])
        triples = [
            (t, o, w)
            for t, o, w in zip(
                pf["input_topics"],
                pf["offsets"] or [0.0] * n_param,
                pf["noise"] or [0.0] * n_param,
            )
            if t in present
        ]
        missing = [t for t in pf["input_topics"] if t not in present]
        if missing:
            print(f"  WARN: param input_topics not in bag (skipped): {missing}")
        lidar_topics = [t for t, _o, _w in triples]
        offsets = [o for _t, o, _w in triples] if strategy == "advanced" else None
        noise = [w for _t, _o, w in triples] if strategy == "advanced" else None
        timeout = args.timeout if args.timeout is not None else pf["timeout_sec"]
        motion = (not args.no_motion_compensation) and pf["is_motion_compensated"]
    else:
        strategy = args.strategy or "naive"
        lidar_topics = sorted(present)
        offsets = args.offsets
        noise = args.noise
        timeout = args.timeout if args.timeout is not None else 0.1
        motion = not args.no_motion_compensation

    if not lidar_topics:
        _err("no input topics resolved (param input_topics not present in the bag?)")
    print(f"strategy={strategy} timeout={timeout} motion_compensated={motion}")
    print(f"input topics ({len(lidar_topics)}):")
    for t in lidar_topics:
        print(f"  {t}")

    pc_type = get_message("sensor_msgs/msg/PointCloud2")
    tf_type = get_message("tf2_msgs/msg/TFMessage")

    # Peek one cloud per topic to map topic -> sensor frame_id.
    topic_to_frame: Dict[str, str] = {}
    for t in lidar_topics:
        tid = topics[t][0]
        row = cur.execute(
            "select data from messages where topic_id=? order by timestamp limit 1", (tid,)
        ).fetchone()
        if row is None:
            _err(f"topic {t} has no messages")
        topic_to_frame[t] = deserialize_message(bytes(row[0]), pc_type).header.frame_id
    sensor_frames = list(topic_to_frame.values())

    # Compose the static TF chain to a single output_frame <- sensor extrinsic per lidar.
    tf_msgs = []
    if "/tf_static" in topics:
        tid = topics["/tf_static"][0]
        for (_data,) in cur.execute(
            "select data from messages where topic_id=? order by timestamp", (tid,)
        ):
            tf_msgs.append(deserialize_message(bytes(_data), tf_type))
    else:
        print("  WARN: bag has no /tf_static; every cloud will be dropped (no extrinsics)")
    tf_static = build_static_transforms(tf_msgs, args.output_frame, sensor_frames)
    print(f"resolved {len(tf_static)}/{len(sensor_frames)} extrinsics to {args.output_frame}")

    if strategy == "advanced" and (not offsets or not noise):
        _err("advanced strategy needs offsets+noise: pass --param-file, or --offsets and --noise")

    concat = Concatenator(
        input_topics=lidar_topics,
        output_frame=args.output_frame,
        tf_static=tf_static,
        timeout_sec=timeout,
        is_motion_compensated=motion,
        matching_strategy=strategy,
        lidar_timestamp_offsets=offsets,
        lidar_timestamp_noise_window=noise,
    )

    n_complete = 0
    n_timeout = 0
    stream_topics = list(lidar_topics) + [args.twist_topic]

    def handle_frames(frames):
        nonlocal n_complete, n_timeout
        for fr in frames:
            cc = fr.result.concatenated_cloud
            if fr.status == CollectorStatus.COMPLETE:
                n_complete += 1
            else:
                n_timeout += 1
            width = cc.width if cc is not None else 0
            dropped = fr.result.dropped_frames_missing_transform
            print(
                f"  [{fr.status:8s}] points={width:7d} step={cc.point_step if cc else 0} "
                f"frame={cc.header.frame_id if cc else '-'} "
                f"no_twist={fr.result.no_twist_available} dropped={dropped}"
            )

    print("\nprocessing...")
    # _iter_messages yields in bag-record (arrival) order; the record timestamp (ns) is the offline
    # analog of the wall-clock arrival the online node sees, so we feed it as arrival_time to drive
    # the per-collector timeout exactly as the node's wall-clock timer would.
    for topic, bag_ts, raw in _iter_messages(cur, topics, stream_topics):
        if topic == args.twist_topic:
            tw = deserialize_message(raw, get_message(topics[topic][1]))
            concat.process_twist(tw)
            continue
        cloud = deserialize_message(raw, pc_type)
        handle_frames(concat.process_cloud(topic, cloud, arrival_time=bag_ts * 1e-9))
        if args.limit and n_complete >= args.limit:
            break

    if not args.limit:
        handle_frames(concat.flush())

    print(f"\ndone: {n_complete} complete, {n_timeout} timeout groups")
    con.close()


if __name__ == "__main__":
    main()
