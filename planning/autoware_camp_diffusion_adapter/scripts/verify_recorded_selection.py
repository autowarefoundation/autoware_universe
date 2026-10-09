#!/usr/bin/env python3
"""Audit a genuine ROS bag; this script neither runs nor renders a simulator."""
import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--storage-id", default="sqlite3")
    parser.add_argument("--pool-topic", default="/planning/camp/candidate_pool")
    parser.add_argument("--accepted-topic", default="/planning/camp/accepted_selection")
    parser.add_argument("--trajectory-topic", default="/planning/camp/trajectory")
    parser.add_argument("--turn-topic", default="/planning/camp/turn_indicators")
    parser.add_argument("--objects-topic", default="/planning/camp/predicted_objects")
    parser.add_argument("--odometry-topic", default="/localization/kinematic_state")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
    from autoware_planning_msgs.msg import Trajectory

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag.resolve()), storage_id=args.storage_id),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    type_names = {x.name: x.type for x in reader.get_all_topics_and_types()}
    wanted = [args.pool_topic, args.accepted_topic, args.trajectory_topic, args.turn_topic,
              args.objects_topic, args.odometry_topic]
    missing = [x for x in wanted if x not in type_names]
    if missing:
        raise ValueError("bag missing required observed topics: " + ", ".join(missing))
    types = {x: get_message(type_names[x]) for x in wanted}
    digest = lambda msg: hashlib.sha256(serialize_message(msg)).hexdigest()
    stamp = lambda msg: (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id)
    pools, selections, trajectories, turns, objects = {}, {}, {}, set(), Counter()
    counts, rows, errors = Counter(), Counter(), []
    odometry_positions = []
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic not in types:
            continue
        message = deserialize_message(payload, types[topic])
        counts[topic] += 1
        if topic == args.pool_topic:
            if message.pool_id in pools:
                errors.append("duplicate pool_id: " + str(message.pool_id))
            if len(message.candidates.candidate_trajectories) != 8 or len(message.raw_atoms) != 128:
                errors.append("invalid frozen pool dimensions: " + str(message.pool_id))
            original = []
            for candidate in message.candidates.candidate_trajectories:
                trajectory = Trajectory()
                trajectory.header = candidate.header
                trajectory.points = candidate.points
                original.append((digest(candidate), digest(trajectory),
                                 digest(candidate.turn_indicators_command)))
            pools[message.pool_id] = (stamp(message), original)
        elif topic == args.accepted_topic:
            if message.pool_id in selections:
                errors.append("duplicate accepted pool_id: " + str(message.pool_id))
            selections[message.pool_id] = (stamp(message), message.selected_index,
                                          digest(message.candidate))
        elif topic == args.trajectory_topic:
            trajectories.setdefault(stamp(message), set()).add(digest(message))
        elif topic == args.turn_topic:
            turns.add(digest(message))
        elif topic == args.objects_topic:
            objects[stamp(message)] += 1
        elif topic == args.odometry_topic:
            p = message.pose.pose.position
            odometry_positions.append((p.x, p.y, p.z))
    for pool_id, (header, row, candidate_hash) in selections.items():
        if pool_id not in pools:
            errors.append("accepted selection missing original pool: " + str(pool_id))
            continue
        pool_header, original = pools[pool_id]
        if header != pool_header or row >= len(original):
            errors.append("accepted selection has wrong frame/row: " + str(pool_id))
            continue
        candidate, trajectory, turn = original[row]
        if candidate_hash != candidate or trajectory not in trajectories.get(header, set()) or turn not in turns:
            errors.append("published output differs from original row: " + str(pool_id))
            continue
        if objects[header] == 0:
            errors.append("selected row lacks same-stamp predicted objects: " + str(pool_id))
            continue
        rows[row] += 1
    moved = len(set(odometry_positions)) > 1
    nonzero = sum(count for row, count in rows.items() if row > 0)
    result = {"bag": str(args.bag.resolve()), "observed_messages": dict(counts),
              "accepted_rows_matched_to_original_outputs": dict(rows),
              "nonzero_matched_ticks": nonzero, "odometry_changed": moved, "errors": errors,
              "passed": bool(nonzero and moved and not errors),
              "limits": ["No verification of video pixels or clock alignment with screen capture",
                         "Objects checked for same stamp only; actor path parity requires ROS/CUDA testing",
                         "Published plan acceptance is not a physical controller execution receipt",
                         "No safety, quality, ADE/FDE or latency conclusion"]}
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
