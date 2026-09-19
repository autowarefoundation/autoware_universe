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

"""Record a minimal rosbag on Domain 1 for in-lane MRM offline analysis.

Run on Domain 1 (main ECU) while psim-main and psim-main-mrm are up.
Uses bridged /mrm/* topics plus main /localization/* for psim (no bidirectional bridge).

Usage:
  ros2 run autoware_in_lane_mrm_planner in_lane_mrm_planner_rosbag_record
  ros2 run autoware_in_lane_mrm_planner in_lane_mrm_planner_rosbag_record \\
    --output-dir ~/mrm_bags --duration 120
"""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import signal
import subprocess
import sys
import time

from in_lane_mrm_rosbag_common import DOMAIN1_RECORD_TOPICS
from in_lane_mrm_rosbag_common import TRAJECTORY_TOPIC_D1
from in_lane_mrm_rosbag_common import format_topic_help
from in_lane_mrm_rosbag_common import record_rosbag
from in_lane_mrm_rosbag_common import validate_rosbag


def default_bag_name() -> str:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"in_lane_mrm_d1_{timestamp}"


def parse_args() -> argparse.Namespace:
    epilog = (
        "Record on ROS Domain 1 only (same as vehicle logging).\n"
        "Requires domain_bridge from MRM domain; do not record on Domain 3.\n\n"
        "Topics:\n"
        f"{format_topic_help()}\n\n"
        "psim note: odom/accel use /localization/* on D1 (main psim). "
        "Vehicle uses /mrm/localization/* when reversed bridge logging is enabled."
    )
    parser = argparse.ArgumentParser(
        description="Record minimal rosbag on Domain 1 for in-lane MRM analysis.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=epilog,
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path.home() / "mrm_rosbags",
        help="Directory for ros2 bag output",
    )
    parser.add_argument(
        "--bag-name",
        type=str,
        default=None,
        help="Bag folder name (default: in_lane_mrm_d1_YYYYMMDD_HHMMSS)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="Recording duration in seconds (Ctrl+C stops early)",
    )
    parser.add_argument(
        "--countdown",
        type=int,
        default=3,
        help="Seconds to wait before starting record",
    )
    parser.add_argument(
        "--no-validate",
        action="store_true",
        help="Skip ros2 bag info validation after recording",
    )
    return parser.parse_args()


def countdown(seconds: int) -> None:
    for remaining in range(seconds, 0, -1):
        print(f"  {remaining}...")
        time.sleep(1.0)


def main() -> None:
    args = parse_args()
    topics = DOMAIN1_RECORD_TOPICS

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    bag_name = args.bag_name or default_bag_name()
    bag_path = output_dir / bag_name

    print("===== in_lane_mrm_planner rosbag record (Domain 1) =====")
    print(f"Output     : {bag_path}")
    print(f"Duration   : {args.duration:.1f} s")
    print(f"Topics ({len(topics)}):")
    for topic in topics:
        print(f"  - {topic}")

    if args.countdown > 0:
        print("Countdown before record:")
        countdown(args.countdown)

    process = record_rosbag(bag_path, topics)
    print(f"Recording... (pid {process.pid})")
    start_time = time.monotonic()
    interrupted = False

    def on_signal(signum, _frame):
        nonlocal interrupted
        interrupted = True
        print(f"\nSignal {signum} received, stopping record...")

    previous_int = signal.signal(signal.SIGINT, on_signal)
    previous_term = signal.signal(signal.SIGTERM, on_signal)

    try:
        while process.poll() is None:
            elapsed = time.monotonic() - start_time
            if not interrupted and elapsed >= args.duration:
                print("Duration reached, stopping record...")
                process.send_signal(signal.SIGINT)
                break
            time.sleep(0.2)
    finally:
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)

    try:
        process.wait(timeout=30.0)
    except subprocess.TimeoutExpired:
        process.kill()
        print("ERROR: ros2 bag record did not exit cleanly.")
        sys.exit(1)

    print(f"Bag saved: {bag_path}")
    print("Analyze with:")
    print(
        f"  ros2 run autoware_in_lane_mrm_planner in_lane_mrm_planner_rosbag_analyzer "
        f"{bag_path} --output-dir {bag_path}_report --no-show"
    )

    if args.no_validate:
        return

    if validate_rosbag(bag_path, [TRAJECTORY_TOPIC_D1]):
        print("Validation OK (required trajectory topic has messages).")
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
