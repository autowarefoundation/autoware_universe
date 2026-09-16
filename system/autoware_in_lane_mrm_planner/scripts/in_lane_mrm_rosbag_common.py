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

"""Shared helpers for in-lane MRM rosbag record / analyze scripts."""

from __future__ import annotations

import bisect
import csv
from dataclasses import asdict
from dataclasses import dataclass
from dataclasses import field
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

from autoware_planning_msgs.msg import Trajectory
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

# --- Domain 1 recording (vehicle + planning_simulator on main ECU) ---
# Bridged MRM topics use /mrm/... prefix on Domain 1 (reversed domain_bridge entries).
TRAJECTORY_TOPIC_D1 = "/mrm/planning/in_lane_mrm_planner/output/trajectory"
PLANNER_STATUS_TOPIC_D1 = "/mrm/planning/in_lane_mrm_planner/debug/planner_status"
TRIGGER_TOPIC_D1 = "/mrm/planning/in_lane_mrm_planner/input/trigger"
ODOM_TOPIC_D1 = "/mrm/localization/kinematic_state"
ACCEL_TOPIC_D1 = "/mrm/localization/acceleration"
# psim: main ECU publishes these on D1; MRM D3 consumes the bridged copy (no /mrm/ echo on D1).
ODOM_TOPIC_D1_MAIN = "/localization/kinematic_state"
ACCEL_TOPIC_D1_MAIN = "/localization/acceleration"
LONGITUDINAL_DIAG_TOPIC_D1 = "/mrm/control/trajectory_follower/longitudinal/diagnostic"
LATERAL_DIAG_TOPIC_D1 = "/mrm/control/trajectory_follower/lateral/diagnostic"
CONTROL_CMD_TOPIC_D1 = "/mrm/control/trajectory_follower/control_cmd"
MRM_DIAGNOSTICS_TOPIC_D1 = "/mrm/diagnostics"
GATE_CONTROL_TOPIC = "/control/control_command_gate/inputs/in_lane_stop/control"
GATE_GEAR_TOPIC = "/control/control_command_gate/inputs/in_lane_stop/gear"
# Domain 1 topic name (main -> mrm bridge source side)
JERK_TRIGGER_TOPIC_D1 = "/control/constant_jerk_deceleration_trigger"
OPERATION_MODE_TOPIC_D1 = "/system/operation_mode/state"

DOMAIN1_RECORD_TOPICS: Tuple[str, ...] = (
    TRAJECTORY_TOPIC_D1,
    PLANNER_STATUS_TOPIC_D1,
    TRIGGER_TOPIC_D1,
    ODOM_TOPIC_D1,
    ODOM_TOPIC_D1_MAIN,
    ACCEL_TOPIC_D1,
    ACCEL_TOPIC_D1_MAIN,
    LONGITUDINAL_DIAG_TOPIC_D1,
    LATERAL_DIAG_TOPIC_D1,
    CONTROL_CMD_TOPIC_D1,
    MRM_DIAGNOSTICS_TOPIC_D1,
    GATE_CONTROL_TOPIC,
    GATE_GEAR_TOPIC,
    JERK_TRIGGER_TOPIC_D1,
    OPERATION_MODE_TOPIC_D1,
)

DOMAIN1_ANALYZE_TOPICS: Tuple[str, ...] = DOMAIN1_RECORD_TOPICS

# Legacy D3 local names (analyzer only, for older bags recorded on MRM domain)
TRAJECTORY_TOPIC_D3 = "/planning/in_lane_mrm_planner/output/trajectory"
PLANNER_STATUS_TOPIC_D3 = "/planning/in_lane_mrm_planner/debug/planner_status"
TRIGGER_TOPIC_D3 = "/planning/in_lane_mrm_planner/input/trigger"
ODOM_TOPIC_D3 = "/localization/kinematic_state"
LONGITUDINAL_DIAG_TOPIC_D3 = "/control/trajectory_follower/longitudinal/diagnostic"
CONTROL_CMD_TOPIC_D3 = "/control/trajectory_follower/control_cmd"
LEGACY_ANALYZE_TOPICS: Tuple[str, ...] = (
    TRAJECTORY_TOPIC_D3,
    PLANNER_STATUS_TOPIC_D3,
    TRIGGER_TOPIC_D3,
    ODOM_TOPIC_D3,
    LONGITUDINAL_DIAG_TOPIC_D3,
    CONTROL_CMD_TOPIC_D3,
)

# Backward-compatible aliases used by the analyzer
TRAJECTORY_TOPIC = TRAJECTORY_TOPIC_D1
TRAJECTORY_TOPIC_FALLBACK = TRAJECTORY_TOPIC_D3
ODOM_TOPIC = ODOM_TOPIC_D1
LONGITUDINAL_DIAG_TOPIC = LONGITUDINAL_DIAG_TOPIC_D1
CONTROL_CMD_TOPIC = CONTROL_CMD_TOPIC_D1

VELOCITY_EPS = 0.01
DEFAULT_MOVING_ODOM_THRESHOLD = 0.5
DEFAULT_STOPPED_ODOM_THRESHOLD = 0.1
DEFAULT_NEGATIVE_STOP_DIST_THRESHOLD = -0.05

LONGITUDINAL_DEBUG_TARGET_VEL = 2
LONGITUDINAL_DEBUG_NEAREST_VEL = 4
LONGITUDINAL_DEBUG_CONTROL_STATE = 13
LONGITUDINAL_DEBUG_ACC_CMD_PUBLISHED = 18
LONGITUDINAL_DEBUG_EMERGENCY_FLAG = 23
LONGITUDINAL_DEBUG_STOP_DIST = 28

# Overshoot-emergency entry: stop_dist < -emergency_state_overshoot_stop_dist AND nearest
# target velocity < vel_epsilon. The MRM (redundancy sub) controller uses 0.8 m; the emergency
# clears (per updateControlState) when EITHER term is no longer satisfied.
# See autoware_pid_longitudinal_controller updateControlState() and
# config/control/.../longitudinal/pid.redundancy.sub.param.yaml.
EMERGENCY_OVERSHOOT_STOP_DIST_SUB = 0.8
EMERGENCY_NEAREST_VEL_EPS = 1.0e-3

# planner_status Float32MultiArrayStamped layout (see README.md)
PLANNER_STATUS_REASON_CODE = 0
PLANNER_STATUS_TRIGGER_ACTIVE = 1
PLANNER_STATUS_IS_LATCHED = 2
PLANNER_STATUS_HAS_LATEST_CANDIDATE = 3
PLANNER_STATUS_DATA_READY = 4
PLANNER_STATUS_PLAN_OK = 5
PLANNER_STATUS_VALIDATION_OK = 6
PLANNER_STATUS_PLANNED_POINTS = 7
PLANNER_STATUS_PUBLISHED_POINTS = 8
PLANNER_STATUS_CYCLE_TIME_MS = 9
PLANNER_STATUS_ODOM_VX = 10
PLANNER_STATUS_MIN_FIELDS = 11

REASON_CODE_NAMES: Dict[int, str] = {
    0: "published_ok",
    1: "waiting_map",
    2: "waiting_route",
    3: "waiting_odometry",
    4: "waiting_accel",
    10: "plan_failed_update_current_lanelet",
    11: "plan_failed_backward_lanelets",
    12: "plan_failed_forward_lanelets",
    13: "plan_failed_invalid_s_range",
    14: "plan_failed_generate_path",
    15: "plan_failed_no_output",
    20: "validation_failed_point_count",
    21: "validation_failed_non_finite",
    30: "latched_output_published",
    31: "latched_without_candidate",
    99: "unknown",
}

# Healthy publish paths; other codes mean trajectory was not updated that cycle.
PLANNER_REASON_OK_PUBLISH = frozenset({0, 30})

CONTROL_STATE_LABELS = {
    0: "DRIVE",
    1: "STOPPING",
    2: "STOPPED",
    3: "EMERGENCY",
}

ANOMALY_MOVING_ALL_ZERO = "moving_all_zero"
ANOMALY_MOVING_EARLY_ZERO = "moving_early_zero"
ANOMALY_MOVING_NEGATIVE_STOP_DIST = "moving_negative_stop_dist"
ANOMALY_STOPPED_NONZERO_TRAJ = "stopped_but_traj_nonzero"
ANOMALY_PLANNER_MOVING_NO_PUBLISH = "planner_moving_no_publish"


@dataclass
class PlannerStatusSample:
    time_sec: float
    reason_code: int
    reason_name: str
    trigger_active: bool
    is_latched: bool
    has_latest_candidate: bool
    data_ready: bool
    plan_ok: bool
    validation_ok: bool
    planned_points: int
    published_points: int
    cycle_time_ms: float
    odom_vx: float

    @property
    def trajectory_published(self) -> bool:
        return self.reason_code in PLANNER_REASON_OK_PUBLISH and self.published_points > 0


@dataclass
class TimeSeries:
    time_sec: List[float] = field(default_factory=list)
    values: List[float] = field(default_factory=list)

    def append(self, time_sec: float, value: float) -> None:
        self.time_sec.append(time_sec)
        self.values.append(value)

    def nearest(self, query_time_sec: float) -> Optional[float]:
        if not self.time_sec:
            return None
        index = bisect.bisect_left(self.time_sec, query_time_sec)
        if index == 0:
            return self.values[0]
        if index >= len(self.time_sec):
            return self.values[-1]
        before_time = self.time_sec[index - 1]
        after_time = self.time_sec[index]
        if abs(query_time_sec - before_time) <= abs(after_time - query_time_sec):
            return self.values[index - 1]
        return self.values[index]


@dataclass
class TrajectorySample:
    time_sec: float
    num_points: int
    v0: float
    v1: float
    v_min: float
    v_max: float
    first_zero_idx: Optional[int]
    all_zero: bool
    early_zero: bool
    odom_vx: Optional[float] = None
    stop_dist: Optional[float] = None
    control_state: Optional[int] = None
    gate_acc: Optional[float] = None
    follower_acc: Optional[float] = None
    planner_reason_code: Optional[int] = None
    planner_reason_name: Optional[str] = None
    planner_published_points: Optional[int] = None
    planner_is_latched: Optional[bool] = None
    anomaly_tags: List[str] = field(default_factory=list)


@dataclass
class AnomalyEvent:
    tag: str
    time_sec: float
    odom_vx: Optional[float]
    v0: float
    v_max: float
    stop_dist: Optional[float]
    control_state: Optional[int]
    all_zero: bool
    early_zero: bool


@dataclass
class EmergencyEpisode:
    """One contiguous span where the longitudinal controller state == EMERGENCY (3)."""

    start_sec: float
    end_sec: float
    min_stop_dist: Optional[float]
    max_stop_dist: Optional[float]
    min_follower_acc: Optional[float]
    odom_vx_at_start: Optional[float]
    odom_vx_min: Optional[float]
    # Would the overshoot-emergency condition have *cleared* at some point during the episode
    # while the vehicle was still moving? If so, an "exit EMERGENCY -> DRIVE before full stop"
    # change (MRM-only) could end the episode early instead of forcing a complete stop.
    cleared_while_moving: bool
    clear_reason: str

    @property
    def duration(self) -> float:
        return self.end_sec - self.start_sec


@dataclass
class RecordingGap:
    """A gap in message timestamps on a high-rate topic (recording interruption / overload)."""

    topic: str
    start_sec: float
    end_sec: float

    @property
    def duration(self) -> float:
        return self.end_sec - self.start_sec


def find_rising_edges(series: "TimeSeries", threshold: float = 0.5) -> List[float]:
    """Times where a 0/1 (or boolean-ish) series transitions from < threshold to >= threshold."""
    edges: List[float] = []
    prev: Optional[float] = None
    for time_sec, value in zip(series.time_sec, series.values):
        if prev is not None and prev < threshold <= value:
            edges.append(time_sec)
        prev = value
    return edges


def find_recording_gaps(
    time_sec: Sequence[float], topic: str, *, expected_dt: float, gap_factor: float = 5.0
) -> List[RecordingGap]:
    """Detect inter-message gaps larger than gap_factor * expected_dt on one topic."""
    gaps: List[RecordingGap] = []
    threshold = expected_dt * gap_factor
    for previous, current in zip(time_sec, time_sec[1:]):
        if current - previous > threshold:
            gaps.append(RecordingGap(topic=topic, start_sec=previous, end_sec=current))
    return gaps


def detect_emergency_episodes(
    control_state: "TimeSeries",
    *,
    stop_dist: "TimeSeries",
    follower_acc: "TimeSeries",
    odom_vx: "TimeSeries",
    nearest_vel: "TimeSeries",
    emergency_state_value: int = 3,
    overshoot_stop_dist: float = EMERGENCY_OVERSHOOT_STOP_DIST_SUB,
    nearest_vel_eps: float = EMERGENCY_NEAREST_VEL_EPS,
    moving_vx: float = DEFAULT_MOVING_ODOM_THRESHOLD,
    merge_gap_sec: float = 0.3,
) -> List[EmergencyEpisode]:
    """Find contiguous EMERGENCY spans and, for each, evaluate whether the overshoot-emergency
    condition would have cleared while the ego was still moving.

    The overshoot-emergency entry condition is (stop_dist < -overshoot_stop_dist AND
    nearest_target_vel < eps). It *clears* when stop_dist >= -overshoot_stop_dist OR
    nearest_target_vel >= eps. If that clear happens before the ego comes to rest, then allowing
    EMERGENCY -> DRIVE before a full stop (MRM-only) would shorten the episode; otherwise the
    vehicle reaches standstill regardless and a controller-side change alone cannot remove the
    stop. This is the key discriminator for the "Approach C" countermeasure.
    """
    # Build contiguous spans where state == EMERGENCY.
    raw_spans: List[Tuple[float, float]] = []
    span_start: Optional[float] = None
    prev_time: Optional[float] = None
    for time_sec, value in zip(control_state.time_sec, control_state.values):
        is_emergency = int(round(value)) == emergency_state_value
        if is_emergency and span_start is None:
            span_start = time_sec
        elif not is_emergency and span_start is not None:
            raw_spans.append((span_start, prev_time if prev_time is not None else time_sec))
            span_start = None
        prev_time = time_sec
    if span_start is not None and prev_time is not None:
        raw_spans.append((span_start, prev_time))

    # Merge spans separated by short gaps (debug topic jitter).
    merged: List[Tuple[float, float]] = []
    for start, end in raw_spans:
        if merged and start - merged[-1][1] <= merge_gap_sec:
            merged[-1] = (merged[-1][0], end)
        else:
            merged.append((start, end))

    def slice_values(series: "TimeSeries", start: float, end: float) -> List[float]:
        return [
            value
            for time_sec, value in zip(series.time_sec, series.values)
            if start <= time_sec <= end
        ]

    episodes: List[EmergencyEpisode] = []
    for start, end in merged:
        sd = slice_values(stop_dist, start, end)
        acc = slice_values(follower_acc, start, end)
        vx = slice_values(odom_vx, start, end)

        cleared = False
        clear_reason = "never cleared while moving (reached standstill)"
        # Walk the episode time-ordered; check if the entry condition is unsatisfied while moving.
        for time_sec, state_value in zip(control_state.time_sec, control_state.values):
            if not (start <= time_sec <= end):
                continue
            sd_now = stop_dist.nearest(time_sec)
            nv_now = nearest_vel.nearest(time_sec)
            vx_now = odom_vx.nearest(time_sec)
            if vx_now is None or vx_now < moving_vx:
                continue  # only meaningful while still moving
            cond_stop = sd_now is not None and sd_now < -overshoot_stop_dist
            cond_vel = nv_now is not None and nv_now < nearest_vel_eps
            if not (cond_stop and cond_vel):
                cleared = True
                if not cond_stop and not cond_vel:
                    clear_reason = "stop_dist recovered and target_vel>eps while moving"
                elif not cond_stop:
                    clear_reason = (
                        "stop_dist recovered (>= -%.2f) while moving" % overshoot_stop_dist
                    )
                else:
                    clear_reason = "target_vel >= eps while moving"
                break

        episodes.append(
            EmergencyEpisode(
                start_sec=start,
                end_sec=end,
                min_stop_dist=min(sd) if sd else None,
                max_stop_dist=max(sd) if sd else None,
                min_follower_acc=min(acc) if acc else None,
                odom_vx_at_start=odom_vx.nearest(start),
                odom_vx_min=min(vx) if vx else None,
                cleared_while_moving=cleared,
                clear_reason=clear_reason,
            )
        )
    return episodes


def resolve_trigger_topic(topics: Dict[str, list]) -> Optional[str]:
    for candidate in (TRIGGER_TOPIC_D1, TRIGGER_TOPIC_D3):
        if candidate in topics:
            return candidate
    return None


def resolve_bag_uri(path: Path) -> Path:
    """Return rosbag2 directory URI (directory or parent of a single .db3)."""
    path = path.expanduser().resolve()
    if path.is_dir():
        return path
    if path.is_file() and path.suffix == ".db3":
        parent = path.parent
        if (parent / "metadata.yaml").is_file():
            return parent
        return path
    raise ValueError(f"Not a rosbag2 directory or .db3 file: {path}")


def detect_storage_id(rosbag_path: Path) -> str:
    metadata_path = rosbag_path / "metadata.yaml"
    if not metadata_path.is_file():
        return "sqlite3"

    text = metadata_path.read_text(encoding="utf-8")
    match = re.search(r"storage_id:\s*['\"]?(\w+)['\"]?", text)
    if match:
        return match.group(1)
    return "sqlite3"


def bag_time_to_sec(timestamp_ns: int) -> float:
    return timestamp_ns * 1e-9


def get_topics(
    rosbag_path: Path,
    topic_names: Sequence[str],
    storage_id: str,
    *,
    quiet_missing: bool = False,
) -> Tuple[Dict[str, list], List[str]]:
    storage_options = rosbag2_py.StorageOptions(uri=str(rosbag_path), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    available = {info.name for info in reader.get_all_topics_and_types()}
    missing = [name for name in topic_names if name not in available]
    if missing and not quiet_missing:
        print("WARNING: topics not found in bag (skipped):")
        for name in missing:
            print(f"  - {name}")

    active_topics = [name for name in topic_names if name in available]
    if not active_topics:
        print("ERROR: none of the requested topics exist in the bag.")
        print("Available topics:")
        for name in sorted(available):
            print(f"  {name}")
        sys.exit(1)

    topic_filter = rosbag2_py.StorageFilter(topics=active_topics)
    reader.set_filter(topic_filter)

    topic_types = reader.get_all_topics_and_types()
    type_map = {info.name: info.type for info in topic_types}

    topics: Dict[str, list] = {name: [] for name in active_topics}

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        topics[topic].append((bag_time_to_sec(timestamp_ns), msg))

    return topics, missing


def resolve_trajectory_topic(topics: Dict[str, list]) -> Optional[str]:
    for candidate in (TRAJECTORY_TOPIC_D1, TRAJECTORY_TOPIC_D3):
        if candidate in topics:
            return candidate
    return None


def resolve_odom_topic(topics: Dict[str, list]) -> Optional[str]:
    for candidate in (ODOM_TOPIC_D1, ODOM_TOPIC_D1_MAIN, ODOM_TOPIC_D3):
        if candidate in topics:
            return candidate
    return None


def resolve_planner_status_topic(topics: Dict[str, list]) -> Optional[str]:
    for candidate in (PLANNER_STATUS_TOPIC_D1, PLANNER_STATUS_TOPIC_D3):
        if candidate in topics:
            return candidate
    return None


def reason_code_name(code: int) -> str:
    return REASON_CODE_NAMES.get(int(code), f"code_{int(code)}")


def parse_planner_status(data: Sequence[float], time_sec: float) -> Optional[PlannerStatusSample]:
    if len(data) < PLANNER_STATUS_MIN_FIELDS:
        return None
    reason_code = int(round(float(data[PLANNER_STATUS_REASON_CODE])))
    return PlannerStatusSample(
        time_sec=time_sec,
        reason_code=reason_code,
        reason_name=reason_code_name(reason_code),
        trigger_active=float(data[PLANNER_STATUS_TRIGGER_ACTIVE]) >= 0.5,
        is_latched=float(data[PLANNER_STATUS_IS_LATCHED]) >= 0.5,
        has_latest_candidate=float(data[PLANNER_STATUS_HAS_LATEST_CANDIDATE]) >= 0.5,
        data_ready=float(data[PLANNER_STATUS_DATA_READY]) >= 0.5,
        plan_ok=float(data[PLANNER_STATUS_PLAN_OK]) >= 0.5,
        validation_ok=float(data[PLANNER_STATUS_VALIDATION_OK]) >= 0.5,
        planned_points=int(round(float(data[PLANNER_STATUS_PLANNED_POINTS]))),
        published_points=int(round(float(data[PLANNER_STATUS_PUBLISHED_POINTS]))),
        cycle_time_ms=float(data[PLANNER_STATUS_CYCLE_TIME_MS]),
        odom_vx=float(data[PLANNER_STATUS_ODOM_VX]),
    )


def nearest_planner_status(
    samples: Sequence[PlannerStatusSample], query_time_sec: float
) -> Optional[PlannerStatusSample]:
    if not samples:
        return None
    times = [sample.time_sec for sample in samples]
    index = bisect.bisect_left(times, query_time_sec)
    if index == 0:
        return samples[0]
    if index >= len(samples):
        return samples[-1]
    before = samples[index - 1]
    after = samples[index]
    if abs(query_time_sec - before.time_sec) <= abs(after.time_sec - query_time_sec):
        return before
    return after


def summarize_trajectory(msg: Trajectory, time_sec: float) -> TrajectorySample:
    if not msg.points:
        return TrajectorySample(
            time_sec=time_sec,
            num_points=0,
            v0=0.0,
            v1=0.0,
            v_min=0.0,
            v_max=0.0,
            first_zero_idx=None,
            all_zero=True,
            early_zero=False,
        )

    velocities = [float(point.longitudinal_velocity_mps) for point in msg.points]
    first_zero_idx = next(
        (index for index, velocity in enumerate(velocities) if velocity <= VELOCITY_EPS),
        None,
    )
    v1 = velocities[1] if len(velocities) > 1 else velocities[0]
    all_zero = all(velocity <= VELOCITY_EPS for velocity in velocities)
    early_zero = not all_zero and velocities[0] <= VELOCITY_EPS and max(velocities) > VELOCITY_EPS
    return TrajectorySample(
        time_sec=time_sec,
        num_points=len(velocities),
        v0=velocities[0],
        v1=v1,
        v_min=min(velocities),
        v_max=max(velocities),
        first_zero_idx=first_zero_idx,
        all_zero=all_zero,
        early_zero=early_zero,
    )


def trajectory_velocities_csv_rows(msg: Trajectory) -> List[Tuple[int, float, float]]:
    rows: List[Tuple[int, float, float]] = []
    arc_length = 0.0
    for index, point in enumerate(msg.points):
        if index > 0:
            previous = msg.points[index - 1]
            dx = float(point.pose.position.x - previous.pose.position.x)
            dy = float(point.pose.position.y - previous.pose.position.y)
            arc_length += (dx * dx + dy * dy) ** 0.5
        rows.append((index, arc_length, float(point.longitudinal_velocity_mps)))
    return rows


def classify_sample(
    sample: TrajectorySample,
    *,
    moving_odom_threshold: float,
    stopped_odom_threshold: float,
    negative_stop_dist_threshold: float,
) -> List[str]:
    tags: List[str] = []
    odom = sample.odom_vx
    if odom is None:
        return tags

    if odom >= moving_odom_threshold and sample.all_zero:
        tags.append(ANOMALY_MOVING_ALL_ZERO)
    if odom >= moving_odom_threshold and sample.early_zero:
        tags.append(ANOMALY_MOVING_EARLY_ZERO)
    if (
        odom >= moving_odom_threshold
        and sample.stop_dist is not None
        and sample.stop_dist < negative_stop_dist_threshold
    ):
        tags.append(ANOMALY_MOVING_NEGATIVE_STOP_DIST)
    if odom <= stopped_odom_threshold and not sample.all_zero and sample.v_max > VELOCITY_EPS:
        tags.append(ANOMALY_STOPPED_NONZERO_TRAJ)
    return tags


def classify_planner_sample(
    sample: PlannerStatusSample,
    *,
    moving_odom_threshold: float,
) -> List[str]:
    tags: List[str] = []
    if sample.odom_vx >= moving_odom_threshold and not sample.trajectory_published:
        tags.append(ANOMALY_PLANNER_MOVING_NO_PUBLISH)
    return tags


def write_planner_status_csv(path: Path, samples: Sequence[PlannerStatusSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "time_sec",
                "reason_code",
                "reason_name",
                "trigger_active",
                "is_latched",
                "has_latest_candidate",
                "data_ready",
                "plan_ok",
                "validation_ok",
                "planned_points",
                "published_points",
                "cycle_time_ms",
                "odom_vx",
                "trajectory_published",
            ]
        )
        for sample in samples:
            writer.writerow(
                [
                    f"{sample.time_sec:.6f}",
                    sample.reason_code,
                    sample.reason_name,
                    int(sample.trigger_active),
                    int(sample.is_latched),
                    int(sample.has_latest_candidate),
                    int(sample.data_ready),
                    int(sample.plan_ok),
                    int(sample.validation_ok),
                    sample.planned_points,
                    sample.published_points,
                    f"{sample.cycle_time_ms:.3f}",
                    f"{sample.odom_vx:.6f}",
                    int(sample.trajectory_published),
                ]
            )


def write_timeline_csv(path: Path, samples: Sequence[TrajectorySample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "time_sec",
                "num_points",
                "v0",
                "v1",
                "v_min",
                "v_max",
                "first_zero_idx",
                "all_zero",
                "early_zero",
                "odom_vx",
                "stop_dist",
                "control_state",
                "control_state_label",
                "gate_acc",
                "follower_acc",
                "planner_reason_code",
                "planner_reason_name",
                "planner_published_points",
                "planner_is_latched",
                "anomaly_tags",
            ]
        )
        for sample in samples:
            state_label = ""
            if sample.control_state is not None:
                state_label = CONTROL_STATE_LABELS.get(
                    int(round(sample.control_state)), str(int(round(sample.control_state)))
                )
            writer.writerow(
                [
                    f"{sample.time_sec:.6f}",
                    sample.num_points,
                    f"{sample.v0:.6f}",
                    f"{sample.v1:.6f}",
                    f"{sample.v_min:.6f}",
                    f"{sample.v_max:.6f}",
                    "" if sample.first_zero_idx is None else sample.first_zero_idx,
                    int(sample.all_zero),
                    int(sample.early_zero),
                    "" if sample.odom_vx is None else f"{sample.odom_vx:.6f}",
                    "" if sample.stop_dist is None else f"{sample.stop_dist:.6f}",
                    "" if sample.control_state is None else int(round(sample.control_state)),
                    state_label,
                    "" if sample.gate_acc is None else f"{sample.gate_acc:.6f}",
                    "" if sample.follower_acc is None else f"{sample.follower_acc:.6f}",
                    "" if sample.planner_reason_code is None else sample.planner_reason_code,
                    sample.planner_reason_name or "",
                    (
                        ""
                        if sample.planner_published_points is None
                        else sample.planner_published_points
                    ),
                    "" if sample.planner_is_latched is None else int(sample.planner_is_latched),
                    "|".join(sample.anomaly_tags),
                ]
            )


def write_anomalies_csv(path: Path, events: Sequence[AnomalyEvent]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "tag",
                "time_sec",
                "odom_vx",
                "v0",
                "v_max",
                "stop_dist",
                "control_state",
                "control_state_label",
                "all_zero",
                "early_zero",
            ],
        )
        writer.writeheader()
        for event in events:
            state_label = ""
            if event.control_state is not None:
                state_label = CONTROL_STATE_LABELS.get(
                    event.control_state, str(event.control_state)
                )
            writer.writerow(
                {
                    "tag": event.tag,
                    "time_sec": f"{event.time_sec:.6f}",
                    "odom_vx": "" if event.odom_vx is None else f"{event.odom_vx:.6f}",
                    "v0": f"{event.v0:.6f}",
                    "v_max": f"{event.v_max:.6f}",
                    "stop_dist": "" if event.stop_dist is None else f"{event.stop_dist:.6f}",
                    "control_state": "" if event.control_state is None else event.control_state,
                    "control_state_label": state_label,
                    "all_zero": int(event.all_zero),
                    "early_zero": int(event.early_zero),
                }
            )


def export_trajectory_profile(
    export_dir: Path,
    time_sec: float,
    msg: Trajectory,
    sample: TrajectorySample,
) -> Path:
    export_dir.mkdir(parents=True, exist_ok=True)
    stamp_label = f"{time_sec:.3f}".replace(".", "p")
    csv_path = export_dir / f"trajectory_{stamp_label}s.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["index", "accumulated_distance_m", "longitudinal_velocity_mps"])
        for index, distance, velocity in trajectory_velocities_csv_rows(msg):
            writer.writerow([index, f"{distance:.6f}", f"{velocity:.6f}"])

    meta_path = export_dir / f"trajectory_{stamp_label}s_meta.json"
    meta_path.write_text(
        json.dumps(asdict(sample), indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return csv_path


def record_rosbag(output_path: Path, topics: Sequence[str]) -> subprocess.Popen:
    command = ["ros2", "bag", "record", "-o", str(output_path), *topics]
    return subprocess.Popen(command, stdin=subprocess.PIPE)


def validate_rosbag(output_path: Path, required_topics: Sequence[str]) -> bool:
    try:
        result = subprocess.run(
            ["ros2", "bag", "info", str(output_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
        )
    except subprocess.CalledProcessError as error:
        print(f"ERROR: ros2 bag info failed: {error.stderr.strip()}")
        return False

    output = result.stdout
    missing: List[str] = []
    for topic_name in required_topics:
        pattern = rf"Topic: {re.escape(topic_name)}\s+\|.+?\| Count: (\d+)"
        match = re.search(pattern, output)
        if not match or int(match.group(1)) == 0:
            missing.append(topic_name)

    if missing:
        print("Recorded bag is missing messages on:")
        for topic_name in missing:
            print(f"  - {topic_name}")
        return False
    return True


def format_topic_help() -> str:
    return "\n".join(f"  - {topic}" for topic in DOMAIN1_RECORD_TOPICS)
