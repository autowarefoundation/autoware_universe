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

"""Analyze Candidate Bank events appended to DiffusionPlanner internal_events.jsonl.

The parser is streaming with respect to JSONL input and caches each candidate geometry only once.
It writes CSV summaries plus optional cycle PNGs under analysis_candidate_bank/.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import time
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

# CANDIDATE_BANK_ANALYZER_PROGRESS_V1_BEGIN
_PROGRESS_LOG_PATH: Optional[Path] = None


def _format_elapsed(seconds: float) -> str:
    seconds = max(0.0, float(seconds))
    if seconds < 60.0:
        return f"{seconds:.1f}s"
    minutes, sec = divmod(int(seconds + 0.5), 60)
    if minutes < 60:
        return f"{minutes}m{sec:02d}s"
    hours, minutes = divmod(minutes, 60)
    return f"{hours}h{minutes:02d}m{sec:02d}s"


def _progress(message: str) -> None:
    line = f"[candidate-bank-analyzer] {message}"
    print(line, flush=True)
    if _PROGRESS_LOG_PATH is not None:
        try:
            with _PROGRESS_LOG_PATH.open("a", encoding="utf-8") as f:
                f.write(line + "\n")
        except OSError:
            # Progress logging must never abort the analysis.
            pass


def _progress_due(last_report: float, interval_sec: float) -> bool:
    return interval_sec > 0.0 and time.monotonic() - last_report >= interval_sec


def _current_rss_mib() -> Optional[float]:
    """Return current resident memory on Linux without adding a dependency."""
    try:
        pages = int(Path("/proc/self/statm").read_text(encoding="utf-8").split()[1])
        return pages * os.sysconf("SC_PAGE_SIZE") / (1024.0 * 1024.0)
    except (OSError, ValueError, IndexError):
        return None


def _rss_text() -> str:
    rss = _current_rss_mib()
    return f" | RSS={rss:.1f} MiB" if rss is not None else ""


# CANDIDATE_BANK_ANALYZER_PROGRESS_V1_END


def finite(v) -> bool:
    return v is not None and isinstance(v, (int, float)) and math.isfinite(float(v))


def _int_or_default(value, default: int = -1) -> int:
    """Convert an optional integer while preserving the valid value 0.

    Using ``int(value or -1)`` incorrectly maps predicted-path index 0 to -1.
    """
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def transform_xy(x: float, y: float, ego: dict) -> Tuple[float, float]:
    dx = x - float(ego.get("x", 0.0))
    dy = y - float(ego.get("y", 0.0))
    yaw = float(ego.get("yaw", 0.0))
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def write_csv(path: Path, rows: List[dict]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields, seen = [], set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


# CANDIDATE_BANK_ANALYZER_LOW_MEMORY_V1_BEGIN
def write_grouped_csv(path: Path, grouped_rows: Dict[int, List[dict]]) -> int:
    """Write already-cached grouped rows without materializing a duplicate flat list."""
    fields, seen = [], set()
    row_count = 0
    for rows in grouped_rows.values():
        for row in rows:
            row_count += 1
            for key in row:
                if key not in seen:
                    seen.add(key)
                    fields.append(key)
    if row_count == 0:
        path.write_text("", encoding="utf-8")
        return 0
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for rows in grouped_rows.values():
            writer.writerows(rows)
    return row_count


# CANDIDATE_BANK_ANALYZER_LOW_MEMORY_V1_END


def flatten_evaluation(event: dict) -> dict:
    row = {k: v for k, v in event.items() if k not in {"score_terms", "thread_id", "event"}}
    for key, value in (event.get("score_terms") or {}).items():
        row[f"score_{key}"] = value
    return row


def draw_cycle(
    cycle_id: int,
    observation: dict,
    evaluations: List[dict],
    selection: Optional[dict],
    geometries: Dict[int, List[list]],
    newest_by_cycle: Dict[int, int],
    out_path: Path,
    backward_m: float,
    forward_m: float,
    lateral_m: float,
    show_labels: bool,
) -> None:
    from matplotlib.patches import Circle
    from matplotlib.patches import Polygon as MplPolygon
    import matplotlib.pyplot as plt

    ego = observation.get("ego") or {"x": 0.0, "y": 0.0, "yaw": 0.0}
    eval_by_id = {int(e["candidate_id"]): e for e in evaluations}
    best_id = int(selection.get("best_id", 0)) if selection else 0
    selected_id = int(selection.get("selected_id", 0)) if selection else 0
    newest_id = int(newest_by_cycle.get(cycle_id, 0))

    fig, ax = plt.subplots(figsize=(11, 7))

    for cid, ev in sorted(eval_by_id.items()):
        points = geometries.get(cid, [])
        start = int(ev.get("live_start_index", 0) or 0)
        points = points[start:]
        if len(points) < 2:
            continue
        xy = [transform_xy(float(pt[0]), float(pt[1]), ego) for pt in points]
        xs = [pt[0] for pt in xy]
        ys = [pt[1] for pt in xy]
        safe = bool(ev.get("safe", False))
        color = "tab:blue" if safe else "tab:red"
        alpha = 0.28 if safe else 0.55
        width = 1.0
        style = "-" if safe else "--"
        if cid == newest_id:
            color, alpha, width = "tab:orange", 0.9, 1.8
        if cid == best_id:
            color, alpha, width = "tab:cyan", 0.95, 2.5
        if cid == selected_id:
            color, alpha, width = "tab:green", 1.0, 3.5
        ax.plot(xs, ys, linestyle=style, linewidth=width, alpha=alpha, color=color)
        # CANDIDATE_BANK_CURRENT_LANE_RULE_V1_BEGIN
        if bool(ev.get("terminal_straddling", False)):
            ax.scatter([xs[-1]], [ys[-1]], marker="X", s=70, color="tab:pink", zorder=45)
        # CANDIDATE_BANK_CURRENT_LANE_RULE_V1_END
        if show_labels:
            # CANDIDATE_BANK_CURRENT_LANE_RULE_V1_BEGIN
            lane_cost = ev.get("lane_boundary_cost")
            suffix = f" LB={lane_cost}" if finite(lane_cost) and float(lane_cost) > 0.0 else ""
            ax.text(xs[-1], ys[-1], f"#{cid}{suffix}", fontsize=8)
            # CANDIDATE_BANK_CURRENT_LANE_RULE_V1_END

    # CANDIDATE_BANK_COLLISION_GEOMETRY_V1_BEGIN
    ego_collision_polygon = ego.get("collision_polygon") or []
    if len(ego_collision_polygon) >= 3:
        ego_poly_xy = [
            transform_xy(float(pt[0]), float(pt[1]), ego) for pt in ego_collision_polygon
        ]
        ax.add_patch(
            MplPolygon(
                ego_poly_xy, closed=True, fill=False, color="black", linewidth=1.5, alpha=0.8
            )
        )

    for obj in observation.get("objects", []):
        x, y = transform_xy(float(obj.get("x", 0.0)), float(obj.get("y", 0.0)), ego)
        is_static = bool(obj.get("is_static", False))
        marker = "s" if is_static else "o"
        color = "black" if is_static else "tab:purple"
        ax.scatter([x], [y], marker=marker, s=45, color=color, alpha=0.8)
        # CANDIDATE_BANK_THREE_CIRCLE_FOOTPRINT_V1_BEGIN
        # New logs contain the exact component circles used by Candidate Bank.
        # Old logs are still supported via the legacy single `radius_m` field.
        collision_polygon = obj.get("collision_polygon") or []
        if len(collision_polygon) >= 3:
            polygon_xy = [transform_xy(float(pt[0]), float(pt[1]), ego) for pt in collision_polygon]
            ax.add_patch(
                MplPolygon(
                    polygon_xy, closed=True, fill=False, color=color, linewidth=1.5, alpha=0.55
                )
            )
        else:
            collision_circles = obj.get("collision_circles") or []
            if collision_circles:
                for circle in collision_circles:
                    radius = circle.get("radius_m")
                    if not finite(radius):
                        continue
                    cx, cy = transform_xy(
                        float(circle.get("x", obj.get("x", 0.0))),
                        float(circle.get("y", obj.get("y", 0.0))),
                        ego,
                    )
                    ax.add_patch(
                        Circle((cx, cy), float(radius), fill=False, color=color, alpha=0.35)
                    )
            else:
                radius = obj.get("radius_m")
                if finite(radius):
                    ax.add_patch(Circle((x, y), float(radius), fill=False, color=color, alpha=0.25))
        # CANDIDATE_BANK_THREE_CIRCLE_FOOTPRINT_V1_END
        # CANDIDATE_BANK_COLLISION_GEOMETRY_V1_END

    ax.scatter([0.0], [0.0], marker="^", s=100, color="black")
    ax.set_xlim(-backward_m, forward_m)
    ax.set_ylim(-lateral_m, lateral_m)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("ego-forward [m]")
    ax.set_ylabel("ego-left [m]")

    safe_count = sum(bool(e.get("safe", False)) for e in evaluations)
    reason = selection.get("reason", "none") if selection else "none"
    switched = selection.get("switched", False) if selection else False
    hysteresis = selection.get("hysteresis_kept", False) if selection else False
    geometry_mode = observation.get("collision_geometry_mode", "legacy")
    ax.set_title(
        f"cycle {cycle_id} | geometry={geometry_mode} | bank={len(evaluations)} safe={safe_count} | "
        f"newest=#{newest_id} best=#{best_id} selected=#{selected_id} | "
        f"switch={switched} hysteresis={hysteresis} reason={reason}"
    )

    selected_eval = eval_by_id.get(selected_id)
    if selected_eval:
        terms = selected_eval.get("score_terms") or {}
        detail = (
            f"selected score={selected_eval.get('score')}  source={selected_eval.get('source')}\n"
            f"progress={selected_eval.get('progress_m')}m  route_dev={selected_eval.get('route_deviation_rms_m')}m  "
            f"static_clearance={selected_eval.get('min_static_clearance_m')}m  "
            f"dynamic_clearance={selected_eval.get('min_dynamic_clearance_m')}m  "
            f"time_gap={selected_eval.get('min_dynamic_time_gap_sec')}s  "
            f"ttc={selected_eval.get('min_dynamic_ttc_sec')}s\n"
            f"temporal_mode={selected_eval.get('dynamic_temporal_cost_mode', 'ttc')}  "
            f"clearance_cost={selected_eval.get('dynamic_clearance_soft_cost')}  "
            f"legacy_gap_cost={selected_eval.get('dynamic_legacy_time_gap_soft_cost')}  "
            f"ttc_cost={selected_eval.get('dynamic_ttc_soft_cost')}\n"
            f"soft_aggregation={selected_eval.get('dynamic_soft_cost_aggregation_mode', 'legacy_sum')}  "
            f"uncertainty_mode={selected_eval.get('dynamic_uncertainty_mode', 'legacy_absolute_speed')}  "
            f"closing_rate_max={selected_eval.get('max_dynamic_closing_rate_mps')}m/s  "
            f"uncertainty_max={selected_eval.get('max_dynamic_uncertainty_margin_m')}m\n"
            f"pred_paths={selected_eval.get('dynamic_prediction_path_count')}  "
            f"pred_conf_sum={selected_eval.get('dynamic_prediction_confidence_sum')}\n"
            f"static_margin_violation={selected_eval.get('static_collision_margin_violated')}  "
            f"dynamic_margin_violation={selected_eval.get('dynamic_collision_margin_violated')}\n"
            f"route_ref_weight_mean={selected_eval.get('route_reference_weight_mean')}  "
            f"route_ref_roughness_max={selected_eval.get('route_reference_max_roughness_m')}m  "
            f"signal_active={selected_eval.get('traffic_signal_stop_active')}  "
            f"signal_crossed={selected_eval.get('traffic_signal_crossed')}  "
            f"traffic_cost={selected_eval.get('traffic_rule_cost')}\n"
            # CANDIDATE_BANK_CURRENT_LANE_RULE_V1_BEGIN
            f"signal_scope={selected_eval.get('traffic_signal_scope_mode', 'route_nearest')}  "
            f"current_lane={selected_eval.get('current_lane_id')} valid={selected_eval.get('current_lane_valid')}  "
            f"max_straddle={selected_eval.get('max_continuous_straddling_sec')}s  "
            f"terminal_straddle={selected_eval.get('terminal_straddling')}  "
            f"lane_boundary_cost={selected_eval.get('lane_boundary_cost')}\n"
            # CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_BEGIN
            f"lc_deadline_active={selected_eval.get('lane_change_deadline_active')}  "
            f"lc_complete_before={selected_eval.get('lane_change_completed_before_deadline')}  "
            f"lc_fallback_stop={selected_eval.get('lane_change_fallback_stop_satisfied')}\n"
            f"lc_target_lane={selected_eval.get('lane_change_target_lane_id')}  "
            f"lc_slack={selected_eval.get('lane_change_slack_m')}m  "
            f"lc_cost={selected_eval.get('lane_change_deadline_cost')}\n"
            # CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_END
            # CANDIDATE_BANK_CURRENT_LANE_RULE_V1_END
            f"score terms: progress={terms.get('progress')} route={terms.get('route_deviation')} "
            f"eff={terms.get('path_efficiency')} static={terms.get('static_clearance')} "
            f"dynamic={terms.get('dynamic_margin')} comfort={terms.get('comfort')} "
            f"traffic={terms.get('traffic_rules')} lane_boundary={terms.get('lane_boundary')} "
            f"lane_change_deadline={terms.get('lane_change_deadline')}"
        )
        ax.text(
            0.01,
            0.01,
            detail,
            transform=ax.transAxes,
            va="bottom",
            ha="left",
            fontsize=8,
            bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "0.7"},
        )

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


# CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_BEGIN
_JOINT_PALETTE = [
    (0.20, 0.70, 1.00),
    (1.00, 0.45, 0.20),
    (0.35, 0.90, 0.35),
    (0.85, 0.35, 0.95),
    (1.00, 0.80, 0.20),
    (0.20, 0.90, 0.85),
    (0.95, 0.35, 0.55),
    (0.65, 0.70, 1.00),
]


def _uuid_bytes(object_id: str) -> bytes:
    compact = "".join(ch for ch in str(object_id).lower() if ch in "0123456789abcdef")
    if len(compact) >= 2 and len(compact) % 2 == 0:
        try:
            return bytes.fromhex(compact)
        except ValueError:
            pass
    return str(object_id).encode("utf-8", errors="replace")


def joint_prediction_base_color(object_id: str) -> Tuple[float, float, float]:
    # Match the FNV-1a UUID palette mapping used by the RViz publisher.
    h = 2166136261
    for byte in _uuid_bytes(object_id):
        h ^= int(byte)
        h = (h * 16777619) & 0xFFFFFFFF
    return _JOINT_PALETTE[h % len(_JOINT_PALETTE)]


# CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_BEGIN
def flatten_cross_collision(event: dict) -> dict:
    row = {k: v for k, v in event.items() if k not in {"thread_id", "event", "stats"}}
    for key, value in (event.get("stats") or {}).items():
        row[f"stats_{key}"] = value
    return row


# CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_BEGIN
def flatten_cross_aggregate(event: dict) -> dict:
    return {k: v for k, v in event.items() if k not in {"thread_id", "event"}}


# CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_END
# CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_END


# CANDIDATE_BANK_CROSS_COLLISION_VISUALIZATION_V1_BEGIN
def _candidate_live_xy_at_time(
    geometry: List[list], live_start_index: int, t_sec: float
) -> Optional[Tuple[float, float]]:
    """Interpolate candidate base-link XY at current-relative time.

    Candidate geometry stores [x, y, velocity, original_time_from_start].  The
    cross-collision checker rebases time at live_start_index, so this helper
    mirrors that convention.
    """
    if not geometry or live_start_index < 0 or live_start_index >= len(geometry):
        return None
    t0 = float(geometry[live_start_index][3]) if len(geometry[live_start_index]) > 3 else 0.0
    timed = []
    last_t = -float("inf")
    for pt in geometry[live_start_index:]:
        if len(pt) < 2:
            continue
        original_t = float(pt[3]) if len(pt) > 3 else 0.0
        t = max(0.0, original_t - t0)
        if t <= last_t + 1.0e-9:
            continue
        timed.append((t, float(pt[0]), float(pt[1])))
        last_t = t
    if not timed:
        return None
    if t_sec <= timed[0][0]:
        return timed[0][1], timed[0][2]
    if t_sec >= timed[-1][0]:
        return timed[-1][1], timed[-1][2]
    for a, b in zip(timed[:-1], timed[1:]):
        if a[0] - 1.0e-9 <= t_sec <= b[0] + 1.0e-9:
            dt = b[0] - a[0]
            u = 0.0 if dt <= 1.0e-9 else max(0.0, min(1.0, (t_sec - a[0]) / dt))
            return a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2])
    return None


# CANDIDATE_BANK_CROSS_COLLISION_FOOTPRINT_VISUALIZATION_V1_BEGIN
def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _interpolate_angle(a0: float, a1: float, u: float) -> float:
    return _normalize_angle(a0 + max(0.0, min(1.0, u)) * _normalize_angle(a1 - a0))


def _candidate_live_pose_at_time(
    geometry: List[list], live_start_index: int, t_sec: float
) -> Optional[Tuple[float, float, float]]:
    """Interpolate candidate XY and estimate yaw from the local path tangent.

    Candidate geometry currently logs XYVT but not yaw.  The cross-collision
    kernel itself uses the trajectory poses from Autoware; for visualization we
    reconstruct heading from the segment tangent.  This affects only the drawn
    footprint outline, never collision classification.
    """
    if not geometry or live_start_index < 0 or live_start_index >= len(geometry):
        return None
    t0 = float(geometry[live_start_index][3]) if len(geometry[live_start_index]) > 3 else 0.0
    timed = []
    last_t = -float("inf")
    for pt in geometry[live_start_index:]:
        if len(pt) < 2:
            continue
        original_t = float(pt[3]) if len(pt) > 3 else 0.0
        t = max(0.0, original_t - t0)
        if t <= last_t + 1.0e-9:
            continue
        timed.append((t, float(pt[0]), float(pt[1])))
        last_t = t
    if not timed:
        return None
    if len(timed) == 1:
        return timed[0][1], timed[0][2], 0.0

    if t_sec <= timed[0][0]:
        i0, i1, u = 0, 1, 0.0
    elif t_sec >= timed[-1][0]:
        i0, i1, u = len(timed) - 2, len(timed) - 1, 1.0
    else:
        i0, i1, u = 0, 1, 0.0
        for idx, (a, b) in enumerate(zip(timed[:-1], timed[1:])):
            if a[0] - 1.0e-9 <= t_sec <= b[0] + 1.0e-9:
                i0, i1 = idx, idx + 1
                dt = b[0] - a[0]
                u = 0.0 if dt <= 1.0e-9 else max(0.0, min(1.0, (t_sec - a[0]) / dt))
                break
    a, b = timed[i0], timed[i1]
    x = a[1] + u * (b[1] - a[1])
    y = a[2] + u * (b[2] - a[2])
    dx, dy = b[1] - a[1], b[2] - a[2]
    if math.hypot(dx, dy) <= 1.0e-6:
        # Search nearby non-degenerate segments before falling back to zero yaw.
        yaw = 0.0
        for j in range(max(0, i0 - 2), min(len(timed) - 1, i1 + 2)):
            q0, q1 = timed[j], timed[j + 1]
            ddx, ddy = q1[1] - q0[1], q1[2] - q0[2]
            if math.hypot(ddx, ddy) > 1.0e-6:
                yaw = math.atan2(ddy, ddx)
                break
    else:
        yaw = math.atan2(dy, dx)
    return x, y, yaw


def _scenario_pose_at_time(
    joint: dict,
    object_id: str,
    path_index: int,
    scenario_age_sec: float,
    current_relative_t_sec: float,
) -> Optional[Tuple[float, float, float]]:
    match = _scenario_object_path(joint, object_id, path_index)
    if match is None:
        return None
    raw = match["path"].get("path_xyyaw") or []
    dt = float(match["path"].get("time_step_sec", 0.0) or 0.0)
    if len(raw) < 1 or dt <= 1.0e-9:
        return None
    original_t = max(0.0, float(scenario_age_sec)) + max(0.0, float(current_relative_t_sec))
    max_t = dt * float(len(raw) - 1)
    if len(raw) == 1 or original_t >= max_t:
        yaw = float(raw[-1][2]) if len(raw[-1]) > 2 else 0.0
        return float(raw[-1][0]), float(raw[-1][1]), yaw
    index_f = original_t / dt
    i0 = max(0, min(len(raw) - 2, int(math.floor(index_f))))
    i1 = i0 + 1
    u = max(0.0, min(1.0, index_f - float(i0)))
    yaw0 = (
        float(raw[i0][2])
        if len(raw[i0]) > 2
        else math.atan2(
            float(raw[i1][1]) - float(raw[i0][1]), float(raw[i1][0]) - float(raw[i0][0])
        )
    )
    yaw1 = float(raw[i1][2]) if len(raw[i1]) > 2 else yaw0
    return (
        float(raw[i0][0]) + u * (float(raw[i1][0]) - float(raw[i0][0])),
        float(raw[i0][1]) + u * (float(raw[i1][1]) - float(raw[i0][1])),
        _interpolate_angle(yaw0, yaw1, u),
    )


def _world_polygon_to_pose_local(
    polygon: List, pose_x: float, pose_y: float, pose_yaw: float
) -> List[Tuple[float, float]]:
    c, sn = math.cos(pose_yaw), math.sin(pose_yaw)
    out = []
    for pt in polygon:
        if len(pt) < 2:
            continue
        dx, dy = float(pt[0]) - pose_x, float(pt[1]) - pose_y
        out.append((c * dx + sn * dy, -sn * dx + c * dy))
    return out


def _place_local_polygon(
    local_polygon: List[Tuple[float, float]], pose: Tuple[float, float, float]
) -> List[Tuple[float, float]]:
    x, y, yaw = pose
    c, sn = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - sn * py, y + sn * px + c * py) for px, py in local_polygon]


def _rectangle_local_polygon(length_m, width_m) -> List[Tuple[float, float]]:
    if not finite(length_m) or not finite(width_m):
        return []
    length_m, width_m = float(length_m), float(width_m)
    if length_m <= 0.0 or width_m <= 0.0:
        return []
    hl, hw = 0.5 * length_m, 0.5 * width_m
    return [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]


def _find_observation_object_for_cross(observation: dict, object_id: str) -> Optional[dict]:
    for obj in observation.get("objects") or []:
        if str(obj.get("object_id", "")) == str(object_id):
            return obj
    return None


def _ego_local_footprint(observation: dict) -> List[Tuple[float, float]]:
    ego = observation.get("ego") or {}
    polygon = ego.get("collision_polygon") or []
    if len(polygon) < 3:
        return []
    return _world_polygon_to_pose_local(
        polygon, float(ego.get("x", 0.0)), float(ego.get("y", 0.0)), float(ego.get("yaw", 0.0))
    )


def _object_local_footprint(
    observation: dict, scenario: dict, object_id: str
) -> List[Tuple[float, float]]:
    # Prefer the exact current logged footprint for the same tracked UUID.  Its
    # local geometry can be transported to the historical predicted pose.
    current = _find_observation_object_for_cross(observation, object_id)
    if current is not None:
        polygon = current.get("collision_polygon") or []
        if len(polygon) >= 3:
            return _world_polygon_to_pose_local(
                polygon,
                float(current.get("x", 0.0)),
                float(current.get("y", 0.0)),
                float(current.get("yaw", 0.0)),
            )
    # Scan object metadata directly; a specific predicted-path index is not needed for dimensions.
    for obj in scenario.get("objects") or []:
        if str(obj.get("object_id", "")) == str(object_id):
            return _rectangle_local_polygon(obj.get("length_m"), obj.get("width_m"))
    return []


def _draw_conflict_footprints(
    ax,
    observation: dict,
    scenario: dict,
    ego_pose_w: Tuple[float, float, float],
    object_pose_w: Tuple[float, float, float],
    object_id: str,
    ego_frame: dict,
    color: str,
    linestyle: str,
    alpha: float,
    zorder: int,
) -> None:
    from matplotlib.patches import Polygon as MplPolygon

    ego_local = _ego_local_footprint(observation)
    if ego_local:
        world_poly = _place_local_polygon(ego_local, ego_pose_w)
        local_poly = [transform_xy(x, y, ego_frame) for x, y in world_poly]
        ax.add_patch(
            MplPolygon(
                local_poly,
                closed=True,
                fill=False,
                edgecolor=color,
                linewidth=1.4,
                linestyle=linestyle,
                alpha=alpha,
                zorder=zorder,
            )
        )

    object_local = _object_local_footprint(observation, scenario, object_id)
    if object_local:
        world_poly = _place_local_polygon(object_local, object_pose_w)
        local_poly = [transform_xy(x, y, ego_frame) for x, y in world_poly]
        ax.add_patch(
            MplPolygon(
                local_poly,
                closed=True,
                fill=False,
                edgecolor=color,
                linewidth=1.4,
                linestyle=linestyle,
                alpha=alpha,
                zorder=zorder,
            )
        )


# CANDIDATE_BANK_CROSS_COLLISION_FOOTPRINT_VISUALIZATION_V1_END


def _scenario_object_path(joint: dict, object_id: str, path_index: int) -> Optional[dict]:
    for obj in joint.get("objects") or []:
        if str(obj.get("object_id", "")) != str(object_id):
            continue
        for path in obj.get("paths") or []:
            if _int_or_default(path.get("path_index"), -1) == int(path_index):
                return {"object": obj, "path": path}
    return None


def _scenario_xy_at_time(
    joint: dict,
    object_id: str,
    path_index: int,
    scenario_age_sec: float,
    current_relative_t_sec: float,
) -> Optional[Tuple[float, float]]:
    """Interpolate a historical PredictedPath at the time used by 0013a7/a8.

    prepare_scenario() evaluates the stored raw path at original_time = age +
    current_relative_time.  Reconstructing it this way keeps the offline marker
    aligned with the actual cross-collision evaluation.
    """
    match = _scenario_object_path(joint, object_id, path_index)
    if match is None:
        return None
    path = match["path"]
    raw = path.get("path_xyyaw") or []
    dt = float(path.get("time_step_sec", 0.0) or 0.0)
    if len(raw) < 1 or dt <= 1.0e-9:
        return None
    original_t = max(0.0, float(scenario_age_sec)) + max(0.0, float(current_relative_t_sec))
    max_t = dt * float(len(raw) - 1)
    if original_t >= max_t:
        return float(raw[-1][0]), float(raw[-1][1])
    index_f = original_t / dt
    i0 = max(0, min(len(raw) - 2, int(math.floor(index_f))))
    i1 = i0 + 1
    u = max(0.0, min(1.0, index_f - float(i0)))
    return (
        float(raw[i0][0]) + u * (float(raw[i1][0]) - float(raw[i0][0])),
        float(raw[i0][1]) + u * (float(raw[i1][1]) - float(raw[i0][1])),
    )


def _cell_status_label(cell: dict) -> str:
    if bool(cell.get("same_time_collision", False)):
        toi = cell.get("first_collision_time_sec")
        return f"C {float(toi):.1f}s" if finite(toi) else "C"
    if bool(cell.get("xyt_conflict", False)):
        rho = cell.get("min_xyt_rho")
        return f"X {float(rho):.2f}" if finite(rho) else "X"
    if not bool(cell.get("valid", False)):
        return "-"
    return "."


def _cross_problem_cycle(
    cycle_id: int,
    cross_by_cycle: Dict[int, List[dict]],
    aggregates_by_cycle: Dict[int, List[dict]],
    selections: Dict[int, dict],
    focus: str,
) -> bool:
    cells = cross_by_cycle.get(cycle_id, [])
    if not cells:
        return False
    selection = selections.get(cycle_id) or {}
    selected_id = int(selection.get("selected_id", 0) or 0)
    shadow = next((r for r in aggregates_by_cycle.get(cycle_id, []) if r.get("shadow_best")), None)
    shadow_id = int((shadow or {}).get("candidate_id", 0) or 0)
    focus_ids = set()
    if focus in {"selected", "selected_and_shadow"} and selected_id:
        focus_ids.add(selected_id)
    if focus in {"shadow", "selected_and_shadow"} and shadow_id:
        focus_ids.add(shadow_id)
    for cell in cells:
        eid = int(cell.get("ego_candidate_id", 0) or 0)
        if focus != "all" and focus_ids and eid not in focus_ids:
            continue
        if bool(cell.get("same_time_collision", False)) or bool(cell.get("xyt_conflict", False)):
            return True
    return any(
        bool(r.get("selector_hard_rejected", False)) for r in aggregates_by_cycle.get(cycle_id, [])
    )


# CANDIDATE_BANK_CROSS_COLLISION_TIME_PANEL_V1_BEGIN
def _candidate_time_trace(
    geometry: List[list], live_start_index: int, ego_frame: dict
) -> List[Tuple[float, float, float]]:
    """Return (future_t_sec, ego_forward_m, ego_left_m) for one live candidate."""
    if not geometry or live_start_index < 0 or live_start_index >= len(geometry):
        return []
    first = geometry[live_start_index]
    t0 = float(first[3]) if len(first) > 3 else 0.0
    out: List[Tuple[float, float, float]] = []
    last_t = -float("inf")
    for pt in geometry[live_start_index:]:
        if len(pt) < 2:
            continue
        t = max(0.0, (float(pt[3]) if len(pt) > 3 else 0.0) - t0)
        if t <= last_t + 1.0e-9:
            continue
        forward, left = transform_xy(float(pt[0]), float(pt[1]), ego_frame)
        out.append((t, forward, left))
        last_t = t
    return out


def _scenario_time_trace(
    scenario: dict,
    object_id: str,
    path_index: int,
    scenario_age_sec: float,
    ego_frame: dict,
) -> List[Tuple[float, float, float]]:
    """Return current-relative (t, forward, left) for a stored historical path.

    Stored PredictedPath sample k is at original time k*dt.  Cross evaluation at
    the current cycle uses original_time = scenario_age + current_relative_time,
    so the plot subtracts scenario_age.  A t=0 interpolation point is inserted
    to make the current alignment explicit even when age is between samples.
    """
    match = _scenario_object_path(scenario, object_id, path_index)
    if match is None:
        return []
    raw = match["path"].get("path_xyyaw") or []
    dt = float(match["path"].get("time_step_sec", 0.0) or 0.0)
    age = max(0.0, float(scenario_age_sec))
    if not raw or dt <= 1.0e-9:
        return []
    max_future_t = dt * float(len(raw) - 1) - age
    if max_future_t < -1.0e-9:
        return []

    out: List[Tuple[float, float, float]] = []
    pose0 = _scenario_pose_at_time(scenario, object_id, path_index, age, 0.0)
    if pose0 is not None:
        fwd0, left0 = transform_xy(pose0[0], pose0[1], ego_frame)
        out.append((0.0, fwd0, left0))

    first_after = max(0, int(math.floor(age / dt)) + 1)
    for k in range(first_after, len(raw)):
        t = float(k) * dt - age
        if t <= 1.0e-9:
            continue
        pt = raw[k]
        if len(pt) < 2:
            continue
        fwd, left = transform_xy(float(pt[0]), float(pt[1]), ego_frame)
        out.append((t, fwd, left))
    return out


# CANDIDATE_BANK_CROSS_COLLISION_TIME_PANEL_V1_END


def draw_cross_collision_cycle(
    cycle_id: int,
    observation: dict,
    evaluations: List[dict],
    selection: Optional[dict],
    geometries: Dict[int, List[list]],
    joint_predictions: Dict[int, dict],
    cross_cells: List[dict],
    aggregates: List[dict],
    out_path: Path,
    backward_m: float,
    forward_m: float,
    lateral_m: float,
    focus: str,
    show_labels: bool,
    show_xyt_labels: bool,
    show_toi_labels: bool,
    show_point_markers: bool,
    marker_size: float,
    show_footprints: bool,
    show_time_panel: bool,
    time_horizon_sec: float,
) -> None:
    """Render E_i x O_j collision correspondence plus selected-plan geometry.

    Left: selected/shadow ego candidate and the exact historical scenario paths
    referenced by conflicting matrix cells.  TOI/XYT markers are reconstructed
    from logged candidate geometry + paired PredictedPath history.
    Right: full candidate x scenario matrix with diagonal pairing, selector
    eligibility, and aggregate risk/TOI statistics.
    """
    from matplotlib.patches import Rectangle
    import matplotlib.pyplot as plt

    ego_frame = observation.get("ego") or {"x": 0.0, "y": 0.0, "yaw": 0.0}
    eval_by_id = {int(e.get("candidate_id", 0) or 0): e for e in evaluations}
    agg_by_id = {int(a.get("candidate_id", 0) or 0): a for a in aggregates}
    selected_id = int((selection or {}).get("selected_id", 0) or 0)
    shadow = next((a for a in aggregates if bool(a.get("shadow_best", False))), None)
    shadow_id = int((shadow or {}).get("candidate_id", 0) or 0)

    if focus == "selected":
        focus_ids = {selected_id} if selected_id else set()
    elif focus == "shadow":
        focus_ids = {shadow_id} if shadow_id else set()
    elif focus == "selected_and_shadow":
        focus_ids = {cid for cid in (selected_id, shadow_id) if cid}
    else:
        focus_ids = {int(c.get("ego_candidate_id", 0) or 0) for c in cross_cells}

    # Top row preserves the spatial correspondence + E_i x O_j matrix from 0013a8f.
    # The optional lower row is a longitudinal space-time diagram: x=ego-forward,
    # y=future time.  A red same-time collision is horizontal (same t); an orange
    # XYT correspondence can be slanted because ego/object times may differ.
    if show_time_panel:
        fig = plt.figure(figsize=(20, 13))
        grid = fig.add_gridspec(
            2,
            2,
            width_ratios=[1.25, 1.0],
            height_ratios=[1.45, 0.72],
            hspace=0.24,
            wspace=0.14,
        )
        ax_xy = fig.add_subplot(grid[0, 0])
        ax_mat = fig.add_subplot(grid[0, 1])
        ax_time = fig.add_subplot(grid[1, :])
    else:
        fig = plt.figure(figsize=(20, 9))
        grid = fig.add_gridspec(1, 2, width_ratios=[1.25, 1.0])
        ax_xy = fig.add_subplot(grid[0, 0])
        ax_mat = fig.add_subplot(grid[0, 1])
        ax_time = None

    candidate_plot_style = {}
    scenario_plot_style = {}

    # Current ego/object footprints provide geometric context.  Conflict-time
    # footprints are drawn later in red/orange at the evaluated future poses.
    from matplotlib.patches import Polygon as MplPolygon

    ego_now_polygon = ego_frame.get("collision_polygon") or []
    if len(ego_now_polygon) >= 3:
        ego_now_local = [transform_xy(float(p[0]), float(p[1]), ego_frame) for p in ego_now_polygon]
        ax_xy.add_patch(
            MplPolygon(
                ego_now_local,
                closed=True,
                fill=False,
                edgecolor="black",
                linewidth=1.2,
                alpha=0.65,
                zorder=7,
            )
        )

    for obj in observation.get("objects") or []:
        ox_w, oy_w = float(obj.get("x", 0.0)), float(obj.get("y", 0.0))
        ox, oy = transform_xy(ox_w, oy_w, ego_frame)
        current_polygon = obj.get("collision_polygon") or []
        if len(current_polygon) >= 3:
            obj_now_local = [
                transform_xy(float(p[0]), float(p[1]), ego_frame) for p in current_polygon
            ]
        else:
            local_box = _rectangle_local_polygon(obj.get("length_m"), obj.get("width_m"))
            world_box = (
                _place_local_polygon(local_box, (ox_w, oy_w, float(obj.get("yaw", 0.0))))
                if local_box
                else []
            )
            obj_now_local = [transform_xy(x, y, ego_frame) for x, y in world_box]
        if obj_now_local:
            ax_xy.add_patch(
                MplPolygon(
                    obj_now_local,
                    closed=True,
                    fill=False,
                    edgecolor="0.30",
                    linewidth=0.9,
                    alpha=0.40,
                    zorder=7,
                )
            )
        ax_xy.scatter([ox], [oy], marker="o", s=12, color="0.25", alpha=0.65, zorder=8)
        if show_labels:
            ax_xy.annotate(
                f"now {str(obj.get('object_id', ''))[:8]}",
                (ox, oy),
                xytext=(4, 4),
                textcoords="offset points",
                fontsize=7,
            )

    # Draw focused ego candidates first.
    for cid in sorted(focus_ids):
        ev = eval_by_id.get(cid, {})
        geom = geometries.get(cid, [])
        start = int(ev.get("live_start_index", 0) or 0)
        points = geom[start:]
        if len(points) < 2:
            continue
        xy = [transform_xy(float(p[0]), float(p[1]), ego_frame) for p in points]
        is_selected = cid == selected_id
        is_shadow = cid == shadow_id
        style = "-" if is_selected else "--"
        width = 3.4 if is_selected else 2.3
        label = f"E{cid}"
        if is_selected:
            label += " selected"
        if is_shadow:
            label += " shadow-best"
        (candidate_line,) = ax_xy.plot(
            [p[0] for p in xy],
            [p[1] for p in xy],
            linestyle=style,
            linewidth=width,
            alpha=0.9,
            label=label,
            zorder=10,
        )
        candidate_plot_style[cid] = {
            "color": candidate_line.get_color(),
            "linestyle": style,
            "linewidth": width,
        }
        if ax_time is not None:
            trace = _candidate_time_trace(geom, start, ego_frame)
            if len(trace) >= 2:
                ax_time.plot(
                    [q[1] for q in trace],
                    [q[0] for q in trace],
                    color=candidate_line.get_color(),
                    linestyle=style,
                    linewidth=max(1.7, 0.72 * width),
                    alpha=0.90,
                    zorder=10,
                )

    # Draw only scenario paths that are directly implicated by focused conflict cells.
    plotted_scenarios = set()
    conflict_cells = []
    for cell in cross_cells:
        eid = int(cell.get("ego_candidate_id", 0) or 0)
        if focus_ids and eid not in focus_ids:
            continue
        if not (
            bool(cell.get("same_time_collision", False)) or bool(cell.get("xyt_conflict", False))
        ):
            continue
        conflict_cells.append(cell)
        sid = int(cell.get("scenario_candidate_id", 0) or 0)
        scenario = joint_predictions.get(sid)
        if scenario is None:
            continue

        # TOI provenance is distinct from XYT provenance in 0013a8.
        if bool(cell.get("same_time_collision", False)):
            object_id = str(cell.get("first_collision_object_id", ""))
            path_index = _int_or_default(cell.get("first_collision_path_index"), -1)
        else:
            object_id = str(cell.get("object_id", ""))
            path_index = _int_or_default(cell.get("path_index"), -1)
        match = _scenario_object_path(scenario, object_id, path_index)
        if match is not None:
            key = (sid, object_id, path_index)
            if key not in plotted_scenarios:
                path = match["path"]
                raw = path.get("path_xyyaw") or []
                dt = float(path.get("time_step_sec", 0.0) or 0.0)
                age = float(cell.get("scenario_age_sec", 0.0) or 0.0)
                start_idx = max(0, min(len(raw), int(math.floor(age / dt)))) if dt > 1.0e-9 else 0
                live_raw = raw[start_idx:]
                if len(live_raw) >= 2:
                    path_xy = [transform_xy(float(p[0]), float(p[1]), ego_frame) for p in live_raw]
                    line_style = "-" if bool(cell.get("paired", False)) else ":"
                    (scenario_line,) = ax_xy.plot(
                        [p[0] for p in path_xy],
                        [p[1] for p in path_xy],
                        linestyle=line_style,
                        linewidth=1.6,
                        alpha=0.55,
                        label=f"O{sid} {object_id[:8]} p{path_index}" if show_labels else None,
                        zorder=5,
                    )
                    scenario_plot_style[key] = {
                        "color": scenario_line.get_color(),
                        "linestyle": line_style,
                        "linewidth": 1.6,
                    }
                    if ax_time is not None:
                        trace = _scenario_time_trace(
                            scenario,
                            object_id,
                            path_index,
                            float(cell.get("scenario_age_sec", 0.0) or 0.0),
                            ego_frame,
                        )
                        if len(trace) >= 2:
                            ax_time.plot(
                                [q[1] for q in trace],
                                [q[0] for q in trace],
                                color=scenario_line.get_color(),
                                linestyle=line_style,
                                linewidth=1.35,
                                alpha=0.52,
                                zorder=5,
                            )
                plotted_scenarios.add(key)

        # Same-time collision marker: show ego base path and object prediction positions at TOI.
        if bool(cell.get("same_time_collision", False)) and finite(
            cell.get("first_collision_time_sec")
        ):
            t = float(cell["first_collision_time_sec"])
            ego_pose_w = _candidate_live_pose_at_time(
                geometries.get(eid, []),
                int(eval_by_id.get(eid, {}).get("live_start_index", 0) or 0),
                t,
            )
            obj_pose_w = _scenario_pose_at_time(
                scenario,
                str(cell.get("first_collision_object_id", "")),
                _int_or_default(cell.get("first_collision_path_index"), -1),
                float(cell.get("scenario_age_sec", 0.0) or 0.0),
                t,
            )
            if ego_pose_w is not None and obj_pose_w is not None:
                ee = transform_xy(ego_pose_w[0], ego_pose_w[1], ego_frame)
                oo = transform_xy(obj_pose_w[0], obj_pose_w[1], ego_frame)
                if show_footprints:
                    _draw_conflict_footprints(
                        ax_xy,
                        observation,
                        scenario,
                        ego_pose_w,
                        obj_pose_w,
                        str(cell.get("first_collision_object_id", "")),
                        ego_frame,
                        "tab:red",
                        "-",
                        0.85,
                        23,
                    )
                if show_point_markers:
                    ax_xy.scatter(
                        [ee[0]], [ee[1]], marker="X", s=marker_size, color="tab:red", zorder=25
                    )
                    ax_xy.scatter(
                        [oo[0]], [oo[1]], marker="X", s=marker_size, color="tab:red", zorder=25
                    )
                ax_xy.plot(
                    [ee[0], oo[0]],
                    [ee[1], oo[1]],
                    color="tab:red",
                    linestyle="--",
                    linewidth=1.2,
                    alpha=0.75,
                    zorder=24,
                )
                if ax_time is not None:
                    # Same-time physical collision: both endpoints necessarily share the same y=t.
                    ax_time.plot(
                        [ee[0], oo[0]],
                        [t, t],
                        color="tab:red",
                        linestyle="--",
                        linewidth=1.7,
                        alpha=0.90,
                        zorder=25,
                    )
                if show_toi_labels:
                    ax_xy.annotate(
                        f"E{eid}×O{sid} TOI={t:.2f}s\n{str(cell.get('first_collision_object_id',''))[:8]} "
                        f"p{cell.get('first_collision_path_index')} c{cell.get('first_collision_ego_circle_index')}/"
                        f"{cell.get('first_collision_object_circle_index')}",
                        ee,
                        xytext=(6, 6),
                        textcoords="offset points",
                        fontsize=7,
                        color="tab:red",
                        bbox={"facecolor": "white", "alpha": 0.70, "edgecolor": "tab:red"},
                        zorder=30,
                    )

        # XYT marker: ego and object can occur at different times, which is the key semantic difference.
        if bool(cell.get("xyt_conflict", False)) and not bool(
            cell.get("same_time_collision", False)
        ):
            te = float(cell.get("xyt_ego_time_sec", 0.0) or 0.0)
            to = float(cell.get("xyt_object_time_sec", 0.0) or 0.0)
            ego_pose_w = _candidate_live_pose_at_time(
                geometries.get(eid, []),
                int(eval_by_id.get(eid, {}).get("live_start_index", 0) or 0),
                te,
            )
            obj_pose_w = _scenario_pose_at_time(
                scenario,
                str(cell.get("object_id", "")),
                _int_or_default(cell.get("path_index"), -1),
                float(cell.get("scenario_age_sec", 0.0) or 0.0),
                to,
            )
            if ego_pose_w is not None and obj_pose_w is not None:
                ee = transform_xy(ego_pose_w[0], ego_pose_w[1], ego_frame)
                oo = transform_xy(obj_pose_w[0], obj_pose_w[1], ego_frame)
                if show_footprints:
                    _draw_conflict_footprints(
                        ax_xy,
                        observation,
                        scenario,
                        ego_pose_w,
                        obj_pose_w,
                        str(cell.get("object_id", "")),
                        ego_frame,
                        "tab:orange",
                        ":",
                        0.55,
                        18,
                    )
                if show_point_markers:
                    ax_xy.scatter(
                        [ee[0]], [ee[1]], marker="D", s=marker_size, color="tab:orange", zorder=20
                    )
                    ax_xy.scatter(
                        [oo[0]], [oo[1]], marker="D", s=marker_size, color="tab:orange", zorder=20
                    )
                ax_xy.plot(
                    [ee[0], oo[0]],
                    [ee[1], oo[1]],
                    color="tab:orange",
                    linestyle=":",
                    linewidth=1.1,
                    alpha=0.7,
                    zorder=19,
                )
                if ax_time is not None:
                    # XYT is not necessarily same-time.  A slanted orange segment directly
                    # visualizes the logged te/to mismatch; a nearly horizontal segment means
                    # the two agents occupy the nearby region at nearly the same time.
                    ax_time.plot(
                        [ee[0], oo[0]],
                        [te, to],
                        color="tab:orange",
                        linestyle=":",
                        linewidth=1.5,
                        alpha=0.82,
                        zorder=20,
                    )
                if show_xyt_labels:
                    rho = cell.get("min_xyt_rho")
                    rho_text = f"{float(rho):.2f}" if finite(rho) else "n/a"
                    ax_xy.annotate(
                        f"XYT E{eid}×O{sid} ρ={rho_text}\n"
                        f"te={te:.2f}s to={to:.2f}s Δt={float(cell.get('xyt_delta_time_sec', 0.0) or 0.0):.2f}s",
                        ee,
                        xytext=(6, -25),
                        textcoords="offset points",
                        fontsize=7,
                        color="tab:orange",
                        bbox={"facecolor": "white", "alpha": 0.65, "edgecolor": "tab:orange"},
                        zorder=22,
                    )

    ax_xy.scatter([0.0], [0.0], marker="^", s=35, color="black", label="ego now", zorder=35)
    ax_xy.set_xlim(-backward_m, forward_m)
    ax_xy.set_ylim(-lateral_m, lateral_m)
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.grid(True, alpha=0.25)
    ax_xy.set_xlabel("ego-forward [m]")
    ax_xy.set_ylabel("ego-left [m]")
    ax_xy.set_title(
        f"collision correspondence | cycle {cycle_id}\n"
        "red=predicted same-time physical collision (TOI); orange=XYT proximity/conflict (not necessarily collision); "
        "solid O=paired, dotted O=off-diagonal"
    )
    handles, labels = ax_xy.get_legend_handles_labels()
    if handles:
        unique = dict(zip(labels, handles))
        ax_xy.legend(unique.values(), unique.keys(), fontsize=7, loc="best")

    if ax_time is not None:
        from matplotlib.lines import Line2D

        ax_time.set_xlim(-backward_m, forward_m)
        ax_time.set_xlabel("ego-forward [m]")
        ax_time.set_ylabel("future time [s]")
        ax_time.set_title(
            "longitudinal space-time correspondence | vertical axis = future time  "
            "(red horizontal = same-time collision; orange slanted = XYT proximity with te != to)"
        )
        ax_time.grid(True, alpha=0.25)
        ax_time.axhline(0.0, color="0.25", linewidth=0.8, alpha=0.55, zorder=1)
        if time_horizon_sec > 0.0:
            ax_time.set_ylim(0.0, time_horizon_sec)
        else:
            # Keep t=0 visible and derive the upper bound from the plotted candidate/scenario horizon.
            _lo, _hi = ax_time.get_ylim()
            ax_time.set_ylim(0.0, max(1.0, _hi))

        # Keep this legend semantic and compact rather than repeating every E_i/O_j ID.
        ax_time.legend(
            handles=[
                Line2D(
                    [0], [0], color="0.25", linestyle="-", linewidth=2.2, label="ego candidate E_i"
                ),
                Line2D(
                    [0], [0], color="0.45", linestyle=":", linewidth=1.5, label="scenario path O_j"
                ),
                Line2D(
                    [0],
                    [0],
                    color="tab:red",
                    linestyle="--",
                    linewidth=1.7,
                    label="same-time physical collision",
                ),
                Line2D(
                    [0],
                    [0],
                    color="tab:orange",
                    linestyle=":",
                    linewidth=1.5,
                    label="XYT proximity / time correspondence",
                ),
            ],
            fontsize=7,
            loc="upper right",
            ncol=2,
        )

    # Matrix panel: all E_i x O_j cells, even when spatial focus is selected-only.
    ego_ids = sorted({int(c.get("ego_candidate_id", 0) or 0) for c in cross_cells})
    scenario_ids = sorted({int(c.get("scenario_candidate_id", 0) or 0) for c in cross_cells})
    cell_map = {
        (int(c.get("ego_candidate_id", 0) or 0), int(c.get("scenario_candidate_id", 0) or 0)): c
        for c in cross_cells
    }
    nrow = max(1, len(ego_ids))
    ncol = max(1, len(scenario_ids))
    ax_mat.set_xlim(-0.5, ncol - 0.5 + 2.3)
    ax_mat.set_ylim(nrow - 0.5, -0.5)
    ax_mat.set_xticks(
        range(ncol), [f"O{x}" for x in scenario_ids], rotation=45, ha="right", fontsize=8
    )
    ax_mat.set_yticks(range(nrow), [f"E{x}" for x in ego_ids], fontsize=8)
    ax_mat.set_xlabel("scenario candidate")
    ax_mat.set_ylabel("ego candidate")
    ax_mat.set_title("E_i × O_j cross-collision matrix")

    for ri, eid in enumerate(ego_ids):
        for ci, sid in enumerate(scenario_ids):
            cell = cell_map.get((eid, sid))
            if cell is None:
                face = "0.95"
                label = ""
            elif bool(cell.get("same_time_collision", False)):
                face = "mistyrose"
                label = _cell_status_label(cell)
            elif bool(cell.get("xyt_conflict", False)):
                face = "moccasin"
                label = _cell_status_label(cell)
            elif not bool(cell.get("valid", False)):
                face = "0.90"
                label = "-"
            else:
                face = "honeydew"
                label = "."
            ax_mat.add_patch(
                Rectangle(
                    (ci - 0.48, ri - 0.48),
                    0.96,
                    0.96,
                    facecolor=face,
                    edgecolor="0.75",
                    linewidth=0.8,
                )
            )
            if cell is not None and bool(cell.get("paired", False)):
                ax_mat.add_patch(
                    Rectangle(
                        (ci - 0.44, ri - 0.44),
                        0.88,
                        0.88,
                        fill=False,
                        edgecolor="black",
                        linewidth=1.4,
                    )
                )
            ax_mat.text(ci, ri, label, ha="center", va="center", fontsize=7)
        agg = agg_by_id.get(eid, {})
        tags = []
        if eid == selected_id:
            tags.append("SEL")
        if eid == shadow_id:
            tags.append("SHADOW")
        if bool(agg.get("selector_hard_rejected", False)):
            tags.append("REJECT")
        risk = agg.get("mean_scenario_risk")
        toi = agg.get("min_toi_sec")
        eff = agg.get("selector_effective_score")
        risk_s = f"R={float(risk):.2f}" if finite(risk) else "R=-"
        toi_s = f"TOI={float(toi):.2f}" if finite(toi) else "TOI=-"
        eff_s = f"J={float(eff):.2f}" if finite(eff) else "J=-"
        # CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_BEGIN
        tier_s = f"T={agg.get('safety_tier', 'DISABLED')}"
        # CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_END
        ax_mat.text(
            ncol + 0.1,
            ri,
            f"{'/'.join(tags)} {tier_s} {risk_s} {toi_s} {eff_s}".strip(),
            ha="left",
            va="center",
            fontsize=7,
        )

    ax_mat.grid(False)
    for spine in ax_mat.spines.values():
        spine.set_visible(False)

    selected_agg = agg_by_id.get(selected_id, {})
    selected_eval = eval_by_id.get(selected_id, {})
    detail = (
        f"selected=E{selected_id} shadow=E{shadow_id}  "
        f"collision_fraction={selected_agg.get('collision_scenario_fraction')}  "
        f"xyt_fraction={selected_agg.get('xyt_conflict_scenario_fraction')}  "
        f"risk={selected_agg.get('mean_scenario_risk')}  minTOI={selected_agg.get('min_toi_sec')}s\n"
        f"old_dynamic: clearance={selected_eval.get('min_dynamic_clearance_m')}m  "
        f"TTC={selected_eval.get('min_dynamic_ttc_sec')}s  "
        f"dynamic_cost={((selected_eval.get('score_terms') or {}).get('dynamic_margin'))}  "
        f"safety_tier={selected_agg.get('safety_tier', (selection or {}).get('selected_safety_tier', 'DISABLED'))}  "
        f"selector_reason={(selection or {}).get('reason', 'none')}"
    )
    fig.text(
        0.01,
        0.01,
        detail,
        ha="left",
        va="bottom",
        fontsize=8,
        bbox={"facecolor": "white", "alpha": 0.78, "edgecolor": "0.75"},
    )
    fig.suptitle(
        "Candidate Bank 0013a8 collision / selector correspondence  "
        "(matrix result ↔ trajectory/scenario ↔ selected/shadow)",
        fontsize=12,
    )
    if show_time_panel:
        # Explicit margins avoid tight_layout warnings with the full-width lower panel
        # and reserve space for the two-line selector detail at the bottom.
        fig.subplots_adjust(left=0.06, right=0.985, bottom=0.09, top=0.92, hspace=0.36, wspace=0.16)
    else:
        fig.tight_layout(rect=[0.0, 0.07, 1.0, 0.96])
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# CANDIDATE_BANK_CROSS_COLLISION_VISUALIZATION_V1_END


def iter_cross_correspondence_rows(
    cross_by_cycle: Dict[int, List[dict]],
    aggregates_by_cycle: Dict[int, List[dict]],
    selections: Dict[int, dict],
    evals_by_cycle: Dict[int, List[dict]],
    observations: Dict[int, dict],
    geometries: Dict[int, List[list]],
    joint_predictions: Dict[int, dict],
):
    """Yield conflict-only correspondence rows one at a time to cap peak memory."""
    for cycle_id in sorted(cross_by_cycle):
        selection = selections.get(cycle_id) or {}
        selected_id = int(selection.get("selected_id", 0) or 0)
        shadow = next(
            (a for a in aggregates_by_cycle.get(cycle_id, []) if bool(a.get("shadow_best", False))),
            None,
        )
        shadow_id = int((shadow or {}).get("candidate_id", 0) or 0)
        agg_by_id = {
            int(a.get("candidate_id", 0) or 0): a for a in aggregates_by_cycle.get(cycle_id, [])
        }
        eval_by_id = {
            int(e.get("candidate_id", 0) or 0): e for e in evals_by_cycle.get(cycle_id, [])
        }
        obs = observations.get(cycle_id) or {}
        ego_frame = obs.get("ego") or {"x": 0.0, "y": 0.0, "yaw": 0.0}
        for cell in cross_by_cycle.get(cycle_id, []):
            if not (
                bool(cell.get("same_time_collision", False))
                or bool(cell.get("xyt_conflict", False))
            ):
                continue
            eid = int(cell.get("ego_candidate_id", 0) or 0)
            sid = int(cell.get("scenario_candidate_id", 0) or 0)
            agg = agg_by_id.get(eid, {})
            ev = eval_by_id.get(eid, {})
            scenario = joint_predictions.get(sid) or {}
            row = {
                "cycle_id": cycle_id,
                "ego_candidate_id": eid,
                "scenario_candidate_id": sid,
                "ego_source": cell.get("ego_source"),
                "scenario_source": cell.get("scenario_source"),
                "selected": eid == selected_id,
                "shadow_best": eid == shadow_id,
                "paired": bool(cell.get("paired", False)),
                "same_time_collision": bool(cell.get("same_time_collision", False)),
                "xyt_conflict": bool(cell.get("xyt_conflict", False)),
                "scenario_age_sec": cell.get("scenario_age_sec"),
                "scenario_risk": cell.get("scenario_risk"),
                "candidate_mean_scenario_risk": agg.get("mean_scenario_risk"),
                "candidate_collision_scenario_fraction": agg.get("collision_scenario_fraction"),
                "candidate_xyt_conflict_scenario_fraction": agg.get(
                    "xyt_conflict_scenario_fraction"
                ),
                "selector_effective_score": agg.get("selector_effective_score"),
                "selector_eligible": agg.get("selector_eligible"),
                "selector_hard_rejected": agg.get("selector_hard_rejected"),
                "old_dynamic_clearance_m": ev.get("min_dynamic_clearance_m"),
                "old_dynamic_ttc_sec": ev.get("min_dynamic_ttc_sec"),
            }
            if bool(cell.get("same_time_collision", False)) and finite(
                cell.get("first_collision_time_sec")
            ):
                t = float(cell["first_collision_time_sec"])
                ew = _candidate_live_xy_at_time(
                    geometries.get(eid, []), int(ev.get("live_start_index", 0) or 0), t
                )
                ow = _scenario_xy_at_time(
                    scenario,
                    str(cell.get("first_collision_object_id", "")),
                    _int_or_default(cell.get("first_collision_path_index"), -1),
                    float(cell.get("scenario_age_sec", 0.0) or 0.0),
                    t,
                )
                row.update(
                    {
                        "first_collision_time_sec": t,
                        "first_collision_object_id": cell.get("first_collision_object_id"),
                        "first_collision_path_index": cell.get("first_collision_path_index"),
                        "first_collision_path_confidence": cell.get(
                            "first_collision_path_confidence"
                        ),
                        "first_collision_ego_circle_index": cell.get(
                            "first_collision_ego_circle_index"
                        ),
                        "first_collision_object_circle_index": cell.get(
                            "first_collision_object_circle_index"
                        ),
                    }
                )
                if ew is not None:
                    el = transform_xy(ew[0], ew[1], ego_frame)
                    row.update(
                        {
                            "toi_ego_world_x": ew[0],
                            "toi_ego_world_y": ew[1],
                            "toi_ego_forward_m": el[0],
                            "toi_ego_left_m": el[1],
                        }
                    )
                if ow is not None:
                    ol = transform_xy(ow[0], ow[1], ego_frame)
                    row.update(
                        {
                            "toi_object_world_x": ow[0],
                            "toi_object_world_y": ow[1],
                            "toi_object_forward_m": ol[0],
                            "toi_object_left_m": ol[1],
                        }
                    )
            if bool(cell.get("xyt_conflict", False)):
                te = float(cell.get("xyt_ego_time_sec", 0.0) or 0.0)
                to = float(cell.get("xyt_object_time_sec", 0.0) or 0.0)
                ew = _candidate_live_xy_at_time(
                    geometries.get(eid, []), int(ev.get("live_start_index", 0) or 0), te
                )
                ow = _scenario_xy_at_time(
                    scenario,
                    str(cell.get("object_id", "")),
                    _int_or_default(cell.get("path_index"), -1),
                    float(cell.get("scenario_age_sec", 0.0) or 0.0),
                    to,
                )
                row.update(
                    {
                        "xyt_object_id": cell.get("object_id"),
                        "xyt_path_index": cell.get("path_index"),
                        "xyt_path_confidence": cell.get("path_confidence"),
                        "min_xyt_rho": cell.get("min_xyt_rho"),
                        "xyt_spatial_clearance_m": cell.get("xyt_spatial_clearance_m"),
                        "xyt_ego_time_sec": te,
                        "xyt_object_time_sec": to,
                        "xyt_delta_time_sec": cell.get("xyt_delta_time_sec"),
                        "xyt_ego_circle_index": cell.get("ego_circle_index"),
                        "xyt_object_circle_index": cell.get("object_circle_index"),
                    }
                )
                if ew is not None:
                    el = transform_xy(ew[0], ew[1], ego_frame)
                    row.update(
                        {
                            "xyt_ego_world_x": ew[0],
                            "xyt_ego_world_y": ew[1],
                            "xyt_ego_forward_m": el[0],
                            "xyt_ego_left_m": el[1],
                        }
                    )
                if ow is not None:
                    ol = transform_xy(ow[0], ow[1], ego_frame)
                    row.update(
                        {
                            "xyt_object_world_x": ow[0],
                            "xyt_object_world_y": ow[1],
                            "xyt_object_forward_m": ol[0],
                            "xyt_object_left_m": ol[1],
                        }
                    )
            yield row


def write_cross_correspondence_csv(
    path: Path,
    cross_by_cycle: Dict[int, List[dict]],
    aggregates_by_cycle: Dict[int, List[dict]],
    selections: Dict[int, dict],
    evals_by_cycle: Dict[int, List[dict]],
    observations: Dict[int, dict],
    geometries: Dict[int, List[list]],
    joint_predictions: Dict[int, dict],
) -> int:
    """Stream correspondence rows through a temporary JSONL spool.

    The spool keeps peak RAM bounded while preserving 0013a8d's exact dynamic
    CSV column discovery/order: fields appear in first-seen order across rows.
    """
    spool_path = path.with_name(path.name + ".tmp.jsonl")
    fields, seen = [], set()
    count = 0
    try:
        with spool_path.open("w", encoding="utf-8") as spool:
            for row in iter_cross_correspondence_rows(
                cross_by_cycle,
                aggregates_by_cycle,
                selections,
                evals_by_cycle,
                observations,
                geometries,
                joint_predictions,
            ):
                for key in row:
                    if key not in seen:
                        seen.add(key)
                        fields.append(key)
                spool.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")
                count += 1

        if count == 0:
            path.write_text("", encoding="utf-8")
            return 0

        with path.open("w", newline="", encoding="utf-8") as out, spool_path.open(
            "r", encoding="utf-8"
        ) as spool:
            writer = csv.DictWriter(out, fieldnames=fields)
            writer.writeheader()
            for line in spool:
                writer.writerow(json.loads(line))
        return count
    finally:
        try:
            spool_path.unlink(missing_ok=True)
        except OSError:
            pass


# CANDIDATE_BANK_ANALYZER_LOW_MEMORY_V1_END


def flatten_joint_predictions(event: dict) -> List[dict]:
    rows: List[dict] = []
    for obj in event.get("objects") or []:
        object_id = str(obj.get("object_id", ""))
        paths = obj.get("paths") or []
        if not paths:
            rows.append(
                {
                    "cycle_id": event.get("cycle_id"),
                    "candidate_id": event.get("candidate_id"),
                    "source": event.get("source"),
                    "batch_index": event.get("batch_index"),
                    "object_index": obj.get("object_index"),
                    "object_id": object_id,
                    "initial_x": obj.get("initial_x"),
                    "initial_y": obj.get("initial_y"),
                    "initial_yaw": obj.get("initial_yaw"),
                    "speed_mps": obj.get("speed_mps"),
                    "shape_type": obj.get("shape_type"),
                    "length_m": obj.get("length_m"),
                    "width_m": obj.get("width_m"),
                    "path_index": None,
                    "confidence": None,
                    "time_step_sec": None,
                    "point_count": 0,
                }
            )
            continue
        for path in paths:
            rows.append(
                {
                    "cycle_id": event.get("cycle_id"),
                    "candidate_id": event.get("candidate_id"),
                    "source": event.get("source"),
                    "batch_index": event.get("batch_index"),
                    "object_index": obj.get("object_index"),
                    "object_id": object_id,
                    "initial_x": obj.get("initial_x"),
                    "initial_y": obj.get("initial_y"),
                    "initial_yaw": obj.get("initial_yaw"),
                    "speed_mps": obj.get("speed_mps"),
                    "shape_type": obj.get("shape_type"),
                    "length_m": obj.get("length_m"),
                    "width_m": obj.get("width_m"),
                    "path_index": path.get("path_index"),
                    "confidence": path.get("confidence"),
                    "time_step_sec": path.get("time_step_sec"),
                    "point_count": path.get("point_count", len(path.get("path_xyyaw") or [])),
                }
            )
    return rows


def draw_joint_prediction_cycle(
    cycle_id: int,
    observation: dict,
    evaluations: List[dict],
    selection: Optional[dict],
    geometries: Dict[int, List[list]],
    joint_predictions: Dict[int, dict],
    newest_by_cycle: Dict[int, int],
    out_path: Path,
    backward_m: float,
    forward_m: float,
    lateral_m: float,
    fade_sec: float,
    show_labels: bool,
) -> None:
    import matplotlib.pyplot as plt

    ego = observation.get("ego") or {"x": 0.0, "y": 0.0, "yaw": 0.0}
    newest_id = int(newest_by_cycle.get(cycle_id, 0))
    selected_id = int(selection.get("selected_id", 0)) if selection else 0
    eval_by_id = {int(e.get("candidate_id", 0)): e for e in evaluations}

    fig, ax = plt.subplots(figsize=(12, 8))

    # Current tracked objects are shown as high-contrast points.  Their UUID hue matches the
    # historical paired predictions, making observation-vs-prediction drift visible directly.
    current_ids = set()
    for obj in observation.get("objects", []):
        object_id = str(obj.get("object_id", ""))
        current_ids.add(object_id)
        x, y = transform_xy(float(obj.get("x", 0.0)), float(obj.get("y", 0.0)), ego)
        base = joint_prediction_base_color(object_id)
        ax.scatter(
            [x],
            [y],
            s=70,
            marker="o",
            color=base,
            edgecolors="black",
            linewidths=0.7,
            alpha=0.95,
            zorder=30,
        )
        if show_labels:
            ax.annotate(
                f"now {object_id[:8]}",
                (x, y),
                xytext=(5, 5),
                textcoords="offset points",
                fontsize=7,
                zorder=31,
            )

    # Draw oldest first so recent hypotheses remain visible on top.
    active = []
    for cid, ev in eval_by_id.items():
        joint = joint_predictions.get(cid)
        if joint is None:
            continue
        age = float(ev.get("age_sec", 0.0) or 0.0)
        active.append((age, cid, ev, joint))
    active.sort(reverse=True)

    fade_horizon = max(0.1, float(fade_sec))
    for age_sec, cid, ev, joint in active:
        freshness = max(0.0, min(1.0, 1.0 - max(0.0, age_sec) / fade_horizon))
        alpha = 0.10 + 0.85 * freshness
        brightness = 0.35 + 0.65 * freshness
        width = 0.7 + 2.1 * freshness
        newest = cid == newest_id
        selected = cid == selected_id

        # Paired ego E_i, neutral gray.  Use Candidate Bank's already-rebased live start.
        points = geometries.get(cid, [])
        start = int(ev.get("live_start_index", 0) or 0)
        points = points[start:]
        if len(points) >= 2:
            xy = [transform_xy(float(pt[0]), float(pt[1]), ego) for pt in points]
            gray = min(1.0, 0.45 + 0.50 * freshness)
            ax.plot(
                [q[0] for q in xy],
                [q[1] for q in xy],
                color=(gray, gray, gray),
                alpha=1.0 if newest else min(0.80, alpha),
                linewidth=max(3.2 if newest else 0.0, width),
                linestyle="-",
                zorder=14 if newest else 6,
            )
            if show_labels:
                suffix = " newest" if newest else (" selected" if selected else "")
                ax.annotate(f"E{cid}{suffix}", xy[-1], fontsize=7, alpha=max(0.35, alpha))

        # Paired other scenario O_i.  Age shifts both opacity and brightness; UUID fixes hue.
        for obj in joint.get("objects") or []:
            object_id = str(obj.get("object_id", ""))
            base = joint_prediction_base_color(object_id)
            color = tuple(component * brightness for component in base)
            for path in obj.get("paths") or []:
                raw = path.get("path_xyyaw") or []
                dt = float(path.get("time_step_sec", 0.0) or 0.0)
                start_index = int(max(0.0, age_sec) / dt) if dt > 1.0e-9 else 0
                start_index = min(len(raw), start_index)
                raw = raw[start_index:]
                if len(raw) < 2:
                    continue
                xy = [transform_xy(float(pt[0]), float(pt[1]), ego) for pt in raw]
                ax.plot(
                    [q[0] for q in xy],
                    [q[1] for q in xy],
                    color=color,
                    alpha=1.0 if newest else alpha,
                    linewidth=max(3.4 if newest else 0.0, width + 0.3),
                    linestyle="-",
                    zorder=20 if newest else 8,
                )
                if show_labels:
                    ax.annotate(
                        f"O{cid}/{object_id[:8]}",
                        xy[-1],
                        fontsize=7,
                        alpha=max(0.35, alpha),
                        color=color,
                    )

    ax.scatter([0.0], [0.0], marker="^", s=120, color="black", zorder=40, label="ego now")
    ax.set_xlim(-backward_m, forward_m)
    ax.set_ylim(-lateral_m, lateral_m)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("ego-forward [m]")
    ax.set_ylabel("ego-left [m]")
    ax.set_title(
        f"paired joint prediction history | cycle {cycle_id} | "
        f"samples={len(active)} newest=#{newest_id} selected=#{selected_id}\n"
        "same UUID=same hue; newer=brighter/thicker; historical paths show live suffix only"
    )
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_END


# CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
def flatten_dynamic_analysis(event: dict) -> dict:
    return {
        k: v
        for k, v in event.items()
        if k not in {"samples", "map_filter_decisions", "thread_id", "event"}
    }


def flatten_map_filter_decisions(event: dict) -> List[dict]:
    rows: List[dict] = []
    for decision in event.get("map_filter_decisions") or []:
        row = {
            "cycle_id": event.get("cycle_id"),
            "role": event.get("role"),
            "candidate_id": event.get("candidate_id"),
            "source": event.get("source"),
            "dynamic_scope_mode": event.get("dynamic_scope_mode", "all"),
        }
        row.update({k: v for k, v in decision.items() if k != "lane_hypotheses"})
        rows.append(row)
    return rows


# CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
def flatten_lane_hypotheses(event: dict) -> List[dict]:
    rows: List[dict] = []
    for decision in event.get("map_filter_decisions") or []:
        for rank, hypothesis in enumerate(decision.get("lane_hypotheses") or [], start=1):
            row = {
                "cycle_id": event.get("cycle_id"),
                "role": event.get("role"),
                "candidate_id": event.get("candidate_id"),
                "source": event.get("source"),
                "object_id": decision.get("object_id"),
                "decision_reason": decision.get("reason"),
                "lane_associated": decision.get("lane_associated"),
                "lane_association_ambiguous": decision.get("lane_association_ambiguous"),
                "hypothesis_rank": rank,
            }
            row.update(hypothesis)
            rows.append(row)
    return rows


def flatten_map_context_lanes(event: dict) -> List[dict]:
    rows: List[dict] = []
    for lane in event.get("lanes") or []:
        row = {
            "context_cycle_id": event.get("cycle_id"),
            "valid": event.get("valid"),
            "forward_search_distance_m": event.get("forward_search_distance_m"),
            "backtrace_distance_m": event.get("backtrace_distance_m"),
            "lane_id": lane.get("lane_id"),
            "is_route_lane": lane.get("is_route_lane"),
            "is_competing_lane": lane.get("is_competing_lane"),
            "route_s_start_m": lane.get("route_s_start_m"),
            "centerline_point_count": len(lane.get("centerline_xy") or []),
        }
        rows.append(row)
    return rows


def flatten_map_context_conflicts(event: dict) -> List[dict]:
    rows: List[dict] = []
    for conflict in event.get("conflicts") or []:
        row = {
            "context_cycle_id": event.get("cycle_id"),
            "ego_route_lane_id": conflict.get("ego_route_lane_id"),
            "competing_lane_id": conflict.get("competing_lane_id"),
            "x": conflict.get("x"),
            "y": conflict.get("y"),
            "ego_route_distance_m": conflict.get("ego_route_distance_m"),
            "approach_lane_ids": ";".join(
                str(v) for v in (conflict.get("approach_lane_ids") or [])
            ),
        }
        rows.append(row)
    return rows


def closest_point_on_polyline(
    point_xy: Tuple[float, float], polyline_xy: List[Tuple[float, float]]
) -> Optional[Tuple[float, float]]:
    if len(polyline_xy) < 2:
        return None
    px, py = point_xy
    best = None
    best_d2 = float("inf")
    for i in range(1, len(polyline_xy)):
        ax, ay = polyline_xy[i - 1]
        bx, by = polyline_xy[i]
        dx, dy = bx - ax, by - ay
        len2 = dx * dx + dy * dy
        if len2 <= 1.0e-12:
            continue
        u = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len2))
        qx, qy = ax + u * dx, ay + u * dy
        d2 = (px - qx) ** 2 + (py - qy) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = (qx, qy)
    return best


# CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END


def flatten_dynamic_samples(event: dict) -> List[dict]:
    rows: List[dict] = []
    worst_index = int(event.get("worst_sample_index", 0) or 0)
    for index, sample in enumerate(event.get("samples") or []):
        row = {
            "cycle_id": event.get("cycle_id"),
            "role": event.get("role"),
            "candidate_id": event.get("candidate_id"),
            "source": event.get("source"),
            "object_id": event.get("object_id"),
            "path_index": event.get("path_index"),
            "is_fallback": event.get("is_fallback"),
            "path_confidence": event.get("path_confidence"),
            "normalized_confidence": event.get("normalized_confidence"),
            "sample_index": index,
            "is_worst_sample": index == worst_index,
        }
        row.update(sample)
        rows.append(row)
    return rows


def _pose_to_local(
    x: float, y: float, pose_x: float, pose_y: float, pose_yaw: float
) -> Tuple[float, float]:
    dx = x - pose_x
    dy = y - pose_y
    c = math.cos(pose_yaw)
    ss = math.sin(pose_yaw)
    return c * dx + ss * dy, -ss * dx + c * dy


def _local_to_world(
    x: float, y: float, pose_x: float, pose_y: float, pose_yaw: float
) -> Tuple[float, float]:
    c = math.cos(pose_yaw)
    ss = math.sin(pose_yaw)
    return pose_x + c * x - ss * y, pose_y + ss * x + c * y


def _find_observation_object(observation: dict, object_id: str) -> Optional[dict]:
    if not object_id:
        return None
    for obj in observation.get("objects", []):
        if str(obj.get("object_id", "")) == object_id:
            return obj
    return None


def _draw_future_footprint(
    ax, observation: dict, sample: dict, object_id: str, ego_frame: dict, alpha: float = 0.75
) -> None:
    from matplotlib.patches import Circle
    from matplotlib.patches import Polygon as MplPolygon

    obj = _find_observation_object(observation, object_id)
    if obj:
        current_x = float(obj.get("x", 0.0))
        current_y = float(obj.get("y", 0.0))
        current_yaw = float(obj.get("yaw", 0.0))
        future_x = float(sample.get("object_x", current_x))
        future_y = float(sample.get("object_y", current_y))
        future_yaw = float(sample.get("object_yaw", current_yaw))

        polygon = obj.get("collision_polygon") or []
        if len(polygon) >= 3:
            future_world = []
            for pt in polygon:
                lx, ly = _pose_to_local(
                    float(pt[0]), float(pt[1]), current_x, current_y, current_yaw
                )
                future_world.append(_local_to_world(lx, ly, future_x, future_y, future_yaw))
            future_local = [transform_xy(x, y, ego_frame) for x, y in future_world]
            ax.add_patch(
                MplPolygon(future_local, closed=True, fill=False, linewidth=2.0, alpha=alpha)
            )
        else:
            circles = obj.get("collision_circles") or []
            if circles:
                for circle in circles:
                    lx, ly = _pose_to_local(
                        float(circle.get("x", current_x)),
                        float(circle.get("y", current_y)),
                        current_x,
                        current_y,
                        current_yaw,
                    )
                    wx, wy = _local_to_world(lx, ly, future_x, future_y, future_yaw)
                    cx, cy = transform_xy(wx, wy, ego_frame)
                    radius = float(circle.get("radius_m", 0.0))
                    ax.add_patch(Circle((cx, cy), radius, fill=False, linewidth=2.0, alpha=alpha))
            elif finite(obj.get("radius_m")):
                cx, cy = transform_xy(future_x, future_y, ego_frame)
                ax.add_patch(
                    Circle((cx, cy), float(obj["radius_m"]), fill=False, linewidth=2.0, alpha=alpha)
                )

    # Ego polygon is logged only in polygon mode. Transform the exact current footprint to the
    # future candidate pose; other collision modes fall back to an ego point marker.
    ego = observation.get("ego") or ego_frame
    ego_polygon = ego.get("collision_polygon") or []
    future_ego_x = float(sample.get("ego_x", ego.get("x", 0.0)))
    future_ego_y = float(sample.get("ego_y", ego.get("y", 0.0)))
    future_ego_yaw = float(sample.get("ego_yaw", ego.get("yaw", 0.0)))
    if len(ego_polygon) >= 3:
        future_world = []
        current_x = float(ego.get("x", 0.0))
        current_y = float(ego.get("y", 0.0))
        current_yaw = float(ego.get("yaw", 0.0))
        for pt in ego_polygon:
            lx, ly = _pose_to_local(float(pt[0]), float(pt[1]), current_x, current_y, current_yaw)
            future_world.append(_local_to_world(lx, ly, future_ego_x, future_ego_y, future_ego_yaw))
        future_local = [transform_xy(x, y, ego_frame) for x, y in future_world]
        ax.add_patch(MplPolygon(future_local, closed=True, fill=False, linewidth=2.0, alpha=alpha))
    else:
        ex, ey = transform_xy(future_ego_x, future_ego_y, ego_frame)
        ax.scatter([ex], [ey], marker="s", s=65, alpha=alpha)


def draw_dynamic_cycle(
    cycle_id: int,
    observation: dict,
    dynamic_events: List[dict],
    evaluations: List[dict],
    geometries: Dict[int, List[list]],
    out_path: Path,
    backward_m: float,
    forward_m: float,
    lateral_m: float,
) -> None:
    import matplotlib.pyplot as plt

    if not dynamic_events:
        return
    ego = observation.get("ego") or {"x": 0.0, "y": 0.0, "yaw": 0.0}
    eval_by_id = {int(e.get("candidate_id", 0)): e for e in evaluations}
    role_order = {"selected": 0, "highest_progress": 1}
    events = sorted(dynamic_events, key=lambda e: role_order.get(str(e.get("role")), 99))

    fig, (ax_xy, ax_clear, ax_cost) = plt.subplots(1, 3, figsize=(19, 6.5))

    labels = []
    dynamic_scores = []
    text_lines = []
    for event in events:
        cid = int(event.get("candidate_id", 0) or 0)
        role = str(event.get("role", "unknown"))
        source = str(event.get("source", "unknown"))
        label = f"{role} #{cid} {source}"
        labels.append(label)
        dynamic_scores.append(float(event.get("score_dynamic_margin", 0.0) or 0.0))

        ev = eval_by_id.get(cid, {})
        start = int(ev.get("live_start_index", 0) or 0)
        geometry = (geometries.get(cid) or [])[start:]
        if len(geometry) >= 2:
            xy = [transform_xy(float(pt[0]), float(pt[1]), ego) for pt in geometry]
            ax_xy.plot(
                [q[0] for q in xy], [q[1] for q in xy], linewidth=1.4, alpha=0.45, label=label
            )

        samples = event.get("samples") or []
        if samples:
            ego_xy = [
                transform_xy(float(q.get("ego_x", 0.0)), float(q.get("ego_y", 0.0)), ego)
                for q in samples
            ]
            object_xy = [
                transform_xy(float(q.get("object_x", 0.0)), float(q.get("object_y", 0.0)), ego)
                for q in samples
            ]
            ax_xy.plot([q[0] for q in ego_xy], [q[1] for q in ego_xy], linewidth=2.2, alpha=0.9)
            ax_xy.plot(
                [q[0] for q in object_xy],
                [q[1] for q in object_xy],
                linestyle="--",
                linewidth=2.0,
                alpha=0.8,
            )
            worst_index = min(
                max(int(event.get("worst_sample_index", 0) or 0), 0), len(samples) - 1
            )
            worst = samples[worst_index]
            ex, ey = ego_xy[worst_index]
            ox, oy = object_xy[worst_index]
            ax_xy.scatter([ex], [ey], marker="s", s=85)
            ax_xy.scatter([ox], [oy], marker="o", s=85)
            ax_xy.plot([ex, ox], [ey, oy], linestyle=":", linewidth=1.5)
            _draw_future_footprint(ax_xy, observation, worst, str(event.get("object_id", "")), ego)

            times = [float(q.get("time_sec", 0.0)) for q in samples]
            geometric = [float(q.get("geometric_clearance_m", math.nan)) for q in samples]
            evaluated = [float(q.get("evaluated_clearance_m", math.nan)) for q in samples]
            uncertainty = [float(q.get("uncertainty_margin_m", math.nan)) for q in samples]
            ax_clear.plot(times, geometric, linewidth=1.6, label=f"{role}: geometric")
            ax_clear.plot(times, evaluated, linewidth=2.1, label=f"{role}: evaluated")
            ax_clear.plot(
                times, uncertainty, linestyle=":", linewidth=1.3, label=f"{role}: uncertainty"
            )
            ax_clear.scatter(
                [float(worst.get("time_sec", 0.0))],
                [float(worst.get("evaluated_clearance_m", 0.0))],
                marker="x",
                s=75,
            )

            text_lines.append(
                f"{label}\n"
                f"  object={str(event.get('object_id', ''))[:12]} path={event.get('path_index')} "
                f"conf={event.get('path_confidence')} norm={event.get('normalized_confidence')}\n"
                f"  worst t={worst.get('time_sec')}s geom={worst.get('geometric_clearance_m')}m "
                f"unc={worst.get('uncertainty_margin_m')}m eval={worst.get('evaluated_clearance_m')}m\n"
                f"  closing={worst.get('closing_rate_mps')}m/s path_cost={event.get('path_total_soft_cost')} "
                f"TTC={event.get('ttc_sec')} violation={event.get('collision_margin_violated')}"
            )
        else:
            text_lines.append(
                f"{label}\n  no dynamic detail (no usable dynamic path/fallback contributor)"
            )

        decisions = event.get("map_filter_decisions") or []
        if decisions:
            decision_lines = []
            for d in decisions[:8]:
                if finite(d.get("conflict_x_map")) and finite(d.get("conflict_y_map")):
                    cx, cy = transform_xy(
                        float(d["conflict_x_map"]), float(d["conflict_y_map"]), ego
                    )
                    ax_xy.scatter([cx], [cy], marker="X", s=95, alpha=0.8)
                    ax_xy.annotate(
                        f"conflict {d.get('conflict_competing_lane_id')}",
                        (cx, cy),
                        xytext=(5, 5),
                        textcoords="offset points",
                        fontsize=7,
                    )
                decision_lines.append(
                    f"{str(d.get('object_id', ''))[:8]}:{d.get('reason')} "
                    f"lane={d.get('best_lane_id')} conf={d.get('best_lane_confidence')} "
                    f"fit={d.get('best_lane_predicted_path_fit_m')} "
                    f"head={d.get('motion_heading_source')} "
                    f"ego_t={d.get('ego_arrival_time_sec')} obj_t={d.get('object_arrival_time_sec')}"
                )
            text_lines.append(
                f"{label} map filter [{event.get('dynamic_scope_mode', 'all')}]\n  "
                + "\n  ".join(decision_lines)
            )

    ax_xy.scatter([0.0], [0.0], marker="^", s=100)
    ax_xy.set_xlim(-backward_m, forward_m)
    ax_xy.set_ylim(-lateral_m, lateral_m)
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.grid(True, alpha=0.25)
    ax_xy.set_xlabel("ego-forward [m]")
    ax_xy.set_ylabel("ego-left [m]")
    ax_xy.set_title("same-time ego/object prediction\n(solid ego, dashed object)")
    ax_xy.legend(fontsize=7, loc="best")

    first = events[0]
    if finite(first.get("collision_margin_m")):
        ax_clear.axhline(
            float(first["collision_margin_m"]),
            linestyle="--",
            linewidth=1.2,
            label="collision margin",
        )
    if finite(first.get("preferred_clearance_m")):
        ax_clear.axhline(
            float(first["preferred_clearance_m"]),
            linestyle="-.",
            linewidth=1.2,
            label="preferred clearance",
        )
    ax_clear.axhline(0.0, linewidth=1.0)
    ax_clear.grid(True, alpha=0.25)
    ax_clear.set_xlabel("future time [s]")
    ax_clear.set_ylabel("distance / margin [m]")
    ax_clear.set_title("clearance decomposition")
    ax_clear.legend(fontsize=7, loc="best")

    positions = list(range(len(labels)))
    ax_cost.bar(positions, dynamic_scores, alpha=0.75)
    ax_cost.set_xticks(positions)
    ax_cost.set_xticklabels(labels, rotation=18, ha="right", fontsize=8)
    ax_cost.set_ylabel("weighted dynamic score term")
    ax_cost.set_title("candidate dynamic-cost attribution")
    ax_cost.grid(True, axis="y", alpha=0.25)
    for pos, event, score in zip(positions, events, dynamic_scores):
        ax_cost.text(
            pos,
            score,
            f"score={score:.3g}\npath={float(event.get('path_total_soft_cost', 0.0) or 0.0):.3g}\n"
            f"clr={float(event.get('path_clearance_soft_cost', 0.0) or 0.0):.3g} "
            f"ttc={float(event.get('path_ttc_soft_cost', 0.0) or 0.0):.3g}",
            ha="center",
            va="bottom",
            fontsize=8,
        )
    ax_cost.text(
        0.02,
        0.02,
        "\n\n".join(text_lines),
        transform=ax_cost.transAxes,
        va="bottom",
        ha="left",
        fontsize=7,
        bbox={"facecolor": "white", "alpha": 0.72, "edgecolor": "0.7"},
    )

    fig.suptitle(f"Candidate Bank dynamic collision analysis | cycle {cycle_id}")
    fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.95])
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN


def _draw_map_filter_vehicle_shape(
    ax,
    obj: dict,
    ego: dict,
    *,
    fill: bool,
    linestyle: str,
    linewidth: float,
    alpha: float,
    zorder: float,
) -> bool:
    """Draw the logged object/ego footprint without inventing dimensions.

    Geometry priority: exact polygon -> exact collision circles -> logged oriented length/width box
    -> legacy radius. Returns True when any footprint was drawn.
    """
    from matplotlib.patches import Circle
    from matplotlib.patches import Polygon as MplPolygon

    collision_polygon = obj.get("collision_polygon") or []
    if len(collision_polygon) >= 3:
        polygon_xy = [
            transform_xy(float(pt[0]), float(pt[1]), ego)
            for pt in collision_polygon
            if isinstance(pt, (list, tuple)) and len(pt) >= 2
        ]
        if len(polygon_xy) >= 3:
            ax.add_patch(
                MplPolygon(
                    polygon_xy,
                    closed=True,
                    fill=fill,
                    linestyle=linestyle,
                    linewidth=linewidth,
                    alpha=alpha,
                    zorder=zorder,
                )
            )
            return True

    collision_circles = obj.get("collision_circles") or []
    drawn = False
    for circle in collision_circles:
        radius = circle.get("radius_m")
        if not finite(radius):
            continue
        cx, cy = transform_xy(
            float(circle.get("x", obj.get("x", 0.0))),
            float(circle.get("y", obj.get("y", 0.0))),
            ego,
        )
        ax.add_patch(
            Circle(
                (cx, cy),
                float(radius),
                fill=fill,
                linestyle=linestyle,
                linewidth=linewidth,
                alpha=alpha,
                zorder=zorder,
            )
        )
        drawn = True
    if drawn:
        return True

    length_m = obj.get("length_m")
    width_m = obj.get("width_m")
    if finite(length_m) and finite(width_m) and float(length_m) > 0.0 and float(width_m) > 0.0:
        cx = float(obj.get("x", 0.0))
        cy = float(obj.get("y", 0.0))
        yaw = float(obj.get("yaw", 0.0) or 0.0)
        half_l = 0.5 * float(length_m)
        half_w = 0.5 * float(width_m)
        c = math.cos(yaw)
        si = math.sin(yaw)
        world_poly = []
        for lx, ly in ((half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)):
            world_poly.append((cx + c * lx - si * ly, cy + si * lx + c * ly))
        polygon_xy = [transform_xy(x, y, ego) for x, y in world_poly]
        ax.add_patch(
            MplPolygon(
                polygon_xy,
                closed=True,
                fill=fill,
                linestyle=linestyle,
                linewidth=linewidth,
                alpha=alpha,
                zorder=zorder,
            )
        )
        return True

    radius = obj.get("radius_m")
    if finite(radius):
        ox, oy = transform_xy(float(obj.get("x", 0.0)), float(obj.get("y", 0.0)), ego)
        ax.add_patch(
            Circle(
                (ox, oy),
                float(radius),
                fill=fill,
                linestyle=linestyle,
                linewidth=linewidth,
                alpha=alpha,
                zorder=zorder,
            )
        )
        return True
    return False


def draw_map_filter_cycle(
    cycle_id: int,
    observation: dict,
    map_context: dict,
    dynamic_events: List[dict],
    out_path: Path,
    backward_m: float,
    forward_m: float,
    lateral_m: float,
) -> None:
    """Visualize the exact map search context and lane-association/filter decisions."""
    import matplotlib.pyplot as plt

    ego = observation.get("ego") or {"x": 0.0, "y": 0.0, "yaw": 0.0}
    selected_events = [e for e in dynamic_events if e.get("role") == "selected"]
    decision_event = (
        selected_events[0] if selected_events else (dynamic_events[0] if dynamic_events else {})
    )
    decisions = decision_event.get("map_filter_decisions") or []
    decision_by_object = {str(d.get("object_id", "")): d for d in decisions}

    fig = plt.figure(figsize=(15, 9))
    grid = fig.add_gridspec(1, 2, width_ratios=[1.8, 1.0])
    ax = fig.add_subplot(grid[0, 0])
    ax_text = fig.add_subplot(grid[0, 1])
    ax_text.axis("off")

    lane_by_id: Dict[int, dict] = {}
    first_route = True
    first_competing = True
    first_other = True
    for lane in map_context.get("lanes") or []:
        lane_id = int(lane.get("lane_id", 0) or 0)
        lane_by_id[lane_id] = lane
        raw_points = lane.get("centerline_xy") or []
        if len(raw_points) < 2:
            continue
        xy = [
            transform_xy(float(pt[0]), float(pt[1]), ego)
            for pt in raw_points
            if isinstance(pt, (list, tuple)) and len(pt) >= 2
        ]
        if len(xy) < 2:
            continue
        xs = [p[0] for p in xy]
        ys = [p[1] for p in xy]
        is_route = bool(lane.get("is_route_lane", False))
        is_competing = bool(lane.get("is_competing_lane", False))
        if is_route:
            label = "ego route / forward-search lanes" if first_route else None
            first_route = False
            ax.plot(xs, ys, linewidth=3.2, alpha=0.9, label=label)
        elif is_competing:
            label = "competing / backtrace lanes" if first_competing else None
            first_competing = False
            ax.plot(xs, ys, linestyle="--", linewidth=2.3, alpha=0.8, label=label)
        else:
            label = "other map-context lanes" if first_other else None
            first_other = False
            ax.plot(xs, ys, linestyle=":", linewidth=1.0, alpha=0.45, label=label)

        # Label only lanes that participate in the search context.
        if is_route or is_competing:
            mid = xy[len(xy) // 2]
            prefix = "R" if is_route else "C"
            ax.annotate(f"{prefix}:{lane_id}", mid, fontsize=7, alpha=0.8)

    for idx, conflict in enumerate(map_context.get("conflicts") or []):
        if not finite(conflict.get("x")) or not finite(conflict.get("y")):
            continue
        cx, cy = transform_xy(float(conflict["x"]), float(conflict["y"]), ego)
        ax.scatter(
            [cx],
            [cy],
            marker="X",
            s=95,
            alpha=0.9,
            label="map conflict point" if idx == 0 else None,
        )
        ax.annotate(
            f"X {conflict.get('ego_route_lane_id')}×{conflict.get('competing_lane_id')}",
            (cx, cy),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=7,
        )

    objects_by_id = {
        str(obj.get("object_id", "")): obj for obj in (observation.get("objects") or [])
    }

    text_lines = [
        f"cycle={cycle_id}",
        f"context_valid={map_context.get('valid')}",
        f"forward_search={map_context.get('forward_search_distance_m')} m",
        f"backtrace={map_context.get('backtrace_distance_m')} m",
        f"lanes={len(map_context.get('lanes') or [])} conflicts={len(map_context.get('conflicts') or [])}",
        f"candidate role={decision_event.get('role', 'none')} id={decision_event.get('candidate_id', 0)}",
        "",
        "Object decisions:",
    ]

    # Encode influence on ego directly in the vehicle footprint:
    #   FULL-EVAL  : solid + filled footprint
    #   LANE-MATCH : dashed + unfilled footprint when not FULL
    #   UNMATCHED  : marker-only (legacy compact representation)
    for object_id, obj in objects_by_id.items():
        ox, oy = transform_xy(float(obj.get("x", 0.0)), float(obj.get("y", 0.0)), ego)
        decision = decision_by_object.get(object_id)
        if decision is None:
            marker = "s" if bool(obj.get("is_static", False)) else "."
            ax.scatter([ox], [oy], marker=marker, s=30, alpha=0.4, zorder=5)
            continue

        associated = bool(decision.get("lane_associated", False))
        full = bool(decision.get("full_dynamic_evaluation", False))
        guard = bool(decision.get("immediate_guard_only", False))

        footprint_drawn = False
        if full:
            footprint_drawn = _draw_map_filter_vehicle_shape(
                ax, obj, ego, fill=True, linestyle="-", linewidth=2.2, alpha=0.42, zorder=8
            )
            status = "FULL-EVAL" if associated else "FULL-EVAL/NO-LANE"
        elif associated:
            footprint_drawn = _draw_map_filter_vehicle_shape(
                ax, obj, ego, fill=False, linestyle="--", linewidth=1.7, alpha=0.72, zorder=7
            )
            status = "LANE-MATCH/GUARD" if guard else "LANE-MATCH/IGNORED"
        else:
            status = "UNMATCHED/GUARD" if guard else "UNMATCHED"

        if not footprint_drawn:
            marker = "*" if full else ("o" if associated else "x")
            size = 150 if full else (80 if associated else 70)
            ax.scatter([ox], [oy], marker=marker, s=size, alpha=0.9, zorder=8)
        else:
            ax.scatter([ox], [oy], marker=".", s=18, alpha=0.85, zorder=9)

        # Draw all top-K lane hypotheses from object to the corresponding map lane.
        hypotheses = decision.get("lane_hypotheses") or []
        for rank, hypothesis in enumerate(hypotheses, start=1):
            lane_id = int(hypothesis.get("lane_id", 0) or 0)
            lane = lane_by_id.get(lane_id)
            if lane is None:
                continue
            raw = lane.get("centerline_xy") or []
            local_poly = [
                transform_xy(float(pt[0]), float(pt[1]), ego)
                for pt in raw
                if isinstance(pt, (list, tuple)) and len(pt) >= 2
            ]
            nearest = closest_point_on_polyline((ox, oy), local_poly)
            if nearest is not None:
                linewidth = 2.0 if rank == 1 else 0.9
                alpha = max(0.2, min(0.95, float(hypothesis.get("confidence", 0.0) or 0.0)))
                ax.plot(
                    [ox, nearest[0]],
                    [oy, nearest[1]],
                    linestyle="-" if rank == 1 else ":",
                    linewidth=linewidth,
                    alpha=alpha,
                )

        # Motion-heading arrow is the heading actually used by lane association.
        if finite(decision.get("motion_heading_yaw")):
            yaw_rel = float(decision["motion_heading_yaw"]) - float(ego.get("yaw", 0.0))
            reliability = float(decision.get("motion_heading_reliability", 0.0) or 0.0)
            arrow_len = 1.5 + 1.5 * max(0.0, min(1.0, reliability))
            ax.arrow(
                ox,
                oy,
                arrow_len * math.cos(yaw_rel),
                arrow_len * math.sin(yaw_rel),
                width=0.03,
                head_width=0.35,
                length_includes_head=True,
                alpha=0.65,
            )

        short_id = object_id[:8]
        ax.annotate(
            f"{short_id}\n{status}\n{decision.get('reason')}",
            (ox, oy),
            xytext=(6, 6),
            textcoords="offset points",
            fontsize=7,
        )

        text_lines.append(f"{short_id}  {status}  reason={decision.get('reason')}")
        text_lines.append(
            f"  best lane={decision.get('best_lane_id')} conf={decision.get('best_lane_confidence')} "
            f"lat={decision.get('best_lane_lateral_distance_m')}m "
            f"heading={decision.get('best_lane_heading_error_deg')}deg "
            f"fit={decision.get('best_lane_predicted_path_fit_m')}m"
        )
        text_lines.append(
            f"  heading source={decision.get('motion_heading_source')} "
            f"ambiguous={decision.get('lane_association_ambiguous')}"
        )
        if finite(decision.get("ego_arrival_time_sec")) or finite(
            decision.get("object_arrival_time_sec")
        ):
            text_lines.append(
                f"  conflict lanes={decision.get('conflict_ego_lane_id')}×"
                f"{decision.get('conflict_competing_lane_id')} "
                f"ego_t={decision.get('ego_arrival_time_sec')} "
                f"obj_t={decision.get('object_arrival_time_sec')}"
            )
        if hypotheses:
            text_lines.append("  top-K hypotheses:")
            for rank, h in enumerate(hypotheses, start=1):
                text_lines.append(
                    f"    #{rank} lane={h.get('lane_id')} conf={h.get('confidence')} "
                    f"route={h.get('is_route_lane')} lat={h.get('lateral_distance_m')}m "
                    f"head={h.get('heading_error_deg')}deg fit={h.get('predicted_path_fit_m')}m"
                )
        else:
            text_lines.append("  top-K hypotheses: none")

    # Ego is also shown as a vehicle footprint. New logs from 0013a5a always contain the exact
    # configured ego box; older logs still fall back to the triangle if no ego footprint exists.
    ego_shape_drawn = _draw_map_filter_vehicle_shape(
        ax, ego, ego, fill=True, linestyle="-", linewidth=2.5, alpha=0.30, zorder=10
    )
    if ego_shape_drawn:
        ax.scatter([0.0], [0.0], marker="^", s=45, alpha=0.9, zorder=11, label="ego vehicle")
    else:
        ax.scatter(
            [0.0],
            [0.0],
            marker="^",
            s=120,
            alpha=0.9,
            zorder=11,
            label="ego (footprint unavailable)",
        )

    ax.set_xlim(-backward_m, forward_m)
    ax.set_ylim(-lateral_m, lateral_m)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("ego-forward [m]")
    ax.set_ylabel("ego-left [m]")
    ax.set_title(
        "HD-map dynamic filter scope\n"
        "filled solid=FULL-EVAL, dashed footprint=LANE-MATCH only, marker=UNMATCHED"
    )
    ax.legend(fontsize=7, loc="best")

    ax_text.text(
        0.0,
        1.0,
        "\n".join(text_lines),
        va="top",
        ha="left",
        fontsize=8,
        family="monospace",
    )
    fig.suptitle(f"Candidate Bank map-conflict filter visualization | cycle {cycle_id}")
    fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.96])
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END


def is_dynamic_problem_cycle(
    cycle: int,
    dynamic_events: Dict[int, List[dict]],
    evaluations: Dict[int, List[dict]],
    dynamic_cost_threshold: float,
) -> bool:
    events = dynamic_events.get(cycle, [])
    if any(bool(e.get("collision_margin_violated", False)) for e in events):
        return True
    for ev in evaluations.get(cycle, []):
        if bool(ev.get("dynamic_collision_margin_violated", False)):
            return True
        terms = ev.get("score_terms") or {}
        score = terms.get("dynamic_margin", 0.0)
        if finite(score) and float(score) >= dynamic_cost_threshold:
            return True
    return False


# CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END

# CANDIDATE_BANK_PARALLEL_RENDER_V1_BEGIN
# Parent-populated, read-only rendering state.  On Linux the worker pool is
# created with the ``fork`` start method, so this large object graph is inherited
# via copy-on-write instead of being pickled once per cycle/task.
_PARALLEL_RENDER_STATE: Optional[dict] = None
_RENDER_BACKEND_READY = False


def _ensure_render_backend() -> None:
    """Initialize the non-interactive Matplotlib backend once per process."""
    global _RENDER_BACKEND_READY
    if _RENDER_BACKEND_READY:
        return
    import matplotlib

    matplotlib.use("Agg", force=True)
    _RENDER_BACKEND_READY = True


def _render_worker_init() -> None:
    # ProcessPoolExecutor initializer: avoid repeating backend selection for every cycle.
    _ensure_render_backend()


def _render_cycle_outputs(cycle: int) -> dict:
    """Render every requested visualization for one cycle.

    Only the integer cycle id is sent through ProcessPoolExecutor.  The large
    dictionaries parsed from internal_events.jsonl are inherited read-only from
    the parent process.  Keeping all plot types for the same cycle in one task
    also avoids repeatedly scheduling the same cycle for normal/joint/cross/
    dynamic/map-filter output.
    """
    state = _PARALLEL_RENDER_STATE
    if state is None:
        raise RuntimeError("parallel render state is not initialized")

    # PNG generation is non-interactive.  The initializer performs this once
    # per worker; the guard also keeps serial mode safe.
    _ensure_render_backend()

    counts = {"normal": 0, "joint": 0, "cross": 0, "dynamic": 0, "map_filter": 0}
    observation = state["observations"].get(cycle)

    if state["normal_enabled"] and observation:
        draw_cycle(
            cycle,
            observation,
            state["evals_by_cycle"].get(cycle, []),
            state["selections"].get(cycle),
            state["geometries"],
            state["newest_by_cycle"],
            state["frames_dir"] / f"cycle_{cycle:08d}.png",
            state["backward_m"],
            state["forward_m"],
            state["lateral_m"],
            state["show_labels"],
        )
        counts["normal"] = 1

    if state["joint_enabled"] and observation:
        evaluations = state["evals_by_cycle"].get(cycle, [])
        if any(int(e.get("candidate_id", 0)) in state["joint_predictions"] for e in evaluations):
            draw_joint_prediction_cycle(
                cycle,
                observation,
                evaluations,
                state["selections"].get(cycle),
                state["geometries"],
                state["joint_predictions"],
                state["newest_by_cycle"],
                state["joint_frames_dir"] / f"cycle_{cycle:08d}.png",
                state["backward_m"],
                state["forward_m"],
                state["lateral_m"],
                state["joint_fade_sec"],
                state["show_labels"],
            )
            counts["joint"] = 1

    if state["cross_enabled"] and observation:
        cells = state["cross_by_cycle"].get(cycle, [])
        problem_ok = not state["cross_only_problem_cycles"] or _cross_problem_cycle(
            cycle,
            state["cross_by_cycle"],
            state["cross_aggregates_by_cycle"],
            state["selections"],
            state["cross_focus"],
        )
        if cells and problem_ok:
            draw_cross_collision_cycle(
                cycle,
                observation,
                state["evals_by_cycle"].get(cycle, []),
                state["selections"].get(cycle),
                state["geometries"],
                state["joint_predictions"],
                cells,
                state["cross_aggregates_by_cycle"].get(cycle, []),
                state["cross_frames_dir"] / f"cycle_{cycle:08d}.png",
                state["backward_m"],
                state["forward_m"],
                state["lateral_m"],
                state["cross_focus"],
                state["show_labels"],
                state["cross_xyt_labels"],
                state["cross_toi_labels"],
                state["cross_point_markers"],
                state["cross_marker_size"],
                state["cross_footprints"],
                state["cross_time_panel"],
                state["cross_time_horizon_sec"],
            )
            counts["cross"] = 1

    if state["dynamic_enabled"]:
        events = state["dynamic_by_cycle"].get(cycle, [])
        problem_ok = not state["dynamic_only_problem_cycles"] or is_dynamic_problem_cycle(
            cycle,
            state["dynamic_by_cycle"],
            state["evals_by_cycle"],
            state["dynamic_cost_threshold"],
        )
        if events and problem_ok and observation:
            draw_dynamic_cycle(
                cycle,
                observation,
                events,
                state["evals_by_cycle"].get(cycle, []),
                state["geometries"],
                state["dynamic_frames_dir"] / f"cycle_{cycle:08d}.png",
                state["backward_m"],
                state["forward_m"],
                state["lateral_m"],
            )
            counts["dynamic"] = 1

    if state["map_filter_enabled"] and observation:
        map_context = state["map_context_by_cycle"].get(cycle)
        events = state["dynamic_by_cycle"].get(cycle, [])
        if (
            map_context
            and events
            and any(e.get("dynamic_scope_mode") == "map_conflict_reachable" for e in events)
        ):
            draw_map_filter_cycle(
                cycle,
                observation,
                map_context,
                events,
                state["map_filter_frames_dir"] / f"cycle_{cycle:08d}.png",
                state["backward_m"],
                state["forward_m"],
                state["lateral_m"],
            )
            counts["map_filter"] = 1

    return counts


def _render_cycle_batch(cycles: List[int]) -> dict:
    """Render a small cycle batch to amortize ProcessPool scheduling overhead."""
    totals = {"normal": 0, "joint": 0, "cross": 0, "dynamic": 0, "map_filter": 0}
    for cycle in cycles:
        counts = _render_cycle_outputs(cycle)
        for key in totals:
            totals[key] += int(counts[key])
    totals["cycles"] = len(cycles)
    totals["pid"] = os.getpid()
    return totals


# CANDIDATE_BANK_PARALLEL_RENDER_V1_END


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path, help="directory containing internal_events.jsonl")
    parser.add_argument("--max-cycles", type=int, default=200)
    parser.add_argument("--frame-stride", type=int, default=5)
    # CANDIDATE_BANK_PARALLEL_RENDER_V1_BEGIN
    parser.add_argument(
        "--workers",
        type=int,
        default=1,
        help="number of cycle-render worker processes; 1 preserves serial rendering",
    )
    parser.add_argument(
        "--render-batch-cycles",
        type=int,
        default=4,
        help="cycles handled per worker task; reduces scheduling overhead (default: 4)",
    )
    parser.add_argument(
        "--progress-interval-sec",
        type=float,
        default=5.0,
        help="seconds between loading/render progress lines; 0 disables periodic lines",
    )
    # CANDIDATE_BANK_PARALLEL_RENDER_V1_END
    parser.add_argument("--summary-only", action="store_true")
    parser.add_argument("--backward-m", type=float, default=10.0)
    parser.add_argument("--forward-m", type=float, default=40.0)
    parser.add_argument("--lateral-m", type=float, default=15.0)
    parser.add_argument("--show-labels", action="store_true")
    # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_BEGIN
    parser.add_argument(
        "--joint-predictions",
        action="store_true",
        help="render historical paired ego/other Diffusion Planner joint predictions",
    )
    parser.add_argument(
        "--joint-only",
        action="store_true",
        help="skip normal Candidate Bank frames and render only paired joint-prediction frames",
    )
    parser.add_argument(
        "--joint-fade-sec",
        type=float,
        default=5.0,
        help="age horizon used to fade historical paired predictions",
    )
    # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_END
    # CANDIDATE_BANK_CROSS_COLLISION_VISUALIZATION_V1_BEGIN
    parser.add_argument(
        "--cross-collision-debug",
        action="store_true",
        help="render E_i x O_j collision/TOI/XYT correspondence frames",
    )
    parser.add_argument(
        "--cross-only",
        action="store_true",
        help="skip normal Candidate Bank frames and render only cross-collision frames",
    )
    parser.add_argument(
        "--cross-only-problem-cycles",
        action="store_true",
        help="with --cross-collision-debug, render only cycles with collision/XYT/reject in focus",
    )
    parser.add_argument(
        "--cross-focus",
        choices=["selected", "shadow", "selected_and_shadow", "all"],
        default="selected_and_shadow",
        help="which ego candidates to draw spatially; matrix always shows all cells",
    )
    parser.add_argument(
        "--cross-xyt-labels",
        action="store_true",
        help="show per-conflict orange XYT text boxes; off by default to avoid plot clutter",
    )
    parser.add_argument(
        "--cross-toi-labels",
        action="store_true",
        help="show per-collision red TOI text boxes; off by default to avoid plot clutter",
    )
    parser.add_argument(
        "--cross-point-markers",
        action="store_true",
        help="show point markers at TOI/XYT correspondence endpoints; off by default because lines/footprints are clearer",
    )
    parser.add_argument(
        "--cross-marker-size",
        type=float,
        default=18.0,
        help="marker area used with --cross-point-markers (default: 18; old display used 60/110)",
    )
    parser.add_argument(
        "--no-cross-footprints",
        action="store_true",
        help="disable ego/object footprint outlines at TOI/XYT correspondence poses",
    )
    parser.add_argument(
        "--no-cross-time-panel",
        action="store_true",
        help="disable the lower longitudinal space-time panel (enabled by default)",
    )
    parser.add_argument(
        "--cross-time-horizon-sec",
        type=float,
        default=0.0,
        help="upper time limit for the lower space-time panel; 0=auto from plotted trajectories",
    )
    # CANDIDATE_BANK_CROSS_COLLISION_VISUALIZATION_V1_END
    # CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
    parser.add_argument(
        "--dynamic-debug",
        action="store_true",
        help="render dynamic collision analysis frames from candidate_dynamic_analysis events",
    )
    parser.add_argument(
        "--dynamic-only",
        action="store_true",
        help="skip normal Candidate Bank frames and render only dynamic-analysis frames",
    )
    parser.add_argument(
        "--dynamic-only-problem-cycles",
        action="store_true",
        help="with --dynamic-debug, render only margin-violation/high-dynamic-cost cycles",
    )
    parser.add_argument(
        "--dynamic-cost-threshold",
        type=float,
        default=5.0,
        help="weighted candidate dynamic score threshold for --dynamic-only-problem-cycles",
    )
    # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
    parser.add_argument(
        "--map-filter-debug",
        action="store_true",
        help="render HD-map search/lane-association/filter-decision frames",
    )
    # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END
    # CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END
    args = parser.parse_args()
    # CANDIDATE_BANK_PARALLEL_RENDER_V1_BEGIN
    if args.workers < 1:
        parser.error("--workers must be >= 1")
    if args.render_batch_cycles < 1:
        parser.error("--render-batch-cycles must be >= 1")
    if args.progress_interval_sec < 0.0:
        parser.error("--progress-interval-sec must be >= 0")
    # CANDIDATE_BANK_PARALLEL_RENDER_V1_END
    if args.dynamic_only:
        args.dynamic_debug = True
    # CANDIDATE_BANK_CROSS_COLLISION_VISUALIZATION_V1_BEGIN
    if args.cross_only:
        args.cross_collision_debug = True
    # CANDIDATE_BANK_CROSS_COLLISION_VISUALIZATION_V1_END
    # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_BEGIN
    if args.joint_only:
        args.joint_predictions = True
    # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_END

    log_path = args.run_dir / "internal_events.jsonl"
    if not log_path.exists():
        raise SystemExit(f"not found: {log_path}")

    out_dir = args.run_dir / "analysis_candidate_bank"
    frames_dir = out_dir / "frames"
    # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_BEGIN
    joint_frames_dir = out_dir / "joint_prediction_frames"
    # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_END
    # CANDIDATE_BANK_CROSS_COLLISION_VISUALIZATION_V1_BEGIN
    cross_frames_dir = out_dir / "cross_collision_frames"
    # CANDIDATE_BANK_CROSS_COLLISION_VISUALIZATION_V1_END
    # CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
    dynamic_frames_dir = out_dir / "dynamic_frames"
    # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
    map_filter_frames_dir = out_dir / "map_filter_frames"
    # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END
    # CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END
    out_dir.mkdir(parents=True, exist_ok=True)
    # CANDIDATE_BANK_ANALYZER_PROGRESS_V1_BEGIN
    global _PROGRESS_LOG_PATH
    _PROGRESS_LOG_PATH = out_dir / "analysis_progress.log"
    try:
        _PROGRESS_LOG_PATH.write_text("", encoding="utf-8")
    except OSError:
        _PROGRESS_LOG_PATH = None
    analysis_started = time.monotonic()
    log_size_bytes = log_path.stat().st_size
    _progress(
        f"start | log={log_path} | size={log_size_bytes / (1024.0 * 1024.0):.1f} MiB | "
        f"requested workers={args.workers} | host logical CPUs={os.cpu_count() or 'unknown'}"
    )
    # CANDIDATE_BANK_ANALYZER_PROGRESS_V1_END
    if not args.summary_only:
        if not args.dynamic_only and not args.joint_only and not args.cross_only:
            frames_dir.mkdir(parents=True, exist_ok=True)
        # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_BEGIN
        if args.joint_predictions:
            joint_frames_dir.mkdir(parents=True, exist_ok=True)
        # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_END
        # CANDIDATE_BANK_CROSS_COLLISION_VISUALIZATION_V1_BEGIN
        if args.cross_collision_debug:
            cross_frames_dir.mkdir(parents=True, exist_ok=True)
        # CANDIDATE_BANK_CROSS_COLLISION_VISUALIZATION_V1_END
        if args.dynamic_debug:
            dynamic_frames_dir.mkdir(parents=True, exist_ok=True)
        # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
        if args.map_filter_debug:
            map_filter_frames_dir.mkdir(parents=True, exist_ok=True)
        # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END

    geometries: Dict[int, List[list]] = {}
    generated_cycle: Dict[int, int] = {}
    evaluations_rows: List[dict] = []
    switches_rows: List[dict] = []
    purges_rows: List[dict] = []
    seed_rows: List[dict] = []
    observations: Dict[int, dict] = {}
    evals_by_cycle: Dict[int, List[dict]] = {}
    selections: Dict[int, dict] = {}
    # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_BEGIN
    joint_predictions: Dict[int, dict] = {}
    joint_prediction_rows: List[dict] = []
    # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_END
    # CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_BEGIN
    # CANDIDATE_BANK_ANALYZER_LOW_MEMORY_V1_BEGIN
    cross_collision_count = 0
    # Store one flattened representation per cell. 0013a8d retained both the
    # original event and a second flattened CSV row, which duplicated the largest table.
    # CANDIDATE_BANK_ANALYZER_LOW_MEMORY_V1_END
    # CANDIDATE_BANK_CROSS_COLLISION_VISUALIZATION_V1_BEGIN
    cross_by_cycle: Dict[int, List[dict]] = {}
    # CANDIDATE_BANK_CROSS_COLLISION_VISUALIZATION_V1_END
    # CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_BEGIN
    cross_aggregate_rows: List[dict] = []
    cross_aggregates_by_cycle: Dict[int, List[dict]] = {}
    # CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_END
    # CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_END
    # CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
    dynamic_analysis_rows: List[dict] = []
    dynamic_sample_rows: List[dict] = []
    dynamic_map_filter_rows: List[dict] = []
    # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
    dynamic_lane_hypothesis_rows: List[dict] = []
    map_context_lane_rows: List[dict] = []
    map_context_conflict_rows: List[dict] = []
    map_context_changes: Dict[int, dict] = {}
    # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END
    dynamic_by_cycle: Dict[int, List[dict]] = {}
    # CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END

    # CANDIDATE_BANK_ANALYZER_PROGRESS_V1_BEGIN
    _progress("[1/5] reading internal_events.jsonl ...")
    read_started = time.monotonic()
    last_read_report = read_started
    lines_read = 0
    bytes_read = 0
    json_errors = 0
    # Binary iteration gives an exact byte count for useful percentage progress.
    with log_path.open("rb") as f:
        for raw_line in f:
            lines_read += 1
            bytes_read += len(raw_line)
            line = raw_line.strip()
            if not line:
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                json_errors += 1
                continue
            kind = event.get("event")
            if _progress_due(last_read_report, args.progress_interval_sec):
                pct = 100.0 * bytes_read / max(1, log_size_bytes)
                elapsed = time.monotonic() - read_started
                mib_s = bytes_read / (1024.0 * 1024.0) / max(elapsed, 1e-9)
                _progress(
                    f"[1/5] reading {pct:5.1f}% | {lines_read:,} lines | "
                    f"{bytes_read / (1024.0 * 1024.0):.1f} MiB | {mib_s:.1f} MiB/s{_rss_text()}"
                )
                last_read_report = time.monotonic()
            cycle = int(event.get("cycle_id", 0) or 0)
            if kind == "candidate_generated":
                cid = int(event.get("candidate_id", 0))
                geometries[cid] = event.get("trajectory_xyvt") or []
                generated_cycle[cid] = cycle
            # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_BEGIN
            elif kind == "candidate_joint_prediction":
                cid = int(event.get("candidate_id", 0))
                joint_predictions[cid] = event
                joint_prediction_rows.extend(flatten_joint_predictions(event))
            # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_END
            # CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_BEGIN
            elif kind == "candidate_cross_collision":
                # CANDIDATE_BANK_ANALYZER_LOW_MEMORY_V1_BEGIN
                # One flattened row serves both visualization/correspondence and CSV output.
                # This removes the full duplicate cross_collision_rows list from 0013a8d.
                cross_by_cycle.setdefault(cycle, []).append(flatten_cross_collision(event))
                cross_collision_count += 1
                # CANDIDATE_BANK_ANALYZER_LOW_MEMORY_V1_END
            # CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_BEGIN
            elif kind == "candidate_cross_aggregate":
                row = flatten_cross_aggregate(event)
                cross_aggregate_rows.append(row)
                cross_aggregates_by_cycle.setdefault(cycle, []).append(row)
            # CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_END
            # CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_END
            elif kind == "candidate_observation":
                observations[cycle] = event
            elif kind == "candidate_evaluation":
                evals_by_cycle.setdefault(cycle, []).append(event)
                evaluations_rows.append(flatten_evaluation(event))
            elif kind == "candidate_selection":
                selections[cycle] = event
                switches_rows.append(
                    {k: v for k, v in event.items() if k not in {"thread_id", "event"}}
                )
            # CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
            elif kind == "candidate_dynamic_analysis":
                dynamic_by_cycle.setdefault(cycle, []).append(event)
                dynamic_analysis_rows.append(flatten_dynamic_analysis(event))
                dynamic_sample_rows.extend(flatten_dynamic_samples(event))
                dynamic_map_filter_rows.extend(flatten_map_filter_decisions(event))
                # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
                dynamic_lane_hypothesis_rows.extend(flatten_lane_hypotheses(event))
                # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END
            # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
            elif kind == "candidate_map_context":
                map_context_changes[cycle] = event
                map_context_lane_rows.extend(flatten_map_context_lanes(event))
                map_context_conflict_rows.extend(flatten_map_context_conflicts(event))
            # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END
            # CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END
            elif kind == "candidate_purged":
                purges_rows.append(
                    {k: v for k, v in event.items() if k not in {"thread_id", "event"}}
                )
            elif kind == "candidate_seed_selection":
                seed_rows.append(
                    {k: v for k, v in event.items() if k not in {"thread_id", "event"}}
                )

    _progress(
        f"[1/5] loading complete | {lines_read:,} lines | json errors={json_errors} | "
        f"candidates={len(geometries)} | observations={len(observations)} | "
        f"cross cells={cross_collision_count:,} | elapsed={_format_elapsed(time.monotonic() - read_started)}{_rss_text()}"
    )

    _progress("[2/5] building and writing CSV tables ...")
    csv_started = time.monotonic()
    write_csv(out_dir / "candidate_evaluations.csv", evaluations_rows)
    write_csv(out_dir / "candidate_switches.csv", switches_rows)
    write_csv(out_dir / "candidate_purges.csv", purges_rows)
    write_csv(out_dir / "candidate_seed_selections.csv", seed_rows)
    # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_BEGIN
    write_csv(out_dir / "candidate_joint_predictions.csv", joint_prediction_rows)
    # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_END
    # CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_BEGIN
    written_cross_collision_count = write_grouped_csv(
        out_dir / "candidate_cross_collisions.csv", cross_by_cycle
    )
    if written_cross_collision_count != cross_collision_count:
        raise RuntimeError(
            f"cross-collision row count mismatch: loaded={cross_collision_count} "
            f"written={written_cross_collision_count}"
        )
    # CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_BEGIN
    write_csv(out_dir / "candidate_cross_aggregates.csv", cross_aggregate_rows)
    selector_comparison_rows: List[dict] = []
    for cycle_id, rows in sorted(cross_aggregates_by_cycle.items()):
        shadow = next((r for r in rows if r.get("shadow_best")), None)
        selection = selections.get(cycle_id)
        actual_selected = int((selection or {}).get("selected_id", 0) or 0)
        shadow_selected = int((shadow or {}).get("candidate_id", 0) or 0)
        eligible_count = sum(bool(r.get("selector_eligible")) for r in rows)
        hard_reject_count = sum(bool(r.get("selector_hard_rejected")) for r in rows)
        selector_comparison_rows.append(
            {
                "cycle_id": cycle_id,
                "actual_selected_candidate_id": actual_selected,
                "shadow_best_candidate_id": shadow_selected,
                "selected_matches_shadow_best": bool(
                    actual_selected and shadow_selected and actual_selected == shadow_selected
                ),
                "eligible_candidate_count": eligible_count,
                "hard_rejected_candidate_count": hard_reject_count,
                "no_selector_eligible_candidate": eligible_count == 0,
            }
        )
    write_csv(out_dir / "candidate_contingency_selector_comparison.csv", selector_comparison_rows)
    # CANDIDATE_BANK_CROSS_COLLISION_VISUALIZATION_V1_BEGIN
    correspondence_started = time.monotonic()
    _progress(
        f"[2/5] streaming cross-collision correspondence from {cross_collision_count:,} matrix cells ..."
        f"{_rss_text()}"
    )
    cross_correspondence_count = write_cross_correspondence_csv(
        out_dir / "candidate_cross_collision_correspondence.csv",
        cross_by_cycle,
        cross_aggregates_by_cycle,
        selections,
        evals_by_cycle,
        observations,
        geometries,
        joint_predictions,
    )
    _progress(
        f"[2/5] correspondence complete | conflict rows={cross_correspondence_count:,} | "
        f"elapsed={_format_elapsed(time.monotonic() - correspondence_started)}{_rss_text()}"
    )
    # CANDIDATE_BANK_CROSS_COLLISION_VISUALIZATION_V1_END
    # CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_END
    # CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_END
    # CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
    write_csv(out_dir / "dynamic_evaluations.csv", dynamic_analysis_rows)
    write_csv(out_dir / "dynamic_samples.csv", dynamic_sample_rows)
    write_csv(out_dir / "dynamic_map_filter.csv", dynamic_map_filter_rows)
    # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
    write_csv(out_dir / "dynamic_lane_hypotheses.csv", dynamic_lane_hypothesis_rows)
    write_csv(out_dir / "map_context_lanes.csv", map_context_lane_rows)
    write_csv(out_dir / "map_context_conflicts.csv", map_context_conflict_rows)
    # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END
    # CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END
    _progress(
        f"[2/5] CSV tables complete | elapsed={_format_elapsed(time.monotonic() - csv_started)}{_rss_text()}"
    )

    _progress("[3/5] preparing cycle indices and carried-forward map context ...")
    prepare_started = time.monotonic()
    cycles = sorted(set(evals_by_cycle) | set(selections))
    if args.max_cycles > 0:
        cycles = cycles[: args.max_cycles]

    newest_by_cycle = {}
    generated_items = sorted((cycle, cid) for cid, cycle in generated_cycle.items())
    latest_id = 0
    pos = 0
    for cycle in cycles:
        while pos < len(generated_items) and generated_items[pos][0] <= cycle:
            latest_id = generated_items[pos][1]
            pos += 1
        newest_by_cycle[cycle] = latest_id

    # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
    # candidate_map_context is emitted only when the map lane/conflict set changes.
    # Carry the latest context forward so every analyzed cycle can be rendered.
    map_context_by_cycle: Dict[int, dict] = {}
    context_change_items = sorted(map_context_changes.items())
    current_context: Optional[dict] = None
    context_pos = 0
    for cycle in cycles:
        while (
            context_pos < len(context_change_items)
            and context_change_items[context_pos][0] <= cycle
        ):
            current_context = context_change_items[context_pos][1]
            context_pos += 1
        if current_context is not None:
            map_context_by_cycle[cycle] = current_context
    # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END
    _progress(
        f"[3/5] render preparation complete | cycles={len(cycles):,} | "
        f"elapsed={_format_elapsed(time.monotonic() - prepare_started)}"
    )

    # CANDIDATE_BANK_PARALLEL_RENDER_V1_BEGIN
    # Rendering is scheduled once per cycle.  With workers > 1, only the cycle
    # integer is transferred to child processes; parsed log data remain shared
    # read-only through Linux fork/copy-on-write.
    frame_count = 0
    joint_frame_count = 0
    cross_frame_count = 0
    dynamic_frame_count = 0
    map_filter_frame_count = 0
    render_cycle_count = 0
    render_mode = "disabled" if args.summary_only else "serial"

    render_cycles = [
        cycle
        for idx, cycle in enumerate(cycles)
        if args.frame_stride <= 1 or idx % args.frame_stride == 0
    ]

    global _PARALLEL_RENDER_STATE
    _PARALLEL_RENDER_STATE = {
        "observations": observations,
        "evals_by_cycle": evals_by_cycle,
        "selections": selections,
        "geometries": geometries,
        "newest_by_cycle": newest_by_cycle,
        "joint_predictions": joint_predictions,
        "cross_by_cycle": cross_by_cycle,
        "cross_aggregates_by_cycle": cross_aggregates_by_cycle,
        "dynamic_by_cycle": dynamic_by_cycle,
        "map_context_by_cycle": map_context_by_cycle,
        "frames_dir": frames_dir,
        "joint_frames_dir": joint_frames_dir,
        "cross_frames_dir": cross_frames_dir,
        "dynamic_frames_dir": dynamic_frames_dir,
        "map_filter_frames_dir": map_filter_frames_dir,
        "backward_m": args.backward_m,
        "forward_m": args.forward_m,
        "lateral_m": args.lateral_m,
        "show_labels": args.show_labels,
        "joint_fade_sec": args.joint_fade_sec,
        "cross_focus": args.cross_focus,
        "cross_xyt_labels": args.cross_xyt_labels,
        "cross_toi_labels": args.cross_toi_labels,
        "cross_point_markers": args.cross_point_markers,
        "cross_marker_size": max(1.0, float(args.cross_marker_size)),
        "cross_footprints": not args.no_cross_footprints,
        "cross_time_panel": not args.no_cross_time_panel,
        "cross_time_horizon_sec": max(0.0, float(args.cross_time_horizon_sec)),
        "cross_only_problem_cycles": args.cross_only_problem_cycles,
        "dynamic_only_problem_cycles": args.dynamic_only_problem_cycles,
        "dynamic_cost_threshold": args.dynamic_cost_threshold,
        "normal_enabled": (
            not args.summary_only
            and not args.dynamic_only
            and not args.joint_only
            and not args.cross_only
        ),
        "joint_enabled": args.joint_predictions and not args.summary_only,
        "cross_enabled": args.cross_collision_debug and not args.summary_only,
        "dynamic_enabled": args.dynamic_debug and not args.summary_only,
        "map_filter_enabled": args.map_filter_debug and not args.summary_only,
    }

    if not args.summary_only and render_cycles:
        enabled_outputs = []
        if _PARALLEL_RENDER_STATE["normal_enabled"]:
            enabled_outputs.append("normal")
        if _PARALLEL_RENDER_STATE["joint_enabled"]:
            enabled_outputs.append("joint")
        if _PARALLEL_RENDER_STATE["cross_enabled"]:
            enabled_outputs.append("cross")
        if _PARALLEL_RENDER_STATE["dynamic_enabled"]:
            enabled_outputs.append("dynamic")
        if _PARALLEL_RENDER_STATE["map_filter_enabled"]:
            enabled_outputs.append("map_filter")
        _progress(
            f"[4/5] rendering start | cycles={len(render_cycles):,} | workers={args.workers} | "
            f"batch={args.render_batch_cycles} cycles/task | outputs={','.join(enabled_outputs) or 'none'}"
        )
        render_started = time.monotonic()
        last_render_report = render_started
        worker_pids = set()

        def accumulate(counts: dict) -> None:
            nonlocal frame_count, joint_frame_count, cross_frame_count
            nonlocal dynamic_frame_count, map_filter_frame_count, render_cycle_count
            render_cycle_count += int(counts.get("cycles", 1))
            frame_count += int(counts["normal"])
            joint_frame_count += int(counts["joint"])
            cross_frame_count += int(counts["cross"])
            dynamic_frame_count += int(counts["dynamic"])
            map_filter_frame_count += int(counts["map_filter"])
            if counts.get("pid"):
                worker_pids.add(int(counts["pid"]))

        def report_render_progress(force: bool = False) -> None:
            nonlocal last_render_report
            now = time.monotonic()
            if not force and not _progress_due(last_render_report, args.progress_interval_sec):
                return
            elapsed = now - render_started
            rate = render_cycle_count / max(elapsed, 1e-9)
            remain = max(0, len(render_cycles) - render_cycle_count)
            eta = remain / rate if rate > 1e-9 else math.inf
            pct = 100.0 * render_cycle_count / max(1, len(render_cycles))
            eta_text = _format_elapsed(eta) if math.isfinite(eta) else "unknown"
            _progress(
                f"[4/5] rendering {pct:5.1f}% | cycles={render_cycle_count:,}/{len(render_cycles):,} | "
                f"{rate:.2f} cycles/s | ETA={eta_text} | workers seen={len(worker_pids)} | "
                f"frames normal={frame_count:,} joint={joint_frame_count:,} cross={cross_frame_count:,} "
                f"dynamic={dynamic_frame_count:,} map={map_filter_frame_count:,}"
            )
            last_render_report = now

        if args.workers == 1:
            render_mode = "serial"
            _ensure_render_backend()
            for cycle in render_cycles:
                counts = _render_cycle_outputs(cycle)
                counts["cycles"] = 1
                counts["pid"] = os.getpid()
                accumulate(counts)
                report_render_progress()
        else:
            from concurrent.futures import ProcessPoolExecutor
            from concurrent.futures import as_completed
            import multiprocessing as mp

            # Linux fork lets workers inherit the parsed dictionaries through
            # copy-on-write. Small cycle batches reduce IPC/scheduler overhead,
            # while as_completed gives true completion progress instead of being
            # blocked by the first slow cycle in ordered executor.map().
            try:
                mp_context = mp.get_context("fork")
            except ValueError as exc:
                raise SystemExit(
                    "--workers > 1 requires a platform with multiprocessing 'fork' support"
                ) from exc

            render_mode = f"parallel-fork({args.workers})"
            batches = [
                render_cycles[i : i + args.render_batch_cycles]
                for i in range(0, len(render_cycles), args.render_batch_cycles)
            ]
            _progress(
                f"[4/5] launching process pool | tasks={len(batches):,} | "
                "note: low CPU before this line means preprocessing/CSV work, not rendering"
            )
            with ProcessPoolExecutor(
                max_workers=args.workers, mp_context=mp_context, initializer=_render_worker_init
            ) as executor:
                futures = [executor.submit(_render_cycle_batch, batch) for batch in batches]
                for future in as_completed(futures):
                    accumulate(future.result())
                    report_render_progress()

        report_render_progress(force=True)
        _progress(
            f"[4/5] rendering complete | elapsed={_format_elapsed(time.monotonic() - render_started)} | "
            f"worker PIDs seen={sorted(worker_pids)}"
        )
    elif args.summary_only:
        _progress("[4/5] rendering skipped (--summary-only)")
    else:
        _progress("[4/5] rendering skipped (no cycles selected)")

    # Release the parent reference after all workers have exited.  The parsed
    # dictionaries themselves remain owned by the local variables used below.
    _PARALLEL_RENDER_STATE = None
    # CANDIDATE_BANK_PARALLEL_RENDER_V1_END

    _progress("[5/5] finalizing summary ...")
    switch_count = sum(bool(row.get("switched")) for row in switches_rows)
    hysteresis_count = sum(bool(row.get("hysteresis_kept")) for row in switches_rows)
    newest_not_best = 0
    for cycle, evs in evals_by_cycle.items():
        safe = [e for e in evs if e.get("safe") and finite(e.get("score"))]
        if safe:
            best = min(safe, key=lambda e: float(e["score"]))
            newest = newest_by_cycle.get(cycle, 0)
            if newest and int(best["candidate_id"]) != newest:
                newest_not_best += 1

    summary = (
        f"candidate geometries: {len(geometries)}\n"
        f"evaluation rows: {len(evaluations_rows)}\n"
        f"selection rows: {len(switches_rows)}\n"
        f"switch count: {switch_count}\n"
        f"hysteresis-kept count: {hysteresis_count}\n"
        f"cycles where newest != best: {newest_not_best}\n"
        f"purge rows: {len(purges_rows)}\n"
        f"seed-selection rows: {len(seed_rows)}\n"
        # CANDIDATE_BANK_PARALLEL_RENDER_V1_BEGIN
        f"render mode: {render_mode}\n"
        f"render workers: {args.workers}\n"
        f"render batch cycles: {args.render_batch_cycles}\n"
        f"progress interval sec: {args.progress_interval_sec}\n"
        f"low-memory cross storage: enabled\n"
        f"cross XYT text labels: {args.cross_xyt_labels}\n"
        f"cross TOI text labels: {args.cross_toi_labels}\n"
        f"cross point markers: {args.cross_point_markers} (size={max(1.0, float(args.cross_marker_size))})\n"
        f"cross conflict footprints: {not args.no_cross_footprints}\n"
        f"cross longitudinal time panel: {not args.no_cross_time_panel}\n"
        f"cross time horizon sec: {max(0.0, float(args.cross_time_horizon_sec))} (0=auto)\n"
        f"final parent RSS MiB: {_current_rss_mib()}\n"
        f"render cycles dispatched: {render_cycle_count}\n"
        # CANDIDATE_BANK_PARALLEL_RENDER_V1_END
        # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_BEGIN
        f"joint-prediction candidates: {len(joint_predictions)}\n"
        f"joint-prediction path rows: {len(joint_prediction_rows)}\n"
        f"joint-prediction frames written: {joint_frame_count}\n"
        # CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_END
        # CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_BEGIN
        f"cross-collision matrix rows: {cross_collision_count}\n"
        # CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_BEGIN
        f"cross-aggregate rows: {len(cross_aggregate_rows)}\n"
        f"cross-collision correspondence rows: {cross_correspondence_count}\n"
        f"cross-collision correspondence frames written: {cross_frame_count}\n"
        f"contingency selector comparison rows: {len(selector_comparison_rows)}\n"
        f"actual/shadow selector mismatches: "
        f"{sum(1 for r in selector_comparison_rows if r.get('actual_selected_candidate_id') and r.get('shadow_best_candidate_id') and not r.get('selected_matches_shadow_best'))}\n"
        f"cycles with no contingency-eligible candidate: "
        f"{sum(1 for r in selector_comparison_rows if r.get('no_selector_eligible_candidate'))}\n"
        # CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_END
        # CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_END
        f"frames written: {frame_count}\n"
        # CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
        f"dynamic-analysis rows: {len(dynamic_analysis_rows)}\n"
        f"dynamic-sample rows: {len(dynamic_sample_rows)}\n"
        f"dynamic frames written: {dynamic_frame_count}\n"
        # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
        f"map-context changes: {len(map_context_changes)}\n"
        f"lane-hypothesis rows: {len(dynamic_lane_hypothesis_rows)}\n"
        f"map-filter frames written: {map_filter_frame_count}\n"
        # CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END
        # CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END
    )
    (out_dir / "summary.txt").write_text(summary, encoding="utf-8")
    print(summary, end="")
    print(f"output: {out_dir}")
    _progress(
        f"[5/5] done | total elapsed={_format_elapsed(time.monotonic() - analysis_started)} | "
        f"output={out_dir}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
