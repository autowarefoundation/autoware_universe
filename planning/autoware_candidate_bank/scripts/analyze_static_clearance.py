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

"""Offline visualization for Candidate Bank static-clearance decisions.

Consumes internal_events.jsonl containing candidate_static_clearance events produced by
CANDIDATE_BANK_STATIC_CLEARANCE_DEBUG_V1.  It never recomputes collision clearance;
all reported distances and witness poses come directly from the C++ evaluator.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
import sys
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence
from typing import Set
from typing import Tuple


def finite(v) -> bool:
    return isinstance(v, (int, float)) and math.isfinite(float(v))


def parse_cycles(spec: str) -> Optional[Set[int]]:
    spec = (spec or "").strip()
    if not spec:
        return None
    out: Set[int] = set()
    for token in spec.split(","):
        token = token.strip()
        if not token:
            continue
        if ":" in token:
            a, b = token.split(":", 1)
            lo, hi = int(a), int(b)
            if hi < lo:
                lo, hi = hi, lo
            out.update(range(lo, hi + 1))
        elif "-" in token and token.count("-") == 1 and not token.startswith("-"):
            a, b = token.split("-", 1)
            lo, hi = int(a), int(b)
            if hi < lo:
                lo, hi = hi, lo
            out.update(range(lo, hi + 1))
        else:
            out.add(int(token))
    return out


def transform_xy(x: float, y: float, ego: dict) -> Tuple[float, float]:
    ex = float(ego.get("x", 0.0) or 0.0)
    ey = float(ego.get("y", 0.0) or 0.0)
    yaw = float(ego.get("yaw", 0.0) or 0.0)
    dx, dy = x - ex, y - ey
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def world_poly_to_local(poly: Sequence[Sequence[float]], ego: dict) -> List[Tuple[float, float]]:
    out = []
    for p in poly or []:
        if isinstance(p, (list, tuple)) and len(p) >= 2:
            out.append(transform_xy(float(p[0]), float(p[1]), ego))
    return out


def witness_valid(w: Optional[dict]) -> bool:
    return bool(w and w.get("valid", False) and finite(w.get("signed_clearance_m")))


def witness_summary(w: Optional[dict]) -> str:
    if not witness_valid(w):
        return "none"
    return (
        f"t={float(w.get('future_time_sec', 0.0)):.3f}s, "
        f"clearance={float(w.get('signed_clearance_m')):.3f}m, "
        f"obj={str(w.get('object_id', ''))[:12]}, "
        f"point={int(w.get('trajectory_point_index', 0) or 0)}"
    )


def load_log(path: Path, cycles_filter: Optional[Set[int]] = None):
    geometries: Dict[int, list] = {}
    generated_cycle: Dict[int, int] = {}
    observations: Dict[int, dict] = {}
    evaluations: Dict[int, Dict[int, dict]] = defaultdict(dict)
    selections: Dict[int, dict] = {}
    static_events: Dict[int, Dict[int, dict]] = defaultdict(dict)

    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            try:
                e = json.loads(line)
            except Exception:
                continue
            event = e.get("event")
            cycle = int(e.get("cycle_id", 0) or 0)
            if event == "candidate_generated":
                cid = int(e.get("candidate_id", 0) or 0)
                geometries[cid] = e.get("trajectory_xyvt") or []
                generated_cycle[cid] = cycle
                continue
            if cycles_filter is not None and cycle not in cycles_filter:
                continue
            if event == "candidate_observation":
                observations[cycle] = e
            elif event == "candidate_evaluation":
                evaluations[cycle][int(e.get("candidate_id", 0) or 0)] = e
            elif event == "candidate_selection":
                selections[cycle] = e
            elif event == "candidate_static_clearance":
                static_events[cycle][int(e.get("candidate_id", 0) or 0)] = e
    return geometries, generated_cycle, observations, evaluations, selections, static_events


def draw_polygon(
    ax,
    poly,
    ego,
    *,
    edgecolor,
    facecolor="none",
    alpha=1.0,
    linewidth=1.8,
    linestyle="-",
    zorder=5,
    label=None,
):
    from matplotlib.patches import Polygon

    xy = world_poly_to_local(poly, ego)
    if len(xy) < 3:
        return False
    ax.add_patch(
        Polygon(
            xy,
            closed=True,
            edgecolor=edgecolor,
            facecolor=facecolor,
            alpha=alpha,
            linewidth=linewidth,
            linestyle=linestyle,
            zorder=zorder,
            label=label,
        )
    )
    return True


def draw_circles(
    ax,
    circles,
    ego,
    *,
    edgecolor,
    facecolor="none",
    alpha=1.0,
    linewidth=1.8,
    linestyle="-",
    zorder=5,
    label=None,
):
    from matplotlib.patches import Circle

    first = True
    drawn = False
    for c in circles or []:
        if not finite(c.get("x")) or not finite(c.get("y")) or not finite(c.get("radius_m")):
            continue
        x, y = transform_xy(float(c["x"]), float(c["y"]), ego)
        ax.add_patch(
            Circle(
                (x, y),
                float(c["radius_m"]),
                edgecolor=edgecolor,
                facecolor=facecolor,
                alpha=alpha,
                linewidth=linewidth,
                linestyle=linestyle,
                zorder=zorder,
                label=label if first else None,
            )
        )
        first = False
        drawn = True
    return drawn


def draw_current_objects(ax, observation, ego):
    for obj in observation.get("objects") or []:
        is_static = bool(obj.get("is_static", False))
        edge = "0.25" if is_static else "0.55"
        poly = obj.get("collision_polygon") or []
        circles = obj.get("collision_circles") or []
        if not draw_polygon(ax, poly, ego, edgecolor=edge, alpha=0.45, linewidth=1.0, zorder=2):
            draw_circles(ax, circles, ego, edgecolor=edge, alpha=0.45, linewidth=1.0, zorder=2)
        if finite(obj.get("x")) and finite(obj.get("y")):
            x, y = transform_xy(float(obj["x"]), float(obj["y"]), ego)
            ax.scatter([x], [y], s=12, alpha=0.55, zorder=3)
            oid = str(obj.get("object_id", ""))
            if oid:
                ax.text(x, y, oid[:6], fontsize=6, alpha=0.65, zorder=4)


def draw_witness_geometry(ax, w: dict, ego: dict, title_prefix: str = ""):
    if not witness_valid(w):
        return
    # These polygons/circles are copied directly from the C++ evaluator at the witness sample.
    draw_polygon(
        ax,
        w.get("ego_polygon") or [],
        ego,
        edgecolor="tab:red",
        facecolor="tab:red",
        alpha=0.22,
        linewidth=2.6,
        zorder=20,
        label=f"{title_prefix}ego @ witness" if title_prefix else "ego @ witness",
    )
    if not draw_polygon(
        ax,
        w.get("object_polygon") or [],
        ego,
        edgecolor="tab:orange",
        facecolor="tab:orange",
        alpha=0.22,
        linewidth=2.6,
        zorder=19,
        label=f"{title_prefix}static object" if title_prefix else "static object",
    ):
        draw_circles(
            ax,
            w.get("object_circles") or [],
            ego,
            edgecolor="tab:orange",
            facecolor="tab:orange",
            alpha=0.22,
            linewidth=2.6,
            zorder=19,
            label=f"{title_prefix}static object" if title_prefix else "static object",
        )
    ep = w.get("ego_pose") or {}
    op = w.get("object_pose") or {}
    if finite(ep.get("x")) and finite(ep.get("y")):
        ex, ey = transform_xy(float(ep["x"]), float(ep["y"]), ego)
        ax.scatter([ex], [ey], marker="x", s=70, linewidths=2.0, zorder=25)
    if finite(op.get("x")) and finite(op.get("y")):
        ox, oy = transform_xy(float(op["x"]), float(op["y"]), ego)
        ax.scatter([ox], [oy], marker="+", s=80, linewidths=2.0, zorder=25)


def candidate_ids_for_cycle(events: Dict[int, dict], selection: dict, focus: str) -> List[int]:
    ids = sorted(events)
    selected = int(selection.get("selected_id", 0) or 0)
    if focus == "all":
        return ids
    if focus == "worst":
        valid = [
            (float((e.get("argmin") or {}).get("signed_clearance_m")), cid)
            for cid, e in events.items()
            if witness_valid(e.get("argmin"))
        ]
        return [min(valid)[1]] if valid else []
    if focus == "selected_and_worst":
        out = []
        if selected in events:
            out.append(selected)
        valid = [
            (float((e.get("argmin") or {}).get("signed_clearance_m")), cid)
            for cid, e in events.items()
            if witness_valid(e.get("argmin"))
        ]
        if valid:
            worst = min(valid)[1]
            if worst not in out:
                out.append(worst)
        return out
    return [selected] if selected in events else []


def frame_is_problem(events: Dict[int, dict], selected_id: int) -> bool:
    if selected_id in events:
        w = events[selected_id].get("argmin") or {}
        if witness_valid(w) and float(w["signed_clearance_m"]) <= float(w.get("margin_m", 0.0)):
            return True
    return any(
        witness_valid(e.get("argmin"))
        and float(e["argmin"]["signed_clearance_m"]) <= float(e["argmin"].get("margin_m", 0.0))
        for e in events.values()
    )


def draw_cycle(
    cycle: int,
    observation: dict,
    evals: Dict[int, dict],
    selection: dict,
    events: Dict[int, dict],
    geometries: Dict[int, list],
    out_path: Path,
    focus: str,
    backward_m: float,
    forward_m: float,
    lateral_m: float,
):
    import matplotlib.pyplot as plt

    ego = observation.get("ego") or {"x": 0.0, "y": 0.0, "yaw": 0.0}
    selected_id = int(selection.get("selected_id", 0) or 0)
    focus_ids = candidate_ids_for_cycle(events, selection, focus)

    fig = plt.figure(figsize=(18, 10.5))
    grid = fig.add_gridspec(
        2, 2, width_ratios=[1.45, 1.0], height_ratios=[1.15, 0.85], hspace=0.22, wspace=0.18
    )
    ax_scene = fig.add_subplot(grid[:, 0])
    ax_zoom = fig.add_subplot(grid[0, 1])
    ax_info = fig.add_subplot(grid[1, 1])
    ax_info.axis("off")

    # Full scene: current collision geometry and every Bank candidate trajectory faintly.
    draw_polygon(
        ax_scene,
        ego.get("collision_polygon") or [],
        ego,
        edgecolor="black",
        alpha=0.7,
        linewidth=1.4,
        zorder=5,
        label="ego now",
    )
    draw_current_objects(ax_scene, observation, ego)

    for cid, ev in sorted(evals.items()):
        geom = geometries.get(cid) or []
        start = int(ev.get("live_start_index", 0) or 0)
        pts = geom[start:]
        if len(pts) < 2:
            continue
        xy = [transform_xy(float(p[0]), float(p[1]), ego) for p in pts]
        is_sel = cid == selected_id
        ax_scene.plot(
            [p[0] for p in xy],
            [p[1] for p in xy],
            linewidth=3.0 if is_sel else 0.9,
            alpha=1.0 if is_sel else 0.22,
            linestyle="-" if is_sel else "--",
            zorder=12 if is_sel else 3,
            label=f"E{cid} selected" if is_sel else None,
        )

    marker_styles = [
        ("first_margin_violation", "M", "tab:purple"),
        ("first_overlap", "O", "tab:red"),
        ("argmin", "A", "tab:orange"),
    ]
    for cid in focus_ids:
        event = events.get(cid) or {}
        for key, label, color in marker_styles:
            w = event.get(key) or {}
            if not witness_valid(w):
                continue
            ep = w.get("ego_pose") or {}
            if finite(ep.get("x")) and finite(ep.get("y")):
                x, y = transform_xy(float(ep["x"]), float(ep["y"]), ego)
                ax_scene.scatter(
                    [x],
                    [y],
                    s=90,
                    marker="o",
                    facecolors="white",
                    edgecolors=color,
                    linewidths=2.0,
                    zorder=22,
                )
                ax_scene.text(x, y, f" {label} E{cid}", fontsize=8, color=color, zorder=23)
        if cid == selected_id and witness_valid(event.get("argmin")):
            draw_witness_geometry(ax_scene, event["argmin"], ego)

    ax_scene.set_xlim(backward_m, forward_m)
    ax_scene.set_ylim(-lateral_m, lateral_m)
    ax_scene.set_aspect("equal", adjustable="box")
    ax_scene.grid(True, alpha=0.25)
    ax_scene.set_xlabel("ego-forward [m]")
    ax_scene.set_ylabel("ego-left [m]")
    ax_scene.set_title(f"Static-clearance decision witnesses | cycle {cycle}")
    handles, labels = ax_scene.get_legend_handles_labels()
    if handles:
        ax_scene.legend(loc="upper right", fontsize=8)

    # Close-up: selected argmin if possible, otherwise worst candidate in the cycle.
    close_id = selected_id if selected_id in events else 0
    if close_id == 0 or not witness_valid((events.get(close_id) or {}).get("argmin")):
        candidates = [
            (float(e["argmin"]["signed_clearance_m"]), cid)
            for cid, e in events.items()
            if witness_valid(e.get("argmin"))
        ]
        close_id = min(candidates)[1] if candidates else 0
    close_event = events.get(close_id) or {}
    argmin = close_event.get("argmin") or {}
    if witness_valid(argmin):
        draw_witness_geometry(ax_zoom, argmin, ego)
        all_xy: List[Tuple[float, float]] = []
        for poly in (argmin.get("ego_polygon") or [], argmin.get("object_polygon") or []):
            all_xy.extend(world_poly_to_local(poly, ego))
        for c in argmin.get("object_circles") or []:
            if finite(c.get("x")) and finite(c.get("y")) and finite(c.get("radius_m")):
                x, y = transform_xy(float(c["x"]), float(c["y"]), ego)
                r = float(c["radius_m"])
                all_xy.extend([(x - r, y - r), (x + r, y + r)])
        if all_xy:
            xs = [p[0] for p in all_xy]
            ys = [p[1] for p in all_xy]
            pad = max(1.0, 0.25 * max(max(xs) - min(xs), max(ys) - min(ys), 1.0))
            ax_zoom.set_xlim(min(xs) - pad, max(xs) + pad)
            ax_zoom.set_ylim(min(ys) - pad, max(ys) + pad)
        clearance = float(argmin["signed_clearance_m"])
        margin = float(argmin.get("margin_m", 0.0) or 0.0)
        semantic = (
            f"penetration depth = {-clearance:.3f} m"
            if clearance < 0.0
            else f"separation = {clearance:.3f} m"
        )
        ax_zoom.set_title(
            f"E{close_id} argmin: signed clearance {clearance:.3f} m\n"
            f"{semantic}; threshold margin={margin:.3f} m"
        )
        ax_zoom.legend(loc="best", fontsize=8)
    else:
        ax_zoom.text(
            0.5,
            0.5,
            "No static-clearance witness",
            ha="center",
            va="center",
            transform=ax_zoom.transAxes,
        )
    ax_zoom.set_aspect("equal", adjustable="box")
    ax_zoom.grid(True, alpha=0.25)
    ax_zoom.set_xlabel("ego-forward [m]")
    ax_zoom.set_ylabel("ego-left [m]")

    # Text diagnosis and per-candidate ranking.
    sel_event = events.get(selected_id) or {}
    sel_eval = evals.get(selected_id) or {}
    lines = [
        f"cycle={cycle}",
        f'selected=E{selected_id}  tier={selection.get("selected_safety_tier")}  reason={selection.get("reason")}',
        f'progress={sel_eval.get("progress_m")} m  live_start={sel_eval.get("live_start_index")}',
        "",
        f'ARGMIN: {witness_summary(sel_event.get("argmin"))}',
        f'FIRST MARGIN: {witness_summary(sel_event.get("first_margin_violation"))}',
        f'FIRST OVERLAP: {witness_summary(sel_event.get("first_overlap"))}',
        "",
        "Candidate signed-clearance ranking:",
    ]
    ranking = []
    for cid, event in events.items():
        w = event.get("argmin") or {}
        if witness_valid(w):
            ranking.append((float(w["signed_clearance_m"]), cid, w))
    ranking.sort()
    for clearance, cid, w in ranking[:18]:
        marker = "*" if cid == selected_id else " "
        lines.append(
            f"{marker} E{cid:<5d} clr={clearance:>7.3f}m "
            f't={float(w.get("future_time_sec", 0.0)):>5.2f}s '
            f'obj={str(w.get("object_id", ""))[:8]}'
        )
    ax_info.text(0.0, 1.0, "\n".join(lines), va="top", ha="left", family="monospace", fontsize=9)

    fig.suptitle(
        "Candidate Bank static-clearance ground-truth diagnostic "
        "(distances/poses are logged by the C++ evaluator; no collision math is recomputed)",
        fontsize=12,
    )
    fig.savefig(out_path, dpi=145, bbox_inches="tight")
    plt.close(fig)


def flatten_witness(prefix: str, w: Optional[dict], row: dict):
    w = w or {}
    row[f"{prefix}_valid"] = bool(w.get("valid", False))
    for key in (
        "object_id",
        "object_speed_mps",
        "trajectory_point_index",
        "live_point_index",
        "future_time_sec",
        "signed_clearance_m",
        "margin_m",
        "deficit_m",
    ):
        row[f"{prefix}_{key}"] = w.get(key)
    ep = w.get("ego_pose") or {}
    op = w.get("object_pose") or {}
    for name, pose in (("ego", ep), ("object", op)):
        for key in ("x", "y", "yaw"):
            row[f"{prefix}_{name}_{key}"] = pose.get(key)


def write_csv(path: Path, static_events, selections, evaluations):
    rows = []
    fields = []
    seen = set()
    for cycle in sorted(static_events):
        sel = selections.get(cycle) or {}
        selected_id = int(sel.get("selected_id", 0) or 0)
        for cid, event in sorted(static_events[cycle].items()):
            ev = (evaluations.get(cycle) or {}).get(cid) or {}
            row = {
                "cycle_id": cycle,
                "candidate_id": cid,
                "selected": cid == selected_id,
                "source": event.get("source"),
                "collision_geometry_mode": event.get("collision_geometry_mode"),
                "selection_reason": sel.get("reason") if cid == selected_id else None,
                "selected_safety_tier": (
                    sel.get("selected_safety_tier") if cid == selected_id else None
                ),
                "progress_m": ev.get("progress_m"),
                "static_collision_margin_violated": ev.get("static_collision_margin_violated"),
                "min_static_clearance_m": ev.get("min_static_clearance_m"),
            }
            flatten_witness("argmin", event.get("argmin"), row)
            flatten_witness("first_margin", event.get("first_margin_violation"), row)
            flatten_witness("first_overlap", event.get("first_overlap"), row)
            for k in row:
                if k not in seen:
                    seen.add(k)
                    fields.append(k)
            rows.append(row)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "run_dir", type=Path, help="Candidate Bank run directory or internal_events.jsonl"
    )
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--cycles", default="", help="e.g. 479,480 or 470:490")
    parser.add_argument(
        "--focus",
        choices=["selected", "worst", "selected_and_worst", "all"],
        default="selected_and_worst",
    )
    parser.add_argument(
        "--only-violations",
        action="store_true",
        help="render only cycles where at least one candidate is <= static margin",
    )
    parser.add_argument("--max-frames", type=int, default=300)
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--backward-m", type=float, default=-10.0)
    parser.add_argument("--forward-m", type=float, default=50.0)
    parser.add_argument("--lateral-m", type=float, default=15.0)
    args = parser.parse_args()

    log_path = args.run_dir if args.run_dir.is_file() else args.run_dir / "internal_events.jsonl"
    if not log_path.is_file():
        print(f"ERROR: not found: {log_path}", file=sys.stderr)
        return 2
    run_dir = log_path.parent
    out_dir = args.output_dir or (run_dir / "analysis_static_clearance")
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    cycles_filter = parse_cycles(args.cycles)
    geometries, generated_cycle, observations, evaluations, selections, static_events = load_log(
        log_path, cycles_filter
    )
    if not static_events:
        print(
            "ERROR: no candidate_static_clearance events found. Rebuild/run with the static-clearance debug patch first.",
            file=sys.stderr,
        )
        return 3

    row_count = write_csv(
        out_dir / "static_clearance_candidates.csv", static_events, selections, evaluations
    )

    candidate_cycles = sorted(static_events)
    if args.only_violations:
        candidate_cycles = [
            c
            for c in candidate_cycles
            if frame_is_problem(
                static_events[c], int((selections.get(c) or {}).get("selected_id", 0) or 0)
            )
        ]
    stride = max(1, args.frame_stride)
    candidate_cycles = candidate_cycles[::stride]
    if args.max_frames > 0 and len(candidate_cycles) > args.max_frames:
        candidate_cycles = candidate_cycles[-args.max_frames :]

    rendered = 0
    for cycle in candidate_cycles:
        observation = observations.get(cycle)
        if not observation:
            continue
        draw_cycle(
            cycle,
            observation,
            evaluations.get(cycle, {}),
            selections.get(cycle, {}),
            static_events.get(cycle, {}),
            geometries,
            frames_dir / f"cycle_{cycle:08d}.png",
            args.focus,
            args.backward_m,
            args.forward_m,
            args.lateral_m,
        )
        rendered += 1

    # Compact text summary, especially useful when plotting is not available.
    with (out_dir / "summary.txt").open("w", encoding="utf-8") as f:
        f.write(f"log={log_path}\n")
        f.write(f"candidate_rows={row_count}\n")
        f.write(f"cycles_with_static_clearance={len(static_events)}\n")
        f.write(f"rendered_frames={rendered}\n")
        for cycle in sorted(static_events)[-50:]:
            sel = selections.get(cycle) or {}
            cid = int(sel.get("selected_id", 0) or 0)
            event = static_events[cycle].get(cid) or {}
            f.write(
                f'cycle={cycle} selected={cid} tier={sel.get("selected_safety_tier")} '
                f'reason={sel.get("reason")} argmin=[{witness_summary(event.get("argmin"))}] '
                f'first_margin=[{witness_summary(event.get("first_margin_violation"))}] '
                f'first_overlap=[{witness_summary(event.get("first_overlap"))}]\n'
            )

    print(f"log: {log_path}")
    print(f"output: {out_dir}")
    print(f"CSV rows: {row_count}")
    print(f"frames: {rendered}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
