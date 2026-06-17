#!/usr/bin/env python3
"""Visualize one MPPI iteration: top rollouts colored purple (low) to green (high weight).

Works with both dump formats:
  - Single-iteration analysis (``*_mppi_rollout_analysis_example``)
  - Closed-loop per-step snapshots (``*_two_lane_double_park_example``, etc.)

For temporal plots over the **entire run** (path, controls, arc length vs time), use
``plot_racer_dubins_temporal_mppi.py`` on the temporal ``*_log.csv``.

Usage:
  python3 scripts/mppi/plot_mppi_rollout_analysis.py dubins_circle_mppi_rollout_analysis
  python3 scripts/mppi/plot_mppi_rollout_analysis.py first_order_dubins_two_lane_double_park_log.csv --step 42
  python3 scripts/mppi/plot_mppi_rollout_analysis.py first_order_dubins_two_lane_double_park_log_rollouts/step_000042
  python3 scripts/mppi/plot_mppi_rollout_analysis.py dubins_circle_mppi_rollout_analysis --no-show -o viz.png
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

from mppi_plot_utils import (
    load_centerline,
    load_combined,
    load_costs,
    draw_road_boundaries,
    load_boundary_limits,
    load_meta,
    load_rollout_segments,
    enable_scroll_zoom,
    resolve_rollout_snapshot,
    sort_rollouts_for_draw,
    weight_to_purple_green,
    weight_to_rollout_linewidths,
    weights_for_rollout_ids,
)


def display_available() -> bool:
    if sys.platform == "darwin":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def default_output_path(snapshot) -> Path:
    if snapshot.analysis_base is not None:
        return Path(str(snapshot.analysis_base) + "_viz.png")
    return snapshot.step_dir / "rollout_analysis_viz.png"


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "path",
        type=Path,
        help="Analysis prefix, temporal log CSV, rollouts directory, or step_*/ folder",
    )
    p.add_argument("--step", type=int, default=0, help="Simulation step (closed-loop logs only)")
    p.add_argument("-o", "--output", type=Path, default=None, help="Output PNG path")
    p.add_argument("--no-show", action="store_true")
    p.add_argument("--show", action="store_true")
    p.add_argument("--top-n", type=int, default=400, help="Draw at most this many highest-weight rollouts")
    p.add_argument("--zoom-pad", type=float, default=2.0,
                   help="Padding [m] around rollout bounding box (use --zoom-pad=-1 for full centerline view)")
    p.add_argument("--no-inset", action="store_true", help="Skip the small full-track overview inset")
    args = p.parse_args()

    snapshot = resolve_rollout_snapshot(args.path, args.step)
    for req in (snapshot.meta_path, snapshot.costs_path, snapshot.rollouts_path, snapshot.combined_path):
        if not req.is_file():
            print(f"error: missing {req}", file=sys.stderr)
            if snapshot.log_csv is not None and args.step != 0:
                print(
                    f"hint: re-run the example to generate step {args.step}, or pick another --step",
                    file=sys.stderr,
                )
            return 1

    want_show = args.show or (not args.no_show and display_available())
    try:
        import matplotlib

        if not want_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection
    except ImportError as e:
        print("error: need matplotlib (pip install matplotlib)", file=sys.stderr)
        print(e, file=sys.stderr)
        return 1

    meta = load_meta(snapshot.meta_path)
    costs = load_costs(snapshot.costs_path)
    segments, seg_ids, _ = load_rollout_segments(snapshot.rollouts_path)
    combined = load_combined(snapshot.combined_path)

    raw = costs["raw_cost"]
    weights = costs["normalized"]
    baseline = meta.get("baseline", float(np.min(raw)))
    normalizer = meta.get("normalizer", float(np.sum(costs["unnormalized"])))

    seg_weights = weights_for_rollout_ids(costs, seg_ids)
    if args.top_n > 0 and len(seg_ids) > args.top_n:
        order = np.argsort(seg_weights)[::-1][: args.top_n]
        segments = [segments[i] for i in order]
        seg_ids = seg_ids[order]
        seg_weights = seg_weights[order]
    segments, seg_weights, seg_ids = sort_rollouts_for_draw(segments, seg_weights, seg_ids)
    seg_colors = weight_to_purple_green(seg_weights)

    if snapshot.centerline_path is not None:
        cpx, cpy = load_centerline(snapshot.centerline_path)
    else:
        cpx, cpy = None, None

    log_hint = snapshot.log_csv or snapshot.analysis_base or snapshot.step_dir
    sim_step = int(meta.get("sim_step", args.step))
    sim_time = float(meta.get("sim_time", sim_step * meta.get("dt", 0.1)))

    fig = plt.figure(figsize=(14, 13))
    gs = fig.add_gridspec(3, 3, height_ratios=[1.55, 0.95, 0.85], width_ratios=[1.2, 1, 1],
                          left=0.06, right=0.97, top=0.94, bottom=0.06, hspace=0.42, wspace=0.28)

    ax_xy = fig.add_subplot(gs[0, :])
    if cpx is not None:
        left_b, right_b = load_boundary_limits(meta, log_hint=log_hint)
        draw_road_boundaries(ax_xy, cpx, cpy, left_b, right_b)
        ax_xy.plot(cpx, cpy, "r-", linewidth=1.5, label="ref centerline", zorder=2)

    lc = LineCollection(segments, colors=seg_colors, linewidths=weight_to_rollout_linewidths(seg_weights), zorder=2)
    ax_xy.add_collection(lc)

    ax_xy.plot(combined["x"], combined["y"], "k-", linewidth=2.2, label="MPPI combined (optimal u)", zorder=5)
    ax_xy.scatter(combined["x"][0], combined["y"][0], c="lime", s=60, edgecolors="k", zorder=6, label="start")

    seg_index_lookup: dict[int, int] = {int(rid): i for i, rid in enumerate(seg_ids)}

    if 0 in seg_index_lookup:
        si = seg_index_lookup[0]
        ax_xy.plot(
            segments[si][:, 0], segments[si][:, 1],
            color="orange", linewidth=1.6, linestyle="--",
            label="rollout #0 (noise-free / pre-iter nominal)", zorder=3,
        )

    best_w_i = int(costs["index"][int(np.argmax(weights))])
    if best_w_i in seg_index_lookup:
        si = seg_index_lookup[best_w_i]
        ax_xy.plot(
            segments[si][:, 0], segments[si][:, 1],
            color="magenta", linewidth=1.8,
            label=f"highest-weight rollout #{best_w_i}", zorder=4,
        )

    best_c_i = int(costs["index"][int(np.argmin(raw))])

    all_xy = np.vstack(segments + [np.column_stack([combined["x"], combined["y"]])])
    x_min, y_min = all_xy.min(axis=0)
    x_max, y_max = all_xy.max(axis=0)
    pad = 0.0
    if args.zoom_pad >= 0:
        pad = max(args.zoom_pad, 0.05 * max(x_max - x_min, y_max - y_min, 1e-3))
        ax_xy.set_xlim(x_min - pad, x_max + pad)
        ax_xy.set_ylim(y_min - pad, y_max + pad)
    else:
        ax_xy.autoscale()
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    n_drawn = len(segments)
    span_m = float(np.hypot(x_max - x_min, y_max - y_min))
    step_label = f"step {sim_step}  t={sim_time:.2f}s  " if snapshot.rollout_root is not None else ""
    ax_xy.set_title(
        f"{step_label}Top {n_drawn} MPPI rollouts (purple low → green high weight)  "
        f"v₀={meta.get('init_vel_x', 0):.2f} m/s  λ={meta.get('lambda', 0):.3g}  "
        f"rollout cluster ≈ {span_m:.2f} m"
    )
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(loc="best", fontsize=8)

    if not args.no_inset and cpx is not None and args.zoom_pad >= 0:
        ax_inset = ax_xy.inset_axes([0.02, 0.02, 0.18, 0.30])
        ax_inset.plot(cpx, cpy, "r-", linewidth=0.9)
        ax_inset.plot(combined["x"], combined["y"], "k-", linewidth=1.2)
        rect_x = [x_min - pad, x_max + pad, x_max + pad, x_min - pad, x_min - pad]
        rect_y = [y_min - pad, y_min - pad, y_max + pad, y_max + pad, y_min - pad]
        ax_inset.plot(rect_x, rect_y, color="orange", linewidth=1.0)
        ax_inset.set_aspect("equal")
        ax_inset.set_xticks([])
        ax_inset.set_yticks([])
        ax_inset.set_title("overview", fontsize=7, pad=2)

    ax_cost = fig.add_subplot(gs[1, 0])
    ax_cost.hist(raw, bins=60, color="steelblue", edgecolor="white", alpha=0.85)
    ax_cost.axvline(baseline, color="crimson", linewidth=2, label=f"baseline = {baseline:.2g}")
    ax_cost.set_xlabel("raw trajectory cost")
    ax_cost.set_ylabel("count")
    ax_cost.set_title("Cost distribution")
    ax_cost.legend(fontsize=8)
    ax_cost.grid(True, alpha=0.3)

    ax_w = fig.add_subplot(gs[1, 1])
    ax_w.hist(weights, bins=60, color="seagreen", edgecolor="white", alpha=0.85)
    ax_w.set_xlabel("normalized importance weight")
    ax_w.set_ylabel("count")
    ax_w.set_title(f"Weight distribution  (Σw = {weights.sum():.2f}, Z = {normalizer:.2g})")
    ax_w.grid(True, alpha=0.3)

    ax_sc = fig.add_subplot(gs[1, 2])
    sc = ax_sc.scatter(raw, weights, c=weights, s=8, cmap="viridis", alpha=0.65)
    ax_sc.set_xlabel("raw cost")
    ax_sc.set_ylabel("normalized weight")
    ax_sc.set_title("Cost vs weight")
    ax_sc.grid(True, alpha=0.3)
    fig.colorbar(sc, ax=ax_sc, label="weight")

    sub = gs[2, :].subgridspec(2, 1, hspace=0.15)
    ax_ua = fig.add_subplot(sub[0, 0])
    ax_us = fig.add_subplot(sub[1, 0], sharex=ax_ua)
    t = combined["t"]

    ax_ua.plot(t, combined["u_accel"], color="tab:blue", linewidth=1.7, label="u_accel (cmd)")
    ax_ua.axhline(0.0, color="0.7", linewidth=0.6, linestyle="--")
    ax_ua.set_ylabel("u_accel [m/s²]")
    ax_ua.set_title("Optimal control over MPPI horizon")
    ax_ua.grid(True, alpha=0.3)
    ax_ua.legend(loc="best", fontsize=8)
    plt.setp(ax_ua.get_xticklabels(), visible=False)

    ax_us.plot(t, combined["u_steer"], color="tab:red", linewidth=1.7, label="u_steer (cmd)")
    ax_us.plot(t, combined["steer"], color="0.4", linewidth=1.2, linestyle="--", label="steer state")
    ax_us.axhline(0.0, color="0.7", linewidth=0.6, linestyle="--")
    ax_us.set_xlabel("t [s]")
    ax_us.set_ylabel("u_steer [rad]")
    ax_us.grid(True, alpha=0.3)
    ax_us.legend(loc="best", fontsize=8)

    n_at_min = int(np.sum(np.isclose(raw, raw.min(), rtol=0, atol=1e-3)))
    fig.suptitle(
        f"MPPI iteration — {n_drawn} rollouts drawn  |  best cost {raw.min():.6g} (idx {best_c_i}, "
        f"{n_at_min} rollouts within 1e-3)  |  max weight {weights.max():.3g} (idx {best_w_i})",
        fontsize=10,
    )

    out_png = args.output if args.output is not None else default_output_path(snapshot)
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"Drew {n_drawn} rollouts from {snapshot.step_dir}")
    print(f"Wrote {out_png}")
    if want_show:
        zoom_axes = [ax_xy, ax_cost, ax_w, ax_sc, ax_ua, ax_us]
        if not args.no_inset and cpx is not None and args.zoom_pad >= 0:
            zoom_axes.append(ax_inset)
        enable_scroll_zoom(fig, zoom_axes)
        print("Close plot window to exit.")
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
