#!/usr/bin/env python3
"""Interactively retune MPPI lambda and visualize how rollout weights change.

Loads raw costs from a dumped step (or single-iteration analysis prefix) and
recomputes importance weights as lambda is adjusted. The weighted-mean rollout
path (black line) and weighted control sequence (black lines) are recomputed
from the same weights — they update with lambda without re-running MPPI.

Use Prev/Next (or ←/→) to move between dumped simulation steps when a rollout
log with multiple steps is loaded. Scroll to zoom, click-drag to pan, and press
r to reset the rollout map view.

Usage:
  python3 scripts/mppi/plot_mppi_lambda_retune.py first_order_dubins_two_lane_double_park_log.csv --step 42
  python3 scripts/mppi/plot_mppi_lambda_retune.py dubins_stadium_mppi_rollout_analysis
"""
from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from mppi_plot_utils import (
    compute_weighted_control_trajectory,
    compute_weighted_xy_trajectory,
    load_centerline,
    load_combined,
    load_costs,
    draw_road_boundaries,
    load_boundary_limits,
    load_meta,
    load_rollout_controls,
    load_rollout_trajectories,
    load_steps_index,
    enable_scroll_zoom,
    recompute_weights,
    resolve_step_prefix,
    sort_rollouts_for_draw,
    weight_to_purple_green,
    weight_to_rollout_linewidths,
)


def display_available() -> bool:
    if sys.platform == "darwin":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


@dataclass
class StepEntry:
    step: int
    sim_time: float
    directory: Path


@dataclass
class StepData:
    step: int
    sim_time: float
    meta: dict[str, float]
    costs: dict[str, np.ndarray]
    trajectories: dict[int, np.ndarray]
    controls: dict[int, tuple[np.ndarray, np.ndarray]]
    combined: dict[str, np.ndarray] | None
    raw: np.ndarray
    rollout_indices: np.ndarray
    baseline: float
    dump_lambda: float
    dt: float


def resolve_controls_path(step_dir: Path, analysis_base: Path | None) -> Path | None:
    local = step_dir / "rollouts_controls.csv"
    if local.is_file():
        return local
    if analysis_base is not None:
        sibling = Path(str(analysis_base) + "_rollouts_controls.csv")
        if sibling.is_file():
            return sibling
    return None


def step_paths(step_dir: Path, analysis_base: Path | None) -> tuple[Path, Path, Path, Path]:
    if analysis_base is not None:
        return (
            analysis_base.with_name(analysis_base.name + "_meta.csv"),
            analysis_base.with_name(analysis_base.name + "_costs.csv"),
            analysis_base.with_name(analysis_base.name + "_rollouts_xy.csv"),
            analysis_base.with_name(analysis_base.name + "_combined.csv"),
        )
    return (
        step_dir / "meta.csv",
        step_dir / "costs.csv",
        step_dir / "rollouts_xy.csv",
        step_dir / "combined.csv",
    )


def load_step_data(step_dir: Path, analysis_base: Path | None) -> StepData:
    meta_path, costs_path, rollouts_path, combined_path = step_paths(step_dir, analysis_base)
    for req in (meta_path, costs_path, rollouts_path):
        if not req.is_file():
            raise FileNotFoundError(req)

    meta = load_meta(meta_path)
    costs = load_costs(costs_path)
    trajectories = load_rollout_trajectories(rollouts_path)
    controls_path = resolve_controls_path(step_dir, analysis_base)
    controls = load_rollout_controls(controls_path) if controls_path is not None else {}
    combined = load_combined(combined_path) if combined_path.is_file() else None

    raw = costs["raw_cost"]
    rollout_indices = costs["index"]
    baseline = float(meta.get("baseline", np.min(raw)))
    dump_lambda = float(meta.get("lambda", 1.0))
    dt = float(meta.get("dt", 0.1))
    if "step" in meta:
        step = int(meta["step"])
    elif step_dir.name.startswith("step_"):
        step = int(step_dir.name.split("_", 1)[1])
    else:
        step = 0
    sim_time = float(meta.get("sim_time", step * dt))

    return StepData(
        step=step,
        sim_time=sim_time,
        meta=meta,
        costs=costs,
        trajectories=trajectories,
        controls=controls,
        combined=combined,
        raw=raw,
        rollout_indices=rollout_indices,
        baseline=baseline,
        dump_lambda=dump_lambda,
        dt=dt,
    )


def resolve_step_directory(prefix: Path | str, rollout_root: Path | None) -> Path:
    """Resolve a step directory from steps_index prefix or step folder name."""
    p = Path(prefix)
    if p.is_dir() and (p / "costs.csv").is_file():
        return p.resolve()

    candidates: list[Path] = [p.resolve()]
    if rollout_root is not None:
        candidates.extend(
            [
                rollout_root.parent / p,
                rollout_root / p.name,
                rollout_root / p,
            ]
        )
    for candidate in candidates:
        if candidate.is_dir() and (candidate / "costs.csv").is_file():
            return candidate.resolve()
    return p.resolve()


def discover_rollout_steps(rollout_root: Path, fallback_step_dir: Path) -> list[StepEntry]:
    index_path = rollout_root / "steps_index.csv"
    if index_path.is_file():
        return [
            StepEntry(
                step=s,
                sim_time=t,
                directory=resolve_step_directory(prefix, rollout_root),
            )
            for s, t, prefix in load_steps_index(index_path)
        ]

    steps: list[StepEntry] = []
    for step_dir in sorted(rollout_root.glob("step_*")):
        if not step_dir.is_dir() or not (step_dir / "costs.csv").is_file():
            continue
        step_num = int(step_dir.name.split("_", 1)[1])
        meta_path = step_dir / "meta.csv"
        sim_time = float("nan")
        if meta_path.is_file():
            meta = load_meta(meta_path)
            sim_time = float(meta.get("sim_time", step_num * meta.get("dt", 0.1)))
        steps.append(StepEntry(step=step_num, sim_time=sim_time, directory=step_dir.resolve()))

    if not steps:
        fallback = resolve_step_directory(fallback_step_dir, rollout_root)
        if fallback.is_dir() and (fallback / "costs.csv").is_file():
            step_num = int(fallback.name.split("_", 1)[1]) if fallback.name.startswith("step_") else 0
            steps.append(StepEntry(step=step_num, sim_time=float("nan"), directory=fallback))
    return steps


def resolve_rollout_root(args_path: Path, step_dir: Path, analysis_base: Path | None) -> Path | None:
    if analysis_base is not None:
        return None

    root: Path | None = None
    if step_dir.name.startswith("step_"):
        root = step_dir.parent
    elif args_path.is_dir() and not args_path.name.startswith("step_"):
        root = args_path
    elif args_path.suffix == ".csv":
        root = args_path.parent / f"{args_path.stem}_rollouts"

    if root is None:
        return None

    if not root.is_absolute():
        anchor = args_path.parent if args_path.suffix == ".csv" else Path.cwd()
        root = (anchor / root).resolve()
    else:
        root = root.resolve()

    return root if root.is_dir() else None


def find_step_index(steps: list[StepEntry], step_dir: Path, requested_step: int) -> int:
    resolved = step_dir.resolve()
    for i, entry in enumerate(steps):
        if entry.directory.resolve() == resolved or entry.step == requested_step:
            return i
    return 0


def resolve_initial_step_dir(args: argparse.Namespace) -> tuple[Path, Path | None]:
    if args.path.is_dir() and args.path.name.startswith("step_"):
        return args.path, None
    if args.path.is_dir():
        return resolve_step_prefix(args.path, args.step), None
    if args.path.suffix == ".csv":
        return resolve_step_prefix(args.path, args.step), None

    base = args.path if args.path.suffix == "" else args.path.with_suffix("")
    return Path(str(base)), base


def load_centerline_for_step(step_dir: Path, analysis_base: Path | None) -> tuple[np.ndarray | None, np.ndarray | None]:
    candidates = [
        step_dir.parent.parent / (step_dir.parent.name.replace("_rollouts", "") + "_centerline.csv"),
        Path(str(step_dir).rsplit("_rollouts", 1)[0] + "_centerline.csv"),
        step_dir.with_name(step_dir.name + "_centerline.csv"),
    ]
    if analysis_base is not None:
        candidates.append(analysis_base.with_name(analysis_base.name + "_centerline.csv"))
    for path in candidates:
        cpx, cpy = load_centerline(path)
        if cpx is not None:
            return cpx, cpy
    return None, None


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("path", type=Path, help="Log CSV, rollouts dir, or analysis prefix")
    p.add_argument("--step", type=int, default=0)
    p.add_argument("--top-n", type=int, default=400, help="Initial number of highest-weight rollouts to draw")
    p.add_argument("--lambda-min", type=float, default=None)
    p.add_argument("--lambda-max", type=float, default=None)
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    try:
        step_dir, analysis_base = resolve_initial_step_dir(args)
        rollout_root = resolve_rollout_root(args.path, step_dir, analysis_base)
        if rollout_root is not None:
            step_dir = resolve_step_directory(step_dir, rollout_root)
        step_data = load_step_data(step_dir, analysis_base)
    except FileNotFoundError as exc:
        print(f"error: missing {exc}", file=sys.stderr)
        return 1

    if not display_available() or args.no_show:
        print("Interactive retune requires a display. Use plot_mppi_rollout_analysis.py for static PNG output.",
              file=sys.stderr)
        return 1

    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.widgets import Button, Slider

    step_entries = discover_rollout_steps(rollout_root, step_dir) if rollout_root is not None else []
    step_idx = find_step_index(step_entries, step_dir, args.step) if step_entries else 0
    multi_step = len(step_entries) > 1

    cpx, cpy = load_centerline_for_step(step_dir, analysis_base)

    init_lambda = step_data.dump_lambda
    lam_min = args.lambda_min if args.lambda_min is not None else max(init_lambda * 0.05, 1e-3)
    lam_max = args.lambda_max if args.lambda_max is not None else init_lambda * 20.0

    max_rollouts = max(len(step_data.trajectories), 1)
    init_top_n = min(max(args.top_n, 1), max_rollouts)

    fig = plt.figure(figsize=(20, 12))
    fig.subplots_adjust(bottom=0.11, left=0.05, right=0.98, top=0.96)
    gs = fig.add_gridspec(
        4,
        6,
        height_ratios=[2.8, 0.95, 0.6, 0.6],
        width_ratios=[1.0, 1.0, 1.0, 0.55, 0.55, 0.55],
        hspace=0.38,
        wspace=0.28,
    )
    ax_xy = fig.add_subplot(gs[0:2, 0:4])
    ax_hist = fig.add_subplot(gs[0, 4:6])
    ax_sc = fig.add_subplot(gs[1, 4:6])
    ax_ua = fig.add_subplot(gs[2, :])
    ax_us = fig.add_subplot(gs[3, :], sharex=ax_ua)

    if multi_step:
        ax_btn_prev = fig.add_axes([0.02, 0.055, 0.06, 0.022])
        ax_btn_next = fig.add_axes([0.09, 0.055, 0.06, 0.022])
        ax_slider_lam = fig.add_axes([0.18, 0.055, 0.67, 0.022])
    else:
        ax_slider_lam = fig.add_axes([0.15, 0.055, 0.7, 0.022])
    ax_slider_n = fig.add_axes([0.15, 0.025, 0.7, 0.022])

    if cpx is not None:
        left_b, right_b = load_boundary_limits(step_data.meta, log_hint=args.path)
        draw_road_boundaries(ax_xy, cpx, cpy, left_b, right_b)
        ax_xy.plot(cpx, cpy, "r-", linewidth=1.2, zorder=1, label="ref centerline")
    lc = LineCollection([], linewidths=0.7, zorder=2)
    ax_xy.add_collection(lc)
    combined_line, = ax_xy.plot([], [], "k-", linewidth=2.2, zorder=5, label="weighted mean path")
    ax_xy.set_aspect("equal")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(loc="best", fontsize=9)

    ua_line, = ax_ua.plot([], [], "k-", linewidth=2.0, label="Σ wᵢ·u_accel")
    us_line, = ax_us.plot([], [], "k-", linewidth=2.0, label="Σ wᵢ·u_steer")
    ref_ua_line, = ax_ua.plot([], [], color="0.65", linewidth=1.2, linestyle="--", label="MPPI u* @ dump λ")
    ref_us_line, = ax_us.plot([], [], color="0.65", linewidth=1.2, linestyle="--")
    ref_steer_line, = ax_us.plot([], [], color="0.45", linewidth=1.0, linestyle=":", label="steer state")
    ax_ua.axhline(0.0, color="0.7", linewidth=0.6, linestyle="--")
    ax_us.axhline(0.0, color="0.7", linewidth=0.6, linestyle="--")
    ax_ua.set_ylabel("u_accel [m/s²]")
    ax_us.set_xlabel("t [s]")
    ax_us.set_ylabel("u_steer [rad]")
    ax_ua.grid(True, alpha=0.3)
    ax_us.grid(True, alpha=0.3)
    ax_ua.legend(loc="best", fontsize=8)
    ax_us.legend(loc="best", fontsize=8)
    plt.setp(ax_ua.get_xticklabels(), visible=False)

    slider_lam = Slider(ax_slider_lam, "lambda", lam_min, lam_max, valinit=init_lambda, valfmt="%.4g")
    slider_n = Slider(
        ax_slider_n,
        "top rollouts",
        1,
        max_rollouts,
        valinit=init_top_n,
        valstep=1,
        valfmt="%0.0f",
    )

    btn_prev = Button(ax_btn_prev, "Prev") if multi_step else None
    btn_next = Button(ax_btn_next, "Next") if multi_step else None

    state: dict[str, object] = {"data": step_data, "step_idx": step_idx, "user_zoomed": False}

    def mark_user_zoom(_ax) -> None:
        state["user_zoomed"] = True

    enable_scroll_zoom(
        fig,
        [ax_xy, ax_hist, ax_sc, ax_ua, ax_us],
        on_user_zoom=mark_user_zoom,
    )

    def refresh_nav_buttons() -> None:
        if not multi_step or btn_prev is None or btn_next is None:
            return
        idx = int(state["step_idx"])
        btn_prev.ax.set_facecolor("0.92" if idx > 0 else "0.97")
        btn_next.ax.set_facecolor("0.92" if idx < len(step_entries) - 1 else "0.97")

    def set_combined_reference(data: StepData) -> None:
        if data.combined is None:
            ref_ua_line.set_data([], [])
            ref_us_line.set_data([], [])
            ref_steer_line.set_data([], [])
            ref_ua_line.set_label("MPPI u* @ dump λ")
            return
        t = data.combined["t"]
        ref_ua_line.set_data(t, data.combined["u_accel"])
        ref_us_line.set_data(t, data.combined["u_steer"])
        ref_steer_line.set_data(t, data.combined["steer"])
        ref_ua_line.set_label(f"MPPI u* @ dump λ={data.dump_lambda:g}")
        ax_ua.legend(loc="best", fontsize=8)
        ax_us.legend(loc="best", fontsize=8)

    def update_top_n_slider(data: StepData) -> None:
        max_n = max(len(data.trajectories), 1)
        slider_n.valmax = max_n
        slider_n.ax.set_xlim(1, max_n)
        if slider_n.val > max_n:
            slider_n.set_val(max_n)

    def update_control_title(data: StepData) -> None:
        if not data.controls:
            ax_ua.set_title("Weighted control (re-run example to dump rollouts_controls.csv for λ-dependent u*)")
        else:
            ax_ua.set_title("Weighted control over MPPI horizon")

    def load_step_at(index: int) -> None:
        entry = step_entries[index]
        data = load_step_data(entry.directory, analysis_base=None)
        state["data"] = data
        state["step_idx"] = index
        state["user_zoomed"] = False
        update_top_n_slider(data)
        set_combined_reference(data)
        update_control_title(data)
        refresh_nav_buttons()
        update()

    def change_step(delta: int) -> None:
        if not multi_step:
            return
        idx = int(state["step_idx"]) + delta
        if idx < 0 or idx >= len(step_entries):
            return
        load_step_at(idx)

    def update(_val: float | None = None) -> None:
        data: StepData = state["data"]  # type: ignore[assignment]
        lam = slider_lam.val
        top_n = int(round(slider_n.val))
        _unnorm, norm, normalizer = recompute_weights(data.raw, data.baseline, lam)
        lookup = {int(i): float(w) for i, w in zip(data.rollout_indices, norm)}

        order = np.argsort(norm)[::-1]
        visible_segments: list[np.ndarray] = []
        visible_weights: list[float] = []
        for idx in order:
            rid = int(data.rollout_indices[idx])
            if rid not in data.trajectories:
                continue
            visible_segments.append(data.trajectories[rid])
            visible_weights.append(lookup[rid])
            if len(visible_segments) >= top_n:
                break

        visible_segments, visible_weights_arr = sort_rollouts_for_draw(
            visible_segments, np.asarray(visible_weights, dtype=float)
        )
        colors = weight_to_purple_green(visible_weights_arr)
        lc.set_segments(visible_segments)
        lc.set_colors(colors)
        lc.set_linewidths(weight_to_rollout_linewidths(visible_weights_arr))

        wx, wy = compute_weighted_xy_trajectory(data.trajectories, lookup)
        combined_line.set_data(wx, wy)

        if data.controls:
            t_u, w_ua, w_us = compute_weighted_control_trajectory(data.controls, lookup, data.dt)
            ua_line.set_data(t_u, w_ua)
            us_line.set_data(t_u, w_us)
        else:
            ua_line.set_data([], [])
            us_line.set_data([], [])

        ax_hist.cla()
        ax_hist.hist(norm, bins=60, color="seagreen", edgecolor="white", alpha=0.85)
        ax_hist.set_xlabel("normalized weight")
        ax_hist.set_ylabel("count")
        ess = 1.0 / max(float(np.sum(norm * norm)), 1e-12)
        ax_hist.set_title(f"Weight distribution  Z={normalizer:.3g}  ESS≈{ess:.1f}/{len(norm)}")

        ax_sc.cla()
        ax_sc.scatter(data.raw, norm, c=norm, s=8, cmap="viridis", alpha=0.65)
        ax_sc.set_xlabel("raw cost")
        ax_sc.set_ylabel("normalized weight")
        ax_sc.set_title("Cost vs weight")
        ax_sc.grid(True, alpha=0.3)

        if wx.size and wy.size and not state["user_zoomed"]:
            all_xy = np.vstack(visible_segments + [np.column_stack([wx, wy])])
            pad = 0.05 * max(
                float(np.max(all_xy[:, 0]) - np.min(all_xy[:, 0])),
                float(np.max(all_xy[:, 1]) - np.min(all_xy[:, 1])),
                1e-3,
            )
            ax_xy.set_xlim(float(np.min(all_xy[:, 0])) - pad, float(np.max(all_xy[:, 0])) + pad)
            ax_xy.set_ylim(float(np.min(all_xy[:, 1])) - pad, float(np.max(all_xy[:, 1])) + pad)

        step_label = f"step {data.step}"
        if np.isfinite(data.sim_time):
            step_label += f"  t={data.sim_time:.3g}s"
        if multi_step:
            step_label += f"  ({int(state['step_idx']) + 1}/{len(step_entries)})"

        ax_xy.set_title(
            f"{step_label}  |  top {len(visible_segments)} rollouts  λ={lam:.4g}  baseline={data.baseline:.4g}  "
            f"black = Σ wᵢ·pathᵢ  (faint purple → bold green by weight)"
        )
        fig.canvas.draw_idle()

    if btn_prev is not None:
        btn_prev.on_clicked(lambda _event: change_step(-1))
    if btn_next is not None:
        btn_next.on_clicked(lambda _event: change_step(1))

    def on_key(event) -> None:
        if event.key in ("left", "p"):
            change_step(-1)
        elif event.key in ("right", "n"):
            change_step(1)
        elif event.key == "r":
            state["user_zoomed"] = False
            update()

    fig.canvas.mpl_connect("key_press_event", on_key)

    slider_lam.on_changed(update)
    slider_n.on_changed(update)
    set_combined_reference(step_data)
    update_control_title(step_data)
    refresh_nav_buttons()
    update()
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
