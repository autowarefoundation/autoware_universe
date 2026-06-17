"""Shared CSV loaders for MPPI rollout / temporal plotting tools."""
from __future__ import annotations

import csv
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class ResolvedRolloutSnapshot:
    """File paths for one MPPI rollout dump (per-step folder or flat analysis prefix)."""

    step_dir: Path
    analysis_base: Path | None
    meta_path: Path
    costs_path: Path
    rollouts_path: Path
    combined_path: Path
    controls_path: Path | None
    centerline_path: Path | None
    rollout_root: Path | None
    log_csv: Path | None


def load_meta(path: Path) -> dict[str, float]:
    out: dict[str, float] = {}
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            out[row["key"]] = float(row["value"])
    return out


def load_costs(path: Path) -> dict[str, np.ndarray]:
    idx, raw, unnorm, norm = [], [], [], []
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            idx.append(int(row["rollout_index"]))
            raw.append(float(row["raw_cost"]))
            unnorm.append(float(row["unnormalized_importance"]))
            norm.append(float(row["normalized_weight"]))
    return {
        "index": np.asarray(idx, dtype=int),
        "raw_cost": np.asarray(raw),
        "unnormalized": np.asarray(unnorm),
        "normalized": np.asarray(norm),
    }


def load_rollout_segments(path: Path) -> tuple[list[np.ndarray], np.ndarray, np.ndarray]:
    """Return LineCollection segments, rollout ids, and normalized weight per segment."""
    trajectories = load_rollout_trajectories(path)
    segments: list[np.ndarray] = []
    seg_ids: list[int] = []
    rollout_ids_flat: list[int] = []
    for rid, xy in trajectories.items():
        if xy.shape[0] >= 2:
            segments.append(xy)
            seg_ids.append(int(rid))
        rollout_ids_flat.extend([int(rid)] * xy.shape[0])
    return segments, np.asarray(seg_ids, dtype=int), np.asarray(rollout_ids_flat, dtype=int)


def load_rollout_trajectories(path: Path) -> dict[int, np.ndarray]:
    """Load per-rollout (x, y) polylines keyed by rollout_index."""
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.size == 0:
        return {}
    if data.ndim == 1:
        data = data.reshape(1, -1)
    rollout_ids = data[:, 0].astype(int)
    steps = data[:, 1].astype(int)
    xy = data[:, 2:4]

    out: dict[int, np.ndarray] = {}
    for rid in np.unique(rollout_ids):
        mask = rollout_ids == rid
        order = np.argsort(steps[mask])
        pts = xy[mask][order]
        if len(pts) >= 1:
            out[int(rid)] = pts
    return out


def compute_weighted_xy_trajectory(
    trajectories: dict[int, np.ndarray],
    weights_by_rollout: dict[int, float],
) -> tuple[np.ndarray, np.ndarray]:
    """MPPI-style weighted average of rollout (x, y) paths for lambda retuning visualization."""
    if not trajectories:
        return np.asarray([]), np.asarray([])

    max_len = max(traj.shape[0] for traj in trajectories.values())
    xs = np.full((len(trajectories), max_len), np.nan, dtype=float)
    ys = np.full((len(trajectories), max_len), np.nan, dtype=float)
    ws = np.zeros(len(trajectories), dtype=float)
    for j, (rid, traj) in enumerate(trajectories.items()):
        n = traj.shape[0]
        xs[j, :n] = traj[:, 0]
        ys[j, :n] = traj[:, 1]
        ws[j] = max(weights_by_rollout.get(int(rid), 0.0), 0.0)

    w_sum = float(np.sum(ws))
    if w_sum <= 0.0:
        candidates = [rid for rid in trajectories if weights_by_rollout.get(int(rid), 0.0) > 0.0]
        if not candidates:
            candidates = list(trajectories.keys())
        best_rid = max(candidates, key=lambda rid: weights_by_rollout.get(int(rid), 0.0))
        traj = trajectories[best_rid]
        return traj[:, 0].copy(), traj[:, 1].copy()

    x_out = np.zeros(max_len, dtype=float)
    y_out = np.zeros(max_len, dtype=float)
    for t in range(max_len):
        w_t = ws.copy()
        w_t[np.isnan(xs[:, t])] = 0.0
        denom = float(np.sum(w_t))
        if denom <= 0.0:
            break
        x_out[t] = float(np.nansum(w_t * xs[:, t]) / denom)
        y_out[t] = float(np.nansum(w_t * ys[:, t]) / denom)
    valid = ~(np.isnan(x_out) | np.isnan(y_out))
    return x_out[valid], y_out[valid]


def load_rollout_controls(path: Path) -> dict[int, tuple[np.ndarray, np.ndarray]]:
    """Load per-rollout control sequences keyed by rollout_index."""
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.size == 0:
        return {}
    if data.ndim == 1:
        data = data.reshape(1, -1)
    rollout_ids = data[:, 0].astype(int)
    steps = data[:, 1].astype(int)
    u_accel = data[:, 2]
    u_steer = data[:, 3]

    out: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    for rid in np.unique(rollout_ids):
        mask = rollout_ids == rid
        order = np.argsort(steps[mask])
        out[int(rid)] = (u_accel[mask][order], u_steer[mask][order])
    return out


def compute_weighted_control_trajectory(
    controls: dict[int, tuple[np.ndarray, np.ndarray]],
    weights_by_rollout: dict[int, float],
    dt: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Weighted average of sampled control sequences (Σ wᵢ·uᵢ), aligned with combined.csv time axis."""
    if not controls or dt <= 0.0:
        return np.asarray([]), np.asarray([]), np.asarray([])

    max_len = max(len(u_a) for u_a, _ in controls.values())
    ua = np.full((len(controls), max_len), np.nan, dtype=float)
    us = np.full((len(controls), max_len), np.nan, dtype=float)
    ws = np.zeros(len(controls), dtype=float)
    for j, (rid, (u_a, u_s)) in enumerate(controls.items()):
        n = len(u_a)
        ua[j, :n] = u_a
        us[j, :n] = u_s
        ws[j] = max(weights_by_rollout.get(int(rid), 0.0), 0.0)

    w_sum = float(np.sum(ws))
    if w_sum <= 0.0:
        candidates = [rid for rid in controls if weights_by_rollout.get(int(rid), 0.0) > 0.0]
        if not candidates:
            candidates = list(controls.keys())
        best_rid = max(candidates, key=lambda rid: weights_by_rollout.get(int(rid), 0.0))
        u_a, u_s = controls[best_rid]
        t = (np.arange(len(u_a), dtype=float) + 1.0) * dt
        return t, u_a.copy(), u_s.copy()

    u_a_out = np.zeros(max_len, dtype=float)
    u_s_out = np.zeros(max_len, dtype=float)
    for t in range(max_len):
        w_t = ws.copy()
        w_t[np.isnan(ua[:, t])] = 0.0
        denom = float(np.sum(w_t))
        if denom <= 0.0:
            break
        u_a_out[t] = float(np.nansum(w_t * ua[:, t]) / denom)
        u_s_out[t] = float(np.nansum(w_t * us[:, t]) / denom)
    valid = ~(np.isnan(u_a_out) | np.isnan(u_s_out))
    t_out = (np.arange(max_len, dtype=float)[valid] + 1.0) * dt
    return t_out, u_a_out[valid], u_s_out[valid]


def load_combined(path: Path) -> dict[str, np.ndarray]:
    cols: dict[str, list[float]] = {
        k: [] for k in ("step", "t", "x", "y", "yaw", "vel", "steer", "u_accel", "u_steer")
    }
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            cols["step"].append(int(row["step"]))
            cols["t"].append(float(row["t"]))
            cols["x"].append(float(row["x"]))
            cols["y"].append(float(row["y"]))
            cols["yaw"].append(float(row["yaw"]))
            cols["vel"].append(float(row["vel_x"]))
            cols["steer"].append(float(row["steer"]))
            cols["u_accel"].append(float(row["u_accel"]))
            cols["u_steer"].append(float(row["u_steer"]))
    return {k: np.asarray(v) for k, v in cols.items()}


def load_centerline(path: Path) -> tuple[np.ndarray | None, np.ndarray | None]:
    if not path.is_file():
        return None, None
    cxy = np.loadtxt(path, delimiter=",", skiprows=1)
    if cxy.ndim == 1:
        cxy = cxy.reshape(1, -1)
    return cxy[:, 0], cxy[:, 1]


def load_boundary_limits(
    meta: dict[str, float],
    default_half_width: float = 0.8,
    log_hint: str | Path | None = None,
) -> tuple[float, float]:
    """Return (left_half, right_half) off-road limits in meters (path-left = +left)."""
    left = meta.get("boundary_threshold_left", -1.0)
    right = meta.get("boundary_threshold_right", -1.0)
    sym = meta.get("boundary_threshold", default_half_width)
    if left >= 0.0 and right >= 0.0:
        return float(left), float(right)
    if left >= 0.0:
        return float(left), float(sym if sym > 0.0 else left)
    if right >= 0.0:
        return float(sym if sym > 0.0 else right), float(right)
    if log_hint is not None:
        hint = str(log_hint).lower()
        if "two_lane_double_park" in hint:
            # Matches first_order_dubins_two_lane_double_park_example.cu cost limits.
            return 0.85, 2.14
    return float(sym), float(sym)


def draw_road_boundaries(
    ax,
    cpx: np.ndarray,
    cpy: np.ndarray,
    left_half: float,
    right_half: float | None = None,
    *,
    color: str = "#2858b0",
    alpha: float = 0.85,
    linewidth: float = 1.6,
    fill_alpha: float = 0.08,
) -> None:
    """Draw cost off-road edges offset from a polyline centerline."""
    if cpx is None or cpy is None or len(cpx) < 2:
        return
    if right_half is None:
        right_half = left_half
    if left_half <= 0.0 or right_half <= 0.0:
        return

    dx = np.diff(cpx)
    dy = np.diff(cpy)
    seg_len = np.hypot(dx, dy)
    seg_len = np.maximum(seg_len, 1e-9)
    tx = dx / seg_len
    ty = dy / seg_len
    px = np.empty(len(cpx))
    py = np.empty(len(cpx))
    px[0] = tx[0]
    py[0] = ty[0]
    px[1:] = tx
    py[1:] = ty

    nx = -py
    ny = px
    left_x = cpx + left_half * nx
    left_y = cpy + left_half * ny
    right_x = cpx - right_half * nx
    right_y = cpy - right_half * ny

    ax.fill(
        np.concatenate([left_x, right_x[::-1]]),
        np.concatenate([left_y, right_y[::-1]]),
        color=color,
        alpha=fill_alpha,
        linewidth=0,
        zorder=0,
    )
    ax.plot(left_x, left_y, color=color, alpha=alpha, linewidth=linewidth, linestyle="-", zorder=1, label="road boundary")
    ax.plot(right_x, right_y, color=color, alpha=alpha, linewidth=linewidth, linestyle="-", zorder=1)


def load_steps_index(path: Path) -> list[tuple[int, float, str]]:
    rows: list[tuple[int, float, str]] = []
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            rows.append((int(row["step"]), float(row["sim_time"]), row["prefix"]))
    return rows


def resolve_step_prefix(log_or_rollout_dir: Path, step: int) -> Path:
    """Resolve a step directory from a temporal log path or rollouts root."""
    if log_or_rollout_dir.is_dir():
        rollout_root = log_or_rollout_dir
    else:
        stem = log_or_rollout_dir.with_suffix("") if log_or_rollout_dir.suffix == ".csv" else log_or_rollout_dir
        rollout_root = Path(str(stem) + "_rollouts")

    index_path = rollout_root / "steps_index.csv"
    if index_path.is_file():
        for s, _t, prefix in load_steps_index(index_path):
            if s == step:
                return Path(prefix)
    return rollout_root / f"step_{step:06d}"


def is_flat_analysis_prefix(path: Path) -> bool:
    """True when *path* names a single-iteration dump (``prefix_meta.csv``, etc.)."""
    base = path.with_suffix("") if path.suffix == ".csv" else path
    return Path(str(base) + "_meta.csv").is_file()


def _analysis_file_paths(base: Path) -> tuple[Path, Path, Path, Path, Path | None]:
    controls = Path(str(base) + "_rollouts_controls.csv")
    return (
        Path(str(base) + "_meta.csv"),
        Path(str(base) + "_costs.csv"),
        Path(str(base) + "_rollouts_xy.csv"),
        Path(str(base) + "_combined.csv"),
        controls if controls.is_file() else None,
    )


def resolve_centerline_for_snapshot(
    step_dir: Path,
    analysis_base: Path | None = None,
    log_csv: Path | None = None,
) -> Path | None:
    candidates: list[Path] = []
    if analysis_base is not None:
        candidates.append(analysis_base.with_name(analysis_base.name + "_centerline.csv"))
    if log_csv is not None:
        candidates.append(log_csv.with_name(log_csv.stem + "_centerline.csv"))
    if step_dir.name.startswith("step_") and step_dir.parent.name.endswith("_rollouts"):
        stem = step_dir.parent.name[: -len("_rollouts")]
        candidates.append(step_dir.parent.parent / f"{stem}_centerline.csv")
    if "_rollouts" in step_dir.as_posix():
        candidates.append(Path(str(step_dir).rsplit("_rollouts", 1)[0] + "_centerline.csv"))
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def resolve_rollout_snapshot(path: Path, step: int = 0) -> ResolvedRolloutSnapshot:
    """Resolve rollout CSV paths from any supported input.

    Accepts:
      - flat analysis prefix (``dubins_circle_mppi_rollout_analysis``)
      - closed-loop temporal log (``*_log.csv``) + *step*
      - rollouts root (``*_log_rollouts/``) + *step*
      - per-step directory (``.../step_000042/``)
    """
    path = Path(path)
    log_csv: Path | None = None
    analysis_base: Path | None = None

    if path.is_dir() and (path / "costs.csv").is_file():
        step_dir = path.resolve()
    elif path.is_dir() and path.name.startswith("step_"):
        step_dir = path.resolve()
    elif is_flat_analysis_prefix(path):
        analysis_base = path.with_suffix("") if path.suffix == ".csv" else path
        step_dir = analysis_base.resolve()
    elif path.suffix == ".csv":
        log_csv = path.resolve()
        step_dir = resolve_step_prefix(path, step).resolve()
    elif path.is_dir():
        step_dir = resolve_step_prefix(path, step).resolve()
    else:
        step_dir = resolve_step_prefix(path, step).resolve()

    if analysis_base is None and is_flat_analysis_prefix(step_dir):
        analysis_base = step_dir

    rollout_root: Path | None = None
    if analysis_base is not None:
        meta_path, costs_path, rollouts_path, combined_path, controls_path = _analysis_file_paths(analysis_base)
    else:
        meta_path = step_dir / "meta.csv"
        costs_path = step_dir / "costs.csv"
        rollouts_path = step_dir / "rollouts_xy.csv"
        combined_path = step_dir / "combined.csv"
        controls_path = step_dir / "rollouts_controls.csv"
        if not controls_path.is_file():
            controls_path = None
        if step_dir.name.startswith("step_"):
            rollout_root = step_dir.parent.resolve()

    centerline_path = resolve_centerline_for_snapshot(step_dir, analysis_base, log_csv)

    return ResolvedRolloutSnapshot(
        step_dir=step_dir,
        analysis_base=analysis_base,
        meta_path=meta_path,
        costs_path=costs_path,
        rollouts_path=rollouts_path,
        combined_path=combined_path,
        controls_path=controls_path,
        centerline_path=centerline_path,
        rollout_root=rollout_root,
        log_csv=log_csv,
    )


def list_rollout_steps(rollout_root: Path) -> list[tuple[int, float, Path]]:
    """Return (step, sim_time, directory) for every dumped step under *rollout_root*."""
    index_path = rollout_root / "steps_index.csv"
    if index_path.is_file():
        return [
            (step, sim_time, Path(prefix).resolve())
            for step, sim_time, prefix in load_steps_index(index_path)
        ]
    steps: list[tuple[int, float, Path]] = []
    for step_dir in sorted(rollout_root.glob("step_*")):
        if not step_dir.is_dir() or not (step_dir / "costs.csv").is_file():
            continue
        step_num = int(step_dir.name.split("_", 1)[1])
        sim_time = float("nan")
        meta_path = step_dir / "meta.csv"
        if meta_path.is_file():
            meta = load_meta(meta_path)
            sim_time = float(meta.get("sim_time", step_num * meta.get("dt", 0.1)))
        steps.append((step_num, sim_time, step_dir.resolve()))
    return steps


def weights_for_rollout_ids(costs: dict[str, np.ndarray], seg_ids: np.ndarray) -> np.ndarray:
    lookup = {int(i): float(w) for i, w in zip(costs["index"], costs["normalized"])}
    return np.asarray([lookup.get(int(rid), 0.0) for rid in seg_ids], dtype=float)


def weight_to_purple_green(
    weights: np.ndarray,
    *,
    alpha_min: float = 0.07,
    alpha_max: float = 0.98,
    alpha_gamma: float = 2.0,
) -> np.ndarray:
    """Map normalized weights to RGBA colors: purple (low) -> green (high).

    Low-weight rollouts are drawn more transparent so high-weight trajectories stay
    visible when many paths overlap. Use sort_rollouts_for_draw() so heavy lines
    render on top.
    """
    w = np.asarray(weights, dtype=float)
    if w.size == 0:
        return w.reshape(0, 4)
    w_min = float(np.min(w))
    w_max = float(np.max(w))
    span = max(w_max - w_min, 1e-12)
    t = np.clip((w - w_min) / span, 0.0, 1.0)
    purple = np.array([0.50, 0.00, 0.55, 1.0])
    green = np.array([0.00, 0.78, 0.20, 1.0])
    colors = purple[None, :] + t[:, None] * (green - purple)[None, :]
    alpha_t = t ** alpha_gamma
    colors[:, 3] = alpha_min + alpha_t * (alpha_max - alpha_min)
    return colors


def weight_to_rollout_linewidths(
    weights: np.ndarray,
    *,
    width_min: float = 0.35,
    width_max: float = 1.5,
) -> np.ndarray:
    """Per-segment line widths scaled by relative weight."""
    w = np.asarray(weights, dtype=float)
    if w.size == 0:
        return w
    w_min = float(np.min(w))
    w_max = float(np.max(w))
    span = max(w_max - w_min, 1e-12)
    t = np.clip((w - w_min) / span, 0.0, 1.0)
    return width_min + t * (width_max - width_min)


def sort_rollouts_for_draw(
    segments: list[np.ndarray],
    weights: np.ndarray,
    ids: np.ndarray | None = None,
) -> tuple[list[np.ndarray], np.ndarray] | tuple[list[np.ndarray], np.ndarray, np.ndarray]:
    """Sort rollouts ascending by weight so high-weight paths draw on top."""
    order = np.argsort(weights)
    sorted_segments = [segments[int(i)] for i in order]
    sorted_weights = weights[order]
    if ids is None:
        return sorted_segments, sorted_weights
    sorted_ids = ids[order]
    return sorted_segments, sorted_weights, sorted_ids


def recompute_weights(raw_cost: np.ndarray, baseline: float, lambda_val: float) -> tuple[np.ndarray, np.ndarray, float]:
    w = np.exp(-(raw_cost - baseline) / max(lambda_val, 1e-12))
    w = np.where(np.isfinite(w), w, 0.0)
    normalizer = float(np.sum(w))
    if normalizer <= 0.0:
        norm = np.zeros_like(w)
    else:
        norm = w / normalizer
    return w, norm, normalizer


def _axis_uses_equal_aspect(ax) -> bool:
    aspect = ax.get_aspect()
    if aspect == "equal":
        return True
    if isinstance(aspect, str):
        return aspect.startswith("equal")
    return False


def _zoom_axis_at_cursor(ax, xdata: float, ydata: float, scale_factor: float) -> None:
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    relx = (x1 - xdata) / max(x1 - x0, 1e-12)
    rely = (y1 - ydata) / max(y1 - y0, 1e-12)

    if _axis_uses_equal_aspect(ax):
        span = max(x1 - x0, y1 - y0, 1e-12)
        new_span = max(span * scale_factor, 1e-9)
        ax.set_xlim(xdata - new_span * relx, xdata + new_span * (1.0 - relx))
        ax.set_ylim(ydata - new_span * rely, ydata + new_span * (1.0 - rely))
        return

    new_xspan = max((x1 - x0) * scale_factor, 1e-9)
    new_yspan = max((y1 - y0) * scale_factor, 1e-9)
    ax.set_xlim(xdata - new_xspan * relx, xdata + new_xspan * (1.0 - relx))
    ax.set_ylim(ydata - new_yspan * rely, ydata + new_yspan * (1.0 - rely))


def enable_plot_navigation(
    fig,
    axes: list | None = None,
    *,
    base_scale: float = 1.15,
    on_user_nav: Callable[[object], None] | None = None,
) -> None:
    """Scroll to zoom and left-click drag to pan on the given axes.

    Widget axes (sliders, buttons) are skipped automatically when *axes* is None
    by ignoring axes shorter than 6% of the figure height.
    """
    nav_axes = axes
    if nav_axes is None:
        nav_axes = [ax for ax in fig.axes if ax.get_position().height > 0.06]
    nav_axes = list(nav_axes)

    pan_state: dict[str, object | None] = {
        "active": False,
        "ax": None,
        "xpress": None,
        "ypress": None,
        "xlim": None,
        "ylim": None,
    }

    def notify(ax) -> None:
        if on_user_nav is not None:
            on_user_nav(ax)

    def on_scroll(event) -> None:
        if event.inaxes is None or event.inaxes not in nav_axes:
            return
        if event.xdata is None or event.ydata is None:
            return
        if event.button == "up":
            scale = 1.0 / base_scale
        elif event.button == "down":
            scale = base_scale
        else:
            return
        _zoom_axis_at_cursor(event.inaxes, float(event.xdata), float(event.ydata), scale)
        notify(event.inaxes)
        fig.canvas.draw_idle()

    def on_press(event) -> None:
        if event.inaxes is None or event.inaxes not in nav_axes or event.button != 1:
            return
        if event.xdata is None or event.ydata is None:
            return
        pan_state["active"] = True
        pan_state["ax"] = event.inaxes
        pan_state["xpress"] = float(event.xdata)
        pan_state["ypress"] = float(event.ydata)
        pan_state["xlim"] = event.inaxes.get_xlim()
        pan_state["ylim"] = event.inaxes.get_ylim()

    def on_motion(event) -> None:
        if not pan_state["active"]:
            return
        ax = pan_state["ax"]
        if ax is None:
            return
        xpress = pan_state["xpress"]
        ypress = pan_state["ypress"]
        xlim0 = pan_state["xlim"]
        ylim0 = pan_state["ylim"]
        if xpress is None or ypress is None or xlim0 is None or ylim0 is None:
            return

        xdata = event.xdata
        ydata = event.ydata
        if xdata is None or ydata is None:
            if event.inaxes is ax:
                return
            # Allow panning to continue when the cursor leaves the axes briefly.
            inv = ax.transData.inverted()
            xdata, ydata = inv.transform((event.x, event.y))

        dx = float(xdata) - float(xpress)
        dy = float(ydata) - float(ypress)
        ax.set_xlim(float(xlim0[0]) - dx, float(xlim0[1]) - dx)
        ax.set_ylim(float(ylim0[0]) - dy, float(ylim0[1]) - dy)
        notify(ax)
        fig.canvas.draw_idle()

    def on_release(event) -> None:
        if event.button == 1:
            pan_state["active"] = False
            pan_state["ax"] = None

    fig.canvas.mpl_connect("scroll_event", on_scroll)
    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("button_release_event", on_release)


def enable_scroll_zoom(
    fig,
    axes: list | None = None,
    *,
    base_scale: float = 1.15,
    on_user_zoom: Callable[[object], None] | None = None,
) -> None:
    """Enable scroll zoom and click-drag pan (alias for enable_plot_navigation)."""
    enable_plot_navigation(fig, axes, base_scale=base_scale, on_user_nav=on_user_zoom)
