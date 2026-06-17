#!/usr/bin/env python3
# Plots: (1) x-y path with reference, (2) key states vs time, (3) controls, (4) arc length / lateral, (5) baseline.
# Requires: numpy, matplotlib
#
# Supported CSV logs (header row required for dubins format; legacy logs still work by column count):
#   dubins_path_tracking_example.cu         — straight line
#   dubins_circle_path_tracking_example.cu  — closed circle
#   Columns: u_accel, u_steer, arc_s, lat_err (+ *_centerline.csv)
#   Legacy RacerDubins examples (if present) — throttle/steer, optional path_u_*, arc_s
#
# Usage:
#   python3 scripts/mppi/plot_racer_dubins_temporal_mppi.py dubins_path_tracking_log.csv
#   python3 scripts/mppi/plot_racer_dubins_temporal_mppi.py first_order_dubins_two_lane_double_park_log.csv
#   python3 scripts/mppi/plot_racer_dubins_temporal_mppi.py log.csv -o viz.png
#   python3 scripts/mppi/plot_racer_dubins_temporal_mppi.py log.csv --no-show
#
# For per-step MPPI rollout clouds (not full-run time series), use:
#   plot_mppi_rollout_analysis.py  or  plot_mppi_rollouts_at_step.py  on the same log + --step N

import argparse
import csv
import os
import sys
from pathlib import Path

import numpy as np


def perimeter_from_centerline(cpx: np.ndarray, cpy: np.ndarray) -> float:
    dx = np.diff(cpx)
    dy = np.diff(cpy)
    return float(np.sum(np.sqrt(dx * dx + dy * dy)))


def arc_s_from_xy_on_polyline(px: np.ndarray, py: np.ndarray, cpx: np.ndarray, cpy: np.ndarray) -> np.ndarray:
    """Closest-point arc length on a closed polyline."""
    s_vert = np.zeros(len(cpx))
    s_vert[1:] = np.cumsum(np.sqrt(np.diff(cpx) ** 2 + np.diff(cpy) ** 2))
    perimeter = s_vert[-1]
    out = np.zeros(len(px))
    for i, (x, y) in enumerate(zip(px, py)):
        best_d2 = np.inf
        best_s = 0.0
        for e in range(len(cpx) - 1):
            ax, ay = cpx[e], cpy[e]
            bx, by = cpx[e + 1], cpy[e + 1]
            ex, ey = bx - ax, by - ay
            elen2 = ex * ex + ey * ey
            t = 0.0 if elen2 < 1e-12 else np.clip(((x - ax) * ex + (y - ay) * ey) / elen2, 0.0, 1.0)
            qx, qy = ax + t * ex, ay + t * ey
            d2 = (x - qx) ** 2 + (y - qy) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_s = s_vert[e] + t * (s_vert[e + 1] - s_vert[e])
        out[i] = best_s % perimeter if perimeter > 0 else 0.0
    return out


def display_available() -> bool:
    if sys.platform == "darwin":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def load_log(csv_path: Path) -> dict:
    """Load CSV with optional header; return column arrays and metadata."""
    with csv_path.open(newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        raw_rows = [row for row in reader if row]

    names = [h.strip() for h in header]
    name_to_idx = {n: i for i, n in enumerate(names)}

    # Keep only rows that are all numeric (skip debug text accidentally written to CSV).
    numeric_rows: list[list[float]] = []
    for row in raw_rows:
        try:
            numeric_rows.append([float(x) for x in row])
        except ValueError:
            continue
    if not numeric_rows:
        raise ValueError(f"no numeric data rows in {csv_path}")

    ncol = len(numeric_rows[0])
    if any(len(r) != ncol for r in numeric_rows):
        raise ValueError(f"inconsistent column count in {csv_path} (expected {ncol})")
    data = np.array(numeric_rows, dtype=float)

    def by_idx(i: int) -> np.ndarray:
        if i < 0 or i >= ncol:
            raise IndexError(f"column index {i} out of range for {ncol} columns")
        return data[:, i]

    def col(name: str, fallback: int | None) -> np.ndarray | None:
        if name in name_to_idx and name_to_idx[name] < ncol:
            return data[:, name_to_idx[name]]
        if fallback is not None and 0 <= fallback < ncol:
            return data[:, fallback]
        return None

    out: dict = {
        "names": names,
        "ncol": ncol,
        "t": by_idx(0),
        "nominal_cost": None,
        "mppi_applied_cost": None,
    }

    # Dubins bicycle: u_accel/u_steer (not throttle). Header may list 18 names while
    # rows only carry 14 fields (compact stadium logger).
    is_dubins = (
        ncol in (14, 18, 19, 20)
        and "u_throttle" not in name_to_idx
        and (col("u_accel", 7) is not None or ncol == 14)
    )
    out["is_dubins"] = is_dubins
    out["px"] = col("pos_x", 1) if col("pos_x", 1) is not None else by_idx(1)
    out["py"] = col("pos_y", 2) if col("pos_y", 2) is not None else by_idx(2)
    out["yaw"] = col("yaw", 3) if col("yaw", 3) is not None else by_idx(3)
    out["vx"] = col("vel_x", 4) if col("vel_x", 4) is not None else by_idx(4)
    out["steer"] = col("steer_angle", 5) if col("steer_angle", 5) is not None else by_idx(5)

    if is_dubins and ncol == 14:
        # Compact layout from dubins_stadium_path_tracking_example (no nom_u / ref_x,y in row).
        out["brake"] = by_idx(6)
        out["u0"] = by_idx(7)
        out["u1"] = by_idx(8)
        out["nom_u0"] = out["nom_u1"] = None
        out["path_u0"] = out["path_u1"] = None
        out["rpx"] = out["rpy"] = None
        out["ryaw"] = by_idx(9)
        out["ref_v"] = by_idx(10)
        out["arc_s"] = by_idx(11)
        out["lat_err"] = by_idx(12)
        out["nominal_cost"] = out["mppi_applied_cost"] = None
        out["baseline"] = by_idx(13)
        out["u0_label"] = "u accel [m/s²]"
        out["u1_label"] = "u steer [rad]"
    elif is_dubins:
        out["brake"] = col("brake_state", 6) if col("brake_state", 6) is not None else np.zeros_like(out["t"])
        out["u0"] = col("u_accel", 7) if col("u_accel", 7) is not None else by_idx(7)
        out["u1"] = col("u_steer", 8) if col("u_steer", 8) is not None else by_idx(8)
        out["nom_u0"] = col("nom_u_accel", 9)
        out["nom_u1"] = col("nom_u_steer", 10)
        out["path_u0"] = out["path_u1"] = None
        out["rpx"] = col("ref_x", 11)
        out["rpy"] = col("ref_y", 12)
        out["ryaw"] = col("ref_yaw", 13)
        out["ref_v_pose"] = col("ref_v_pose", 14)
        if out["ref_v_pose"] is None:
            out["ref_v_pose"] = col("ref_v", 14)
        out["ref_v_target"] = col("ref_v_target", 15)
        out["ref_v"] = out["ref_v_pose"]
        out["arc_s"] = col("arc_s", 16 if out["ref_v_target"] is not None else 15)
        out["lat_err"] = col("lat_err", 17 if out["ref_v_target"] is not None else 16)
        out["nominal_cost"] = col("nominal_cost", 17)
        out["mppi_applied_cost"] = col("mppi_applied_cost", 18)
        out["baseline"] = col("baseline", 18 if out["ref_v_target"] is not None else 17)
        if out["baseline"] is None:
            out["baseline"] = col("baseline", 19)
        out["u0_label"] = "u accel [m/s²]"
        out["u1_label"] = "u steer [rad]"
    elif ncol >= 19:
        out["brake"] = col("brake_state", 6)
        out["u0"] = col("u_throttle", 7)
        out["u1"] = col("u_steer", 8)
        out["nom_u0"] = col("nom_u_throttle", 9)
        out["nom_u1"] = col("nom_u_steer", 10)
        out["path_u0"] = col("path_u_throttle", 11)
        out["path_u1"] = col("path_u_steer", 12)
        out["rpx"] = col("ref_x", 13)
        out["rpy"] = col("ref_y", 14)
        out["ryaw"] = col("ref_yaw", 15)
        out["ref_v"] = col("ref_v", 16)
        out["arc_s"] = col("arc_s", 17)
        out["lat_err"] = None
        out["baseline"] = col("baseline", 18)
        out["u0_label"] = "u throttle (applied)"
        out["u1_label"] = "u steer (applied)"
    elif ncol >= 18:
        out["brake"] = col("brake_state", 6)
        out["u0"] = col("u_throttle", 7)
        out["u1"] = col("u_steer", 8)
        out["nom_u0"] = col("nom_u_throttle", 9)
        out["nom_u1"] = col("nom_u_steer", 10)
        out["path_u0"] = col("path_u_throttle", 11)
        out["path_u1"] = col("path_u_steer", 12)
        out["rpx"] = col("ref_x", 13)
        out["rpy"] = col("ref_y", 14)
        out["ryaw"] = col("ref_yaw", 15)
        out["ref_v"] = col("ref_v", 16)
        out["arc_s"] = None
        out["lat_err"] = None
        out["baseline"] = col("baseline", 17)
        out["u0_label"] = "u throttle (applied)"
        out["u1_label"] = "u steer (applied)"
    else:
        out["brake"] = col("brake_state", 6) if ncol > 6 else np.zeros_like(out["t"])
        out["u0"] = col("u_throttle", 7)
        out["u1"] = col("u_steer", 8)
        out["nom_u0"] = out["nom_u1"] = out["path_u0"] = out["path_u1"] = None
        out["rpx"] = col("ref_x", 9)
        out["rpy"] = col("ref_y", 10)
        out["ryaw"] = col("ref_yaw", 11)
        out["ref_v"] = col("ref_v", 12)
        out["arc_s"] = out["lat_err"] = None
        out["baseline"] = col("baseline", 13)
        out["u0_label"] = "u0"
        out["u1_label"] = "u1"

    if out.get("ref_v_target") is None:
        out["ref_v_target"] = col("ref_v_target")
    if out.get("ref_v_pose") is None:
        out["ref_v_pose"] = col("ref_v_pose")
        if out["ref_v_pose"] is None:
            out["ref_v_pose"] = out.get("ref_v")
    if out.get("ref_v") is None and out.get("ref_v_pose") is not None:
        out["ref_v"] = out["ref_v_pose"]

    return out


def main() -> int:
    p = argparse.ArgumentParser(
        description="Visualize Dubins path-tracking or legacy RacerDubins temporal MPPI CSV logs.",
    )
    p.add_argument("csv", type=Path, help="Path to exported CSV from example binary")
    p.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Save main figure PNG (default: <csv_stem>_viz.png). Use '-' for display only.",
    )
    p.add_argument("--dpi", type=int, default=150, help="Figure DPI when saving (default: 150)")
    p.add_argument("--show", action="store_true", help="Open interactive plot windows")
    p.add_argument("--no-show", action="store_true", help="Do not open windows; only write PNG files")
    args = p.parse_args()
    if not args.csv.is_file():
        print(f"error: not a file: {args.csv}", file=sys.stderr)
        return 1

    want_show = args.show or (not args.no_show and display_available())
    save_files = args.output != Path("-")
    if args.output is None:
        save_files = True

    try:
        import matplotlib

        if not want_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as e:
        print("error: need matplotlib. Install with: pip install matplotlib", file=sys.stderr)
        print(e, file=sys.stderr)
        return 1

    log = load_log(args.csv)
    t = log["t"]
    px, py, yaw, vx = log["px"], log["py"], log["yaw"], log["vx"]
    steer, brake = log["steer"], log["brake"]
    u0, u1 = log["u0"], log["u1"]
    nom_u0, nom_u1 = log["nom_u0"], log["nom_u1"]
    path_u0, path_u1 = log["path_u0"], log["path_u1"]
    rpx, rpy, ryaw = log["rpx"], log["rpy"], log["ryaw"]
    ref_v_pose = log.get("ref_v_pose")
    if ref_v_pose is None:
        ref_v_pose = log.get("ref_v")
    ref_v_target = log.get("ref_v_target")
    arc_s = log["arc_s"]
    lat_err = log["lat_err"]
    nominal_cost = log["nominal_cost"]
    mppi_applied_cost = log["mppi_applied_cost"]
    baseline = log["baseline"]
    is_dubins = log["is_dubins"]
    u0_label, u1_label = log["u0_label"], log["u1_label"]

    out_png = args.csv.with_name(args.csv.stem + "_viz.png")
    if args.output is not None:
        out_png = args.output

    centerline_path = args.csv.with_name(args.csv.stem + "_centerline.csv")
    have_cl = centerline_path.is_file()
    cpx = cpy = None
    perimeter = None
    if have_cl:
        cxy = np.loadtxt(centerline_path, delimiter=",", skiprows=1)
        if cxy.ndim == 1:
            cxy = cxy.reshape(1, -1)
        cpx, cpy = cxy[:, 0], cxy[:, 1]
        perimeter = perimeter_from_centerline(cpx, cpy)

    if arc_s is None and have_cl and cpx is not None:
        arc_s = arc_s_from_xy_on_polyline(px, py, cpx, cpy)
        print(f"note: no arc_s in log; estimated from {centerline_path.name}", file=sys.stderr)

    title_suffix = " (Dubins bicycle)" if is_dubins else ""
    n_progress_rows = 2 if lat_err is not None else 1
    has_nom_compare = nom_u0 is not None and nom_u1 is not None
    if not has_nom_compare and is_dubins:
        print(
            "note: log has no nom_u_accel / nom_u_steer columns; re-run example after rebuilding "
            "so CSV rows match the 18-column header.",
            file=sys.stderr,
        )
    n_nom_rows = 2 if has_nom_compare else 0
    height_ratios = [1.2, 1, 1, 1] + [0.75] * n_progress_rows
    if has_nom_compare:
        height_ratios += [1.05, 0.85]  # MPPI vs nominal overlay, then |delta|
    fig = plt.figure(
        figsize=(12, 11 + 0.8 * (n_progress_rows - 1) + 1.7 * n_nom_rows),
        constrained_layout=True,
    )
    gs = fig.add_gridspec(
        4 + n_progress_rows + n_nom_rows,
        2,
        height_ratios=height_ratios,
        width_ratios=[1, 1],
    )

    ax_xy = fig.add_subplot(gs[0, :])
    sc = ax_xy.scatter(px, py, c=t, s=10, cmap="viridis", label="position (time → color)")
    fig.colorbar(sc, ax=ax_xy, label="time [s]", shrink=0.8)
    ax_xy.plot(px, py, "k-", linewidth=0.7, alpha=0.4, label="vehicle path")
    if have_cl and cpx is not None:
        ax_xy.plot(cpx, cpy, "r-", linewidth=1.4, label="ref centerline", alpha=0.95, zorder=2)
    if rpx is not None and rpy is not None:
        ax_xy.plot(rpx, rpy, "r--", linewidth=1.1, label="ref (horizon t=0)", alpha=0.65)
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title(f"Plan view{title_suffix}")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(loc="best")

    for ax, yv, name, c in [
        (fig.add_subplot(gs[1, 0]), px, "pos x [m]", "C0"),
        (fig.add_subplot(gs[1, 1]), py, "pos y [m]", "C0"),
        (fig.add_subplot(gs[2, 0]), yaw, "yaw [rad]", "C1"),
        (fig.add_subplot(gs[2, 1]), vx, "vel x [m/s]", "C2"),
    ]:
        ax.plot(t, yv, color=c, linewidth=1.2)
        if name == "pos x [m]" and rpx is not None:
            ax.plot(t, rpx, "r--", alpha=0.6, label="ref")
        elif name == "pos y [m]" and rpy is not None:
            ax.plot(t, rpy, "r--", alpha=0.6, label="ref")
        elif name == "yaw [rad]" and ryaw is not None:
            ax.plot(t, ryaw, "r--", alpha=0.6, label="ref")
        elif name == "vel x [m/s]":
            if ref_v_target is not None:
                ax.plot(t, ref_v_target, "r--", alpha=0.85, linewidth=1.4, label="ref speed target (speedAt)")
            if ref_v_pose is not None:
                ax.plot(
                    t,
                    ref_v_pose,
                    color="C3",
                    linestyle=":",
                    alpha=0.55,
                    linewidth=1.0,
                    label="ref pose v (t=0)",
                )
        ax.set_ylabel(name)
        ax.set_xlabel("t [s]")
        ax.grid(True, alpha=0.3)
        if name.startswith("pos") or name in ("yaw [rad]", "vel x [m/s]"):
            ax.legend(loc="best", fontsize=7)

    ax_st = fig.add_subplot(gs[3, 0])
    ax_st.plot(t, steer, color="C3", label="steer angle [state]")
    ax_st.set_ylabel("steer angle (state)")
    ax_st.set_xlabel("t [s]")
    ax_st.grid(True, alpha=0.3)
    ax_st.legend(loc="best", fontsize=7)

    ax_br = fig.add_subplot(gs[3, 1])
    mppi_u0_lbl = f"MPPI applied ({u0_label})" if has_nom_compare else u0_label
    mppi_u1_lbl = f"MPPI applied ({u1_label})" if has_nom_compare else u1_label
    ax_br.plot(t, u0, label=mppi_u0_lbl, color="C4", linewidth=1.2)
    ax_br.plot(t, u1, label=mppi_u1_lbl, color="C5", alpha=0.85, linewidth=1.2)
    if has_nom_compare:
        ax_br.plot(
            t,
            nom_u0,
            "--",
            color="C4",
            linewidth=1.4,
            alpha=0.9,
            label="feedforward nominal (u_nom)",
        )
        ax_br.plot(
            t,
            nom_u1,
            "--",
            color="C5",
            linewidth=1.4,
            alpha=0.9,
            label="feedforward nominal steer (u_nom)",
        )
    if path_u0 is not None:
        ax_br.plot(t, path_u0, ":", color="C4", linewidth=1.0, alpha=0.65, label="path u throttle")
        ax_br.plot(t, path_u1, ":", color="C5", linewidth=1.0, alpha=0.65, label="path u steer")
    if not is_dubins and np.any(brake != 0):
        ax_br.plot(t, brake, ":", color="0.35", linewidth=1.0, label="brake state")
    ax_br.set_ylabel("controls")
    ax_br.set_xlabel("t [s]")
    ax_br.set_title("Applied controls (overview)" if has_nom_compare else None)
    ax_br.legend(loc="best", fontsize=6)
    ax_br.grid(True, alpha=0.3)

    row_after_controls = 4
    if has_nom_compare:
        row_cmp = row_after_controls
        du0 = u0 - nom_u0
        du1 = u1 - nom_u1
        tol = 1.0e-6
        n_equal = int(np.sum((np.abs(du0) < tol) & (np.abs(du1) < tol)))
        print(
            f"MPPI vs feedforward nominal: max |Δaccel|={np.max(np.abs(du0)):.6g}  "
            f"max |Δsteer|={np.max(np.abs(du1)):.6g}  "
            f"RMS Δaccel={np.sqrt(np.mean(du0**2)):.6g}  RMS Δsteer={np.sqrt(np.mean(du1**2)):.6g}  "
            f"steps with u≈u_nom (both channels): {n_equal}/{len(t)}",
            file=sys.stderr,
        )

        ax_u0 = fig.add_subplot(gs[row_cmp, 0])
        ax_u0.plot(t, u0, "-", color="C4", linewidth=1.3, label="MPPI applied")
        ax_u0.plot(t, nom_u0, "--", color="C2", linewidth=1.3, label="feedforward u_nom")
        ax_u0.set_ylabel(u0_label)
        ax_u0.set_title("Longitudinal: MPPI vs nominal")
        ax_u0.grid(True, alpha=0.3)
        ax_u0.legend(loc="best", fontsize=7)

        ax_u1 = fig.add_subplot(gs[row_cmp, 1])
        ax_u1.plot(t, u1, "-", color="C5", linewidth=1.3, label="MPPI applied")
        ax_u1.plot(t, nom_u1, "--", color="C2", linewidth=1.3, label="feedforward u_nom")
        ax_u1.set_ylabel(u1_label)
        ax_u1.set_title("Steering: MPPI vs nominal")
        ax_u1.grid(True, alpha=0.3)
        ax_u1.legend(loc="best", fontsize=7)

        row_diff = row_cmp + 1
        ax_d0 = fig.add_subplot(gs[row_diff, 0])
        ax_d0.plot(t, du0, color="C4", linewidth=1.1, label="u_mppi − u_nom")
        ax_d0.axhline(0.0, color="0.4", linewidth=0.8, linestyle=":")
        ax_d0.set_ylabel(f"Δ {u0_label}")
        ax_d0.set_xlabel("t [s]")
        ax_d0.set_title("|MPPI − nominal| (accel)")
        ax_d0.grid(True, alpha=0.3)
        ax_d0.legend(loc="best", fontsize=7)

        ax_d1 = fig.add_subplot(gs[row_diff, 1])
        ax_d1.plot(t, du1, color="C5", linewidth=1.1, label="u_mppi − u_nom")
        ax_d1.axhline(0.0, color="0.4", linewidth=0.8, linestyle=":")
        ax_d1.set_ylabel(f"Δ {u1_label}")
        ax_d1.set_xlabel("t [s]")
        ax_d1.set_title("|MPPI − nominal| (steer)")
        ax_d1.grid(True, alpha=0.3)
        ax_d1.legend(loc="best", fontsize=7)

        row_prog = row_diff + 1
    else:
        row_prog = row_after_controls
    if arc_s is not None:
        ax_s = fig.add_subplot(gs[row_prog, :])
        ax_s.plot(t, arc_s, color="C6", linewidth=1.2, label="arc s (projected)")
        if ref_v_target is not None and len(ref_v_target):
            v_ref = float(np.median(ref_v_target))
            v_ref_label = "speedAt target"
        elif ref_v_pose is not None and len(ref_v_pose):
            v_ref = float(np.median(ref_v_pose))
            v_ref_label = "pose v"
        else:
            v_ref = 3.0
            v_ref_label = "ref"
        if perimeter and perimeter > 0:
            s_unwrap = np.zeros(len(arc_s))
            s_unwrap[0] = arc_s[0]
            for i in range(1, len(arc_s)):
                ds = arc_s[i] - arc_s[i - 1]
                if ds < -0.5 * perimeter:
                    ds += perimeter
                elif ds > 0.5 * perimeter:
                    ds -= perimeter
                s_unwrap[i] = s_unwrap[i - 1] + ds
            ax_s.plot(
                t,
                s_unwrap,
                "-.",
                color="C6",
                alpha=0.65,
                linewidth=1.0,
                label="arc s (unwrapped)",
            )
            ax_s.plot(
                t,
                v_ref * t,
                "r--",
                alpha=0.55,
                linewidth=1.0,
                label=f"v_ref·t ({v_ref:.1f} m/s, {v_ref_label})",
            )
        else:
            ax_s.plot(
                t,
                v_ref * t,
                "r--",
                alpha=0.55,
                linewidth=1.0,
                label=f"v_ref·t ({v_ref:.1f} m/s, {v_ref_label})",
            )
        ax_s.set_ylabel("arc length [m]")
        ax_s.set_xlabel("t [s]")
        ax_s.set_title("Progress along path")
        ax_s.legend(loc="best", fontsize=7)
        ax_s.grid(True, alpha=0.3)
        row_prog += 1

    if lat_err is not None:
        ax_lat = fig.add_subplot(gs[row_prog, :])
        ax_lat.plot(t, lat_err, color="C7", linewidth=1.2, label="signed lateral error")
        ax_lat.axhline(0.0, color="0.4", linewidth=0.8, linestyle=":")
        ax_lat.set_ylabel("lateral [m] (+ = left of path)")
        ax_lat.set_xlabel("t [s]")
        ax_lat.set_title("Projection: signed lateral offset from path")
        ax_lat.legend(loc="best", fontsize=7)
        ax_lat.grid(True, alpha=0.3)

    fig2, axb = plt.subplots(1, 1, figsize=(10, 3.2), constrained_layout=True)
    if nominal_cost is not None and mppi_applied_cost is not None:
        axb.plot(
            t,
            nominal_cost,
            "--",
            color="C2",
            linewidth=1.2,
            label="feedforward u_nom (1-step running cost)",
        )
        axb.plot(
            t,
            mppi_applied_cost,
            "-",
            color="C4",
            linewidth=1.2,
            label="MPPI applied u (1-step running cost)",
        )
        axb.plot(
            t,
            baseline,
            ":",
            color="0.35",
            linewidth=1.1,
            label="MPPI min rollout cost (best of 32k samples)",
        )
        axb.set_title("Per-step cost: nominal vs MPPI applied vs best rollout")
    else:
        axb.plot(t, baseline, color="0.2", linewidth=1.0, label="baseline (min rollout cost)")
        axb.set_title("MPPI minimum rollout cost (re-run example for nominal/mppi_applied_cost columns)")
    axb.set_ylabel("cost")
    axb.set_xlabel("t [s]")
    axb.legend(loc="best", fontsize=7)
    axb.grid(True, alpha=0.3)

    if save_files:
        base = out_png if out_png.suffix.lower() == ".png" else out_png.with_suffix(".png")
        fig.savefig(base, dpi=args.dpi)
        baseline_png = base.with_name(base.stem + "_baseline.png")
        fig2.savefig(baseline_png, dpi=args.dpi)
        print(f"Wrote {base}")
        print(f"Wrote {baseline_png}")

    if want_show:
        print("Close all plot windows to exit.", file=sys.stderr)
        plt.show()
    else:
        plt.close(fig)
        plt.close(fig2)
        if not save_files:
            print("error: no display and -o - given; use --show on a machine with a GUI.", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
