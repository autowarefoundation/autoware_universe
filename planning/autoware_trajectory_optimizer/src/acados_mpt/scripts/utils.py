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

from __future__ import annotations

import argparse
import importlib.util
import os
from pathlib import Path
import re
import sys

import matplotlib.pyplot as plt
import numpy as np

NY_STAGE_DEFAULT = 6


def _ensure_acados_template_on_path() -> None:
    """Ensure ``acados_template`` is importable (same layout as CMake / Autoware acados)."""
    if importlib.util.find_spec("acados_template") is not None:
        return
    acados = os.environ.get("ACADOS_SOURCE_DIR", "/opt/acados")
    tpl = Path(acados) / "interfaces" / "acados_template"
    if tpl.is_dir():
        p = str(tpl.resolve())
        if p not in sys.path:
            sys.path.insert(0, p)
    if importlib.util.find_spec("acados_template") is None:
        raise RuntimeError(
            "Cannot import acados_template. Install acados and either set ACADOS_SOURCE_DIR to "
            "its root or prepend interfaces/acados_template to PYTHONPATH, e.g.\n"
            "  export ACADOS_SOURCE_DIR=/opt/acados\n"
            '  export PYTHONPATH="$ACADOS_SOURCE_DIR/interfaces/acados_template:$PYTHONPATH"'
        )


def translate_xy_horizon_to_local(
    x_off: float,
    y_off: float,
    x0: np.ndarray,
    yref: np.ndarray,
    yref_e: np.ndarray,
) -> None:
    """Subtract ``(x_off, y_off)`` from world-frame ``x,y`` in ``x0``, ``yref``, ``yref_e`` (in-place).

    Kinematic bicycle dynamics do not depend on absolute ``x,y``; centering at the ego shrinks
    magnitudes for the NLP (map coordinates ~1e5 m) without changing the physical problem.
    """
    x0[0] -= x_off
    x0[1] -= y_off
    for k in range(yref.shape[0]):
        yref[k, 0] -= x_off
        yref[k, 1] -= y_off
    yref_e[0] -= x_off
    yref_e[1] -= y_off


def unshift_sol_x_xy(sol_x: np.ndarray, x_off: float, y_off: float) -> None:
    """Add ``(x_off, y_off)`` back to predicted positions (in-place)."""
    sol_x[:, 0] += x_off
    sol_x[:, 1] += y_off


def build_yref_row(
    x_r: np.ndarray,
    y_r: np.ndarray,
    v_r: np.ndarray,
    psi_r: np.ndarray,
    j: int,
    psi_bias: float = 0.0,
) -> np.ndarray:
    """Return one LINEAR_LS stage reference row (state + inputs) at index ``j``."""
    return np.array(
        [
            x_r[j],
            y_r[j],
            psi_r[j] + psi_bias,
            v_r[j],
            0.0,
            0.0,
        ]
    )


def run_closed_loop_mpc(
    solver,
    dt: float,
    N: int,
    n_steps: int,
    x_r: np.ndarray,
    y_r: np.ndarray,
    v_r: np.ndarray,
    psi_r: np.ndarray,
    x0: np.ndarray,
    p_vec: np.ndarray,
    ny_stage: int,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Apply receding-horizon MPC for ``n_steps`` steps using u0 and acados x1."""
    nx = x0.shape[0]
    nu = 2
    sim_x = np.zeros((n_steps + 1, nx))
    sim_u = np.zeros((n_steps, nu))
    sim_x[0, :] = x0
    last_status = 0

    two_pi = 2.0 * np.pi
    for m in range(n_steps):
        # LINEAR_LS cost on psi vs unwrapped ref: shift reference by k*2pi so first stage matches
        # vehicle yaw branch (avoids false ~2pi heading error after laps).
        psi_bias = float(np.round((sim_x[m, 2] - psi_r[m]) / two_pi) * two_pi)
        yref = np.zeros((N, ny_stage))
        for k in range(N):
            if k == 0:
                # Match fixed x[0]=sim_x[m]; do not track ref[m] here (avoids cost vs lbx conflict).
                yref[k, :] = np.array(
                    [
                        float(sim_x[m, 0]),
                        float(sim_x[m, 1]),
                        float(sim_x[m, 2]),
                        float(sim_x[m, 3]),
                        0.0,
                        0.0,
                    ]
                )
                continue
            j = m + k
            if j >= len(x_r):
                j = len(x_r) - 1
            yref[k, :] = build_yref_row(x_r, y_r, v_r, psi_r, j, psi_bias=psi_bias)
        j_e = min(m + N, len(x_r) - 1)
        yref_e = np.array([x_r[j_e], y_r[j_e], psi_r[j_e] + psi_bias, v_r[j_e]])

        x_off = float(sim_x[m, 0])
        y_off = float(sim_x[m, 1])
        x_stage = np.array(sim_x[m, :], dtype=float, copy=True)
        translate_xy_horizon_to_local(x_off, y_off, x_stage, yref, yref_e)

        solver.set(0, "lbx", x_stage)
        solver.set(0, "ubx", x_stage)
        for i in range(N):
            solver.set(i, "yref", yref[i])
            solver.set(i, "p", p_vec)
        solver.set(N, "yref", yref_e)
        solver.set(N, "p", p_vec)

        last_status = solver.solve()
        sim_u[m, :] = solver.get(0, "u").flatten()
        sim_x[m + 1, :] = solver.get(1, "x").flatten()
        sim_x[m + 1, 0] += x_off
        sim_x[m + 1, 1] += y_off

    return sim_x, sim_u, last_status


def solve_open_loop_mpc(
    solver,
    N: int,
    x_r: np.ndarray,
    y_r: np.ndarray,
    v_r: np.ndarray,
    psi_r: np.ndarray,
    x0: np.ndarray,
    p_vec: np.ndarray,
    ny_stage: int,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Solve one open-loop MPC problem and return full state/control trajectories.

    Delegates to :func:`solve_autoware_temporal_mpc` so horizon references match the C++ plugin:
    ``start_idx`` = closest path index to ego; stages ``k>=1`` use ``ref[min(start_idx+k, n-1)]``,
    not ``ref[k]`` from the polyline origin (which misaligns when ``x0`` is not at ``ref[0]``).
    """
    return solve_autoware_temporal_mpc(solver, N, x0, x_r, y_r, psi_r, v_r, p_vec, ny_stage)


def solve_autoware_temporal_mpc(
    solver,
    N: int,
    x0: np.ndarray,
    x_pts: np.ndarray,
    y_pts: np.ndarray,
    psi_pts: np.ndarray,
    v_pts: np.ndarray,
    p_vec: np.ndarray,
    ny_stage: int,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Single-shot MPC matching ``TrajectoryTemporalMPTOptimizer`` (C++ plugin).

    ``start_idx`` = closest trajectory index to ego ``(x0[0], x0[1])``; stage ``k==0`` uses
    ``yref`` from ``x0``; stages ``k>=1`` use ``min(start_idx+k, n-1)`` on the discrete path;
    terminal reference at ``min(start_idx+N, n-1)``. Reference yaw uses the same ``psi_bias`` as
    :func:`run_closed_loop_mpc` (integer multiple of ``2π`` so ``psi_ref[start_idx]`` matches
    ``x0[2]`` in the LS cost, avoiding wrap-induced steering).

    ``x,y`` are recentered at the ego for the NLP (:func:`translate_xy_horizon_to_local`); the
    returned ``sol_x`` is in **map/world** frame again (:func:`unshift_sol_x_xy`).

    Parameters
    ----------
    x_pts, y_pts, psi_pts, v_pts
        Same length ``n_pts`` (Autoware trajectory reference columns).
    x0
        Length-4 state ``[x, y, psi, v]`` (e.g. odometry), **map/world frame**.
    """
    x_pts = np.asarray(x_pts, dtype=float)
    y_pts = np.asarray(y_pts, dtype=float)
    psi_pts = np.asarray(psi_pts, dtype=float)
    v_pts = np.asarray(v_pts, dtype=float)
    x0 = np.array(x0, dtype=float, copy=True)

    n_pts = x_pts.shape[0]
    if n_pts < 2:
        raise ValueError("need at least 2 reference points")
    nx = int(x0.shape[0])
    best_d2 = float("inf")
    start_idx = 0
    for i in range(n_pts):
        dx = float(x_pts[i]) - float(x0[0])
        dy = float(y_pts[i]) - float(x0[1])
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            start_idx = i

    two_pi = 2.0 * np.pi
    # Same branch alignment as run_closed_loop_mpc: shift all ref headings by k*2π so the
    # reference at start_idx matches ego yaw in LINEAR_LS (avoids false ~2π psi cost when logs
    # wrap or use a different atan2 branch than the bicycle state).
    psi_bias = float(np.round((float(x0[2]) - float(psi_pts[start_idx])) / two_pi) * two_pi)

    yref = np.zeros((N, ny_stage))
    for k in range(N):
        if k == 0:
            yref[k, :] = np.array(
                [float(x0[0]), float(x0[1]), float(x0[2]), float(x0[3]), 0.0, 0.0]
            )
        else:
            idx = min(start_idx + k, n_pts - 1)
            yref[k, :] = build_yref_row(x_pts, y_pts, v_pts, psi_pts, idx, psi_bias=psi_bias)

    j_e = min(start_idx + N, n_pts - 1)
    yref_e = np.array(
        [
            float(x_pts[j_e]),
            float(y_pts[j_e]),
            float(psi_pts[j_e]) + psi_bias,
            float(v_pts[j_e]),
        ]
    )

    x_off = float(x0[0])
    y_off = float(x0[1])
    translate_xy_horizon_to_local(x_off, y_off, x0, yref, yref_e)

    solver.set(0, "lbx", x0)
    solver.set(0, "ubx", x0)
    for i in range(N):
        solver.set(i, "yref", yref[i])
        solver.set(i, "p", p_vec)
    solver.set(N, "yref", yref_e)
    solver.set(N, "p", p_vec)

    status = int(solver.solve())

    sol_x = np.zeros((N + 1, nx))
    sol_u = np.zeros((N, 2))
    for i in range(N + 1):
        sol_x[i, :] = solver.get(i, "x").flatten()
    for i in range(N):
        sol_u[i, :] = solver.get(i, "u").flatten()

    unshift_sol_x_xy(sol_x, x_off, y_off)

    return sol_x, sol_u, status


def parse_stop_times(stop_times_arg: str) -> tuple[float, ...]:
    """Parse comma-separated stop times [s] into a tuple of floats."""
    if not stop_times_arg.strip():
        return ()
    return tuple(float(v.strip()) for v in stop_times_arg.split(",") if v.strip())


def load_xy_yaw_file(
    path: Path,
) -> tuple[np.ndarray | None, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Load reference file: optional ``x0: x y yaw v`` line, then path rows.

    Each path row is ``x, y, yaw`` or ``x, y, yaw, v_ref`` (comma- or space-separated). If the
    fourth value is omitted, ``v_ref`` defaults to ``0`` for that row (legacy three-column files).

    Legacy (no ``x0:``): every data row is the path; use ``--x0`` or first pose for initial state.

    Accepts UTF-8 BOM, ``x0 :`` (spaces around ``:``), and skips duplicate ``x0:`` lines in the
    pose block (defensive).
    """
    # First line may be BOM-prefixed; strip once per line for matching.
    _x0_header_re = re.compile(r"^x0\s*:\s*(.*)$", re.IGNORECASE)

    lines: list[str] = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            s = line.strip().lstrip("\ufeff")
            if not s or s.startswith("#"):
                continue
            lines.append(s)

    if not lines:
        raise ValueError(f"No data lines in {path}")

    file_x0: np.ndarray | None = None
    start_i = 0
    m0 = _x0_header_re.match(lines[0])
    if m0:
        payload = m0.group(1).strip()
        if not payload:
            raise ValueError(f"Empty x0 payload in {path} (line: {lines[0]!r})")
        file_x0 = parse_x0_string(payload)
        start_i = 1

    xs: list[float] = []
    ys: list[float] = []
    psis: list[float] = []
    vs: list[float] = []
    for line in lines[start_i:]:
        s = line.strip().lstrip("\ufeff")
        if _x0_header_re.match(s):
            continue
        parts = s.replace(",", " ").split()
        if len(parts) < 3:
            continue
        try:
            xs.append(float(parts[0]))
            ys.append(float(parts[1]))
            psis.append(float(parts[2]))
            vs.append(float(parts[3]) if len(parts) >= 4 else 0.0)
        except ValueError as e:
            raise ValueError(
                f"Bad trajectory row in {path} (expected three or four numbers): {s!r}"
            ) from e
    if len(xs) < 2:
        raise ValueError(f"Need at least 2 trajectory points in {path}, got {len(xs)}")
    return file_x0, np.array(xs), np.array(ys), np.array(psis), np.array(vs, dtype=float)


def pad_time_series_reference(
    x: np.ndarray,
    y: np.ndarray,
    psi: np.ndarray,
    min_len: int,
    v: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Hold the last pose (and optional ``v_ref``) so the sequence has length ``min_len``.

    If ``v`` is ``None``, reference speed is treated as all zeros before padding.
    """
    n = len(x)
    if v is None:
        v = np.zeros(n, dtype=float)
    elif len(v) != n:
        raise ValueError("pad_time_series_reference: v must match x,y,psi length")
    if n >= min_len:
        return x, y, psi, v
    if n < 1:
        raise ValueError("pad_time_series_reference: empty arrays")
    pad = min_len - n
    v_hold = max(0.0, float(v[-1]))
    return (
        np.concatenate([x, np.full(pad, x[-1])]),
        np.concatenate([y, np.full(pad, y[-1])]),
        np.concatenate([psi, np.full(pad, psi[-1])]),
        np.concatenate([v, np.full(pad, v_hold)]),
    )


def parse_x0_string(s: str) -> np.ndarray:
    """Parse ``x,y,psi`` or ``x,y,psi,v`` into length-4 state (legacy 5-tuple ignores last delta)."""
    parts = [float(p) for p in s.replace(",", " ").split() if p.strip()]
    if len(parts) == 3:
        return np.array([parts[0], parts[1], parts[2], 0.0])
    if len(parts) == 4:
        return np.array(parts)
    if len(parts) == 5:
        return np.array(parts[:4])
    raise ValueError("--x0 expects 3 numbers (x,y,psi) or 4 (x,y,psi,v); optional 5th ignored")


def plot_mpc_run(
    dt: float,
    x_r: np.ndarray,
    y_r: np.ndarray,
    v_r: np.ndarray,
    psi_r: np.ndarray,
    sol_x: np.ndarray,
    sol_u: np.ndarray,
    save_path: Path | None = None,
    show: bool = True,
    track_a: float | None = None,
    track_b: float | None = None,
    title: str = "Temporal MPC: reference vs trajectory",
    draw_ref_to_state_links: bool = False,
    reuse_figure_id: str | None = None,
) -> None:
    """Plot the reference versus simulated trajectory, states, and inputs versus time.

    By default opens an interactive window. If ``save_path`` is set, also writes a PNG there.

    If ``reuse_figure_id`` is set, the plot is drawn into a figure with that ``num`` (see
    :func:`matplotlib.pyplot.figure`), clearing any previous content. When ``show`` is False,
    that figure is **not** closed so callers can refresh the same window (e.g. directory
    browser mode) without destroying the GUI event loop.

    If ``draw_ref_to_state_links`` is True, draws cross markers on reference points, hollow
    circles on MPC states, and a line segment from each reference pose ``(x_r[i], y_r[i])`` to the
    MPC state ``(sol_x[i,0], sol_x[i,1])`` at the same index (per-time / per-index pairing used in
    the plot arrays). Uses ``min(len(x_r), len(sol_x))`` points when lengths differ.
    """
    if plt is None:
        print("matplotlib not installed; skip plot")
        return

    n_states = sol_x.shape[0]
    t_x = np.arange(n_states) * dt
    n_u = sol_u.shape[0]

    if reuse_figure_id is not None:
        fig, axes = plt.subplots(
            2,
            2,
            figsize=(11, 9),
            layout="constrained",
            num=reuse_figure_id,
            clear=True,
        )
    else:
        fig, axes = plt.subplots(2, 2, figsize=(11, 9), layout="constrained")
    ax_xy, ax_xyt, ax_vel, ax_in = axes.ravel()

    if track_a is not None and track_b is not None:
        th_full = np.linspace(0.0, 2.0 * np.pi, 200)
        ax_xy.plot(
            track_a * np.cos(th_full),
            track_b * np.sin(th_full),
            color="0.8",
            linewidth=1.0,
            label="track",
        )
    ax_xy.plot(x_r, y_r, "k--", label="ref segment", linewidth=1.5)
    ax_xy.plot(sol_x[:, 0], sol_x[:, 1], "C0-", label="MPC", linewidth=1.5)
    if draw_ref_to_state_links:
        n_link = min(int(x_r.shape[0]), int(sol_x.shape[0]))
        ax_xy.scatter(
            x_r[:n_link],
            y_r[:n_link],
            marker="x",
            s=30.0,
            c="k",
            linewidths=1.0,
            zorder=4,
            label="ref points",
        )
        ax_xy.scatter(
            sol_x[:n_link, 0],
            sol_x[:n_link, 1],
            marker="o",
            s=22.0,
            facecolors="none",
            edgecolors="C0",
            linewidths=1.1,
            zorder=4,
            label="MPC states",
        )
        for i in range(n_link):
            ax_xy.plot(
                [float(x_r[i]), float(sol_x[i, 0])],
                [float(y_r[i]), float(sol_x[i, 1])],
                color="C3",
                alpha=0.45,
                linewidth=0.9,
                solid_capstyle="round",
                zorder=2,
                label="ref to state (same index)" if i == 0 else None,
            )
    else:
        ax_xy.scatter([x_r[0]], [y_r[0]], c="k", s=36, zorder=5, label="start")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Path")
    ax_xy.legend()
    ax_xy.grid(True)

    ax_xyt.plot(t_x, x_r, "k--", label="x ref")
    ax_xyt.plot(t_x, y_r, "k:", label="y ref")
    ax_xyt.plot(t_x, sol_x[:, 0], "C0-", label="x")
    ax_xyt.plot(t_x, sol_x[:, 1], "C1-", label="y")
    ax_xyt.set_xlabel("t [s]")
    ax_xyt.set_ylabel("[m]")
    ax_xyt.set_title("Position vs time")
    ax_xyt.legend(loc="upper right", fontsize=8)
    ax_xyt.grid(True)

    ax_vel.plot(t_x, v_r, "k--", label="v ref")
    ax_vel.plot(t_x, sol_x[:, 3], "C0-", label="v")
    # Unwrap for display so ref vs actual heading are comparable (no fake atan2 branch cuts).
    ax_vel.plot(t_x, np.unwrap(psi_r.copy()), "k:", label="psi ref")
    ax_vel.plot(t_x, np.unwrap(sol_x[:, 2].copy()), "C2-", label="psi")
    ax_vel.set_xlabel("t [s]")
    ax_vel.set_ylabel("v [m/s] / psi [rad]")
    ax_vel.set_title("Speed and yaw")
    ax_vel.legend(loc="upper right", fontsize=8)
    ax_vel.grid(True)

    t_edges = np.linspace(0.0, n_u * dt, n_u + 1)
    ax_in.stairs(sol_u[:, 0], t_edges, label="a", color="C0")
    ax_in.stairs(sol_u[:, 1], t_edges, label="delta", color="C1")
    ax_in.set_xlabel("t [s]")
    ax_in.set_ylabel("accel [m/s^2] / angle [rad]")
    ax_in.set_title("MPC outputs (inputs)")
    ax_in.legend(loc="upper right", fontsize=8)
    ax_in.grid(True)

    fig.suptitle(title, fontsize=12)
    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=150)
        print(f"Saved figure: {save_path}")
    if show:
        plt.show()
    elif reuse_figure_id is None:
        plt.close(fig)


def example_build_dir() -> Path:
    """Directory used for acados Python build artifacts (shared by example scripts)."""
    return Path(__file__).resolve().parent / "_example_temporal_xyv_build"


def print_ref_vs_state_table(
    dt: float,
    x_r: np.ndarray,
    y_r: np.ndarray,
    psi_r: np.ndarray,
    v_r: np.ndarray,
    sol_x: np.ndarray,
) -> None:
    """Print each MPC state row with the reference row at the **same index** (stdout).

    Includes planar Euclidean tracking error ``e_xy`` between ``(x,y)`` and ``(x_ref,y_ref)`` [m].
    """
    n = min(int(x_r.shape[0]), int(sol_x.shape[0]))
    print("\n# Per-index reference vs MPC state (world frame; t = index * dt)")
    hdr = (
        f"{'i':>5} {'t_s':>8} | "
        f"{'x_ref':>12} {'y_ref':>12} {'psi_ref':>10} {'v_ref':>10} || "
        f"{'x':>12} {'y':>12} {'psi':>10} {'v':>10} {'e_xy[m]':>12}"
    )
    print(hdr)
    print("-" * len(hdr))
    for i in range(n):
        t_s = float(i) * dt
        e_xy = float(
            np.hypot(float(sol_x[i, 0]) - float(x_r[i]), float(sol_x[i, 1]) - float(y_r[i]))
        )
        print(
            f"{i:5d} {t_s:8.3f} | "
            f"{float(x_r[i]):12.6f} {float(y_r[i]):12.6f} {float(psi_r[i]):10.6f} {float(v_r[i]):10.6f} || "
            f"{float(sol_x[i, 0]):12.6f} {float(sol_x[i, 1]):12.6f} "
            f"{float(sol_x[i, 2]):10.6f} {float(sol_x[i, 3]):10.6f} {e_xy:12.6f}"
        )


def run_temporal_mpc_experiment(
    *,
    dt: float,
    n_horizon: int,
    open_loop: bool,
    t_sim: float,
    generate_only: bool,
    x_r: np.ndarray,
    y_r: np.ndarray,
    v_r: np.ndarray,
    psi_r: np.ndarray,
    x0: np.ndarray,
    no_plot: bool,
    save_figure: str | None,
    no_show: bool,
    plot_title: str,
    track_a: float | None,
    track_b: float | None,
    plot_draw_ref_to_state_links: bool = False,
    print_ref_vs_state: bool = False,
) -> None:
    """Chdir to build dir, create solver, run open- or closed-loop MPC, print stats, optional plot."""
    _ensure_acados_template_on_path()

    Tf = float(n_horizon) * dt
    ny_stage = NY_STAGE_DEFAULT

    build_dir = example_build_dir()
    build_dir.mkdir(parents=True, exist_ok=True)
    old_cwd = os.getcwd()
    os.chdir(build_dir)

    try:
        from generators.path_tracking_mpc_temporal import PathTrackingMPCTemporal

        mpc = PathTrackingMPCTemporal(
            Tf,
            n_horizon,
            build=not generate_only,
            generate=True,
        )
        if generate_only:
            print(f"Generated code in {build_dir}")
            return

        solver = mpc.acados_solver
        if solver is None:
            raise RuntimeError("acados_solver is None (build=False path should not happen here)")

        lf, lr = 1.0, 1.0
        p_vec = np.array([lf, lr])

        if open_loop:
            sol_x, sol_u, status = solve_open_loop_mpc(
                solver, n_horizon, x_r, y_r, v_r, psi_r, x0, p_vec, ny_stage
            )
            x_r_plot = x_r[: n_horizon + 1]
            y_r_plot = y_r[: n_horizon + 1]
            v_r_plot = v_r[: n_horizon + 1]
            psi_r_plot = psi_r[: n_horizon + 1]
        else:
            n_steps = max(int(round(t_sim / dt)), 1)
            sol_x, sol_u, status = run_closed_loop_mpc(
                solver,
                dt,
                n_horizon,
                n_steps,
                x_r,
                y_r,
                v_r,
                psi_r,
                x0,
                p_vec,
                ny_stage,
            )
            x_r_plot = x_r[: n_steps + 1]
            y_r_plot = y_r[: n_steps + 1]
            v_r_plot = v_r[: n_steps + 1]
            psi_r_plot = psi_r[: n_steps + 1]

        err_xy = np.hypot(sol_x[:, 0] - x_r_plot, sol_x[:, 1] - y_r_plot)
        print(f"acados status (last solve): {status} (0 = success)")
        print(f"max |x-x_ref|, |y-y_ref| [m]: {float(np.max(err_xy)):.4f}")
        print(f"final state x,y,v: {sol_x[-1, 0]:.3f}, {sol_x[-1, 1]:.3f}, {sol_x[-1, 3]:.3f}")

        if print_ref_vs_state:
            print_ref_vs_state_table(dt, x_r_plot, y_r_plot, psi_r_plot, v_r_plot, sol_x)

        if not no_plot:
            save_png = Path(save_figure).resolve() if save_figure else None
            plot_mpc_run(
                dt,
                x_r_plot,
                y_r_plot,
                v_r_plot,
                psi_r_plot,
                sol_x,
                sol_u,
                save_path=save_png,
                show=not no_show,
                track_a=track_a,
                track_b=track_b,
                title=plot_title,
                draw_ref_to_state_links=plot_draw_ref_to_state_links,
            )
    finally:
        os.chdir(old_cwd)


def add_common_temporal_mpc_arguments(
    parser: argparse.ArgumentParser,
    *,
    include_closed_loop_sim_cli: bool = True,
) -> None:
    """Shared CLI flags for example_track_xyv*.py scripts.

    If ``include_closed_loop_sim_cli`` is False, ``--t-sim`` and ``--open-loop`` are omitted
    (for scripts that only support a single open-loop solve over ``N`` stages).
    """
    parser.add_argument("--dt", type=float, default=0.1, help="reference sampling period [s]")
    parser.add_argument("--N", type=int, default=30, help="MPC horizon length (stages)")
    if include_closed_loop_sim_cli:
        parser.add_argument(
            "--t-sim",
            type=float,
            default=120.0,
            help="closed-loop simulation length [s] (receding horizon); ignored with --open-loop",
        )
        parser.add_argument(
            "--open-loop",
            action="store_true",
            help="single solve over N stages only (horizon = N*dt); do not roll out long MPC",
        )
    parser.add_argument(
        "--generate-only", action="store_true", help="only export codegen, do not compile/solve"
    )
    parser.add_argument("--no-plot", action="store_true", help="skip matplotlib figures")
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="do not open an interactive window (use with --save-figure for headless PNG)",
    )
    parser.add_argument(
        "--save-figure",
        type=str,
        default=None,
        metavar="PATH",
        help="optional: also save the figure to this PNG path (default is window only, no file)",
    )
    parser.add_argument(
        "--x0", type=str, default="", help='Initial bicycle state: "x,y,psi" or "x,y,psi,v".'
    )
