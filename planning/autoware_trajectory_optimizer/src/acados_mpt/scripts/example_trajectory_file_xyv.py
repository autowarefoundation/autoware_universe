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
import os
from pathlib import Path
import sys

import numpy as np
from scripts.utils import NY_STAGE_DEFAULT
from scripts.utils import _ensure_acados_template_on_path
from scripts.utils import add_common_temporal_mpc_arguments
from scripts.utils import example_build_dir
from scripts.utils import load_xy_yaw_file
from scripts.utils import pad_time_series_reference
from scripts.utils import parse_x0_string
from scripts.utils import plot_mpc_run
from scripts.utils import print_ref_vs_state_table
from scripts.utils import run_temporal_mpc_experiment
from scripts.utils import solve_open_loop_mpc

# Stable figure id so directory mode can redraw in-place without closing the GUI window.
_MPC_DIR_BROWSER_FIG_ID = "temporal_mpc_xyv_dir_browser"


def _collect_txt_paths(directory: Path) -> list[Path]:
    paths = sorted(p.resolve() for p in directory.glob("*.txt") if p.is_file())
    return paths


def _x0_from_args_and_file(
    file_x0: np.ndarray | None,
    x0_arg: str,
    x_r: np.ndarray,
    y_r: np.ndarray,
    psi_r: np.ndarray,
    v_r: np.ndarray,
) -> np.ndarray:
    if file_x0 is not None:
        if x0_arg.strip():
            print(
                "Ignoring --x0: reference file contains an 'x0:' header line.",
                file=sys.stderr,
            )
        return file_x0
    if x0_arg.strip():
        return parse_x0_string(x0_arg)
    return np.array([x_r[0], y_r[0], psi_r[0], max(0.0, float(v_r[0]))])


def run_interactive_directory(
    *,
    directory: Path,
    dt: float,
    N: int,
    generate_only: bool,
    x0_arg: str,
    save_figure: str | None,
    no_print_ref_state: bool,
) -> None:
    """Open one figure; arrow keys cycle ``*.txt`` files and re-run the open-loop MPC solve."""
    paths = _collect_txt_paths(directory)
    if not paths:
        print(f"No .txt files in {directory}", file=sys.stderr)
        sys.exit(1)

    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        raise RuntimeError("directory mode requires matplotlib") from e

    _ensure_acados_template_on_path()
    # Import after acados_template is on sys.path (not at module level: import would run too early).
    from generators.path_tracking_mpc_temporal import PathTrackingMPCTemporal

    n_ref_segments = N
    n_need = n_ref_segments + 1
    Tf = float(N) * dt

    build_dir = example_build_dir()
    build_dir.mkdir(parents=True, exist_ok=True)
    old_cwd = os.getcwd()

    os.chdir(build_dir)
    try:
        mpc = PathTrackingMPCTemporal(
            Tf,
            N,
            build=not generate_only,
            generate=True,
        )
        if generate_only:
            print(f"Generated code in {build_dir}")
            return

        solver = mpc.acados_solver
        if solver is None:
            raise RuntimeError("acados_solver is None")

        p_vec = np.array([1.0, 1.0])
        ny_stage = NY_STAGE_DEFAULT

        idx = [0]
        key_connected = [False]

        def solve_one(path: Path) -> tuple:
            file_x0, raw_x, raw_y, raw_psi, raw_v = load_xy_yaw_file(path)
            x_r, y_r, psi_r, v_r = pad_time_series_reference(raw_x, raw_y, raw_psi, n_need, v=raw_v)
            if len(raw_x) < n_need:
                print(
                    f"{path.name}: {len(raw_x)} poses < needed {n_need}; holding last pose.",
                    file=sys.stderr,
                )
            x0 = _x0_from_args_and_file(file_x0, x0_arg, x_r, y_r, psi_r, v_r)
            sol_x, sol_u, status = solve_open_loop_mpc(
                solver, N, x_r, y_r, v_r, psi_r, x0, p_vec, ny_stage
            )
            x_r_plot = x_r[: N + 1]
            y_r_plot = y_r[: N + 1]
            v_r_plot = v_r[: N + 1]
            psi_r_plot = psi_r[: N + 1]
            return (
                sol_x,
                sol_u,
                status,
                x_r_plot,
                y_r_plot,
                v_r_plot,
                psi_r_plot,
                path,
            )

        def on_key(event) -> None:
            if event.key is None:
                return
            k = str(event.key).lower()
            if k in ("q", "escape"):
                plt.close(event.canvas.figure)
                return
            if k in ("left", "arrowleft"):
                idx[0] = (idx[0] - 1) % len(paths)
                redraw()
            elif k in ("right", "arrowright"):
                idx[0] = (idx[0] + 1) % len(paths)
                redraw()

        def redraw() -> None:
            (
                sol_x,
                sol_u,
                status,
                x_r_plot,
                y_r_plot,
                v_r_plot,
                psi_r_plot,
                path,
            ) = solve_one(paths[idx[0]])
            err_xy = np.hypot(sol_x[:, 0] - x_r_plot, sol_x[:, 1] - y_r_plot)
            print(
                f"\n--- [{idx[0] + 1}/{len(paths)}] {path.name} | acados status={status} | "
                f"max pos err [m]={float(np.max(err_xy)):.4f} ---\n"
            )
            if not no_print_ref_state:
                print_ref_vs_state_table(dt, x_r_plot, y_r_plot, psi_r_plot, v_r_plot, sol_x)

            title = (
                f"Temporal MPC (dir) [{idx[0] + 1}/{len(paths)}] {path.name} | status={status} "
                "| Left/Right = prev/next | Q = quit"
            )
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
                show=False,
                track_a=None,
                track_b=None,
                title=title,
                draw_ref_to_state_links=True,
                reuse_figure_id=_MPC_DIR_BROWSER_FIG_ID,
            )
            fig = plt.gcf()
            if not key_connected[0]:
                fig.canvas.mpl_connect("key_press_event", on_key)
                key_connected[0] = True
            fig.canvas.draw_idle()

        redraw()
        plt.show()
    finally:
        os.chdir(old_cwd)


def main() -> None:
    """Parse CLI flags and run a single open-loop MPC solve with a reference trajectory from disk."""
    parser = argparse.ArgumentParser(
        description="Temporal MPC example: open-loop solve from file(s): x, y, yaw[, v_ref] per line.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Use either --reference-file or --reference-directory (not both). "
            "Directory mode: Left/Right browse *.txt files, Q quits; requires an interactive display."
        ),
    )
    add_common_temporal_mpc_arguments(parser, include_closed_loop_sim_cli=False)
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument(
        "--reference-file",
        type=str,
        help="One text file: optional 'x0:' line, then x, y, yaw[, v_ref] per row.",
    )
    src.add_argument(
        "--reference-directory",
        type=str,
        metavar="DIR",
        help="Directory of *.txt replay files; Left/Right switch files, Q quits (no --no-show / --no-plot).",
    )
    parser.add_argument(
        "--no-print-ref-state",
        action="store_true",
        help="do not print the per-index reference vs MPC state table to stdout",
    )
    args = parser.parse_args()

    dt = args.dt
    N = args.N
    n_ref_segments = N

    if args.reference_directory:
        if args.no_plot or args.no_show:
            print(
                "Directory mode needs an interactive plot window; omit --no-plot and --no-show.",
                file=sys.stderr,
            )
            sys.exit(2)
        if args.generate_only:
            print("Use --reference-file with --generate-only.", file=sys.stderr)
            sys.exit(2)
        run_interactive_directory(
            directory=Path(args.reference_directory).expanduser().resolve(),
            dt=dt,
            N=N,
            generate_only=args.generate_only,
            x0_arg=args.x0,
            save_figure=args.save_figure,
            no_print_ref_state=args.no_print_ref_state,
        )
        return

    n_need = n_ref_segments + 1
    file_x0, raw_x, raw_y, raw_psi, raw_v = load_xy_yaw_file(Path(args.reference_file))
    x_r, y_r, psi_r, v_r = pad_time_series_reference(raw_x, raw_y, raw_psi, n_need, v=raw_v)
    if len(raw_x) < n_need:
        print(
            f"reference-file: {len(raw_x)} poses < needed {n_need}; "
            f"holding last pose (time-indexed at dt={dt:g} s).",
            file=sys.stderr,
        )

    x0 = _x0_from_args_and_file(file_x0, args.x0, x_r, y_r, psi_r, v_r)

    plot_title = "Temporal MPC (file ref): open-loop prediction (single solve)"

    run_temporal_mpc_experiment(
        dt=dt,
        n_horizon=N,
        open_loop=True,
        t_sim=0.0,
        generate_only=args.generate_only,
        x_r=x_r,
        y_r=y_r,
        v_r=v_r,
        psi_r=psi_r,
        x0=x0,
        no_plot=args.no_plot,
        save_figure=args.save_figure,
        no_show=args.no_show,
        plot_title=plot_title,
        track_a=None,
        track_b=None,
        plot_draw_ref_to_state_links=True,
        print_ref_vs_state=not args.no_print_ref_state,
    )


if __name__ == "__main__":
    main()
