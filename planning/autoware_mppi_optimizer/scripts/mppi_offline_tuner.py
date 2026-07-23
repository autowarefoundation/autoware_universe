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

r"""Thin wrapper: launches mppi_debug_visualizer.py in offline compare+retune mode.

Uses the same plots as the debug visualizer (XY + heading/vel/accel/steer/steer-rate),
overlaying diffusion reference, logged MPPI output, and a retuned MPPI result.

Example:
  ros2 run autoware_mppi_optimizer mppi_offline_tuner.py -- \
    --log-dir "$HOME/.cache/autoware/mppi_debug_log" \
    --params-yaml $(ros2 pkg prefix autoware_mppi_optimizer)/share/autoware_mppi_optimizer/config/mppi_optimizer.param.yaml
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys


def find_visualizer() -> Path:
    env = os.environ.get("MPPI_DEBUG_VISUALIZER")
    if env:
        candidate = Path(env).expanduser().resolve()
        if candidate.is_file() and candidate.suffix == ".py":
            return candidate
        raise FileNotFoundError(f"MPPI_DEBUG_VISUALIZER={env!r} is not an existing .py file.")
    try:
        prefix = subprocess.check_output(
            ["ros2", "pkg", "prefix", "autoware_diffusion_planner"],
            text=True,
            timeout=30,
        ).strip()
        candidate = Path(prefix) / "lib" / "autoware_diffusion_planner" / "mppi_debug_visualizer.py"
        if candidate.is_file():
            return candidate
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
        pass
    # Source-tree fallback (dev without install).
    here = Path(__file__).resolve()
    candidate = (
        here.parents[2] / "autoware_diffusion_planner" / "scripts" / "mppi_debug_visualizer.py"
    )
    if candidate.is_file():
        return candidate
    raise FileNotFoundError(
        "mppi_debug_visualizer.py not found. Source the workspace or set MPPI_DEBUG_VISUALIZER."
    )


def _existing_dir(value: str) -> str:
    path = Path(value).expanduser().resolve()
    if not path.is_dir():
        raise argparse.ArgumentTypeError(f"not an existing directory: {value}")
    return str(path)


def _optional_existing_file(value: str) -> str:
    if not value:
        return ""
    path = Path(value).expanduser().resolve()
    if not path.is_file():
        raise argparse.ArgumentTypeError(f"not an existing file: {value}")
    return str(path)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "mppi_offline_tuner.py — compare logged MPPI I/O and retune interactively. "
            "Forwards a validated option set to mppi_debug_visualizer.py --enable-retune."
        )
    )
    parser.add_argument(
        "--log-dir",
        required=True,
        type=_existing_dir,
        help="Directory written by enable_debug_trajectory_log",
    )
    parser.add_argument(
        "--params-yaml",
        default="",
        type=_optional_existing_file,
        help="Baseline mppi_optimizer.param.yaml",
    )
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument(
        "--retune-bin",
        default="",
        type=_optional_existing_file,
        help="Path to mppi_offline_retune binary",
    )
    parser.add_argument("--wheel-base", type=float, default=None)
    parser.add_argument("--ego-width", type=float, default=None)
    parser.add_argument("--ego-length", type=float, default=None)
    # Reject unknown flags (no raw argv forwarding).
    return parser.parse_args(argv)


def build_visualizer_cmd(visualizer: Path, args: argparse.Namespace) -> list[str]:
    cmd = [
        sys.executable,
        str(visualizer),
        "--enable-retune",
        "--log-dir",
        args.log_dir,
        "--start-frame",
        str(args.start_frame),
    ]
    if args.params_yaml:
        cmd.extend(["--params-yaml", args.params_yaml])
    if args.retune_bin:
        cmd.extend(["--retune-bin", args.retune_bin])
    if args.wheel_base is not None:
        cmd.extend(["--wheel-base", str(args.wheel_base)])
    if args.ego_width is not None:
        cmd.extend(["--ego-width", str(args.ego_width)])
    if args.ego_length is not None:
        cmd.extend(["--ego-length", str(args.ego_length)])
    return cmd


def main() -> None:
    argv = [a for a in sys.argv[1:] if a != "--"]
    args = parse_args(argv)
    visualizer = find_visualizer()
    cmd = build_visualizer_cmd(visualizer, args)
    os.execv(cmd[0], cmd)


if __name__ == "__main__":
    main()
