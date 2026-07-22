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

"""Thin wrapper: launches mppi_debug_visualizer.py in offline compare+retune mode.

Uses the same plots as the debug visualizer (XY + heading/vel/accel/steer/steer-rate),
overlaying diffusion reference, logged MPPI output, and a retuned MPPI result.

Example:
  ros2 run autoware_mppi_optimizer mppi_offline_tuner.py -- \
    --log-dir /tmp/mppi_debug_log \
    --params-yaml $(ros2 pkg prefix autoware_mppi_optimizer)/share/autoware_mppi_optimizer/config/mppi_optimizer.param.yaml
"""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys


def find_visualizer() -> Path:
    env = os.environ.get("MPPI_DEBUG_VISUALIZER")
    if env:
        return Path(env)
    try:
        prefix = subprocess.check_output(
            ["ros2", "pkg", "prefix", "autoware_diffusion_planner"], text=True
        ).strip()
        candidate = Path(prefix) / "lib" / "autoware_diffusion_planner" / "mppi_debug_visualizer.py"
        if candidate.is_file():
            return candidate
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    # Source-tree fallback (dev without install).
    here = Path(__file__).resolve()
    candidate = (
        here.parents[2]
        / "autoware_diffusion_planner"
        / "scripts"
        / "mppi_debug_visualizer.py"
    )
    if candidate.is_file():
        return candidate
    raise FileNotFoundError(
        "mppi_debug_visualizer.py not found. Source the workspace or set MPPI_DEBUG_VISUALIZER."
    )


def main() -> None:
    argv = [a for a in sys.argv[1:] if a != "--"]
    if "--help" in argv or "-h" in argv:
        print(
            "mppi_offline_tuner.py — compare logged MPPI I/O and retune interactively.\n"
            "Forwards to mppi_debug_visualizer.py --log-dir ... --enable-retune.\n\n"
            "Required:\n"
            "  --log-dir DIR\n\n"
            "Useful options (same as visualizer):\n"
            "  --params-yaml FILE\n"
            "  --start-frame N\n"
            "  --retune-bin PATH\n"
            "  --wheel-base / --ego-width / --ego-length\n"
        )
        return

    if "--log-dir" not in argv:
        print("Error: --log-dir is required.", file=sys.stderr)
        print("Example: mppi_offline_tuner.py -- --log-dir /tmp/mppi_debug_log", file=sys.stderr)
        sys.exit(2)

    visualizer = find_visualizer()
    cmd = [sys.executable, str(visualizer), "--enable-retune", *argv]
    os.execv(sys.executable, cmd)


if __name__ == "__main__":
    main()
