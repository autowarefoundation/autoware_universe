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

"""Plot one Candidate Bank E_i x O_j cross-collision matrix from analyzer CSV."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict
from typing import List

import matplotlib.pyplot as plt
import numpy as np


def finite(v):
    try:
        x = float(v)
        return math.isfinite(x)
    except (TypeError, ValueError):
        return False


def truth(v: str) -> bool:
    return str(v).strip().lower() in {"1", "true", "yes"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_path", type=Path)
    ap.add_argument("--cycle", default="latest", help="cycle id or latest")
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()

    with args.csv_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit("no rows")
    cycles = sorted({int(r["cycle_id"]) for r in rows})
    cycle = cycles[-1] if args.cycle == "latest" else int(args.cycle)
    rows = [r for r in rows if int(r["cycle_id"]) == cycle]
    if not rows:
        raise SystemExit(f"cycle {cycle} not found")

    ego_ids = sorted({int(r["ego_candidate_id"]) for r in rows})
    scenario_ids = sorted({int(r["scenario_candidate_id"]) for r in rows})
    ei: Dict[int, int] = {v: i for i, v in enumerate(ego_ids)}
    sj: Dict[int, int] = {v: j for j, v in enumerate(scenario_ids)}

    # Categorical matrix: 0=safe/no relevant conflict, 1=XYT conflict, 2=same-time collision.
    z = np.zeros((len(ego_ids), len(scenario_ids)), dtype=float)
    labels: List[List[str]] = [["" for _ in scenario_ids] for _ in ego_ids]
    for r in rows:
        i, j = ei[int(r["ego_candidate_id"])], sj[int(r["scenario_candidate_id"])]
        hard = truth(r.get("same_time_collision", ""))
        xyt = truth(r.get("xyt_conflict", ""))
        z[i, j] = 2.0 if hard else (1.0 if xyt else 0.0)
        rho = r.get("min_xyt_rho", "")
        clearance = r.get("min_same_time_clearance_m", "")
        bits = []
        if finite(rho):
            bits.append(f"rho={float(rho):.2f}")
        if finite(clearance):
            bits.append(f"c={float(clearance):.2f}m")
        if truth(r.get("paired", "")):
            bits.append("paired")
        labels[i][j] = "\n".join(bits) if bits else ("hard" if hard else ("XYT" if xyt else "far"))

    fig, ax = plt.subplots(
        figsize=(max(7.0, len(scenario_ids) * 0.7), max(5.5, len(ego_ids) * 0.6))
    )
    image = ax.imshow(z, vmin=0, vmax=2, interpolation="nearest", aspect="auto")
    cbar = fig.colorbar(image, ax=ax, ticks=[0, 1, 2])
    cbar.ax.set_yticklabels(["no relevant conflict", "XYT conflict", "same-time collision"])
    ax.set_xticks(
        range(len(scenario_ids)), [f"O#{v}" for v in scenario_ids], rotation=45, ha="right"
    )
    ax.set_yticks(range(len(ego_ids)), [f"E#{v}" for v in ego_ids])
    ax.set_xlabel("paired other-agent scenario O_j")
    ax.set_ylabel("ego candidate E_i")
    ax.set_title(f"Candidate Bank cross trajectory conflict | cycle {cycle}")
    for i in range(len(ego_ids)):
        for j in range(len(scenario_ids)):
            ax.text(j, i, labels[i][j], ha="center", va="center", fontsize=7)
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
