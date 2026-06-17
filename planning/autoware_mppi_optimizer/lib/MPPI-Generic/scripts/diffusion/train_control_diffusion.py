#!/usr/bin/env python3
"""Train a conditional diffusion model on MPPI rollout controls + obstacle context."""
from __future__ import annotations

import argparse
import csv
import random
from pathlib import Path

import numpy as np
import torch

from control_diffusion import (
    DEFAULT_CONTEXT_DIM,
    ControlDiffusionModel,
    DiffusionConfig,
    export_diffusion_weights,
)


def _read_control_row(row: dict[str, str | None], control_dim: int) -> list[float] | None:
    if control_dim == 2 and row.get("u_accel") is not None and row.get("u_steer") is not None:
        try:
            return [float(row["u_accel"]), float(row["u_steer"])]
        except (TypeError, ValueError):
            return None
    vals: list[float] = []
    for key in sorted(k for k in row.keys() if k and k.startswith("u_")):
        raw = row.get(key)
        if raw is None or raw == "":
            return None
        try:
            vals.append(float(raw))
        except ValueError:
            return None
    return vals if len(vals) == control_dim else None


def load_controls_csv(path: Path, horizon: int, control_dim: int) -> dict[int, np.ndarray]:
    by_rollout: dict[int, list[tuple[int, list[float]]]] = {}
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            vals = _read_control_row(row, control_dim)
            if vals is None:
                continue
            r = int(row["rollout_index"])
            step = int(row["step"])
            by_rollout.setdefault(r, []).append((step, vals))
    out: dict[int, np.ndarray] = {}
    for r, samples in by_rollout.items():
        samples.sort(key=lambda x: x[0])
        if len(samples) < horizon:
            continue
        arr = np.asarray([v for _s, v in samples[:horizon]], dtype=np.float32)
        out[r] = arr.reshape(-1)
    return out


def load_meta_int(meta_path: Path, key: str) -> int | None:
    with meta_path.open(newline="") as f:
        for row in csv.DictReader(f):
            if row["key"] == key:
                return int(float(row["value"]))
    return None


def detect_dump_horizons(rollout_roots: list[Path]) -> set[int]:
    horizons: set[int] = set()
    for rollout_root in rollout_roots:
        for step_dir in sorted(rollout_root.glob("step_*")):
            meta_path = step_dir / "meta.csv"
            if not meta_path.is_file():
                continue
            step_horizon = load_meta_int(meta_path, "horizon")
            if step_horizon is not None:
                horizons.add(step_horizon)
    return horizons


def resolve_training_horizon(rollout_roots: list[Path], requested: int) -> int:
    found = detect_dump_horizons(rollout_roots)
    if requested <= 0:
        if not found:
            raise RuntimeError("could not detect horizon from meta.csv in rollout dumps")
        if len(found) != 1:
            raise RuntimeError(
                f"mixed horizons in dumps: {sorted(found)}. "
                "Pass --horizon 50 or --horizon 80 to train on compatible steps only."
            )
        return next(iter(found))
    if found and requested not in found and max(found) < requested:
        raise RuntimeError(
            f"--horizon {requested} exceeds all dump horizons {sorted(found)}. "
            f"Use --horizon {max(found)} or regenerate rollouts with a longer MPPI horizon."
        )
    return requested


def load_nominal_from_combined(path: Path, horizon: int, control_dim: int) -> np.ndarray:
    steps: list[list[float]] = []
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            step = int(row["step"])
            if step >= horizon:
                continue
            vals = [float(row["u_accel"]), float(row["u_steer"])]
            while len(steps) <= step:
                steps.append(vals if len(steps) == step else steps[-1])
            steps[step] = vals
    if len(steps) < horizon:
        raise RuntimeError(f"combined.csv shorter than horizon in {path}")
    return np.asarray(steps[:horizon], dtype=np.float32).reshape(-1)


def load_context_csv(path: Path, context_dim: int) -> np.ndarray:
    values: dict[int, float] = {}
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            values[int(row["index"])] = float(row["value"])
    out = np.zeros(context_dim, dtype=np.float32)
    for i in range(context_dim):
        if i in values:
            out[i] = values[i]
    return out


def fallback_context_from_meta(meta_path: Path, context_dim: int) -> np.ndarray:
    """Legacy dumps without context.csv: scene tail only (boundaries + ego speed)."""
    meta: dict[str, float] = {}
    with meta_path.open(newline="") as f:
        for row in csv.DictReader(f):
            meta[row["key"]] = float(row["value"])
    out = np.zeros(context_dim, dtype=np.float32)
    scene_base = context_dim - 3
    if scene_base >= 0:
        out[scene_base + 0] = meta.get("boundary_threshold_left", meta.get("boundary_threshold", 0.0))
        out[scene_base + 1] = meta.get("boundary_threshold_right", meta.get("boundary_threshold", 0.0))
        out[scene_base + 2] = meta.get("init_vel_x", 0.0)
    return out


def load_step_context(step_dir: Path, context_dim: int) -> np.ndarray:
    context_path = step_dir / "context.csv"
    if context_path.is_file():
        return load_context_csv(context_path, context_dim)
    return fallback_context_from_meta(step_dir / "meta.csv", context_dim)


def collect_training_pairs(rollout_roots: list[Path], horizon: int, control_dim: int, context_dim: int, top_k: int,
                           max_steps: int | None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    deltas: list[np.ndarray] = []
    noms: list[np.ndarray] = []
    contexts: list[np.ndarray] = []
    skipped_short_horizon = 0
    skipped_missing = 0
    used_steps = 0
    for rollout_root in rollout_roots:
        step_dirs = sorted(rollout_root.glob("step_*"))
        if max_steps is not None:
            step_dirs = step_dirs[:max_steps]
        for step_dir in step_dirs:
            controls_path = step_dir / "rollouts_controls.csv"
            combined_path = step_dir / "combined.csv"
            costs_path = step_dir / "costs.csv"
            meta_path = step_dir / "meta.csv"
            if not controls_path.is_file() or not combined_path.is_file() or not costs_path.is_file():
                skipped_missing += 1
                continue
            if not meta_path.is_file():
                skipped_missing += 1
                continue
            step_horizon = load_meta_int(meta_path, "horizon")
            if step_horizon is None or step_horizon < horizon:
                skipped_short_horizon += 1
                continue
            try:
                u_by_rollout = load_controls_csv(controls_path, horizon, control_dim)
                u_nom = load_nominal_from_combined(combined_path, horizon, control_dim)
            except RuntimeError:
                skipped_missing += 1
                continue
            if not u_by_rollout:
                skipped_missing += 1
                continue
            context = load_step_context(step_dir, context_dim)
            ranked: list[tuple[float, int]] = []
            with costs_path.open(newline="") as f:
                for row in csv.DictReader(f):
                    ranked.append((float(row["normalized_weight"]), int(row["rollout_index"])))
            ranked.sort(key=lambda x: x[0], reverse=True)
            pick = [idx for _w, idx in ranked[:top_k] if idx in u_by_rollout]
            if not pick:
                skipped_missing += 1
                continue
            u_top = np.stack([u_by_rollout[i] for i in pick], axis=0)
            deltas.append(u_top - u_nom[None, :])
            noms.append(np.repeat(u_nom[None, :], u_top.shape[0], axis=0))
            contexts.append(np.repeat(context[None, :], u_top.shape[0], axis=0))
            used_steps += 1
    if not deltas:
        found = sorted(detect_dump_horizons(rollout_roots))
        raise RuntimeError(
            "no training pairs. "
            f"Requested horizon={horizon}, dump horizons={found}. "
            "Use --horizon 50 for the intersection/stadium examples (double-park truncates), "
            "or --horizon 80 with only double-park rollouts."
        )
    print(
        f"Loaded {used_steps} MPPI steps "
        f"(skipped {skipped_short_horizon} with horizon<{horizon}, {skipped_missing} incomplete/empty)"
    )
    return np.concatenate(deltas, axis=0), np.concatenate(noms, axis=0), np.concatenate(contexts, axis=0)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("rollout_roots", type=Path, nargs="+", help="One or more step_* rollout dump directories")
    p.add_argument("--out", type=Path, default=Path("control_diffusion.bin"))
    p.add_argument("--horizon", type=int, default=0,
                   help="MPPI horizon (0=auto from dumps; use 50 for intersection/stadium, 80 for double-park only)")
    p.add_argument("--control-dim", type=int, default=2)
    p.add_argument("--context-dim", type=int, default=DEFAULT_CONTEXT_DIM)
    p.add_argument("--hidden-dim", type=int, default=128)
    p.add_argument("--epochs", type=int, default=40)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--top-k", type=int, default=64, help="Top-weight rollouts per step")
    p.add_argument("--max-steps", type=int, default=None, help="Limit MPPI steps per rollout root")
    p.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    horizon = resolve_training_horizon(args.rollout_roots, args.horizon)
    if args.horizon <= 0:
        print(f"Auto-detected horizon={horizon}")
    elif horizon != args.horizon:
        print(f"Using horizon={horizon} (requested {args.horizon})")

    delta_u, u_nom, context = collect_training_pairs(args.rollout_roots, horizon, args.control_dim,
                                                     args.context_dim, args.top_k, args.max_steps)
    print(f"Training samples: {delta_u.shape[0]}  traj_dim={delta_u.shape[1]}  context_dim={context.shape[1]}")

    cfg = DiffusionConfig(horizon=horizon, control_dim=args.control_dim, hidden_dim=args.hidden_dim,
                          context_dim=args.context_dim)
    model = ControlDiffusionModel(cfg).to(args.device)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)

    delta_t = torch.from_numpy(delta_u).to(args.device)
    nom_t = torch.from_numpy(u_nom).to(args.device)
    ctx_t = torch.from_numpy(context).to(args.device)
    n = delta_t.shape[0]

    for epoch in range(args.epochs):
        perm = torch.randperm(n, device=args.device)
        loss_sum = 0.0
        batches = 0
        for start in range(0, n, args.batch_size):
            idx = perm[start : start + args.batch_size]
            loss = model.training_loss(delta_t[idx], nom_t[idx], ctx_t[idx])
            opt.zero_grad()
            loss.backward()
            opt.step()
            loss_sum += float(loss.item())
            batches += 1
        print(f"epoch {epoch + 1}/{args.epochs}  loss={loss_sum / max(batches, 1):.6f}")

    export_diffusion_weights(model, args.out)
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
