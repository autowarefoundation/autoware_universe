"""Small conditional DDPM over MPPI control trajectories (residuals w.r.t. u_nom)."""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
except ImportError as exc:  # pragma: no cover
    raise ImportError("PyTorch is required: pip install torch") from exc


MAGIC = b"MPPIDIFF"
VERSION = 2
TIME_EMB_DIM = 16
# Must match include/mppi/sampling_distributions/diffusion/obstacle_context.hpp
MAX_CONTEXT_OBSTACLES = 4
OBSTACLE_FEATURE_DIM = 8
SCENE_CONTEXT_DIM = 3
DEFAULT_CONTEXT_DIM = MAX_CONTEXT_OBSTACLES * OBSTACLE_FEATURE_DIM + SCENE_CONTEXT_DIM


def sinusoidal_embedding(t: torch.Tensor, dim: int) -> torch.Tensor:
    half = dim // 2
    freqs = torch.exp(-math.log(10000.0) * torch.arange(half, device=t.device, dtype=t.dtype) / half)
    args = t[:, None] * freqs[None, :]
    return torch.cat([torch.sin(args), torch.cos(args)], dim=-1)


@dataclass(frozen=True)
class DiffusionConfig:
    horizon: int
    control_dim: int
    hidden_dim: int = 128
    context_dim: int = DEFAULT_CONTEXT_DIM
    train_timesteps: int = 100
    inference_timesteps: int = 8

    @property
    def traj_dim(self) -> int:
        return self.horizon * self.control_dim


class ControlDiffusionMLP(nn.Module):
    def __init__(self, cfg: DiffusionConfig):
        super().__init__()
        self.cfg = cfg
        in_dim = 2 * cfg.traj_dim + cfg.context_dim + TIME_EMB_DIM
        h = cfg.hidden_dim
        self.fc1 = nn.Linear(in_dim, h)
        self.fc2 = nn.Linear(h, h)
        self.fc3 = nn.Linear(h, cfg.traj_dim)

    def forward(self, x_t: torch.Tensor, u_nom: torch.Tensor, context: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
        t = t.float()
        if t.ndim == 0:
            t = t[None]
        t_emb = sinusoidal_embedding(t / max(self.cfg.train_timesteps - 1, 1), TIME_EMB_DIM)
        if t_emb.shape[0] == 1 and x_t.shape[0] > 1:
            t_emb = t_emb.expand(x_t.shape[0], -1)
        h = torch.cat([x_t, u_nom, context, t_emb], dim=-1)
        h = F.relu(self.fc1(h))
        h = F.relu(self.fc2(h))
        return self.fc3(h)


class ControlDiffusionModel(nn.Module):
    """DDPM on trajectory residuals delta_u = u - u_nom, conditioned on u_nom and obstacle context."""

    def __init__(self, cfg: DiffusionConfig):
        super().__init__()
        self.cfg = cfg
        self.net = ControlDiffusionMLP(cfg)
        betas = torch.linspace(1e-4, 0.02, cfg.train_timesteps)
        alphas = 1.0 - betas
        alpha_bar = torch.cumprod(alphas, dim=0)
        self.register_buffer("betas", betas)
        self.register_buffer("alphas", alphas)
        self.register_buffer("alpha_bar", alpha_bar)
        self.register_buffer("sqrt_alpha_bar", torch.sqrt(alpha_bar))
        self.register_buffer("sqrt_one_minus_alpha_bar", torch.sqrt(1.0 - alpha_bar))

    def q_sample(self, x0: torch.Tensor, t: torch.Tensor, noise: torch.Tensor | None = None) -> torch.Tensor:
        if noise is None:
            noise = torch.randn_like(x0)
        sa = self.sqrt_alpha_bar[t][:, None]
        so = self.sqrt_one_minus_alpha_bar[t][:, None]
        return sa * x0 + so * noise

    def training_loss(self, delta_u: torch.Tensor, u_nom: torch.Tensor, context: torch.Tensor) -> torch.Tensor:
        b = delta_u.shape[0]
        t = torch.randint(0, self.cfg.train_timesteps, (b,), device=delta_u.device)
        noise = torch.randn_like(delta_u)
        x_t = self.q_sample(delta_u, t, noise)
        pred = self.net(x_t, u_nom, context, t)
        return F.mse_loss(pred, noise)

    @torch.no_grad()
    def sample(self, u_nom: torch.Tensor, context: torch.Tensor, num_samples: int,
               generator: torch.Generator | None = None) -> torch.Tensor:
        """Return full control trajectories [num_samples, traj_dim]."""
        device = u_nom.device
        cfg = self.cfg
        if u_nom.ndim == 1:
            u_nom = u_nom[None, :].expand(num_samples, -1)
        elif u_nom.shape[0] == 1 and num_samples > 1:
            u_nom = u_nom.expand(num_samples, -1)
        if context.ndim == 1:
            context = context[None, :].expand(num_samples, -1)
        elif context.shape[0] == 1 and num_samples > 1:
            context = context.expand(num_samples, -1)

        x = torch.randn(num_samples, cfg.traj_dim, device=device, generator=generator)
        steps = np.linspace(cfg.train_timesteps - 1, 0, cfg.inference_timesteps, dtype=int)
        for i, t_val in enumerate(steps):
            t = torch.full((num_samples,), int(t_val), device=device, dtype=torch.long)
            eps = self.net(x, u_nom, context, t)
            alpha = self.alphas[t][:, None]
            alpha_bar = self.alpha_bar[t][:, None]
            beta = self.betas[t][:, None]
            coef1 = 1.0 / torch.sqrt(alpha)
            coef2 = beta / torch.sqrt(1.0 - alpha_bar)
            x = coef1 * (x - coef2 * eps)
            if i + 1 < len(steps):
                x = x + torch.sqrt(beta) * torch.randn_like(x, generator=generator)
        return u_nom + x


def export_diffusion_weights(model: ControlDiffusionModel, path: Path) -> None:
    cfg = model.cfg
    path = Path(path)
    arrays = [
        ("W1", model.net.fc1.weight.detach().cpu().numpy().astype(np.float32)),
        ("b1", model.net.fc1.bias.detach().cpu().numpy().astype(np.float32)),
        ("W2", model.net.fc2.weight.detach().cpu().numpy().astype(np.float32)),
        ("b2", model.net.fc2.bias.detach().cpu().numpy().astype(np.float32)),
        ("W3", model.net.fc3.weight.detach().cpu().numpy().astype(np.float32)),
        ("b3", model.net.fc3.bias.detach().cpu().numpy().astype(np.float32)),
        ("betas", model.betas.detach().cpu().numpy().astype(np.float32)),
        ("alpha_bar", model.alpha_bar.detach().cpu().numpy().astype(np.float32)),
    ]
    with path.open("wb") as f:
        f.write(MAGIC)
        f.write(struct.pack("<8I", VERSION, cfg.horizon, cfg.control_dim, cfg.hidden_dim, cfg.context_dim,
                            cfg.train_timesteps, cfg.inference_timesteps, TIME_EMB_DIM))
        for _name, arr in arrays:
            f.write(struct.pack("<I", arr.size))
            f.write(arr.tobytes())


def _read_array(f) -> np.ndarray:
    (n,) = struct.unpack("<I", f.read(4))
    return np.frombuffer(f.read(4 * n), dtype=np.float32)


def load_diffusion_weights_np(path: Path) -> dict[str, np.ndarray | int]:
    with Path(path).open("rb") as f:
        magic = f.read(8)
        if magic != MAGIC:
            raise ValueError(f"bad magic in {path}")
        header = struct.unpack("<8I", f.read(32))
        version, horizon, control_dim, hidden_dim = header[0], header[1], header[2], header[3]
        context_dim = header[4] if version >= 2 else 0
        train_steps, infer_steps, time_emb_dim = header[5], header[6], header[7]
        if version not in (1, 2):
            raise ValueError(f"unsupported version {version}")
        weights: dict[str, np.ndarray | int] = {
            "horizon": horizon,
            "control_dim": control_dim,
            "hidden_dim": hidden_dim,
            "context_dim": context_dim,
            "train_timesteps": train_steps,
            "inference_timesteps": infer_steps,
            "time_emb_dim": time_emb_dim,
        }
        for key in ("W1", "b1", "W2", "b2", "W3", "b3", "betas", "alpha_bar"):
            weights[key] = _read_array(f)
        return weights
