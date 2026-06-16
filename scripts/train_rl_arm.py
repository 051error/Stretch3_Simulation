#!/usr/bin/env python3
"""RL training script: train Stretch 3 arm with PPO or SAC.

Uses stable-baselines3 in a standalone MuJoCo environment (no ROS 2).

Usage:
    python scripts/train_rl_arm.py                                    # PPO on tomato1
    python scripts/train_rl_arm.py --algo sac                         # SAC on tomato1
    python scripts/train_rl_arm.py --stage approach --algo sac --timesteps 300000
    python scripts/train_rl_arm.py --target tomato2 --algo ppo --timesteps 500000

Dependencies:
    pip install gymnasium stable-baselines3 tensorboard tqdm rich
"""

import argparse
import os
import sys
import time
from pathlib import Path

# Ensure project src is on path
_src = Path(__file__).resolve().parents[1] / "src"
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))

import numpy as np

# ── CLI ────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="Train RL agent for Stretch arm picking")
parser.add_argument(
    "--algo", type=str, default="ppo", choices=["ppo", "sac"],
    help="RL algorithm: ppo (on-policy) or sac (off-policy, more sample-efficient)",
)
parser.add_argument(
    "--stage", type=str, default="pick",
    choices=["approach", "grasp", "pick"],
    help="Training stage: approach (EE near target), grasp (close gripper+lift), "
         "pick (full single-stage, default)",
)
parser.add_argument(
    "--target", type=str, default="tomato1",
    choices=["tomato1", "tomato2", "tomato3"],
    help="Target tomato to pick (default: tomato1)",
)
parser.add_argument(
    "--timesteps", type=int, default=500_000,
    help="Total training timesteps (default: 500k)",
)
parser.add_argument(
    "--render", action="store_true",
    help="Render during training (slow, for debugging)",
)
parser.add_argument(
    "--seed", type=int, default=42,
    help="Random seed (default: 42)",
)
parser.add_argument(
    "--lr", type=float, default=3e-4,
    help="Learning rate (default: 3e-4)",
)
parser.add_argument(
    "--output", type=str, default=None,
    help="Output model path (default: models/<algo>_<stage>_<target>.zip)",
)
parser.add_argument(
    "--resume", type=str, default=None,
    help="Resume training from a checkpoint (path to .zip file)",
)
parser.add_argument(
    "--checkpoint-freq", type=int, default=100_000,
    help="Save checkpoint every N timesteps (default: 100k)",
)
args = parser.parse_args()

_ALGO = args.algo.upper()

# ── imports after path setup ───────────────────────────────────────
try:
    from stable_baselines3 import PPO, SAC
    from stable_baselines3.common.callbacks import BaseCallback, EvalCallback
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
    from stable_baselines3.common.noise import NormalActionNoise
except ImportError:
    print("stable-baselines3 is required. Install with:")
    print("  pip install stable-baselines3")
    sys.exit(1)

try:
    from tqdm import tqdm
except ImportError:
    print("tqdm is required. Install with: pip install tqdm")
    sys.exit(1)

try:
    from rich.console import Console
    from rich.table import Table
    from rich.panel import Panel
    from rich.progress import (
        Progress, SpinnerColumn, BarColumn, TextColumn,
        TimeElapsedColumn, TimeRemainingColumn,
    )
    RICH_AVAILABLE = True
except ImportError:
    RICH_AVAILABLE = False

from stretch_sim.rl_env import StretchPickEnv, StretchReachEnv, StretchGraspEnv

# ── output path ────────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parents[1]
MODELS_DIR = ROOT / "models"
MODELS_DIR.mkdir(exist_ok=True)

if args.output is None:
    stage_tag = f"{args.algo}_{args.stage}_{args.target}"
    args.output = str(MODELS_DIR / f"{stage_tag}.zip")

# ── environment dispatch ───────────────────────────────────────────
_ENV_CLASSES = {
    "approach": StretchReachEnv,
    "grasp": StretchGraspEnv,
    "pick": StretchPickEnv,
}
ENV_CLASS = _ENV_CLASSES[args.stage]
ENV_KWARGS = {"tomato_name": args.target, "max_episode_steps": 200}
if args.stage == "approach":
    ENV_KWARGS["max_episode_steps"] = 120
elif args.stage == "grasp":
    ENV_KWARGS["max_episode_steps"] = 150

# ── algorithm-specific defaults ────────────────────────────────────
_ALGO_DEFAULTS = {
    "PPO": {
        "policy": "MlpPolicy",
        "policy_kwargs": {"net_arch": [256, 256]},
        "learning_rate": args.lr,
        "n_steps": 2048,
        "batch_size": 64,
        "n_epochs": 10,
        "gamma": 0.99,
        "gae_lambda": 0.95,
        "clip_range": 0.2,
        "ent_coef": 0.02,
        "vf_coef": 0.5,
        "max_grad_norm": 0.5,
        "use_sde": False,
    },
    "SAC": {
        "policy": "MlpPolicy",
        "learning_rate": 3e-4,
        "buffer_size": 100_000,
        "learning_starts": 5_000,
        "batch_size": 256,
        "tau": 0.005,
        "gamma": 0.99,
        "train_freq": 1,
        "gradient_steps": 1,
        "ent_coef": "auto",
        "use_sde": False,
        "policy_kwargs": {"net_arch": [256, 256]},
    },
}
_ALGO_KWARGS = _ALGO_DEFAULTS[_ALGO]

# PPO benefits from VecNormalize; SAC can use it but is more robust without
_USE_VECNORM = (_ALGO == "PPO")

# ── custom progress callback ───────────────────────────────────────
class RichProgressCallback(BaseCallback):
    """Training progress callback — renders bar, speed, reward, ETA in terminal."""

    def __init__(self, total_timesteps: int, verbose: int = 0):
        super().__init__(verbose)
        self._total = total_timesteps
        self._start_time = None
        self._last_timestep = 0
        self._episode_rewards = []
        self._episode_lengths = []

    def _on_training_start(self):
        self._start_time = time.time()

        if RICH_AVAILABLE:
            self._console = Console()
            self._progress = Progress(
                SpinnerColumn(),
                TextColumn("[bold blue]{task.description}"),
                BarColumn(bar_width=40),
                TextColumn("[progress.percentage]{task.percentage:>4.0f}%"),
                TextColumn("│"),
                TextColumn("{task.fields[steps]}"),
                TextColumn("│"),
                TimeElapsedColumn(),
                TextColumn("<"),
                TimeRemainingColumn(),
                console=self._console,
                expand=False,
            )
            self._task = self._progress.add_task(
                f"[cyan]{_ALGO} Training",
                total=self._total,
                steps="0 steps",
            )
            self._progress.start()
        else:
            self._pbar = tqdm(
                total=self._total, unit="steps", desc=f"{_ALGO} Training",
                ncols=100,
                bar_format="{l_bar}{bar}| {n_fmt}/{total_fmt} [{elapsed}<{remaining}, {rate_fmt}]",
            )

    def _on_step(self) -> bool:
        step_delta = self.num_timesteps - self._last_timestep
        self._last_timestep = self.num_timesteps

        try:
            # Pierce through env wrappers to reach Monitor
            base_env = self.training_env
            if _USE_VECNORM:
                base_env = base_env.venv
            monitor = base_env.envs[0]
            if hasattr(monitor, "get_episode_rewards"):
                rewards = monitor.get_episode_rewards()
                if rewards:
                    self._episode_rewards = rewards
        except Exception:
            pass

        completed = min(self.num_timesteps, self._total)
        elapsed = time.time() - self._start_time if self._start_time else 1
        speed = self.num_timesteps / elapsed if elapsed > 0 else 0

        if RICH_AVAILABLE:
            status_parts = [f"speed:{speed:5.0f} step/s"]
            if self._episode_rewards:
                recent = self._episode_rewards[-5:]
                avg_r = sum(recent) / len(recent)
                status_parts.append(f"avg:{avg_r:+6.1f}")
                status_parts.append(f"best:{max(self._episode_rewards):+6.1f}")
            status = "  ".join(status_parts)

            self._progress.update(
                self._task,
                completed=completed,
                steps=f"{completed:,}/{self._total:,}",
                description=f"[cyan]{status}",
            )
        else:
            self._pbar.update(step_delta)
            if self._episode_rewards:
                recent_avg = sum(self._episode_rewards[-5:]) / max(len(self._episode_rewards[-5:]), 1)
                self._pbar.set_postfix({
                    "avg": f"{recent_avg:+.1f}",
                    "best": f"{max(self._episode_rewards):+.1f}",
                })

        return True

    def _on_training_end(self):
        if RICH_AVAILABLE:
            self._progress.stop()
        else:
            self._pbar.close()

        elapsed = time.time() - (self._start_time or time.time())
        mins, secs = divmod(elapsed, 60)
        hrs, mins = divmod(mins, 60)

        print()
        print("=" * 60)
        print(f"  TRAINING COMPLETE".center(60))
        print("=" * 60)
        print(f"  Algorithm:    {_ALGO}")
        print(f"  Total steps:  {self.num_timesteps:,}")
        print(f"  Duration:     {int(hrs)}h {int(mins)}m {secs:.0f}s")
        print(f"  Episodes:     {len(self._episode_rewards)}")
        if self._episode_rewards:
            print(f"  Avg reward:   {sum(self._episode_rewards)/len(self._episode_rewards):+.1f}")
            print(f"  Best reward:  {max(self._episode_rewards):+.1f}")
            if len(self._episode_rewards) >= 10:
                print(f"  Last 10 ep:   {sum(self._episode_rewards[-10:])/10:+.1f}")
        print("=" * 60)


class CheckpointCallback(BaseCallback):
    """Periodically save model so training can be resumed."""

    def __init__(self, save_path: str, train_env, save_freq: int, verbose: int = 0):
        super().__init__(verbose)
        self._save_path = save_path
        self._train_env = train_env
        self._save_freq = save_freq

    def _on_step(self) -> bool:
        if self.num_timesteps > 0 and self.num_timesteps % self._save_freq == 0:
            self.model.save(self._save_path)
            if _USE_VECNORM:
                norm_path = self._save_path.replace(".zip", "_vecnormalize.pkl")
                self._train_env.save(norm_path)
            tag = f"  ✓ Checkpoint saved at {self.num_timesteps:,} steps"
            if RICH_AVAILABLE:
                self._console = getattr(self, '_console', Console())
                self._console.print(f"[green]{tag}[/green]")
            else:
                print(tag)
        return True


# ── environment factory ────────────────────────────────────────────
def make_env(render: bool = False, seed: int = 42):
    env = ENV_CLASS(render_mode="human" if render else None, **ENV_KWARGS)
    env = Monitor(env)
    return env


# ── setup display ──────────────────────────────────────────────────
console = Console() if RICH_AVAILABLE else None
algo_title = "PPO (on-policy)" if _ALGO == "PPO" else "SAC (off-policy)"

if RICH_AVAILABLE:
    console.print()
    table = Table(title=f"🚀 {_ALGO} Training — Stretch 3 Arm Pick", show_header=False)
    table.add_column(style="bold cyan")
    table.add_column()
    table.add_row("Algorithm", algo_title)
    table.add_row("Stage", args.stage)
    table.add_row("Target", args.target)
    table.add_row("Total steps", f"{args.timesteps:,}")
    table.add_row("Learning rate", str(args.lr))
    table.add_row("VecNormalize", "yes" if _USE_VECNORM else "no")
    table.add_row("Output", args.output)
    console.print(table)
    console.print()
else:
    print("=" * 60)
    print(f"{_ALGO} Training — Stretch 3 Arm Pick")
    print(f"  Target:      {args.target}")
    print(f"  Stage:       {args.stage}")
    print(f"  Timesteps:   {args.timesteps:,}")
    print(f"  Output:      {args.output}")
    print("=" * 60)
    print()

# ── envs ───────────────────────────────────────────────────────────
train_env = DummyVecEnv([lambda: make_env(render=args.render, seed=args.seed)])
if _USE_VECNORM:
    train_env = VecNormalize(train_env, norm_obs=True, norm_reward=True, clip_obs=10.0)

eval_env = DummyVecEnv([lambda: make_env(render=False, seed=args.seed + 1000)])
if _USE_VECNORM:
    eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=True, clip_obs=10.0)

# ── model ──────────────────────────────────────────────────────────
_ALGO_CLASS = PPO if _ALGO == "PPO" else SAC

if args.resume:
    if RICH_AVAILABLE:
        console.print(f"[cyan]Resuming from: {args.resume}[/cyan]")
    else:
        print(f"Resuming from: {args.resume}")
    model = _ALGO_CLASS.load(args.resume, env=train_env)

    if _USE_VECNORM:
        norm_stat_path = args.resume.replace(".zip", "_vecnormalize.pkl")
        if os.path.exists(norm_stat_path):
            from stable_baselines3.common.vec_env import VecNormalize as _VN
            train_env = _VN.load(norm_stat_path, train_env)
            eval_env = _VN.load(norm_stat_path, eval_env)
            if RICH_AVAILABLE:
                console.print("[green]✓ VecNormalize stats loaded[/green]")
else:
    # Add action noise for SAC exploration (helps with continuous control)
    if _ALGO == "SAC":
        n_actions = ENV_CLASS(**ENV_KWARGS).action_space.shape[-1]
        _ALGO_KWARGS["action_noise"] = NormalActionNoise(
            mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions)
        )

    model = _ALGO_CLASS(
        **_ALGO_KWARGS,
        env=train_env,
        verbose=0,
        tensorboard_log=str(MODELS_DIR / "tensorboard"),
        seed=args.seed,
    )

# ── callbacks ──────────────────────────────────────────────────────
eval_callback = EvalCallback(
    eval_env,
    best_model_save_path=str(MODELS_DIR),
    log_path=str(MODELS_DIR),
    eval_freq=max(20_000, args.timesteps // 25),
    n_eval_episodes=10,
    deterministic=True,
    render=False,
)

progress_callback = RichProgressCallback(total_timesteps=args.timesteps)
checkpoint_callback = CheckpointCallback(
    save_path=args.output,
    train_env=train_env,
    save_freq=min(args.checkpoint_freq, args.timesteps),
)

from stable_baselines3.common.callbacks import CallbackList
callback = CallbackList([progress_callback, eval_callback, checkpoint_callback])

# ── train ──────────────────────────────────────────────────────────
try:
    model.learn(
        total_timesteps=args.timesteps,
        callback=callback,
        progress_bar=False,
    )
except KeyboardInterrupt:
    if RICH_AVAILABLE:
        console.print()
        console.print("[yellow]Interrupted — saving checkpoint...[/yellow]")
    else:
        print("\nInterrupted — saving checkpoint...")
    model.save(args.output)
    if _USE_VECNORM:
        train_env.save(args.output.replace(".zip", "_vecnormalize.pkl"))
    resume_cmd = (f"python scripts/train_rl_arm.py --algo {args.algo} "
                  f"--stage {args.stage} --target {args.target} --resume {args.output}")
    if RICH_AVAILABLE:
        console.print(f"[green]✓ Saved: {args.output}[/green]")
        console.print(f"[yellow]Resume:[/yellow] [cyan]{resume_cmd}[/cyan]")
    else:
        print(f"✓ Saved: {args.output}")
        print(f"Resume: {resume_cmd}")
    sys.exit(0)

# ── save ───────────────────────────────────────────────────────────
model.save(args.output)
if _USE_VECNORM:
    normalize_path = args.output.replace(".zip", "_vecnormalize.pkl")
    train_env.save(normalize_path)

if RICH_AVAILABLE:
    console.print()
    norm_info = f"[green]VecNormalize:[/green] {normalize_path}\n\n" if _USE_VECNORM else ""
    console.print(Panel.fit(
        f"[green]Model:[/green] {args.output}\n{norm_info}"
        f"[bold]Usage:[/bold]\n"
        f"  1. [cyan]make sim[/cyan]\n"
        f"  2. [cyan]make controller[/cyan]\n"
        f"  3. [cyan]rl_get_tomato[/cyan]",
        title="Training Complete",
    ))
else:
    print(f"\n✓ Model saved to: {args.output}")
    if _USE_VECNORM:
        print(f"✓ Normalization stats saved to: {normalize_path}")
    print(f"\nTo use:")
    print(f"  1. make sim")
    print(f"  2. make controller")
    print(f"  3. rl_get_tomato")
