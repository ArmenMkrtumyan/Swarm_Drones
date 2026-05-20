"""Adapter from :class:`HoverPretrainVecEnv` to SB3's ``VecEnv`` API.

SB3's ``VecEnv`` expects:
  - ``num_envs`` int
  - single-env ``observation_space`` and ``action_space``
  - ``reset()`` -> obs (np.ndarray shape (num_envs, ...))
  - ``step(actions)`` -> obs, rewards, dones, infos
  - ``close()``
  - a few introspection helpers (``env_is_wrapped``, ``get_attr``, ...)

Our :class:`HoverPretrainVecEnv` already returns batched numpy at the boundary
and exposes the right spaces; this thin adapter satisfies the SB3 abstract
methods so the trainer can pass it to ``TD3(..., env=vec_env)`` directly.
"""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np
from stable_baselines3.common.vec_env import VecEnv

from lab.rl.envs.hover_pretrain_v0 import (
    HoverPretrainVecEnv, PretrainConfig,
)


class HoverPretrainSB3VecEnv(VecEnv):
    """Make a :class:`HoverPretrainVecEnv` look like an SB3 ``VecEnv``."""

    def __init__(self, cfg: PretrainConfig | None = None) -> None:
        self._inner = HoverPretrainVecEnv(cfg)
        super().__init__(
            num_envs=self._inner.num_envs,
            observation_space=self._inner.observation_space,
            action_space=self._inner.action_space,
        )
        self._actions: np.ndarray | None = None

    # ------------------------------------------------------ reset / step API

    def reset(self) -> np.ndarray:
        obs, _ = self._inner.reset()
        return obs.astype(np.float32, copy=False)

    def step_async(self, actions: np.ndarray) -> None:
        self._actions = actions

    def step_wait(self):
        assert self._actions is not None, "step_async must be called first"
        obs, reward, terminated, truncated, infos = self._inner.step(self._actions)
        # SB3 expects a single done array; gymnasium splits terminated/truncated.
        # Convention: `done = terminated | truncated`. Pass `TimeLimit.truncated`
        # in info so SB3 bootstrapping is correct.
        done = np.logical_or(terminated, truncated)
        info_list: list[dict] = []
        for i in range(self.num_envs):
            d = dict(infos[i]) if isinstance(infos[i], dict) else {}
            if truncated[i] and not terminated[i]:
                d["TimeLimit.truncated"] = True
            info_list.append(d)
        return (obs.astype(np.float32, copy=False),
                reward.astype(np.float32, copy=False),
                done,
                info_list)

    def close(self) -> None:
        self._inner.close()

    # ----------------------------------------------------------- introspection

    def get_attr(self, attr_name: str, indices=None) -> list[Any]:
        v = getattr(self._inner, attr_name)
        return [v] * self.num_envs

    def set_attr(self, attr_name: str, value: Any, indices=None) -> None:
        setattr(self._inner, attr_name, value)

    def env_method(self, method_name: str, *args,
                   indices=None, **method_kwargs) -> list[Any]:
        m = getattr(self._inner, method_name)
        return [m(*args, **method_kwargs)] * self.num_envs

    def env_is_wrapped(self, wrapper_class, indices=None) -> list[bool]:
        return [False] * self.num_envs

    def seed(self, seed: int | None = None) -> Sequence[int | None]:
        # Gymnasium seeding goes through reset()'s seed kwarg.
        if seed is not None:
            self._inner.reset(seed=seed)
        return [seed] * self.num_envs
