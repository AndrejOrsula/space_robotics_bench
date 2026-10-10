"""Batched TD-MPC2 agent and trainer for SRB environments."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from tdmpc2 import TDMPC2
from tensordict.tensordict import TensorDict
from trainer.online_trainer import OnlineTrainer

if TYPE_CHECKING:
    from isaacsim.simulation_app import SimulationApp

    from srb.integrations.tdmpc2.wrapper import SrbTdmpc2Wrapper


class BatchedTDMPC2(TDMPC2):
    """Support batched observations and MPPI planning across environments."""

    def __init__(self, cfg):
        super().__init__(cfg)
        self.num_envs: int = getattr(cfg, "num_envs", 1)
        self._prev_mean_dict: dict[int, torch.Tensor] = {}

    # ------------------------------------------------------------------
    # Batched value estimation
    # ------------------------------------------------------------------

    @torch.no_grad()
    def _estimate_value_batched(self, z, actions, task):
        """Estimate trajectory value for batched latent states and actions."""
        import common.math as tmath

        G, discount = 0, 1
        termination = torch.zeros(z.shape[0], 1, dtype=torch.float32, device=z.device)
        for t in range(self.cfg.horizon):
            reward = tmath.two_hot_inv(self.model.reward(z, actions[t], task), self.cfg)
            z = self.model.next(z, actions[t], task)
            G = G + discount * (1 - termination) * reward
            discount_update = (
                self.discount[torch.tensor(task)]
                if self.cfg.multitask
                else self.discount
            )
            discount = discount * discount_update
            if self.cfg.episodic:
                termination = torch.max(
                    termination,
                    (self.model.termination(z, task) > 0.5).float(),
                )
        action, _ = self.model.pi(z, task)
        return G + discount * (1 - termination) * self.model.Q(
            z, action, task, return_type="avg"
        )

    # ------------------------------------------------------------------
    # Batched action selection
    # ------------------------------------------------------------------

    @torch.no_grad()
    def act(self, obs, t0, eval_mode=False, task=None):
        """Select actions for batched observations."""
        if isinstance(obs, dict):
            obs = {k: v.to(self.device, non_blocking=True) for k, v in obs.items()}
        else:
            obs = obs.to(self.device, non_blocking=True)

        if task is not None:
            task = torch.tensor([task], device=self.device)
        if self.cfg.mpc:
            return self._plan_batched(obs, t0=t0, eval_mode=eval_mode, task=task)
        z = self.model.encode(obs, task)
        action, info = self.model.pi(z, task)
        if eval_mode:
            action = info["mean"]
        return action

    # ------------------------------------------------------------------
    # Batched MPPI planning
    # ------------------------------------------------------------------

    @torch.no_grad()
    def _plan_batched(self, obs, t0, eval_mode=False, task=None):
        if isinstance(obs, dict):
            B = next(iter(obs.values())).shape[0]
        else:
            B = obs.shape[0]
        z = self.model.encode(obs, task)

        # Keep planner history per batch size.
        if B not in self._prev_mean_dict:
            self._prev_mean_dict[B] = torch.zeros(
                B,
                self.cfg.horizon,
                self.cfg.action_dim,
                device=self.device,
            )
        prev_mean = self._prev_mean_dict[B]

        # Sample policy trajectories.
        if self.cfg.num_pi_trajs > 0:
            pi_actions = torch.empty(
                self.cfg.horizon,
                B,
                self.cfg.num_pi_trajs,
                self.cfg.action_dim,
                device=self.device,
            )
            _z = (
                z.unsqueeze(1)
                .repeat(1, self.cfg.num_pi_trajs, 1)
                .view(B * self.cfg.num_pi_trajs, -1)
            )
            for t in range(self.cfg.horizon - 1):
                act, _ = self.model.pi(_z, task)
                pi_actions[t] = act.view(B, self.cfg.num_pi_trajs, self.cfg.action_dim)
                _z = self.model.next(_z, act, task)
            act, _ = self.model.pi(_z, task)
            pi_actions[-1] = act.view(B, self.cfg.num_pi_trajs, self.cfg.action_dim)

        # Expand latent states across samples.
        z = (
            z.unsqueeze(1)
            .repeat(1, self.cfg.num_samples, 1)
            .view(B * self.cfg.num_samples, -1)
        )

        mean = torch.zeros(B, self.cfg.horizon, self.cfg.action_dim, device=self.device)
        std = torch.full(
            (B, self.cfg.horizon, self.cfg.action_dim),
            self.cfg.max_std,
            dtype=torch.float,
            device=self.device,
        )

        # Shift planner history unless each row starts a new episode.
        if isinstance(t0, torch.Tensor) and t0.numel() == B:
            mask = (~t0).view(B, 1, 1)
            mean[:, :-1] = torch.where(mask, prev_mean[:, 1:], mean[:, :-1])
        elif isinstance(t0, bool) and not t0:
            mean[:, :-1] = prev_mean[:, 1:]

        actions = torch.empty(
            self.cfg.horizon,
            B,
            self.cfg.num_samples,
            self.cfg.action_dim,
            device=self.device,
        )
        if self.cfg.num_pi_trajs > 0:
            actions[:, :, : self.cfg.num_pi_trajs] = pi_actions

        # Run MPPI updates.
        for _ in range(self.cfg.iterations):
            r = torch.randn(
                self.cfg.horizon,
                B,
                self.cfg.num_samples - self.cfg.num_pi_trajs,
                self.cfg.action_dim,
                device=std.device,
            )
            mean_expanded = mean.transpose(0, 1).unsqueeze(2)  # (H, B, 1, A)
            std_expanded = std.transpose(0, 1).unsqueeze(2)  # (H, B, 1, A)
            actions_sample = (mean_expanded + std_expanded * r).clamp(-1, 1)
            actions[:, :, self.cfg.num_pi_trajs :] = actions_sample

            actions_flat = actions.view(
                self.cfg.horizon,
                B * self.cfg.num_samples,
                self.cfg.action_dim,
            )
            value_flat = self._estimate_value_batched(z, actions_flat, task).nan_to_num(
                0
            )
            value = value_flat.view(B, self.cfg.num_samples)

            # Select elite trajectories.
            elite_idxs = torch.topk(value, self.cfg.num_elites, dim=1).indices  # (B, E)
            elite_value = value.gather(1, elite_idxs)  # (B, E)

            idx_exp = (
                elite_idxs.unsqueeze(0)
                .unsqueeze(-1)
                .expand(
                    self.cfg.horizon,
                    B,
                    self.cfg.num_elites,
                    self.cfg.action_dim,
                )
            )
            elite_actions = actions.gather(2, idx_exp)  # (H, B, E, A)

            # Update the action distribution.
            max_value = elite_value.max(dim=1, keepdim=True).values
            score = torch.exp(self.cfg.temperature * (elite_value - max_value))
            score = score / score.sum(dim=1, keepdim=True)

            score_exp = score.unsqueeze(0).unsqueeze(-1)
            mean = (score_exp * elite_actions).sum(dim=2) / (
                score_exp.sum(dim=2) + 1e-9
            )
            mean = mean.transpose(0, 1)  # (B, H, A)

            std = (
                (
                    score_exp * (elite_actions - mean.transpose(0, 1).unsqueeze(2)) ** 2
                ).sum(dim=2)
                / (score_exp.sum(dim=2) + 1e-9)
            ).sqrt()
            std = std.transpose(0, 1).clamp(self.cfg.min_std, self.cfg.max_std)

            if self.cfg.multitask:
                mean = mean * self.model._action_masks[task]
                std = std * self.model._action_masks[task]

        # Select the final action.
        import common.math as tmath

        rand_idx = tmath.gumbel_softmax_sample(score, dim=1)  # (B,)
        rand_idx_exp = rand_idx.view(1, B, 1, 1).expand(
            self.cfg.horizon, B, 1, self.cfg.action_dim
        )
        actions = elite_actions.gather(2, rand_idx_exp).squeeze(2)  # (H, B, A)
        a, std_sel = actions[0], std[:, 0, :]  # (B, A)

        if not eval_mode:
            a = a + std_sel * torch.randn(B, self.cfg.action_dim, device=std_sel.device)

        self._prev_mean_dict[B].copy_(mean)
        return a.clamp(-1, 1)


class BatchedOnlineTrainer(OnlineTrainer):
    """Trainer for batched multi-environment TD-MPC2 training.

    Keeps one in-progress sequence per vector row. SRB autoresets in ``step``;
    policy gets post-reset observations; replay stores final pre-reset ones.
    """

    def __init__(
        self,
        cfg,
        env: "SrbTdmpc2Wrapper",
        agent: BatchedTDMPC2,
        buffer,
        logger,
        sim_app: "SimulationApp",
    ):
        super().__init__(cfg=cfg, env=env, agent=agent, buffer=buffer, logger=logger)
        self.sim_app = sim_app

    # ------------------------------------------------------------------
    # TensorDict construction
    # ------------------------------------------------------------------

    def to_td(self, obs, action=None, reward=None, terminated=None):
        num_envs = self.env.num_envs
        if isinstance(obs, dict):
            obs = TensorDict(obs, batch_size=(num_envs,), device=self.agent.device)
        elif not isinstance(obs, torch.Tensor):
            obs = torch.as_tensor(obs, device=self.agent.device)
        else:
            obs = obs.to(self.agent.device)

        if action is None:
            action = torch.full_like(
                self.env.rand_act(),
                float("nan"),
                device=self.agent.device,
            )
        if reward is None:
            reward = torch.full((num_envs,), float("nan"), device=self.agent.device)
        if terminated is None:
            terminated = torch.full((num_envs,), float("nan"), device=self.agent.device)

        return TensorDict(
            obs=obs,
            action=action,
            reward=reward,
            terminated=terminated,
            batch_size=(num_envs,),
            device=self.agent.device,
        )

    def _split_td(self, td: TensorDict) -> list[TensorDict]:
        """Return independent scalar-row views for each environment."""
        return [td[index].clone() for index in range(self.env.num_envs)]

    def _transition_observation(
        self,
        obs: torch.Tensor,
        info: dict[str, torch.Tensor],
        done: torch.Tensor,
    ) -> torch.Tensor:
        """Replace ended rows with the validated pre-reset final snapshot."""
        if not bool(done.any()):
            return obs

        final_observation = info.get("final_observation")
        final_ids_raw = info.get("final_observation_ids")
        final_mask_raw = info.get("final_observation_mask")
        if final_observation is None or final_ids_raw is None or final_mask_raw is None:
            raise RuntimeError(
                "TD-MPC2 done transition has no complete final_observation "
                "snapshot (expected final_observation, ids, and mask)"
            )

        final_ids = torch.as_tensor(
            final_ids_raw, device=self.agent.device, dtype=torch.long
        ).reshape(-1)
        expected_ids = done.nonzero(as_tuple=False).flatten()
        if not torch.equal(final_ids, expected_ids):
            raise RuntimeError(
                "TD-MPC2 final_observation_ids do not match the done mask"
            )
        final_mask = torch.as_tensor(
            final_mask_raw, device=self.agent.device, dtype=torch.bool
        ).reshape(-1)
        if final_mask.shape != done.shape or not torch.equal(final_mask, done):
            raise RuntimeError(
                "TD-MPC2 final_observation_mask does not match the done mask"
            )

        final = torch.as_tensor(final_observation, device=self.agent.device)
        if final.ndim == obs.ndim and tuple(final.shape) == tuple(obs.shape):
            rows = final.index_select(0, final_ids)
        elif (
            final.ndim == obs.ndim
            and final.shape[0] == final_ids.numel()
            and tuple(final.shape[1:]) == tuple(obs.shape[1:])
        ):
            rows = final
        else:
            trailing = ", ".join(map(str, obs.shape[1:]))
            raise RuntimeError(
                "TD-MPC2 final_observation shape does not match the live "
                f"observation: got {tuple(final.shape)}, expected either "
                f"{tuple(obs.shape)} or ({final_ids.numel()}, {trailing})"
            )

        transition_obs = obs.clone()
        # The live observation keeps the environment's device.  The validated
        # ids were normalized to the agent device for comparison/selection,
        # so move only the index used by this in-place update to the live
        # observation device; never force CPU simulator observations onto the
        # agent/GPU boundary here.
        transition_ids = final_ids.to(device=transition_obs.device)
        transition_obs.index_copy_(0, transition_ids, rows.to(obs))
        return transition_obs

    def _store_episode(self, rows: list[TensorDict]) -> int:
        """Store one complete ``(T,)`` sequence using upstream's ``add`` API."""
        if len(rows) < 2:
            raise RuntimeError("TD-MPC2 replay sequence has no transition")
        episode = torch.stack(rows, dim=0)
        # Qualified upstream Buffer.add expects one TensorDict with batch shape
        # ``(T,)`` and assigns one trajectory id. ``load`` is for an already
        # rectangular ``(N,T)`` batch; asynchronous rows cannot be packed into
        # that shape without padding transitions into the wrong episode.
        return int(self.buffer.add(episode))

    @staticmethod
    def _episode_reward(rows: list[TensorDict]) -> float:
        rewards = torch.stack([row["reward"] for row in rows[1:]])
        return float(torch.nansum(rewards).item())

    def _log_completed(
        self,
        completed: list[dict[str, float | int]],
        train_metrics: dict,
    ) -> None:
        if not completed:
            return
        train_metrics.update(
            episode_reward=sum(float(row["reward"]) for row in completed)
            / len(completed),
            episode_length=sum(int(row["length"]) for row in completed)
            / len(completed),
            episode_terminated=sum(float(row["terminated"]) for row in completed)
            / len(completed),
        )
        # Success is an optional task signal. Do not retain a previous
        # episode's value when the current environment omitted it.
        train_metrics.pop("episode_success", None)
        successes = [row["success"] for row in completed if "success" in row]
        if len(successes) == len(completed):
            train_metrics["episode_success"] = sum(
                float(value) for value in successes
            ) / len(successes)
        train_metrics.update(self.common_metrics())
        self.logger.log(train_metrics, "train")

    # ------------------------------------------------------------------
    # Training loop
    # ------------------------------------------------------------------

    def train(self):
        train_metrics = {}
        num_envs = self.env.num_envs
        self._pretrained = False

        obs = self.env.reset()
        self._tds = [[row] for row in self._split_td(self.to_td(obs))]
        episode_starts = torch.ones(
            num_envs, dtype=torch.bool, device=self.agent.device
        )

        while self._step < self.cfg.steps:
            # Graceful shutdown on sim exit
            if not self.sim_app.is_running():
                break

            # Collect experience
            if self._step > self.cfg.seed_steps:
                action = self.agent.act(obs, t0=episode_starts.clone())
                # Random seed actions do not advance the planner history. Keep
                # the start mask intact until the first planner action for
                # each row, then clear it for the following action.
                episode_starts.zero_()
            else:
                action = self.env.rand_act()

            obs, reward, dones, info = self.env.step(action)
            dones = torch.as_tensor(
                dones, device=self.agent.device, dtype=torch.bool
            ).reshape(-1)
            if dones.numel() != num_envs:
                raise ValueError(
                    "TD-MPC2 environment done mask must contain one value per "
                    f"environment, got shape {tuple(dones.shape)}"
                )
            transition_obs = self._transition_observation(obs, info, dones)

            terminated = torch.as_tensor(
                info["terminated"], device=self.agent.device, dtype=torch.float32
            ).reshape(-1)
            if terminated.numel() != num_envs:
                raise ValueError(
                    "TD-MPC2 terminated signal must contain one value per "
                    f"environment, got shape {tuple(terminated.shape)}"
                )
            if not bool(torch.isfinite(terminated).all()) or bool(
                ((terminated != 0) & (terminated != 1)).any()
            ):
                raise ValueError(
                    "TD-MPC2 terminated signal must contain only 0/1 values"
                )
            if bool(((terminated > 0) & ~dones).any()):
                raise RuntimeError(
                    "TD-MPC2 terminated signal marks an environment that did not end"
                )
            if bool((dones & (terminated > 0)).any()) and not self.cfg.episodic:
                raise ValueError(
                    "Termination detected but you are not in episodic mode. "
                    "Set `episodic=true` to enable support for terminations."
                )

            transition_td = self.to_td(
                transition_obs,
                action,
                reward,
                info["terminated"],
            )
            post_reset_td = self.to_td(obs)
            completed: list[dict[str, float | int]] = []
            success = info.get("success")
            success_values = None
            if success is not None:
                success_values = torch.as_tensor(
                    success, device=self.agent.device, dtype=torch.float32
                ).reshape(-1)
                if success_values.numel() != num_envs:
                    raise ValueError(
                        "TD-MPC2 success signal must contain one value per "
                        f"environment, got shape {tuple(success_values.shape)}"
                    )

            for index in range(num_envs):
                self._tds[index].append(transition_td[index].clone())
                if not bool(dones[index]):
                    continue

                row = {
                    "reward": self._episode_reward(self._tds[index]),
                    "length": len(self._tds[index]) - 1,
                    "terminated": float(terminated[index].item()),
                }
                if success_values is not None:
                    row["success"] = float(success_values[index].item())
                self._ep_idx = self._store_episode(self._tds[index])
                completed.append(row)
                # The live observation for a completed row is already the
                # first post-reset state. Keep it as t=0 of the next sequence.
                self._tds[index] = [post_reset_td[index].clone()]
                episode_starts[index] = True

            if completed:
                self._log_completed(completed, train_metrics)

            # Update agent
            if self._step >= self.cfg.seed_steps and self.buffer.num_eps > 0:
                if not self._pretrained:
                    num_updates = self.cfg.seed_steps
                    print(f"Pretraining agent on seed data for {num_updates} steps...")
                    self._pretrained = True
                else:
                    num_updates = num_envs

                for _ in range(num_updates):
                    _train_metrics = self.agent.update(self.buffer)
                train_metrics.update(_train_metrics)

            previous_step = self._step
            self._step += num_envs
            # Periodic checkpoints follow the step counter, not episode
            # completions: save whenever this vector step crossed an
            # ``eval_freq`` boundary, named by the env steps completed so far.
            if self._step // self.cfg.eval_freq > previous_step // self.cfg.eval_freq:
                self.logger.save_agent(self.agent, identifier=f"{self._step}")

        self.logger.finish(self.agent)

    # ------------------------------------------------------------------
    # Evaluation
    # ------------------------------------------------------------------
