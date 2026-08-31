"""
multi_agent_deadlock_safety.py

Deadlock-aware wrapper around the central winner/yielder safety policy.

This layer detects stalled rover clusters, especially near the target zone,
chooses a less-constrained rover as the winner, freezes the others as temporary
obstacles, and gives the winner a direct escape/recovery command. If the winner
does not make progress, it may back up only when the reverse sweep is clear.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Dict, Optional, Sequence, Set, Tuple

import numpy as np

from multi_agent_collision_safety import (
    AgentBehavior,
    ClusterEpisode,
    EpisodePhase,
    MultiAgentCollisionSafety,
    PolicyConfig,
    SafetyAgentContext,
    dist,
    wrap_to_pi,
)

from target_zones import resolve_target_zone

Vec2 = Tuple[float, float]


@dataclass
class DeadlockPolicyConfig(PolicyConfig):
    target_zone_radius: float = 0.8
    target_zone: Any = None
    target_zone_buffer: float = 0.22
    deadlock_pair_radius: float = 1.35
    deadlock_target_band: float = 0.95
    deadlock_speed_thresh: float = 0.045
    deadlock_motion_epsilon: float = 0.025
    deadlock_duration: float = 1.8
    deadlock_release_dist: float = 1.65
    enable_reverse_recovery: bool = True
    deadlock_winner_reverse_after: float = 1.2
    reverse_speed: float = -0.38
    reverse_distance: float = 0.58
    reverse_clearance: float = 0.78
    escape_step_distance: float = 0.85
    target_repulsion_gain: float = 1.25
    rover_repulsion_gain: float = 0.75


def _xy(ctx: SafetyAgentContext) -> Vec2:
    return float(ctx.state[0]), float(ctx.state[1])


def _norm(v: np.ndarray) -> float:
    return float(np.linalg.norm(v))


def _unit(v: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    n = _norm(v)
    if n < 1e-9:
        f = np.array(fallback, dtype=float)
        fn = _norm(f)
        if fn < 1e-9:
            return np.array([1.0, 0.0], dtype=float)
        return f / fn
    return v / n


def _command_to_point(
    state: np.ndarray,
    target: Vec2,
    speed_scale: float,
    v_max: float = 1.0,
    w_max: float = 7.0,
) -> Tuple[float, float]:
    x, y, yaw, v_fwd, _ = state
    dx = float(target[0]) - float(x)
    dy = float(target[1]) - float(y)
    d = math.hypot(dx, dy)
    if d < 0.05:
        return 0.0, 0.0

    theta_des = math.atan2(dy, dx)
    e_theta = wrap_to_pi(theta_des - float(yaw))
    if abs(float(v_fwd)) < 0.03 and abs(e_theta) > math.radians(55.0):
        return 0.0, math.copysign(min(w_max, 18.0), e_theta)

    align = max(0.0, math.cos(e_theta))
    v_cmd = v_max * speed_scale * (align ** 3) * min(1.0, d / 0.45)
    w_cmd = max(-w_max, min(w_max, 5.5 * e_theta))
    if abs(e_theta) < math.radians(5.0):
        w_cmd = 0.0
    return float(v_cmd), float(w_cmd)


class DeadlockAwareCollisionSafety(MultiAgentCollisionSafety):
    def __init__(self, config: Optional[DeadlockPolicyConfig] = None):
        super().__init__(config or DeadlockPolicyConfig())
        self.config: DeadlockPolicyConfig
        self.target_zone = resolve_target_zone(self.config.target_zone, self.config.target_zone_radius)
        self._last_pos: Dict[int, Vec2] = {}
        self._stalled_since: Dict[int, float] = {}
        self._winner_last_pos: Optional[Vec2] = None
        self._winner_progress_since: float = 0.0
        self._deadlock_episode_key = None
        self._active_deadlock = False
        self._sim_time = 0.0
        self._last_reverse_print = -10.0

    def update(self, contexts: Sequence[SafetyAgentContext], sim_time: float) -> None:
        self._sim_time = sim_time
        contexts = self.with_priorities(contexts)
        self._update_stall_trackers(contexts, sim_time)

        candidate = self._choose_deadlock_episode(contexts, sim_time)
        can_start_deadlock = sim_time >= self.rearm_until and candidate is not None
        if can_start_deadlock and (
            not self.episode.active
            or not self._active_deadlock
            or self.episode.winner_idx != candidate.winner_idx
        ):
            self._activate_deadlock_episode(candidate, sim_time)
        elif not self._active_deadlock:
            super().update(contexts, sim_time)

        if self.episode.active and self.episode.winner_idx is not None:
            self._update_winner_progress(contexts, sim_time)

        if self._active_deadlock and self.episode.active:
            self._update_deadlock_release(contexts, sim_time)

        if self._active_deadlock and not self.episode.active:
            self._active_deadlock = False

    def _activate_deadlock_episode(self, candidate: ClusterEpisode, sim_time: float) -> None:
        candidate.started_at = sim_time
        candidate.stop_triggered_at = sim_time
        candidate.escape_built_at = sim_time
        candidate.winner_escape_field_ready = True
        self.episode = candidate
        self._active_deadlock = True
        self._winner_last_pos = None
        self._winner_progress_since = sim_time
        key = (candidate.winner_idx, tuple(sorted(candidate.yielder_indices)))
        if key != self._deadlock_episode_key:
            self._deadlock_episode_key = key
            print(
                f"[SAFETY] Deadlock recovery: winner=R{candidate.winner_idx}, "
                f"frozen={[f'R{i}' for i in sorted(candidate.yielder_indices)]}"
            )

    def _update_deadlock_release(
        self,
        contexts: Sequence[SafetyAgentContext],
        sim_time: float,
    ) -> None:
        if self.episode.winner_idx is None:
            return
        by_idx = {ctx.idx: ctx for ctx in contexts if ctx.active}
        winner = by_idx.get(self.episode.winner_idx)
        if winner is None:
            self.episode = ClusterEpisode()
            self.rearm_until = sim_time + self.config.rearm_delay
            return

        yielders = [
            by_idx[y_idx]
            for y_idx in self.episode.yielder_indices
            if y_idx in by_idx
        ]
        if not yielders:
            self.episode = ClusterEpisode()
            self.rearm_until = sim_time + self.config.rearm_delay
            return

        min_yielder_dist = min(dist(_xy(winner), _xy(yielder)) for yielder in yielders)
        winner_goal_dist = (
            dist(_xy(winner), winner.goal)
            if winner.goal is not None else float("inf")
        )
        held_long_enough = sim_time - self.episode.started_at >= self.config.min_hold_after_escape
        if held_long_enough and (
            min_yielder_dist >= self.config.deadlock_release_dist
            or winner_goal_dist <= self.config.goal_tol
        ):
            self.episode.phase = EpisodePhase.RELEASE
            self.episode.active = False
            self.rearm_until = sim_time + self.config.rearm_delay
            print("[SAFETY] Deadlock recovery released.")

    def _update_stall_trackers(self, contexts: Sequence[SafetyAgentContext], sim_time: float) -> None:
        active_indices = {ctx.idx for ctx in contexts if ctx.active}
        for idx in list(self._last_pos):
            if idx not in active_indices:
                self._last_pos.pop(idx, None)
                self._stalled_since.pop(idx, None)

        for ctx in contexts:
            if not ctx.active:
                continue
            pos = _xy(ctx)
            last = self._last_pos.get(ctx.idx)
            moved = 0.0 if last is None else dist(pos, last)
            speed = abs(float(ctx.state[3]))

            if last is None:
                self._last_pos[ctx.idx] = pos
                self._stalled_since[ctx.idx] = sim_time
                continue

            if speed > self.config.deadlock_speed_thresh or moved > self.config.deadlock_motion_epsilon:
                self._stalled_since[ctx.idx] = sim_time
                self._last_pos[ctx.idx] = pos

    def _is_stalled(self, ctx: SafetyAgentContext, sim_time: float) -> bool:
        since = self._stalled_since.get(ctx.idx, sim_time)
        speed = abs(float(ctx.state[3]))
        return speed <= self.config.deadlock_speed_thresh and (
            sim_time - since >= self.config.deadlock_duration
        )

    def _target_signed_distance(self, point) -> float:
        return self.target_zone.signed_distance_world(float(point[0]), float(point[1]))

    def _target_outward(self, point) -> np.ndarray:
        return np.array(
            self.target_zone.outward_direction_world(float(point[0]), float(point[1])),
            dtype=float)

    def _near_target_zone(self, ctx: SafetyAgentContext) -> bool:
        return self._target_signed_distance(_xy(ctx)) <= self.config.deadlock_target_band

    def _choose_deadlock_episode(
        self,
        contexts: Sequence[SafetyAgentContext],
        sim_time: float,
    ) -> Optional[ClusterEpisode]:
        candidates = [
            ctx for ctx in contexts
            if ctx.active and self._is_stalled(ctx, sim_time)
        ]
        if len(candidates) < 2:
            return None

        groups = self._stalled_groups(candidates)
        if not groups:
            return None

        group = max(groups, key=len)
        if len(group) < 2:
            return None
        if not any(self._near_target_zone(ctx) for ctx in group):
            return None

        winner = self._choose_deadlock_winner(group)
        if winner is None:
            return None

        yielders = {ctx.idx for ctx in group if ctx.idx != winner.idx}
        if not yielders:
            return None

        return ClusterEpisode(
            active=True,
            phase=EpisodePhase.WINNER_ESCAPE,
            winner_idx=winner.idx,
            yielder_indices=yielders,
        )

    def _stalled_groups(self, candidates: Sequence[SafetyAgentContext]):
        n = len(candidates)
        adj = [[] for _ in range(n)]
        for i in range(n):
            for j in range(i + 1, n):
                if dist(_xy(candidates[i]), _xy(candidates[j])) <= self.config.deadlock_pair_radius:
                    adj[i].append(j)
                    adj[j].append(i)

        seen = [False] * n
        groups = []
        for i in range(n):
            if seen[i]:
                continue
            stack = [i]
            seen[i] = True
            group = []
            while stack:
                u = stack.pop()
                group.append(candidates[u])
                for v in adj[u]:
                    if not seen[v]:
                        seen[v] = True
                        stack.append(v)
            if len(group) >= 2:
                groups.append(group)
        return groups

    def _choose_deadlock_winner(self, group: Sequence[SafetyAgentContext]) -> Optional[SafetyAgentContext]:
        def score(ctx: SafetyAgentContext) -> float:
            x, y = _xy(ctx)
            target_clearance = self._target_signed_distance((x, y))
            goal_dist = dist(_xy(ctx), ctx.goal) if ctx.goal is not None else 2.0
            free_bonus = 100.0 if not ctx.path_constrained else 0.0
            outside_bonus = 10.0 if target_clearance > self.config.target_zone_buffer else 0.0
            return free_bonus + outside_bonus + 0.2 * target_clearance - 0.1 * goal_dist - 0.01 * ctx.idx

        return max(group, key=score) if group else None

    def _update_winner_progress(self, contexts: Sequence[SafetyAgentContext], sim_time: float) -> None:
        by_idx = {ctx.idx: ctx for ctx in contexts}
        winner = by_idx.get(self.episode.winner_idx)
        if winner is None:
            return

        pos = _xy(winner)
        if self._winner_last_pos is None:
            self._winner_last_pos = pos
            self._winner_progress_since = sim_time
            return

        if dist(pos, self._winner_last_pos) > self.config.deadlock_motion_epsilon:
            self._winner_progress_since = sim_time
            self._winner_last_pos = pos

    def behavior_for(self, idx: int, contexts: Sequence[SafetyAgentContext]) -> AgentBehavior:
        return super().behavior_for(idx, contexts)

    def filter_controls(
        self,
        contexts: Sequence[SafetyAgentContext],
        nominal_controls: Dict[int, Tuple[float, float]],
    ) -> Dict[int, Tuple[float, float]]:
        contexts = self.with_priorities(contexts)
        out = super().filter_controls(contexts, nominal_controls)

        if not self.episode.active or self.episode.winner_idx is None:
            return out

        by_idx = {ctx.idx: ctx for ctx in contexts}
        winner = by_idx.get(self.episode.winner_idx)
        if winner is None:
            return out

        behavior = self.behavior_for(winner.idx, contexts)
        if behavior != AgentBehavior.WINNER_ESCAPE_DIRECT:
            return out

        yielders = [
            by_idx[y_idx]
            for y_idx in self.episode.yielder_indices
            if y_idx in by_idx
        ]
        emergency_close = any(
            dist(_xy(winner), _xy(other)) < self.config.emergency_stop_distance
            for other in contexts
            if other.idx != winner.idx and other.active
        )

        # A PUSH/TURN_TO_PUSH winner owns a protected material corridor. Its
        # nominal controller may turn, stop, or reacquire the endpoint, but a
        # free-space escape target can pull it past/off that corridor and leave
        # path progress clamped at the end. Preserve nominal path control here;
        # the cooperative recovery layer can still use bounded on-path assist.
        if self._active_deadlock and not emergency_close and not winner.path_constrained:
            target = self._deadlock_escape_target(winner, yielders)
            out[winner.idx] = _command_to_point(
                winner.state,
                target,
                speed_scale=self.config.winner_escape_speed_scale,
            )

        no_progress_for = self._sim_time - self._winner_progress_since
        v_cmd, w_cmd = out.get(winner.idx, (0.0, 0.0))
        stalled_command = abs(v_cmd) < 0.05 and abs(w_cmd) < 0.15
        if (
            self._active_deadlock
            and self.config.enable_reverse_recovery
            and no_progress_for >= self.config.deadlock_winner_reverse_after
            and (stalled_command or abs(float(winner.state[3])) < self.config.deadlock_speed_thresh)
            and self._reverse_is_safe(winner, contexts)
        ):
            out[winner.idx] = (self.config.reverse_speed, 0.0)
            if self._sim_time - self._last_reverse_print >= 1.0:
                self._last_reverse_print = self._sim_time
                print(f"[SAFETY] {winner.agent_id} guarded reverse recovery.")

        return out

    def _deadlock_escape_target(
        self,
        winner: SafetyAgentContext,
        yielders: Sequence[SafetyAgentContext],
    ) -> Vec2:
        pos = np.array(winner.state[:2], dtype=float)

        if winner.goal is not None:
            goal_vec = np.array(winner.goal, dtype=float) - pos
        else:
            yaw = float(winner.state[2])
            goal_vec = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)

        direction = _unit(goal_vec, np.array([1.0, 0.0], dtype=float))

        for yielder in yielders:
            rel = pos - np.array(yielder.state[:2], dtype=float)
            d = max(_norm(rel), 0.20)
            direction += self.config.rover_repulsion_gain * rel / (d * d)

        target_clearance = self._target_signed_distance(pos)
        if target_clearance < self.config.target_zone_buffer + 0.45:
            outward = self._target_outward(pos)
            direction += self.config.target_repulsion_gain * outward

        direction = _unit(direction, goal_vec)
        target = pos + self.config.escape_step_distance * direction

        target_clearance = self._target_signed_distance(target)
        if target_clearance < self.config.target_zone_buffer:
            outward = self._target_outward(target)
            tangent = np.array([-outward[1], outward[0]], dtype=float)
            if winner.goal is not None:
                goal_side = float(np.dot(np.array(winner.goal, dtype=float) - pos, tangent))
                tangent *= 1.0 if goal_side >= 0.0 else -1.0
            correction = self.config.target_zone_buffer - target_clearance + 0.05
            target = target + correction * outward + 0.45 * tangent

        return float(target[0]), float(target[1])

    def _reverse_is_safe(
        self,
        winner: SafetyAgentContext,
        contexts: Sequence[SafetyAgentContext],
    ) -> bool:
        x, y, yaw = float(winner.state[0]), float(winner.state[1]), float(winner.state[2])
        back = np.array([-math.cos(yaw), -math.sin(yaw)], dtype=float)
        start = np.array([x, y], dtype=float)
        current_clearance = self._target_signed_distance(start)

        others = [ctx for ctx in contexts if ctx.idx != winner.idx and ctx.active]
        for step in (0.33, 0.66, 1.0):
            sample = start + step * self.config.reverse_distance * back
            sample_clearance = self._target_signed_distance(sample)

            if current_clearance >= self.config.target_zone_buffer and sample_clearance < self.config.target_zone_buffer:
                return False
            if current_clearance < self.config.target_zone_buffer and sample_clearance < current_clearance - 0.03:
                return False

            for other in others:
                if _norm(sample - np.array(other.state[:2], dtype=float)) < self.config.reverse_clearance:
                    return False

        return True

