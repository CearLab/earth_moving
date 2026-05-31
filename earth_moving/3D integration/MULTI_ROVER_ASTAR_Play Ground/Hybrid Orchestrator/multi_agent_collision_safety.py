"""
multi_agent_collision_safety.py

Central, PyBullet-free collision safety policy for the hybrid multi-agent
orchestrator.

The planning threads decide what each rover wants to do next. This module is
for the always-running 3D/control thread: it watches live rover states, detects
predicted conflicts, chooses one right-of-way winner, and modifies nominal
commands before they are sent to PyBullet.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np


Vec2 = Tuple[float, float]

WIND_DIR = math.radians(60.0)
PATH_LOOKAHEAD_DIST = 0.45


class EpisodePhase(str, Enum):
    IDLE = "idle"
    APPROACH = "approach"
    YIELD_STOP = "yield_stop"
    WINNER_ESCAPE = "winner_escape"
    RELEASE = "release"


class AgentBehavior(str, Enum):
    NOMINAL = "nominal"
    YIELDER_SLOW = "yielder_slow"
    YIELDER_STOP = "yielder_stop"
    YIELDER_CLEAR_PATH = "yielder_clear_path"
    WINNER_ESCAPE_DIRECT = "winner_escape_direct"


@dataclass(frozen=True)
class SafetyAgentContext:
    idx: int
    agent_id: str
    state: np.ndarray
    active: bool
    path_constrained: bool
    path_points: Tuple[Vec2, ...] = ()
    goal: Optional[Vec2] = None
    priority: float = 0.5


@dataclass
class AgentSnapshot:
    idx: int
    rover_id: str
    pos: Vec2
    yaw: float
    v_fwd: float
    priority: float
    goal: Vec2


@dataclass
class ClusterEpisode:
    active: bool = False
    phase: EpisodePhase = EpisodePhase.IDLE
    winner_idx: Optional[int] = None
    yielder_indices: Set[int] = field(default_factory=set)
    started_at: float = 0.0
    stop_triggered_at: Optional[float] = None
    escape_built_at: Optional[float] = None
    winner_escape_field_ready: bool = False


@dataclass
class PolicyConfig:
    near_radius: float = 4.0
    conflict_tau: float = 4.0
    conflict_r_sum: float = 0.92
    yield_trigger_dist: float = 1.20
    stop_speed_thresh: float = 0.02
    escape_build_speed_thresh: float = 0.18
    escape_build_max_delay: float = 0.20
    goal_tol: float = 0.25
    min_hold_after_escape: float = 0.20
    predicted_speed: float = 1.20
    rearm_delay: float = 1.0
    path_clearance_radius: float = 0.65
    path_clear_target_offset: float = 0.95
    path_clear_speed_scale: float = 0.75
    winner_escape_speed_scale: float = 0.65
    winner_escape_forward: float = 0.75
    winner_escape_lateral: float = 0.75
    emergency_stop_distance: float = 0.42
    path_priority: float = 0.0


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def sailing_priority(yaw: float, wind_dir: float = WIND_DIR) -> float:
    rel = wrap_to_pi(yaw - wind_dir)
    return 0.5 - 0.5 * math.sin(rel)


def heading_vector(yaw: float, speed: float) -> Vec2:
    return (math.cos(yaw) * speed, math.sin(yaw) * speed)


def dist(a: Vec2, b: Vec2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def orca_sees_collision_potential(
    a: AgentSnapshot,
    b: AgentSnapshot,
    tau: float,
    r_sum: float,
    safety_margin: float = 1.05,
) -> bool:
    pa = a.pos
    pb = b.pos
    va = heading_vector(a.yaw, a.v_fwd)
    vb = heading_vector(b.yaw, b.v_fwd)

    p_rel = (pb[0] - pa[0], pb[1] - pa[1])
    v_rel = (vb[0] - va[0], vb[1] - va[1])
    r_eff = r_sum * safety_margin

    dist0 = math.hypot(p_rel[0], p_rel[1])
    if dist0 < r_eff:
        return True

    v2 = v_rel[0] * v_rel[0] + v_rel[1] * v_rel[1]
    if v2 < 1e-8:
        return False

    t_star = -(p_rel[0] * v_rel[0] + p_rel[1] * v_rel[1]) / v2
    if t_star < 0.0:
        d_min = dist0
    elif t_star > tau:
        d_min = math.hypot(p_rel[0] + v_rel[0] * tau, p_rel[1] + v_rel[1] * tau)
    else:
        d_min = math.hypot(p_rel[0] + v_rel[0] * t_star, p_rel[1] + v_rel[1] * t_star)

    return d_min < r_eff


def are_on_collision_course(
    a: AgentSnapshot,
    b: AgentSnapshot,
    tau: float,
    r_sum: float,
) -> bool:
    pa = a.pos
    pb = b.pos
    va = heading_vector(a.yaw, a.v_fwd)
    vb = heading_vector(b.yaw, b.v_fwd)

    p_rel = (pb[0] - pa[0], pb[1] - pa[1])
    v_rel = (vb[0] - va[0], vb[1] - va[1])

    dist0 = math.hypot(p_rel[0], p_rel[1])
    if dist0 < r_sum:
        return True

    v2 = v_rel[0] * v_rel[0] + v_rel[1] * v_rel[1]
    if v2 < 1e-8:
        return False

    t_star = -(p_rel[0] * v_rel[0] + p_rel[1] * v_rel[1]) / v2
    if t_star < 0.0 or t_star > tau:
        return False

    d_min = math.hypot(p_rel[0] + v_rel[0] * t_star, p_rel[1] + v_rel[1] * t_star)
    return d_min < r_sum


def closest_point_on_path(point: Vec2, path_points: Sequence[Vec2]):
    p_xy = np.array(point, dtype=float)
    best_point = np.array(path_points[0], dtype=float)
    best_tangent = np.array([1.0, 0.0], dtype=float)
    best_dist = float("inf")
    best_s = 0.0
    s_before = 0.0

    for i in range(len(path_points) - 1):
        a = np.array(path_points[i], dtype=float)
        b = np.array(path_points[i + 1], dtype=float)
        seg = b - a
        seg_len = float(np.linalg.norm(seg))
        if seg_len < 1e-9:
            continue
        t = float(np.dot(p_xy - a, seg) / (seg_len * seg_len))
        t = max(0.0, min(1.0, t))
        q = a + t * seg
        d = float(np.linalg.norm(p_xy - q))
        if d < best_dist:
            best_dist = d
            best_point = q
            best_tangent = seg / seg_len
            best_s = s_before + t * seg_len
        s_before += seg_len

    return best_point, best_tangent, best_dist, best_s


def distance_to_path(point_xy: Vec2, path_points: Sequence[Vec2]) -> float:
    if len(path_points) < 2:
        return float("inf")
    _, _, d, _ = closest_point_on_path(point_xy, path_points)
    return d


def path_clearance_target(
    yielder: SafetyAgentContext,
    path_winner: SafetyAgentContext,
    config: PolicyConfig,
) -> Optional[Vec2]:
    if len(path_winner.path_points) < 2:
        return None

    xy = np.array(yielder.state[:2], dtype=float)
    closest, tangent, dist_to_path, _ = closest_point_on_path(
        (float(xy[0]), float(xy[1])),
        path_winner.path_points,
    )
    if dist_to_path >= config.path_clearance_radius:
        return None

    normal = np.array([-tangent[1], tangent[0]], dtype=float)
    side = float(np.dot(xy - closest, normal))
    if abs(side) < 0.05:
        if yielder.goal is not None:
            goal_side = float(np.dot(np.array(yielder.goal, dtype=float) - closest, normal))
            side = goal_side if abs(goal_side) > 0.05 else 1.0
        else:
            side = 1.0
    side_sign = 1.0 if side >= 0.0 else -1.0
    target = closest + side_sign * config.path_clear_target_offset * normal
    return (float(target[0]), float(target[1]))


def path_lookahead_target(
    agent: SafetyAgentContext,
    lookahead: float = PATH_LOOKAHEAD_DIST,
) -> Optional[Vec2]:
    if len(agent.path_points) < 2:
        return agent.goal

    closest, _, _, s_now = closest_point_on_path(
        (float(agent.state[0]), float(agent.state[1])),
        agent.path_points,
    )
    target_s = s_now + lookahead
    s_before = 0.0

    for i in range(len(agent.path_points) - 1):
        a = np.array(agent.path_points[i], dtype=float)
        b = np.array(agent.path_points[i + 1], dtype=float)
        seg = b - a
        seg_len = float(np.linalg.norm(seg))
        if seg_len < 1e-9:
            continue
        if target_s <= s_before + seg_len:
            t = max(0.0, min(1.0, (target_s - s_before) / seg_len))
            q = a + t * seg
            return (float(q[0]), float(q[1]))
        s_before += seg_len

    if agent.path_points:
        return agent.path_points[-1]
    return (float(closest[0]), float(closest[1]))


def winner_escape_target(
    winner: SafetyAgentContext,
    yielders: Sequence[SafetyAgentContext],
    config: PolicyConfig,
) -> Optional[Vec2]:
    if winner.path_constrained:
        return path_lookahead_target(winner)

    if winner.goal is None:
        return None

    pos = np.array(winner.state[:2], dtype=float)
    goal = np.array(winner.goal, dtype=float)
    to_goal = goal - pos
    dist_goal = float(np.linalg.norm(to_goal))
    if dist_goal < 1e-6:
        return winner.goal

    d_hat = to_goal / dist_goal
    normal = np.array([-d_hat[1], d_hat[0]], dtype=float)

    closest_yielder = None
    closest_dist = float("inf")
    for yielder in yielders:
        y_pos = np.array(yielder.state[:2], dtype=float)
        d = float(np.linalg.norm(y_pos - pos))
        if d < closest_dist:
            closest_dist = d
            closest_yielder = y_pos

    if closest_yielder is None:
        return winner.goal

    rel = closest_yielder - pos
    lateral_side = float(np.dot(rel, normal))
    side_sign = -1.0 if lateral_side >= 0.0 else 1.0
    forward = min(config.winner_escape_forward, max(0.35, dist_goal))
    lateral = config.winner_escape_lateral
    target = pos + forward * d_hat + side_sign * lateral * normal
    return (float(target[0]), float(target[1]))


def build_conflict_components(
    snapshots: Sequence[AgentSnapshot],
    config: PolicyConfig,
) -> List[List[int]]:
    n = len(snapshots)
    adj: List[List[int]] = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if are_on_collision_course(
                snapshots[i],
                snapshots[j],
                tau=config.conflict_tau,
                r_sum=config.conflict_r_sum,
            ):
                adj[i].append(j)
                adj[j].append(i)

    visited = [False] * n
    comps: List[List[int]] = []
    for i in range(n):
        if visited[i]:
            continue
        stack = [i]
        visited[i] = True
        comp: List[int] = []
        while stack:
            u = stack.pop()
            comp.append(u)
            for v in adj[u]:
                if not visited[v]:
                    visited[v] = True
                    stack.append(v)
        if len(comp) >= 2:
            comps.append(comp)
    return comps


def _snapshot_from_context(ctx: SafetyAgentContext, config: PolicyConfig) -> AgentSnapshot:
    x, y, yaw, v_fwd, _ = ctx.state
    yaw_pred = float(yaw)

    if ctx.path_constrained and len(ctx.path_points) >= 2:
        _, tangent, _, _ = closest_point_on_path((float(x), float(y)), ctx.path_points)
        yaw_pred = math.atan2(float(tangent[1]), float(tangent[0]))
    elif ctx.goal is not None:
        dx = float(ctx.goal[0]) - float(x)
        dy = float(ctx.goal[1]) - float(y)
        if math.hypot(dx, dy) > 1e-6:
            yaw_pred = math.atan2(dy, dx)

    speed = max(abs(float(v_fwd)), config.predicted_speed)
    return AgentSnapshot(
        idx=ctx.idx,
        rover_id=ctx.agent_id,
        pos=(float(x), float(y)),
        yaw=yaw_pred,
        v_fwd=speed,
        priority=float(ctx.priority),
        goal=ctx.goal if ctx.goal is not None else (float(x), float(y)),
    )


def _command_to_point(
    state: np.ndarray,
    target: Vec2,
    speed_scale: float,
    v_max: float = 1.2,
    w_max: float = 7.5,
) -> Tuple[float, float]:
    x, y, yaw, v_fwd, _ = state
    dx = float(target[0]) - float(x)
    dy = float(target[1]) - float(y)
    dist_target = math.hypot(dx, dy)
    if dist_target < 0.05:
        return 0.0, 0.0

    theta_des = math.atan2(dy, dx)
    e_theta = wrap_to_pi(theta_des - float(yaw))
    if abs(float(v_fwd)) < 0.03 and abs(e_theta) > math.radians(50.0):
        return 0.0, math.copysign(min(w_max, 25.0), e_theta)

    align = max(0.0, math.cos(e_theta))
    v_cmd = v_max * speed_scale * (align ** 4) * min(1.0, dist_target / 0.45)
    w_cmd = max(-w_max, min(w_max, 6.0 * e_theta))
    if abs(e_theta) < math.radians(5.0):
        w_cmd = 0.0
    return float(v_cmd), float(w_cmd)


class MultiAgentCollisionSafety:
    def __init__(self, config: Optional[PolicyConfig] = None):
        self.config = config or PolicyConfig()
        self.episode = ClusterEpisode()
        self.rearm_until = 0.0
        self._last_episode_key = None

    def priority_for_context(self, ctx: SafetyAgentContext) -> float:
        if ctx.path_constrained:
            return self.config.path_priority + 1e-3 * ctx.idx
        return sailing_priority(float(ctx.state[2])) + 1e-3 * ctx.idx

    def with_priorities(self, contexts: Iterable[SafetyAgentContext]) -> List[SafetyAgentContext]:
        out = []
        for ctx in contexts:
            out.append(
                SafetyAgentContext(
                    idx=ctx.idx,
                    agent_id=ctx.agent_id,
                    state=ctx.state,
                    active=ctx.active,
                    path_constrained=ctx.path_constrained,
                    path_points=ctx.path_points,
                    goal=ctx.goal,
                    priority=self.priority_for_context(ctx),
                )
            )
        return out

    def _active_contexts(self, contexts: Sequence[SafetyAgentContext]) -> List[SafetyAgentContext]:
        return [ctx for ctx in contexts if ctx.active]

    def _choose_episode(self, contexts: Sequence[SafetyAgentContext]) -> Optional[ClusterEpisode]:
        active = self._active_contexts(contexts)
        if len(active) < 2:
            return None

        snapshots = [_snapshot_from_context(ctx, self.config) for ctx in active]
        comps = build_conflict_components(snapshots, self.config)
        if not comps:
            return None

        best_comp = min(comps, key=lambda comp: min(snapshots[i].priority for i in comp))
        winner_local = min(best_comp, key=lambda i: snapshots[i].priority)
        yielder_locals = [
            i for i in best_comp
            if i != winner_local and orca_sees_collision_potential(
                snapshots[winner_local],
                snapshots[i],
                tau=self.config.conflict_tau,
                r_sum=self.config.conflict_r_sum,
            )
        ]
        if not yielder_locals:
            return None

        return ClusterEpisode(
            active=True,
            phase=EpisodePhase.APPROACH,
            winner_idx=active[winner_local].idx,
            yielder_indices={active[i].idx for i in yielder_locals},
        )

    def _snapshot_by_idx(self, contexts: Sequence[SafetyAgentContext]) -> Dict[int, AgentSnapshot]:
        return {
            ctx.idx: _snapshot_from_context(ctx, self.config)
            for ctx in self._active_contexts(contexts)
        }

    def _path_winner_can_release(self, contexts: Sequence[SafetyAgentContext]) -> bool:
        if not self.episode.active or self.episode.winner_idx is None:
            return False
        by_idx = {ctx.idx: ctx for ctx in contexts}
        winner_ctx = by_idx.get(self.episode.winner_idx)
        if winner_ctx is None or not winner_ctx.path_constrained:
            return False

        snapshots = self._snapshot_by_idx(contexts)
        winner_pred = snapshots.get(winner_ctx.idx)
        if winner_pred is None:
            return False

        for y_idx in self.episode.yielder_indices:
            y_ctx = by_idx.get(y_idx)
            y_pred = snapshots.get(y_idx)
            if y_ctx is None or y_pred is None:
                continue
            if distance_to_path((float(y_ctx.state[0]), float(y_ctx.state[1])), winner_ctx.path_points) < self.config.path_clearance_radius:
                return False
            if orca_sees_collision_potential(
                winner_pred,
                y_pred,
                tau=self.config.conflict_tau,
                r_sum=self.config.conflict_r_sum,
            ):
                return False
        return True

    def update(self, contexts: Sequence[SafetyAgentContext], sim_time: float) -> None:
        contexts = self.with_priorities(contexts)
        prev_active = self.episode.active
        prev_winner = self.episode.winner_idx
        prev_yielders = set(self.episode.yielder_indices)

        if not self.episode.active and sim_time >= self.rearm_until:
            candidate = self._choose_episode(contexts)
            if candidate is not None:
                candidate.started_at = sim_time
                self.episode = candidate
                key = (candidate.winner_idx, tuple(sorted(candidate.yielder_indices)))
                if key != self._last_episode_key:
                    self._last_episode_key = key
                    print(
                        f"[SAFETY] New episode: winner=R{candidate.winner_idx}, "
                        f"yielders={[f'R{i}' for i in sorted(candidate.yielder_indices)]}"
                    )

        if self.episode.active:
            self._update_active_episode(contexts, sim_time)

        if (
            prev_active
            and prev_winner is not None
            and not self.episode.active
            and prev_yielders
        ):
            self.rearm_until = sim_time + self.config.rearm_delay
            print("[SAFETY] Episode released; handing control back to nominal ORCA.")

    def _update_active_episode(self, contexts: Sequence[SafetyAgentContext], sim_time: float) -> None:
        by_idx = {ctx.idx: ctx for ctx in contexts if ctx.active}
        if self.episode.winner_idx not in by_idx:
            self.episode = ClusterEpisode()
            return

        winner = by_idx[self.episode.winner_idx]
        yielders = [by_idx[i] for i in self.episode.yielder_indices if i in by_idx]
        if not yielders:
            self.episode = ClusterEpisode()
            return

        min_dist = min(dist((float(winner.state[0]), float(winner.state[1])), (float(y.state[0]), float(y.state[1]))) for y in yielders)
        max_yielder_speed = max(abs(float(y.state[3])) for y in yielders)
        dist_winner_goal = (
            dist((float(winner.state[0]), float(winner.state[1])), winner.goal)
            if winner.goal is not None else float("inf")
        )

        if self.episode.phase == EpisodePhase.APPROACH and min_dist < self.config.yield_trigger_dist:
            self.episode.phase = EpisodePhase.YIELD_STOP
            self.episode.stop_triggered_at = sim_time
            print("[SAFETY] Yield stop triggered.")

        if (
            self.episode.phase == EpisodePhase.YIELD_STOP
            and (
                max_yielder_speed < self.config.escape_build_speed_thresh
                or (
                    self.episode.stop_triggered_at is not None
                    and (sim_time - self.episode.stop_triggered_at) >= self.config.escape_build_max_delay
                    and max_yielder_speed < 0.20
                )
            )
        ):
            self.episode.phase = EpisodePhase.WINNER_ESCAPE
            self.episode.escape_built_at = sim_time
            self.episode.winner_escape_field_ready = True
            print("[SAFETY] Winner escape/clearance phase.")

        if self.episode.phase == EpisodePhase.WINNER_ESCAPE:
            hold_elapsed = (
                self.episode.escape_built_at is not None
                and (sim_time - self.episode.escape_built_at) >= self.config.min_hold_after_escape
            )

            if winner.path_constrained:
                can_release = self._path_winner_can_release(contexts)
            else:
                snapshots = self._snapshot_by_idx(contexts)
                winner_pred = snapshots.get(winner.idx)
                can_release = winner_pred is not None and not any(
                    y_idx in snapshots and orca_sees_collision_potential(
                        winner_pred,
                        snapshots[y_idx],
                        tau=self.config.conflict_tau,
                        r_sum=self.config.conflict_r_sum,
                    )
                    for y_idx in self.episode.yielder_indices
                )

            if (hold_elapsed and can_release) or dist_winner_goal < self.config.goal_tol:
                self.episode.phase = EpisodePhase.RELEASE
                self.episode.active = False

    def behavior_for(self, idx: int, contexts: Sequence[SafetyAgentContext]) -> AgentBehavior:
        if not self.episode.active or self.episode.winner_idx is None:
            return AgentBehavior.NOMINAL
        if idx == self.episode.winner_idx:
            if self.episode.phase == EpisodePhase.WINNER_ESCAPE:
                return AgentBehavior.WINNER_ESCAPE_DIRECT
            return AgentBehavior.NOMINAL
        if idx not in self.episode.yielder_indices:
            return AgentBehavior.NOMINAL

        if self.episode.phase == EpisodePhase.APPROACH:
            return AgentBehavior.YIELDER_SLOW

        if self.episode.phase in (EpisodePhase.YIELD_STOP, EpisodePhase.WINNER_ESCAPE):
            by_idx = {ctx.idx: ctx for ctx in contexts}
            winner = by_idx.get(self.episode.winner_idx)
            yielder = by_idx.get(idx)
            if winner is not None and yielder is not None and winner.path_constrained and not yielder.path_constrained:
                target = path_clearance_target(yielder, winner, self.config)
                if target is not None:
                    return AgentBehavior.YIELDER_CLEAR_PATH
            return AgentBehavior.YIELDER_STOP

        return AgentBehavior.NOMINAL

    def should_force_avoid(self, ego_idx: int, other_idx: int) -> bool:
        return (
            self.episode.active
            and self.episode.winner_idx == ego_idx
            and other_idx in self.episode.yielder_indices
            and self.episode.phase in (EpisodePhase.YIELD_STOP, EpisodePhase.WINNER_ESCAPE)
        )

    def filter_controls(
        self,
        contexts: Sequence[SafetyAgentContext],
        nominal_controls: Dict[int, Tuple[float, float]],
    ) -> Dict[int, Tuple[float, float]]:
        contexts = self.with_priorities(contexts)
        by_idx = {ctx.idx: ctx for ctx in contexts}
        out = dict(nominal_controls)

        for ctx in contexts:
            if ctx.idx not in out:
                continue
            behavior = self.behavior_for(ctx.idx, contexts)
            if behavior == AgentBehavior.WINNER_ESCAPE_DIRECT:
                yielders = [
                    by_idx[y_idx]
                    for y_idx in self.episode.yielder_indices
                    if y_idx in by_idx
                ]
                target = winner_escape_target(ctx, yielders, self.config)
                fallback_cmd = (
                    _command_to_point(ctx.state, target, self.config.winner_escape_speed_scale)
                    if target is not None else out[ctx.idx]
                )
                if ctx.path_constrained:
                    nominal_v, nominal_w = out[ctx.idx]
                    if abs(nominal_v) < 0.05 and abs(nominal_w) < 0.20:
                        out[ctx.idx] = fallback_cmd
                else:
                    out[ctx.idx] = fallback_cmd
            elif behavior == AgentBehavior.YIELDER_STOP:
                out[ctx.idx] = (0.0, 0.0)
            elif behavior == AgentBehavior.YIELDER_SLOW:
                out[ctx.idx] = self._slow_yielder(ctx, out[ctx.idx])
            elif behavior == AgentBehavior.YIELDER_CLEAR_PATH:
                winner = by_idx.get(self.episode.winner_idx)
                if winner is None:
                    out[ctx.idx] = (0.0, 0.0)
                else:
                    target = path_clearance_target(ctx, winner, self.config)
                    out[ctx.idx] = (
                        _command_to_point(ctx.state, target, self.config.path_clear_speed_scale)
                        if target is not None else (0.0, 0.0)
                    )

        for a in contexts:
            for b in contexts:
                if b.idx <= a.idx:
                    continue
                d = dist((float(a.state[0]), float(a.state[1])), (float(b.state[0]), float(b.state[1])))
                if d < self.config.emergency_stop_distance:
                    out[a.idx] = (0.0, 0.0)
                    out[b.idx] = (0.0, 0.0)
                    print(f"[SAFETY] Emergency stop: {a.agent_id}-{b.agent_id} distance={d:.3f}m")

        return out

    def _slow_yielder(
        self,
        ctx: SafetyAgentContext,
        nominal: Tuple[float, float],
    ) -> Tuple[float, float]:
        if self.episode.winner_idx is None:
            return nominal
        # Conservative by default: slow to half until the stop phase takes over.
        slow = 0.5
        return float(nominal[0]) * slow, float(nominal[1])
