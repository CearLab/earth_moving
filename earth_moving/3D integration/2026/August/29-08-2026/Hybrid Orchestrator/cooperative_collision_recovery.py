"""Bounded, task-aware cooperative recovery for close or stalled rover pairs."""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, Optional, Sequence, Tuple

from target_zones import resolve_target_zone


Vec2 = Tuple[float, float]
Control = Tuple[float, float]


@dataclass(frozen=True)
class RecoveryAgent:
    idx: int
    agent_id: str
    state: Sequence[float]
    active: bool
    phase: str
    priority: float
    collision_radius: float = 0.46
    static_clearance: float = 0.25
    path_points: Tuple[Vec2, ...] = ()

    @property
    def position(self) -> Vec2:
        return float(self.state[0]), float(self.state[1])


@dataclass
class CooperativeRecoveryConfig:
    base_trigger_distance: float = 0.55
    base_release_distance: float = 0.82
    base_collision_radius: float = 0.46
    footprint_trigger_clearance: float = 0.08
    footprint_release_clearance: float = 0.24
    stuck_speed: float = 0.07
    stuck_command: float = 0.06
    stuck_duration: float = 0.60
    reverse_speed: float = 0.22
    reverse_distance: float = 0.42
    reverse_max_time: float = 0.90
    progress_epsilon: float = 0.035
    progress_timeout: float = 0.45
    detour_distance: float = 0.62
    detour_max_time: float = 1.60
    winner_assist_distance: float = 0.42
    winner_assist_max_time: float = 1.20
    cooperative_clear_after: float = 2.50
    cooperative_clear_yielder_distance: float = 0.72
    cooperative_clear_winner_distance: float = 0.46
    cooperative_clear_max_time: float = 1.80
    cooperative_clear_retry_timeout: float = 0.65
    cooperative_clear_max_attempts: int = 3
    max_escalations: int = 4
    escalation_distance_gain: float = 0.20
    pebble_penetration_tolerance: float = 0.015
    total_max_time: float = 7.50
    cooldown: float = 1.50
    boundary_margin: float = 0.05
    # Recovery may leave the material-map radius, but not this navigation envelope.
    navigation_outside_margin: float = 1.00
    target_buffer: float = 0.10
    pebble_radius: float = 0.05
    other_rover_clearance: float = 0.44
    drive_speed: float = 0.28
    turn_gain: float = 4.0
    turn_limit: float = 5.0


@dataclass
class RecoveryEpisode:
    winner_idx: int
    yielder_idx: int
    phase: str
    started_at: float
    phase_started_at: float
    yielder_start: Vec2
    winner_start: Vec2
    best_separation: float
    last_progress_at: float
    target: Optional[Vec2] = None
    winner_assist_reverse: bool = False
    clear_targets: Dict[int, Vec2] = field(default_factory=dict)
    clear_reverse: Dict[int, bool] = field(default_factory=dict)
    clear_rejections: Dict[int, Dict[str, int]] = field(default_factory=dict)
    clear_attempt: int = 0
    escalation_level: int = 0
    blocked_reported: bool = False


@dataclass
class RecoveryResult:
    controls: Dict[int, Control]
    replan_indices: set = field(default_factory=set)
    messages: list = field(default_factory=list)
    active_pair: Optional[Tuple[int, int]] = None
    phase: Optional[str] = None


def _distance(a: Vec2, b: Vec2) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _unit(vector: Vec2, fallback: Vec2 = (1.0, 0.0)) -> Vec2:
    norm = math.hypot(float(vector[0]), float(vector[1]))
    if norm <= 1e-9:
        fallback_norm = math.hypot(float(fallback[0]), float(fallback[1]))
        if fallback_norm <= 1e-9:
            return 1.0, 0.0
        return float(fallback[0]) / fallback_norm, float(fallback[1]) / fallback_norm
    return float(vector[0]) / norm, float(vector[1]) / norm


class CooperativeRecoveryManager:
    """Owns the only reverse command used by the scheduled hybrid runner."""

    def __init__(self, config: Optional[CooperativeRecoveryConfig] = None):
        self.config = config or CooperativeRecoveryConfig()
        self.episode: Optional[RecoveryEpisode] = None
        self._close_since: Dict[Tuple[int, int], float] = {}
        self._cooldown_until: Dict[Tuple[int, int], float] = {}
        self.target_zone = None

    def pair_distances(self, a: RecoveryAgent, b: RecoveryAgent) -> Tuple[float, float]:
        extra = max(0.0, float(a.collision_radius) - self.config.base_collision_radius)
        extra += max(0.0, float(b.collision_radius) - self.config.base_collision_radius)
        footprint_sum = float(a.collision_radius) + float(b.collision_radius)
        return (
            max(
                self.config.base_trigger_distance + extra,
                footprint_sum + self.config.footprint_trigger_clearance,
            ),
            max(
                self.config.base_release_distance + extra,
                footprint_sum + self.config.footprint_release_clearance,
            ),
        )

    def update(
        self,
        agents: Sequence[RecoveryAgent],
        controls: Dict[int, Control],
        pebbles: Iterable[Vec2],
        env_radius: float,
        target_radius: float,
        sim_time: float,
        target_zone=None,
    ) -> RecoveryResult:
        self.target_zone = resolve_target_zone(target_zone, target_radius)
        by_idx = {agent.idx: agent for agent in agents if agent.active}
        out = dict(controls)
        pebbles = tuple((float(p[0]), float(p[1])) for p in pebbles)
        messages = []
        replans = set()

        if self.episode is not None:
            winner = by_idx.get(self.episode.winner_idx)
            yielder = by_idx.get(self.episode.yielder_idx)
            if winner is None or yielder is None:
                self._finish(sim_time, messages, "pair no longer active")
            else:
                self._run_episode(
                    winner, yielder, by_idx, out, pebbles,
                    float(env_radius), float(target_radius), float(sim_time),
                    replans, messages,
                )

        if self.episode is None:
            candidate = self._find_stuck_pair(tuple(by_idx.values()), out, float(sim_time))
            if candidate is not None:
                a, b = candidate
                winner, yielder = (a, b) if (a.priority, a.idx) < (b.priority, b.idx) else (b, a)
                separation = _distance(winner.position, yielder.position)
                self.episode = RecoveryEpisode(
                    winner_idx=winner.idx,
                    yielder_idx=yielder.idx,
                    phase="YIELDER_RETREAT",
                    started_at=float(sim_time),
                    phase_started_at=float(sim_time),
                    yielder_start=yielder.position,
                    winner_start=winner.position,
                    best_separation=separation,
                    last_progress_at=float(sim_time),
                )
                messages.append(
                    f"[RECOVERY] start winner={winner.agent_id}({winner.phase}) "
                    f"yielder={yielder.agent_id}({yielder.phase}) d={separation:.3f}m"
                )
                self._enter_retreat_or_detour(
                    winner, yielder, by_idx, pebbles,
                    float(env_radius), float(target_radius), float(sim_time), messages,
                )
                self._run_episode(
                    winner, yielder, by_idx, out, pebbles,
                    float(env_radius), float(target_radius), float(sim_time),
                    replans, messages,
                )

        active_pair = None
        phase = None
        if self.episode is not None:
            active_pair = (self.episode.winner_idx, self.episode.yielder_idx)
            phase = self.episode.phase
        return RecoveryResult(out, replans, messages, active_pair, phase)

    def _find_stuck_pair(
        self,
        agents: Sequence[RecoveryAgent],
        controls: Dict[int, Control],
        sim_time: float,
    ) -> Optional[Tuple[RecoveryAgent, RecoveryAgent]]:
        current_keys = set()
        candidates = []
        for i, a in enumerate(agents):
            for b in agents[i + 1:]:
                key = tuple(sorted((a.idx, b.idx)))
                current_keys.add(key)
                if sim_time < self._cooldown_until.get(key, -1e9):
                    self._close_since.pop(key, None)
                    continue
                trigger, _ = self.pair_distances(a, b)
                separation = _distance(a.position, b.position)
                a_control = controls.get(a.idx, (0.0, 0.0))
                b_control = controls.get(b.idx, (0.0, 0.0))
                slow_motion = max(abs(float(a.state[3])), abs(float(b.state[3]))) <= self.config.stuck_speed
                blocked_commands = (
                    abs(float(a_control[0])) <= self.config.stuck_command
                    and abs(float(b_control[0])) <= self.config.stuck_command
                )
                if separation <= trigger and (slow_motion or blocked_commands):
                    since = self._close_since.setdefault(key, sim_time)
                    if sim_time - since >= self.config.stuck_duration:
                        candidates.append((separation, a, b))
                else:
                    self._close_since.pop(key, None)
        for key in tuple(self._close_since):
            if key not in current_keys:
                self._close_since.pop(key, None)
        if not candidates:
            return None
        _, a, b = min(candidates, key=lambda item: item[0])
        return a, b

    def _run_episode(
        self,
        winner: RecoveryAgent,
        yielder: RecoveryAgent,
        by_idx: Dict[int, RecoveryAgent],
        controls: Dict[int, Control],
        pebbles: Sequence[Vec2],
        env_radius: float,
        target_radius: float,
        sim_time: float,
        replans: set,
        messages: list,
    ) -> None:
        episode = self.episode
        if episode is None:
            return
        _, release_distance = self.pair_distances(winner, yielder)
        separation = _distance(winner.position, yielder.position)
        if separation >= release_distance:
            if episode.phase == "COOPERATIVE_CLEAR":
                for mover in (winner, yielder):
                    if mover.phase in ("APPROACH", "PARKING"):
                        replans.add(mover.idx)
            elif yielder.phase in ("APPROACH", "PARKING"):
                replans.add(yielder.idx)
            self._finish(sim_time, messages, f"clearance restored d={separation:.3f}m")
            return
        if episode.phase == "BLOCKED_SAFE_STOP":
            controls[winner.idx] = (0.0, 0.0)
            controls[yielder.idx] = (0.0, 0.0)
            return
        if separation >= episode.best_separation + self.config.progress_epsilon:
            episode.best_separation = separation
            episode.last_progress_at = sim_time
        if sim_time - episode.started_at >= self.config.total_max_time:
            controls[winner.idx] = (0.0, 0.0)
            controls[yielder.idx] = (0.0, 0.0)
            episode.escalation_level += 1
            if episode.escalation_level >= self.config.max_escalations:
                episode.phase = "BLOCKED_SAFE_STOP"
                episode.phase_started_at = sim_time
                if not episode.blocked_reported:
                    episode.blocked_reported = True
                    messages.append(
                        "[RECOVERY-BLOCKED] no verified separation route after "
                        f"{episode.escalation_level} bounded escalations; "
                        "holding both rovers in an explicit safe-stop state"
                    )
                return
            episode.started_at = sim_time
            episode.clear_attempt = max(
                0, self.config.cooperative_clear_max_attempts - 1,
            )
            messages.append(
                "[RECOVERY] pair still below release distance; "
                f"escalating joint clearance level={episode.escalation_level}"
            )
            self._enter_cooperative_clear(
                winner, yielder, by_idx, pebbles,
                env_radius, target_radius, sim_time, messages,
            )
            return

        if (
            episode.phase != "COOPERATIVE_CLEAR"
            and sim_time - episode.started_at >= self.config.cooperative_clear_after
        ):
            self._enter_cooperative_clear(
                winner, yielder, by_idx, pebbles,
                env_radius, target_radius, sim_time, messages,
            )
            episode = self.episode
            if episode is None:
                return

        if episode.phase == "COOPERATIVE_CLEAR":
            for mover, speed_scale in ((winner, 0.72), (yielder, 1.0)):
                target = episode.clear_targets.get(mover.idx)
                if target is None:
                    controls[mover.idx] = (0.0, 0.0)
                else:
                    controls[mover.idx] = self._drive_to_signed(
                        mover, target,
                        reverse=bool(episode.clear_reverse.get(mover.idx, False)),
                        speed_scale=speed_scale,
                    )
            has_targets = bool(episode.clear_targets)
            reached = has_targets and all(
                _distance(by_idx[idx].position, target) <= 0.10
                for idx, target in episode.clear_targets.items()
                if idx in by_idx
            )
            retry_without_target = (
                not has_targets
                and sim_time - episode.phase_started_at
                >= self.config.cooperative_clear_retry_timeout
            )
            stalled = (
                has_targets
                and sim_time - episode.last_progress_at
                >= self.config.cooperative_clear_retry_timeout
            )
            timed_out = (
                sim_time - episode.phase_started_at
                >= self.config.cooperative_clear_max_time
            )
            if reached or retry_without_target or stalled or timed_out:
                if episode.clear_attempt >= self.config.cooperative_clear_max_attempts:
                    messages.append(
                        "[RECOVERY] clearance not restored; retaining ownership "
                        "and retrying the emergency escape plan"
                    )
                    episode.clear_attempt = (
                        self.config.cooperative_clear_max_attempts - 1
                    )
                self._enter_cooperative_clear(
                    winner, yielder, by_idx, pebbles,
                    env_radius, target_radius, sim_time, messages,
                )
            return

        if episode.phase == "YIELDER_RETREAT":
            controls[winner.idx] = (0.0, 0.0)
            elapsed = sim_time - episode.phase_started_at
            moved = _distance(yielder.position, episode.yielder_start)
            reverse_safe = self._reverse_is_safe(
                yielder, winner, by_idx, pebbles, env_radius, target_radius,
                min(0.18, self.config.reverse_distance),
            )
            if (
                not reverse_safe
                or elapsed >= self.config.reverse_max_time
                or moved >= self.config.reverse_distance
                or sim_time - episode.last_progress_at >= self.config.progress_timeout
            ):
                self._enter_detour(
                    winner, yielder, by_idx, pebbles,
                    env_radius, target_radius, sim_time, messages,
                )
            else:
                controls[yielder.idx] = (-self.config.reverse_speed, 0.0)
                return

        if self.episode is None:
            return
        episode = self.episode
        if episode.phase == "YIELDER_DETOUR":
            controls[winner.idx] = (0.0, 0.0)
            if episode.target is None:
                self._enter_winner_assist(
                    winner, yielder, by_idx, pebbles,
                    env_radius, target_radius, sim_time, messages,
                )
            else:
                controls[yielder.idx] = self._drive_to(yielder, episode.target)
                reached = _distance(yielder.position, episode.target) <= 0.10
                stalled = sim_time - episode.last_progress_at >= self.config.progress_timeout + 0.25
                timed_out = sim_time - episode.phase_started_at >= self.config.detour_max_time
                if reached or stalled or timed_out:
                    self._enter_winner_assist(
                        winner, yielder, by_idx, pebbles,
                        env_radius, target_radius, sim_time, messages,
                    )
                else:
                    return

        if self.episode is None:
            return
        episode = self.episode
        if episode.phase == "WINNER_ASSIST":
            controls[yielder.idx] = (0.0, 0.0)
            if episode.target is None:
                controls[winner.idx] = (0.0, 0.0)
            elif episode.winner_assist_reverse:
                controls[winner.idx] = (-min(0.16, self.config.reverse_speed), 0.0)
            else:
                controls[winner.idx] = self._drive_to(winner, episode.target)
            reached = episode.target is not None and _distance(winner.position, episode.target) <= 0.10
            timed_out = sim_time - episode.phase_started_at >= self.config.winner_assist_max_time
            if reached or timed_out:
                self._enter_cooperative_clear(
                    winner, yielder, by_idx, pebbles,
                    env_radius, target_radius, sim_time, messages,
                )

    def _select_escape_motion(
        self, mover, blocker, by_idx, pebbles,
        env_radius, target_radius, distance, allow_pebbles,
        relax_pebbles=False, relax_target_zone=False,
        rejection_counts=None,
    ) -> Tuple[Optional[Vec2], bool]:
        yaw = float(mover.state[2])
        heading = (math.cos(yaw), math.sin(yaw))
        away = _unit((
            mover.position[0] - blocker.position[0],
            mover.position[1] - blocker.position[1],
        ))
        current_sep = _distance(mover.position, blocker.position)
        best = None
        best_reverse = False
        best_score = -float("inf")

        # Evaluate the two immediately executable directions first.
        for reverse in (False, True):
            sign = -1.0 if reverse else 1.0
            direction = (sign * heading[0], sign * heading[1])
            for scale in (1.0, 0.72, 0.48):
                target = (
                    mover.position[0] + float(distance) * scale * direction[0],
                    mover.position[1] + float(distance) * scale * direction[1],
                )
                final_sep = _distance(target, blocker.position)
                if final_sep <= current_sep + 0.04:
                    if rejection_counts is not None:
                        rejection_counts["insufficient_separation_gain"] = (
                            rejection_counts.get("insufficient_separation_gain", 0) + 1
                        )
                    continue
                if not self._segment_is_safe(
                    mover.position, target, mover, blocker, by_idx, pebbles,
                    env_radius, target_radius,
                    allow_pebbles=(allow_pebbles or relax_pebbles),
                    rejection_counts=rejection_counts,
                    allow_target_zone=relax_target_zone,
                ):
                    continue
                alignment = direction[0] * away[0] + direction[1] * away[1]
                score = 2.0 * (final_sep - current_sep) + alignment
                if reverse:
                    score -= 0.03
                if score > best_score:
                    best_score = score
                    best = target
                    best_reverse = reverse

        if best is not None:
            return best, best_reverse

        # A turning clearance move is the fallback if both straight choices are blocked.
        target = self._find_clearance_target(
            mover, blocker, by_idx, pebbles, env_radius, target_radius,
            distance,
            allow_pebbles=(allow_pebbles or relax_pebbles),
            rejection_counts=rejection_counts,
            allow_target_zone=relax_target_zone,
        )
        return target, False

    def _enter_cooperative_clear(
        self, winner, yielder, by_idx, pebbles,
        env_radius, target_radius, sim_time, messages,
    ):
        episode = self.episode
        if episode is None:
            return
        episode.phase = "COOPERATIVE_CLEAR"
        episode.phase_started_at = sim_time
        episode.last_progress_at = sim_time
        episode.clear_attempt += 1
        episode.clear_targets = {}
        episode.clear_reverse = {}
        episode.clear_rejections = {}

        relax_pebbles = episode.clear_attempt >= 2
        relax_target_zone = episode.clear_attempt >= 3
        constraint_mode = (
            "emergency_target_relax" if relax_target_zone
            else "soft_pebbles" if relax_pebbles
            else "strict"
        )
        distance_scale = 1.0 + min(
            0.80,
            self.config.escalation_distance_gain * episode.escalation_level,
        )
        for mover, blocker, distance, allow_pebbles in (
            (yielder, winner, self.config.cooperative_clear_yielder_distance * distance_scale, yielder.phase == "PUSH"),
            (winner, yielder, self.config.cooperative_clear_winner_distance * distance_scale, winner.phase == "PUSH"),
        ):
            rejection_counts = {}
            target, reverse = self._select_escape_motion(
                mover, blocker, by_idx, pebbles,
                env_radius, target_radius, distance, allow_pebbles,
                relax_pebbles=relax_pebbles,
                relax_target_zone=relax_target_zone,
                rejection_counts=rejection_counts,
            )
            episode.clear_rejections[mover.idx] = rejection_counts
            if target is not None:
                episode.clear_targets[mover.idx] = target
                episode.clear_reverse[mover.idx] = reverse

        descriptions = []
        for mover in (winner, yielder):
            target = episode.clear_targets.get(mover.idx)
            mode = (
                "no-target" if target is None
                else "reverse" if episode.clear_reverse.get(mover.idx, False)
                else "forward"
            )
            descriptions.append(
                f"{mover.agent_id}:{mode}->"
                f"{None if target is None else tuple(round(v, 2) for v in target)} "
                f"rejects={episode.clear_rejections.get(mover.idx, {})}"
            )
        messages.append(
            f"[RECOVERY] cooperative clear attempt={episode.clear_attempt} "
            f"escalation={episode.escalation_level} "
            f"constraints={constraint_mode}: " + ", ".join(descriptions)
        )

    def _enter_retreat_or_detour(
        self, winner, yielder, by_idx, pebbles,
        env_radius, target_radius, sim_time, messages,
    ):
        if self._reverse_is_safe(
            yielder, winner, by_idx, pebbles, env_radius, target_radius,
            self.config.reverse_distance,
        ):
            away = _unit((
                yielder.position[0] - winner.position[0],
                yielder.position[1] - winner.position[1],
            ))
            reverse = (-math.cos(float(yielder.state[2])), -math.sin(float(yielder.state[2])))
            if reverse[0] * away[0] + reverse[1] * away[1] >= 0.25:
                messages.append(f"[RECOVERY] {yielder.agent_id} bounded reverse retreat")
                return
        self._enter_detour(
            winner, yielder, by_idx, pebbles,
            env_radius, target_radius, sim_time, messages,
        )

    def _enter_detour(
        self, winner, yielder, by_idx, pebbles,
        env_radius, target_radius, sim_time, messages,
    ):
        episode = self.episode
        if episode is None:
            return
        episode.phase = "YIELDER_DETOUR"
        episode.phase_started_at = sim_time
        episode.last_progress_at = sim_time
        episode.target = self._find_clearance_target(
            yielder, winner, by_idx, pebbles, env_radius, target_radius,
            self.config.detour_distance, allow_pebbles=False,
        )
        messages.append(
            f"[RECOVERY] {yielder.agent_id} detour target="
            f"{None if episode.target is None else tuple(round(v, 2) for v in episode.target)}"
        )

    def _enter_winner_assist(
        self, winner, yielder, by_idx, pebbles,
        env_radius, target_radius, sim_time, messages,
    ):
        episode = self.episode
        if episode is None:
            return
        episode.phase = "WINNER_ASSIST"
        episode.phase_started_at = sim_time
        episode.last_progress_at = sim_time
        target, reverse = self._winner_assist_target(
            winner, yielder, by_idx, pebbles, env_radius, target_radius,
        )
        episode.target = target
        episode.winner_assist_reverse = reverse
        messages.append(
            f"[RECOVERY] yielder blocked; {winner.agent_id} bounded assist "
            f"mode={'reverse' if reverse else 'forward/clearance'} "
            f"target={None if target is None else tuple(round(v, 2) for v in target)}"
        )

    def _winner_assist_target(
        self, winner, yielder, by_idx, pebbles, env_radius, target_radius,
    ) -> Tuple[Optional[Vec2], bool]:
        current_sep = _distance(winner.position, yielder.position)
        if winner.path_points:
            nearest = min(
                range(len(winner.path_points)),
                key=lambda i: _distance(winner.position, winner.path_points[i]),
            )
            for index in range(nearest + 1, min(len(winner.path_points), nearest + 30)):
                target = winner.path_points[index]
                if _distance(target, winner.position) < self.config.winner_assist_distance:
                    continue
                if _distance(target, yielder.position) <= current_sep + 0.08:
                    continue
                if self._segment_is_safe(
                    winner.position, target, winner, yielder, by_idx, pebbles,
                    env_radius, target_radius, allow_pebbles=True,
                ):
                    return (float(target[0]), float(target[1])), False

        if winner.phase not in ("PUSH", "TURN_TO_PUSH"):
            target = self._find_clearance_target(
                winner, yielder, by_idx, pebbles, env_radius, target_radius,
                self.config.winner_assist_distance, allow_pebbles=False,
            )
            if target is not None:
                return target, False

        yaw = float(winner.state[2])
        back = (-math.cos(yaw), -math.sin(yaw))
        away = _unit((
            winner.position[0] - yielder.position[0],
            winner.position[1] - yielder.position[1],
        ))
        if back[0] * away[0] + back[1] * away[1] >= 0.25:
            target = (
                winner.position[0] + 0.22 * back[0],
                winner.position[1] + 0.22 * back[1],
            )
            if self._segment_is_safe(
                winner.position, target, winner, yielder, by_idx, pebbles,
                env_radius, target_radius, allow_pebbles=winner.phase == "PUSH",
            ):
                return target, True
        return None, False

    def _find_clearance_target(
        self, mover, blocker, by_idx, pebbles, env_radius, target_radius,
        distance, allow_pebbles, rejection_counts=None,
        allow_target_zone=False,
    ) -> Optional[Vec2]:
        away = _unit((
            mover.position[0] - blocker.position[0],
            mover.position[1] - blocker.position[1],
        ))
        base_angle = math.atan2(away[1], away[0])
        offsets = (0, 30, -30, 60, -60, 90, -90, 120, -120, 180)
        best = None
        best_score = -float("inf")
        current_sep = _distance(mover.position, blocker.position)
        for radius_scale in (1.0, 0.75):
            radius = float(distance) * radius_scale
            for offset in offsets:
                angle = base_angle + math.radians(offset)
                target = (
                    mover.position[0] + radius * math.cos(angle),
                    mover.position[1] + radius * math.sin(angle),
                )
                final_sep = _distance(target, blocker.position)
                if final_sep <= current_sep + 0.06:
                    if rejection_counts is not None:
                        rejection_counts["insufficient_separation_gain"] = (
                            rejection_counts.get("insufficient_separation_gain", 0) + 1
                        )
                    continue
                if not self._segment_is_safe(
                    mover.position, target, mover, blocker, by_idx, pebbles,
                    env_radius, target_radius, allow_pebbles,
                    rejection_counts=rejection_counts,
                    allow_target_zone=allow_target_zone,
                ):
                    continue
                score = final_sep - 0.15 * abs(offset) / 180.0
                if score > best_score:
                    best_score = score
                    best = target
        return best

    def _reverse_is_safe(
        self, mover, blocker, by_idx, pebbles,
        env_radius, target_radius, distance,
    ) -> bool:
        yaw = float(mover.state[2])
        target = (
            mover.position[0] - float(distance) * math.cos(yaw),
            mover.position[1] - float(distance) * math.sin(yaw),
        )
        return self._segment_is_safe(
            mover.position, target, mover, blocker, by_idx, pebbles,
            env_radius, target_radius, allow_pebbles=False,
        )

    def _segment_is_safe(
        self, start, target, mover, blocker, by_idx, pebbles,
        env_radius, target_radius, allow_pebbles,
        rejection_counts=None, allow_target_zone=False,
    ) -> bool:
        def reject(reason):
            if rejection_counts is not None:
                rejection_counts[reason] = int(rejection_counts.get(reason, 0)) + 1
            return False

        start_blocker_dist = _distance(start, blocker.position)
        zone = self.target_zone or resolve_target_zone(None, target_radius)
        start_clearance = zone.signed_distance_world(*start)
        pebble_start_distances = (
            tuple(_distance(start, pebble) for pebble in pebbles)
            if not allow_pebbles else ()
        )
        for step in range(1, 11):
            t = step / 10.0
            sample = (
                float(start[0]) + t * (float(target[0]) - float(start[0])),
                float(start[1]) + t * (float(target[1]) - float(start[1])),
            )
            radial = math.hypot(*sample)
            navigation_limit = (
                float(env_radius)
                + max(0.0, float(self.config.navigation_outside_margin))
                - max(0.0, float(self.config.boundary_margin))
            )
            if radial > navigation_limit:
                return reject("navigation_boundary")
            sample_clearance = zone.signed_distance_world(*sample)
            target_limit = 0.0 if allow_target_zone else self.config.target_buffer
            if start_clearance >= target_limit:
                if sample_clearance < target_limit:
                    return reject("target_zone")
            elif sample_clearance < start_clearance - 0.02:
                # Existing overlap may escape, but recovery must not drive a rover
                # deeper through delivered material.
                return reject("worsens_existing_target_intrusion")
            if _distance(sample, blocker.position) < start_blocker_dist - 0.015:
                return reject("moves_toward_blocker")
            if not allow_pebbles:
                clearance = float(mover.static_clearance) + self.config.pebble_radius
                for pebble, start_distance in zip(pebbles, pebble_start_distances):
                    sample_distance = _distance(sample, pebble)
                    if start_distance >= clearance:
                        if sample_distance < clearance:
                            return reject("new_pebble_intrusion")
                    elif sample_distance < start_distance - self.config.pebble_penetration_tolerance:
                        return reject("worsens_existing_pebble_intrusion")
            for other in by_idx.values():
                if other.idx in (mover.idx, blocker.idx):
                    continue
                required = max(
                    self.config.other_rover_clearance,
                    float(mover.collision_radius)
                    + float(other.collision_radius)
                    + self.config.footprint_trigger_clearance,
                )
                if _distance(sample, other.position) < required:
                    return reject("other_rover")
        return True

    def _drive_to_signed(
        self,
        agent: RecoveryAgent,
        target: Vec2,
        reverse: bool,
        speed_scale: float = 1.0,
    ) -> Control:
        dx = float(target[0]) - agent.position[0]
        dy = float(target[1]) - agent.position[1]
        distance = math.hypot(dx, dy)
        if distance <= 0.05:
            return 0.0, 0.0
        travel_heading = math.atan2(dy, dx)
        desired_body_heading = _wrap(travel_heading + (math.pi if reverse else 0.0))
        error = _wrap(desired_body_heading - float(agent.state[2]))
        turn = max(
            -self.config.turn_limit,
            min(self.config.turn_limit, self.config.turn_gain * error),
        )
        if abs(error) > math.radians(55.0):
            return 0.0, turn
        align = max(0.0, math.cos(error))
        speed = (
            self.config.drive_speed
            * max(0.1, float(speed_scale))
            * align * align
            * min(1.0, distance / 0.35)
        )
        return (-speed if reverse else speed), turn

    def _drive_to(self, agent: RecoveryAgent, target: Vec2) -> Control:
        dx = float(target[0]) - agent.position[0]
        dy = float(target[1]) - agent.position[1]
        distance = math.hypot(dx, dy)
        if distance <= 0.05:
            return 0.0, 0.0
        desired = math.atan2(dy, dx)
        error = _wrap(desired - float(agent.state[2]))
        turn = max(-self.config.turn_limit, min(self.config.turn_limit, self.config.turn_gain * error))
        if abs(error) > math.radians(55.0):
            return 0.0, turn
        align = max(0.0, math.cos(error))
        speed = self.config.drive_speed * align * align * min(1.0, distance / 0.35)
        return speed, turn

    def _finish(self, sim_time: float, messages: list, reason: str) -> None:
        if self.episode is None:
            return
        pair = tuple(sorted((self.episode.winner_idx, self.episode.yielder_idx)))
        self._cooldown_until[pair] = float(sim_time) + self.config.cooldown
        self._close_since.pop(pair, None)
        messages.append(f"[RECOVERY] release {pair}: {reason}")
        self.episode = None


