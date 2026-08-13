"""Profile-aware, priority-ordered trajectory reservations.

The task allocator reserves material cells. This module additionally predicts
where each rover intends to be over a short time horizon. It has no PyBullet or
planner dependency, so conflict decisions stay deterministic and testable.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, Optional, Sequence, Tuple


Vec2 = Tuple[float, float]


@dataclass(frozen=True)
class ReservationAgent:
    idx: int
    agent_id: str
    priority: float
    position: Vec2
    speed: float
    collision_radius: float
    phase: str
    active: bool
    can_replan: bool
    path_points: Tuple[Vec2, ...] = ()
    replan_failures: int = 0


@dataclass(frozen=True)
class TimedOccupancy:
    time_offset: float
    position: Vec2
    radius: float


@dataclass(frozen=True)
class ReservationConflict:
    winner_idx: int
    yielder_idx: int
    replan_idx: Optional[int]
    blocker_idx: Optional[int]
    time_to_conflict: float
    predicted_distance: float
    required_distance: float
    yielder_stalled: bool
    reason: str


@dataclass
class ReservationDecision:
    reservations: Dict[int, Tuple[TimedOccupancy, ...]] = field(default_factory=dict)
    conflicts: list = field(default_factory=list)
    replan_blockers: Dict[int, set] = field(default_factory=dict)
    blocker_points: Dict[int, Tuple[Vec2, ...]] = field(default_factory=dict)
    hold_indices: set = field(default_factory=set)


@dataclass
class PrioritizedReservationConfig:
    horizon_s: float = 7.0
    sample_dt: float = 0.35
    nominal_speed: float = 0.55
    push_speed: float = 0.38
    safety_margin: float = 0.16
    motion_epsilon: float = 0.025
    stalled_speed: float = 0.045
    stalled_after: float = 0.90
    imminent_hold_s: float = 1.60
    request_cooldown: float = 1.20


def _distance(a: Vec2, b: Vec2) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _project_remaining_path(position: Vec2, points: Sequence[Vec2]) -> Tuple[Vec2, ...]:
    clean = tuple((float(point[0]), float(point[1])) for point in points)
    if not clean:
        return (position,)
    if len(clean) == 1:
        return (position, clean[0])

    best = None
    best_key = (float("inf"), 0)
    for index, (a, b) in enumerate(zip(clean[:-1], clean[1:])):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            projection = a
        else:
            u = max(
                0.0,
                min(
                    1.0,
                    ((position[0] - a[0]) * dx + (position[1] - a[1]) * dy)
                    / length_sq,
                ),
            )
            projection = (a[0] + u * dx, a[1] + u * dy)
        key = (_distance(position, projection), index)
        if key < best_key:
            best_key = key
            best = (projection, index)

    projection, index = best if best is not None else (clean[0], 0)
    remaining = [position]
    if _distance(position, projection) > 1e-6:
        remaining.append(projection)
    remaining.extend(clean[index + 1 :])
    if len(remaining) == 1:
        remaining.append(clean[-1])
    return tuple(remaining)


def _point_at_distance(points: Sequence[Vec2], distance: float) -> Vec2:
    if not points:
        return 0.0, 0.0
    remaining = max(0.0, float(distance))
    for a, b in zip(points[:-1], points[1:]):
        segment = _distance(a, b)
        if segment <= 1e-9:
            continue
        if remaining <= segment:
            ratio = remaining / segment
            return (
                float(a[0]) + ratio * (float(b[0]) - float(a[0])),
                float(a[1]) + ratio * (float(b[1]) - float(a[1])),
            )
        remaining -= segment
    return float(points[-1][0]), float(points[-1][1])


class PrioritizedTrajectoryCoordinator:
    """Predict conflicts and choose the rover that must replan.

    Smaller numeric priority wins. A stalled yielder is treated as a physical
    obstacle, so the nominal winner routes around it instead of approaching it
    and waiting at emergency-stop distance.
    """

    def __init__(self, config: Optional[PrioritizedReservationConfig] = None):
        self.config = config or PrioritizedReservationConfig()
        self._previous_position: Dict[int, Vec2] = {}
        self._stalled_since: Dict[int, float] = {}
        self._last_request: Dict[Tuple[int, int, int], float] = {}

    def clear_agent(self, idx: int) -> None:
        self._previous_position.pop(int(idx), None)
        self._stalled_since.pop(int(idx), None)
        for key in tuple(self._last_request):
            if int(idx) in key:
                self._last_request.pop(key, None)

    def _update_motion(
        self, agents: Sequence[ReservationAgent], sim_time: float,
    ) -> Dict[int, float]:
        active = {int(agent.idx) for agent in agents if agent.active}
        for idx in tuple(self._previous_position):
            if idx not in active:
                self.clear_agent(idx)

        stalled_for = {}
        for agent in agents:
            if not agent.active:
                continue
            idx = int(agent.idx)
            previous = self._previous_position.get(idx)
            moved = 0.0 if previous is None else _distance(previous, agent.position)
            moving = (
                abs(float(agent.speed)) > self.config.stalled_speed
                or moved > self.config.motion_epsilon
            )
            if previous is None or moving:
                self._stalled_since[idx] = float(sim_time)
            else:
                self._stalled_since.setdefault(idx, float(sim_time))
            self._previous_position[idx] = agent.position
            stalled_for[idx] = max(
                0.0, float(sim_time) - self._stalled_since[idx],
            )
        return stalled_for

    def _reservation(self, agent: ReservationAgent) -> Tuple[TimedOccupancy, ...]:
        path = _project_remaining_path(agent.position, agent.path_points)
        phase = str(agent.phase).upper()
        speed = (
            self.config.push_speed
            if phase in {"PUSH", "TURN_TO_PUSH", "ROLLBACK", "TARGET_EXIT"}
            else self.config.nominal_speed
        )
        speed = max(0.08, float(speed))
        steps = max(1, int(math.ceil(self.config.horizon_s / self.config.sample_dt)))
        return tuple(
            TimedOccupancy(
                time_offset=min(
                    self.config.horizon_s, float(step) * self.config.sample_dt,
                ),
                position=_point_at_distance(
                    path,
                    speed
                    * min(
                        self.config.horizon_s,
                        float(step) * self.config.sample_dt,
                    ),
                ),
                radius=float(agent.collision_radius),
            )
            for step in range(steps + 1)
        )

    def update(
        self,
        agents: Iterable[ReservationAgent],
        sim_time: float,
        owned_pairs: Iterable[Tuple[int, int]] = (),
    ) -> ReservationDecision:
        agents = tuple(agent for agent in agents if agent.active)
        decision = ReservationDecision()
        stalled_for = self._update_motion(agents, sim_time)
        if len(agents) < 2:
            return decision

        ordered = sorted(
            agents, key=lambda agent: (float(agent.priority), int(agent.idx)),
        )
        decision.reservations = {
            int(agent.idx): self._reservation(agent) for agent in ordered
        }
        owned = {
            tuple(sorted((int(pair[0]), int(pair[1])))) for pair in owned_pairs
        }

        for winner_position, winner in enumerate(ordered):
            for yielder in ordered[winner_position + 1 :]:
                pair = tuple(sorted((int(winner.idx), int(yielder.idx))))
                if pair in owned:
                    continue
                winner_reservation = decision.reservations[int(winner.idx)]
                yielder_reservation = decision.reservations[int(yielder.idx)]
                required = (
                    float(winner.collision_radius)
                    + float(yielder.collision_radius)
                    + self.config.safety_margin
                )
                conflict_sample = None
                for winner_sample, yielder_sample in zip(
                    winner_reservation, yielder_reservation,
                ):
                    separation = _distance(
                        winner_sample.position, yielder_sample.position,
                    )
                    if separation < required:
                        conflict_sample = (
                            winner_sample.time_offset, separation,
                        )
                        break
                if conflict_sample is None:
                    continue

                time_to_conflict, predicted_distance = conflict_sample
                yielder_stalled = (
                    stalled_for.get(int(yielder.idx), 0.0)
                    >= self.config.stalled_after
                    or int(yielder.replan_failures) >= 2
                    or not yielder.can_replan
                )
                if yielder.can_replan and not yielder_stalled:
                    replan_idx = int(yielder.idx)
                    blocker_idx = int(winner.idx)
                    reason = "lower_priority_avoids_reserved_trajectory"
                    blocker_points = tuple(
                        sample.position for sample in winner_reservation
                    )
                elif winner.can_replan:
                    replan_idx = int(winner.idx)
                    blocker_idx = int(yielder.idx)
                    reason = "winner_avoids_unyielding_rover"
                    blocker_points = (yielder.position,)
                    if time_to_conflict <= self.config.imminent_hold_s:
                        decision.hold_indices.add(int(winner.idx))
                else:
                    replan_idx = None
                    blocker_idx = None
                    reason = "joint_conflict_coordinator_required"
                    blocker_points = ()

                decision.conflicts.append(
                    ReservationConflict(
                        winner_idx=int(winner.idx),
                        yielder_idx=int(yielder.idx),
                        replan_idx=replan_idx,
                        blocker_idx=blocker_idx,
                        time_to_conflict=float(time_to_conflict),
                        predicted_distance=float(predicted_distance),
                        required_distance=float(required),
                        yielder_stalled=bool(yielder_stalled),
                        reason=reason,
                    )
                )
                if replan_idx is None or blocker_idx is None:
                    continue

                request_key = (
                    int(winner.idx), int(yielder.idx), int(replan_idx),
                )
                last_request = self._last_request.get(
                    request_key, -float("inf"),
                )
                if (
                    float(sim_time) - last_request
                    < self.config.request_cooldown
                ):
                    continue
                self._last_request[request_key] = float(sim_time)
                decision.replan_blockers.setdefault(
                    replan_idx, set(),
                ).add(blocker_idx)
                decision.blocker_points[replan_idx] = blocker_points

        return decision
