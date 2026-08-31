"""Fleet-level deadlock detection and coordinated escape control.

The existing collision controller is intentionally fast and pairwise.  This
module handles the slower failure mode where a connected group of rovers has
stopped making progress.  It owns the complete conflict component, selects one
right-of-way winner, and moves the remaining members one at a time to verified
escape routes.

`PUSH` is a high-commitment phase, not an absolute lock.  A single pusher wins
against non-pushers.  If several pushers conflict, the pusher closest to
finishing wins and the other pushers first try a task-preserving reverse move.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import itertools
import math
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Set, Tuple


Vec2 = Tuple[float, float]
Control = Tuple[float, float]


def _distance(a: Vec2, b: Vec2) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _wrap(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class FleetRecoveryAgent:
    idx: int
    agent_id: str
    state: Sequence[float]
    active: bool
    phase: str
    priority: float
    collision_radius: float
    remaining_path_m: Optional[float] = None
    expected_quantity: float = 0.0
    path_progress_m: Optional[float] = None

    @property
    def position(self) -> Vec2:
        return float(self.state[0]), float(self.state[1])

    @property
    def linear_speed(self) -> float:
        return float(self.state[3]) if len(self.state) > 3 else 0.0

    @property
    def angular_speed(self) -> float:
        return float(self.state[4]) if len(self.state) > 4 else 0.0


@dataclass(frozen=True)
class EscapeRoute:
    points: Tuple[Vec2, ...]
    reverse: bool
    mode: str

    @property
    def goal(self) -> Optional[Vec2]:
        return self.points[-1] if self.points else None


@dataclass
class FleetDeadlockConfig:
    # This is deliberately at least as conservative as the local recovery
    # release envelope.  It removes the old stop/recovery threshold dead band.
    trigger_clearance: float = 0.24
    release_clearance: float = 0.36
    stuck_duration: float = 3.0
    stuck_speed: float = 0.06
    stuck_angular_speed: float = 0.10
    stuck_command: float = 0.04
    progress_epsilon: float = 0.055
    progress_path_epsilon: float = 0.05
    release_hold_time: float = 0.80
    waypoint_tolerance: float = 0.09
    drive_speed: float = 0.30
    reverse_speed: float = 0.24
    turn_gain: float = 3.2
    turn_limit: float = 5.0
    # Escape rovers often need several seconds to turn in place before any
    # translational progress is visible.  The progress watchdog also observes
    # heading error, so a stable route is kept while the rover is aligning.
    route_progress_timeout: float = 5.0
    route_timeout: float = 14.0
    total_timeout: float = 35.0
    heading_progress_epsilon: float = math.radians(4.0)
    retain_push_attempts: int = 2
    max_route_attempts: int = 5
    cooldown: float = 1.5


@dataclass
class _ComponentTrack:
    last_progress_at: float
    positions: Dict[int, Vec2]
    path_progress: Dict[int, Optional[float]]


@dataclass
class FleetRecoveryEpisode:
    component: Tuple[int, ...]
    winner_idx: int
    mover_order: Tuple[int, ...]
    original_phases: Dict[int, str]
    started_at: float
    phase_started_at: float
    last_progress_at: float
    current_mover_idx: Optional[int] = None
    mover_cursor: int = 0
    route: Optional[EscapeRoute] = None
    waypoint_index: int = 1
    route_attempt: int = 0
    mover_start: Optional[Vec2] = None
    best_mover_distance: float = 0.0
    best_goal_distance: Optional[float] = None
    best_heading_error: Optional[float] = None
    last_group_progress_at: float = 0.0
    clear_since: Optional[float] = None
    member_clear_since: Dict[int, float] = field(default_factory=dict)
    aborted_task_indices: Set[int] = field(default_factory=set)
    recovery_cycle: int = 0
    blocked: bool = False


@dataclass
class FleetRecoveryResult:
    controls: Dict[int, Control]
    active_component: Tuple[int, ...] = ()
    winner_idx: Optional[int] = None
    mover_idx: Optional[int] = None
    phase: Optional[str] = None
    abort_task_indices: Set[int] = field(default_factory=set)
    resume_indices: Set[int] = field(default_factory=set)
    events: List[dict] = field(default_factory=list)


RoutePlanner = Callable[
    [FleetRecoveryAgent, Sequence[FleetRecoveryAgent], bool, int],
    Optional[EscapeRoute],
]


class FleetDeadlockRecoveryManager:
    """Own long-lived connected deadlocks after local pair recovery stalls."""

    def __init__(self, config: Optional[FleetDeadlockConfig] = None):
        self.config = config or FleetDeadlockConfig()
        self.episode: Optional[FleetRecoveryEpisode] = None
        self._tracks: Dict[Tuple[int, ...], _ComponentTrack] = {}
        self._cooldown_until: Dict[Tuple[int, ...], float] = {}

    def owned_pairs(self) -> Set[Tuple[int, int]]:
        if self.episode is None:
            return set()
        return {
            tuple(sorted(pair))
            for pair in itertools.combinations(self.episode.component, 2)
        }

    def pair_trigger_distance(
        self, a: FleetRecoveryAgent, b: FleetRecoveryAgent,
    ) -> float:
        return (
            float(a.collision_radius)
            + float(b.collision_radius)
            + float(self.config.trigger_clearance)
        )

    def pair_release_distance(
        self, a: FleetRecoveryAgent, b: FleetRecoveryAgent,
    ) -> float:
        return (
            float(a.collision_radius)
            + float(b.collision_radius)
            + float(self.config.release_clearance)
        )

    def update(
        self,
        agents: Sequence[FleetRecoveryAgent],
        controls: Dict[int, Control],
        sim_time: float,
        route_planner: RoutePlanner,
    ) -> FleetRecoveryResult:
        now = float(sim_time)
        active = {int(a.idx): a for a in agents if a.active}
        out = dict(controls)
        result = FleetRecoveryResult(out)

        if self.episode is not None:
            present = set(self.episode.component).intersection(active)
            if len(present) < 2:
                self._finish(now, result, "component_no_longer_active")
            else:
                self._expand_episode_if_needed(active, now, result)
                self._shrink_episode_if_needed(active, now, result)
                if self.episode is not None:
                    self._run_episode(active, now, route_planner, result)

        if self.episode is None:
            candidate = self._find_stuck_component(tuple(active.values()), out, now)
            if candidate:
                self._start_episode(candidate, now, route_planner, result)
                if self.episode is not None:
                    self._run_episode(active, now, route_planner, result)

        episode = self.episode
        if episode is not None:
            result.active_component = episode.component
            result.winner_idx = episode.winner_idx
            result.mover_idx = episode.current_mover_idx
            result.phase = "BLOCKED_SAFE_STOP" if episode.blocked else "GROUP_ESCAPE"
        return result

    def _components(
        self, agents: Sequence[FleetRecoveryAgent],
    ) -> List[Tuple[FleetRecoveryAgent, ...]]:
        by_idx = {a.idx: a for a in agents}
        adjacency = {a.idx: set() for a in agents}
        for left, right in itertools.combinations(agents, 2):
            if _distance(left.position, right.position) <= self.pair_trigger_distance(left, right):
                adjacency[left.idx].add(right.idx)
                adjacency[right.idx].add(left.idx)
        components = []
        unseen = set(adjacency)
        while unseen:
            seed = unseen.pop()
            stack = [seed]
            members = {seed}
            while stack:
                current = stack.pop()
                for neighbor in adjacency[current]:
                    if neighbor in unseen:
                        unseen.remove(neighbor)
                        members.add(neighbor)
                        stack.append(neighbor)
            if len(members) >= 2:
                components.append(tuple(by_idx[idx] for idx in sorted(members)))
        return components

    def _find_stuck_component(
        self,
        agents: Sequence[FleetRecoveryAgent],
        controls: Dict[int, Control],
        now: float,
    ) -> Optional[Tuple[FleetRecoveryAgent, ...]]:
        current_keys = set()
        candidates = []
        for component in self._components(agents):
            key = tuple(a.idx for a in component)
            current_keys.add(key)
            if now < self._cooldown_until.get(key, -1e9):
                self._tracks.pop(key, None)
                continue
            track = self._tracks.get(key)
            if track is None:
                self._tracks[key] = _ComponentTrack(
                    last_progress_at=now,
                    positions={a.idx: a.position for a in component},
                    path_progress={a.idx: a.path_progress_m for a in component},
                )
                continue

            moved = any(
                _distance(a.position, track.positions.get(a.idx, a.position))
                >= self.config.progress_epsilon
                for a in component
            )
            path_advanced = any(
                a.path_progress_m is not None
                and track.path_progress.get(a.idx) is not None
                and abs(float(a.path_progress_m) - float(track.path_progress[a.idx]))
                >= self.config.progress_path_epsilon
                for a in component
            )
            slow = all(
                abs(a.linear_speed) <= self.config.stuck_speed
                and not (
                    str(a.phase).upper() == "TURN_TO_PUSH"
                    and abs(a.angular_speed) > self.config.stuck_angular_speed
                )
                for a in component
            )
            # Measured motion is authoritative.  A rover can have a non-zero
            # command while its wheels are physically blocked by another rover.
            # That is exactly the case this slower fleet layer must catch.
            if moved or path_advanced or not slow:
                track.last_progress_at = now
                track.positions = {a.idx: a.position for a in component}
                track.path_progress = {a.idx: a.path_progress_m for a in component}
                continue
            if now - track.last_progress_at >= self.config.stuck_duration:
                closest = min(
                    _distance(a.position, b.position)
                    for a, b in itertools.combinations(component, 2)
                )
                candidates.append((closest, component))

        for key in tuple(self._tracks):
            if key not in current_keys:
                self._tracks.pop(key, None)
        if not candidates:
            return None
        return min(candidates, key=lambda item: item[0])[1]

    @staticmethod
    def _is_pusher(phase: str) -> bool:
        return str(phase).upper() == "PUSH"

    def _winner_key(self, agent: FleetRecoveryAgent):
        remaining = (
            float(agent.remaining_path_m)
            if agent.remaining_path_m is not None and math.isfinite(float(agent.remaining_path_m))
            else float("inf")
        )
        return (
            remaining,
            -max(0.0, float(agent.expected_quantity)),
            float(agent.priority),
            int(agent.idx),
        )

    def _choose_winner(
        self,
        component: Sequence[FleetRecoveryAgent],
        preferred_idx: Optional[int] = None,
    ) -> FleetRecoveryAgent:
        pushers = [agent for agent in component if self._is_pusher(agent.phase)]
        if pushers:
            preferred_pusher = next(
                (agent for agent in pushers if agent.idx == preferred_idx), None,
            )
            if preferred_pusher is not None:
                return preferred_pusher
            return min(pushers, key=self._winner_key)
        # A rover whose task was already aborted is an evacuee, never the
        # normal right-of-way winner.  This prevents group expansion from
        # promoting GROUP_ESCAPE merely because its synthetic priority is low.
        productive = [
            agent for agent in component
            if str(agent.phase).upper() != "GROUP_ESCAPE"
        ]
        eligible = productive or list(component)
        preferred = next(
            (agent for agent in eligible if agent.idx == preferred_idx), None,
        )
        if preferred is not None:
            return preferred
        return min(eligible, key=lambda a: (float(a.priority), int(a.idx)))

    def _mover_key(self, agent: FleetRecoveryAgent):
        phase_order = {
            "PARKING": 0,
            "PARKED": 1,
            "IDLE": 2,
            "PLANNING": 3,
            "GROUP_ESCAPE": 4,
            "APPROACH": 5,
            "TURN_TO_PUSH": 6,
            "ROLLBACK": 7,
            "TARGET_EXIT": 8,
            "PUSH": 9,
        }
        return (
            phase_order.get(str(agent.phase).upper(), 5),
            -float(agent.priority),
            int(agent.idx),
        )

    def _start_episode(
        self,
        component: Sequence[FleetRecoveryAgent],
        now: float,
        route_planner: RoutePlanner,
        result: FleetRecoveryResult,
    ) -> None:
        winner = self._choose_winner(component)
        movers = tuple(
            agent.idx
            for agent in sorted(
                (a for a in component if a.idx != winner.idx),
                key=self._mover_key,
            )
        )
        self.episode = FleetRecoveryEpisode(
            component=tuple(sorted(a.idx for a in component)),
            winner_idx=winner.idx,
            mover_order=movers,
            original_phases={a.idx: str(a.phase) for a in component},
            started_at=now,
            phase_started_at=now,
            last_progress_at=now,
            last_group_progress_at=now,
        )
        result.events.append({
            "event": "GROUP_DEADLOCK_STARTED",
            "component": list(self.episode.component),
            "winner_idx": winner.idx,
            "winner_phase": winner.phase,
            "pusher_indices": [a.idx for a in component if self._is_pusher(a.phase)],
            "mover_order": list(movers),
        })
        self._assign_next_route(
            {a.idx: a for a in component}, now, route_planner, result,
        )

    def _expand_episode_if_needed(
        self,
        active: Dict[int, FleetRecoveryAgent],
        now: float,
        result: FleetRecoveryResult,
    ) -> None:
        episode = self.episode
        if episode is None:
            return
        members = set(episode.component)
        changed = True
        while changed:
            changed = False
            for candidate in active.values():
                if candidate.idx in members:
                    continue
                if any(
                    _distance(candidate.position, active[idx].position)
                    <= self.pair_trigger_distance(candidate, active[idx])
                    for idx in members if idx in active
                ):
                    members.add(candidate.idx)
                    changed = True
        if members == set(episode.component):
            return
        expanded = tuple(active[idx] for idx in sorted(members) if idx in active)
        winner = self._choose_winner(expanded, preferred_idx=episode.winner_idx)
        previous = episode.component
        episode.component = tuple(a.idx for a in expanded)
        episode.winner_idx = winner.idx
        # Preserve the real task phase recorded at episode entry.  Existing
        # members may now report GROUP_ESCAPE after their task was aborted.
        for agent in expanded:
            episode.original_phases.setdefault(agent.idx, str(agent.phase))
        episode.mover_order = tuple(
            a.idx for a in sorted(
                (a for a in expanded if a.idx != winner.idx), key=self._mover_key,
            )
        )
        episode.mover_cursor = 0
        episode.current_mover_idx = None
        episode.route = None
        episode.route_attempt = 0
        episode.phase_started_at = now
        episode.last_progress_at = now
        episode.last_group_progress_at = now
        episode.best_goal_distance = None
        episode.best_heading_error = None
        episode.member_clear_since.clear()
        result.events.append({
            "event": "GROUP_DEADLOCK_EXPANDED",
            "previous_component": list(previous),
            "component": list(episode.component),
            "winner_idx": winner.idx,
        })

    def _shrink_episode_if_needed(
        self,
        active: Dict[int, FleetRecoveryAgent],
        now: float,
        result: FleetRecoveryResult,
    ) -> None:
        """Release members that have independently cleared the live conflict.

        A rover must remain outside every other member's release envelope for
        the configured hold time.  The unresolved subgroup continues recovery;
        a rover that has already escaped is never frozen by the old component.
        """
        episode = self.episode
        if episode is None:
            return
        members = [active[idx] for idx in episode.component if idx in active]
        if len(members) < 2 or self._component_clear(episode, active):
            return

        clear_now = set()
        for member in members:
            others = [other for other in members if other.idx != member.idx]
            if others and all(
                _distance(member.position, other.position)
                >= self.pair_release_distance(member, other)
                for other in others
            ):
                clear_now.add(member.idx)
                episode.member_clear_since.setdefault(member.idx, now)
            else:
                episode.member_clear_since.pop(member.idx, None)

        released = {
            idx for idx in clear_now
            if now - episode.member_clear_since.get(idx, now)
            >= self.config.release_hold_time
        }
        if not released:
            return

        previous = episode.component
        remaining = tuple(idx for idx in episode.component if idx not in released)
        result.resume_indices.update(released)
        result.events.append({
            "event": "GROUP_MEMBERS_RELEASED",
            "previous_component": list(previous),
            "released_indices": sorted(released),
            "component": list(remaining),
            "reason": "independent_clearance_held",
        })
        episode.member_clear_since = {
            idx: value for idx, value in episode.member_clear_since.items()
            if idx in remaining
        }
        episode.component = remaining
        episode.last_group_progress_at = now

        if len(remaining) < 2:
            self._finish(now, result, "unresolved_subgroup_too_small")
            return

        remaining_agents = [active[idx] for idx in remaining if idx in active]
        previous_winner = episode.winner_idx
        winner = self._choose_winner(
            remaining_agents,
            preferred_idx=(previous_winner if previous_winner in remaining else None),
        )
        episode.winner_idx = winner.idx
        episode.mover_order = tuple(
            agent.idx for agent in sorted(
                (agent for agent in remaining_agents if agent.idx != winner.idx),
                key=self._mover_key,
            )
        )
        # Replan against the smaller live component; the previous escape route
        # may have been selected around a rover that has just been released.
        episode.current_mover_idx = None
        episode.route = None
        episode.route_attempt = 0
        episode.mover_cursor = 0
        episode.phase_started_at = now
        episode.last_progress_at = now
        episode.best_goal_distance = None
        episode.best_heading_error = None

    def _component_clear(
        self, episode: FleetRecoveryEpisode, active: Dict[int, FleetRecoveryAgent],
    ) -> bool:
        members = [active[idx] for idx in episode.component if idx in active]
        if len(members) < 2:
            return True
        return all(
            _distance(left.position, right.position)
            >= self.pair_release_distance(left, right)
            for left, right in itertools.combinations(members, 2)
        )

    @staticmethod
    def _route_heading_error(
        mover: FleetRecoveryAgent, target: Vec2, reverse: bool,
    ) -> float:
        travel_heading = math.atan2(
            float(target[1]) - mover.position[1],
            float(target[0]) - mover.position[0],
        )
        desired = _wrap(travel_heading + (math.pi if reverse else 0.0))
        return abs(_wrap(desired - float(mover.state[2])))

    def _rearbitrate_episode(
        self,
        active: Dict[int, FleetRecoveryAgent],
        now: float,
        result: FleetRecoveryResult,
        reason: str,
        rotate_winner: bool,
    ) -> None:
        """Start another bounded recovery cycle without latching a fleet stop."""
        episode = self.episode
        if episode is None:
            return
        members = [active[idx] for idx in episode.component if idx in active]
        if len(members) < 2:
            self._finish(now, result, "component_no_longer_active")
            return

        previous_winner = episode.winner_idx
        if rotate_winner:
            ordered = [agent.idx for agent in sorted(members, key=self._mover_key)]
            alternatives = [idx for idx in ordered if idx != previous_winner]
            # Making another member the temporary winner lets the former
            # winner become a mover.  This is the final fallback when every
            # ordinary yielder route failed, including all-pusher conflicts.
            winner_idx = alternatives[episode.recovery_cycle % len(alternatives)] if alternatives else previous_winner
        else:
            winner_idx = self._choose_winner(
                members, preferred_idx=previous_winner,
            ).idx

        episode.recovery_cycle += 1
        episode.winner_idx = winner_idx
        episode.mover_order = tuple(
            agent.idx for agent in sorted(
                (agent for agent in members if agent.idx != winner_idx),
                key=self._mover_key,
            )
        )
        # Rotate which yielder tries first on successive cycles.
        if episode.mover_order:
            shift = episode.recovery_cycle % len(episode.mover_order)
            episode.mover_order = (
                episode.mover_order[shift:] + episode.mover_order[:shift]
            )
        episode.mover_cursor = 0
        episode.current_mover_idx = None
        episode.route = None
        episode.route_attempt = 0
        episode.waypoint_index = 1
        episode.mover_start = None
        episode.best_mover_distance = 0.0
        episode.best_goal_distance = None
        episode.best_heading_error = None
        episode.phase_started_at = now
        episode.last_progress_at = now
        episode.last_group_progress_at = now
        episode.clear_since = None
        episode.blocked = False
        result.events.append({
            "event": "GROUP_DEADLOCK_REARBITRATED",
            "component": list(episode.component),
            "previous_winner_idx": previous_winner,
            "winner_idx": winner_idx,
            "mover_order": list(episode.mover_order),
            "recovery_cycle": episode.recovery_cycle,
            "reason": reason,
        })

    def _run_episode(
        self,
        active: Dict[int, FleetRecoveryAgent],
        now: float,
        route_planner: RoutePlanner,
        result: FleetRecoveryResult,
    ) -> None:
        episode = self.episode
        if episode is None:
            return
        for idx in episode.component:
            result.controls[idx] = (0.0, 0.0)

        if self._component_clear(episode, active):
            if episode.clear_since is None:
                episode.clear_since = now
            if now - episode.clear_since >= self.config.release_hold_time:
                self._finish(now, result, "clearance_held")
            return
        episode.clear_since = None

        if now - episode.last_group_progress_at >= self.config.total_timeout:
            self._rearbitrate_episode(
                active, now, result,
                reason="group_no_progress_timeout",
                rotate_winner=True,
            )
            return

        if episode.current_mover_idx is None or episode.route is None:
            self._assign_next_route(active, now, route_planner, result)
            if episode.current_mover_idx is None or episode.route is None:
                return

        mover = active.get(episode.current_mover_idx)
        if mover is None:
            episode.current_mover_idx = None
            episode.route = None
            return

        route = episode.route
        previous_waypoint_index = episode.waypoint_index
        while (
            episode.waypoint_index < len(route.points)
            and _distance(mover.position, route.points[episode.waypoint_index])
            <= self.config.waypoint_tolerance
        ):
            episode.waypoint_index += 1
        if episode.waypoint_index != previous_waypoint_index:
            episode.best_goal_distance = None
            episode.best_heading_error = None
            episode.last_progress_at = now
            episode.last_group_progress_at = now
        if episode.waypoint_index >= len(route.points):
            result.events.append({
                "event": "GROUP_ESCAPE_REACHED",
                "agent_idx": mover.idx,
                "goal": list(route.goal) if route.goal else None,
                "mode": route.mode,
            })
            episode.current_mover_idx = None
            episode.route = None
            episode.route_attempt = 0
            episode.phase_started_at = now
            episode.last_progress_at = now
            episode.last_group_progress_at = now
            self._assign_next_route(active, now, route_planner, result)
            return

        moved = (
            _distance(mover.position, episode.mover_start)
            if episode.mover_start is not None else 0.0
        )
        target = route.points[episode.waypoint_index]
        goal_distance = _distance(mover.position, target)
        heading_error = self._route_heading_error(mover, target, route.reverse)
        translated = moved >= episode.best_mover_distance + self.config.progress_epsilon
        approached = (
            episode.best_goal_distance is None
            or goal_distance
            <= episode.best_goal_distance - self.config.progress_epsilon
        )
        aligned = (
            episode.best_heading_error is None
            or heading_error
            <= episode.best_heading_error - self.config.heading_progress_epsilon
        )
        if translated or approached or aligned:
            episode.best_mover_distance = max(episode.best_mover_distance, moved)
            episode.best_goal_distance = min(
                goal_distance,
                episode.best_goal_distance
                if episode.best_goal_distance is not None else goal_distance,
            )
            episode.best_heading_error = min(
                heading_error,
                episode.best_heading_error
                if episode.best_heading_error is not None else heading_error,
            )
            episode.last_progress_at = now
            episode.last_group_progress_at = now

        stalled = now - episode.last_progress_at >= self.config.route_progress_timeout
        timed_out = now - episode.phase_started_at >= self.config.route_timeout
        if stalled or timed_out:
            episode.route_attempt += 1
            result.events.append({
                "event": "GROUP_ESCAPE_REPLAN",
                "agent_idx": mover.idx,
                "attempt": episode.route_attempt,
                "reason": "no_progress" if stalled else "route_timeout",
                "retaining_push_task": (
                    self._is_pusher(episode.original_phases.get(mover.idx, ""))
                    and mover.idx not in episode.aborted_task_indices
                ),
            })
            episode.route = None
            episode.phase_started_at = now
            episode.last_progress_at = now
            self._assign_route_for_current(active, now, route_planner, result)
            return

        result.controls[mover.idx] = self._drive_to(mover, target, route.reverse)

    def _assign_next_route(
        self,
        active: Dict[int, FleetRecoveryAgent],
        now: float,
        route_planner: RoutePlanner,
        result: FleetRecoveryResult,
    ) -> None:
        episode = self.episode
        if episode is None or not episode.mover_order:
            return
        attempts = 0
        while attempts < len(episode.mover_order):
            idx = episode.mover_order[episode.mover_cursor % len(episode.mover_order)]
            episode.mover_cursor = (episode.mover_cursor + 1) % len(episode.mover_order)
            attempts += 1
            if idx not in active:
                continue
            episode.current_mover_idx = idx
            episode.route_attempt = 0
            self._assign_route_for_current(active, now, route_planner, result)
            if episode.route is not None:
                return
        episode.current_mover_idx = None
        result.events.append({
            "event": "GROUP_DEADLOCK_ESCALATED",
            "component": list(episode.component),
            "reason": "no_member_has_verified_escape_route",
        })
        self._rearbitrate_episode(
            active, now, result,
            reason="no_member_has_verified_escape_route",
            rotate_winner=True,
        )

    def _assign_route_for_current(
        self,
        active: Dict[int, FleetRecoveryAgent],
        now: float,
        route_planner: RoutePlanner,
        result: FleetRecoveryResult,
    ) -> None:
        episode = self.episode
        if episode is None or episode.current_mover_idx is None:
            return
        mover = active.get(episode.current_mover_idx)
        if mover is None:
            episode.route = None
            return
        original_phase = episode.original_phases.get(mover.idx, mover.phase)
        was_pushing = self._is_pusher(original_phase)
        retain_push = (
            was_pushing
            and mover.idx not in episode.aborted_task_indices
            and episode.route_attempt < self.config.retain_push_attempts
        )
        if not retain_push and mover.idx not in episode.aborted_task_indices:
            episode.aborted_task_indices.add(mover.idx)
            result.abort_task_indices.add(mover.idx)
        component = [active[idx] for idx in episode.component if idx in active]
        route = route_planner(mover, component, retain_push, episode.route_attempt)
        if route is None:
            if retain_push:
                episode.route_attempt = max(
                    episode.route_attempt + 1,
                    self.config.retain_push_attempts,
                )
                result.events.append({
                    "event": "GROUP_PUSH_ROLLBACK_UNAVAILABLE",
                    "agent_idx": mover.idx,
                    "action": "abort_task_and_try_general_escape",
                })
                self._assign_route_for_current(active, now, route_planner, result)
                return
            episode.route_attempt += 1
            if episode.route_attempt < self.config.max_route_attempts:
                route = route_planner(mover, component, False, episode.route_attempt)
        episode.route = route
        episode.waypoint_index = 1
        episode.phase_started_at = now
        episode.last_progress_at = now
        episode.mover_start = mover.position
        episode.best_mover_distance = 0.0
        episode.best_goal_distance = (
            _distance(mover.position, route.points[episode.waypoint_index])
            if route is not None and len(route.points) > episode.waypoint_index
            else None
        )
        episode.best_heading_error = (
            self._route_heading_error(
                mover, route.points[episode.waypoint_index], route.reverse,
            )
            if route is not None and len(route.points) > episode.waypoint_index
            else None
        )
        if route is None:
            result.events.append({
                "event": "GROUP_ESCAPE_ROUTE_FAILED",
                "agent_idx": mover.idx,
                "attempt": episode.route_attempt,
            })
            return
        result.events.append({
            "event": "GROUP_ESCAPE_ASSIGNED",
            "agent_idx": mover.idx,
            "winner_idx": episode.winner_idx,
            "goal": list(route.goal) if route.goal else None,
            "route": [list(point) for point in route.points],
            "reverse": route.reverse,
            "mode": route.mode,
            "task_retained": retain_push,
            "attempt": episode.route_attempt,
        })

    def _drive_to(
        self, agent: FleetRecoveryAgent, target: Vec2, reverse: bool,
    ) -> Control:
        dx = float(target[0]) - agent.position[0]
        dy = float(target[1]) - agent.position[1]
        distance = math.hypot(dx, dy)
        if distance <= self.config.waypoint_tolerance:
            return 0.0, 0.0
        travel_heading = math.atan2(dy, dx)
        desired = _wrap(travel_heading + (math.pi if reverse else 0.0))
        error = _wrap(desired - float(agent.state[2]))
        turn = max(
            -self.config.turn_limit,
            min(self.config.turn_limit, self.config.turn_gain * error),
        )
        if abs(error) > math.radians(55.0):
            return 0.0, turn
        align = max(0.0, math.cos(error))
        base = self.config.reverse_speed if reverse else self.config.drive_speed
        speed = base * align * align * min(1.0, distance / 0.35)
        return (-speed if reverse else speed), turn

    def _finish(
        self, now: float, result: FleetRecoveryResult, reason: str,
    ) -> None:
        episode = self.episode
        if episode is None:
            return
        result.resume_indices.update(episode.component)
        result.events.append({
            "event": "GROUP_DEADLOCK_RESOLVED",
            "component": list(episode.component),
            "winner_idx": episode.winner_idx,
            "aborted_task_indices": sorted(episode.aborted_task_indices),
            "duration_s": max(0.0, float(now) - episode.started_at),
            "reason": reason,
        })
        self._cooldown_until[episode.component] = float(now) + self.config.cooldown
        self._tracks.pop(episode.component, None)
        self.episode = None
