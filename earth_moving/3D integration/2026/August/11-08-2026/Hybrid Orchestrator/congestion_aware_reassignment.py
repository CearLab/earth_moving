"""Small, optional task-level response to persistent approach congestion.

This module does not perform collision avoidance.  It only observes whether an
APPROACH is making useful progress while conflict controllers repeatedly hold
or recover it.  The orchestrator can then cancel the least-committed approach,
stage that rover, and temporarily avoid task starts in the same work area.
"""
from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Iterable, Optional, Tuple


Vec2 = Tuple[float, float]


@dataclass(frozen=True)
class CongestionConfig:
    enabled: bool = False
    window_s: float = 20.0
    sample_interval_s: float = 0.50
    minimum_task_age_s: float = 10.0
    minimum_gate_progress_m: float = 0.15
    conflict_fraction: float = 0.50
    failed_replans: int = 3
    same_blocker_samples: int = 5
    cooldown_s: float = 40.0
    exclusion_radius_m: float = 1.35
    rearm_s: float = 15.0


@dataclass(frozen=True)
class ApproachObservation:
    idx: int
    agent_id: str
    sim_time: float
    task_id: Optional[str]
    task_started_at: float
    position: Vec2
    gate: Vec2
    gate_distance_m: float
    conflicted: bool
    blocker_indices: Tuple[int, ...] = ()
    failed_replans_total: int = 0
    expected_load_ratio: float = 0.0
    task_type: str = "unknown"
    priority: float = 60.0


@dataclass(frozen=True)
class CongestionDecision:
    mover_idx: int
    member_indices: Tuple[int, ...]
    blocker_indices: Tuple[int, ...]
    gate_progress_m: float
    conflict_fraction: float
    failed_replans: int
    dominant_blocker_idx: Optional[int]
    commitment: Tuple[float, ...]
    task_zone: Vec2
    conflict_zone: Vec2
    diagnostics: dict = field(default_factory=dict)


class ApproachCongestionMonitor:
    """Sliding-window detector with one deterministic reassignment decision."""

    def __init__(self, config: Optional[CongestionConfig] = None):
        self.config = config or CongestionConfig()
        self._samples: Dict[int, Deque[ApproachObservation]] = {}
        self._last_sample_at: Dict[int, float] = {}
        self._last_task_id: Dict[int, Optional[str]] = {}
        self._rearm_until: Dict[int, float] = {}

    def reset_agent(self, idx: int, sim_time: Optional[float] = None) -> None:
        idx = int(idx)
        self._samples.pop(idx, None)
        self._last_sample_at.pop(idx, None)
        self._last_task_id.pop(idx, None)
        if sim_time is not None:
            self._rearm_until[idx] = float(sim_time) + float(self.config.rearm_s)

    @staticmethod
    def _commitment(observation: ApproachObservation, progress_fraction: float):
        # Larger means the task is more valuable to preserve.  The tuple is
        # intentionally simple and lexicographic: progress dominates load.
        task_bonus = 1.0 if observation.task_type == "target" else 0.0
        return (
            round(max(0.0, min(1.0, progress_fraction)), 6),
            round(max(0.0, observation.expected_load_ratio), 6),
            task_bonus,
            -float(observation.priority),
            -float(observation.idx),
        )

    def update(
        self,
        observations: Iterable[ApproachObservation],
        sim_time: float,
    ) -> Optional[CongestionDecision]:
        if not self.config.enabled:
            return None
        now = float(sim_time)
        current = {int(item.idx): item for item in observations}
        for idx in tuple(self._samples):
            if idx not in current:
                self.reset_agent(idx)

        candidates = []
        for idx, observation in current.items():
            if observation.task_id != self._last_task_id.get(idx):
                self._samples[idx] = deque()
                self._last_sample_at.pop(idx, None)
                self._last_task_id[idx] = observation.task_id
            if now < self._rearm_until.get(idx, -1e9):
                continue
            if (
                now - self._last_sample_at.get(idx, -1e9)
                >= self.config.sample_interval_s
            ):
                self._samples.setdefault(idx, deque()).append(observation)
                self._last_sample_at[idx] = now
            samples = self._samples.setdefault(idx, deque())
            while samples and now - samples[0].sim_time > self.config.window_s:
                samples.popleft()
            if len(samples) < 2:
                continue
            covered = samples[-1].sim_time - samples[0].sim_time
            task_age = now - float(observation.task_started_at)
            if covered < 0.80 * self.config.window_s:
                continue
            if task_age < self.config.minimum_task_age_s:
                continue

            gate_progress = samples[0].gate_distance_m - samples[-1].gate_distance_m
            conflicted_samples = sum(bool(sample.conflicted) for sample in samples)
            conflict_fraction = conflicted_samples / float(len(samples))
            failed_replans = max(
                0,
                int(samples[-1].failed_replans_total)
                - int(samples[0].failed_replans_total),
            )
            blockers = Counter(
                blocker
                for sample in samples
                for blocker in sample.blocker_indices
                if int(blocker) != idx
            )
            dominant_blocker = blockers.most_common(1)[0] if blockers else (None, 0)
            repeated_blocker = dominant_blocker[1] >= self.config.same_blocker_samples
            conflict_trigger = (
                conflict_fraction >= self.config.conflict_fraction
                or failed_replans >= self.config.failed_replans
            )
            if gate_progress >= self.config.minimum_gate_progress_m:
                continue
            if not conflict_trigger or not repeated_blocker:
                continue

            initial_distance = max(samples[0].gate_distance_m, 1e-9)
            progress_fraction = max(
                0.0,
                min(1.0, (initial_distance - samples[-1].gate_distance_m) / initial_distance),
            )
            candidates.append((
                self._commitment(observation, progress_fraction),
                observation,
                gate_progress,
                conflict_fraction,
                failed_replans,
                dominant_blocker[0],
                dict(blockers),
            ))

        if not candidates:
            return None

        commitment, mover, gate_progress, fraction, failures, dominant, blockers = min(
            candidates, key=lambda item: item[0]
        )
        member_indices = tuple(sorted({mover.idx, *blockers.keys()}))
        blocker_positions = [
            current[idx].position for idx in blockers if idx in current
        ]
        points = [mover.position, *blocker_positions]
        conflict_zone = (
            sum(point[0] for point in points) / len(points),
            sum(point[1] for point in points) / len(points),
        )
        decision = CongestionDecision(
            mover_idx=int(mover.idx),
            member_indices=member_indices,
            blocker_indices=tuple(sorted(blockers)),
            gate_progress_m=float(gate_progress),
            conflict_fraction=float(fraction),
            failed_replans=int(failures),
            dominant_blocker_idx=(int(dominant) if dominant is not None else None),
            commitment=commitment,
            task_zone=(float(mover.gate[0]), float(mover.gate[1])),
            conflict_zone=(float(conflict_zone[0]), float(conflict_zone[1])),
            diagnostics={
                "task_age_s": now - float(mover.task_started_at),
                "window_s": self.config.window_s,
                "sample_count": len(self._samples[mover.idx]),
                "blocker_sample_counts": blockers,
            },
        )
        self.reset_agent(mover.idx, now)
        return decision
