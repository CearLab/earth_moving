"""Structured, flush-on-write diagnostics for multi-rover simulation runs."""

from __future__ import annotations

import csv
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import uuid
from typing import Any, Dict, Optional


def _json_safe(value: Any) -> Any:
    """Convert numpy-like and custom values to finite JSON-compatible data."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set, frozenset)):
        return [_json_safe(item) for item in value]
    if hasattr(value, "tolist"):
        return _json_safe(value.tolist())
    if hasattr(value, "x") and hasattr(value, "y"):
        return [float(value.x), float(value.y)]
    try:
        numeric = float(value)
        return numeric if math.isfinite(numeric) else str(numeric)
    except (TypeError, ValueError):
        return str(value)


class SimulationEventLogger:
    """Write detailed JSONL events and one compact CSV row per completed task."""

    SUMMARY_FIELDS = (
        "run_id", "task_id", "agent_id", "rover_type", "task_type",
        "plan_id", "map_epoch", "start_sim_time", "end_sim_time",
        "actual_duration_s", "estimated_lower_s", "estimated_upper_s",
        "outcome", "material_value_mode",
        "expected_objects", "capacity_objects",
        "expected_material_mass", "capacity_material_mass",
        "expected_planning_quantity", "capacity_planning_quantity",
        "allocated_waypoints", "approach_waypoints", "max_path_radius_m",
        "phase_durations_json",
    )

    def __init__(self, log_dir: str | Path, pose_interval: float = 0.50):
        self.log_dir = Path(log_dir).resolve()
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.run_id = f"{stamp}_{uuid.uuid4().hex[:6]}"
        self.jsonl_path = self.log_dir / f"simulation_{self.run_id}.jsonl"
        self.summary_path = self.log_dir / f"simulation_{self.run_id}_tasks.csv"
        self.latest_path = self.log_dir / "LATEST_RUN.txt"
        self.pose_interval = max(0.05, float(pose_interval))
        self._lock = threading.RLock()
        self._jsonl = self.jsonl_path.open("a", encoding="utf-8", buffering=1)
        self._summary = self.summary_path.open("a", encoding="utf-8", newline="", buffering=1)
        self._summary_writer = csv.DictWriter(self._summary, fieldnames=self.SUMMARY_FIELDS)
        self._summary_writer.writeheader()
        self._active: Dict[str, Dict[str, Any]] = {}
        self._task_counter = 0
        self._closed = False
        self.latest_path.write_text(
            f"JSONL={self.jsonl_path}\nTASKS_CSV={self.summary_path}\n",
            encoding="utf-8",
        )
        self.event("RUN_STARTED", 0.0, log_schema=1)

    def event(self, event_type: str, sim_time: float, agent_id: Optional[str] = None, **data: Any) -> None:
        with self._lock:
            if self._closed:
                return
            record = {
                "run_id": self.run_id,
                "wall_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                "sim_time": float(sim_time),
                "event": str(event_type),
            }
            if agent_id is not None:
                record["agent_id"] = str(agent_id)
            record.update(data)
            self._jsonl.write(json.dumps(_json_safe(record), ensure_ascii=False, separators=(",", ":")) + "\n")
            self._jsonl.flush()

    def start_task(self, agent_id: str, sim_time: float, **task: Any) -> str:
        with self._lock:
            if agent_id in self._active:
                self.finish_task(agent_id, sim_time, "replaced_by_new_assignment")
            self._task_counter += 1
            task_id = f"{agent_id}-T{self._task_counter:04d}"
            record = dict(task)
            record.update({
                "task_id": task_id,
                "agent_id": str(agent_id),
                "start_sim_time": float(sim_time),
                "phase": "APPROACH",
                "phase_started_at": float(sim_time),
                "phase_durations": {},
            })
            self._active[str(agent_id)] = record
            self.event("TASK_ASSIGNED", sim_time, agent_id, **task, task_id=task_id, phase="APPROACH")
            return task_id

    def phase(self, agent_id: str, phase: str, sim_time: float, **data: Any) -> None:
        with self._lock:
            active = self._active.get(str(agent_id))
            previous = None
            elapsed = None
            task_id = None
            if active is not None:
                previous = active.get("phase")
                elapsed = max(0.0, float(sim_time) - float(active.get("phase_started_at", sim_time)))
                if previous:
                    durations = active.setdefault("phase_durations", {})
                    durations[previous] = float(durations.get(previous, 0.0)) + elapsed
                active["phase"] = str(phase)
                active["phase_started_at"] = float(sim_time)
                task_id = active.get("task_id")
            self.event(
                "PHASE_CHANGED", sim_time, agent_id,
                task_id=task_id, previous_phase=previous, phase=str(phase),
                previous_phase_duration_s=elapsed, **data,
            )

    def finish_task(self, agent_id: str, sim_time: float, outcome: str = "completed", **data: Any) -> None:
        with self._lock:
            active = self._active.pop(str(agent_id), None)
            if active is None:
                self.event("TASK_FINISHED_WITHOUT_ACTIVE_RECORD", sim_time, agent_id, outcome=outcome, **data)
                return
            phase = active.get("phase")
            elapsed = max(0.0, float(sim_time) - float(active.get("phase_started_at", sim_time)))
            if phase:
                durations = active.setdefault("phase_durations", {})
                durations[phase] = float(durations.get(phase, 0.0)) + elapsed
            actual = max(0.0, float(sim_time) - float(active["start_sim_time"]))
            self.event(
                "TASK_FINISHED", sim_time, agent_id,
                task_id=active["task_id"], outcome=outcome,
                actual_duration_s=actual,
                estimated_lower_s=active.get("estimated_lower_s"),
                estimated_upper_s=active.get("estimated_upper_s"),
                phase_durations=active.get("phase_durations", {}), **data,
            )
            row = {
                "run_id": self.run_id,
                "task_id": active.get("task_id"),
                "agent_id": agent_id,
                "rover_type": active.get("rover_type"),
                "task_type": active.get("task_type"),
                "plan_id": active.get("plan_id"),
                "map_epoch": active.get("map_epoch"),
                "start_sim_time": active.get("start_sim_time"),
                "end_sim_time": float(sim_time),
                "actual_duration_s": actual,
                "estimated_lower_s": active.get("estimated_lower_s"),
                "estimated_upper_s": active.get("estimated_upper_s"),
                "outcome": outcome,
                "material_value_mode": active.get("material_value_mode"),
                "expected_objects": active.get("expected_objects"),
                "capacity_objects": active.get("capacity_objects"),
                "expected_material_mass": active.get("expected_material_mass"),
                "capacity_material_mass": active.get("capacity_material_mass"),
                "expected_planning_quantity": active.get("expected_planning_quantity"),
                "capacity_planning_quantity": active.get("capacity_planning_quantity"),
                "allocated_waypoints": active.get("allocated_waypoints"),
                "approach_waypoints": active.get("approach_waypoints"),
                "max_path_radius_m": active.get("max_path_radius_m"),
                "phase_durations_json": json.dumps(_json_safe(active.get("phase_durations", {})), separators=(",", ":")),
            }
            self._summary_writer.writerow(row)
            self._summary.flush()

    def close(self, sim_time: float, outcome: str = "simulation_stopped") -> None:
        with self._lock:
            if self._closed:
                return
            for agent_id in list(self._active):
                self.finish_task(agent_id, sim_time, outcome)
            self.event("RUN_FINISHED", sim_time, outcome=outcome)
            self._closed = True
            self._jsonl.flush()
            self._summary.flush()
            self._jsonl.close()
            self._summary.close()

