"""Benchmark-grade physical attribution, environment metrics, and 2D artifacts.

The planner's predictions and the physics observations are intentionally kept in
separate columns.  A pebble is attributed to a push only after a PyBullet contact;
the swept-corridor set is retained as a weaker, geometric diagnostic.
"""

from __future__ import annotations

import csv
import gzip
import json
import math
import re
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, Mapping, Optional, Sequence

def _safe(value: Any) -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Mapping):
        return {str(key): _safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set, frozenset)):
        return [_safe(item) for item in value]
    if hasattr(value, "tolist"):
        return _safe(value.tolist())
    if hasattr(value, "x") and hasattr(value, "y"):
        return [int(value.x), int(value.y)]
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (TypeError, ValueError):
        return str(value)


def _json(value: Any) -> str:
    return json.dumps(_safe(value), ensure_ascii=False, separators=(",", ":"))


def _weighted_quantile(values: Sequence[float], weights: Sequence[float], q: float) -> float:
    if not values:
        return 0.0
    pairs = sorted(zip(values, weights), key=lambda item: item[0])
    threshold = max(0.0, min(1.0, float(q))) * sum(max(0.0, w) for _, w in pairs)
    running = 0.0
    for value, weight in pairs:
        running += max(0.0, weight)
        if running >= threshold:
            return float(value)
    return float(pairs[-1][0])


def point_to_polyline_distance(point: Sequence[float], path: Sequence[Sequence[float]]) -> float:
    """Return the shortest planar distance from a point to a polyline."""
    px, py = float(point[0]), float(point[1])
    if not path:
        return float("inf")
    if len(path) == 1:
        return math.hypot(px - float(path[0][0]), py - float(path[0][1]))
    best = float("inf")
    for left, right in zip(path, path[1:]):
        ax, ay = float(left[0]), float(left[1])
        bx, by = float(right[0]), float(right[1])
        dx, dy = bx - ax, by - ay
        denom = dx * dx + dy * dy
        t = 0.0 if denom <= 1e-12 else ((px - ax) * dx + (py - ay) * dy) / denom
        t = max(0.0, min(1.0, t))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


def _cell_weight(cell: Any, material_mode: str) -> float:
    if material_mode == "mass":
        return float(getattr(cell, "material_mass", getattr(cell, "num_objects", 0.0)))
    return float(getattr(cell, "physical_object_count", getattr(cell, "num_objects", 0.0)))


def calculate_environment_metrics(
    env: Any,
    material_mode: str = "count",
    reference_highway_ratio: float = 0.50,
) -> Dict[str, Any]:
    """Calculate topology and material-weighted highway metrics from a 2D map."""
    occupied = [cell for cell in getattr(env, "cells_with_objects", []) if _cell_weight(cell, material_mode) > 0]
    target_cells = [cell for cell in getattr(env, "target_zone_cells", []) if _cell_weight(cell, material_mode) > 0]
    outside_total = sum(_cell_weight(cell, material_mode) for cell in occupied)
    target_total = sum(_cell_weight(cell, material_mode) for cell in target_cells)
    total = outside_total + target_total
    heats = [max(0.0, float(getattr(cell, "heat_map", 0.0))) for cell in getattr(env, "all_cells", [])]
    max_heat = max(heats, default=0.0)
    planner_threshold = max(0.0, float(getattr(env, "highway_threshold", 0.0)))
    reference_threshold = max_heat * max(0.0, min(1.0, float(reference_highway_ratio)))

    def evaluate_threshold(threshold: float) -> Dict[str, Any]:
        highway = {
            (int(cell.x), int(cell.y)): cell for cell in occupied
            if max_heat > 0.0 and float(getattr(cell, "heat_map", 0.0)) >= threshold
        }
        remaining = set(highway)
        components = []
        while remaining:
            seed = remaining.pop()
            component = {seed}
            stack = [seed]
            while stack:
                x, y = stack.pop()
                for nx in range(x - 1, x + 2):
                    for ny in range(y - 1, y + 2):
                        key = (nx, ny)
                        if key in remaining:
                            remaining.remove(key)
                            component.add(key)
                            stack.append(key)
            components.append(component)
        component_weights = [sum(_cell_weight(highway[key], material_mode) for key in comp) for comp in components]
        highway_material = sum(component_weights)
        target_connected = 0.0
        for comp, weight in zip(components, component_weights):
            if any(float(getattr(highway[key], "distance_to_target", float("inf"))) <= 1.5 for key in comp):
                target_connected += weight
        vectors = []
        for cell in highway.values():
            vector = getattr(cell, "velocity_highway", (0.0, 0.0))
            norm = math.hypot(float(vector[0]), float(vector[1]))
            if norm > 1e-9:
                vectors.append((float(vector[0]) / norm, float(vector[1]) / norm, _cell_weight(cell, material_mode)))
        vector_weight = sum(item[2] for item in vectors)
        coherence = 0.0
        if vector_weight > 0.0:
            vx = sum(x * weight for x, _, weight in vectors) / vector_weight
            vy = sum(y * weight for _, y, weight in vectors) / vector_weight
            coherence = math.hypot(vx, vy)
        return {
            "threshold": threshold,
            "occupied_highway_cells": len(highway),
            "highway_material": highway_material,
            "highway_material_fraction": highway_material / outside_total if outside_total else 1.0,
            "component_count": len(components),
            "largest_component_material_share": max(component_weights, default=0.0) / highway_material if highway_material else 0.0,
            "target_connected_material_share": target_connected / highway_material if highway_material else 0.0,
            "directional_coherence": coherence,
            "component_cells": [sorted(comp) for comp in sorted(components, key=len, reverse=True)],
        }

    distances = [float(getattr(cell, "distance_to_target", 0.0)) for cell in occupied]
    weights = [_cell_weight(cell, material_mode) for cell in occupied]
    weighted_distance = sum(d * w for d, w in zip(distances, weights)) / outside_total if outside_total else 0.0
    weighted_heat = sum(
        _cell_weight(cell, material_mode) * max(0.0, float(getattr(cell, "heat_map", 0.0)))
        for cell in occupied
    )
    direct_feasible = sum(
        _cell_weight(cell, material_mode) for cell in occupied
        if bool(getattr(cell, "best_path_target", None))
    )
    coords = [(float(cell.x) + 0.5, float(cell.y) + 0.5) for cell in occupied]
    concentration = 1.0
    if outside_total and coords:
        cx = sum(x * w for (x, _), w in zip(coords, weights)) / outside_total
        cy = sum(y * w for (_, y), w in zip(coords, weights)) / outside_total
        rms = math.sqrt(sum(w * ((x - cx) ** 2 + (y - cy) ** 2) for (x, y), w in zip(coords, weights)) / outside_total)
        concentration = math.exp(-3.0 * rms / max(1.0, float(getattr(env, "grid_size", 1))))
    return {
        "material_value_mode": material_mode,
        "total_material": total,
        "target_material": target_total,
        "outside_material": outside_total,
        "delivery_fraction": target_total / total if total else 1.0,
        "potential_weighted_score": weighted_heat / (outside_total * max_heat) if outside_total and max_heat else 0.0,
        "mean_distance_to_target_cells": weighted_distance,
        "median_distance_to_target_cells": _weighted_quantile(distances, weights, 0.50),
        "p90_distance_to_target_cells": _weighted_quantile(distances, weights, 0.90),
        "spatial_concentration_score": concentration,
        "direct_target_feasible_fraction": direct_feasible / outside_total if outside_total else 1.0,
        "max_heat": max_heat,
        "planner_metrics": evaluate_threshold(planner_threshold),
        "reference_metrics": {
            "definition": "fixed_normalized_heat_threshold",
            "normalized_highway_threshold_ratio": reference_highway_ratio,
            **evaluate_threshold(reference_threshold),
        },
    }


class BenchmarkTelemetry:
    PUSH_FIELDS = (
        "run_id", "task_id", "agent_id", "rover_type", "task_type", "map_epoch",
        "push_start_sim_time", "push_end_sim_time", "observation_sim_time", "observation_delay_s",
        "predicted_collected", "predicted_delivered", "predicted_spillage",
        "corridor_candidate_count", "contacted_count", "contacted_mass",
        "entered_target_count", "entered_target_mass", "retained_target_count", "retained_target_mass",
        "left_target_count", "left_target_mass", "lateral_spill_count", "lateral_spill_mass",
        "forward_undelivered_count", "forward_undelivered_mass", "mean_target_progress_m",
        "delivery_efficiency_count", "delivery_efficiency_mass", "lateral_spillage_rate_count",
        "ambiguous_contact_count", "exclusive_contact_count", "transport_gain_m_mass",
        "corridor_candidate_ids_json", "contacted_ids_json", "ambiguous_ids_json", "pebble_results_json",
    )
    SNAPSHOT_FIELDS = (
        "run_id", "snapshot_id", "reason", "sim_time", "map_epoch", "delivered_count",
        "remaining_count", "delivered_material_mass", "remaining_material_mass",
        "delivery_fraction", "target_material", "outside_material", "potential_weighted_score",
        "mean_distance_to_target_cells", "median_distance_to_target_cells", "p90_distance_to_target_cells",
        "spatial_concentration_score", "direct_target_feasible_fraction",
        "planner_highway_material_fraction", "planner_component_count", "planner_largest_component_share",
        "planner_target_connected_share", "planner_directional_coherence",
        "reference_highway_material_fraction", "reference_component_count", "reference_largest_component_share",
        "reference_target_connected_share", "reference_directional_coherence",
        "data_path", "image_path",
    )

    def __init__(
        self,
        log_dir: str | Path,
        run_id: str,
        event_callback: Optional[Callable[..., None]] = None,
        capture_images: bool = True,
        capture_push_images: bool = True,
        save_snapshot_data: bool = True,
        observation_delay_s: float = 1.0,
        corridor_margin_m: float = 0.08,
        reference_highway_ratio: float = 0.50,
        milestones: Sequence[float] = (0.25, 0.50, 0.75, 0.90, 1.0),
        image_dpi: int = 150,
    ):
        self.log_dir = Path(log_dir).resolve()
        self.run_id = str(run_id)
        self.event_callback = event_callback
        self.capture_images = bool(capture_images)
        self.capture_push_images = bool(capture_push_images)
        self.save_snapshot_data = bool(save_snapshot_data)
        self.observation_delay_s = max(0.0, float(observation_delay_s))
        self.corridor_margin_m = max(0.0, float(corridor_margin_m))
        self.reference_highway_ratio = max(0.0, min(1.0, float(reference_highway_ratio)))
        self.milestones = tuple(sorted(set(max(0.0, min(1.0, float(x))) for x in milestones)))
        self.image_dpi = max(72, int(image_dpi))
        # Notebook experiment folders can already be ~200 characters long.
        # Keep generated names compact enough for Windows installations that
        # still enforce MAX_PATH (260), while the run_id remains in every row.
        compact_run_id = re.sub(r"[^A-Za-z0-9]", "", self.run_id)[-12:]
        self.file_tag = compact_run_id or "run"
        self.artifact_dir = self.log_dir / f"b_{self.file_tag}"
        self.data_dir = self.artifact_dir / "data"
        self.image_dir = self.artifact_dir / "img"
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self.pushes_path = self.log_dir / f"b_{self.file_tag}_push.csv"
        self.snapshots_path = self.log_dir / f"b_{self.file_tag}_snap.csv"
        self.run_summary_path = self.log_dir / f"b_{self.file_tag}_run.json"
        self._push_file = self.pushes_path.open("w", encoding="utf-8", newline="", buffering=1)
        self._snapshot_file = self.snapshots_path.open("w", encoding="utf-8", newline="", buffering=1)
        self._push_writer = csv.DictWriter(self._push_file, fieldnames=self.PUSH_FIELDS)
        self._snapshot_writer = csv.DictWriter(self._snapshot_file, fieldnames=self.SNAPSHOT_FIELDS)
        self._push_writer.writeheader()
        self._snapshot_writer.writeheader()
        self.active_pushes: Dict[str, Dict[str, Any]] = {}
        self.pending_observations: list[Dict[str, Any]] = []
        self.push_rows: list[Dict[str, Any]] = []
        self.snapshot_rows: list[Dict[str, Any]] = []
        self.reached_milestones: set[float] = set()
        self.snapshot_counter = 0
        self._closed = False

    def config_dict(self) -> Dict[str, Any]:
        return {
            "capture_images": self.capture_images,
            "capture_push_images": self.capture_push_images,
            "save_snapshot_data": self.save_snapshot_data,
            "post_push_observation_delay_s": self.observation_delay_s,
            "corridor_margin_m": self.corridor_margin_m,
            "reference_highway_threshold_ratio": self.reference_highway_ratio,
            "delivery_milestones": self.milestones,
            "pushes_csv": str(self.pushes_path),
            "snapshots_csv": str(self.snapshots_path),
            "run_summary_json": str(self.run_summary_path),
            "artifact_dir": str(self.artifact_dir),
        }

    def start_push(self, agent: Mapping[str, Any], sim_time: float, path: Sequence[Sequence[float]], pebble_states: Mapping[int, Mapping[str, Any]]) -> None:
        agent_id = str(agent.get("id"))
        selection = agent.get("selection") or {}
        path_info = selection.get("path_info") or {}
        width = float(selection.get("shovel_width") or getattr(agent.get("rover_profile"), "shovel_width", 0.22))
        radius = 0.5 * width + self.corridor_margin_m
        candidate_ids = {
            int(pid) for pid, state in pebble_states.items()
            if point_to_polyline_distance(state["position"], path) <= radius
        }
        record = {
            "agent_id": agent_id,
            "task_id": agent.get("event_task_id"),
            "rover_type": getattr(agent.get("rover_profile"), "name", "legacy"),
            "task_type": selection.get("path_type"),
            "map_epoch": path_info.get("map_epoch"),
            "start_sim_time": float(sim_time),
            "path": [[float(p[0]), float(p[1])] for p in path],
            "corridor_radius_m": radius,
            "corridor_candidate_ids": candidate_ids,
            "contacted_ids": set(),
            "shovel_contact_ids": set(),
            "ambiguous_ids": set(),
            "start_states": {int(k): dict(v) for k, v in pebble_states.items()},
            "predicted_collected": path_info.get("expected_collected"),
            "predicted_delivered": path_info.get("expected_delivered", path_info.get("total_objects_target")),
            "predicted_spillage": path_info.get("expected_spillage"),
        }
        self.active_pushes[agent_id] = record
        self._emit("PUSH_PHYSICS_TRACKING_STARTED", sim_time, agent_id=agent_id,
                   task_id=record["task_id"], corridor_radius_m=radius,
                   corridor_candidate_ids=sorted(candidate_ids),
                   predicted_collected=record["predicted_collected"],
                   predicted_delivered=record["predicted_delivered"],
                   predicted_spillage=record["predicted_spillage"])

    def record_contacts(self, agent_id: str, pebble_ids: Iterable[int], shovel_ids: Iterable[int] = ()) -> None:
        record = self.active_pushes.get(str(agent_id))
        if record is None:
            return
        for pebble_id in pebble_ids:
            pid = int(pebble_id)
            record["contacted_ids"].add(pid)
            # Ambiguity is concurrent, not historical: a later rover touching
            # the same pebble in another task must not invalidate this push.
            for other_id, other in self.active_pushes.items():
                if other_id != str(agent_id) and pid in other["contacted_ids"]:
                    record["ambiguous_ids"].add(pid)
                    other["ambiguous_ids"].add(pid)
        record["shovel_contact_ids"].update(int(pid) for pid in shovel_ids)

    def end_push(self, agent_id: str, sim_time: float, pebble_states: Mapping[int, Mapping[str, Any]]) -> None:
        record = self.active_pushes.pop(str(agent_id), None)
        if record is None:
            return
        record["push_end_sim_time"] = float(sim_time)
        record["immediate_states"] = {int(k): dict(v) for k, v in pebble_states.items()}
        record["observe_at"] = float(sim_time) + self.observation_delay_s
        self.pending_observations.append(record)
        self._emit("PUSH_PHYSICS_IMMEDIATE", sim_time, agent_id=str(agent_id),
                   task_id=record["task_id"], contacted_ids=sorted(record["contacted_ids"]),
                   shovel_contact_ids=sorted(record["shovel_contact_ids"]),
                   observation_due_sim_time=record["observe_at"])

    def observe_due(self, sim_time: float, pebble_states: Mapping[int, Mapping[str, Any]]) -> list[Dict[str, Any]]:
        completed = []
        waiting = []
        for record in self.pending_observations:
            if float(sim_time) + 1e-9 < float(record["observe_at"]):
                waiting.append(record)
                continue
            row = self._finish_push_record(record, sim_time, pebble_states)
            completed.append(row)
        self.pending_observations = waiting
        return completed

    def _finish_push_record(self, record: Mapping[str, Any], sim_time: float, pebble_states: Mapping[int, Mapping[str, Any]]) -> Dict[str, Any]:
        contacted = set(record["contacted_ids"])
        ambiguous = set(record.get("ambiguous_ids", ())).intersection(contacted)
        results = []
        totals = {key: 0.0 for key in (
            "contacted_count", "contacted_mass", "entered_target_count", "entered_target_mass",
            "retained_target_count", "retained_target_mass", "left_target_count", "left_target_mass",
            "lateral_spill_count", "lateral_spill_mass", "forward_undelivered_count",
            "forward_undelivered_mass", "transport_gain_m_mass",
        )}
        progress_values = []
        for pid in sorted(contacted):
            start = record["start_states"].get(pid)
            end = pebble_states.get(pid) or record.get("immediate_states", {}).get(pid)
            if start is None or end is None:
                continue
            mass = float(start.get("material_mass", 1.0))
            entered = not bool(start["in_target"]) and bool(end["in_target"])
            retained = bool(end["in_target"])
            left = bool(start["in_target"]) and not bool(end["in_target"])
            target_progress = float(start["signed_target_distance_m"]) - float(end["signed_target_distance_m"])
            lateral_distance = point_to_polyline_distance(end["position"], record["path"])
            lateral = (not bool(end["in_target"]) and lateral_distance > float(record["corridor_radius_m"]))
            forward_undelivered = (not bool(end["in_target"]) and target_progress > 0.0 and not lateral)
            totals["contacted_count"] += 1.0
            totals["contacted_mass"] += mass
            totals["entered_target_count"] += float(entered)
            totals["entered_target_mass"] += mass if entered else 0.0
            totals["retained_target_count"] += float(retained)
            totals["retained_target_mass"] += mass if retained else 0.0
            totals["left_target_count"] += float(left)
            totals["left_target_mass"] += mass if left else 0.0
            totals["lateral_spill_count"] += float(lateral)
            totals["lateral_spill_mass"] += mass if lateral else 0.0
            totals["forward_undelivered_count"] += float(forward_undelivered)
            totals["forward_undelivered_mass"] += mass if forward_undelivered else 0.0
            totals["transport_gain_m_mass"] += target_progress * mass
            progress_values.append(target_progress)
            results.append({
                "pebble_id": pid, "profile": start.get("profile"), "material_mass": mass,
                "ambiguous_contact": pid in ambiguous, "start": start, "observed": end,
                "entered_target": entered, "retained_in_target": retained, "left_target": left,
                "target_progress_m": target_progress, "lateral_distance_m": lateral_distance,
                "lateral_spill": lateral, "forward_undelivered": forward_undelivered,
            })
        count = totals["contacted_count"]
        mass = totals["contacted_mass"]
        row = {
            "run_id": self.run_id, "task_id": record.get("task_id"), "agent_id": record["agent_id"],
            "rover_type": record.get("rover_type"), "task_type": record.get("task_type"),
            "map_epoch": record.get("map_epoch"), "push_start_sim_time": record["start_sim_time"],
            "push_end_sim_time": record["push_end_sim_time"], "observation_sim_time": float(sim_time),
            "observation_delay_s": float(sim_time) - float(record["push_end_sim_time"]),
            "predicted_collected": record.get("predicted_collected"),
            "predicted_delivered": record.get("predicted_delivered"),
            "predicted_spillage": record.get("predicted_spillage"),
            "corridor_candidate_count": len(record["corridor_candidate_ids"]),
            **totals,
            "mean_target_progress_m": sum(progress_values) / len(progress_values) if progress_values else 0.0,
            "delivery_efficiency_count": totals["entered_target_count"] / count if count else 0.0,
            "delivery_efficiency_mass": totals["entered_target_mass"] / mass if mass else 0.0,
            "lateral_spillage_rate_count": totals["lateral_spill_count"] / count if count else 0.0,
            "ambiguous_contact_count": len(ambiguous), "exclusive_contact_count": len(contacted - ambiguous),
            "corridor_candidate_ids_json": _json(sorted(record["corridor_candidate_ids"])),
            "contacted_ids_json": _json(sorted(contacted)), "ambiguous_ids_json": _json(sorted(ambiguous)),
            "pebble_results_json": _json(results),
        }
        self._push_writer.writerow({field: row.get(field) for field in self.PUSH_FIELDS})
        self._push_file.flush()
        self.push_rows.append(row)
        self._emit("PUSH_PHYSICS_RESULT", sim_time, agent_id=record["agent_id"],
                   **{key: value for key, value in row.items() if key not in {"run_id", "agent_id", "pebble_results_json"}},
                   pebble_results=results)
        return row

    def milestone_reasons(self, delivered_fraction: float) -> list[str]:
        reasons = []
        for milestone in self.milestones:
            if delivered_fraction + 1e-12 >= milestone and milestone not in self.reached_milestones:
                self.reached_milestones.add(milestone)
                reasons.append(f"delivery_{int(round(100 * milestone)):03d}pct")
        return reasons

    def record_snapshot(
        self,
        reason: str,
        sim_time: float,
        env: Any,
        map_epoch: Optional[int],
        material_progress: Mapping[str, Any],
        pebble_states: Mapping[int, Mapping[str, Any]],
        rover_states: Sequence[Mapping[str, Any]],
        active_paths: Sequence[Mapping[str, Any]],
        planning_config: Mapping[str, Any],
        env_radius: float,
        target_polygon: Sequence[Sequence[float]],
        force_image: bool = False,
    ) -> Dict[str, Any]:
        self.snapshot_counter += 1
        snapshot_id = f"S{self.snapshot_counter:05d}"
        mode = material_progress.get("material_value_mode", "count")
        metrics = calculate_environment_metrics(env, mode, self.reference_highway_ratio)
        physical_total = (
            float(material_progress.get("delivered_material_mass", 0.0))
            + float(material_progress.get("remaining_material_mass", 0.0))
            if mode == "mass" else
            float(material_progress.get("delivered_count", 0.0))
            + float(material_progress.get("remaining_count", 0.0))
        )
        physical_delivered = (
            float(material_progress.get("delivered_material_mass", 0.0))
            if mode == "mass" else float(material_progress.get("delivered_count", 0.0))
        )
        physical_remaining = max(0.0, physical_total - physical_delivered)
        metrics["total_material"] = physical_total
        metrics["target_material"] = physical_delivered
        metrics["outside_material"] = physical_remaining
        metrics["delivery_fraction"] = physical_delivered / physical_total if physical_total else 1.0
        payload = {
            "schema": 2, "run_id": self.run_id, "snapshot_id": snapshot_id, "reason": str(reason),
            "sim_time": float(sim_time), "map_epoch": map_epoch, "material_progress": dict(material_progress),
            "environment_metrics": metrics, "planning_config": dict(planning_config),
            "pebbles": list(pebble_states.values()), "rovers": list(rover_states), "active_paths": list(active_paths),
            "target_polygon": [[float(x), float(y)] for x, y in target_polygon], "env_radius": float(env_radius),
            "cells": self._serialize_cells(env),
        }
        # The complete reason and time are stored in the files and snapshot CSV;
        # filenames are deliberately compact for Windows path compatibility.
        stem = f"{self.snapshot_counter:05d}"
        data_path = self.data_dir / f"{stem}.json.gz"
        image_path = self.image_dir / f"{stem}.png"
        if self.save_snapshot_data:
            with gzip.open(data_path, "wt", encoding="utf-8") as handle:
                json.dump(_safe(payload), handle, ensure_ascii=False, separators=(",", ":"))
        write_image = self.capture_images and (force_image or self.capture_push_images or not str(reason).startswith(("before_push", "after_push")))
        if write_image:
            try:
                self._render_snapshot(payload, image_path)
            except Exception as exc:
                image_path = None
                self._emit("BENCHMARK_IMAGE_FAILED", sim_time, error=repr(exc), snapshot_id=snapshot_id)
        else:
            image_path = None
        planner = metrics["planner_metrics"]
        reference = metrics["reference_metrics"]
        row = {
            "run_id": self.run_id, "snapshot_id": snapshot_id, "reason": reason, "sim_time": float(sim_time),
            "map_epoch": map_epoch, **{key: material_progress.get(key) for key in (
                "delivered_count", "remaining_count", "delivered_material_mass", "remaining_material_mass")},
            **{key: metrics.get(key) for key in (
                "delivery_fraction", "target_material", "outside_material", "potential_weighted_score",
                "mean_distance_to_target_cells", "median_distance_to_target_cells", "p90_distance_to_target_cells",
                "spatial_concentration_score", "direct_target_feasible_fraction")},
            "planner_highway_material_fraction": planner["highway_material_fraction"],
            "planner_component_count": planner["component_count"],
            "planner_largest_component_share": planner["largest_component_material_share"],
            "planner_target_connected_share": planner["target_connected_material_share"],
            "planner_directional_coherence": planner["directional_coherence"],
            "reference_highway_material_fraction": reference["highway_material_fraction"],
            "reference_component_count": reference["component_count"],
            "reference_largest_component_share": reference["largest_component_material_share"],
            "reference_target_connected_share": reference["target_connected_material_share"],
            "reference_directional_coherence": reference["directional_coherence"],
            "data_path": str(data_path) if self.save_snapshot_data else None,
            "image_path": str(image_path) if image_path else None,
        }
        self._snapshot_writer.writerow({field: row.get(field) for field in self.SNAPSHOT_FIELDS})
        self._snapshot_file.flush()
        self.snapshot_rows.append(row)
        self._emit("ENVIRONMENT_SNAPSHOT", sim_time, snapshot_id=snapshot_id, reason=reason,
                   map_epoch=map_epoch, metrics=metrics, data_path=row["data_path"], image_path=row["image_path"])
        return row

    @staticmethod
    def _serialize_cells(env: Any) -> list[Dict[str, Any]]:
        rows = []
        for cell in getattr(env, "all_cells", []):
            heat = float(getattr(cell, "heat_map", 0.0))
            count = float(getattr(cell, "physical_object_count", getattr(cell, "num_objects", 0.0)))
            mass = float(getattr(cell, "material_mass", getattr(cell, "num_objects", 0.0)))
            if heat == 0.0 and count == 0.0 and mass == 0.0 and not getattr(cell, "is_target_zone", False):
                continue
            rows.append({
                "x": int(cell.x), "y": int(cell.y), "heat": heat, "count": count, "mass": mass,
                "in_target": bool(getattr(cell, "is_target_zone", False)),
                "distance_to_target": float(getattr(cell, "distance_to_target", 0.0)),
                "velocity_target": getattr(cell, "velocity_target", (0.0, 0.0)),
                "velocity_highway": getattr(cell, "velocity_highway", (0.0, 0.0)),
            })
        return rows

    def _render_snapshot(self, payload: Mapping[str, Any], image_path: Path) -> None:
        import numpy as np
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure
        from matplotlib.patches import Polygon as PolygonPatch

        figure = Figure(figsize=(9, 8), dpi=self.image_dpi)
        FigureCanvasAgg(figure)
        axis = figure.add_subplot(111)
        radius = float(payload["env_radius"])
        cells = payload["cells"]
        if cells:
            max_heat = max((float(cell["heat"]) for cell in cells), default=1.0) or 1.0
            grid_size = max(max(int(cell["x"]), int(cell["y"])) for cell in cells) + 1
            heat = np.zeros((grid_size, grid_size), dtype=float)
            for cell in cells:
                heat[int(cell["y"]), int(cell["x"])] = float(cell["heat"]) / max_heat
            axis.imshow(heat, origin="lower", extent=(-radius, radius, -radius, radius),
                        cmap="YlOrRd", vmin=0.0, vmax=1.0, alpha=0.40, interpolation="nearest")
            threshold = float(payload["environment_metrics"]["planner_metrics"]["threshold"])
            highway = [cell for cell in cells if float(cell["heat"]) >= threshold and (cell["count"] > 0 or cell["mass"] > 0)]
            if highway:
                scale = 2.0 * radius / grid_size
                hx = [-radius + (float(cell["x"]) + 0.5) * scale for cell in highway]
                hy = [-radius + (float(cell["y"]) + 0.5) * scale for cell in highway]
                axis.scatter(hx, hy, marker="s", s=55, facecolors="none", edgecolors="#8c2d04", linewidths=1.2, label="highway cells")
        polygon = payload["target_polygon"]
        if polygon:
            axis.add_patch(PolygonPatch(polygon, closed=True, facecolor="#2ca25f", alpha=0.20,
                                        edgecolor="#006d2c", linewidth=2.0, label="target zone"))
        profile_colors = {"small": "#3182bd", "medium": "#756bb1", "large": "#de2d26"}
        for profile in sorted({str(item.get("profile", "unknown")) for item in payload["pebbles"]}):
            items = [item for item in payload["pebbles"] if str(item.get("profile", "unknown")) == profile]
            axis.scatter([item["position"][0] for item in items], [item["position"][1] for item in items],
                         s=[18 + 12 * float(item.get("visual_scale", 1.0)) for item in items],
                         c=profile_colors.get(profile, "#636363"), edgecolors="white", linewidths=0.4,
                         label=f"{profile} pebbles", zorder=5)
        for rover in payload["rovers"]:
            x, y, yaw = rover["pose"][:3]
            color = np.array(rover.get("color", (0.1, 0.1, 0.1)), dtype=float) / (255.0 if max(rover.get("color", (1,))) > 1 else 1.0)
            axis.scatter([x], [y], marker="^", s=100, c=[color[:3]], edgecolors="black", zorder=8)
            axis.arrow(x, y, 0.20 * math.cos(yaw), 0.20 * math.sin(yaw), color=color[:3], width=0.012, zorder=8)
            axis.text(x + 0.06, y + 0.06, str(rover["agent_id"]), fontsize=8, zorder=9)
        styles = {"target": "-", "highway": "--"}
        for path in payload["active_paths"]:
            points = path.get("points") or []
            if len(points) < 2:
                continue
            color = np.array(path.get("color", (0, 90, 220)), dtype=float) / (255.0 if max(path.get("color", (1,))) > 1 else 1.0)
            axis.plot([point[0] for point in points], [point[1] for point in points],
                      styles.get(path.get("task_type"), "-"), color=color[:3], linewidth=2.0,
                      label=f"{path.get('agent_id')} {path.get('task_type')}", zorder=7)
        progress = payload["material_progress"]
        axis.set_title(
            f"{payload['reason']} | t={payload['sim_time']:.2f}s | map={payload['map_epoch']} | "
            f"delivered={progress.get('delivered_count', 0)} remaining={progress.get('remaining_count', 0)}"
        )
        axis.set_xlim(-radius, radius)
        axis.set_ylim(-radius, radius)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("world x [m]")
        axis.set_ylabel("world y [m]")
        axis.grid(True, alpha=0.18)
        handles, labels = axis.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        if unique:
            axis.legend(unique.values(), unique.keys(), loc="upper right", fontsize=7, framealpha=0.85)
        figure.tight_layout()
        figure.savefig(image_path)

    def close(self, sim_time: float, outcome: str, final_progress: Mapping[str, Any]) -> Dict[str, Any]:
        if self._closed:
            return {}
        # A manual/exception stop can occur before the configured delay. Preserve
        # those pushes using the latest available physics state instead of losing
        # the row entirely.
        for agent_id, record in list(self.active_pushes.items()):
            record["push_end_sim_time"] = float(sim_time)
            record["immediate_states"] = record.get("start_states", {})
            record["observe_at"] = float(sim_time)
            self.pending_observations.append(record)
        self.active_pushes.clear()
        for record in list(self.pending_observations):
            latest = record.get("immediate_states", record.get("start_states", {}))
            self._finish_push_record(record, sim_time, latest)
        self.pending_observations.clear()
        contacted = sum(float(row.get("contacted_count", 0.0)) for row in self.push_rows)
        contacted_mass = sum(float(row.get("contacted_mass", 0.0)) for row in self.push_rows)
        entered = sum(float(row.get("entered_target_count", 0.0)) for row in self.push_rows)
        entered_mass = sum(float(row.get("entered_target_mass", 0.0)) for row in self.push_rows)
        lateral = sum(float(row.get("lateral_spill_count", 0.0)) for row in self.push_rows)
        summary = {
            "schema": 2, "run_id": self.run_id, "outcome": outcome, "end_sim_time": float(sim_time),
            "push_count": len(self.push_rows), "environment_snapshot_count": len(self.snapshot_rows),
            "contacted_count_sum": contacted, "contacted_mass_sum": contacted_mass,
            "entered_target_count_sum": entered, "entered_target_mass_sum": entered_mass,
            "lateral_spill_count_sum": lateral,
            "aggregate_delivery_efficiency_count": entered / contacted if contacted else 0.0,
            "aggregate_delivery_efficiency_mass": entered_mass / contacted_mass if contacted_mass else 0.0,
            "aggregate_lateral_spillage_rate_count": lateral / contacted if contacted else 0.0,
            "final_material_progress": dict(final_progress), "config": self.config_dict(),
        }
        self._emit("BENCHMARK_RUN_SUMMARY", sim_time, **summary)
        self.run_summary_path.write_text(_json(summary) + "\n", encoding="utf-8")
        self._push_file.flush(); self._snapshot_file.flush()
        self._push_file.close(); self._snapshot_file.close()
        self._closed = True
        return summary

    def _emit(self, event_type: str, sim_time: float, **data: Any) -> None:
        if self.event_callback is not None:
            self.event_callback(event_type, float(sim_time), **data)
