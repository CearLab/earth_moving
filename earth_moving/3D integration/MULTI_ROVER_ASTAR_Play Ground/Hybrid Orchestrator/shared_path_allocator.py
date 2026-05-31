"""
shared_path_allocator.py

Lightweight path allocation on top of one shared 2D environment snapshot.

The expensive 2D heat-map/path computation should happen once per world
snapshot. Each rover then asks this module to allocate one available path from
that shared snapshot while respecting path and object-cell reservations.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Any, Dict, Iterable, Optional, Sequence, Set, Tuple

import numpy as np


CellKey = Tuple[int, int]


@dataclass(frozen=True)
class SharedMapBuildRequest:
    epoch: int
    env_radius: float
    target_zone_radius: float
    shovel_width: float
    pebbles_xy: Tuple[Tuple[float, float], ...]
    use_spillage_model: bool
    visualize_potential: bool


@dataclass
class SharedMapSnapshot:
    epoch: int
    env_radius: float
    env_2d: Any
    coord_converter: Any
    built_wall_time: float
    pebbles_count: int


@dataclass(frozen=True)
class PathAllocationRequest:
    agent_id: str
    plan_id: int
    map_epoch: int
    agent_pose_xy: Tuple[float, float]
    reserved_path_cells: Tuple[CellKey, ...] = ()
    reserved_object_cells: Tuple[CellKey, ...] = ()
    consumed_object_cells: Tuple[CellKey, ...] = ()


@dataclass
class PathAllocationResult:
    agent_id: str
    plan_id: int
    map_epoch: int
    env_radius: float
    env_2d: Any
    coord_converter: Any
    best_cell: Any = None
    best_choice: str = "target"
    best_path_info: Optional[Dict[str, Any]] = None
    reserved_path_cells: Set[CellKey] = field(default_factory=set)
    reserved_object_cells: Set[CellKey] = field(default_factory=set)
    error: Optional[str] = None


def normalize_cell_key(value: Any) -> Optional[CellKey]:
    if value is None:
        return None
    if hasattr(value, "x") and hasattr(value, "y"):
        return int(value.x), int(value.y)
    if isinstance(value, np.ndarray):
        value = value.tolist()
    if isinstance(value, (tuple, list)) and len(value) >= 2:
        return int(value[0]), int(value[1])
    return None


def normalize_cell_keys(values: Iterable[Any]) -> Set[CellKey]:
    out: Set[CellKey] = set()
    for value in values:
        key = normalize_cell_key(value)
        if key is not None:
            out.add(key)
    return out


def path_cells(path_info: Dict[str, Any]) -> Set[CellKey]:
    cells = normalize_cell_keys(path_info.get("path", []))
    impacted = path_info.get("impacted_cells", {}) or {}
    cells.update(normalize_cell_keys(impacted.keys()))
    return cells


def object_cells_for_path(root_cell: Any, path_info: Dict[str, Any]) -> Set[CellKey]:
    cells: Set[CellKey] = set()

    root_key = normalize_cell_key(root_cell)
    if root_key is not None:
        cells.add(root_key)

    for cell in path_info.get("path", []) or []:
        key = normalize_cell_key(cell)
        if key is not None and getattr(cell, "num_objects", 0) > 0:
            cells.add(key)

    impacted = path_info.get("impacted_cells", {}) or {}
    cells.update(normalize_cell_keys(impacted.keys()))
    return cells


def build_shared_map_snapshot(
    request: SharedMapBuildRequest,
    compute_2d_env,
    coordinate_converter_cls,
) -> SharedMapSnapshot:
    pebbles_3d = [(float(x), float(y), 0.01) for x, y in request.pebbles_xy]
    env_2d = compute_2d_env(
        request.env_radius,
        request.target_zone_radius,
        request.shovel_width,
        pebbles_3d,
        manual_mode=False,
        use_spillage_model=request.use_spillage_model,
        visualize_potential=request.visualize_potential,
    )
    coord_converter = coordinate_converter_cls(
        request.env_radius,
        request.target_zone_radius,
        request.shovel_width,
    )
    return SharedMapSnapshot(
        epoch=request.epoch,
        env_radius=request.env_radius,
        env_2d=env_2d,
        coord_converter=coord_converter,
        built_wall_time=time.time(),
        pebbles_count=len(request.pebbles_xy),
    )


def allocate_path_from_shared_map(
    snapshot: SharedMapSnapshot,
    request: PathAllocationRequest,
) -> PathAllocationResult:
    try:
        blocked_path_cells = set(request.reserved_path_cells)
        blocked_object_cells = set(request.reserved_object_cells)
        blocked_object_cells.update(request.consumed_object_cells)

        best_cell = None
        best_choice = "target"
        best_path_info = None
        best_path_cells: Set[CellKey] = set()
        best_object_cells: Set[CellKey] = set()
        best_score = -float("inf")

        def consider(cell: Any, choice: str, path_info: Dict[str, Any], score_objects: float) -> None:
            nonlocal best_cell, best_choice, best_path_info
            nonlocal best_path_cells, best_object_cells, best_score

            traj = path_info.get("path", []) or []
            if not traj:
                return

            required_path_cells = path_cells(path_info)
            required_object_cells = object_cells_for_path(cell, path_info)

            if required_path_cells.intersection(blocked_path_cells):
                return
            if required_object_cells.intersection(blocked_object_cells):
                return

            start_c = traj[0]
            start_wx, start_wy = snapshot.coord_converter.convert_2d_to_3d(start_c.x, start_c.y)
            dist_to_start = math.hypot(
                request.agent_pose_xy[0] - start_wx,
                request.agent_pose_xy[1] - start_wy,
            )
            score = float(score_objects) - 0.5 * dist_to_start

            if score > best_score:
                best_score = score
                best_cell = cell
                best_choice = choice
                best_path_info = path_info
                best_path_cells = required_path_cells
                best_object_cells = required_object_cells

        # Stage 1: target paths with positive delivered objects.
        for cell in getattr(snapshot.env_2d, "cells_with_objects", []):
            paths = snapshot.env_2d.get_path_for_preview(cell, "target")
            if not paths:
                continue
            path_info = paths[0]
            delivered_objects = path_info.get("objects", 0)
            if delivered_objects > 0:
                consider(cell, "target", path_info, delivered_objects)

        # Stage 2: highway fallback.
        if best_cell is None:
            for cell in getattr(snapshot.env_2d, "cells_with_objects", []):
                paths = snapshot.env_2d.get_path_for_preview(cell, "highway")
                if not paths:
                    continue
                path_info = paths[0]
                highway_objects = path_info.get("objects", 0)
                if highway_objects > 0:
                    consider(cell, "highway", path_info, highway_objects)

        # Stage 3: raw target fallback when spillage predicts zero delivery.
        if best_cell is None:
            for cell in getattr(snapshot.env_2d, "cells_with_objects", []):
                paths = snapshot.env_2d.get_path_for_preview(cell, "target")
                if not paths:
                    continue
                path_info = paths[0]
                raw_objects = path_info.get("raw_objects", getattr(cell, "num_objects", 0))
                if raw_objects > 0:
                    consider(cell, "target", path_info, raw_objects)

        return PathAllocationResult(
            agent_id=request.agent_id,
            plan_id=request.plan_id,
            map_epoch=request.map_epoch,
            env_radius=snapshot.env_radius,
            env_2d=snapshot.env_2d,
            coord_converter=snapshot.coord_converter,
            best_cell=best_cell,
            best_choice=best_choice,
            best_path_info=best_path_info,
            reserved_path_cells=best_path_cells,
            reserved_object_cells=best_object_cells,
            error=None,
        )
    except Exception as exc:
        return PathAllocationResult(
            agent_id=request.agent_id,
            plan_id=request.plan_id,
            map_epoch=request.map_epoch,
            env_radius=snapshot.env_radius,
            env_2d=snapshot.env_2d,
            coord_converter=snapshot.coord_converter,
            error=str(exc),
        )
