import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data
from tqdm.auto import tqdm

import grid_utils


@dataclass
class ArenaConfig:
    x_min: float = 0.35
    x_max: float = 1.05
    y_min: float = -0.35
    y_max: float = 0.35
    z_spawn: float = 0.12
    cube_size: float = 0.02
    spacing: float = 0.022
    density: float = 0.06
    jitter_xy: float = 0.003


@dataclass
class CollectionConfig:
    n_arenas: int = 3
    paths_per_arena: int = 4
    steps_per_path: int = 12
    move_steps: int = 160
    settle_steps: int = 120
    start_pose: tuple = (0.60, 0.00, 0.00)
    x_bounds: tuple = (0.45, 0.95)
    y_bounds: tuple = (-0.25, 0.25)
    max_step_xy: float = 0.08


def connect(gui=False):
    cid = p.connect(p.GUI if gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0.0, 0.0, -9.81)
    p.loadURDF("plane.urdf")
    return cid


def draw_arena_bounds(arena_cfg, collect_cfg=None):
    """Draw debug lines showing the arena bounds (yellow) and optional operating bounds (cyan)."""
    z = 0.025  # just above blade height
    def _rect(x0, x1, y0, y1, color, width=3):
        corners = [[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]]
        for i in range(4):
            p.addUserDebugLine(corners[i], corners[(i + 1) % 4],
                               lineColorRGB=color, lineWidth=width, lifeTime=0)
    # Arena extent — yellow
    _rect(arena_cfg.x_min, arena_cfg.x_max, arena_cfg.y_min, arena_cfg.y_max,
          [1.0, 1.0, 0.0])
    # Operating / path bounds — cyan (if provided)
    if collect_cfg is not None:
        _rect(collect_cfg.x_bounds[0], collect_cfg.x_bounds[1],
              collect_cfg.y_bounds[0], collect_cfg.y_bounds[1],
              [0.0, 1.0, 1.0], width=2)


def _uniform_scatter_mask(nx, ny, density, rng):
    density = float(np.clip(density, 0.0, 1.0))
    total_cells = nx * ny
    num_filled = int(round(density * total_cells))

    mask = np.zeros(total_cells, dtype=bool)
    if num_filled <= 0:
        return mask.reshape(ny, nx)
    if num_filled >= total_cells:
        mask[:] = True
        return mask.reshape(ny, nx)

    chosen = rng.choice(total_cells, size=num_filled, replace=False)
    mask[chosen] = True
    return mask.reshape(ny, nx)


def spawn_scattered_cubes(cube_urdf, arena_cfg, seed):
    rng = np.random.default_rng(seed)
    xs = np.arange(arena_cfg.x_min, arena_cfg.x_max, arena_cfg.spacing)
    ys = np.arange(arena_cfg.y_min, arena_cfg.y_max, arena_cfg.spacing)
    nx, ny = len(xs), len(ys)
    mask = _uniform_scatter_mask(nx, ny, arena_cfg.density, rng)

    cube_ids = []
    for iy, y in enumerate(ys):
        for ix, x in enumerate(xs):
            if not mask[iy, ix]:
                continue
            jx = rng.uniform(-arena_cfg.jitter_xy, arena_cfg.jitter_xy)
            jy = rng.uniform(-arena_cfg.jitter_xy, arena_cfg.jitter_xy)
            bid = p.loadURDF(
                cube_urdf,
                [float(x + jx), float(y + jy), arena_cfg.z_spawn],
                p.getQuaternionFromEuler([0.0, 0.0, 0.0]),
            )
            cube_ids.append(bid)

    for _ in range(240):
        p.stepSimulation()
    return cube_ids


def spawn_clustered_cubes(cube_urdf, arena_cfg, seed):
    return spawn_scattered_cubes(cube_urdf, arena_cfg, seed)

def _cube_positions(cube_ids):
    pts = []
    for bid in cube_ids:
        pos, _ = p.getBasePositionAndOrientation(bid)
        pts.append([pos[0], pos[1]])
    return np.array(pts, dtype=np.float32) if pts else np.zeros((0, 2), dtype=np.float32)


def _anchor_near_cubes(cube_positions, x_bounds, y_bounds, rng):
    """Return the XY of a random cube clipped to operating bounds, plus the raw anchor."""
    anchor = cube_positions[rng.integers(len(cube_positions))]
    x = float(np.clip(anchor[0], x_bounds[0], x_bounds[1]))
    y = float(np.clip(anchor[1], y_bounds[0], y_bounds[1]))
    return x, y, anchor  # anchor is the raw cube XY used as tether centre


def _tethered_path(start_pose, anchor_xy, tether_radius, steps, x_bounds, y_bounds, max_step_xy, rng):
    """Random walk that stays within tether_radius of anchor_xy."""
    x, y, yaw = start_pose
    path = []
    for _ in range(steps):
        nx = x + rng.uniform(-max_step_xy, max_step_xy)
        ny = y + rng.uniform(-max_step_xy, max_step_xy)
        dist = float(np.hypot(nx - anchor_xy[0], ny - anchor_xy[1]))
        if dist > tether_radius:
            scale = tether_radius / dist
            nx = anchor_xy[0] + (nx - anchor_xy[0]) * scale
            ny = anchor_xy[1] + (ny - anchor_xy[1]) * scale
        x = float(np.clip(nx, x_bounds[0], x_bounds[1]))
        y = float(np.clip(ny, y_bounds[0], y_bounds[1]))
        path.append((x, y, yaw))
    return path


def _current_pose(controller):
    pos, yaw = controller.forward_kinematics_2d()
    return float(pos[0]), float(pos[1]), float(yaw)


def _current_pose_full(controller):
    ee_pos, yaw = controller.forward_kinematics_2d()
    return float(ee_pos[0]), float(ee_pos[1]), float(ee_pos[2]), float(yaw)


def _gravel_summary(cube_ids, ref_xy=None, initial_com=None):
    pts = []
    for bid in cube_ids:
        pos, _ = p.getBasePositionAndOrientation(bid)
        pts.append([float(pos[0]), float(pos[1]), float(pos[2])])

    if pts:
        arr = np.asarray(pts, dtype=np.float32)
        com = arr.mean(axis=0)
        bbox_min = arr.min(axis=0)
        bbox_max = arr.max(axis=0)
        spread_xy_rms = float(np.sqrt(np.mean(np.sum((arr[:, :2] - com[:2]) ** 2, axis=1))))
        nearest_xy = float(np.min(np.linalg.norm(arr[:, :2] - np.asarray(ref_xy, dtype=np.float32), axis=1))) if ref_xy is not None else float("nan")
    else:
        com = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        bbox_min = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        bbox_max = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        spread_xy_rms = float("nan")
        nearest_xy = float("nan")

    summary = {
        "gravel_com_x": float(com[0]),
        "gravel_com_y": float(com[1]),
        "gravel_com_z": float(com[2]),
        "gravel_bbox_xmin": float(bbox_min[0]),
        "gravel_bbox_xmax": float(bbox_max[0]),
        "gravel_bbox_ymin": float(bbox_min[1]),
        "gravel_bbox_ymax": float(bbox_max[1]),
        "gravel_bbox_zmin": float(bbox_min[2]),
        "gravel_bbox_zmax": float(bbox_max[2]),
        "gravel_spread_xy_rms": spread_xy_rms,
        "shovel_to_gravel_com_xy": float("nan"),
        "shovel_to_nearest_gravel_xy": nearest_xy,
    }

    if ref_xy is not None and np.isfinite(summary["gravel_com_x"]):
        ref_xy = np.asarray(ref_xy, dtype=np.float32)
        summary["shovel_to_gravel_com_xy"] = float(np.linalg.norm(ref_xy - com[:2]))

    if initial_com is not None and np.isfinite(summary["gravel_com_x"]):
        initial_com = np.asarray(initial_com, dtype=np.float32)
        summary["gravel_com_shift_x"] = float(com[0] - initial_com[0])
        summary["gravel_com_shift_y"] = float(com[1] - initial_com[1])
        summary["gravel_com_shift_z"] = float(com[2] - initial_com[2])
        summary["gravel_com_shift_xy"] = float(np.linalg.norm(com[:2] - initial_com[:2]))
    else:
        summary["gravel_com_shift_x"] = float("nan")
        summary["gravel_com_shift_y"] = float("nan")
        summary["gravel_com_shift_z"] = float("nan")
        summary["gravel_com_shift_xy"] = float("nan")

    return summary


TRACE_FIELDS = [
    "run_label",
    "arena_id",
    "path_id",
    "step_id",
    "phase",
    "phase_step",
    "alpha",
    "num_cubes",
    "anchor_x",
    "anchor_y",
    "start_x",
    "start_y",
    "start_theta",
    "target_world_x",
    "target_world_y",
    "target_world_theta",
    "command_joint_x",
    "command_joint_y",
    "command_joint_theta",
    "shovel_x",
    "shovel_y",
    "shovel_z",
    "shovel_theta",
    "error_x",
    "error_y",
    "error_theta",
    "error_xy",
    "joint_x_pos",
    "joint_y_pos",
    "joint_theta_pos",
    "joint_x_vel",
    "joint_y_vel",
    "joint_theta_vel",
    "base_x",
    "base_y",
    "base_z",
    "gravel_com_x",
    "gravel_com_y",
    "gravel_com_z",
    "gravel_bbox_xmin",
    "gravel_bbox_xmax",
    "gravel_bbox_ymin",
    "gravel_bbox_ymax",
    "gravel_bbox_zmin",
    "gravel_bbox_zmax",
    "gravel_spread_xy_rms",
    "shovel_to_gravel_com_xy",
    "shovel_to_nearest_gravel_xy",
    "gravel_com_shift_x",
    "gravel_com_shift_y",
    "gravel_com_shift_z",
    "gravel_com_shift_xy",
]


class RunTraceLogger:
    def __init__(self, csv_path):
        self.csv_path = Path(csv_path)
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=TRACE_FIELDS, extrasaction="ignore")
        self._writer.writeheader()

    def log(self, row):
        full_row = {key: row.get(key, "") for key in TRACE_FIELDS}
        self._writer.writerow(full_row)

    def close(self):
        self._file.flush()
        self._file.close()


def _world_to_joint(target_x, target_y, controller):
    """Convert world-frame XY target to joint-space commands.

    The robot IK naively sets joint_x = target_x, but the EE world X =
    base_world_x + joint_x.  We compensate by subtracting the base world X so
    the EE actually reaches (target_x, target_y) in world coordinates.
    Y prismatic starts at base_world_y = 0, so no correction is needed there.
    """
    base_pos, _ = p.getBasePositionAndOrientation(controller.robotId)
    return target_x - base_pos[0], target_y


def collect_dataset(controller, cube_urdf, grid_cfg, arena_cfg, collect_cfg, seed=7, trace_csv_path=None, run_label=None):
    rng = np.random.default_rng(seed)
    samples = []
    half_crop = (grid_cfg.h * grid_cfg.cell_size) / 2.0  # anchor radius = half the crop
    trace_logger = RunTraceLogger(trace_csv_path) if trace_csv_path is not None else None
    run_label = run_label if run_label is not None else f"seed_{seed}"

    def _trace_row_base(arena_id, path_id, step_id, phase, phase_step, anchor, start, target_pose, cur_pose, current_gravel, num_cubes, step_info=None):
        if hasattr(controller, "world_to_joint"):
            cmd_jx, cmd_jy, cmd_jtheta = controller.world_to_joint(target_pose[0], target_pose[1], target_pose[2])
        else:
            cmd_jx = float(target_pose[0] - controller.base_pos[0]) if hasattr(controller, "base_pos") else float("nan")
            cmd_jy = float(target_pose[1] - controller.base_pos[1]) if hasattr(controller, "base_pos") else float("nan")
            cmd_jtheta = float(target_pose[2])

        row = {
            "run_label": run_label,
            "arena_id": arena_id,
            "path_id": path_id,
            "step_id": step_id,
            "phase": phase,
            "phase_step": phase_step,
            "alpha": 0.0 if step_info is None else float(step_info["alpha"]),
            "num_cubes": num_cubes,
            "anchor_x": float(anchor[0]),
            "anchor_y": float(anchor[1]),
            "start_x": float(start[0]),
            "start_y": float(start[1]),
            "start_theta": float(start[2]),
            "target_world_x": float(target_pose[0]),
            "target_world_y": float(target_pose[1]),
            "target_world_theta": float(target_pose[2]),
            "command_joint_x": float(cmd_jx),
            "command_joint_y": float(cmd_jy),
            "command_joint_theta": float(cmd_jtheta),
            "shovel_x": float(cur_pose[0]),
            "shovel_y": float(cur_pose[1]),
            "shovel_z": float(cur_pose[2]),
            "shovel_theta": float(cur_pose[3]),
            "error_x": float(target_pose[0] - cur_pose[0]),
            "error_y": float(target_pose[1] - cur_pose[1]),
            "error_theta": float(np.arctan2(np.sin(target_pose[2] - cur_pose[3]), np.cos(target_pose[2] - cur_pose[3]))),
            "error_xy": float(np.hypot(target_pose[0] - cur_pose[0], target_pose[1] - cur_pose[1])),
            "base_x": float(controller.base_pos[0]) if hasattr(controller, "base_pos") else float("nan"),
            "base_y": float(controller.base_pos[1]) if hasattr(controller, "base_pos") else float("nan"),
            "base_z": float(controller.base_pos[2]) if hasattr(controller, "base_pos") else float("nan"),
        }
        if step_info is not None:
            row.update(step_info)
        row.update(current_gravel)
        return row

    try:
        arenas = tqdm(range(collect_cfg.n_arenas), desc="arenas")
        for arena_id in arenas:
            cube_ids = spawn_scattered_cubes(cube_urdf, arena_cfg, seed + arena_id)
            cube_pos = _cube_positions(cube_ids)
            arena_gravel_initial = _gravel_summary(cube_ids)
            arena_gravel_initial_vec = np.array(
                [arena_gravel_initial["gravel_com_x"], arena_gravel_initial["gravel_com_y"], arena_gravel_initial["gravel_com_z"]],
                dtype=np.float32,
            )

            for path_id in range(collect_cfg.paths_per_arena):
                sx, sy, anchor = _anchor_near_cubes(cube_pos, collect_cfg.x_bounds, collect_cfg.y_bounds, rng)
                start = (sx, sy, collect_cfg.start_pose[2])

                if hasattr(controller, "reset_world_pose"):
                    controller.reset_world_pose(start[0], start[1], start[2])
                else:
                    controller.apply_cartesian_control_sequential(start[0], start[1], start[2], total_steps=180, verbose=False)

                if trace_logger is not None:
                    cur_pose = _current_pose_full(controller)
                    current_gravel = _gravel_summary(cube_ids, ref_xy=cur_pose[:2], initial_com=arena_gravel_initial_vec)
                    trace_logger.log(_trace_row_base(
                        arena_id, path_id, -1, "reset", 0, anchor, start, start, cur_pose, current_gravel, len(cube_ids),
                    ))

                path = _tethered_path(
                    start,
                    anchor,
                    half_crop,
                    collect_cfg.steps_per_path,
                    collect_cfg.x_bounds,
                    collect_cfg.y_bounds,
                    collect_cfg.max_step_xy,
                    rng,
                )

                for step_id, next_pose in enumerate(path):
                    cur_pose = _current_pose(controller)
                    g_t = grid_utils.cubes_grid(p, cube_ids, cur_pose, grid_cfg)
                    s_t = grid_utils.shovel_grid_at_pose(cur_pose, cur_pose, grid_cfg)
                    s_next = grid_utils.shovel_grid_at_pose(next_pose, cur_pose, grid_cfg)

                    def _trace_callback(step_info):
                        if trace_logger is None:
                            return
                        current_gravel = _gravel_summary(cube_ids, ref_xy=(step_info["shovel_x"], step_info["shovel_y"]), initial_com=arena_gravel_initial_vec)
                        trace_logger.log(_trace_row_base(
                            arena_id,
                            path_id,
                            step_id,
                            "move",
                            int(round(step_info["alpha"] * max(1, collect_cfg.move_steps - 1))),
                            anchor,
                            start,
                            next_pose,
                            (step_info["shovel_x"], step_info["shovel_y"], step_info["shovel_z"], step_info["shovel_theta"]),
                            current_gravel,
                            len(cube_ids),
                            step_info,
                        ))

                    controller.apply_cartesian_control_sequential(
                        next_pose[0],
                        next_pose[1],
                        next_pose[2],
                        total_steps=collect_cfg.move_steps,
                        verbose=False,
                        step_callback=_trace_callback,
                    )

                    for settle_step in range(collect_cfg.settle_steps):
                        p.stepSimulation()
                        if trace_logger is not None:
                            cur_pose = _current_pose_full(controller)
                            current_gravel = _gravel_summary(cube_ids, ref_xy=cur_pose[:2], initial_com=arena_gravel_initial_vec)
                            trace_logger.log(_trace_row_base(
                                arena_id,
                                path_id,
                                step_id,
                                "settle",
                                settle_step,
                                anchor,
                                start,
                                next_pose,
                                cur_pose,
                                current_gravel,
                                len(cube_ids),
                                {"alpha": 1.0},
                            ))

                    g_t1 = grid_utils.cubes_grid(p, cube_ids, cur_pose, grid_cfg)
                    x = np.stack([g_t, s_t, s_next], axis=0).astype(np.float32)
                    y = g_t1[None, :, :].astype(np.float32)

                    samples.append(
                        {
                            "input": x,
                            "target": y,
                            "metadata": {
                                "arena_id": arena_id,
                                "path_id": path_id,
                                "step_id": step_id,
                                "ref_pose": cur_pose,
                                "next_pose": next_pose,
                                "cell_size": grid_cfg.cell_size,
                                "grid_hw": (grid_cfg.h, grid_cfg.w),
                                "num_cubes": len(cube_ids),
                            },
                        }
                    )

            for bid in cube_ids:
                p.removeBody(bid)

        return samples
    finally:
        if trace_logger is not None:
            trace_logger.close()
