from dataclasses import dataclass

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
    fill_ratio: float = 0.60
    clusters: int = 5
    sigma_cells: float = 4.0


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


def _clustered_mask(nx, ny, fill_ratio, clusters, sigma_cells, rng):
    yy, xx = np.mgrid[0:ny, 0:nx]
    score = np.zeros((ny, nx), dtype=np.float32)
    for _ in range(clusters):
        cx = rng.uniform(0, nx - 1)
        cy = rng.uniform(0, ny - 1)
        amp = rng.uniform(0.8, 1.2)
        score += amp * np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2.0 * sigma_cells ** 2))
    score += 0.15 * rng.random((ny, nx), dtype=np.float32)
    k = int(round(fill_ratio * nx * ny))
    th = np.partition(score.ravel(), -k)[-k]
    return score >= th


def spawn_clustered_cubes(cube_urdf, arena_cfg, seed):
    rng = np.random.default_rng(seed)
    xs = np.arange(arena_cfg.x_min, arena_cfg.x_max, arena_cfg.spacing)
    ys = np.arange(arena_cfg.y_min, arena_cfg.y_max, arena_cfg.spacing)
    nx, ny = len(xs), len(ys)
    mask = _clustered_mask(nx, ny, arena_cfg.fill_ratio, arena_cfg.clusters, arena_cfg.sigma_cells, rng)

    cube_ids = []
    for iy, y in enumerate(ys):
        for ix, x in enumerate(xs):
            if not mask[iy, ix]:
                continue
            jx = rng.uniform(-0.003, 0.003)
            jy = rng.uniform(-0.003, 0.003)
            bid = p.loadURDF(
                cube_urdf,
                [float(x + jx), float(y + jy), arena_cfg.z_spawn],
                p.getQuaternionFromEuler([0.0, 0.0, 0.0]),
            )
            cube_ids.append(bid)

    for _ in range(240):
        p.stepSimulation()
    return cube_ids


def _random_path(start_pose, steps, x_bounds, y_bounds, max_step_xy, rng):
    x, y, yaw = start_pose
    path = []
    for _ in range(steps):
        dx = rng.uniform(-max_step_xy, max_step_xy)
        dy = rng.uniform(-max_step_xy, max_step_xy)
        x = float(np.clip(x + dx, x_bounds[0], x_bounds[1]))
        y = float(np.clip(y + dy, y_bounds[0], y_bounds[1]))
        path.append((x, y, yaw))
    return path


def _current_pose(controller):
    pos, yaw = controller.forward_kinematics_2d()
    return float(pos[0]), float(pos[1]), float(yaw)


def collect_dataset(controller, cube_urdf, grid_cfg, arena_cfg, collect_cfg, seed=7):
    rng = np.random.default_rng(seed)
    samples = []

    arenas = tqdm(range(collect_cfg.n_arenas), desc="arenas")
    for arena_id in arenas:
        cube_ids = spawn_clustered_cubes(cube_urdf, arena_cfg, seed + arena_id)

        for path_id in range(collect_cfg.paths_per_arena):
            sx = rng.uniform(collect_cfg.x_bounds[0], collect_cfg.x_bounds[1])
            sy = rng.uniform(collect_cfg.y_bounds[0], collect_cfg.y_bounds[1])
            start = (sx, sy, collect_cfg.start_pose[2])
            controller.apply_cartesian_control_sequential(start[0], start[1], start[2], total_steps=180, verbose=False)

            path = _random_path(start, collect_cfg.steps_per_path, collect_cfg.x_bounds, collect_cfg.y_bounds, collect_cfg.max_step_xy, rng)

            for step_id, next_pose in enumerate(path):
                cur_pose = _current_pose(controller)
                g_t = grid_utils.cubes_grid(p, cube_ids, cur_pose, grid_cfg)
                s_t = grid_utils.shovel_grid_at_pose(cur_pose, cur_pose, grid_cfg)
                s_next = grid_utils.shovel_grid_at_pose(next_pose, cur_pose, grid_cfg)

                controller.apply_cartesian_control_sequential(next_pose[0], next_pose[1], next_pose[2], total_steps=collect_cfg.move_steps, verbose=False)
                for _ in range(collect_cfg.settle_steps):
                    p.stepSimulation()

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
