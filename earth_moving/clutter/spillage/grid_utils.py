from dataclasses import dataclass

import numpy as np


@dataclass
class GridConfig:
    h: int = 32
    w: int = 32
    cell_size: float = 0.025


def yaw_from_quat(quat):
    x, y, z, w = quat
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)


def world_to_body(points_xy, ref_pose):
    x_ref, y_ref, yaw_ref = ref_pose
    pts = np.asarray(points_xy, dtype=np.float32)
    shifted = pts - np.array([x_ref, y_ref], dtype=np.float32)
    c, s = np.cos(-yaw_ref), np.sin(-yaw_ref)
    r = np.array([[c, -s], [s, c]], dtype=np.float32)
    return shifted @ r.T


def points_to_grid(points_local, grid_cfg):
    h, w, cs = grid_cfg.h, grid_cfg.w, grid_cfg.cell_size
    grid = np.zeros((h, w), dtype=np.float32)
    pts = np.asarray(points_local, dtype=np.float32)
    if pts.size == 0:
        return grid
    col = np.floor(pts[:, 0] / cs + w / 2.0).astype(np.int32)
    row = np.floor(pts[:, 1] / cs + h / 2.0).astype(np.int32)
    m = (row >= 0) & (row < h) & (col >= 0) & (col < w)
    grid[row[m], col[m]] = 1.0
    return grid


def sample_box_points(center_xy, yaw, half_x, half_y, nx=9, ny=25):
    xs = np.linspace(-half_x, half_x, nx, dtype=np.float32)
    ys = np.linspace(-half_y, half_y, ny, dtype=np.float32)
    lx, ly = np.meshgrid(xs, ys)
    local = np.stack([lx.reshape(-1), ly.reshape(-1)], axis=1)
    c, s = np.cos(yaw), np.sin(yaw)
    r = np.array([[c, -s], [s, c]], dtype=np.float32)
    world = local @ r.T + np.array(center_xy, dtype=np.float32)
    return world


def shovel_grid_at_pose(query_pose, ref_pose, grid_cfg, blade_half_extents=(0.005, 0.10)):
    qx, qy, qyaw = query_pose
    hx, hy = blade_half_extents
    pts_world = sample_box_points((qx, qy), qyaw, hx, hy)
    pts_local = world_to_body(pts_world, ref_pose)
    return points_to_grid(pts_local, grid_cfg)


def cubes_grid(pybullet_client, cube_ids, ref_pose, grid_cfg):
    pts_world = []
    for bid in cube_ids:
        pos, _ = pybullet_client.getBasePositionAndOrientation(bid)
        pts_world.append([pos[0], pos[1]])
    if not pts_world:
        return np.zeros((grid_cfg.h, grid_cfg.w), dtype=np.float32)
    pts_local = world_to_body(np.asarray(pts_world, dtype=np.float32), ref_pose)
    return points_to_grid(pts_local, grid_cfg)
