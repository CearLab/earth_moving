"""Generate random occupancy grids using PreprocessEnvironment.

Matches the preprocessing flow used in test_v1.ipynb:
particles -> density grid -> occupancy grid.
"""

from __future__ import annotations

from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

repo_root = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(repo_root))
from earth_moving.clutter.mcts_vanilla_v1.preprocess_environment_v1 import (
    PreprocessEnvironment,
)


def build_boundary_points(grid_size: int) -> list[tuple[float, float]]:
    return (
        [(0.0, y) for y in np.linspace(0.0, 1.0, grid_size, endpoint=False)]
        + [(x, 1.0) for x in np.linspace(0.0, 1.0, grid_size, endpoint=False)]
        + [(1.0, y) for y in np.linspace(1.0, 0.0, grid_size, endpoint=False)]
        + [(x, 0.0) for x in np.linspace(1.0, 0.0, grid_size, endpoint=False)]
    )


def save_grid_png(grid: np.ndarray, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(4, 4))
    plt.imshow(grid, origin="lower", cmap="gray")
    plt.axis("off")
    plt.tight_layout(pad=0)
    plt.savefig(path, dpi=120)
    plt.close()


def generate_oriented_rect_particles(
    rng: np.random.Generator,
    num_particles: int,
    center: np.ndarray,
    length: float,
    width: float,
    angle_rad: float,
) -> np.ndarray:
    dir_vector = np.array([np.cos(angle_rad), np.sin(angle_rad)])
    perp_vector = np.array([-dir_vector[1], dir_vector[0]])
    t_vals = rng.uniform(-length / 2.0, length / 2.0, num_particles)
    s_vals = rng.uniform(-width / 2.0, width / 2.0, num_particles)
    return center + np.outer(t_vals, dir_vector) + np.outer(s_vals, perp_vector)


def main() -> None:
    num_grids = 100
    # num_particles_range = (500, 3000)
    num_particles_range = (100, 1000)
    length_range = (0.2, 0.8)
    width_range = (0.05, 0.1)
    angle_options = np.linspace(0, np.pi, 9)
    grid_size = 20
    push_width = 0.2
    particle_radius_world = 0.005
    bounding_box = np.array([[0.0, 0.0], [1.0, 1.0]])
    stochasticity = False
    binary_grid = not stochasticity
    rng = np.random.default_rng()

    boundary_points = build_boundary_points(grid_size)
    preprocess = PreprocessEnvironment(
        boundary_points=boundary_points,
        push_width=push_width,
        stochasticity=stochasticity,
        grid_size=grid_size,
        binary_grid=binary_grid,
    )

    out_dir = Path(__file__).parent / "data" / "deterministic" / "1_rect_occupancy_grids"
    out_dir.mkdir(parents=True, exist_ok=True)

    for idx in range(num_grids):
        num_particles = int(rng.integers(num_particles_range[0], num_particles_range[1] + 1))
        rect_length = float(rng.uniform(length_range[0], length_range[1]))
        rect_width = float(rng.uniform(width_range[0], width_range[1]))
        rect_angle = float(rng.choice(angle_options))
        rect_center = rng.uniform(low=bounding_box[0], high=bounding_box[1], size=2)
        particles = generate_oriented_rect_particles(
            rng=rng,
            num_particles=num_particles,
            center=rect_center,
            length=rect_length,
            width=rect_width,
            angle_rad=rect_angle,
        )
        density_grid = preprocess.particles_to_binary_img(
            particles,
            binary_grid_size=1000,
            bounding_box=bounding_box,
            particle_radius_world=particle_radius_world,
        )
        occ_grid, _ = preprocess.binary_img_to_grid(density_grid)
        occ_grid2, _ = preprocess.particles_to_grid(particles)

        np.save(out_dir / f"occ_{idx:04d}.npy", occ_grid.astype(np.uint8))
        save_grid_png(density_grid, out_dir / "vis" / f"img_{idx:04d}.png")
        preprocess.draw_grid_environment(occ_grid, save_path=out_dir / "vis" / f"occ_{idx:04d}.png")


if __name__ == "__main__":
    main()
