import numpy as np
import pybullet as p

class Grid:
    def __init__(self, cell_size, x_range, y_range):
        self.cell_size = cell_size
        self.x_range = x_range
        self.y_range = y_range
        self.cells = {}

        self.x_cells = int(np.ceil((self.x_range[1] - self.x_range[0]) / self.cell_size))
        self.y_cells = int(np.ceil((self.y_range[1] - self.y_range[0]) / self.cell_size))

    def get_cell_index(self, position):
        x_index = int((position[0] - self.x_range[0]) / self.cell_size)
        y_index = int((position[1] - self.y_range[0]) / self.cell_size)
        return x_index, y_index

    def assign_pebbles_to_cells(self, pebble_positions):
        self.cells = { (x, y): [] for x in range(self.x_cells) for y in range(self.y_cells)}
        for pos in pebble_positions:
            x_index, y_index = self.get_cell_index(pos)
            self.cells[(x_index, y_index)].append(pos)

    def draw_grid(self):
        for i in range(self.x_cells + 1):
            x = self.x_range[0] + i * self.cell_size
            p.addUserDebugLine([x, self.y_range[0], 0], [x, self.y_range[1], 0], [0.5, 0.5, 0.5], lineWidth=1)

        for j in range(self.y_cells + 1):
            y = self.y_range[0] + j * self.cell_size
            p.addUserDebugLine([self.x_range[0], y, 0], [self.x_range[1], y, 0], [0.5, 0.5, 0.5], lineWidth=1)

    def get_cell_center(self, x_index, y_index):
        x_center = self.x_range[0] + (x_index + 0.5) * self.cell_size
        y_center = self.y_range[0] + (y_index + 0.5) * self.cell_size
        return np.array([x_center, y_center])

    def get_cells_with_pebbles(self):
        return {cell: pebbles for cell, pebbles in self.cells.items() if pebbles}
