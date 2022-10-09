import numpy as np
from utils import *


class Grid:
    def __init__(self, x_dim, y_dim):
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.map = (x_dim, y_dim)

        self.grid_map = np.zeros(self.map, dtype=np.uint8)

        self.visited = {}

    def get_map(self):
        return self.grid_map

    def is_empty(self, coord: (int, int)) -> bool:
        (x, y) = (round(coord[0]), round(coord[1]))
        (row, col) = (x, y)

        return self.grid_map[row][col] == EMPTY

    def in_field(self, coord: (int, int)) -> bool:
        (x, y) = coord
        return 0 <= x < self.x_dim and 0 <= y < self.y_dim

    def set_block(self, coord: (int, int)):
        (x, y) = (round(coord[0]), round(coord[1]))
        (row, col) = (x, y)
        self.grid_map[row, col] = BLOCK

    def remove_block(self, coord: (int, int)):
        (x, y) = (round(coord[0]), round(coord[1]))
        (row, col) = (x, y)
        self.grid_map[row, col] = EMPTY

    def succ(self, v: (int, int), avoid_blocks: bool = False) -> list:
        (x, y) = v

        movements = get_neighbours_4(x=x, y=y)
        if (x + y) % 2 == 0:
            movements.reverse()
        filtered_movements = self.filter(neighbors=movements, avoid_blocks=avoid_blocks)
        return list(filtered_movements)

    def filter(self, neighbors: list, avoid_blocks: bool):
        if avoid_blocks:
            return [node for node in neighbors if self.in_field(node) and self.is_empty(node)]
        return [node for node in neighbors if self.in_field(node)]

    def local_observation(self, global_position: (int, int), view_range: int = 2) -> dict:
        (px, py) = global_position
        nodes = [(x, y) for x in range(px - view_range, px + view_range + 1)
                 for y in range(py - view_range, py + view_range + 1)
                 if self.in_field((x, y))]
        return {node: EMPTY if self.is_empty(coord=node) else BLOCK for node in nodes}




class SLAM:
    def __init__(self, map: Grid, view_range: int):
        self.ground_truth_map = map
        self.slam_map = Grid(x_dim=map.x_dim,
                             y_dim=map.y_dim)
        self.view_range = view_range

    def set_ground_truth_map(self, gt_map: Grid):
        self.ground_truth_map = gt_map

    def c(self, u: (int, int), v: (int, int)) -> float:
        if not self.slam_map.is_empty(u) or not self.slam_map.is_empty(v):
            return float('inf')
        else:
            return heuristic(u, v)

    def rescan(self, global_position: (int, int)):
        local_observation = self.ground_truth_map.local_observation(global_position=global_position,
                                                                    view_range=self.view_range)

        vertices = self.update_changed_edge_costs(local_grid=local_observation)
        return vertices, self.slam_map

    def update_changed_edge_costs(self, local_grid: dict) -> Vertices:
        vertices = Vertices()
        for node, value in local_grid.items():
            if value == BLOCK:
                if self.slam_map.is_empty(node):
                    v = Vertex(coord=node)
                    succ = self.slam_map.succ(node)
                    for u in succ:
                        v.add_cost(v=u, cost=self.c(u, v.coord))
                    vertices.add_vertex(v)
                    self.slam_map.set_block(node)
            else:
                if not self.slam_map.is_empty(node):
                    v = Vertex(coord=node)
                    succ = self.slam_map.succ(node)
                    for u in succ:
                        v.add_cost(v=u, cost=self.c(u, v.coord))
                    vertices.add_vertex(v)
                    self.slam_map.remove_block(node)
        return vertices
