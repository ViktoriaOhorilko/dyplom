from priority_queue import PriorityQueue, Priority
import numpy as np
from grid import Grid
from utils import *


class DStarLite:
    def __init__(self, map: Grid, start: (int, int), goal: (int, int)):
        self.new_edges_and_old_costs = None

        self.start = start
        self.goal = goal
        self.last = start
        self.k_m = 0
        self.U = PriorityQueue()
        self.rhs = np.ones((map.x_dim, map.y_dim)) * np.inf
        self.g = self.rhs.copy()

        self.sensed_map = Grid(x_dim=map.x_dim,
                               y_dim=map.y_dim)
        self.rhs[self.goal] = 0
        self.U.insert(self.goal, Priority(heuristic(self.start, self.goal), 0))

    def count_key(self, v: (int, int)):
        k1 = min(self.g[v], self.rhs[v]) + heuristic(self.start, v) + self.k_m
        k2 = min(self.g[v], self.rhs[v])
        return Priority(k1, k2)

    def count_value(self, u: (int, int), v: (int, int)) -> float:
        if not self.sensed_map.is_empty(u) or not self.sensed_map.is_empty(v):
            return float('inf')
        else:
            return heuristic(u, v)

    def contain(self, u: (int, int)):
        return u in self.U.vertices_in_heap

    def update_vertex(self, u: (int, int)):
        if self.g[u] != self.rhs[u] and self.contain(u):
            self.U.update(u, self.count_key(u))
        elif self.g[u] != self.rhs[u] and not self.contain(u):
            self.U.insert(u, self.count_key(u))
        elif self.g[u] == self.rhs[u] and self.contain(u):
            self.U.remove(u)

    def compute_shortest_path(self):
        while self.U.top_key() < self.count_key(self.start) or self.rhs[self.start] > self.g[self.start]:
            u = self.U.top()
            k_old = self.U.top_key()
            k_new = self.count_key(u)

            if k_old < k_new:
                self.U.update(u, k_new)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                self.U.remove(u)
                pred = self.sensed_map.succ(v=u)
                for s in pred:
                    if s != self.goal:
                        self.rhs[s] = min(self.rhs[s], self.count_value(s, u) + self.g[u])
                    self.update_vertex(s)
            else:
                self.g_old = self.g[u]
                self.g[u] = float('inf')
                pred = self.sensed_map.succ(v=u)
                pred.append(u)
                for s in pred:
                    if self.rhs[s] == self.count_value(s, u) + self.g_old:
                        if s != self.goal:
                            min_s = float('inf')
                            succ = self.sensed_map.succ(v=s)
                            for s_ in succ:
                                temp = self.count_value(s, s_) + self.g[s_]
                                if min_s > temp:
                                    min_s = temp
                            self.rhs[s] = min_s
                    self.update_vertex(u)

    def rescan(self) -> Vertices:
        new_edges_and_old_costs = self.new_edges_and_old_costs
        self.new_edges_and_old_costs = None
        return new_edges_and_old_costs

    def move_and_replan(self, robot_position: (int, int)):
        path = [robot_position]
        self.start = robot_position
        self.last = self.start
        self.compute_shortest_path()

        while self.start != self.goal:
            assert (self.rhs[self.start] != float('inf')), "There is no known path!"

            succ = self.sensed_map.succ(self.start, avoid_blocks=False)
            min_s = float('inf')
            arg_min = None
            for s_ in succ:
                temp = self.count_value(self.start, s_) + self.g[s_]
                if temp < min_s:
                    min_s = temp
                    arg_min = s_

            self.start = arg_min
            path.append(self.start)
            changed_edges_with_old_cost = self.rescan()
            if changed_edges_with_old_cost:
                self.k_m += heuristic(self.last, self.start)
                self.last = self.start

                vertices = changed_edges_with_old_cost.vertices
                for vertex in vertices:
                    v = vertex.coord
                    succ_v = vertex.get_costs
                    for u, c_old in succ_v.items():
                        c_new = self.count_value(u, v)
                        if c_old > c_new:
                            if u != self.goal:
                                self.rhs[u] = min(self.rhs[u], self.count_value(u, v) + self.g[v])
                        elif self.rhs[u] == c_old + self.g[v]:
                            if u != self.goal:
                                min_s = float('inf')
                                succ_u = self.sensed_map.succ(v=u)
                                for s_ in succ_u:
                                    temp = self.count_value(u, s_) + self.g[s_]
                                    if min_s > temp:
                                        min_s = temp
                                self.rhs[u] = min_s
                            self.update_vertex(u)
            self.compute_shortest_path()
        print("path found!")
        return path, self.g, self.rhs
