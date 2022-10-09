import math

version = 'v.0.1'
viewing_range = 2

EMPTY = 0
BLOCK = -1
GOAL = 1


class Colors:
    black = (0, 0, 0)
    white = (245, 242, 223)
    blue = (120, 99, 255)
    green = (99, 255, 125)
    yellow = (221, 255, 99)
    orange = (255, 156, 99)
    brown = (82, 71, 65)

    defined_colors = {
        EMPTY: white,
        GOAL: green,
        BLOCK: brown
    }

    @classmethod
    def get_color(cls, cell):
        return Colors().defined_colors.get(cell, Colors().brown)


class Vertex:
    def __init__(self, coord: (int, int)):
        self.coord = coord
        self.movement_costs = {}

    def add_cost(self, v: (int, int), cost: float):
        if v != self.coord:
            self.movement_costs[v] = cost

    @property
    def get_costs(self):
        return self.movement_costs


class Vertices:
    def __init__(self):
        self.list = []

    def add_vertex(self, v: Vertex):
        self.list.append(v)

    @property
    def vertices(self):
        return self.list


def heuristic(p: (int, int), q: (int, int)) -> float:
    return math.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)


def get_neighbours_4(x: int, y: int) -> list:
    return [
        (x + 1, y + 0),
        (x + 0, y + 1),
        (x - 1, y + 0),
        (x + 0, y - 1)
    ]
