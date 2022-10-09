import pygame
import time
from grid import Grid
from utils import *


class Window:
    def __init__(self, x_dim=100, y_dim=50,
                 start: (int, int) = (0, 0),
                 goal: (int, int) = (50, 50),
                 viewing_range = 3):

        self.title = f'D* Lte {version}'
        self.width = 10
        self.height = 10
        self.margin = 0
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.start = start
        self.current = start
        self.observation = {"pos": None, "type": None}
        self.goal = goal
        self.field = Grid(x_dim=x_dim, y_dim=y_dim)
        self.viewing_range = viewing_range

        pygame.init()
        window_size = [(self.width + self.margin) * y_dim + self.margin,
                       (self.height + self.margin) * x_dim + self.margin]
        self.screen = pygame.display.set_mode(window_size)
        pygame.display.set_caption(self.title)

        self.quit = False
        self.clock = pygame.time.Clock()

    def get_position(self):
        return self.current

    def set_position(self, coord: (int, int)):
        self.current = coord

    def get_goal(self):
        return self.goal

    def set_goal(self, goal: (int, int)):
        self.goal = goal

    def set_start(self, start: (int, int)):
        self.start = start

    def draw_path(self, path=None):
        if not path:
            return
        for cell in path:
            cell_center = [round(cell[1] * (self.width + self.margin) + self.width / 2) + self.margin,
                           round(cell[0] * (self.height + self.margin) + self.height / 2) + self.margin]
            pygame.draw.circle(self.screen, Colors.orange, cell_center, round(self.width / 2) - 2)

    def draw_blocks(self, blocks=None):
        if not blocks:
            return
        for block in blocks:
            pygame.draw.rect(self.screen, Colors.brown, [(self.margin + self.width) * block[1] + self.margin,
                                                         (self.margin + self.height) * block[0] + self.margin,
                                                         self.width,
                                                         self.height])

    def run(self, path=None):
        global viewing_range
        if not path:
            path = []

        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                print("quit")
                self.quit = True

            elif e.type == pygame.KEYDOWN and e.key == pygame.K_SPACE:
                if path:
                    (x, y) = path[1]
                    self.set_position((x, y))

            elif pygame.mouse.get_pressed()[0]:
                (col, row) = pygame.mouse.get_pos()
                x = row // (self.height + self.margin)
                y = col // (self.width + self.margin)

                grid_cell = (x, y)

                if self.field.is_empty(grid_cell):
                    self.field.set_block(grid_cell)
                    self.observation = {"pos": grid_cell, "type": BLOCK}
                else:
                    self.field.remove_block(grid_cell)
                    self.observation = {"pos": grid_cell, "type": EMPTY}

        self.screen.fill(Colors.black)
        for row in range(self.x_dim):
            for column in range(self.y_dim):
                pygame.draw.rect(self.screen, Colors.get_color(self.field.grid_map[row][column]),
                                 [(self.margin + self.width) * column + self.margin,
                                  (self.margin + self.height) * row + self.margin,
                                  self.width,
                                  self.height])

        self.draw_path(path=path)
        pygame.draw.rect(self.screen, Colors.get_color(GOAL), [(self.margin + self.width) * self.goal[1] + self.margin,
                                                               (self.margin + self.height) * self.goal[0] + self.margin,
                                                               self.width,
                                                               self.height])

        robot_center = [round(self.current[1] * (self.width + self.margin) + self.width / 2) + self.margin,
                        round(self.current[0] * (self.height + self.margin) + self.height / 2) + self.margin]
        pygame.draw.circle(self.screen, Colors.blue, robot_center, round(self.width / 2) - 2)
        pygame.draw.rect(self.screen, Colors.blue,
                         [robot_center[0] - self.viewing_range * (self.height + self.margin),
                          robot_center[1] - self.viewing_range * (self.width + self.margin),
                          2 * self.viewing_range * (self.height + self.margin),
                          2 * self.viewing_range * (self.width + self.margin)], 2)

        self.clock.tick(20)
        pygame.display.flip()
    pygame.quit()
