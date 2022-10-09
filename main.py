from D_Lite import DStarLite
from utils import *
from grid import Grid, SLAM
from window import Window

if __name__ == '__main__':
    x_dim = 50
    y_dim = 50
    start = (10, 10)
    goal = (40, 40)
    window = Window(x_dim=x_dim,
                    y_dim=y_dim,
                    start=start,
                    goal=goal,
                    viewing_range=viewing_range)

    old_map = new_map = window.field
    prev_coord = new_coord = start

    dstar = DStarLite(map=new_map, start=start, goal=goal)
    slam = SLAM(map=new_map, view_range=viewing_range)

    path, g, rhs = dstar.move_and_replan(robot_position=new_coord)
    while not window.quit:
        window.run(path=path)
        new_coord = window.current
        new_observation = window.observation
        new_map = window.field

        if new_observation is not None:
            old_map = new_map
            slam.set_ground_truth_map(gt_map=new_map)

        if new_coord != prev_coord:
            prev_coord = new_coord

            new_edges_and_old_costs, slam_map = slam.rescan(global_position=new_coord)

            dstar.new_edges_and_old_costs = new_edges_and_old_costs
            dstar.sensed_map = slam_map

            path, g, rhs = dstar.move_and_replan(robot_position=new_coord)




