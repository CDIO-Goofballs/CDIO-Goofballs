import traceback

from ImageRecognition.RoboFlow.MainImageRecognition import (get_wall_corners, get_vip_ball, get_balls, get_cross,
                                                            get_egg,
                                                            get_small_goal, get_big_goal, get_angle, get_position,
                                                            get_size)
from Pathfinding.Pathing import path_finding


def pathing():
    try:
        cross = get_cross()
        egg = get_egg()
        start = get_position()
        vip = get_vip_ball()
        balls = get_balls()
        wall_corners = get_wall_corners()
        end = get_small_goal()
        width, height = get_size()
        start_angle = get_angle()

        if not end:
            end = get_big_goal()
        if end:
            end = (end[0] + (width / 2 - end[0]) / 2.9, end[1])

        path = path_finding(
            cross=cross,egg = egg, start=start, vip=vip, balls=balls, end=end,
            wall_corners=wall_corners, width=width, height=height, start_angle=start_angle,)

        if path is not None:
            print("Path found:", path)
            return path
        else:
            print("No path found.")
            return []
    except Exception as e:
        print("Error in pathing thread:", e)
        traceback.print_exc()
        return []