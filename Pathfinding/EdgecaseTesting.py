import unittest
import random
import traceback
from Pathing import path_finding
from Polygons import convert_cross_to_polygons, create_egg
from shapely.geometry import Point
from unittest.mock import patch


def generate_cross_points(center_x, center_y, size=20):
    half = size / 2
    top = (center_x, center_y + half)
    bottom = (center_x, center_y - half)
    right = (center_x + half, center_y)
    left = (center_x - half, center_y)
    return (top, bottom, right, left)


class TestPathFinding(unittest.TestCase):
    def get_xy(self, pt):
        return (pt.x, pt.y) if hasattr(pt, 'x') else pt

    def test_cross_close_to_wall(self):
        width, height = 160, 120
        cross = generate_cross_points(width/2, height/2 + 10)
        robot_radius = 10.5
        start = (20, 60)
        end = (140, 60)

        path = path_finding(cross=cross, egg=None, start=start, vip=None, balls=[], end=end,
                            wall_corners=((0, 0), (0, height), (width, height), (width, 0)),
                            robot_radius=robot_radius, width=width, height=height, debug=True)

        self.assertGreater(len(path), 0, "Path should be found when cross is close to wall")

    def test_start_equals_end(self):
        width, height = 160, 120
        point = (60, 30)
        cross = generate_cross_points(80, 60)
        robot_radius = 10.5

        path = path_finding(cross=cross, egg=None, start=point, vip=None, balls=[], end=point,
                            wall_corners=((0, 0), (0, height), (width, height), (width, 0)),
                            robot_radius=robot_radius, width=width, height=height, debug=True)

        self.assertGreater(len(path), 0, "Path should be found when start equals end")

    def test_start_inside_obstacle(self):
        width, height = 160, 120
        cross = generate_cross_points(80, 60)
        robot_radius = 10.5
        obstacles = convert_cross_to_polygons(cross, 3)
        start = list(obstacles[0].centroid.coords)[0]
        end = (150, 110)

        path = path_finding(cross=cross, egg=None, start=start, vip=None, balls=[], end=end,
                            wall_corners=((0, 0), (0, height), (width, height), (width, 0)),
                            robot_radius=robot_radius, width=width, height=height, debug=True)

        self.assertEqual(len(path), 0)

    def test_start_outside_bounds(self):
        width, height = 160, 120
        start = (-10, -10)
        end = (80, 60)
        robot_radius = 10.5

        path = path_finding(cross=None, egg=None, start=start, vip=None, balls=[], end=end,
                            wall_corners=((0, 0), (0, height), (width, height), (width, 0)),
                            robot_radius=robot_radius, width=width, height=height, debug=True)

        self.assertEqual(len(path), 0)

    def test_end_inside_obstacle(self):
        width, height = 160, 120
        cross = generate_cross_points(80, 60)
        robot_radius = 10.5
        obstacles = convert_cross_to_polygons(cross, 3)
        start = (10, 10)
        end = list(obstacles[0].centroid.coords)[0]

        path = path_finding(cross=cross, egg=None, start=start, vip=None, balls=[], end=end,
                            wall_corners=((0, 0), (0, height), (width, height), (width, 0)),
                            robot_radius=robot_radius, width=width, height=height, debug=True)

        self.assertEqual(len(path), 0)

    def test_completely_blocked_path(self):
        width, height = 160, 120
        robot_radius = 10.5
        cross = ((80, 110), (80, 10), (110, 60), (50, 60))
        start = (20, 60)
        end = (140, 60)

        path = path_finding(cross=cross, egg=None, start=start, vip=None, balls=[], end=end,
                            wall_corners=((0, 0), (0, height), (width, height), (width, 0)),
                            robot_radius=robot_radius, width=width, height=height, debug=True)

        self.assertEqual(len(path), 0)

    def test_path_around_border(self):
        width, height = 160, 120
        robot_radius = 10.5
        cross = ((80, 70), (80, 50), (90, 60), (70, 60))
        start = (20, 60)
        end = (140, 60)

        path = path_finding(cross=cross, egg=None, start=start, vip=None, balls=[], end=end,
                            wall_corners=((0, 0), (0, height), (width, height), (width, 0)),
                            robot_radius=robot_radius, width=width, height=height, debug=True)

        self.assertGreater(len(path), 0)

    def test_dense_ball_clustering(self):
        width, height = 160, 120
        robot_radius = 10.5
        cluster_center = (80, 60)
        balls = [(cluster_center[0] + dx, cluster_center[1] + dy) for dx in range(-5, 6, 2) for dy in range(-5, 6, 2)]
        start = (20, 60)
        end = (140, 60)

        path = path_finding(cross=None, egg=None, start=start, vip=None, balls=balls, end=end,
                            wall_corners=((0, 0), (0, height), (width, height), (width, 0)),
                            robot_radius=robot_radius, width=width, height=height, debug=True)

        self.assertGreater(len(path), 0)


    def test_vip_present_but_not_blocking(self):
        width, height = 160, 120
        robot_radius = 10.5
        vip = (30, 30)
        start = (20, 20)
        end = (140, 100)

        path = path_finding(cross=None, egg=None, start=start, vip=vip, balls=[], end=end,
                            wall_corners=((0, 0), (0, height), (width, height), (width, 0)),
                            robot_radius=robot_radius, width=width, height=height, debug=True)

        self.assertGreater(len(path), 0)

if __name__ == "__main__":
    unittest.main()
