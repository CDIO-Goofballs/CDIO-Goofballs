import math
import random
import unittest

from shapely.geometry import Point

from pathing import path_finding
from polygons import convert_cross_to_polygons


def rotate_point(px, py, cx, cy, angle_rad):
    s = math.sin(angle_rad)
    c = math.cos(angle_rad)

    px -= cx
    py -= cy

    xnew = px * c - py * s
    ynew = px * s + py * c

    px = xnew + cx
    py = ynew + cy
    return px, py


def rotate_cross(cross_points, angle_deg):
    angle_rad = math.radians(angle_deg)
    xs = [p[0] for p in cross_points]
    ys = [p[1] for p in cross_points]
    cx = sum(xs) / len(xs)
    cy = sum(ys) / len(ys)
    return tuple(rotate_point(x, y, cx, cy, angle_rad) for (x, y) in cross_points)


class TestCrossRotations(unittest.TestCase):
    def setUp(self):
        self.width = 160
        self.height = 120
        self.wall_corners = ((0, 0), (0, self.height), (self.width, self.height), (self.width, 0))
        self.robot_radius = 17

        # Base cross size ~20 units centered at (80, 60)
        half_size = 10  # half of 20
        cx, cy = 80, 70
        self.base_cross = (
            (cx, cy + half_size),  # top
            (cx, cy - half_size),  # bottom
            (cx + half_size, cy),  # right
            (cx - half_size, cy),  # left
        )
        self.start = (20, 60)
        self.end = (140, 60)

    def test_cross_at_multiple_angles(self):
        angle = 45
        rotated_cross = rotate_cross(self.base_cross, angle)

        with self.subTest(angle=angle):
            path = path_finding(
                cross=rotated_cross,
                egg=None,
                start=self.start,
                vip= None, #(random.uniform(margin, offset_width), random.uniform(margin, offset_height)) if random.choice([True, False]) else None,
                balls = [(80, 74), (80, 75), (80, 76), (80, 77)], #[(random.uniform(5, self.width - 5), random.uniform(5, self.height-5)) for _ in range(random.randint(0, 10))],
                end=self.end,
                wall_corners=self.wall_corners,
                robot_radius=self.robot_radius,
                width=self.width,
                height=self.height,
                debug=True,
            )

            self.assertIsInstance(path, list)

            obstacles = convert_cross_to_polygons(rotated_cross, 3)
            inflated_obstacles = [obs.buffer(self.robot_radius).simplify(1.5) for obs in obstacles]
            start_point = Point(self.start)
            end_point = Point(self.end)
            if any(obs.contains(start_point) or obs.contains(end_point) for obs in inflated_obstacles):
                self.assertEqual(len(path), 0, f"Path should be empty if start or end inside obstacle at angle {angle}")
            else:
                self.assertGreater(len(path), 0, f"Path should exist at angle {angle}")


if __name__ == "__main__":
    unittest.main()
