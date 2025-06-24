import random
import unittest

from shapely.geometry import Point, Polygon

from pathing import path_finding
from polygons import convert_cross_to_polygons, create_egg


def generate_random_cross(center_x, center_y, size=20):
    half = size / 2
    top = (center_x, center_y + half)
    bottom = (center_x, center_y - half)
    right = (center_x + half, center_y)
    left = (center_x - half, center_y)
    return top, bottom, right, left

class TestPathFinding(unittest.TestCase):
    def test_random_path_finding_multiple_runs(self):
        width, height = 160, 120
        wall_corners = ((0, 0), (0, height), (width, height), (width, 0))

        for i in range(5):  # Run 5 times
            margin = 20
            offset_height = height - margin
            offset_width = width - margin
            cx = random.uniform(width/2 - 10, width/2 + 10)
            cy = random.uniform(height/2 - 10, height/2 + 10)
            cross = generate_random_cross(cx, cy, size=20)
            robot_radius = 17

            start = (random.uniform(margin, offset_width), random.uniform(margin, offset_height))
            end = (random.uniform(margin, offset_width), random.uniform(margin, offset_height))
            num_objects = random.randint(0, 10)
            balls = [(random.uniform(5, width - 5), random.uniform(5, height-5)) for _ in range(num_objects)]
            vip = (random.uniform(margin, offset_width), random.uniform(margin, offset_height)) if random.choice([True, False]) else None
            egg = None

            path = path_finding(cross=cross, egg=egg, start=start, vip=vip, balls=balls, end=end,
                                wall_corners=wall_corners, robot_radius=robot_radius, width=width, height=height, debug=True)

            self.assertIsInstance(path, list)

            obstacles = []
            obstacles += convert_cross_to_polygons(cross, 3)
            obstacles += create_egg(egg, 4.5) if egg else []
            inflated_obstacles = [obs.buffer(robot_radius).simplify(1.5) for obs in obstacles]
            if any(Polygon(obs).contains(Point(start)) or Polygon(obs).contains(Point(end)) for obs in inflated_obstacles):
                self.assertEqual(len(path), 0, "There should be no path if start or end is inside an obstacle")
            else:
                self.assertGreater(len(path), 0, "Path should not be empty")

if __name__ == "__main__":
    unittest.main()