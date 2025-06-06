import unittest
from point import Point
import point 

class TestPoint(unittest.TestCase):

    def test_distance_calculation(self):
        p1 = Point(0, 0) 
        p2 = Point(0, 200) 
        self.assertEqual(point.calculate_distance(p1, p2), 200)

    def test_zero_distance_calculation(self):
        p1 = Point(0, 0) 
        p2 = Point(0, 0)
        self.assertEqual(point.calculate_distance(p1, p2), 0)

    def test_angle_calculation(self):
        p1 = Point(0, 0) 
        p2 = Point(200, 0)
        orientation = 0
        self.assertEqual(point.calculate_turn(p1, p2, orientation), 0)

    def test_angle_antiparallel_calculation(self):
        p1 = Point(0, 0) 
        p2 = Point(-200, 0)
        orientation = 0
        self.assertEqual(point.calculate_turn(p1, p2, orientation), -180)


    def test_turn_calculation_on_oblique_path(self):
        positions = [
            Point(0,0),
            Point(100,100),
            Point(160,0),
            Point(160,-120),
            Point(40,100),
            Point(0,0)
        ]
        self.assertAlmostEqual(point.calculate_turn(positions[0], positions[1], 0), 45)
        self.assertAlmostEqual(point.calculate_turn(positions[1], positions[2], 45), -104.04, 1)
        self.assertAlmostEqual(point.calculate_turn(positions[2], positions[3], -104.04+45), -30.96, 1)
        self.assertAlmostEqual(point.calculate_turn(positions[3], positions[4], -30.96-104.04+45), -151.39, 1)
        
        
if __name__ == '__main__':
    unittest.main()