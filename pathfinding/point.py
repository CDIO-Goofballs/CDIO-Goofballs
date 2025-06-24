import math

import numpy as np


class MyPoint:
    def __init__(self, x, y, type=None, target=None):
        self.x = x
        self.y = y
        self.type = type
        self.target = target

    def __sub__(self, other):
        return np.array([self.x, self.y]) - np.array([other.x, other.y])

    def __repr__(self):
        return f"MyPoint({self.x}, {self.y}, type={self.type})"

    def to_tuple(self):
        return self.x, self.y

def calculate_turn(p1, p2, orientation: float):
    v = (p2.x - p1.x, p2.y - p1.y)
    target_angle = math.degrees(math.atan2(v[1], v[0]))
    turn_angle = target_angle - orientation
    return (turn_angle + 180) % 360 - 180 # Normalize turn_angle to the range -180 to 180

def calculate_distance(p1, p2):
    return math.sqrt((p2.x - p1.x)**2+(p2.y - p1.y)**2)