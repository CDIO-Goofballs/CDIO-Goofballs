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