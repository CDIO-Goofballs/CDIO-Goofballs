import math

class Point:
    x = 0
    y = 0
    def __init__(self, x, y):
        self.x = x
        self.y = y

def calculate_turn(p1: Point, p2: Point, orientation: float):
    v = Point(p2.x - p1.x, p2.y - p1.y)
    target_angle = math.degrees(math.atan2(v.y, v.x))
    turn_angle = target_angle - orientation
    return (turn_angle + 180) % 360 - 180 # Normalize turn_angle to the range -180 to 180

def calculate_distance(p1: Point, p2: Point):
    return math.sqrt((p2.x - p1.x)**2+(p2.y - p1.y)**2)