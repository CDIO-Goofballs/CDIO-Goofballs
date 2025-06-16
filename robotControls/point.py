import math

def calculate_turn(p1, p2, orientation: float):
    v = (p2.x - p1.x, p2.y - p1.y)
    target_angle = math.degrees(math.atan2(v[1], v[0]))
    turn_angle = target_angle - orientation
    return (turn_angle + 180) % 360 - 180 # Normalize turn_angle to the range -180 to 180

def calculate_distance(p1, p2):
    return math.sqrt((p2.x - p1.x)**2+(p2.y - p1.y)**2)