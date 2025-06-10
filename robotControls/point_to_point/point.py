import math

def calculate_turn(p1, p2, orientation: float):
    v = (p2[0] - p1[0], p2[1] - p1[1])
    target_angle = math.degrees(math.atan2(v[1], v[0]))
    turn_angle = target_angle - orientation
    return (turn_angle + 180) % 360 - 180 # Normalize turn_angle to the range -180 to 180

def calculate_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2+(p2[1] - p1[1])**2)*10 - 270 # subtract robot length