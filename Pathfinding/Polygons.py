import math

import numpy as np
from shapely import Point
from shapely.geometry import Polygon
from shapely.ops import unary_union

from Pathfinding.Point import MyPoint


def convert_cross_to_polygons(cross_points, arm_width=3):
    """
    Convert a cross defined by 4 points (top, bottom, right, left)
    into two rectangular polygons (vertical and horizontal arms),
    even if the cross is rotated.

    Parameters:
        cross_points: tuple of 4 points (top, bottom, right, left)
        arm_width: width of the cross arms
    Returns:
        List of two shapely Polygons (vertical arm, horizontal arm)
    """
    if cross_points is None or len(cross_points) != 4:
        return []

    top, bottom, right, left = cross_points
    top = np.array(top)
    bottom = np.array(bottom)
    left = np.array(left)
    right = np.array(right)

    def create_arm(p1, p2):
        """Create a rectangle between p1 and p2 with given width."""
        direction = p2 - p1
        length = np.linalg.norm(direction)
        if length == 0:
            return None
        direction = direction / length
        normal = np.array([-direction[1], direction[0]])  # Rotate 90 degrees
        offset = normal * (arm_width / 2)

        return Polygon([
            tuple(p1 + offset),
            tuple(p2 + offset),
            tuple(p2 - offset),
            tuple(p1 - offset)
        ])

    vertical = create_arm(top, bottom)
    horizontal = create_arm(left, right)

    polygons = []
    if vertical: polygons.append(vertical)
    if horizontal: polygons.append(horizontal)

    return polygons

def create_wall_polygon(p1, p2, thickness=1.5):
    """
    Create a thin rectangular polygon between two points, offset perpendicular to the direction.
    """
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return None

    # Unit perpendicular vector (normalized)
    nx, ny = -dy / length, dx / length
    offset_x, offset_y = nx * thickness / 2, ny * thickness / 2

    # Rectangle corners
    return Polygon([
        (p1[0] + offset_x, p1[1] + offset_y),
        (p2[0] + offset_x, p2[1] + offset_y),
        (p2[0] - offset_x, p2[1] - offset_y),
        (p1[0] - offset_x, p1[1] - offset_y)
    ])

def create_egg(egg, radius=4.5):
    return [Point(egg).buffer(radius).simplify(1.5)]

def create_boundary_walls_from_corners(wall_corners, thickness=1.5):
    """
    Given 4 corners: (top_left, bottom_left, bottom_right, top_right),
    returns 4 wall polygons forming the boundary.
    """
    top_left, bottom_left, bottom_right, top_right = wall_corners
    return [
        create_wall_polygon(top_left, top_right, thickness),     # Top wall
        create_wall_polygon(top_right, bottom_right, thickness), # Right wall
        create_wall_polygon(bottom_right, bottom_left, thickness), # Bottom wall
        create_wall_polygon(bottom_left, top_left, thickness)    # Left wall
    ]

def unit(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def inward_normal(p1, p2, center):
    edge = p2 - p1
    normal = np.array([-edge[1], edge[0]])  # Perpendicular vector
    normal = unit(normal)
    midpoint = (p1 + p2) / 2
    test_point = midpoint + normal
    if np.linalg.norm(test_point - center) < np.linalg.norm(midpoint - normal - center):
        return normal
    else:
        return -normal

def intersect_lines(p1, d1, p2, d2):
    """Find intersection point of two lines defined by p1 + t*d1 and p2 + s*d2"""
    A = np.column_stack((d1, -d2))
    b = p2 - p1
    t_s = np.linalg.solve(A, b)
    return p1 + t_s[0] * d1

def compute_inset_rectangle(corners, margin):
    corners = [np.array(pt) for pt in corners]
    center = sum(corners) / 4

    # Edges and their inward normals
    edges = [(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    normals = [inward_normal(a, b, center) for a, b in edges]

    # Shift each point inward
    shifted_edges = []
    for (a, b), n in zip(edges, normals):
        shifted_edges.append((a + margin * n, b + margin * n, b - a))

    # Find intersection of adjacent lines
    inset_corners = []
    for i in range(4):
        a1, b1, dir1 = shifted_edges[i]
        a2, b2, dir2 = shifted_edges[(i + 1) % 4]
        pt = intersect_lines(a1, dir1, a2, dir2)
        inset_corners.append(pt)

    return inset_corners

def generate_edge_points(corners, long_n=10, short_n=6):
    points = []
    edges = [(corners[i], corners[(i+1)%4]) for i in range(4)]
    lengths = [np.linalg.norm(b - a) for a, b in edges]
    long_sides = sorted(range(4), key=lambda i: lengths[i], reverse=True)[:2]

    for i, (a, b) in enumerate(edges):
        n = long_n if i in long_sides else short_n
        for j in range(n):
            t = j / (n - 1) if n > 1 else 0.5
            pt = a + t * (b - a)
            points.append(MyPoint(pt[0], pt[1]))
    return points

def generate_safe_points(course_corners, margin=34, long_points=10, short_points=6):
    inset_corners = compute_inset_rectangle(course_corners, margin)
    return generate_edge_points(inset_corners, long_n=long_points, short_n=short_points)
