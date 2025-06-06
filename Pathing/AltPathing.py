import math

import numpy as np
from shapely.geometry import Point, LineString, Polygon
import networkx as nx
import itertools
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon, Circle
import random

def is_visible(p1, p2, obstacles):
    """
    Check if the line segment p1->p2 does not intersect any obstacle (except at endpoints).
    p1, p2: tuples (x, y)
    obstacles: list of shapely Polygons
    """
    line = LineString([p1, p2])
    for obs in obstacles:
        if line.crosses(obs) or line.within(obs):
            return False
    return True

def build_visibility_graph(points, obstacles):
    """
    Build visibility graph including:
    - all points (start, vip, others, end)
    - all obstacle vertices
    """
    G = nx.Graph()
    # Add points
    all_nodes = list(points)

    # Add obstacle vertices
    for obs in obstacles:
        all_nodes.extend(list(obs.exterior.coords)[:-1])  # skip repeated last vertex

    # Add nodes to graph
    for i, node in enumerate(all_nodes):
        G.add_node(i, coord=node)

    # Connect visible nodes
    n = len(all_nodes)
    for i in range(n):
        for j in range(i+1, n):
            p1 = all_nodes[i]
            p2 = all_nodes[j]
            if is_visible(p1, p2, obstacles):
                dist = Point(p1).distance(Point(p2))
                G.add_edge(i, j, weight=dist)

    return G, all_nodes

def shortest_path_on_visibility_graph(G, all_nodes, start_idx, end_idx):
    """
    Returns shortest path coordinates from start_idx to end_idx on graph G
    """
    path_nodes = nx.shortest_path(G, source=start_idx, target=end_idx, weight='weight')
    path_coords = [all_nodes[i] for i in path_nodes]
    return path_coords

def compute_distance_matrix(G, all_nodes, points_indices):
    """
    points_indices: indices of points in all_nodes list for start, vip, others, end
    Returns dist matrix and paths between those points
    """
    n = len(points_indices)
    dist = [[float('inf')] * n for _ in range(n)]
    paths = [[None]*n for _ in range(n)]

    for i in range(n):
        for j in range(n):
            if i != j:
                try:
                    pth = nx.shortest_path(G, source=points_indices[i], target=points_indices[j], weight='weight')
                    dist[i][j] = nx.shortest_path_length(G, source=points_indices[i], target=points_indices[j], weight='weight')
                    paths[i][j] = [all_nodes[idx] for idx in pth]
                except:
                    print(f"Path not found between {points_indices[i]} and {points_indices[j]}")

    return dist, paths

def solve_vip_tsp_route(dist):
    n = len(dist)
    others_indices = list(range(2, n-1))  # indices of other balls

    best_order = None
    best_length = float('inf')

    for perm in itertools.permutations(others_indices):
        order = [1] + list(perm)  # include vip at the beginning
        length = dist[0][1]  # start -> vip
        length += dist[1][perm[0]]  # vip -> first other
        for i in range(len(perm)-1):
            length += dist[perm[i]][perm[i+1]]
        length += dist[perm[-1]][n-1]  # last other -> end

        if length < best_length:
            best_length = length
            best_order = order

    return best_order, best_length

def solve_tsp_no_vip(dist):
    n = len(dist)
    others_indices = list(range(1, n-1))  # Exclude start and end

    best_order = None
    best_length = float('inf')

    for perm in itertools.permutations(others_indices):
        length = dist[0][perm[0]]  # start -> first other
        for i in range(len(perm)-1):
            length += dist[perm[i]][perm[i+1]]
        length += dist[perm[-1]][n-1]  # last other -> end

        if length < best_length:
            best_length = length
            best_order = perm

    return best_order, best_length

def reconstruct_full_path(paths, best_order):
    n = len(paths)
    full_path = []
    full_path.extend(paths[0][best_order[0]])
    for i in range(len(best_order) - 1):
        full_path.extend(paths[best_order[i]][best_order[i+1]][1:])
    full_path.extend(paths[best_order[-1]][n-1][1:])
    return full_path


def plan_route_free_space(start, vip, others, end, obstacles):
    points = [start]
    vip_idx = None

    if vip is not None:
        points.append(vip)
        vip_idx = 1

    points += others
    points.append(end)

    G, all_nodes = build_visibility_graph(points, obstacles)
    points_indices = list(range(len(points)))
    dist, paths = compute_distance_matrix(G, all_nodes, points_indices)

    # Adjust TSP based on whether VIP is present
    if vip is not None:
        best_order, best_length = solve_vip_tsp_route(dist)
    else:
        best_order, best_length = solve_tsp_no_vip(dist)

    if best_order:
        full_path = reconstruct_full_path(paths, best_order)
        return best_order, best_length, full_path, vip is not None
    else:
        return [], 0, [], vip is not None


def convert_cross_to_polygons(cross_points, arm_width=4):
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

def plot_route(start, vip, others, end, obstacles, full_path, best_order, has_vip, width, height, ball_diameter=4):
    fig, ax = plt.subplots(figsize=(10,8))
    radius = ball_diameter / 2

    # Obstacles
    for obs in obstacles:
        patch = MplPolygon(list(obs.exterior.coords), closed=True, facecolor='lightgray', edgecolor='black')
        ax.add_patch(patch)

    ax.add_patch(Circle(start, radius, color='green', label='Start'))
    if vip:
        ax.add_patch(Circle(vip, radius, color='magenta', label='VIP'))

    for i, pt in enumerate(others):
        ax.add_patch(Circle(pt, radius, color='blue', label='Other Balls' if i == 0 else None))
        ax.text(pt[0]+radius, pt[1]+radius, f'O{i}', color='blue')

    ax.add_patch(Circle(end, radius, color='red', label='End'))

    # Path
    if full_path:
        xs, ys = zip(*full_path)
        ax.plot(xs, ys, 'r-', linewidth=2, label='Planned path')

    # Title
    if has_vip:
        order_text = "Visit order: Start"
        offset = 2
    else:
        order_text = "Visit order: Start"
        offset = 1

    for idx in best_order:
        if has_vip and idx == 1:
            order_text += " → VIP"
        else:
            order_text += f" → O{idx - offset}"
    order_text += " → End"

    ax.set_title(order_text)
    ax.legend()
    ax.set_aspect('equal')
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    plt.grid(True)
    plt.show()


def path_finding(cross, start, vip, balls, end, wall_corners, width=160, height=120):
    if not balls:
        return []
    if cross:
        # Convert to two rectangular obstacles (vertical + horizontal arms)
        cross_obstacles = convert_cross_to_polygons(cross, 2)
    else:
        cross_obstacles = []

    if wall_corners:
        # Build 4 boundary walls using wall_corners
        boundary_walls = create_boundary_walls_from_corners(wall_corners, thickness=1.5)
    else:
        boundary_walls = []

    # Combine all obstacles
    obstacles = cross_obstacles + boundary_walls

    if not start:
        start = (200, 200) # TODO remove later

    # Plan and plot
    best_order, best_length, full_path, has_vip = plan_route_free_space(start, vip, balls, end, obstacles)
    plot_route(start, vip, balls, end, obstacles, full_path, best_order, has_vip, width=width, height=height)
    return full_path

def generate_random_cross(center_x, center_y, size=15):
    half = size / 2
    top = (center_x, center_y + half)
    bottom = (center_x, center_y - half)
    right = (center_x + half, center_y)
    left = (center_x - half, center_y)
    return (top, bottom, right, left)

def test_random_path_finding():
    width, height = 160, 120

    wall_corners = ( (0,0), (0,height), (width,height), (width,0) )

    # Generate random center for cross within bounds
    margin = 20
    cx = random.uniform(margin, width - margin)
    cy = random.uniform(margin, height - margin)
    cross = generate_random_cross(cx, cy, size=30)

    # Random start and end points
    start = (random.uniform(0, width), random.uniform(0, height))
    end = (random.uniform(0, width), random.uniform(0, height))

    # Random number of objects (0 to 10)
    num_objects = random.randint(0, 10)
    objects = [(random.uniform(0, width), random.uniform(0, height)) for _ in range(num_objects)]

    # Randomly include a VIP (50% chance)
    vip = (random.uniform(0, width), random.uniform(0, height)) if random.choice([True, False]) else None

    print(f"Start: {start}")
    print(f"VIP: {vip}")
    print(f"Objects: {objects}")
    print(f"End: {end}")
    print(f"Cross center: ({cx}, {cy})")

    # Call the main function
    path_finding(cross, start, vip, objects, end, wall_corners)

if __name__ == "__main__":
    # Run the test function to generate random path finding scenario
    test_random_path_finding()
