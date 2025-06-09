import math
import unittest

import numpy as np
from shapely.geometry import Point, LineString, Polygon
from shapely.prepared import prep
import networkx as nx
from networkx.algorithms.approximation import traveling_salesman_problem
import itertools
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon, Circle
import random

def is_visible(p1, p2, obstacles):
    """
    Check if the line segment p1->p2 does not intersect any obstacle (except at endpoints).
    p1, p2: tuples (x, y)
    obstacles: list of shapely Polygons or PreparedGeometry
    """
    line = LineString([p1, p2])
    for obs in obstacles:
        if obs.crosses(line) or obs.contains(line):
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

def solve_tsp_approx(dist, start_idx=0, end_idx=None, must_visit_first=None):
    n = len(dist)
    G = nx.complete_graph(n)

    for i in range(n):
        for j in range(i + 1, n):
            G[i][j]['weight'] = dist[i][j]
            G[j][i]['weight'] = dist[j][i]

    try:
        tsp_path = traveling_salesman_problem(G, cycle=False, weight='weight', method=nx.approximation.christofides)
    except Exception as e:
        print("TSP solver failed:", e)
        return [], float('inf')

    if start_idx not in tsp_path or (end_idx is not None and end_idx not in tsp_path):
        return [], float('inf')

    # Reorder to start at start_idx
    start_pos = tsp_path.index(start_idx)
    tsp_path = tsp_path[start_pos:] + tsp_path[:start_pos]

    # Ensure it ends at end_idx
    if end_idx is not None:
        if tsp_path[-1] != end_idx:
            if end_idx in tsp_path:
                tsp_path.remove(end_idx)
            tsp_path.append(end_idx)

    # Handle must_visit_first (place immediately after start)
    if must_visit_first is not None and must_visit_first in tsp_path:
        tsp_path.remove(must_visit_first)
        tsp_path.insert(1, must_visit_first)

    # Sanity check again
    if len(tsp_path) < 2 or start_idx != tsp_path[0] or (end_idx is not None and tsp_path[-1] != end_idx):
        return [], float('inf')

    # Calculate total length
    total_length = sum(dist[tsp_path[i]][tsp_path[i + 1]] for i in range(len(tsp_path) - 1))

    # Remove start and end from internal path
    internal_path = tsp_path[1:-1] if end_idx is not None else tsp_path[1:]
    return internal_path, total_length

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

    # Check reachability from start (index 0)
    reachable = set()
    for i in points_indices:
        if i == 0 or nx.has_path(G, source=0, target=i):
            reachable.add(i)

    # Reconstruct the filtered points
    new_vip = vip if vip_idx in reachable else None
    filtered_others = [pt for i, pt in enumerate(others, start=2 if vip is not None else 1) if i in reachable]
    filtered_end = end if (len(points) - 1) in reachable else None

    if filtered_end is None:
        # If end is not reachable, no valid path possible
        return [], 0, [], vip is not None

    new_points = [start]
    if new_vip:
        new_points.append(new_vip)
    new_points += filtered_others
    new_points.append(filtered_end)

    G, all_nodes = build_visibility_graph(new_points, obstacles)
    points_indices = list(range(len(new_points)))
    dist, paths = compute_distance_matrix(G, all_nodes, points_indices)

    if new_vip:
        vip_idx = 1  # VIP index in points
        best_order, best_length = solve_tsp_approx(dist, start_idx=0, end_idx=len(points_indices) - 1,
                                                   must_visit_first=vip_idx)
    else:
        best_order, best_length = solve_tsp_approx(dist, start_idx=0, end_idx=len(points_indices) - 1)

    if best_order:
        full_path = reconstruct_full_path(paths, best_order)
        return best_order, best_length, full_path, new_vip is not None
    else:
        return [], 0, [], new_vip is not None

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

def plot_route(start, vip, others, end, obstacles, full_path, best_order, has_vip, width, height, ball_diameter=4, original_obstacles=None):
    plt.ion() # Interactive mode

    plt.clf()
    fig, ax = plt.gcf(), plt.gca()
    fig.set_size_inches(8, 6, True)
    radius = ball_diameter / 2

    if original_obstacles is None:
        original_obstacles = obstacles

    # Plot inflated obstacles with original ones inside
    for inflated_obs, original_obs in zip(obstacles, original_obstacles):
    # Inflated obstacle (outer)
        inflated_patch = MplPolygon(list(inflated_obs.exterior.coords), closed=True, facecolor='lightgray', edgecolor='gray', alpha=0.6)
        ax.add_patch(inflated_patch)
    # Original obstacle (inner)
        original_patch = MplPolygon(list(original_obs.exterior.coords), closed=True, facecolor='dimgray', edgecolor='black', alpha=1.0)
        ax.add_patch(original_patch)

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
    order_text = "Visit order: Start"
    offset = 2 if has_vip else 1
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
    ax.grid(True)

    # Draw the figure non-blocking
    fig.canvas.draw()
    fig.canvas.flush_events()

def extract_turn_points(path, angle_threshold_degrees=5):
    """
    Returns points in the path where the robot should turn (i.e., direction changes).
    Ignores small deviations below `angle_threshold_degrees`.
    """
    if len(path) < 3:
        return []  # Not enough points to form a turn

    turn_points = []
    threshold_rad = math.radians(angle_threshold_degrees)

    for i in range(1, len(path) - 1):
        p1 = np.array(path[i - 1])
        p2 = np.array(path[i])
        p3 = np.array(path[i + 1])

        v1 = p2 - p1
        v2 = p3 - p2

        if np.linalg.norm(v1) == 0 or np.linalg.norm(v2) == 0:
            continue  # Ignore degenerate segments

        # Normalize vectors
        v1 = v1 / np.linalg.norm(v1)
        v2 = v2 / np.linalg.norm(v2)

        angle = math.acos(np.clip(np.dot(v1, v2), -1.0, 1.0))
        if angle > threshold_rad:
            turn_points.append(tuple(p2))  # Add the turning point

    return turn_points

def path_finding(cross, start, vip, balls, end, wall_corners, robot_radius=2, width=160, height=120):
    if not balls:
        return []
    if cross:
        # Convert to two rectangular obstacles (vertical + horizontal arms)
        cross_obstacles = convert_cross_to_polygons(cross, 3)
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
        start = (100, 100) # TODO: Remove after goal position is used

    # Inflate obstacles
    inflated_obstacles = [obs.buffer(robot_radius).simplify(0.5) for obs in obstacles]

    # Prepare for fast visibility checks
    from shapely.prepared import prep
    prepared_obstacles = [prep(obs) for obs in inflated_obstacles]

    # Pass raw inflated to build_visibility_graph (needs .coords),
    # but prepared obstacles to is_visible (for performance)
    best_order, best_length, full_path, has_vip = plan_route_free_space(
        start, vip, balls, end, inflated_obstacles  # <- RAW inflated
    )

    plot_route(start, vip, balls, end, inflated_obstacles, full_path, best_order, has_vip, width=width, height=height, original_obstacles=obstacles)
    turn_points = extract_turn_points(full_path)
    return full_path, turn_points


def generate_random_cross(center_x, center_y, size=20):
    half = size / 2
    top = (center_x, center_y + half)
    bottom = (center_x, center_y - half)
    right = (center_x + half, center_y)
    left = (center_x - half, center_y)
    return (top, bottom, right, left)

class TestPathFinding(unittest.TestCase):
    def test_random_path_finding_multiple_runs(self):
        width, height = 160, 120
        wall_corners = ((0, 0), (0, height), (width, height), (width, 0))

        for _ in range(5):  # Run 5 times
            margin = 20
            offset_height = height - margin
            offset_width = width - margin
            cx = random.uniform(margin, offset_width)
            cy = random.uniform(margin, offset_height)
            cross = generate_random_cross(cx, cy, size=20)
            robot_radius = 2

            start = (random.uniform(margin, offset_width), random.uniform(margin, offset_height))
            end = (random.uniform(margin, offset_width), random.uniform(margin, offset_height))
            num_objects = random.randint(7, 10)
            objects = [(random.uniform(margin, offset_width), random.uniform(margin, offset_height)) for _ in range(num_objects)]
            vip = (random.uniform(margin, offset_width), random.uniform(margin, offset_height)) if random.choice([True, False]) else None

            path, turn_points = path_finding(cross, start, vip, objects, end, wall_corners, robot_radius=robot_radius, width=width, height=height)

            self.assertIsInstance(path, list)
            self.assertIsInstance(turn_points, list)

            obstacles = []
            obstacles += convert_cross_to_polygons(cross, 3)
            inflated_obstacles = [obs.buffer(robot_radius).simplify(0.5) for obs in obstacles]
            if any(Polygon(obs).contains(Point(start)) or Polygon(obs).contains(Point(end)) for obs in inflated_obstacles):
                self.assertEqual(len(path), 0, "There should be no path if start or end is inside an obstacle")
            else:
                self.assertGreater(len(path), 0, "Path should not be empty")


if __name__ == "__main__":
    unittest.main()
