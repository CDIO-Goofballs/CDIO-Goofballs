import math
import traceback

import networkx as nx
import numpy as np
from networkx.algorithms.approximation import traveling_salesman_problem
from shapely.geometry import LineString, Point

from pathfinding.plotting import plot_route
from pathfinding.point import MyPoint, calculate_distance
from pathfinding.polygons import convert_cross_to_polygons, create_egg, create_boundary_walls_from_corners, \
    generate_safe_points

safe_points = []

def get_safe_points():
    """
    Returns the list of safe points.
    """
    global safe_points
    return safe_points

def get_end_safe_point(end, width):
    safe = safe_points.copy()
    if end.x < width / 2:
        safe.sort(key=lambda pt: pt.x + pt.y)
        return MyPoint(safe[0].x, safe[0].y, type='safe', target=end)
        # Find upper safe point
    else:
        safe.sort(key=lambda pt: - pt.x - pt.y)
        return MyPoint(safe[0].x, safe[0].y, type='safe', target=end)
        # Find lower safe point


def find_nearest_free_point(point, obstacles, search_radius=24, step=1):
    """
    Given a point inside an obstacle, return the nearest visible point outside all obstacles.
    """
    for r in np.arange(step, search_radius, step):
        # Sample points around the circle
        for angle in np.linspace(0, 2 * math.pi, int(2 * math.pi * r / step)):
            dx, dy = r * math.cos(angle), r * math.sin(angle)
            candidate = Point(point.x + dx, point.y + dy)
            if not any(obs.contains(candidate) for obs in obstacles):
                return MyPoint(candidate.x, candidate.y)
    return None


def find_aligned_safe_point(ball, inflated_obstacles, original_obstacles, width, height, min_distance=22):
    """
    Step 1: Find the closest free point to the ball.
    Step 2: Push that point.
    Step 3: Find the closest safe point to the pushed point.
    Returns the nearest safe point.
    """


    def point_inline(stuck_ball, free_point, t):
        """
        Returns a point along the line from ball to free_point.
        t = 0   → ball
        t = 1   → free_point
        t < 1   → between ball and free point
        t > 1   → beyond free point
        """
        dx = free_point.x - stuck_ball.x
        dy = free_point.y - stuck_ball.y
        return MyPoint(stuck_ball.x + dx * t, stuck_ball.y + dy * t)

    def distance(a, b):
        return np.linalg.norm(np.array([a.x, a.y]) - np.array([b.x, b.y]))

    def angle_between_vectors(a, b):
        """Compute angle (in radians) between two 2D vectors."""
        dot = a[0] * b[0] + a[1] * b[1]
        norm_a = math.hypot(*a)
        norm_b = math.hypot(*b)
        if norm_a == 0 or norm_b == 0:
            return 0
        cos_theta = max(-1, min(1, dot / (norm_a * norm_b)))
        return math.acos(cos_theta)  # radians

    # Step 1: Find closest free point to ball
    closest_free = find_nearest_free_point(ball, inflated_obstacles)

    if closest_free is None:
        return find_nearest_safe_point(ball, original_obstacles)

    # Step 2: Push the point along the ball→closest_free vector
    pushed_point = point_inline(ball,closest_free, t=2)

    # Step 3: Find the closest safe point to the pushed point
    nearest_safe = None
    nearest_dist = float('inf')
    max_allowed_angle = 30  # degrees

    # Vector A: from ball to closest_free (push direction)
    vec_push = (closest_free.x - ball.x, closest_free.y - ball.y)

    dist_from_ball_to_center = distance(ball, MyPoint(width / 2, height / 2))

    for pt in safe_points:
        if not is_visible((ball.x, ball.y), (pt.x, pt.y), original_obstacles):
            continue

        # Vector B: from ball to current safe point
        vec_safe = (pt.x - ball.x, pt.y - ball.y)

        # Calculate angle between push direction and candidate safe direction
        angle_rad = angle_between_vectors(vec_push, vec_safe)
        angle_deg = math.degrees(angle_rad)

        if (angle_deg > max_allowed_angle and calculate_distance(closest_free, ball) > 14
                and dist_from_ball_to_center < 30):
            continue

        pushed_to_safe = distance(pushed_point, pt)
        ball_to_safe = distance(ball, pt)

        if min_distance <= ball_to_safe and pushed_to_safe < nearest_dist:
            nearest_dist = pushed_to_safe
            nearest_safe = pt

    if nearest_safe is None:
        vec = np.array([pushed_point.x - ball.x, pushed_point.y - ball.y])
        length = np.linalg.norm(vec)
        if length == 0:
            return None

        norm = (vec / length) * 25
        new_x = ball.x + norm[0]
        new_y = ball.y + norm[1]
        return MyPoint(new_x, new_y, type='safeV3', target=ball)

    # Wrap result
    aligned_pt = None if nearest_safe is None else MyPoint(nearest_safe.x, nearest_safe.y, type='safeV1', target=ball)

    return aligned_pt


def find_nearest_safe_point(point, obstacles, min_distance = 25):
    """
    Given a point inside an obstacle, return the nearest safe point outside
    all obstacles and at a distance of minimum length corresponding to robot length.
    """
    shortest_distance = float('inf')
    candidate_x = 0
    candidate_y = 0

    for safe_pt in safe_points:
        distance = np.linalg.norm(np.array([point.x, point.y]) - np.array([safe_pt.x, safe_pt.y]))
        if (is_visible((point.x, point.y), (safe_pt.x, safe_pt.y), obstacles) and
                shortest_distance > distance >= min_distance):
            candidate_x = safe_pt.x
            candidate_y = safe_pt.y
            shortest_distance = distance

    if shortest_distance == float('inf'):
        return None

    return MyPoint(candidate_x, candidate_y, type='safeV2', target=point)

def is_visible(p1, p2, obstacles, p1_obj=None, p2_obj=None):
    """
    p1, p2: tuples (x, y)
    p1_obj, p2_obj: optionally MyPoint instances to check ignore_buffer flag
    """
    line = LineString([p1, p2])
    for obs in obstacles:
        if obs.crosses(line) or obs.contains(line):
            return False
    return True

def build_visibility_graph(points, obstacles):
    """
    Build visibility graph from all points and obstacle vertices.
    """
    G = nx.Graph()
    all_nodes = list((p.x, p.y) for p in points)

    all_nodes.extend(list((p.x, p.y) for p in safe_points))

    for i, node in enumerate(all_nodes):
        G.add_node(i, coord=node)

    n = len(all_nodes)
    for i in range(n):
        for j in range(i + 1, n):
            p1, p2 = all_nodes[i], all_nodes[j]
            if is_visible(p1, p2, obstacles):
                dist = Point(p1).distance(Point(p2))
                G.add_edge(i, j, weight=dist)

    return G, all_nodes

def shortest_path_on_visibility_graph(G, all_nodes, start_idx, end_idx):
    """
    Compute shortest path between nodes on the visibility graph.
    """
    path_nodes = nx.shortest_path(G, source=start_idx, target=end_idx, weight='weight')
    path_coords = [all_nodes[i] for i in path_nodes]
    return path_coords

def compute_distance_matrix(G, all_nodes, points_indices):
    """
    Compute shortest distances and paths between key points.
    """
    n = len(points_indices)
    dist = [[float('inf')] * n for _ in range(n)]
    paths = [[None] * n for _ in range(n)]

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
    """
    Solve TSP approximately and return best order.
    """
    n = len(dist)
    G = nx.complete_graph(n)

    if end_idx is None:
        return [], float('inf')

    if n == 2:
        return [1], dist[0][1]

    for i in range(n):
        for j in range(i + 1, n):
            G[i][j]['weight'] = dist[i][j]
            G[j][i]['weight'] = dist[j][i]

    try:
        tsp_path = traveling_salesman_problem(G, cycle=False, weight='weight', method=nx.approximation.greedy_tsp)
    except Exception as e:
        print("TSP solver failed:", e)
        return [], float('inf')

    if start_idx not in tsp_path or (end_idx is not None and end_idx not in tsp_path):
        return [], float('inf')

    start_pos = tsp_path.index(start_idx)
    tsp_path = tsp_path[start_pos:] + tsp_path[:start_pos]

    if end_idx is not None:
        if tsp_path[-1] != end_idx:
            if end_idx in tsp_path:
                tsp_path.remove(end_idx)
            tsp_path.append(end_idx)

    if must_visit_first is not None and must_visit_first in tsp_path:
        tsp_path.remove(must_visit_first)
        tsp_path.insert(1, must_visit_first)

    if len(tsp_path) < 2 or start_idx != tsp_path[0] or (end_idx is not None and tsp_path[-1] != end_idx):
        return [], float('inf')

    total_length = sum(dist[tsp_path[i]][tsp_path[i + 1]] for i in range(len(tsp_path) - 1))
    internal_path = tsp_path[1:-1] if end_idx is not None else tsp_path[1:]
    return internal_path, total_length

def reconstruct_full_path(paths, best_order, points):
    """
    Reconstruct full path with types and all point metadata.

    points: list of MyPoint with correct types and properties
    """
    n = len(paths)
    full_path = []

    if n == 2:
        start_idx = 0
        end_idx = 1
        subpath = paths[start_idx][end_idx]

        for i, (x, y) in enumerate(subpath):
            if i == 0:
                pt = points[start_idx]
            elif i == len(subpath) - 1:
                pt = points[end_idx]
            else:
                pt = MyPoint(x, y, type='turn')
            full_path.append(MyPoint(x, y, type=pt.type, target=getattr(pt, 'target', None)))

        return full_path

    # From start to first visited point
    start_idx = 0
    first_target_idx = best_order[0]
    subpath = paths[start_idx][first_target_idx]

    for i, (x, y) in enumerate(subpath):
        if i == 0:
            # Start point — use original metadata
            pt = points[start_idx]
        elif i == len(subpath) - 1:
            pt = points[first_target_idx]
        else:
            pt = MyPoint(x, y, type='turn')
        full_path.append(MyPoint(x, y, type=pt.type, target=getattr(pt, 'target', None)))
    # Intermediate legs
    for i in range(len(best_order) - 1):
        start_idx = best_order[i]
        end_idx = best_order[i + 1]
        subpath = paths[start_idx][end_idx]

        for j, (x, y) in enumerate(subpath[1:]):  # Skip first to avoid duplicate
            if j == len(subpath[1:]) - 1:
                pt = points[end_idx]
            else:
                pt = MyPoint(x, y, type='turn')
            full_path.append(MyPoint(x, y, type=pt.type, target=getattr(pt, 'target', None)))

    # From last visited to end
    last_idx = best_order[-1]
    end_idx = n - 1
    subpath = paths[last_idx][end_idx]


    for i, (x, y) in enumerate(subpath[1:]):  # Skip first to avoid duplicate
        pt = points[end_idx] if i == len(subpath[1:]) - 1 else MyPoint(x, y, type='turn')
        full_path.append(MyPoint(x, y, type=pt.type, target=getattr(pt, 'target', None)))

    return full_path

def plan_route_free_space(start, vip, others, end, inflated_obstacles, original_obstacles,
                          width, height):
    """
    Plan route avoiding obstacles with VIP and balls.
    """
    points = [start]
    vip_idx = None

    if vip is not None:
        points.append(vip)
        vip_idx = 1

    if others:
        points += others

    points.append(end)

    G, all_nodes = build_visibility_graph(points, inflated_obstacles)
    points_indices = list(range(len(points)))

    reachable = set()
    for i in points_indices:
        if i == 0 or nx.has_path(G, source=0, target=i):
            reachable.add(i)

    if vip:
        if vip_idx in reachable:
            vip_replacement = vip
        else:
            replacement = find_aligned_safe_point(vip, inflated_obstacles, original_obstacles, width, height)
            if replacement:
                vip_replacement = replacement
            else:
                vip_replacement = None
                print("Replacement vip not found")
    else:
        vip_replacement = None

    ball_replacements = []
    ball_start_idx = 2 if vip is not None else 1
    for i, pt in enumerate(others):
        idx = i + ball_start_idx
        if idx in reachable:
            ball_replacements.append(pt)
        else:
            replacement = find_aligned_safe_point(pt, inflated_obstacles, original_obstacles, width, height)
            if replacement:
                ball_replacements.append(replacement)
            else:
                replacement = find_nearest_safe_point(pt, inflated_obstacles, original_obstacles)
                if replacement:
                    ball_replacements.append(replacement)
                    print("No good replacement found")
                else:
                    print(f"Replacement for ball {i} not found, skipping")


    filtered_end = get_end_safe_point(end, width)
    if filtered_end is None:
        return [], 0, [], vip is not None

    replacement_points = [start]
    if vip_replacement:
        replacement_points.append(vip_replacement)
    replacement_points += ball_replacements
    replacement_points.append(filtered_end)

    G, all_nodes = build_visibility_graph(replacement_points, inflated_obstacles)
    replacements_indices = list(range(len(replacement_points)))

    new_reachable = set()
    for i in replacements_indices:
        if i == 0 or nx.has_path(G, source=0, target=i):
            new_reachable.add(i)

    filtered_balls = []
    for i, pt in enumerate(ball_replacements):
        idx = i + ball_start_idx
        if idx in new_reachable:
            filtered_balls.append(pt)

    if vip_idx in new_reachable:
        new_vip = vip_replacement
    else:
        new_vip = None

    print(f"Filtered balls: {filtered_balls}")

    new_points = [start]
    if new_vip:
        new_points.append(new_vip)
    new_points += filtered_balls
    new_points.append(filtered_end)

    G, all_nodes = build_visibility_graph(new_points, inflated_obstacles)
    points_indices = list(range(len(new_points)))
    dist, paths = compute_distance_matrix(G, all_nodes, points_indices)

    if vip_replacement:
        vip_idx = 1
        best_order, best_length = solve_tsp_approx(dist, start_idx=0, end_idx=len(points_indices) - 1,
                                                   must_visit_first=vip_idx)
    else:
        best_order, best_length = solve_tsp_approx(dist, start_idx=0, end_idx=len(points_indices) - 1)

    try:
        if best_order and best_length < float('inf'):
            full_path = reconstruct_full_path(paths, best_order, new_points)
            return best_order, best_length, full_path, vip_replacement is not None
        else:
            return [], 0, [], vip_replacement is not None
    except Exception as e:
        print(e)
        traceback.print_exc()
        return [], 0, [], vip_replacement is not None


def path_finding(
        cross, egg, start, vip, balls, end, wall_corners, robot_radius=17,
        width=160, height=120, debug=False, start_angle=0):
    """
    Main interface to compute full route.
    """
    global safe_points

    cross_obstacles = convert_cross_to_polygons(cross) if cross else []
    boundary_walls = create_boundary_walls_from_corners(wall_corners, thickness=1.5) if wall_corners else []
    egg_obstacle = create_egg(egg) if egg else []

    obstacles = cross_obstacles + egg_obstacle + boundary_walls

    if not start:
        start = (60, 60)

    start = MyPoint(start[0], start[1], type='start')

    vip = MyPoint(*vip, type='vip') if vip else None
    balls = [MyPoint(*ball, type='ball') for ball in balls]
    end = MyPoint(*end, type='end')

    inflated_obstacles = [obs.buffer(robot_radius).simplify(1.5) for obs in obstacles]

    safe_points = generate_safe_points(wall_corners) if wall_corners else []

    min_clearance = 25
    safe_points = [
        pt for pt in safe_points
        if all(Point(pt.x, pt.y).distance(obs) >= min_clearance for obs in obstacles)
    ]

    best_order, best_length, full_path, has_vip = plan_route_free_space(
        start, vip, balls, end, inflated_obstacles, obstacles, width=width, height=height
    )

    plot_route(start, vip, balls, end, inflated_obstacles, safe_points, full_path,
               best_order, has_vip, width=width, height=height, original_obstacles=obstacles,
               debug=debug, start_angle=start_angle)

    return full_path