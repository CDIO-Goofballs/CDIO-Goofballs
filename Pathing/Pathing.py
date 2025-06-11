import math
import numpy as np
from shapely.geometry import LineString
import networkx as nx
from networkx.algorithms.approximation import traveling_salesman_problem
from shapely.geometry import Point
from Plotting import plot_route
from Polygons import convert_cross_to_polygons, create_egg, create_boundary_walls_from_corners, generate_safe_points

def find_nearest_free_point(point, obstacles, search_radius=12, step=1):
    """
    Given a point inside an obstacle, return the nearest visible point outside all obstacles.
    """
    for r in np.arange(step, search_radius, step):
        # Sample points around the circle
        for angle in np.linspace(0, 2 * math.pi, int(2 * math.pi * r / step)):
            dx, dy = r * math.cos(angle), r * math.sin(angle)
            candidate = Point(point[0] + dx, point[1] + dy)
            if not any(obs.contains(candidate) for obs in obstacles):
                return (candidate.x, candidate.y)
    return None

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

    # Start to first ball
    subpath = paths[0][best_order[0]]
    for i, point in enumerate(subpath):
        if i == 0:
            label = 'start'
        elif i == len(subpath) - 1:
            label = 'ball'
        else:
            label = 'turn_point'
        full_path.append((*point, label))

    # Intermediate balls
    for i in range(len(best_order) - 1):
        subpath = paths[best_order[i]][best_order[i + 1]]
        for j, point in enumerate(subpath[1:]):
            if j == len(subpath) - 2:
                label = 'ball'
            else:
                label = 'turn_point'
            full_path.append((*point, label))

    # Last ball to end
    subpath = paths[best_order[-1]][n - 1]
    for k, point in enumerate(subpath[1:]):
        if k == len(subpath) - 2:
            label = 'end'
        else:
            label = 'turn_point'
        full_path.append((*point, label))
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
    #filtered_others = []
    #ball_start_idx = 2 if vip is not None else 1
    #for i, pt in enumerate(others):
    #    idx = i + ball_start_idx
    #    if idx in reachable:
    #        filtered_others.append(pt)
    #    else:
    #        # Try to find nearest reachable point outside inflated obstacles
    #        replacement = find_nearest_free_point(pt, obstacles)
    #        if replacement:
    #            filtered_others.append(replacement)

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

def path_finding(
        cross, egg, start, vip, balls, end, wall_corners, robot_radius=10.5, width=160, height=120, debug=False):

    if not balls:
        return []

    cross_obstacles = convert_cross_to_polygons(cross, 3) if cross else []
    boundary_walls = create_boundary_walls_from_corners(wall_corners, thickness=1.5) if wall_corners else []
    egg_obstacle = create_egg(egg) if egg else []

    # Combine all obstacles
    obstacles = cross_obstacles + egg_obstacle + boundary_walls

    if not start:
        start = (100, 100) # TODO: Remove after goal position is used

    # Inflate obstacles
    inflated_obstacles = [obs.buffer(robot_radius).simplify(1.5) for obs in obstacles]

    safe_points = generate_safe_points(wall_corners, margin=30, long_points=5, short_points=3) if wall_corners else []
    # Remove safe points that are too close to any obstacle
    min_clearance = 29
    safe_points = [
        pt for pt in safe_points
        if all(Point(pt).distance(obs) >= min_clearance for obs in obstacles)
    ]

    best_order, best_length, full_path, has_vip = plan_route_free_space(
        start, vip, balls, end, inflated_obstacles + boundary_walls
    )

    plot_route(start, vip, balls, end, inflated_obstacles, safe_points, full_path, best_order, has_vip, width=width, height=height, original_obstacles=obstacles, debug=debug)
    return full_path
