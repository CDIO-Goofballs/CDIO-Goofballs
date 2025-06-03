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
                pth = nx.shortest_path(G, source=points_indices[i], target=points_indices[j], weight='weight')
                dist[i][j] = nx.shortest_path_length(G, source=points_indices[i], target=points_indices[j], weight='weight')
                paths[i][j] = [all_nodes[idx] for idx in pth]

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

    full_path = reconstruct_full_path(paths, best_order)
    return best_order, best_length, full_path, vip is not None


def convert_cross_to_polygons(cross_points, arm_width=4):
    """
    Convert a cross defined by 4 points (top, bottom, right, left)
    into two rectangular polygons: vertical and horizontal arms.

    cross_points: (top, bottom, right, left)
    arm_width: width of the cross arms (default 4)
    """
    if cross_points is None or len(cross_points) != 4:
        return []

    top, bottom, right, left = cross_points
    cx = (left[0] + right[0]) / 2
    cy = (top[1] + bottom[1]) / 2

    # Vertical arm polygon
    vertical = Polygon([
        (cx - arm_width/2, top[1]),
        (cx + arm_width/2, top[1]),
        (cx + arm_width/2, bottom[1]),
        (cx - arm_width/2, bottom[1])
    ])

    # Horizontal arm polygon
    horizontal = Polygon([
        (left[0], cy - arm_width/2),
        (right[0], cy - arm_width/2),
        (right[0], cy + arm_width/2),
        (left[0], cy + arm_width/2)
    ])

    return [vertical, horizontal]

def plot_route(start, vip, others, end, obstacles, full_path, best_order, has_vip, ball_diameter=4):
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
    ax.set_xlim(0, 160)
    ax.set_ylim(0, 120)
    plt.grid(True)
    plt.show()


def path_finding(cross, start, vip, balls, end):
    # Convert to two rectangular obstacles (vertical + horizontal arms)
    cross_obstacles = convert_cross_to_polygons(cross, 2)

    # Add to main obstacle list
    obstacles = cross_obstacles

    # Plan and plot
    best_order, best_length, full_path, has_vip = plan_route_free_space(start, vip, balls, end, obstacles)
    plot_route(start, vip, balls, end, obstacles, full_path, best_order, has_vip)
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
    path_finding(cross, start, vip, objects, end)

test_random_path_finding()
