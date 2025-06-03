import numpy as np
import networkx as nx
from shapely.geometry import LineString, Polygon, Point
from itertools import combinations
import matplotlib.pyplot as plt

# -------------------------------
# Environment & Geometry Helpers
# -------------------------------
class Environment:
    def __init__(self, objects, vip_object, obstacles, pickup_radius=2):
        self.pickup_radius = pickup_radius

        # Offset object points to pickup positions
        self.vip = self._offset_point(vip_object)
        self.objects = [self._offset_point(obj) for obj in objects]  # list of 10 (x, y) tuples
        self.main_points = [self.vip] + self.objects

        # Inflate obstacles
        self.obstacles = [Polygon(obs).buffer(self.pickup_radius) for obs in obstacles]

        # Extract obstacle vertices as extra navigation points
        self.obstacle_points = []
        for poly in self.obstacles:
            self.obstacle_points.extend(list(poly.exterior.coords)[:-1])

        self.all_points = self.main_points + self.obstacle_points

    def _offset_point(self, point):
        # Apply a small fixed offset along a default diagonal
        offset = np.array([1, 1]) / np.sqrt(2) * self.pickup_radius
        return tuple(np.array(point) - offset)

    def is_visible(self, p1, p2):
        line = LineString([p1, p2])
        for obstacle in self.obstacles:
            if line.crosses(obstacle) or line.within(obstacle):
                return False
            inter = line.intersection(obstacle)
            if not inter.is_empty and not (inter.geom_type == 'Point' or inter.geom_type == 'MultiPoint'):
                return False
        return True

# ----------------------
# Visibility Graph Build
# ----------------------
def build_visibility_graph(env):
    G = nx.Graph()
    points = env.all_points
    for i, j in combinations(range(len(points)), 2):
        p1, p2 = points[i], points[j]
        if env.is_visible(p1, p2):
            dist = np.linalg.norm(np.array(p1) - np.array(p2))
            G.add_edge(i, j, weight=dist)
    for idx in range(len(points)):
        G.add_node(idx)
    return G

# ------------------
# Path Length Matrix
# ------------------
def compute_distance_matrix(env, G):
    n = len(env.main_points)
    dist_matrix = np.full((n, n), np.inf)
    for i in range(n):
        for j in range(n):
            if i == j:
                dist_matrix[i][j] = 0
            else:
                try:
                    dist_matrix[i][j] = nx.shortest_path_length(G, i, j, weight='weight')
                except nx.NetworkXNoPath:
                    pass
    return dist_matrix

# -------------------------
# Christofides TSP Routine
# -------------------------
def christofides_tsp(dist_matrix):
    G = nx.Graph()
    n = len(dist_matrix)
    for i in range(n):
        for j in range(i + 1, n):
            if not np.isinf(dist_matrix[i][j]):
                G.add_edge(i, j, weight=dist_matrix[i][j])

    T = nx.minimum_spanning_tree(G)
    odd_deg = [v for v in T.nodes if T.degree[v] % 2 == 1]
    M = nx.Graph()
    for i, j in combinations(odd_deg, 2):
        if not np.isinf(dist_matrix[i][j]):
            M.add_edge(i, j, weight=dist_matrix[i][j])
    matching = nx.algorithms.matching.max_weight_matching(M, maxcardinality=True, weight='weight')

    multigraph = nx.MultiGraph()
    multigraph.add_edges_from(T.edges)
    multigraph.add_edges_from(matching)
    euler_circuit = list(nx.eulerian_circuit(multigraph, source=0))

    visited = set()
    path = []
    for u, v in euler_circuit:
        if u not in visited:
            visited.add(u)
            path.append(u)
    path.append(path[0])  # to make it a cycle
    return path

# ---------------------------
# Final Route with VIP First
# ---------------------------
def reorder_path_to_start_with_vip(path):
    vip_index = path.index(0)
    return path[vip_index:] + path[1:vip_index+1]

# -------------------------------------
# Expand TSP path to full route coords
# -------------------------------------
def expand_full_route(env, G, tsp_path):
    route = []
    for i in range(len(tsp_path) - 1):
        source = tsp_path[i]
        target = tsp_path[i+1]
        subpath_indices = nx.shortest_path(G, source=source, target=target, weight='weight')
        for idx in subpath_indices[:-1]:
            route.append(env.all_points[idx])
    route.append(env.all_points[tsp_path[-1]])
    return route

# ----------------------
# Public Entry Function
# ----------------------
def plan_robot_path(vip, objects, obstacle_polygons, pickup_radius=2):
    """
    Plan a robot path to pick up objects and a VIP object while avoiding obstacles.
    :param vip: Tuple (x, y) for the VIP object
    :param objects: List of tuples [(x1, y1), (x2, y2), ...] for normal objects
    :param obstacle_polygons: List of lists of tuples representing obstacle vertices
    :param pickup_radius: Radius around each object for pickup
    :return: List of tuples representing the full route
    """
    env = Environment(objects, vip, obstacle_polygons, pickup_radius)
    G = build_visibility_graph(env)
    dist_mat = compute_distance_matrix(env, G)
    tsp_path = christofides_tsp(dist_mat)
    ordered_path = reorder_path_to_start_with_vip(tsp_path)
    full_route = expand_full_route(env, G, ordered_path)
    plot_path(full_route, env, G)
    return full_route

# -------------------
# Plotting Function
# -------------------
def plot_path(route, env, graph):
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot([0, 160, 160, 0, 0], [0, 0, 120, 120, 0], color='black', linestyle='--')

    for i, poly in enumerate(env.obstacles):
        x, y = poly.exterior.xy
        ax.fill(x, y, color='red', alpha=0.5, label='Obstacle' if i == 0 else None)

    ax.scatter(*zip(*env.objects), color='green', s=50, label='Normal Object')
    ax.scatter(*zip(*[env.vip]), color='gold', s=70, edgecolors='black', label='VIP Object')

    for i in range(len(route) - 1):
        x_vals = [route[i][0], route[i+1][0]]
        y_vals = [route[i][1], route[i+1][1]]
        ax.plot(x_vals, y_vals, marker='o', linestyle='-', color='blue', label='Route' if i == 0 else None)

    for pt in set(route):
        label = ""
        if pt == env.vip:
            label = "VIP Object"
            color = 'gold'
        elif pt in env.objects:
            label = "Normal Object"
            color = 'green'
        elif pt in env.obstacle_points:
            label = "Turn Point"
            color = 'blue'
        ax.text(pt[0], pt[1], label, fontsize=7, ha='left', va='bottom', color=color)

    ax.set_xlim(0, 160)
    ax.set_ylim(0, 120)
    ax.set_title("Robot Path Plan")
    ax.set_aspect('equal')
    plt.grid(True)
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())
    plt.show()

# -----------------
# Example Usage:
# -----------------
if __name__ == '__main__':
    vip = (40, 30)
    objects = [
        (80, 18), (16, 48), (120, 36), (72, 72), (48, 102),
        (104, 84), (8, 6), (128, 12), (56, 48), (32, 78)
    ]
    obstacles = [
        [(56, 42), (72, 42), (72, 54), (56, 54)],
        [(88, 60), (112, 60), (112, 78), (88, 78)]
    ]

    final_path = plan_robot_path(vip, objects, obstacles, pickup_radius=2)
    print("Robot Pickup Path:")
    for pt in final_path:
        print(pt)
