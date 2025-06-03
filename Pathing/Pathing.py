import numpy as np
import networkx as nx
from shapely.geometry import LineString, Polygon
from scipy.spatial import distance_matrix
from itertools import combinations
import matplotlib.pyplot as plt

# -------------------------------
# Environment & Geometry Helpers
# -------------------------------
class Environment:
    def __init__(self, objects, vip_object, obstacles):
        self.vip = vip_object
        self.objects = objects  # list of 10 (x, y) tuples
        self.all_points = [vip_object] + objects
        self.obstacles = [Polygon(obs) for obs in obstacles]  # list of polygons

    def is_visible(self, p1, p2):
        line = LineString([p1, p2])
        for obstacle in self.obstacles:
            if line.crosses(obstacle) or line.within(obstacle) or line.intersects(obstacle):
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
            turn_cost = compute_turn_cost(p1, p2)
            total_cost = dist + turn_cost
            G.add_edge(i, j, weight=total_cost)
    # Ensure all nodes are added
    for idx in range(len(points)):
        G.add_node(idx)
    return G

# ------------------------
# Direction Change Penalty
# ------------------------
def compute_turn_cost(p1, p2):
    return 1  # unit cost for needing to stop and turn

# ------------------
# Path Length Matrix
# ------------------
def compute_distance_matrix(env, G):
    n = len(env.all_points)
    dist_matrix = np.full((n, n), np.inf)
    for i in range(n):
        for j in range(n):
            if i == j:
                dist_matrix[i][j] = 0
            else:
                if G.has_node(i) and G.has_node(j):
                    try:
                        path_len = nx.shortest_path_length(G, i, j, weight='weight')
                        dist_matrix[i][j] = path_len
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
    vip_index = path.index(0)  # VIP is always at index 0
    return path[vip_index:] + path[1:vip_index+1]

# ----------------------
# Public Entry Function
# ----------------------
def plan_robot_path(vip, objects, obstacle_polygons):
    env = Environment(objects, vip, obstacle_polygons)
    G = build_visibility_graph(env)
    dist_mat = compute_distance_matrix(env, G)
    tsp_path = christofides_tsp(dist_mat)
    ordered_path = reorder_path_to_start_with_vip(tsp_path)
    coords_path = [env.all_points[i] for i in ordered_path]
    plot_path(coords_path, env.obstacles, G, env.all_points)
    return coords_path

# -------------------
# Plotting Function
# -------------------
def plot_path(path, obstacles, graph, all_points):
    fig, ax = plt.subplots()
    # Draw full graph edges to reflect possible routes
    for (u, v) in graph.edges():
        x_vals = [all_points[u][0], all_points[v][0]]
        y_vals = [all_points[u][1], all_points[v][1]]
        ax.plot(x_vals, y_vals, linestyle='dotted', color='gray', alpha=0.5)
    # Draw the final computed path
    for i in range(len(path) - 1):
        x_vals = [path[i][0], path[i+1][0]]
        y_vals = [path[i][1], path[i+1][1]]
        ax.plot(x_vals, y_vals, marker='o', linestyle='-', color='blue')
    for poly in obstacles:
        x, y = poly.exterior.xy
        ax.fill(x, y, color='red', alpha=0.5)
    for idx, pt in enumerate(path):
        ax.text(pt[0], pt[1], str(idx), fontsize=9, ha='right')
    ax.set_title("Robot Path Plan")
    ax.set_aspect('equal')
    plt.grid(True)
    plt.show()

# -----------------
# Example Usage:
# -----------------
if __name__ == '__main__':
    vip = (5, 5)
    objects = [(10, 3), (2, 8), (15, 6), (9, 12), (6, 17), (13, 14), (1, 1), (16, 2), (7, 8), (4, 13)]
    obstacles = [[(7, 7), (9, 7), (9, 9), (7, 9)], [(11, 10), (14, 10), (14, 13), (11, 13)]]

    final_path = plan_robot_path(vip, objects, obstacles)
    print("Robot Pickup Path:")
    for pt in final_path:
        print(pt)
