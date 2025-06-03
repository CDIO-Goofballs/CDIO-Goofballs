import numpy as np
import networkx as nx
from shapely.geometry import LineString, Polygon, Point
from itertools import combinations
import matplotlib.pyplot as plt

# -------------------------------
# Environment & Geometry Helpers
# -------------------------------
class Environment:
    def __init__(self, objects, vip_object, obstacles):
        self.vip = vip_object
        self.objects = objects  # list of 10 (x, y) tuples
        self.obstacles = [Polygon(obs) for obs in obstacles]  # list of polygons
        
        # Collect obstacle corners as points
        self.obstacle_points = []
        for poly in self.obstacles:
            self.obstacle_points.extend(list(poly.exterior.coords)[:-1])  # exclude repeated last point

        # All points = vip + objects + obstacle corners
        self.main_points = [vip_object] + objects
        self.all_points = self.main_points + self.obstacle_points

    def is_visible(self, p1, p2):
        line = LineString([p1, p2])
        # Check line against all obstacles; line touching polygon edge is allowed, but crossing is not
        for obstacle in self.obstacles:
            if line.crosses(obstacle) or line.within(obstacle):
                return False
            # intersection at a single point on boundary is OK (e.g. touching corner)
            inter = line.intersection(obstacle)
            if not inter.is_empty and not (inter.geom_type == 'Point' or inter.geom_type == 'MultiPoint'):
                # If intersection is a line or polygon, reject
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
            # Turn cost could be refined, but for now no added cost here
            G.add_edge(i, j, weight=dist)
    # Ensure all nodes are added (in case isolated)
    for idx in range(len(points)):
        G.add_node(idx)
    return G

# ------------------
# Path Length Matrix
# ------------------
def compute_distance_matrix(env, G):
    n = len(env.main_points)  # Only main points for TSP
    dist_matrix = np.full((n, n), np.inf)
    main_indices = range(n)  # main points are first n in env.all_points
    for i in main_indices:
        for j in main_indices:
            if i == j:
                dist_matrix[i][j] = 0
            else:
                try:
                    dist_matrix[i][j] = nx.shortest_path_length(G, source=i, target=j, weight='weight')
                except nx.NetworkXNoPath:
                    dist_matrix[i][j] = np.inf
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

# -------------------------------------
# Expand TSP path to full route coords
# -------------------------------------
def expand_full_route(env, G, tsp_path):
    route = []
    for i in range(len(tsp_path) - 1):
        source = tsp_path[i]
        target = tsp_path[i+1]
        # Get shortest path in visibility graph between source and target
        subpath_indices = nx.shortest_path(G, source=source, target=target, weight='weight')
        # Append all points except last to avoid duplicates, last will be start of next segment
        for idx in subpath_indices[:-1]:
            route.append(env.all_points[idx])
    # Add last point explicitly
    route.append(env.all_points[tsp_path[-1]])
    return route

# ----------------------
# Public Entry Function
# ----------------------
def plan_robot_path(vip, objects, obstacle_polygons):
    env = Environment(objects, vip, obstacle_polygons)
    G = build_visibility_graph(env)
    dist_mat = compute_distance_matrix(env, G)
    tsp_path = christofides_tsp(dist_mat)
    ordered_path = reorder_path_to_start_with_vip(tsp_path)
    full_route = expand_full_route(env, G, ordered_path)
    plot_path(full_route, env.obstacles, G, env.all_points, env.main_points)
    return full_route

# -------------------
# Plotting Function
# -------------------
def plot_path(route, obstacles, graph, all_points, main_points):
    fig, ax = plt.subplots()
    # Draw obstacles
    for poly in obstacles:
        x, y = poly.exterior.xy
        ax.fill(x, y, color='red', alpha=0.5)

    # Draw visibility graph edges
    # for (u, v) in graph.edges():
    #     x_vals = [all_points[u][0], all_points[v][0]]
    #     y_vals = [all_points[u][1], all_points[v][1]]
    #     ax.plot(x_vals, y_vals, linestyle='dotted', color='gray', alpha=0.3)

    # Draw route path
    x_vals = [p[0] for p in route]
    y_vals = [p[1] for p in route]
    ax.plot(x_vals, y_vals, marker='o', linestyle='-', color='blue')

    # Mark main points (VIP + objects)
    for idx, pt in enumerate(main_points):
        ax.scatter(pt[0], pt[1], c='green' if idx == 0 else 'orange', s=80, zorder=5)
        ax.text(pt[0], pt[1], f'M{idx}' if idx > 0 else 'VIP', fontsize=9, ha='right')

    ax.set_title("Robot Path Plan")
    ax.set_aspect('equal')
    plt.grid(True)
    plt.show()

# -----------------
# Example Usage:
# -----------------
if __name__ == '__main__':
    import random
    
    # Generate random VIP position (between 0 and 20)
    vip = (random.uniform(0, 20), random.uniform(0, 20))
    
    # Generate 10 random object positions
    objects = []
    for _ in range(10):
        objects.append((random.uniform(0, 20), random.uniform(0, 20)))
    
    # Generate 2-4 random rectangular obstacles
    obstacles = []
    for _ in range(random.randint(2, 4)):
        # Create a random rectangle
        x = random.uniform(0, 15)
        y = random.uniform(0, 15)
        width = random.uniform(1, 4)
        height = random.uniform(1, 4)
        # Add rectangle as polygon points (clockwise)
        obstacles.append([(x, y), (x + width, y), (x + width, y + height), (x, y + height)])
    
    final_path = plan_robot_path(vip, objects, obstacles)
    print("Robot Pickup Path:")
    for pt in final_path:
        print(pt)
