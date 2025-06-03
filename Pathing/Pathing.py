import numpy as np
import networkx as nx
from shapely.geometry import LineString, Polygon, Point
from itertools import combinations
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

class Environment:
    def __init__(self, objects, vip_object, obstacles, pickup_radius=2):
        self.pickup_radius = pickup_radius

        # Validate inputs
        self.vip = self._offset_point(vip_object) if vip_object is not None else None
        self.objects = [self._offset_point(obj) for obj in objects] if objects else []
        self.main_points = ([self.vip] if self.vip else []) + self.objects

        # Inflate obstacles
        self.obstacles = [Polygon(obs).buffer(self.pickup_radius) for obs in obstacles] if obstacles else []

        # Extract obstacle vertices as extra navigation points
        self.obstacle_points = []
        for poly in self.obstacles:
            self.obstacle_points.extend(list(poly.exterior.coords)[:-1])

        self.all_points = self.main_points + self.obstacle_points

    def _offset_point(self, point):
        if point is None:
            return None
        offset = np.array([1, 1]) / np.sqrt(2) * self.pickup_radius
        return tuple(np.array(point) - offset)

    def is_visible(self, p1, p2):
        if p1 is None or p2 is None:
            return False
        line = LineString([p1, p2])
        for obstacle in self.obstacles:
            if line.crosses(obstacle) or line.within(obstacle):
                return False
            inter = line.intersection(obstacle)
            if not inter.is_empty and not (inter.geom_type == 'Point' or inter.geom_type == 'MultiPoint'):
                return False
        return True

def build_visibility_graph(env):
    G = nx.Graph()
    points = env.all_points
    if not points:
        return G
    for i, j in combinations(range(len(points)), 2):
        p1, p2 = points[i], points[j]
        if env.is_visible(p1, p2):
            dist = np.linalg.norm(np.array(p1) - np.array(p2))
            G.add_edge(i, j, weight=dist)
    for idx in range(len(points)):
        G.add_node(idx)
    return G

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
                except (nx.NetworkXNoPath, nx.NodeNotFound):
                    pass
    return dist_matrix

def christofides_tsp(dist_matrix):
    n = len(dist_matrix)
    if n == 0:
        return []
    G = nx.Graph()
    for i in range(n):
        for j in range(i + 1, n):
            if not np.isinf(dist_matrix[i][j]):
                G.add_edge(i, j, weight=dist_matrix[i][j])

    if G.number_of_edges() == 0:
        return list(range(n))

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

def reorder_path_to_start_with_vip(path):
    if not path:
        return []
    vip_index = 0 if 0 in path else None
    if vip_index is None:
        return path
    vip_index = path.index(0)
    return path[vip_index:] + path[1:vip_index+1]

def expand_full_route(env, G, tsp_path):
    if not tsp_path:
        return []
    route = []
    for i in range(len(tsp_path) - 1):
        source = tsp_path[i]
        target = tsp_path[i+1]
        try:
            subpath_indices = nx.shortest_path(G, source=source, target=target, weight='weight')
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            continue
        for idx in subpath_indices[:-1]:
            route.append(env.all_points[idx])
    if tsp_path[-1] < len(env.all_points):
        route.append(env.all_points[tsp_path[-1]])
    return route

def convert_cross_to_polygons(cross_points, arm_width=4):
    """Convert cross definition to two rectangular polygons (vertical and horizontal arms)."""
    if cross_points is None:
        return []

    top, bottom, right, left = cross_points
    cx, cy = (top[0] + bottom[0]) / 2, (left[1] + right[1]) / 2

    # Vertical arm
    vertical = Polygon([
        (cx - arm_width/2, top[1]),
        (cx + arm_width/2, top[1]),
        (cx + arm_width/2, bottom[1]),
        (cx - arm_width/2, bottom[1])
    ])

    # Horizontal arm
    horizontal = Polygon([
        (left[0], cy - arm_width/2),
        (right[0], cy - arm_width/2),
        (right[0], cy + arm_width/2),
        (left[0], cy + arm_width/2)
    ])

    return [vertical, horizontal]


def plot_path(route, env, graph):
    fig, ax = plt.subplots(figsize=(10, 8))

    # Plot area boundary
    ax.plot([0, 160, 160, 0, 0], [0, 0, 120, 120, 0], color='black', linestyle='--')

    # Ensure aspect ratio for accurate circle size
    ax.set_aspect('equal', adjustable='box')

    # Draw inflated obstacles
    for i, poly in enumerate(env.obstacles):
        x, y = poly.exterior.xy
        ax.fill(x, y, color='red', alpha=0.5, label='Obstacle' if i == 0 else None)

    # Draw normal objects (Balls) as circles
    for i, obj in enumerate(env.objects):
        circ = Circle(obj, radius=env.pickup_radius, edgecolor='black', facecolor='none',
                      lw=1.5, label='Ball' if i == 0 else None, transform=ax.transData)
        ax.add_patch(circ)

    # Draw VIP as gold circle
    if env.vip:
        vip_circ = Circle(env.vip, radius=env.pickup_radius, edgecolor='black', facecolor='gold',
                          lw=1.5, label='VIP Ball', transform=ax.transData)
        ax.add_patch(vip_circ)

    # Draw turn points
    if env.obstacle_points:
        ax.scatter(*zip(*env.obstacle_points), color='blue', s=30, label='Turn Point')

    # Draw path route
    if route:
        for i in range(len(route) - 1):
            x_vals = [route[i][0], route[i + 1][0]]
            y_vals = [route[i][1], route[i + 1][1]]
            ax.plot(x_vals, y_vals, marker='o', linestyle='-', color='blue', label='Route' if i == 0 else None)
        for idx, pt in enumerate(route):
            ax.text(pt[0], pt[1], str(idx), fontsize=9, ha='right')

    # Annotate VIP and normal balls
    for idx, point in enumerate(env.main_points):
        label = "VIP Ball" if idx == 0 and env.vip else "Ball"
        ax.text(point[0], point[1], f'{label} {idx}', fontsize=8, ha='center', va='center',
                color='black', bbox=dict(facecolor='white', edgecolor='black', boxstyle='circle,pad=0.3'))

    # Finalize layout
    ax.set_xlim(0, 160)
    ax.set_ylim(0, 120)
    ax.set_title("Robot Path Plan with Obstacle Avoidance")
    plt.grid(True)
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())
    plt.show()

def plan_robot_path(vip, objects, cross, pickup_radius=2):
    if vip is None and (not objects or len(objects) == 0):
        print("No VIP or objects provided; no path to plan.")
        return []

    # Convert cross input to obstacle polygons
    obstacle_polygons = convert_cross_to_polygons(cross)

    env = Environment(objects, vip, obstacle_polygons, pickup_radius)
    G = build_visibility_graph(env)
    dist_mat = compute_distance_matrix(env, G)
    tsp_path = christofides_tsp(dist_mat)
    ordered_path = reorder_path_to_start_with_vip(tsp_path)
    full_route = expand_full_route(env, G, ordered_path)

    if full_route:
        plot_path(full_route, env, G)
    else:
        print("No valid route found with given inputs.")
    return full_route

# Example usage with missing inputs:
if __name__ == '__main__':
    vip = None
    objects = [(50, 30), (100, 60)]
    cross = ((80, 100), (80, 40), (120, 70), (40, 70))  # Top, Bottom, Right, Left

    final_path = plan_robot_path(vip, objects, cross, pickup_radius=2)
    print("Robot Pickup Path:")
    for pt in final_path:
        print(pt)
