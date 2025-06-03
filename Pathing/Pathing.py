import numpy as np
import networkx as nx
from shapely.geometry import LineString, Polygon, Point
from itertools import combinations
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from shapely.geometry import Point

class Environment:
    def __init__(self, objects, vip_object, obstacles, start_point=None):

        self.start_point = start_point
        self.vip = vip_object
        self.objects = objects
        #self.main_points = self.objects
        self.main_points = ([self.start_point] if self.start_point else []) + ([self.vip] if self.vip else []) + self.objects
        self.obstacles = obstacles

        # Extract obstacle vertices as extra navigation points
        self.obstacle_points = []
        for poly in self.obstacles:
            self.obstacle_points.extend(list(poly.exterior.coords)[:-1])

        self.all_points = self.main_points + self.obstacle_points

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

def is_line_clear(env, p1, p2):
    if p1 is None or p2 is None:
        return False
    line = LineString([p1, p2])

    # Check polygon obstacles
    for obstacle in env.obstacles:
        if line.crosses(obstacle) or line.within(obstacle):
            return False
        inter = line.intersection(obstacle)
        if not inter.is_empty and not (inter.geom_type == 'Point' or inter.geom_type == 'MultiPoint'):
            return False

    radius = 2  # radius of balls

    # Check object obstacles (balls) except p1 and p2
    for obj in env.main_points:
        if obj == p1 or obj == p2:
            continue
        circle = Point(obj).buffer(radius)
        if line.crosses(circle) or line.within(circle):
            return False

    return True

def build_visibility_graph(env):
    G = nx.Graph()
    points = env.all_points
    if not points:
        return G
    for i, j in combinations(range(len(points)), 2):
        p1, p2 = points[i], points[j]
        if is_line_clear(env, p1, p2):
            dist = np.linalg.norm(np.array(p1) - np.array(p2))
            G.add_edge(i, j, weight=dist)
    for idx in range(len(points)):
        G.add_node(idx)
    return G

#start_idx = 0
#vip_idx = 1
#if not G.has_edge(start_idx, vip_idx):
#    print("No direct collision-free path from start to VIP!")
    # Handle this case: maybe abort or try alternate routing

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

        # Force direct line for start->VIP
        if source == 0 and target == 1:
            route.append(env.all_points[source])
            route.append(env.all_points[target])
            continue

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
        circ = Circle(obj, radius=2, edgecolor='black', facecolor='none',
                      lw=1.5, label='Ball' if i == 0 else None, transform=ax.transData)
        ax.add_patch(circ)

    # Draw VIP as gold circle
    if env.vip:
        vip_circ = Circle(env.vip, radius=2, edgecolor='black', facecolor='gold',
                          lw=1.5, label='VIP Ball', transform=ax.transData)
        ax.add_patch(vip_circ)    # Draw robot start location as a green circle
    start_point = env.start_point
    if start_point:
        start_circ = Circle(start_point, radius=2, edgecolor='black', facecolor='green',
                        lw=1.5, label='Robot Start', transform=ax.transData)
        ax.add_patch(start_circ)
                        
    # Draw goal location as a blue circle (if specified)
    if hasattr(env, 'goal_point') and env.goal_point:
        goal_circ = Circle(env.goal_point, radius=2, edgecolor='black', facecolor='blue',
                        lw=1.5, label='Goal Point', transform=ax.transData)
        ax.add_patch(goal_circ)

    # Draw turn points
    if env.obstacle_points:
        ax.scatter(*zip(*env.obstacle_points), color='blue', s=30, label='Turn Point')

    # Draw path route
    if route:
        for i in range(len(route) - 1):
            x_vals = [route[i][0], route[i + 1][0]]
            y_vals = [route[i][1], route[i + 1][1]]
            ax.plot(x_vals, y_vals, marker='o', linestyle='-', color='green', label='Route' if i == 0 else None)
        for idx, pt in enumerate(route):
            ax.text(pt[0], pt[1], str(idx), fontsize=9, ha='right')

    # Draw normal objects (Balls) as circles
    for i, obj in enumerate(env.objects):
        circ = Circle(obj, radius=2, edgecolor='black', facecolor='white',
                      lw=1.5, transform=ax.transData, zorder = 2)
        ax.add_patch(circ)

    # Draw VIP as gold circle
    if env.vip:
        vip_circ = Circle(env.vip, radius=2, edgecolor='black', facecolor='gold',
                          lw=1.5, transform=ax.transData, zorder = 2)
        ax.add_patch(vip_circ)

    # Finalize layout
    ax.set_xlim(0, 160)
    ax.set_ylim(0, 120)
    ax.set_title("Robot Path Plan")
    plt.grid(True)
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())
    plt.show()

def plan_robot_path(vip, objects, cross, start_point=None, goal_point=None):
    if vip is None and (not objects or len(objects) == 0):
        print("No VIP or objects provided; no path to plan.")
        return []
    
    obstacle_polygons = convert_cross_to_polygons(cross)
    
    # Create environment with just start point and VIP for the first segment
    # This ensures priority pathing from start to VIP
    env_start_to_vip = Environment([], vip, obstacle_polygons, start_point=start_point)
    G_start_vip = build_visibility_graph(env_start_to_vip)
    
    # Find shortest path from start (0) to vip (1)
    try:
        path_start_to_vip_indices = nx.shortest_path(G_start_vip, source=0, target=1, weight='weight')
        path_start_to_vip = [env_start_to_vip.all_points[i] for i in path_start_to_vip_indices]
    except (nx.NetworkXNoPath, nx.NodeNotFound):
        print("No collision-free path from start to VIP.")
        return []
    
    # Now create environment for the objects only (excluding start and VIP from obstacles)
    # This prevents the robot from treating objects as obstacles when it's going to visit them
    env_objects = Environment(objects, None, obstacle_polygons, start_point=vip)
    G_objects = build_visibility_graph(env_objects)
    
    if len(objects) == 0:
        # If there are no objects, just go from start to VIP to goal (if specified)
        if goal_point:
            # Create environment for VIP to goal
            env_goal = Environment([], None, obstacle_polygons, start_point=vip)
            # Add goal point as a "virtual object"
            env_goal.objects = [goal_point]
            env_goal.main_points = [vip, goal_point]
            env_goal.all_points = env_goal.main_points + env_goal.obstacle_points
            
            # Build graph for VIP to goal
            G_goal = build_visibility_graph(env_goal)
            
            # Find path from VIP to goal
            try:
                vip_to_goal_indices = nx.shortest_path(G_goal, source=0, target=1, weight='weight')
                vip_to_goal = [env_goal.all_points[i] for i in vip_to_goal_indices]
                # Return combined path: start -> VIP -> goal
                return path_start_to_vip[:-1] + vip_to_goal
            except (nx.NetworkXNoPath, nx.NodeNotFound):
                print("No collision-free path from VIP to goal.")
                return path_start_to_vip
        else:
            return path_start_to_vip
        
    # Create optimized TSP tour through all objects
    # First, compute distance matrix only among objects (not including obstacle vertices)
    dist_mat = compute_distance_matrix(env_objects, G_objects)
    
    # Run TSP algorithm starting from the VIP point (index 0)
    tsp_indices = christofides_tsp(dist_mat)
    
    # Make sure path starts with VIP point (index 0)
    if 0 not in tsp_indices:
        tsp_indices.insert(0, 0)
    elif tsp_indices[0] != 0:
        vip_index = tsp_indices.index(0)
        tsp_indices = tsp_indices[vip_index:] + tsp_indices[:vip_index]
    
    # Get the actual visitation sequence of main points only (VIP + objects)
    main_point_sequence = []
    for idx in tsp_indices:
        if idx < len(env_objects.main_points):  # Only include main points, not turning points
            point = env_objects.all_points[idx]
            if not main_point_sequence or point != main_point_sequence[-1]:  # Avoid duplicates
                main_point_sequence.append(point)
    
    # Now build the complete path connecting these main points in sequence
    # with optimal navigation around obstacles
    object_route = []
    for i in range(len(main_point_sequence) - 1):
        source_pt = main_point_sequence[i]
        target_pt = main_point_sequence[i + 1]
        
        # Find source and target indices in the full point list
        source_idx = env_objects.all_points.index(source_pt)
        target_idx = env_objects.all_points.index(target_pt)
        
        # Find shortest path between these points
        try:
            subpath_indices = nx.shortest_path(G_objects, source=source_idx, target=target_idx, weight='weight')
            
            # Add all points except the last one (which will be the start of next segment)
            for idx in subpath_indices[:-1]:
                object_route.append(env_objects.all_points[idx])
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            # If no path found, just add the source point and continue
            object_route.append(source_pt)
            print(f"Warning: No path found between {source_pt} and {target_pt}")
            continue
    
    # Add the final destination point
    if main_point_sequence:
        object_route.append(main_point_sequence[-1])
    
    # Combine the start→VIP path with the object tour path
    # But avoid duplicating the VIP point which is included in both
    final_route = path_start_to_vip
    if len(object_route) > 0 and object_route[0] == vip:
        final_route.extend(object_route[1:])
    else:
        final_route.extend(object_route)
        
    # If goal point is specified, add path from last object to goal
    if goal_point:
        last_point = final_route[-1]
        
        # Create environment for last object to goal
        env_goal = Environment([], None, obstacle_polygons, start_point=last_point)
        # Add goal point as a "virtual object"
        env_goal.objects = [goal_point]
        env_goal.main_points = [last_point, goal_point]
        env_goal.all_points = env_goal.main_points + env_goal.obstacle_points
        
        # Build graph for last object to goal
        G_goal = build_visibility_graph(env_goal)
        
        # Find path from last object to goal
        try:
            to_goal_indices = nx.shortest_path(G_goal, source=0, target=1, weight='weight')
            # Skip the first point since it's already in the final_route
            to_goal = [env_goal.all_points[i] for i in to_goal_indices[1:]]
            final_route.extend(to_goal)
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            print(f"Warning: No collision-free path from last object to goal. Goal point {goal_point} will be ignored.")
    
    # Create a full environment for visualization
    env_full = Environment(objects, vip, obstacle_polygons, start_point=start_point)
    # Add goal point to visualization
    if goal_point:
        env_full.goal_point = goal_point
    else:
        env_full.goal_point = None
    G_full = build_visibility_graph(env_full)
    
    # Plot the full route
    plot_path(final_route, env_full, G_full)
    
    return final_route



# Example usage with random values:
if __name__ == '__main__':
    import random
    
    # Set random seed for reproducibility (remove this line for true randomness)
    # random.seed(42)
    
    # Generate random coordinates within the game area (0-160, 0-120)
    start_point = (random.uniform(10, 40), random.uniform(20, 100))  # Robot initial location
    
    # VIP position - keep away from edges and center for better path finding
    vip = (random.uniform(100, 140), random.uniform(70, 110))
    
    # Goal point - where the robot should end up after collecting all objects
    goal_point = (random.uniform(120, 150), random.uniform(10, 40))  # Bottom-right area
    
    # Generate random objects
    num_objects = 10
    objects = []
    for _ in range(num_objects):
        objects.append((random.uniform(20, 140), random.uniform(10, 110)))
    
    # Generate random cross obstacle
    # We'll create the cross with some constraints to avoid blocking all paths
    cx = random.uniform(60, 100)  # Cross center X
    cy = random.uniform(40, 80)   # Cross center Y
    cross_height = random.uniform(30, 50)
    cross_width = random.uniform(40, 60)
    
    cross = (
        (cx, cy + cross_height/2),  # Top
        (cx, cy - cross_height/2),  # Bottom
        (cx + cross_width/2, cy),   # Right
        (cx - cross_width/2, cy)    # Left
    )

    print(f"Start point: {start_point}")
    print(f"VIP position: {vip}")
    print(f"Goal point: {goal_point}")
    print(f"Objects: {objects}")
    print(f"Cross: {cross}")
    
    final_path = plan_robot_path(vip, objects, cross, start_point, goal_point)
    print("\nRobot Pickup Path:")
    for pt in final_path:
        print(pt)

