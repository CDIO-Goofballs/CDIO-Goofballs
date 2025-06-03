def plan_robot_path(vip, objects, cross, start_point=None):
    if vip is None and (not objects or len(objects) == 0):
        print("No VIP or objects provided; no path to plan.")
        return []
    
    obstacle_polygons = convert_cross_to_polygons(cross)
    
    # First, build environment including ALL points (start, VIP, and objects)
    env_full = Environment(objects, vip, obstacle_polygons, start_point=start_point)

    # Build visibility graph for all points
    G_full = build_visibility_graph(env_full)

    # Step 1: Find shortest path from start (0) to vip (1)
    try:
        path_start_to_vip_indices = nx.shortest_path(G_full, source=0, target=1, weight='weight')
        path_start_to_vip = [env_full.all_points[i] for i in path_start_to_vip_indices]
    except (nx.NetworkXNoPath, nx.NodeNotFound):
        print("No collision-free path from start to VIP.")
        return []
    
    # Step 2: Build TSP path for objects, with VIP as starting point
    # Create a new environment with VIP as the start point and only objects
    env_objects_only = Environment(objects, None, obstacle_polygons, start_point=vip)
    G_objects = build_visibility_graph(env_objects_only)
    dist_mat_objects = compute_distance_matrix(env_objects_only, G_objects)

    # Get TSP path through objects
    if len(objects) == 0:
        object_route = []
    else:
        tsp_path_objects = christofides_tsp(dist_mat_objects)
        
        # Make sure VIP point (index 0) is at the start of the path
        if 0 not in tsp_path_objects:
            tsp_path_objects.insert(0, 0)
        elif tsp_path_objects[0] != 0:
            vip_index = tsp_path_objects.index(0)
            tsp_path_objects = tsp_path_objects[vip_index:] + tsp_path_objects[:vip_index]
            
        # Expand TSP path indices into coordinates
        object_route = []
        for i in range(len(tsp_path_objects) - 1):
            try:
                subpath_indices = nx.shortest_path(G_objects, source=tsp_path_objects[i], target=tsp_path_objects[i+1], weight='weight')
            except (nx.NetworkXNoPath, nx.NodeNotFound):
                continue
                
            # Skip the first point if it's the first segment (to avoid duplicating the VIP)
            start_idx = 1 if i == 0 else 0
            for idx in subpath_indices[start_idx:-1]:
                object_route.append(env_objects_only.all_points[idx])
                
        # Add the last point
        if tsp_path_objects and tsp_path_objects[-1] < len(env_objects_only.all_points):
            object_route.append(env_objects_only.all_points[tsp_path_objects[-1]])

    # Final combined path: start -> VIP -> objects
    final_route = path_start_to_vip + object_route

    # Plot the path
    plot_path(final_route, env_full, G_full)

    return final_route
