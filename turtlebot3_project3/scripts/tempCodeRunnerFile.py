    # def a_star(start, goal, map, clearance, robot_radius):
    #     # Define the robot's radius and other parameters
    #     r = 0.038
    #     L = 0.354
    #     dt = 70
    #     open_list = []
    #     closed_list = set()

    #     heapq.heappush(open_list, (0, start))  # 0 being the priority for start
    #     came_from = {}  # parent nodes dictionary
    #     cost_so_far = {start: 0}  # node: cost to reach dictionary
    #     path_iteration = 0

    #     while open_list:
    #         current_cost, current_node = heapq.heappop(open_list)
    #         # Check if the current node's (x, y) matches the goal, and theta is within the tolerance range
    #         if (abs(current_node[0] - goal[0]) <= 10 and abs(current_node[1] - goal[1]) <= 10):

    #             # Reached the goal (within tolerance), reconstruct the path
    #             path = []
    #             node = current_node
    #             while node != start:
    #                 path.append(node)
    #                 node = came_from[node]
    #             path.append(start)
    #             path.reverse()
    #             return path

    #         closed_list.add(current_node)
    #         for move in action_set:
    #             r1, rr = move
    #             ul = (2 * 3.14 * r1)/60
    #             ur = (2 * 3.14 * rr)/60
    #             next_theta = current_node[2] + (r / L) * (ur - ul) * dt
    #             if (next_theta == 0):
    #                 next_theta = 0
    #             elif (next_theta == 360):
    #                 next_theta = 360
    #             else :
    #                 next_theta = next_theta % 360
    #             next_x = current_node[0] + 0.5*r * (ul + ur) * np.cos(np.radians(next_theta)) * dt
    #             next_y = current_node[1] +  0.5*r * (ul + ur) * np.sin(np.radians(next_theta)) * dt
    #             next_node = (
    #                 int(round(next_x)),
    #                 int(round(next_y)),
    #                 int(next_theta)
    #             )

    #             if (clearance <= next_node[0] < (map.shape[0] - robot_radius)) and (clearance <= next_node[1] < (map.shape[1] - robot_radius)) and (
    #                     next_node[:2] not in closed_list) and (np.array_equal(map[next_node[0], next_node[1]], black)):

    #                 new_cost = cost_so_far[current_node] + 1  # Use the cost directly from the action set
    #                 heuristic_cost = heuristic(next_node, goal)  # Calculate heuristic cost
    #                 priority = new_cost + heuristic_cost  # Update priority with heuristic cost

    #                 if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
    #                     cost_so_far[next_node] = new_cost
    #                     heapq.heappush(open_list, (priority, next_node))
    #                     came_from[next_node] = current_node  # Store the parent node with the correct theta value
    #                     # Update the color of the scanned pixel to green
    #                     map[next_node[0], next_node[1]] = [0, 255, 0]  # Green color
    #                     print(f"Scanning pixel: {next_node}")
    #     return None  # No path found