import numpy as np
import matplotlib.pyplot as plt
import cv2
import heapq
import time

# Generating map
map = np.zeros((2000, 6000, 3))
red = np.array([255, 0, 0])
yellow = np.array([255, 255, 0])
black = np.array([0, 0, 0])
clearance = int(input("Enter the clearance value:"))
robot_radius = int(input("Enter the robot radius: "))

# Generating boundary clearance
cv2.rectangle(map, pt1=(0000, 0000), pt2=(6000, 2000), color=(255, 255, 0), thickness=-1)
cv2.rectangle(map, pt1=(0000 + (clearance + robot_radius), 0000 + (clearance + robot_radius)), pt2=((6000 - (clearance + robot_radius)), (2000 - (clearance + robot_radius))), color=(0, 0, 0), thickness=-1)

# Generating rectangles clearance
cv2.rectangle(map, pt1=(1500 - (clearance + robot_radius), 0), pt2=(1750 + (clearance + robot_radius), 1000 + (clearance + robot_radius)), color=(255, 255, 0), thickness=-1)
cv2.rectangle(map, pt1=(2500 - (clearance + robot_radius), 1000 - (clearance + robot_radius)), pt2=((2750 + (clearance + robot_radius)), 2000), color=(255, 255, 0), thickness=-1)

# Generating rectangles obstacle
cv2.rectangle(map, pt1=(1500, 0), pt2=(1750, 1000), color=(255, 0, 0), thickness=-1)
cv2.rectangle(map, pt1=(2500, 1000), pt2=(2750, 2000), color=(255, 0, 0), thickness=-1)


# Generating circle clearance
center_circle = (4200, 800)   
radius_circle = 600 + (clearance + robot_radius)
cv2.circle(map, center_circle, radius_circle, (255, 255, 0), -1)

# Generating circle obstacle 
center_circle = (4200, 800)   
radius_circle = 600
cv2.circle(map, center_circle, radius_circle, (255, 0, 0), -1)


# Define the video file path
video_name = 'D:\Desktop\A_star_scan.mp4'
out = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'mp4v'), 10, (map.shape[1], map.shape[0]))

# Display the image
plt.imshow(map.astype(int))
plt.title("Initial Map 'red = obstacle' 'yellow = clearance + robot radius'\n\n" r"$\bf{(close\ this\ window\ to\ continue)}$")
plt.show()

def a_star(start, goal, map, clearance, robot_radius):
    open_list = []
    closed_list = set()
    heapq.heappush(open_list, (0, start))  # 0 being the priority for start
    came_from = {}  # parent nodes dictionary
    cost_so_far = {start: 0}  # node: cost to reach dictionary
    path_iteration = 0
    theta_tolerance = 30  # Define the tolerance for theta in degrees

    while open_list:
        current_cost, current_node = heapq.heappop(open_list)

        # # Check if the current node's (x, y) matches the goal, and theta is within the tolerance range
        # if (abs(current_node[0] - goal[0]) <= 1 and abs(current_node[1] - goal[1]) <= 1 and
        #     min(abs(current_node[2] - goal[2]), 360 - abs(current_node[2] - goal[2])) <= theta_tolerance):

        # Check if the current node's (x, y) matches the goal, and theta is within the tolerance range
        if (abs(current_node[0] - goal[0]) <= 1 and abs(current_node[1] - goal[1]) <= 1):

            # Reached the goal (within tolerance), reconstruct the path
            path = []
            node = current_node
            while node != start:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path

        closed_list.add(current_node)
        for move in actions_set:
            next_x = current_node[0] + move[0]
            next_y = current_node[1] + move[1]
            next_theta = (current_node[2] + move[2]) % 360

            next_node = (
                int(round(next_x)),
                int(round(next_y)),
                int(next_theta)
            )

            if (clearance <= next_node[0] < (map.shape[0] - robot_radius)) and (clearance <= next_node[1] < (map.shape[1] - robot_radius)) and (
                    next_node[:2] not in closed_list) and (np.array_equal(map[next_node[0], next_node[1]], black)):

                new_cost = cost_so_far[current_node] + move[3]  # Use the cost directly from the action set
                heuristic_cost = heuristic(next_node, goal)  # Calculate heuristic cost
                priority = new_cost + heuristic_cost  # Update priority with heuristic cost

                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    heapq.heappush(open_list, (priority, next_node))
                    came_from[next_node] = current_node  # Store the parent node with the correct theta value
                    # Update the color of the scanned pixel to green
                    map[next_node[0], next_node[1]] = [0, 255, 0]  # Green color
                    print(f"Scanning pixel: {next_node}")
                    if path_iteration % 10000 == 0:
                        frame = cv2.cvtColor(map.astype(np.uint8), cv2.COLOR_RGB2BGR)
                        out.write(frame)
                    path_iteration += 1

    return None  # No path found


# Define action set and cost calculation
step_size = float(input("Enter the step size: "))
thetas = [-60, -30, 0, 30, 60]  # List of theta values
actions_set = []
for theta in thetas:
    for i in range(5):
        actions_set.append((step_size * np.cos(np.radians(theta * i)), step_size * np.sin(np.radians(theta * i)), theta * i, step_size))

# Define the heuristic function (Euclidean distance)
def heuristic(node, goal):
    return np.sqrt((goal[0] - node[0]) ** 2 + (goal[1] - node[1]) ** 2)

# Prompting user for the start coordinates and initial theta
while True:
    start_x = int(input("\nEnter your start x-coordinate: "))
    start_y = int(input("Enter your start y-coordinate: "))
    theta_i = int(input("Enter start theta value: "))

    # Access the pixel value using the index
    if ((clearance + robot_radius) <= start_y <= (map.shape[0]- (clearance + robot_radius))) and ((clearance + robot_radius) <= start_x < (map.shape[1]-(clearance + robot_radius))):
        start_pixel_value = [int(value) for value in map[start_y, start_x]]
    else:
        print("Your values are not within boundary or on clearances. Please re-enter the start coordinates.")
        continue

    # Check if the pixel value corresponds to red or yellow and if the start coordinates are on the boundary
    if np.array_equal(start_pixel_value, red) or np.array_equal(start_pixel_value, yellow) or ((clearance + robot_radius) >= start_x >= (map.shape[0]-(clearance + robot_radius))) or ((clearance + robot_radius) >= start_y < (map.shape[1]-(clearance + robot_radius))):
        print("Your start is on an obstacle. Please re-enter the start coordinates.")

    else:
        print("Your start coordinates are correct, please proceed ahead.")
        break

# Prompting user for the goal coordinates and goal theta
while True:
    goal_x = int(input("\nEnter your goal x-coordinate: "))
    goal_y = int(input("Enter your goal y-coordinate: "))


    # Access the pixel value using the index
    if ((clearance + robot_radius) <= goal_y <= (map.shape[0]-(clearance + robot_radius))) and ((clearance + robot_radius) <= goal_x <= (map.shape[1]-(clearance + robot_radius))):
        goal_pixel_value = [int(value) for value in map[goal_y, goal_x]]
    else:
        print("Your values are not within boundary or on clearances. Please re-enter the goal coordinates.")
        continue

    # Check if the pixel value corresponds to red or yellow and if the goal coordinates are on the boundary
    if np.array_equal(goal_pixel_value, red) or np.array_equal(goal_pixel_value, yellow):
        print("Your goal is on an obstacle. Please re-enter the goal coordinates.")
        print(goal_pixel_value)
    else:
        print("Your goal coordinates are correct. Search in progress. PLEASE WAIT...")
        print("Goal coordinate has the following pixel values: ", goal_pixel_value)
        break

# Start time
time_start = time.time()

# Run A_star's algorithm
start_node = (start_y, start_x, theta_i)  # Assuming the robot starts from the bottom-left corner
goal_node = (goal_y, goal_x)
path = a_star(start_node, goal_node, map, clearance, robot_radius)

path_iteration = 0
# Print the path
if path:
    print("Path found:", path)
    for node in path:
        # Change color to black for nodes in the path
        map[node[0], node[1]] = [255, 255, 255]
        if path_iteration % 10 == 0:
            frame = cv2.cvtColor(map.astype(np.uint8), cv2.COLOR_RGB2BGR)
            out.write(frame)
        path_iteration += 1

    # Draw circles for initial and goal positions
    cv2.circle(map, (start_x, start_y), 30, (0, 100, 100), -1)  # Green circle for initial position
    cv2.circle(map, (goal_x, goal_y), 30, (0, 0, 255), -1)   # Red circle for goal position

    # Draw the full path on the map
    for i in range(len(path) - 1):
        cv2.line(map, (path[i][1], path[i][0]), (path[i+1][1], path[i+1][0]), (0, 100, 255), 2)

else:
    print("No path found.")

# Computing time taken for the search
time_end = time.time()
time_taken = time_end - time_start

# Display the updated image after scanning
plt.imshow(map.astype(int))
plt.title(f'Map after scanning, Run time: {time_taken} seconds')
plt.show()

out.release()
cv2.destroyAllWindows()