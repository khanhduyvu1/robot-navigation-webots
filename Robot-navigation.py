from fairis_tools.my_robot import MyRobot
import pickle
import heapq

# === Load saved map ===
with open('../lab5_task1/wall_config.pkl', 'rb') as f: # change location if needed
    wall_map = pickle.load(f)

# === Build graph from wall_map ===
graph = {}
for cell, walls in wall_map.items():  # walls = [W, N, E, S]
    neighbors = {}
    if walls[0] == 0 and cell % 5 != 1:      # West
        neighbors[cell - 1] = 1
    if walls[1] == 0 and cell > 5:           # North
        neighbors[cell - 5] = 1
    if walls[2] == 0 and cell % 5 != 0:      # East
        neighbors[cell + 1] = 1
    if walls[3] == 0 and cell <= 20:         # South
        neighbors[cell + 5] = 1
    graph[cell] = neighbors

# === Dijkstra's Algorithm ===
def dijkstra(graph, start, goal):
    dist = {node: float('inf') for node in graph}
    dist[start] = 0
    prev = {}
    pq = [(0, start)]

    while pq:
        cost, u = heapq.heappop(pq)
        if u == goal:
            break
        for v in graph[u]:
            if cost + graph[u][v] < dist[v]:
                dist[v] = cost + graph[u][v]
                prev[v] = u
                heapq.heappush(pq, (dist[v], v))

    path = []
    u = goal
    while u in prev:
        path.insert(0, u)
        u = prev[u]
    path.insert(0, start)
    return path

# === Create robot ===
robot = MyRobot()
maze_file = '../../worlds/Spring25/maze8.xml'
robot.load_environment(maze_file)
robot.move_to_start()

# === Get starting cell ===
start_pos = robot.starting_position
x = start_pos.x
y = start_pos.y
theta = start_pos.theta

# Convert (x, y) to cell number
def get_initial_cell(x, y):
    col = int(x + 2.5)
    row = 4 - int(y + 2.5)
    return row * 5 + col + 1

start_cell = get_initial_cell(x, y)
goal_cell = 17

# === Calculate path before any movement ===
path = dijkstra(graph, start_cell, goal_cell)
path_str = " → ".join([f"C{cell}" for cell in path])
print(f"\nStarting Cell: {start_cell}\nCalculated Shortest Path:\n{path_str}\n")

# === Move robot and print pose ===
def robot_speed(left_speed, right_speed):
    robot.set_left_motors_velocity(left_speed)
    robot.set_right_motors_velocity(right_speed)

def print_pose(prev_cell, curr_cell, x, y, distance_moved):
    heading = robot.get_compass_reading()
    dx = curr_cell - prev_cell

    if dx == 1:       # moved east
        x += distance_moved
    elif dx == -1:    # moved west
        x -= distance_moved
    elif dx == -5:    # moved north
        y += distance_moved
    elif dx == 5:     # moved south
        y -= distance_moved

    print(f"Pose: (x = {x:.2f}, y = {y:.2f}, Cell = {curr_cell}, θ = {heading:.0f}°)")
    return x, y

def turn(degree):
    target_direction = degree
    while robot.experiment_supervisor.step(robot.timestep) != -1:
        current_direction = robot.get_compass_reading()
        error = (target_direction - current_direction + 360) % 360
        if error <= 180:
            robot_speed(-5, 5)  # spin counterclockwise (left)
            initial_direction = 'left'
        else:
            robot_speed(5, -5)  # spin clockwise (right)
            initial_direction = 'right'
        if error < 1:
            robot.stop()
            break
        if error <= 10 and initial_direction == 'left':
            robot_speed(-0.5, 0.5)
        elif error <= 10 and initial_direction == 'right':
            robot_speed(0.5,-0.5)

def get_front_distance():
    lidar_data = robot.get_lidar_range_image()
    return round(lidar_data[400], 3)

def get_move_direction(from_cell, to_cell):
    dx = to_cell - from_cell
    if dx == 1:
        return 0    # East
    elif dx == -1:
        return 180  # West
    elif dx == -5:
        return 90   # North
    elif dx == 5:
        return 270  # South
    else:
        return None  # Invalid move

def move_forward_1m(speed=10, threshold=1.0):
    initial_distance = get_front_distance()
    robot_speed(speed, speed)
    while robot.experiment_supervisor.step(robot.timestep) != -1:
        current_distance = get_front_distance()
        if initial_distance - current_distance >= threshold:
            break

    return initial_distance - current_distance

heading = robot.get_compass_reading()
print(f"Pose: (x = {x:.2f}, y = {y:.2f}, Cell = {start_cell}, θ = {heading:.0f}°)")

# === Path Execution ===
for i in range(len(path) - 1):
    from_cell = path[i]
    to_cell = path[i + 1]

    angle = get_move_direction(from_cell, to_cell)
    if angle is not None:
        turn(angle)
        robot.experiment_supervisor.step(robot.timestep)
        distance_moved = move_forward_1m()
        x, y = print_pose(path[i], path[i+1], x, y, distance_moved)

print("Goal Reached at Cell 17!")
robot.stop()


