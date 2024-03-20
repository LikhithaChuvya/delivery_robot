#!/usr/bin/env python
# coding: utf-8

# In[1]:


import heapq

class State:
    def __init__(self, robot_position, package_A_position, package_B_position, carrying_package_A, carrying_package_B):
        self.robot_position = robot_position
        self.package_A_position = package_A_position
        self.package_B_position = package_B_position
        self.carrying_package_A = carrying_package_A
        self.carrying_package_B = carrying_package_B
        self.parent = None  # To store the parent state for path tracing

    def __eq__(self, other):
        return (
            self.robot_position == other.robot_position and
            self.package_A_position == other.package_A_position and
            self.package_B_position == other.package_B_position and
            self.carrying_package_A == other.carrying_package_A and
            self.carrying_package_B == other.carrying_package_B
        )

    def __lt__(self, other):
        return True  # Allow comparison for heapq

    def __hash__(self):
        return hash((
            self.robot_position,
            self.package_A_position,
            self.package_B_position,
            self.carrying_package_A,
            self.carrying_package_B
        ))

    def __str__(self):
        return f"Robot: {self.robot_position}, Package A: {self.package_A_position}, Package B: {self.package_B_position}, Carrying A: {self.carrying_package_A}, Carrying B: {self.carrying_package_B}"

# Define grid dimensions
GRID_WIDTH = 10
GRID_HEIGHT = 10

# Define walls on the grid
walls = [(1, 1), (1, 4), (2, 1), (3, 1), (3, 2), (4, 3), (5, 3), (5, 4),(6,4),(7,0),(8,3),(0,9)]

# Function to print the grid along with walls
def print_grid_with_walls(robot_position, package_A_position, package_B_position, destination_A, destination_B):
    print("Grid with Walls:")
    for y in range(GRID_HEIGHT):
        for x in range(GRID_WIDTH):
            if (x, y) == robot_position:
                print("R", end=" ")
            elif (x, y) == package_A_position:
                print("A", end=" ")
            elif (x, y) == package_B_position:
                print("B", end=" ")
            elif (x, y) == destination_A:
                print("X", end=" ")
            elif (x, y) == destination_B:
                print("Y", end=" ")
            elif (x, y) in walls:
                print("#", end=" ")
            else:
                print(".", end=" ")
        print()

# Function to ask user for inputs
def get_user_input():
    # Print walls
    print("Walls:", walls)

    while True:
        # Ask for robot position
        robot_position = tuple(map(int, input("Enter robot position (x y): ").split()))
        if 0 <= robot_position[0] < GRID_WIDTH and 0 <= robot_position[1] < GRID_HEIGHT:
            break
        else:
            print("Invalid input! Coordinates must be within the range of 0 to 9.")

    while True:
        # Ask for package A position
        package_A_position = tuple(map(int, input("Enter package A position (x y): ").split()))
        if 0 <= package_A_position[0] < GRID_WIDTH and 0 <= package_A_position[1] < GRID_HEIGHT:
            break
        else:
            print("Invalid input! Coordinates must be within the range of 0 to 9.")

    while True:
        # Ask for package B position
        package_B_position = tuple(map(int, input("Enter package B position (x y): ").split()))
        if 0 <= package_B_position[0] < GRID_WIDTH and 0 <= package_B_position[1] < GRID_HEIGHT:
            break
        else:
            print("Invalid input! Coordinates must be within the range of 0 to 9.")

    while True:
        # Ask for destination A position
        destination_A = tuple(map(int, input("Enter destination A position (x y): ").split()))
        if 0 <= destination_A[0] < GRID_WIDTH and 0 <= destination_A[1] < GRID_HEIGHT:
            break
        else:
            print("Invalid input! Coordinates must be within the range of 0 to 9.")

    while True:
        # Ask for destination B position
        destination_B = tuple(map(int, input("Enter destination B position (x y): ").split()))
        if 0 <= destination_B[0] < GRID_WIDTH and 0 <= destination_B[1] < GRID_HEIGHT:
            break
        else:
            print("Invalid input! Coordinates must be within the range of 0 to 9.")

    return robot_position, package_A_position, package_B_position, destination_A, destination_B

# Define the goal test
def goal_test(state, destination_A, destination_B):
    if state.package_A_position is None or state.package_B_position is None:
        return False
    return state.package_A_position == destination_A and state.package_B_position == destination_B

# Define the heuristic function (Manhattan distance)
def heuristic(state, destination_A, destination_B):
    distance_A = abs(state.robot_position[0] - destination_A[0]) + abs(state.robot_position[1] - destination_A[1])
    distance_B = abs(state.robot_position[0] - destination_B[0]) + abs(state.robot_position[1] - destination_B[1])
    if state.carrying_package_A:
        distance_A = abs(state.robot_position[0] - state.package_A_position[0]) + abs(state.robot_position[1] - state.package_A_position[1])
    if state.carrying_package_B:
        distance_B = abs(state.robot_position[0] - state.package_B_position[0]) + abs(state.robot_position[1] - state.package_B_position[1])
    return distance_A + distance_B


# Define the A* search algorithm
def astar(start_state, destination_A, destination_B):
    visited = set()
    frontier = []
    heapq.heappush(frontier, (heuristic(start_state, destination_A, destination_B), start_state))

    while frontier:
        _, current_state = heapq.heappop(frontier)

        if goal_test(current_state, destination_A, destination_B):
            path = []
            while current_state:
                path.append(current_state)
                current_state = current_state.parent
            path.reverse()
            return path

        visited.add(current_state)

        # Generate next possible states
        for action in possible_actions(current_state, destination_A, destination_B):  # Pass destinations here
            next_state = apply_action(current_state, action)
            if next_state not in visited:
                next_state.parent = current_state  # Set parent for path tracing
                heapq.heappush(frontier, (heuristic(next_state, destination_A, destination_B), next_state))

    return None


# Define possible actions
def possible_actions(state, destination_A, destination_B):
    actions = []

    # Define possible movements: move up, down, left, right
    movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    for dx, dy in movements:
        new_x = state.robot_position[0] + dx
        new_y = state.robot_position[1] + dy
        if 0 <= new_x < GRID_WIDTH and 0 <= new_y < GRID_HEIGHT and (new_x, new_y) not in walls:
            actions.append(('MOVE', (new_x, new_y)))

    # Add pick up and drop off actions if the robot is at the package positions
    if state.robot_position == state.package_A_position and not state.carrying_package_A:
        actions.append(('PICK_UP', 'A'))
    if state.robot_position == state.package_B_position and not state.carrying_package_B:
        actions.append(('PICK_UP', 'B'))
    if state.robot_position == destination_A and state.carrying_package_A:
        actions.append(('DROP_OFF', 'A'))
    if state.robot_position == destination_B and state.carrying_package_B:
        actions.append(('DROP_OFF', 'B'))

    return actions


# Define function to apply action to current state and return next state
def apply_action(state, action):
    new_state = State(
        robot_position=state.robot_position,
        package_A_position=state.package_A_position,
        package_B_position=state.package_B_position,
        carrying_package_A=state.carrying_package_A,
        carrying_package_B=state.carrying_package_B
    )

    if action[0] == 'MOVE':
        new_state.robot_position = action[1]
    elif action[0] == 'PICK_UP':
        if action[1] == 'A' and not state.carrying_package_B:
            new_state.carrying_package_A = True
        elif action[1] == 'B' and not state.carrying_package_A:
            new_state.carrying_package_B = True
    elif action[0] == 'DROP_OFF':
        if action[1] == 'A':
            new_state.package_A_position = state.robot_position
            new_state.carrying_package_A = False
        elif action[1] == 'B':
            new_state.package_B_position = state.robot_position
            new_state.carrying_package_B = False

    return new_state

# Function to find the path to pick up the packages and deliver them to destinations
def find_path(robot_position, package_A_position, package_B_position, destination_A, destination_B):
    start_state = State(robot_position, package_A_position, package_B_position, False, False)
    path_A = astar(start_state, destination_A, destination_B)
    if path_A is None:
        print("No solution found for Package A to Destination A.")
        return
    path_B = astar(path_A[-1], destination_A, destination_B)
    if path_B is None:
        print("No solution found for Package B to Destination B.")
        return
    print("Path to pick up Package A and deliver to Destination A:")
    print_states(path_A)
    print("Path to pick up Package B and deliver to Destination B:")
    print_states(path_B)
    
# Function to print the states
def print_states(states):
    for state in states:
        print(state)

# Main function to drive the program
def main():
    # Ask user for inputs
    robot_position, package_A_position, package_B_position, destination_A, destination_B = get_user_input()

    # Print the grid with walls
    print_grid_with_walls(robot_position, package_A_position, package_B_position, destination_A, destination_B)

    # Find the path to pick up the packages and deliver them to destinations
    find_path(robot_position, package_A_position, package_B_position, destination_A, destination_B)

if __name__ == "__main__":
    main()


# In[ ]:




