import sys
import heapq
from collections import deque

class Node:
    def __init__(self, state, parent, action, heuristic=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.heuristic = heuristic
    
    def __lt__(self, other):
        return self.heuristic < other.heuristic

class StackFrontier:
    def __init__(self):
        self.frontier = []

    def add(self, node):
        self.frontier.append(node)

    def contains_state(self, state):
        return any(node.state == state for node in self.frontier)

    def empty(self):
        return len(self.frontier) == 0

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[-1]
            self.frontier = self.frontier[:-1]
            return node

class QueueFrontier(StackFrontier):
    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[0]
            self.frontier = self.frontier[1:]
            return node

class GreedyFrontier:
    def __init__(self):
        self.frontier = []
        self.frontier_states = set()
    
    def add(self, node):
        heapq.heappush(self.frontier, node)
        self.frontier_states.add(node.state)
    
    def contains_state(self, state):
        return state in self.frontier_states
    
    def empty(self):
        return len(self.frontier) == 0
    
    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = heapq.heappop(self.frontier)
            self.frontier_states.remove(node.state)
            return node

def manhattan_distance(pos1, pos2):
    """Calculate Manhattan distance between two positions"""
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

def euclidean_distance(pos1, pos2):
    """Calculate Euclidean distance between two positions"""
    return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5

def read_maze(file_path):
    with open(file_path, 'r') as file:
        maze = [list(line.strip()) for line in file]
    return maze

def find_path_uniform(maze, start, goal):
    """Original uniform cost search (BFS)"""
    start_pos = find_position(maze, start)
    goal_pos = find_position(maze, goal)
    
    frontier = QueueFrontier()
    frontier.add(Node(state=start_pos, parent=None, action=None))
    explored = set()

    while True:
        if frontier.empty():
            raise Exception("no solution")

        node = frontier.remove()

        if node.state == goal_pos:
            path = []
            while node.parent is not None:
                path.append(node.action)
                node = node.parent
            path.reverse()
            return path

        explored.add(node.state)

        for action, (di, dj) in [("up", (-1, 0)), ("down", (1, 0)), ("left", (0, -1)), ("right", (0, 1))]:
            i, j = node.state
            next_i, next_j = i + di, j + dj
            
            if 0 <= next_i < len(maze) and 0 <= next_j < len(maze[0]) and maze[next_i][next_j] != '#':
                next_state = (next_i, next_j)
                if not frontier.contains_state(next_state) and next_state not in explored:
                    child = Node(state=next_state, parent=node, action=action)
                    frontier.add(child)

def find_path_greedy(maze, start, goal, heuristic_func=manhattan_distance):
    """Greedy Best-First Search using heuristic function"""
    start_pos = find_position(maze, start)
    goal_pos = find_position(maze, goal)
    
    # Initialize frontier with greedy frontier (priority queue)
    frontier = GreedyFrontier()
    start_heuristic = heuristic_func(start_pos, goal_pos)
    frontier.add(Node(state=start_pos, parent=None, action=None, heuristic=start_heuristic))
    explored = set()
    
    nodes_expanded = 0  # Counter for analysis

    while True:
        if frontier.empty():
            raise Exception("no solution")

        node = frontier.remove()
        nodes_expanded += 1

        if node.state == goal_pos:
            path = []
            while node.parent is not None:
                path.append(node.action)
                node = node.parent
            path.reverse()
            print(f"Nodes expanded: {nodes_expanded}")
            return path

        explored.add(node.state)

        # Explore neighbors
        for action, (di, dj) in [("up", (-1, 0)), ("down", (1, 0)), ("left", (0, -1)), ("right", (0, 1))]:
            i, j = node.state
            next_i, next_j = i + di, j + dj
            
            # Check if the move is valid
            if 0 <= next_i < len(maze) and 0 <= next_j < len(maze[0]) and maze[next_i][next_j] != '#':
                next_state = (next_i, next_j)
                
                # Add to frontier if not already explored or in frontier
                if not frontier.contains_state(next_state) and next_state not in explored:
                    heuristic_value = heuristic_func(next_state, goal_pos)
                    child = Node(state=next_state, parent=node, action=action, heuristic=heuristic_value)
                    frontier.add(child)

def find_position(maze, char):
    for i, row in enumerate(maze):
        for j, val in enumerate(row):
            if val == char:
                return (i, j)
    raise Exception(f"Character '{char}' not found in the maze")

def print_maze_with_path(maze, path):
    """Visualize the solution path on the maze"""
    # Create a copy of the maze
    maze_copy = [row[:] for row in maze]
    
    # Find start position
    start_pos = find_position(maze, 'A')
    current_pos = start_pos
    
    # Mark the path
    for action in path:
        if action == "up":
            current_pos = (current_pos[0] - 1, current_pos[1])
        elif action == "down":
            current_pos = (current_pos[0] + 1, current_pos[1])
        elif action == "left":
            current_pos = (current_pos[0], current_pos[1] - 1)
        elif action == "right":
            current_pos = (current_pos[0], current_pos[1] + 1)
        
        # Mark path with * (but don't overwrite A or B)
        if maze_copy[current_pos[0]][current_pos[1]] not in ['A', 'B']:
            maze_copy[current_pos[0]][current_pos[1]] = '*'
    
    # Print the maze with path
    for row in maze_copy:
        print(''.join(row))

def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: python maze_solver.py maze.txt")

    maze = read_maze(sys.argv[1])
    
    try:
        print("=== Uniform Cost Search (BFS) ===")
        path_uniform = find_path_uniform(maze, 'A', 'B')
        print("Path found:", path_uniform)
        print("Path length:", len(path_uniform))
        print()
        
        print("=== Greedy Best-First Search (Manhattan Distance) ===")
        path_greedy_manhattan = find_path_greedy(maze, 'A', 'B', manhattan_distance)
        print("Path found:", path_greedy_manhattan)
        print("Path length:", len(path_greedy_manhattan))
        print()
        
        print("=== Greedy Best-First Search (Euclidean Distance) ===")
        path_greedy_euclidean = find_path_greedy(maze, 'A', 'B', euclidean_distance)
        print("Path found:", path_greedy_euclidean)
        print("Path length:", len(path_greedy_euclidean))
        print()
        
        print("=== Maze with Greedy Best-First Solution (Manhattan) ===")
        print_maze_with_path(maze, path_greedy_manhattan)
        
    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()