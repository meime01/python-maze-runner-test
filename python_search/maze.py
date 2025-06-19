import sys
from collections import deque

class Node:
    def __init__(self, state, parent, action):
        self.state = state
        self.parent = parent
        self.action = action

class QueueFrontier:
    def __init__(self):
        self.frontier = deque()

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
            node = self.frontier.popleft()
            return node

def read_maze(file_path):
    with open(file_path, 'r') as file:
        maze = [list(line.strip()) for line in file]
    return maze

def find_path(maze, start, goal):
    # Find start and goal positions
    start_pos = find_position(maze, start)
    goal_pos = find_position(maze, goal)

    # Initialize frontier and explored set
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

def find_position(maze, char):
    for i, row in enumerate(maze):
        for j, val in enumerate(row):
            if val == char:
                return (i, j)
    raise Exception(f"Character '{char}' not found in the maze")

def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: python maze_solver.py maze.txt")

    maze = read_maze(sys.argv[1])
    try:
        path = find_path(maze, 'A', 'B')
        print("Path found:", path)
    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()
