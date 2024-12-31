from queue import Queue, LifoQueue, PriorityQueue
import math

# Directions: Up, Right, Down, Left
directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # URDL (Up, Right, Down, Left)

# Helper function to find start and goal positions in the map
def find_positions(maze):
    start, goal = None, None
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            if maze[i][j] == 's':
                start = (i, j)
            elif maze[i][j] == 'g':
                goal = (i, j)
    return start, goal

# Check if the given (x, y) is within the bounds and is not an obstacle
def is_valid(maze, x, y):
    return 0 <= x < len(maze) and 0 <= y < len(maze[0]) and maze[x][y] != 'o'

# BFS algorithm
def bfs(maze):
    start, goal = find_positions(maze)
    q = Queue()
    q.put((start, []))
    visited = set()
    visited.add(start)

    while not q.empty():
        (x, y), path = q.get()
        if (x, y) == goal:
            return path + [(x, y)]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if is_valid(maze, nx, ny) and (nx, ny) not in visited:
                visited.add((nx, ny))
                q.put(((nx, ny), path + [(x, y)]))
    
    return None  # No path found

# DFS algorithm
def dfs(maze):
    start, goal = find_positions(maze)
    stack = LifoQueue()
    stack.put((start, []))
    visited = set()
    visited.add(start)

    while not stack.empty():
        (x, y), path = stack.get()
        if (x, y) == goal:
            return path + [(x, y)]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if is_valid(maze, nx, ny) and (nx, ny) not in visited:
                visited.add((nx, ny))
                stack.put(((nx, ny), path + [(x, y)]))
    
    return None  # No path found

# Manhattan distance heuristic
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* algorithm
def a_star(maze):
    start, goal = find_positions(maze)
    pq = PriorityQueue()
    pq.put((0, start, []))
    visited = set()
    visited.add(start)

    while not pq.empty():
        cost, (x, y), path = pq.get()
        if (x, y) == goal:
            return path + [(x, y)]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if is_valid(maze, nx, ny) and (nx, ny) not in visited:
                visited.add((nx, ny))
                g = len(path) + 1
                h = manhattan((nx, ny), goal)
                f = g + h
                pq.put((f, (nx, ny), path + [(x, y)]))
    
    return None  # No path found

# IDA* algorithm
def ida_star(maze):
    start, goal = find_positions(maze)

    def search(path, g, bound):
        node = path[-1]
        f = g + manhattan(node, goal)
        if f > bound:
            return f
        if node == goal:
            return path
        min_bound = math.inf
        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if is_valid(maze, nx, ny) and (nx, ny) not in path:
                path.append((nx, ny))
                result = search(path, g + 1, bound)
                if isinstance(result, list):
                    return result
                if result < min_bound:
                    min_bound = result
                path.pop()
        return min_bound

    bound = manhattan(start, goal)
    path = [start]
    while True:
        result = search(path, 0, bound)
        if isinstance(result, list):
            return result
        if result == math.inf:
            return None
        bound = result

# Example maze
maze = [
    ['s', ' ', 'o', ' ', ' ', ' '],
    [' ', 'o', ' ', 'o', ' ', ' '],
    [' ', ' ', ' ', ' ', 'o', ' '],
    [' ', 'o', 'o', ' ', ' ', 'g'],
    [' ', ' ', ' ', ' ', ' ', ' ']
]

# Choose algorithm to run (bfs, dfs, a_star, ida_star)
print("BFS Path:", bfs(maze))
print("DFS Path:", dfs(maze))
print("A* Path:", a_star(maze))
print("IDA* Path:", ida_star(maze))
