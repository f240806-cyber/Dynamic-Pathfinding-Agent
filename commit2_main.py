#Implement A* and Greedy BFS algorithms
import random
import heapq
import math

EMPTY = 0
WALL = 1
START = 2
GOAL = 3

def create_grid(rows, cols):
    return [[EMPTY] * cols for _ in range(rows)]

def generate_map(rows, cols, density, start, goal):
    grid = create_grid(rows, cols)
    for r in range(rows):
        for c in range(cols):
            if (r, c) != start and (r, c) != goal:
                if random.random() < density:
                    grid[r][c] = WALL
    return grid

def get_neighbors(pos, grid, rows, cols):
    r, c = pos
    dirs = [(-1,0),(1,0),(0,-1),(0,1)]
    result = []
    for dr, dc in dirs:
        nr, nc = r+dr, c+dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] != WALL:
            result.append((nr, nc))
    return result

def heuristic(a, b, h_type):
    if h_type == "manhattan":
        return abs(a[0]-b[0]) + abs(a[1]-b[1])
    elif h_type == "euclidean":
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    elif h_type == "chebyshev":
        return max(abs(a[0]-b[0]), abs(a[1]-b[1]))

def reconstruct_path(came_from, current):
    path = []
    while current:
        path.append(current)
        current = came_from[current]
    return list(reversed(path))

def gbfs(grid, start, goal, rows, cols, h_type="manhattan"):
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal, h_type), start))
    came_from = {start: None}
    visited = set()
    order = []
    while open_set:
        _, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)
        order.append(("visit", current))
        if current == goal:
            return reconstruct_path(came_from, current), visited, order
        for nb in get_neighbors(current, grid, rows, cols):
            if nb not in visited and nb not in came_from:
                came_from[nb] = current
                h = heuristic(nb, goal, h_type)
                heapq.heappush(open_set, (h, nb))
                order.append(("frontier", nb))
    return None, visited, order

def astar(grid, start, goal, rows, cols, h_type="manhattan"):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {start: None}
    g_score = {start: 0}
    visited = set()
    order = []
    while open_set:
        _, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)
        order.append(("visit", current))
        if current == goal:
            return reconstruct_path(came_from, current), visited, order
        for nb in get_neighbors(current, grid, rows, cols):
            tg = g_score[current] + 1
            if nb not in g_score or tg < g_score[nb]:
                came_from[nb] = current
                g_score[nb] = tg
                f = tg + heuristic(nb, goal, h_type)
                heapq.heappush(open_set, (f, nb))
                order.append(("frontier", nb))
    return None, visited, order

if __name__ == "__main__":
    rows, cols = 20, 25
    start = (2, 2)
    goal = (17, 22)
    grid = generate_map(rows, cols, 0.3, start, goal)
    path, visited, _ = astar(grid, start, goal, rows, cols, "manhattan")
    print(f"A* Path length: {len(path)-1 if path else 'No path'}")
    print(f"A* Nodes visited: {len(visited)}")
    path2, visited2, _ = gbfs(grid, start, goal, rows, cols, "manhattan")
    print(f"GBFS Path length: {len(path2)-1 if path2 else 'No path'}")
    print(f"GBFS Nodes visited: {len(visited2)}")
