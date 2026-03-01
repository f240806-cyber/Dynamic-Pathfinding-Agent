#Add grid setup and random map generation
import random

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

if __name__ == "__main__":
    rows, cols = 20, 25
    start = (2, 2)
    goal = (17, 22)
    grid = generate_map(rows, cols, 0.3, start, goal)
    print(f"Grid created: {rows}x{cols}")
    print(f"Start: {start}, Goal: {goal}")
    print(f"Walls: {sum(grid[r][c] == WALL for r in range(rows) for c in range(cols))}")
