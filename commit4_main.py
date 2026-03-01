#Add dynamic obstacles and agent re-planning
import random
import heapq
import math
import time
import pygame

EMPTY = 0
WALL = 1

pygame.init()
WIDTH, HEIGHT = 900, 700
PANEL_W = 220
GRID_W = WIDTH - PANEL_W
WHITE = (255,255,255)
BLACK = (20,20,20)
GRAY = (180,180,180)
GREEN = (50,200,100)
BLUE = (60,120,220)
YELLOW = (240,200,50)
ORANGE = (240,140,30)
PURPLE = (160,80,200)
CYAN = (50,200,220)
RED = (220,60,60)
BG = (15,15,25)
PANEL_BG = (25,25,40)
BTN = (50,50,80)
BTN_A = (80,60,140)
LIGHT = (230,230,230)

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Dynamic Pathfinding Agent")
clock = pygame.time.Clock()
font_sm = pygame.font.SysFont("consolas", 12)
font_md = pygame.font.SysFont("consolas", 14)
font_lg = pygame.font.SysFont("consolas", 16, bold=True)
font_xl = pygame.font.SysFont("consolas", 20, bold=True)

def heuristic(a, b, h_type):
    if h_type == "manhattan":
        return abs(a[0]-b[0]) + abs(a[1]-b[1])
    elif h_type == "euclidean":
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    elif h_type == "chebyshev":
        return max(abs(a[0]-b[0]), abs(a[1]-b[1]))

def get_neighbors(pos, grid, rows, cols):
    r, c = pos
    result = []
    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
        nr, nc = r+dr, c+dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] != WALL:
            result.append((nr, nc))
    return result

def reconstruct_path(came_from, current):
    path = []
    while current:
        path.append(current)
        current = came_from[current]
    return list(reversed(path))

def gbfs(grid, start, goal, rows, cols, h_type):
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
                heapq.heappush(open_set, (heuristic(nb, goal, h_type), nb))
                order.append(("frontier", nb))
    return None, visited, order

def astar(grid, start, goal, rows, cols, h_type):
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
                heapq.heappush(open_set, (tg + heuristic(nb, goal, h_type), nb))
                order.append(("frontier", nb))
    return None, visited, order

def generate_map(rows, cols, density, start, goal):
    grid = [[EMPTY]*cols for _ in range(rows)]
    for r in range(rows):
        for c in range(cols):
            if (r,c) != start and (r,c) != goal:
                if random.random() < density:
                    grid[r][c] = WALL
    return grid

def run_search(grid, start, goal, rows, cols, algorithm, h_type):
    t0 = time.time()
    if algorithm == "gbfs":
        path, vis, order = gbfs(grid, start, goal, rows, cols, h_type)
    else:
        path, vis, order = astar(grid, start, goal, rows, cols, h_type)
    exec_time = (time.time() - t0) * 1000
    return path, vis, order, exec_time

def spawn_dynamic_obstacle(grid, rows, cols, start, goal, agent_pos):
    r = random.randint(0, rows-1)
    c = random.randint(0, cols-1)
    if (r,c) != start and (r,c) != goal and (r,c) != agent_pos and grid[r][c] == EMPTY:
        grid[r][c] = WALL
        return (r, c)
    return None

def path_is_blocked(path, agent_idx, grid):
    remaining = path[agent_idx:]
    return any(grid[p[0]][p[1]] == WALL for p in remaining)

def draw_grid(screen, grid, rows, cols, cell_size, start, goal, visited, frontier, path, agent_pos):
    cs = cell_size
    surf = pygame.Surface((GRID_W, HEIGHT))
    surf.fill(BG)
    for r in range(rows):
        for c in range(cols):
            x, y = c*cs, r*cs
            pos = (r,c)
            if grid[r][c] == WALL:
                color = (50,50,60)
            elif pos == start:
                color = BLUE
            elif pos == goal:
                color = GREEN
            elif pos == agent_pos:
                color = ORANGE
            elif pos in frontier:
                color = YELLOW
            elif pos in visited:
                color = (80,60,100)
            else:
                color = (30,30,45)
            pygame.draw.rect(surf, color, (x+1,y+1,cs-2,cs-2), border_radius=2)
    for pos in path:
        r, c = pos
        pygame.draw.rect(surf, GREEN, (c*cs+1,r*cs+1,cs-2,cs-2), border_radius=2)
    if agent_pos:
        r, c = agent_pos
        pygame.draw.circle(surf, ORANGE, (c*cs+cs//2, r*cs+cs//2), cs//2-2)
    for r in range(rows+1):
        pygame.draw.line(surf, (40,40,55),(0,r*cs),(GRID_W,r*cs))
    for c in range(cols+1):
        pygame.draw.line(surf, (40,40,55),(c*cs,0),(c*cs,HEIGHT))
    screen.blit(surf, (0,0))

def draw_panel(screen, algorithm, h_type, edit_mode, dynamic_mode, status):
    panel = pygame.Surface((PANEL_W, HEIGHT))
    panel.fill(PANEL_BG)
    pygame.draw.line(panel,(60,60,100),(0,0),(0,HEIGHT),2)
    y = 10
    panel.blit(font_xl.render("PathFinder", True, CYAN),(10,y)); y+=30
    panel.blit(font_sm.render("Dynamic Agent v1.0", True, GRAY),(10,y)); y+=25
    pygame.draw.line(panel,(60,60,100),(5,y),(PANEL_W-5,y)); y+=10
    panel.blit(font_lg.render("ALGORITHM", True, LIGHT),(10,y)); y+=20
    for alg, lbl in [("astar","A* Search"),("gbfs","Greedy BFS")]:
        active = algorithm == alg
        r = pygame.Rect(10,y,PANEL_W-20,26)
        pygame.draw.rect(panel, BTN_A if active else BTN, r, border_radius=4)
        if active:
            pygame.draw.rect(panel, PURPLE, r, 1, border_radius=4)
        panel.blit(font_md.render(lbl, True, WHITE if active else GRAY),(r.x+8,r.y+5))
        y+=30
    y+=5
    panel.blit(font_lg.render("HEURISTIC", True, LIGHT),(10,y)); y+=20
    for h in ["manhattan","euclidean","chebyshev"]:
        active = h_type == h
        r = pygame.Rect(10,y,PANEL_W-20,26)
        pygame.draw.rect(panel, BTN_A if active else BTN, r, border_radius=4)
        if active:
            pygame.draw.rect(panel, CYAN, r, 1, border_radius=4)
        panel.blit(font_md.render(h.capitalize(), True, WHITE if active else GRAY),(r.x+8,r.y+5))
        y+=30
    y+=5
    panel.blit(font_lg.render("EDIT MODE", True, LIGHT),(10,y)); y+=20
    for mode in ["wall","start","goal"]:
        active = edit_mode == mode
        r = pygame.Rect(10,y,PANEL_W-20,26)
        pygame.draw.rect(panel, BTN_A if active else BTN, r, border_radius=4)
        panel.blit(font_md.render(mode.capitalize(), True, WHITE if active else GRAY),(r.x+8,r.y+5))
        y+=30
    y+=5
    pygame.draw.line(panel,(60,60,100),(5,y),(PANEL_W-5,y)); y+=8
    panel.blit(font_lg.render("CONTROLS", True, LIGHT),(10,y)); y+=20
    btn_rects = {}
    for lbl, col in [("Generate Map",GREEN),("Run & Animate",BLUE),("Move Agent",ORANGE),("Clear Search",GRAY)]:
        r = pygame.Rect(10,y,PANEL_W-20,28)
        pygame.draw.rect(panel, col, r, border_radius=5)
        t = font_md.render(lbl, True, BLACK)
        panel.blit(t,(r.x+(r.w-t.get_width())//2,r.y+(r.h-t.get_height())//2))
        btn_rects[lbl] = pygame.Rect(GRID_W+r.x, r.y, r.w, r.h)
        y+=34
    dyn_r = pygame.Rect(10,y,PANEL_W-20,28)
    pygame.draw.rect(panel,(50,150,80) if dynamic_mode else BTN, dyn_r, border_radius=5)
    panel.blit(font_md.render("Dynamic: "+("ON" if dynamic_mode else "OFF"), True, WHITE),(dyn_r.x+8,dyn_r.y+6))
    btn_rects["Dynamic"] = pygame.Rect(GRID_W+dyn_r.x, dyn_r.y, dyn_r.w, dyn_r.h)
    y+=38
    pygame.draw.line(panel,(60,60,100),(5,y),(PANEL_W-5,y)); y+=8
    panel.blit(font_lg.render("STATUS", True, LIGHT),(10,y)); y+=20
    col = GREEN if "Found" in status or "Reached" in status else RED if "No" in status else CYAN
    panel.blit(font_md.render(status, True, col),(10,y)); y+=25
    panel.blit(font_lg.render("LEGEND", True, LIGHT),(10,y)); y+=18
    for lbl, col in [("Start",BLUE),("Goal",GREEN),("Agent",ORANGE),("Frontier",YELLOW),("Visited",(80,60,100)),("Path",GREEN),("Wall",(50,50,60))]:
        pygame.draw.rect(panel, col,(10,y+2,12,12),border_radius=2)
        panel.blit(font_sm.render(lbl, True, GRAY),(28,y))
        y+=16
    screen.blit(panel,(GRID_W,0))
    return btn_rects

if __name__ == "__main__":
    rows, cols = 20, 25
    start, goal = (2,2), (17,22)
    cell_size = min(GRID_W//cols, HEIGHT//rows)
    grid = generate_map(rows, cols, 0.3, start, goal)
    algorithm = "astar"
    h_type = "manhattan"
    edit_mode = "wall"
    dynamic_mode = False
    spawn_prob = 0.02
    path, visited, frontier = [], set(), set()
    anim_order, anim_idx = [], 0
    animating = False
    anim_speed = 3
    agent_pos = None
    agent_idx = 0
    running_agent = False
    nodes_visited = path_cost = exec_time = 0
    status = "Ready"
    placing = None
    agent_timer = 0
    agent_delay = 100

    while True:
        dt = clock.tick(60)
        agent_timer += dt

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                if mx < GRID_W:
                    c = mx // cell_size
                    r = my // cell_size
                    if 0 <= r < rows and 0 <= c < cols:
                        pos = (r, c)
                        if edit_mode == "wall" and pos != start and pos != goal:
                            placing = WALL if grid[r][c] != WALL else EMPTY
                            grid[r][c] = placing
                            path, visited, frontier = [], set(), set()
                            status = "Ready"
                        elif edit_mode == "start" and pos != goal:
                            start = pos
                            path, visited, frontier = [], set(), set()
                            status = "Ready"
                        elif edit_mode == "goal" and pos != start:
                            goal = pos
                            path, visited, frontier = [], set(), set()
                            status = "Ready"
            if event.type == pygame.MOUSEMOTION:
                if event.buttons[0] and event.pos[0] < GRID_W and edit_mode == "wall" and placing is not None:
                    mx, my = event.pos
                    c = mx // cell_size
                    r = my // cell_size
                    if 0 <= r < rows and 0 <= c < cols and (r,c) != start and (r,c) != goal:
                        grid[r][c] = placing
            if event.type == pygame.MOUSEBUTTONUP:
                placing = None
            if event.type == pygame.MOUSEBUTTONDOWN:
                btn_rects = draw_panel(screen, algorithm, h_type, edit_mode, dynamic_mode, status)
                for lbl, r in btn_rects.items():
                    if r.collidepoint(event.pos):
                        if lbl == "Generate Map":
                            grid = generate_map(rows, cols, 0.3, start, goal)
                            path, visited, frontier, anim_order = [], set(), set(), []
                            agent_pos = None
                            running_agent = False
                            animating = False
                            status = "Ready"
                            nodes_visited = path_cost = exec_time = 0
                        elif lbl == "Run & Animate":
                            path, vis, order, exec_time = run_search(grid, start, goal, rows, cols, algorithm, h_type)
                            nodes_visited = len(vis)
                            path_cost = len(path)-1 if path else 0
                            anim_order = order
                            anim_idx = 0
                            visited, frontier = set(), set()
                            animating = True
                            running_agent = False
                            status = "Path Found" if path else "No Path!"
                        elif lbl == "Move Agent":
                            if not path:
                                path, vis, order, exec_time = run_search(grid, start, goal, rows, cols, algorithm, h_type)
                                nodes_visited = len(vis)
                                path_cost = len(path)-1 if path else 0
                                visited = vis
                            if path:
                                agent_pos = start
                                agent_idx = 0
                                running_agent = True
                                animating = False
                                status = "Agent Moving"
                        elif lbl == "Clear Search":
                            path, visited, frontier, anim_order = [], set(), set(), []
                            agent_pos = None
                            running_agent = animating = False
                            nodes_visited = path_cost = exec_time = 0
                            status = "Ready"
                        elif lbl == "Dynamic":
                            dynamic_mode = not dynamic_mode
                        elif lbl == "A* Search":
                            algorithm = "astar"
                            path, visited, frontier = [], set(), set()
                            status = "Ready"
                        elif lbl == "Greedy BFS":
                            algorithm = "gbfs"
                            path, visited, frontier = [], set(), set()
                            status = "Ready"
                        elif lbl in ["manhattan","euclidean","chebyshev"]:
                            h_type = lbl
                            path, visited, frontier = [], set(), set()
                            status = "Ready"
                        elif lbl in ["wall","start","goal"]:
                            edit_mode = lbl
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    grid = generate_map(rows, cols, 0.3, start, goal)
                    path, visited, frontier, anim_order = [], set(), set(), []
                    agent_pos = None
                    running_agent = animating = False
                    status = "Ready"
                    nodes_visited = path_cost = exec_time = 0
                if event.key == pygame.K_SPACE:
                    path, vis, order, exec_time = run_search(grid, start, goal, rows, cols, algorithm, h_type)
                    nodes_visited = len(vis)
                    path_cost = len(path)-1 if path else 0
                    anim_order = order
                    anim_idx = 0
                    visited, frontier = set(), set()
                    animating = True
                    status = "Path Found" if path else "No Path!"
                if event.key == pygame.K_m:
                    if not path:
                        path, vis, order, exec_time = run_search(grid, start, goal, rows, cols, algorithm, h_type)
                        nodes_visited = len(vis)
                        path_cost = len(path)-1 if path else 0
                        visited = vis
                    if path:
                        agent_pos = start
                        agent_idx = 0
                        running_agent = True
                        animating = False
                if event.key == pygame.K_d:
                    dynamic_mode = not dynamic_mode

        if animating:
            for _ in range(anim_speed):
                if anim_idx >= len(anim_order):
                    animating = False
                    break
                action, pos = anim_order[anim_idx]
                if action == "visit":
                    visited.add(pos)
                    frontier.discard(pos)
                elif action == "frontier":
                    if pos not in visited:
                        frontier.add(pos)
                anim_idx += 1

        if running_agent and agent_timer >= agent_delay:
            agent_timer = 0
            if agent_idx < len(path):
                agent_pos = path[agent_idx]
                agent_idx += 1
                if dynamic_mode and random.random() < spawn_prob:
                    new_wall = spawn_dynamic_obstacle(grid, rows, cols, start, goal, agent_pos)
                    if new_wall and path_is_blocked(path, agent_idx, grid):
                        new_path, new_vis, _, new_time = run_search(grid, agent_pos, goal, rows, cols, algorithm, h_type)
                        if new_path:
                            path = new_path
                            agent_idx = 1
                            visited = new_vis
                            nodes_visited = len(new_vis)
                            path_cost = len(path)-1
                            exec_time = new_time
                            status = "Re-planned!"
                        else:
                            running_agent = False
                            status = "No Path!"
            else:
                running_agent = False
                status = "Goal Reached!"

        draw_grid(screen, grid, rows, cols, cell_size, start, goal, visited, frontier, path, agent_pos)
        draw_panel(screen, algorithm, h_type, edit_mode, dynamic_mode, status)
        pygame.display.flip()
