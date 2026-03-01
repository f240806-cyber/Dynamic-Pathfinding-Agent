#Fix button click detection and edit mode toggling
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

class App:
    def __init__(self):
        self.rows = 20
        self.cols = 25
        self.start = (2, 2)
        self.goal = (17, 22)
        self.cell_size = min(GRID_W // self.cols, HEIGHT // self.rows)
        self.grid = generate_map(self.rows, self.cols, 0.3, self.start, self.goal)
        self.algorithm = "astar"
        self.h_type = "manhattan"
        self.edit_mode = "wall"
        self.dynamic_mode = False
        self.spawn_prob = 0.02
        self.path = []
        self.visited = set()
        self.frontier = set()
        self.anim_order = []
        self.anim_idx = 0
        self.animating = False
        self.anim_speed = 3
        self.agent_pos = None
        self.agent_idx = 0
        self.running_agent = False
        self.nodes_visited = 0
        self.path_cost = 0
        self.exec_time = 0.0
        self.status = "Ready"
        self.placing = None
        self.agent_timer = 0
        self.agent_delay = 100
        self.btn_rects = {}

    def reset_search(self):
        self.path = []
        self.visited = set()
        self.frontier = set()
        self.anim_order = []
        self.anim_idx = 0
        self.animating = False
        self.agent_pos = None
        self.agent_idx = 0
        self.running_agent = False
        self.nodes_visited = 0
        self.path_cost = 0
        self.exec_time = 0.0
        self.status = "Ready"

    def do_search(self, from_pos=None):
        src = from_pos if from_pos else self.start
        self.path, vis, order, self.exec_time = run_search(
            self.grid, src, self.goal, self.rows, self.cols, self.algorithm, self.h_type)
        self.nodes_visited = len(vis)
        self.path_cost = len(self.path)-1 if self.path else 0
        self.anim_order = order
        self.anim_idx = 0
        self.visited = vis
        self.frontier = set()
        self.status = "Path Found" if self.path else "No Path!"
        return self.path

    def handle_panel_click(self, pos):
        for lbl, rect in self.btn_rects.items():
            if rect.collidepoint(pos):
                if lbl == "Generate Map":
                    self.grid = generate_map(self.rows, self.cols, 0.3, self.start, self.goal)
                    self.reset_search()
                elif lbl == "Run & Animate":
                    self.do_search()
                    self.visited = set()
                    self.frontier = set()
                    self.animating = True
                    self.running_agent = False
                elif lbl == "Move Agent":
                    if not self.path:
                        self.do_search()
                    if self.path:
                        self.agent_pos = self.start
                        self.agent_idx = 0
                        self.running_agent = True
                        self.animating = False
                        self.status = "Agent Moving"
                elif lbl == "Clear Search":
                    self.reset_search()
                elif lbl == "Dynamic":
                    self.dynamic_mode = not self.dynamic_mode
                elif lbl == "A* Search":
                    self.algorithm = "astar"
                    self.reset_search()
                elif lbl == "Greedy BFS":
                    self.algorithm = "gbfs"
                    self.reset_search()
                elif lbl in ["manhattan", "euclidean", "chebyshev"]:
                    self.h_type = lbl
                    self.reset_search()
                elif lbl in ["wall", "start", "goal"]:
                    self.edit_mode = lbl

    def handle_grid_click(self, mx, my):
        c = mx // self.cell_size
        r = my // self.cell_size
        if not (0 <= r < self.rows and 0 <= c < self.cols):
            return
        pos = (r, c)
        if self.edit_mode == "wall" and pos != self.start and pos != self.goal:
            self.placing = WALL if self.grid[r][c] != WALL else EMPTY
            self.grid[r][c] = self.placing
            self.reset_search()
        elif self.edit_mode == "start" and pos != self.goal:
            self.start = pos
            self.reset_search()
        elif self.edit_mode == "goal" and pos != self.start:
            self.goal = pos
            self.reset_search()

    def handle_grid_drag(self, mx, my):
        if self.edit_mode != "wall" or self.placing is None:
            return
        c = mx // self.cell_size
        r = my // self.cell_size
        if 0 <= r < self.rows and 0 <= c < self.cols:
            if (r,c) != self.start and (r,c) != self.goal:
                self.grid[r][c] = self.placing

    def draw_panel(self):
        panel = pygame.Surface((PANEL_W, HEIGHT))
        panel.fill(PANEL_BG)
        pygame.draw.line(panel,(60,60,100),(0,0),(0,HEIGHT),2)
        y = 10
        panel.blit(font_xl.render("PathFinder", True, CYAN),(10,y)); y+=30
        panel.blit(font_sm.render("Dynamic Agent v1.0", True, GRAY),(10,y)); y+=25
        pygame.draw.line(panel,(60,60,100),(5,y),(PANEL_W-5,y)); y+=10

        panel.blit(font_lg.render("ALGORITHM", True, LIGHT),(10,y)); y+=20
        for alg, lbl in [("astar","A* Search"),("gbfs","Greedy BFS")]:
            active = self.algorithm == alg
            r = pygame.Rect(10,y,PANEL_W-20,26)
            pygame.draw.rect(panel, BTN_A if active else BTN, r, border_radius=4)
            if active:
                pygame.draw.rect(panel, PURPLE, r, 1, border_radius=4)
            panel.blit(font_md.render(lbl, True, WHITE if active else GRAY),(r.x+8,r.y+5))
            self.btn_rects[lbl] = pygame.Rect(GRID_W+r.x, r.y, r.w, r.h)
            y+=30

        y+=5
        panel.blit(font_lg.render("HEURISTIC", True, LIGHT),(10,y)); y+=20
        for h in ["manhattan","euclidean","chebyshev"]:
            active = self.h_type == h
            r = pygame.Rect(10,y,PANEL_W-20,26)
            pygame.draw.rect(panel, BTN_A if active else BTN, r, border_radius=4)
            if active:
                pygame.draw.rect(panel, CYAN, r, 1, border_radius=4)
            panel.blit(font_md.render(h.capitalize(), True, WHITE if active else GRAY),(r.x+8,r.y+5))
            self.btn_rects[h] = pygame.Rect(GRID_W+r.x, r.y, r.w, r.h)
            y+=30

        y+=5
        panel.blit(font_lg.render("EDIT MODE", True, LIGHT),(10,y)); y+=20
        for mode in ["wall","start","goal"]:
            active = self.edit_mode == mode
            r = pygame.Rect(10,y,PANEL_W-20,26)
            pygame.draw.rect(panel, BTN_A if active else BTN, r, border_radius=4)
            panel.blit(font_md.render(mode.capitalize(), True, WHITE if active else GRAY),(r.x+8,r.y+5))
            self.btn_rects[mode] = pygame.Rect(GRID_W+r.x, r.y, r.w, r.h)
            y+=30

        y+=5
        pygame.draw.line(panel,(60,60,100),(5,y),(PANEL_W-5,y)); y+=8
        panel.blit(font_lg.render("CONTROLS", True, LIGHT),(10,y)); y+=20
        for lbl, col in [("Generate Map",GREEN),("Run & Animate",BLUE),("Move Agent",ORANGE),("Clear Search",GRAY)]:
            r = pygame.Rect(10,y,PANEL_W-20,28)
            pygame.draw.rect(panel, col, r, border_radius=5)
            t = font_md.render(lbl, True, BLACK)
            panel.blit(t,(r.x+(r.w-t.get_width())//2,r.y+(r.h-t.get_height())//2))
            self.btn_rects[lbl] = pygame.Rect(GRID_W+r.x, r.y, r.w, r.h)
            y+=34

        dyn_r = pygame.Rect(10,y,PANEL_W-20,28)
        pygame.draw.rect(panel,(50,150,80) if self.dynamic_mode else BTN, dyn_r, border_radius=5)
        panel.blit(font_md.render("Dynamic: "+("ON" if self.dynamic_mode else "OFF"), True, WHITE),(dyn_r.x+8,dyn_r.y+6))
        self.btn_rects["Dynamic"] = pygame.Rect(GRID_W+dyn_r.x, dyn_r.y, dyn_r.w, dyn_r.h)
        y+=38

        pygame.draw.line(panel,(60,60,100),(5,y),(PANEL_W-5,y)); y+=8
        panel.blit(font_lg.render("METRICS", True, LIGHT),(10,y)); y+=20
        for label, val, col in [
            ("Nodes", str(self.nodes_visited), CYAN),
            ("Path Cost", str(self.path_cost), CYAN),
            ("Time", f"{self.exec_time:.1f}ms", CYAN),
            ("Status", self.status, GREEN if "Found" in self.status or "Reached" in self.status else RED if "No" in self.status else YELLOW)
        ]:
            panel.blit(font_sm.render(label+":", True, GRAY),(10,y))
            panel.blit(font_sm.render(val, True, col),(130,y))
            y+=18

        y+=8
        pygame.draw.line(panel,(60,60,100),(5,y),(PANEL_W-5,y)); y+=8
        panel.blit(font_lg.render("LEGEND", True, LIGHT),(10,y)); y+=18
        for lbl, col in [("Start",BLUE),("Goal",GREEN),("Agent",ORANGE),("Frontier",YELLOW),("Visited",(80,60,100)),("Path",GREEN),("Wall",(50,50,60))]:
            pygame.draw.rect(panel, col,(10,y+2,12,12),border_radius=2)
            panel.blit(font_sm.render(lbl, True, GRAY),(28,y))
            y+=16

        y+=8
        panel.blit(font_sm.render("SPACE=Run  M=Agent", True, GRAY),(10,y)); y+=16
        panel.blit(font_sm.render("R=Regen  D=Dynamic", True, GRAY),(10,y))
        screen.blit(panel,(GRID_W,0))

    def update_animation(self):
        if not self.animating:
            return
        for _ in range(self.anim_speed):
            if self.anim_idx >= len(self.anim_order):
                self.animating = False
                break
            action, pos = self.anim_order[self.anim_idx]
            if action == "visit":
                self.visited.add(pos)
                self.frontier.discard(pos)
            elif action == "frontier":
                if pos not in self.visited:
                    self.frontier.add(pos)
            self.anim_idx += 1

    def update_agent(self):
        if not self.running_agent:
            return
        if self.agent_idx < len(self.path):
            self.agent_pos = self.path[self.agent_idx]
            self.agent_idx += 1
            if self.dynamic_mode and random.random() < self.spawn_prob:
                new_wall = spawn_dynamic_obstacle(self.grid, self.rows, self.cols, self.start, self.goal, self.agent_pos)
                if new_wall and path_is_blocked(self.path, self.agent_idx, self.grid):
                    new_path = self.do_search(from_pos=self.agent_pos)
                    if new_path:
                        self.agent_idx = 1
                        self.status = "Re-planned!"
                    else:
                        self.running_agent = False
        else:
            self.running_agent = False
            self.status = "Goal Reached!"

    def run(self):
        while True:
            dt = clock.tick(60)
            self.agent_timer += dt

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = event.pos
                    if mx < GRID_W:
                        self.handle_grid_click(mx, my)
                    else:
                        self.handle_panel_click(event.pos)
                if event.type == pygame.MOUSEMOTION:
                    if event.buttons[0] and event.pos[0] < GRID_W:
                        self.handle_grid_drag(event.pos[0], event.pos[1])
                if event.type == pygame.MOUSEBUTTONUP:
                    self.placing = None
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        self.grid = generate_map(self.rows, self.cols, 0.3, self.start, self.goal)
                        self.reset_search()
                    if event.key == pygame.K_SPACE:
                        self.do_search()
                        self.visited = set()
                        self.frontier = set()
                        self.animating = True
                    if event.key == pygame.K_m:
                        if not self.path:
                            self.do_search()
                        if self.path:
                            self.agent_pos = self.start
                            self.agent_idx = 0
                            self.running_agent = True
                            self.animating = False
                    if event.key == pygame.K_d:
                        self.dynamic_mode = not self.dynamic_mode

            self.update_animation()

            if self.agent_timer >= self.agent_delay:
                self.agent_timer = 0
                self.update_agent()

            draw_grid(screen, self.grid, self.rows, self.cols, self.cell_size,
                      self.start, self.goal, self.visited, self.frontier, self.path, self.agent_pos)
            self.draw_panel()
            pygame.display.flip()

if __name__ == "__main__":
    App().run()
