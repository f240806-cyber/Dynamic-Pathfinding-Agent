# Dynamic Pathfinding Agent

A real-time pathfinding visualizer built with **Pygame** implementing **A\*** and **Greedy Best-First Search** on a dynamic grid environment.

## Features

- **A\* Search** and **Greedy Best-First Search (GBFS)**
- Three heuristics: Manhattan, Euclidean, Chebyshev
- Dynamic obstacle spawning during agent traversal with real-time re-planning
- Interactive map editor (click/drag to place walls, set start/goal)
- Random maze generation with configurable obstacle density
- Real-time visualization of frontier nodes, visited nodes, and final path
- Metrics dashboard: nodes visited, path cost, execution time

## Installation

```bash
pip install pygame
```

## Running

```bash
python main.py
```

## Controls

| Input | Action |
|-------|--------|
| Left Click (grid) | Place/remove wall / set start or goal |
| Left Click (panel) | Select algorithm, heuristic, edit mode |
| `SPACE` | Run search and animate |
| `M` | Move agent along path |
| `R` | Regenerate random map |
| `D` | Toggle dynamic obstacle mode |
| `1` | Switch to A* |
| `2` | Switch to Greedy BFS |

## Project Structure

```
pathfinding/
├── main.py        # Main application (algorithms + GUI)
└── README.md      # This file
```

## Algorithm Notes

- **A\***: Optimal and complete. Uses f(n) = g(n) + h(n).
- **GBFS**: Faster but not optimal. Uses f(n) = h(n) only.
- **Dynamic mode**: New walls spawn with ~2% probability per step; agent re-plans if path is blocked.
