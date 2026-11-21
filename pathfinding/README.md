# Pathfinding Module (UCS + A*)

Owner: Member 2. Works directly on the `Environment` class.

## Quick use

```python
from pathfinding import find_path_env, nearest_exit, cost_matrix, is_path_valid

path, cost, nodes = find_path_env(env, (3, 2), (0, 8), "astar")   # or "ucs"
exit_cell, path, cost = nearest_exit(env, (3, 2))
```

`path` is a list of `(x, y)` from start to goal, or `None` if there is no route (then `cost = math.inf`).

## Functions

| Function | Who uses it | What it does |
|---|---|---|
| `find_path_env(env, start, goal, algorithm)` | everyone | UCS / A* on the Environment. Returns `(path, cost, nodes_explored)` |
| `nearest_exit(env, start)` | Multi-Agent, Baseline | Cheapest reachable exit: `(exit, path, cost)` |
| `nearest_target(env, start, targets)` | Multi-Agent, Baseline | Same, for any list of targets |
| `cost_matrix(env, sources, targets)` | **CSP** | `{(robot_pos, person_pos): cost}` for task allocation |
| `is_path_valid(env, path)` | **Multi-Agent** | `False` if fire/obstacle now blocks the path -> replan |
| `path_cost_env(env, start, goal)` | CSP | Cost only |
| `find_path(grid, start, goal)` | tests | Same algorithms on a plain `grid[y][x]` list |

## How it connects to the Environment

Uses only `env.is_walkable`, `env.get_neighbors`, `env.move_cost`, `env.get_exits`, `env.min_move_cost`.
So fire, obstacles and smoke (cost 5) are respected automatically, and any future cost works if it is **>= 1**
(a lower cost would break the A* heuristic).

## Replanning loop (for Multi-Agent)

```python
if not is_path_valid(env, robot.path):
    robot.path, cost, _ = find_path_env(env, robot.pos, robot.goal)
```

## Compare UCS vs A*

```bash
python -m pathfinding.compare_algorithms     # prints a table and saves comparison.png
```

Both algorithms return the same optimal cost; A* explores far fewer nodes.
