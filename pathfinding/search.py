"""
RESQ-AI | Pathfinding module (UCS + A*)

Two entry points:
  find_path_env(env, start, goal, algorithm)  -> works directly on the teammate's Environment
  find_path(grid, start, goal, algorithm)     -> works on a plain 2D list (for tests / mock grids)

Conventions (shared by the whole team):
  - grid[y][x], coordinates are (x, y), 4-directional movement
"""
import heapq
import math

# ---- Plain-grid cell values (only used by find_path on mock grids) ----
EMPTY, WALL, FIRE, SMOKE = 0, 1, 2, 3
COSTS = {EMPTY: 1, SMOKE: 5}      # cost of ENTERING a cell; anything else = impassable
MOVES = [(0, -1), (0, 1), (-1, 0), (1, 0)]


def cell_cost(value):
    """Cost of entering a plain-grid cell. math.inf means impassable."""
    return COSTS.get(value, math.inf)


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _search(start, goal, neighbors, step_cost, algorithm, min_cost=1):
    """
    Generic UCS / A* core.
      neighbors(node)  -> iterable of walkable neighbor nodes
      step_cost(node)  -> cost of stepping INTO node
      min_cost         -> smallest possible step cost (keeps the heuristic admissible)
    Returns (path, cost, nodes_explored).
    """
    if start == goal:
        return [start], 0, 0

    use_h = algorithm == "astar"
    h = (lambda n: manhattan(n, goal) * min_cost) if use_h else (lambda n: 0)

    counter = 0                        # tie-breaker for the heap
    open_heap = [(h(start), counter, start)]
    g_cost = {start: 0}
    parent = {start: None}
    closed = set()
    explored = 0

    while open_heap:
        _, _, node = heapq.heappop(open_heap)
        if node in closed:
            continue
        closed.add(node)
        explored += 1

        if node == goal:
            return _rebuild(parent, node), g_cost[node], explored

        for nxt in neighbors(node):
            new_g = g_cost[node] + step_cost(nxt)
            if nxt not in g_cost or new_g < g_cost[nxt]:
                g_cost[nxt] = new_g
                parent[nxt] = node
                counter += 1
                heapq.heappush(open_heap, (new_g + h(nxt), counter, nxt))

    return None, math.inf, explored


def _check_algorithm(algorithm):
    algorithm = algorithm.lower()
    if algorithm not in ("astar", "ucs"):
        raise ValueError("algorithm must be 'astar' or 'ucs'")
    return algorithm


# ------------------------------------------------------------------
# 1) Works directly on the Environment class (the real project)
# ------------------------------------------------------------------
def find_path_env(env, start, goal, algorithm="astar"):
    """
    Find a path on an Environment object.
    Uses env.is_walkable / env.get_neighbors / env.move_cost, so fire, obstacles
    and any future cost (smoke, ...) are respected automatically.

    Returns (path, cost, nodes_explored). path is None if there is no route.
    """
    algorithm = _check_algorithm(algorithm)
    if not env.is_walkable(*start) or not env.is_walkable(*goal):
        return None, math.inf, 0

    return _search(
        start, goal,
        neighbors=lambda n: env.get_neighbors(*n),
        step_cost=lambda n: env.move_cost(*n),
        algorithm=algorithm,
        min_cost=getattr(env, "min_move_cost", 1),
    )


def path_cost_env(env, start, goal, algorithm="astar"):
    """Cost only (for the CSP member: robot -> person distance)."""
    return find_path_env(env, start, goal, algorithm)[1]


def nearest_exit(env, start, algorithm="astar"):
    """Path to the cheapest reachable exit: (exit, path, cost), or (None, None, inf)."""
    return nearest_target(env, start, env.get_exits(), algorithm)


def nearest_target(env, start, targets, algorithm="astar"):
    """Cheapest reachable target among `targets`: (target, path, cost), or (None, None, inf)."""
    best = (None, None, math.inf)
    for t in targets:
        path, cost, _ = find_path_env(env, start, t, algorithm)
        if path is not None and cost < best[2]:
            best = (t, path, cost)
    return best


def cost_matrix(env, sources, targets, algorithm="astar"):
    """
    Path cost from every source to every target (for the CSP / task allocation member).
    Returns {(source, target): cost}. Unreachable pairs have cost math.inf.
    """
    return {(s, t): find_path_env(env, s, t, algorithm)[1]
            for s in sources for t in targets}


def is_path_valid(env, path):
    """
    True if every cell of `path` is still walkable (no new fire or obstacle on it).
    The Multi-Agent member calls this each step; when it returns False -> replan.
    """
    if not path:
        return False
    for a, b in zip(path, path[1:]):
        if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
            return False
    return all(env.is_walkable(*cell) for cell in path)


# ------------------------------------------------------------------
# 2) Works on a plain 2D list (mock grids / unit tests)
# ------------------------------------------------------------------
def find_path(grid, start, goal, algorithm="astar"):
    """Same as find_path_env, but on a plain grid[y][x] list using COSTS above."""
    algorithm = _check_algorithm(algorithm)
    h, w = len(grid), len(grid[0])

    def ok(n):
        x, y = n
        return 0 <= x < w and 0 <= y < h and cell_cost(grid[y][x]) != math.inf

    if not ok(start) or not ok(goal):
        return None, math.inf, 0

    def neighbors(n):
        return [(n[0] + dx, n[1] + dy) for dx, dy in MOVES if ok((n[0] + dx, n[1] + dy))]

    return _search(start, goal, neighbors,
                   lambda n: cell_cost(grid[n[1]][n[0]]), algorithm,
                   min_cost=min(COSTS.values()))


def path_cost(grid, start, goal, algorithm="astar"):
    return find_path(grid, start, goal, algorithm)[1]


def _rebuild(parent, node):
    path = []
    while node is not None:
        path.append(node)
        node = parent[node]
    path.reverse()
    return path
