"""
Compare UCS vs A* on the same maps.
Run from the project root:  python -m pathfinding.compare_algorithms
Uses the real hospital from main.py plus random mock maps.
"""
import math
import time

from scenarios.demo_environment import build_demo_environment
from pathfinding.search import find_path, find_path_env
from pathfinding import mock_grids as mg

N_MAPS, SIZE = 30, 40


def timed(fn, *args):
    t = time.perf_counter()
    path, cost, nodes = fn(*args)
    return (time.perf_counter() - t) * 1000, cost, nodes


def main():
    # --- Real hospital: person -> exit ---
    env = build_demo_environment()
    print("Real hospital (person -> exit):")
    print(f"{'person':>10}{'exit':>10}{'UCS nodes':>11}{'A* nodes':>10}{'cost':>6}")
    for person in env.get_people():
        for ex in env.get_exits():
            u = timed(find_path_env, env, person, ex, "ucs")
            a = timed(find_path_env, env, person, ex, "astar")
            if math.isinf(u[1]):
                continue
            assert u[1] == a[1]
            print(f"{str(person):>10}{str(ex):>10}{u[2]:>11}{a[2]:>10}{u[1]:>6}")

    # --- Random maps ---
    rows = []
    for seed in range(N_MAPS):
        g = mg.random_grid(SIZE, SIZE, seed=seed)
        s, t = (0, 0), (SIZE - 1, SIZE - 1)
        u, a = timed(find_path, g, s, t, "ucs"), timed(find_path, g, s, t, "astar")
        if math.isinf(u[1]):
            continue
        assert u[1] == a[1], "costs differ -> heuristic not admissible!"
        rows.append((u, a))

    n = len(rows)
    avg = lambda k, i: sum(r[k][i] for r in rows) / n
    print(f"\nRandom maps: {n} solvable of {N_MAPS} ({SIZE}x{SIZE})")
    print(f"{'':6}{'time(ms)':>10}{'nodes':>9}{'cost':>8}")
    print(f"{'UCS':6}{avg(0,0):>10.2f}{avg(0,2):>9.0f}{avg(0,1):>8.1f}")
    print(f"{'A*':6}{avg(1,0):>10.2f}{avg(1,2):>9.0f}{avg(1,1):>8.1f}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.5))
    for ax, (title, i) in zip(axes, [("Time (ms)", 0), ("Nodes explored", 2), ("Path cost", 1)]):
        ax.bar(["UCS", "A*"], [avg(0, i), avg(1, i)], color=["#888", "#2a9d8f"])
        ax.set_title(title)
    plt.tight_layout()
    plt.savefig("comparison.png", dpi=150)
    print("Saved chart: comparison.png")


if __name__ == "__main__":
    main()
