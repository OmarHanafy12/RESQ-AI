# scenarios.py
# Random scenarios on the sample hospital, for the evaluation experiments.
# The same seed always gives the same scenario, so the baseline system and the
# AI system can be tested on exactly the same emergency.

import random

from environment.environment import Environment
from environment.building_layout import build_sample_hospital
from environment.constants import FLOOR

EXITS = [(0, 8), (19, 8)]
ROBOT_BASES = [(1, 8), (18, 8), (2, 8), (17, 8)]
DOORS = [(3, 5), (9, 5), (16, 5), (10, 8)]   # never blocked at the start


def build_random_scenario(seed=0, n_people=6, n_fires=2, n_obstacles=3,
                          n_smoke=0, n_robots=2):
    """Return a ready Environment (20 x 12 hospital). Same seed = same scenario."""
    rng = random.Random(seed)
    env = Environment(20, 12, seed=seed)
    build_sample_hospital(env.grid)

    for x, y in EXITS:
        env.add_exit(x, y)

    reserved = set(DOORS) | set(ROBOT_BASES)
    free = [(x, y)
            for y in range(env.height) for x in range(env.width)
            if env.get_cell_type(x, y) == FLOOR and (x, y) not in reserved]
    rng.shuffle(free)

    def take(n):
        if n > len(free):
            raise ValueError("Not enough free cells for this scenario")
        return [free.pop() for _ in range(n)]

    for x, y in take(n_people):
        env.add_person(x, y, priority=rng.choice([1, 2, 3]))
    for x, y in take(n_fires):
        env.add_fire(x, y)
    for x, y in take(n_obstacles):
        env.add_obstacle(x, y)
    for x, y in take(n_smoke):
        env.add_smoke(x, y)
    for x, y in ROBOT_BASES[:n_robots]:
        env.add_robot_base(x, y)

    return env
