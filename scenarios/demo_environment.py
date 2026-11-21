# demo_environment.py
# Demo scenario: a small hospital on fire.
# Run from the project root folder: python demo_environment.py

from environment.environment import Environment
from environment.building_layout import build_sample_hospital
from environment.cell import cell_symbol


def build_demo_environment():
    # seed=1 makes the fire spread the same way every time we run the demo
    env = Environment(20, 12, seed=1)
    build_sample_hospital(env.grid)

    # Emergency exits in the outer wall
    env.add_exit(0, 8)
    env.add_exit(19, 8)

    # People who need rescue: (x, y, priority)
    for x, y, priority in [(3, 2, 2), (9, 3, 3), (16, 2, 1), (4, 9, 1), (15, 10, 2)]:
        env.add_person(x, y, priority)

    # Where the rescue robots start
    env.add_robot_base(1, 8)
    env.add_robot_base(18, 8)

    # Fire and obstacles
    for x, y in [(8, 2), (9, 2), (8, 3)]:
        env.add_fire(x, y)
    for x, y in [(7, 8), (12, 7)]:
        env.add_obstacle(x, y)

    # Smoke around the door of the middle room (walkable, but costs 5)
    for x, y in [(9, 4), (9, 5), (9, 6)]:
        env.add_smoke(x, y)

    return env


def draw(env):
    """Print the environment: F = fire, O = obstacle, P = person, S = smoke, R = robot base."""
    bases = set(env.get_robot_bases())
    for y in range(env.height):
        row = []
        for x in range(env.width):
            if env.has_fire(x, y):
                row.append("F")
            elif env.has_obstacle(x, y):
                row.append("O")
            elif env.has_person(x, y):
                row.append("P")
            elif (x, y) in bases:
                row.append("R")
            elif env.has_smoke(x, y):
                row.append("S")
            else:
                row.append(cell_symbol(env.get_cell_type(x, y)))
        print(" ".join(row))


if __name__ == "__main__":
    env = build_demo_environment()
    draw(env)
    print()
    print("Exits:", env.get_exits())
    print("People (most urgent first):", env.get_people_by_priority())
    print("Robot bases:", env.get_robot_bases())
    print("Fire:", env.get_fire_zones())
    print("Obstacles:", env.get_obstacles())
    print("Smoke:", env.get_smoke_zones())
    print("Neighbors of (3, 2):", env.get_neighbors(3, 2))
    print("Cost into door (9, 5):", env.move_cost(9, 5))
    print("Cost into normal cell (3, 2):", env.move_cost(3, 2))

    # One simulation step: fire + smoke spread
    print()
    result = env.step(fire_chance=0.5)
    print("New fire cells:", result["new_fire"])
    print("New smoke cells:", result["new_smoke"])
    print("People hit by fire:", result["people_hit"])
    draw(env)
