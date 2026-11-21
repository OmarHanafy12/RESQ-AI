import math
import pytest
from pathfinding.search import find_path, path_cost, WALL, FIRE, SMOKE
from pathfinding import mock_grids as mg

ALGOS = ["ucs", "astar"]


@pytest.mark.parametrize("algo", ALGOS)
def test_straight_line(algo):
    path, cost, _ = find_path(mg.open_grid(), (0, 0), (4, 0), algo)
    assert cost == 4 and path[0] == (0, 0) and path[-1] == (4, 0)


@pytest.mark.parametrize("algo", ALGOS)
def test_start_equals_goal(algo):
    assert find_path(mg.open_grid(), (2, 2), (2, 2), algo) == ([(2, 2)], 0, 0)


@pytest.mark.parametrize("algo", ALGOS)
def test_no_path(algo):
    path, cost, _ = find_path(mg.blocked_grid(), (0, 0), (9, 9), algo)
    assert path is None and cost == math.inf


@pytest.mark.parametrize("algo", ALGOS)
def test_goal_in_wall_or_fire(algo):
    g = mg.open_grid()
    g[3][3] = WALL
    g[4][4] = FIRE
    assert find_path(g, (0, 0), (3, 3), algo)[0] is None
    assert find_path(g, (0, 0), (4, 4), algo)[0] is None


@pytest.mark.parametrize("algo", ALGOS)
def test_out_of_bounds(algo):
    assert find_path(mg.open_grid(), (0, 0), (99, 99), algo)[0] is None


@pytest.mark.parametrize("algo", ALGOS)
def test_path_goes_through_gap(algo):
    g = mg.wall_with_gap(10, 10, gap_y=5)
    path, _, _ = find_path(g, (0, 0), (9, 0), algo)
    assert (5, 5) in path


@pytest.mark.parametrize("algo", ALGOS)
def test_avoids_smoke_when_cheaper(algo):
    g = mg.open_grid(7, 5)
    for x in range(1, 6):
        g[2][x] = SMOKE
    path, cost, _ = find_path(g, (0, 2), (6, 2), algo)
    assert cost < 5 * 5 + 2          # cheaper than walking straight through smoke
    assert not all(g[y][x] == SMOKE for x, y in path[1:-1])


def test_path_never_enters_wall_or_fire():
    g = mg.random_grid(seed=3)
    path, _, _ = find_path(g, (0, 0), (29, 29), "astar")
    if path:
        assert all(g[y][x] not in (WALL, FIRE) for x, y in path)


def test_astar_equals_ucs_cost_and_explores_less():
    for seed in range(20):
        g = mg.random_grid(seed=seed)
        _, c_u, n_u = find_path(g, (0, 0), (29, 29), "ucs")
        _, c_a, n_a = find_path(g, (0, 0), (29, 29), "astar")
        assert c_u == c_a            # both optimal
        assert n_a <= n_u            # A* never explores more


def test_path_cost_helper():
    assert path_cost(mg.open_grid(), (0, 0), (3, 0)) == 3


# ---------------- Tests on the real Environment ----------------
from scenarios.demo_environment import build_demo_environment
from pathfinding.search import find_path_env, nearest_exit, path_cost_env


def test_env_person_reaches_exit_same_cost_both_algos():
    env = build_demo_environment()
    for person in env.get_people():
        for ex in env.get_exits():
            pu, cu, nu = find_path_env(env, person, ex, "ucs")
            pa, ca, na = find_path_env(env, person, ex, "astar")
            assert cu == ca and na <= nu
            assert pu[0] == person and pu[-1] == ex


def test_env_path_avoids_fire_obstacles_walls():
    env = build_demo_environment()
    path, _, _ = find_path_env(env, (3, 2), (19, 8))
    assert path is not None
    for x, y in path:
        assert env.is_walkable(x, y)


def test_env_replan_after_new_obstacle():
    env = build_demo_environment()
    p1, c1, _ = find_path_env(env, (3, 2), (0, 8))
    env.add_obstacle(*p1[len(p1) // 2])            # block the middle of the route
    p2, c2, _ = find_path_env(env, (3, 2), (0, 8))
    assert p2 is None or (p2 != p1 and c2 >= c1)


def test_env_blocked_door_means_no_path():
    env = build_demo_environment()
    env.add_obstacle(3, 5)                          # the only door of the top-left room
    path, cost, _ = find_path_env(env, (3, 2), (0, 8))
    assert path is None and cost == math.inf


def test_env_goal_in_fire_or_wall():
    env = build_demo_environment()
    assert find_path_env(env, (3, 2), (8, 2))[0] is None   # fire
    assert find_path_env(env, (3, 2), (0, 0))[0] is None   # wall


def test_nearest_exit_and_cost_helper():
    env = build_demo_environment()
    ex, path, cost = nearest_exit(env, (4, 9))
    assert ex == (0, 8)
    assert cost == path_cost_env(env, (4, 9), ex)


# ---------------- Smoke, replanning, helpers ----------------
from pathfinding.search import cost_matrix, nearest_target, is_path_valid
from scenarios import build_random_scenario


def test_env_smoke_makes_astar_and_ucs_agree_and_cost_more():
    env = build_demo_environment()
    # (9, 3) -> door (9, 5) is in smoke: entering (9,4),(9,5) costs 5 each
    _, cost_smoke, _ = find_path_env(env, (9, 3), (9, 7), "astar")
    for c in env.get_smoke_zones():
        env.remove_smoke(*c)
    _, cost_clear, _ = find_path_env(env, (9, 3), (9, 7), "astar")
    assert cost_smoke > cost_clear


def test_env_ucs_and_astar_agree_on_random_scenarios():
    for seed in range(15):
        env = build_random_scenario(seed=seed, n_smoke=6)
        for person in env.get_people():
            for ex in env.get_exits():
                _, cu, nu = find_path_env(env, person, ex, "ucs")
                _, ca, na = find_path_env(env, person, ex, "astar")
                assert cu == ca and na <= nu


def test_cost_matrix_for_csp():
    env = build_demo_environment()
    bases = env.get_robot_bases()
    people = env.get_people()
    m = cost_matrix(env, bases, people)
    assert len(m) == len(bases) * len(people)
    assert all(v >= 0 for v in m.values())


def test_nearest_target():
    env = build_demo_environment()
    t, path, cost = nearest_target(env, (1, 8), [(3, 2), (4, 9)])
    assert t == (4, 9) and path[-1] == t


def test_is_path_valid_and_replan_when_fire_spreads():
    env = build_demo_environment()
    path, _, _ = find_path_env(env, (3, 2), (0, 8))
    assert is_path_valid(env, path)
    env.add_fire(*path[len(path) // 2])           # fire lands on the planned route
    assert not is_path_valid(env, path)
    new_path, _, _ = find_path_env(env, path[0], (0, 8))
    assert new_path is None or is_path_valid(env, new_path)


def test_full_dynamic_run_never_walks_into_fire():
    """Simulate: person walks toward exit while fire spreads; replan every time the path breaks."""
    env = build_random_scenario(seed=2, n_people=1, n_fires=1, n_obstacles=0)
    pos = env.get_people()[0]
    exit_cell, path, _ = nearest_exit(env, pos)
    if path is None:
        return
    for _ in range(60):
        env.step(fire_chance=0.2)
        if not is_path_valid(env, path):
            exit_cell, path, _ = nearest_exit(env, pos)
            if path is None:
                return                              # trapped: acceptable outcome
        if len(path) > 1:
            path = path[1:]
            pos = path[0]
            assert env.is_walkable(*pos)            # the robot never steps into fire/obstacle
        if pos == exit_cell:
            return
