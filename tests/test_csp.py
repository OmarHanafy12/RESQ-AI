"""Automated tests for the CSP task allocation module."""
import itertools
import math
import random
import pytest

from csp import (
    AllocationResult,
    RobotSpec,
    allocate_tasks,
    greedy_allocate_tasks,
    plan_all_waves,
    precompute_mission_costs,
)
from csp.allocator import _score_solution
from csp.constraints import is_task_feasible, no_task_conflict
from environment.building_layout import build_sample_hospital
from environment.environment import Environment
from scenarios import build_random_scenario
from scenarios.demo_environment import build_demo_environment


def test_demo_hospital_wave1_prioritizes_critical_without_conflict():
    env = build_demo_environment()
    res = allocate_tasks(env)

    assigned = res.assigned_people()
    assert len(assigned) == 2
    assert res.for_robot("R0").person != res.for_robot("R1").person
    assert (9, 3) in assigned
    assert res.total_priority == 5
    assert len(res.unassigned_people) == 3
    assert res.unreachable_people == []
    assert res.robot_to_person() == {
        "R0": res.for_robot("R0").person,
        "R1": res.for_robot("R1").person,
    }


def test_ucs_and_astar_produce_identical_csp_allocation():
    env = build_demo_environment()
    res_astar = allocate_tasks(env, algorithm="astar")
    res_ucs = allocate_tasks(env, algorithm="ucs")

    assert res_astar.robot_to_person() == res_ucs.robot_to_person()
    assert res_astar.total_cost == res_ucs.total_cost
    assert res_astar.total_priority == res_ucs.total_priority


def test_no_conflict_when_two_robots_start_next_to_same_person():
    env = Environment(10, 6)
    env.add_exit(0, 0)
    env.add_person(2, 2, priority=3)
    env.add_person(8, 4, priority=1)

    robots = [RobotSpec("R0", pos=(2, 1)), RobotSpec("R1", pos=(2, 3))]
    res = allocate_tasks(env, robots=robots)
    assert res.assigned_people() == {(2, 2), (8, 4)}
    assert res.for_robot("R0").person != res.for_robot("R1").person


def test_more_robots_than_people_leaves_extra_robot_idle():
    env = Environment(8, 5)
    env.add_exit(0, 0)
    env.add_person(3, 2, priority=2)

    robots = [RobotSpec("R0", (1, 1)), RobotSpec("R1", (6, 4))]
    res = allocate_tasks(env, robots=robots)

    assert res.assigned_people() == {(3, 2)}
    assert res.for_robot("R0").person == (3, 2)
    assert res.for_robot("R1") is None
    assert res.unassigned_people == []


def test_higher_priority_beats_closer_low_priority():
    env = Environment(12, 5)
    env.add_exit(0, 0)
    env.add_person(2, 0, priority=1)
    env.add_person(9, 4, priority=3)

    res = allocate_tasks(env, robots=[RobotSpec("R0", (1, 0))])
    assert res.for_robot("R0").person == (9, 4)
    assert res.unassigned_people == [(2, 0)]


def test_triage_objective_ranks_critical_victims_ahead_of_head_count_and_cost():
    # 1) On a real map, two robots pick (3, 2) [cost 10] over much cheaper (2, 2) [cost 4].
    env = Environment(10, 5)
    env.add_exit(0, 0)
    env.add_person(4, 0, priority=3)
    env.add_person(1, 0, priority=2)
    env.add_person(0, 1, priority=2)

    robots = [RobotSpec("R0", (0, 0)), RobotSpec("R1", (0, 0))]
    res = allocate_tasks(env, robots=robots)
    assert (4, 0) in res.assigned_people()
    assert res.total_priority == 5

    # 2) Score contract: priority 3 beats 1+1 (sum 3 > 2), 1+1+1 ((3,) > (1,1,1)), and (3,1) > (2,2).
    specs = [RobotSpec("R0", (0, 0)), RobotSpec("R1", (0, 0)), RobotSpec("R2", (0, 0))]
    prios = {(9, 0): 3, (4, 0): 2, (5, 0): 2, (1, 0): 1, (2, 0): 1, (3, 0): 1}
    mc = {(r.robot_id, p): float(p[0] * 2) for r in specs for p in prios}

    s_crit = _score_solution({"R0": (9, 0), "R1": None, "R2": None}, specs, prios, mc, "cost")
    s_two_low = _score_solution({"R0": (1, 0), "R1": (2, 0), "R2": None}, specs, prios, mc, "cost")
    s_three_low = _score_solution({"R0": (1, 0), "R1": (2, 0), "R2": (3, 0)}, specs, prios, mc, "cost")
    s_3_1 = _score_solution({"R0": (9, 0), "R1": (1, 0), "R2": None}, specs, prios, mc, "cost")
    s_2_2 = _score_solution({"R0": (4, 0), "R1": (5, 0), "R2": None}, specs, prios, mc, "cost")
    assert s_crit > s_two_low
    assert s_crit > s_three_low
    assert s_3_1 > s_2_2


def test_exact_optimality_matches_brute_force_beyond_old_cap():
    # 1) Verify against exhaustive permutations on 5 robots x 13 people (154,440 perms > 50k).
    rng = random.Random(7)
    env = Environment(24, 24)
    env.add_exit(0, 0)
    env.add_exit(23, 23)
    cells = rng.sample([(x, y) for x in range(1, 23) for y in range(1, 23)], 18)
    for cell in cells[:13]:
        env.add_person(*cell, priority=rng.randint(1, 3))
    robots = [RobotSpec(f"R{i}", c) for i, c in enumerate(cells[13:])]

    res = allocate_tasks(env, robots=robots)
    people = sorted(env.get_people())
    prios = {p: env.get_priority(*p) for p in people}
    _, _, mc = precompute_mission_costs(env, robots, people)

    best_brute = max(
        (
            sum(prios[p] for p in combo),
            tuple(sorted((prios[p] for p in combo), reverse=True)),
            -sum(mc[(r.robot_id, p)] for r, p in zip(robots, combo)),
        )
        for combo in itertools.permutations(people, len(robots))
    )
    assert res.total_priority == best_brute[0]
    assert tuple(sorted((t.priority for t in res.tasks), reverse=True)) == best_brute[1]
    assert res.total_cost == -best_brute[2]

    # 2) Regression check on the 6x15 probe where the old 50k cap returned cost 174 instead of 166.
    rng6 = random.Random(0)
    env6 = Environment(30, 30)
    env6.add_exit(0, 0)
    env6.add_exit(29, 29)
    cells6 = rng6.sample([(x, y) for x in range(1, 29) for y in range(1, 29)], 21)
    for cell in cells6[:15]:
        env6.add_person(*cell, priority=rng6.randint(1, 3))
    robots6 = [RobotSpec(f"R{i}", c) for i, c in enumerate(cells6[15:])]
    res6 = allocate_tasks(env6, robots=robots6)
    assert res6.total_priority == 18
    assert res6.total_cost == 166


def test_trapped_person_and_sealed_room_without_exit_are_unreachable():
    env = build_demo_environment()
    env.add_fire(3, 5)

    res = allocate_tasks(env)
    assert (3, 2) in res.unreachable_people
    assert (3, 2) not in res.assigned_people()

    sealed = Environment(6, 6)
    sealed.add_person(2, 2, priority=3)
    res_no_exit = allocate_tasks(sealed, robots=[RobotSpec("R0", (2, 3))])
    assert res_no_exit.assigned_people() == set()
    assert res_no_exit.unreachable_people == [(2, 2)]
    assert res_no_exit.num_solutions is None


def test_battery_constraint_coordinates_low_and_high_battery_robots():
    env = Environment(15, 5)
    env.add_exit(0, 0)
    env.add_person(2, 0, priority=1)
    env.add_person(12, 0, priority=3)

    robots = [
        RobotSpec("R_low", pos=(1, 0), battery=5),
        RobotSpec("R_high", pos=(1, 1), battery=40),
    ]
    res = allocate_tasks(env, robots=robots)
    assert res.for_robot("R_low").person == (2, 0)
    assert res.for_robot("R_high").person == (12, 0)


def test_csp_beats_greedy_when_greedy_strands_person():
    env = Environment(12, 4)
    env.add_exit(0, 0)
    env.add_person(2, 0, priority=2)
    env.add_person(8, 0, priority=2)

    robots = [
        RobotSpec("R0", pos=(1, 0), battery=20),
        RobotSpec("R1", pos=(4, 0), battery=10),
    ]
    greedy_res = greedy_allocate_tasks(env, robots=robots)
    csp_res = allocate_tasks(env, robots=robots)

    assert greedy_res.num_solutions is None
    assert len(greedy_res.assigned_people()) == 1
    assert len(csp_res.assigned_people()) == 2
    assert csp_res.robot_to_person() == {"R0": (8, 0), "R1": (2, 0)}


def test_makespan_vs_cost_objective_preference():
    # R0=(2,0) and R1=(6,1) rescuing p1=(2,0) [exit leg=2] and p2=(2,3) [exit leg=5]:
    # - (R0->p1, R1->p2): costs (2, 11) => total=13, makespan=11 (wins on cost)
    # - (R0->p2, R1->p1): costs (8, 7)  => total=15, makespan=8  (wins on makespan)
    env = Environment(15, 10)
    env.add_exit(0, 0)
    p1, p2 = (2, 0), (2, 3)
    env.add_person(*p1, priority=2)
    env.add_person(*p2, priority=2)
    robots = [RobotSpec("R0", (2, 0)), RobotSpec("R1", (6, 1))]

    res_cost = allocate_tasks(env, robots=robots, objective="cost")
    res_make = allocate_tasks(env, robots=robots, objective="makespan")

    assert res_cost.robot_to_person() == {"R0": p1, "R1": p2}
    assert (res_cost.total_cost, res_cost.makespan) == (13, 11)

    assert res_make.robot_to_person() == {"R0": p2, "R1": p1}
    assert (res_make.total_cost, res_make.makespan) == (15, 8)


def test_smoke_shifts_assignment_to_cheaper_clear_route():
    env = Environment(11, 5)
    env.add_exit(5, 4)
    env.add_person(5, 2, priority=2)

    for x in range(2, 5):
        env.add_smoke(x, 2)
        env.add_obstacle(x, 1)
        env.add_obstacle(x, 3)

    robots = [RobotSpec("R0", (1, 2)), RobotSpec("R1", (9, 2))]
    res = allocate_tasks(env, robots=robots)
    assert res.for_robot("R1").person == (5, 2)
    assert res.for_robot("R0") is None


def test_unavailable_robot_and_excluded_people():
    env = build_demo_environment()
    robots = [
        RobotSpec("R0", pos=(1, 8), available=False),
        RobotSpec("R1", pos=(18, 8), available=True),
    ]
    res = allocate_tasks(env, robots=robots, excluded_people={(9, 3)})

    assert res.for_robot("R0") is None
    assert (9, 3) not in res.assigned_people()
    assert res.for_robot("R1").priority == 2


def test_multi_wave_schedule_tracks_exits_and_stranded_people():
    env = build_demo_environment()
    robots = [
        RobotSpec("R0", pos=(1, 8), battery=80),
        RobotSpec("R1", pos=(18, 8), battery=80),
    ]
    waves, stranded = plan_all_waves(env, robots=robots)

    assert stranded == []
    rescued_sets = [w.assigned_people() for w in waves]
    assert sum(len(s) for s in rescued_sets) == len(env.get_people())
    assert set().union(*rescued_sets) == set(env.get_people())

    # Verify wave k+1 starts each busy robot at wave k's drop-off exit.
    for prev_wave, next_wave in zip(waves, waves[1:]):
        for r_id, prev_task in prev_wave.assignments.items():
            next_task = next_wave.for_robot(r_id)
            if prev_task is not None and next_task is not None:
                assert next_task.robot_pos == prev_task.exit

    # Low battery + fire leaves unreachable/unaffordable people reported in `stranded`.
    env.add_fire(9, 5)
    tight_robots = [RobotSpec("R0", pos=(1, 8), battery=20)]
    tight_waves, tight_stranded = plan_all_waves(
        env, robots=tight_robots, excluded_people={(16, 2)}
    )
    rescued_tight = set().union(*(w.assigned_people() for w in tight_waves))
    assert (9, 3) in tight_stranded
    assert (16, 2) not in rescued_tight and (16, 2) not in tight_stranded
    assert rescued_tight | set(tight_stranded) == set(env.get_people()) - {(16, 2)}


def test_robot_spec_validation():
    with pytest.raises(TypeError):
        RobotSpec("R0", pos=[1, 2])  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        RobotSpec("R0", pos=(True, 2))  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        RobotSpec("R0", pos=(1, 2), battery=True)  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        RobotSpec("R0", pos=(1, 2), battery=-1)
    with pytest.raises(ValueError):
        RobotSpec("R0", pos=(1, 2), battery=math.nan)


def test_allocate_tasks_input_validation_and_empty_cases():
    env = Environment(20, 12)
    build_sample_hospital(env.grid)
    env.add_exit(0, 8)
    env.add_person(4, 9, priority=2)

    with pytest.raises(TypeError):
        allocate_tasks(env, robots=[(1, 8)])  # type: ignore[list-item]
    with pytest.raises(ValueError):
        allocate_tasks(env, robots=[RobotSpec("dup", (1, 8)), RobotSpec("dup", (2, 8))])
    with pytest.raises(ValueError):
        allocate_tasks(env, robots=[RobotSpec("R0", (1, 8))], objective="invalid")

    empty_env = Environment(6, 6)
    empty_res = allocate_tasks(empty_env, robots=[RobotSpec("R0", (1, 1))])
    assert isinstance(empty_res, AllocationResult)
    assert empty_res.assigned_people() == set()
    assert empty_res.num_solutions is None

    no_robots_res = allocate_tasks(env, robots=[])
    assert no_robots_res.assigned_people() == set()
    assert no_robots_res.num_solutions is None

    assert is_task_feasible(10.0, RobotSpec("R0", (0, 0), battery=10.0))
    assert not is_task_feasible(math.inf, RobotSpec("R0", (0, 0)))
    assert no_task_conflict(None, None)
    assert not no_task_conflict((1, 1), (1, 1))


def test_csp_invariants_across_random_scenarios():
    batteries = (34, 18, 14)
    for seed in range(25):
        env = build_random_scenario(
            seed=seed, n_people=8, n_fires=2, n_obstacles=3, n_smoke=3, n_robots=3
        )
        robots = [
            RobotSpec(f"R{i}", pos=pos, battery=b)
            for i, (pos, b) in enumerate(zip(env.get_robot_bases(), batteries))
        ]
        csp_res = allocate_tasks(env, robots=robots)
        greedy_res = greedy_allocate_tasks(env, robots=robots)

        picked = [t.person for t in csp_res.tasks]
        assert len(picked) == len(set(picked))

        for robot in robots:
            task = csp_res.for_robot(robot.robot_id)
            if task is not None:
                assert math.isfinite(task.total_cost)
                assert task.total_cost <= robot.battery

        assert csp_res.total_priority >= greedy_res.total_priority
