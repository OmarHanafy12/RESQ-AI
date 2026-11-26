"""CSP rescue task allocator, branch-and-bound solver, and multi-wave planner."""
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
import math
from constraint import Problem, Solver

if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pathfinding import cost_matrix, nearest_exit
from csp.constraints import NoTaskConflict, is_task_feasible
from csp.models import AllocationResult, Coord, RobotId, RobotSpec, TaskAssignment


@dataclass(frozen=True)
class _AllocationContext:
    robots: list[RobotSpec]
    robot_by_id: dict[RobotId, RobotSpec]
    priorities: dict[Coord, int]
    exit_info: dict[Coord, tuple[Coord | None, float]]
    to_person_costs: dict[tuple[RobotId, Coord], float]
    mission_costs: dict[tuple[RobotId, Coord], float]
    feasible_by_robot: dict[RobotId, list[Coord]]
    globally_feasible: set[Coord]
    unreachable_people: list[Coord]

    def make_task(self, robot: RobotSpec, person: Coord) -> TaskAssignment:
        ex_cell, ex_cost = self.exit_info[person]
        assert ex_cell is not None
        return TaskAssignment(
            robot_id=robot.robot_id,
            robot_pos=robot.pos,
            person=person,
            exit=ex_cell,
            priority=self.priorities[person],
            to_person_cost=self.to_person_costs[(robot.robot_id, person)],
            to_exit_cost=ex_cost,
            total_cost=self.mission_costs[(robot.robot_id, person)],
        )

    def to_result(
        self,
        assignments: dict[RobotId, TaskAssignment | None],
        num_solutions: int | None = None,
    ) -> AllocationResult:
        assigned = {t.person for t in assignments.values() if t is not None}
        unassigned = sorted(
            (p for p in self.globally_feasible if p not in assigned),
            key=lambda p: (-self.priorities[p], p),
        )
        return AllocationResult(assignments, unassigned, self.unreachable_people, num_solutions)


def _resolve_robots(env, robots: Sequence[RobotSpec] | None = None) -> list[RobotSpec]:
    """Return validated `RobotSpec` list from explicit specs or `env.get_robot_bases()`."""
    if robots is None:
        return [RobotSpec(f"R{i}", tuple(pos)) for i, pos in enumerate(env.get_robot_bases())]

    specs = list(robots)
    seen: set[RobotId] = set()
    for spec in specs:
        if not isinstance(spec, RobotSpec):
            raise TypeError(f"Expected RobotSpec instances, got {type(spec).__name__}.")
        if spec.robot_id in seen:
            raise ValueError(f"Duplicate robot_id detected: {spec.robot_id!r}")
        seen.add(spec.robot_id)
    return specs


def precompute_mission_costs(
    env,
    robots: Iterable[RobotSpec],
    people: Iterable[Coord],
    algorithm: str = "astar",
):
    """Precompute `person -> nearest_exit` and `robot -> person -> exit` path costs."""
    specs = list(robots)
    targets = list(people)
    exit_info = {p: nearest_exit(env, p, algorithm=algorithm)[::2] for p in targets}
    sources = list({r.pos for r in specs})
    raw = cost_matrix(env, sources, targets, algorithm=algorithm) if (sources and targets) else {}

    to_person, mission = {}, {}
    for r in specs:
        for p in targets:
            leg1 = raw.get((r.pos, p), math.inf)
            to_person[(r.robot_id, p)] = leg1
            mission[(r.robot_id, p)] = leg1 + exit_info[p][1]
    return exit_info, to_person, mission


def _prepare(
    env,
    robots: Sequence[RobotSpec] | None,
    people: Iterable[Coord] | None,
    excluded_people: Iterable[Coord] | None,
    algorithm: str,
) -> _AllocationContext:
    """Normalize inputs, precompute path costs, and classify feasible/unreachable people."""
    specs = _resolve_robots(env, robots)
    excluded = set(excluded_people or ())
    raw_people = env.get_people() if people is None else people
    targets = sorted(p for p in set(raw_people) if p not in excluded)

    priorities = {p: env.get_priority(*p) for p in targets}
    exit_info, to_person, mission = precompute_mission_costs(env, specs, targets, algorithm)

    feasible = {r.robot_id: [] for r in specs}
    globally_feasible: set[Coord] = set()
    for r in specs:
        for p in targets:
            if is_task_feasible(mission[(r.robot_id, p)], r):
                feasible[r.robot_id].append(p)
                globally_feasible.add(p)

    unreachable = [p for p in targets if p not in globally_feasible]
    return _AllocationContext(
        robots=specs,
        robot_by_id={r.robot_id: r for r in specs},
        priorities=priorities,
        exit_info=exit_info,
        to_person_costs=to_person,
        mission_costs=mission,
        feasible_by_robot=feasible,
        globally_feasible=globally_feasible,
        unreachable_people=unreachable,
    )


def _select_candidates(feasible_people, r_id, priorities, mission_costs, max_robots):
    """Keep up to `max_robots` cheapest options per priority tier (lossless by pigeonhole)."""
    by_tier: dict[int, list[Coord]] = {}
    for p in feasible_people:
        by_tier.setdefault(priorities[p], []).append(p)

    kept = [
        p
        for tier in by_tier.values()
        for p in sorted(tier, key=lambda x: (mission_costs[(r_id, x)], x))[:max_robots]
    ]
    return sorted(kept, key=lambda p: (-priorities[p], mission_costs[(r_id, p)], p))


def _score_solution(solution, robots, priorities, mission_costs, objective) -> tuple:
    """Rank valid assignments: priority sum -> critical profile -> count -> cost -> reserve."""
    per_robot = {r.robot_id: 0.0 for r in robots}
    prios = []
    for r in robots:
        if (p := solution.get(r.robot_id)) is not None:
            per_robot[r.robot_id] = mission_costs[(r.robot_id, p)]
            prios.append(priorities[p])

    total_cost = sum(per_robot.values())
    makespan = max(per_robot.values(), default=0.0)
    cost_terms = (-makespan, -total_cost) if objective == "makespan" else (-total_cost, -makespan)
    min_rem = min((r.battery - per_robot[r.robot_id] for r in robots if r.available), default=0.0)
    tie = tuple((-p[0], -p[1]) if (p := solution.get(r.robot_id)) is not None else (1, 1) for r in robots)
    return (sum(prios), tuple(sorted(prios, reverse=True)), len(prios), *cost_terms, min_rem, tie)


class BranchAndBoundSolver(Solver):
    """Exact `python-constraint` solver with MRV, forward checking, and upper-bound pruning."""

    def __init__(self, robots, priorities, mission_costs, objective="cost") -> None:
        super().__init__()
        self.robots = robots
        self.priorities = priorities
        self.mc = mission_costs
        self.objective = objective
        self.solutions_found = 0

    def _can_improve(self, domains, assignments, best_score, cur_prio, cur_prios, cur_cost, cur_make) -> bool:
        if best_score is None:
            return True
        pr, mc = self.priorities, self.mc
        unassigned_doms, rem_people, robot_max = [], set(), []
        for var, dom in domains.items():
            if var not in assignments and (opts := [p for p in dom if p is not None]):
                unassigned_doms.append((var, opts))
                rem_people.update(opts)
                robot_max.append(max(pr[p] for p in opts))

        q = min(len(unassigned_doms), len(rem_people))
        p_top = sorted((pr[p] for p in rem_people), reverse=True)[:q]
        r_top = sorted(robot_max, reverse=True)[:q]
        extra = [e for e in (min(a, b) for a, b in zip(p_top, r_top)) if e > 0]

        ub_prio = cur_prio + sum(extra)
        if ub_prio != best_score[0] or len(extra) < q:
            return ub_prio >= best_score[0] if len(extra) < q else ub_prio > best_score[0]

        ub_head = (tuple(sorted(cur_prios + extra, reverse=True)), len(cur_prios) + q)
        if ub_head != best_score[1:3]:
            return ub_head > best_score[1:3]

        floor_prio = extra[-1] if q > 0 else 0
        min_costs = sorted(
            min(elev)
            for var, opts in unassigned_doms
            if (elev := [mc[(var, p)] for p in opts if pr[p] >= floor_prio])
        )
        if len(min_costs) < q:
            return False
        lb_total = cur_cost + sum(min_costs[:q])
        lb_make = max(cur_make, min_costs[q - 1] if q > 0 else 0.0)
        ub_costs = (-lb_make, -lb_total) if self.objective == "makespan" else (-lb_total, -lb_make)
        return ub_costs >= best_score[3:5]

    def getSolution(self, domains, constraints, vconstraints):
        self.solutions_found = 0
        best_sol, best_score = None, None
        assignments: dict[RobotId, Coord | None] = {}
        pr, mc = self.priorities, self.mc

        def search(cur_prio: int, cur_prios: list[int], cur_cost: float, cur_make: float) -> None:
            nonlocal best_sol, best_score
            if not self._can_improve(domains, assignments, best_score, cur_prio, cur_prios, cur_cost, cur_make):
                return
            unassigned = [v for v in domains if v not in assignments]
            if not unassigned:
                self.solutions_found += 1
                score = _score_solution(assignments, self.robots, pr, mc, self.objective)
                if best_score is None or score > best_score:
                    best_score, best_sol = score, assignments.copy()
                return

            var = min(unassigned, key=lambda v: len(domains[v]))
            push = [domains[v] for v in unassigned if v != var]
            values = sorted(
                domains[var],
                key=lambda p: (1, 0, 0.0, ()) if p is None else (0, -pr[p], mc[(var, p)], p),
            )
            for val in values:
                assignments[var] = val
                for dom in push:
                    dom.pushState()
                if all(c(vs, domains, assignments, True) for c, vs in vconstraints[var]):
                    if val is None:
                        search(cur_prio, cur_prios, cur_cost, cur_make)
                    else:
                        c_val = mc[(var, val)]
                        search(cur_prio + pr[val], cur_prios + [pr[val]], cur_cost + c_val, max(cur_make, c_val))
                for dom in push:
                    dom.popState()
            del assignments[var]

        search(0, [], 0.0, 0.0)
        return best_sol


def allocate_tasks(
    env,
    robots: Sequence[RobotSpec] | None = None,
    people: Iterable[Coord] | None = None,
    excluded_people: Iterable[Coord] | None = None,
    algorithm: str = "astar",
    objective: str = "cost",
) -> AllocationResult:
    """Solve the single-wave rescue task allocation CSP and return the optimal result."""
    if objective not in ("cost", "makespan"):
        raise ValueError("objective must be 'cost' or 'makespan'")

    ctx = _prepare(env, robots, people, excluded_people, algorithm)
    assignments: dict[RobotId, TaskAssignment | None] = {r.robot_id: None for r in ctx.robots}
    active = [r for r in ctx.robots if ctx.feasible_by_robot[r.robot_id]]
    if not active:
        return ctx.to_result(assignments)

    solver = BranchAndBoundSolver(ctx.robots, ctx.priorities, ctx.mission_costs, objective)
    problem = Problem(solver)
    for r in active:
        cands = _select_candidates(
            ctx.feasible_by_robot[r.robot_id], r.robot_id, ctx.priorities, ctx.mission_costs, len(active)
        )
        problem.addVariable(r.robot_id, cands + [None])
    problem.addConstraint(NoTaskConflict(), [r.robot_id for r in active])

    best = problem.getSolution() or {}
    for r_id, person in best.items():
        if person is not None:
            assignments[r_id] = ctx.make_task(ctx.robot_by_id[r_id], person)
    return ctx.to_result(assignments, solver.solutions_found)


def plan_all_waves(
    env,
    robots: Sequence[RobotSpec] | None = None,
    people: Iterable[Coord] | None = None,
    excluded_people: Iterable[Coord] | None = None,
    algorithm: str = "astar",
    objective: str = "cost",
) -> tuple[list[AllocationResult], list[Coord]]:
    """Plan consecutive CSP waves from drop-off exits; return `(waves, stranded_people)`."""
    fleet = _resolve_robots(env, robots)
    excluded = set(excluded_people or ())
    remaining = {p for p in (env.get_people() if people is None else people) if p not in excluded}
    waves: list[AllocationResult] = []

    while remaining:
        res = allocate_tasks(env, robots=fleet, people=remaining, algorithm=algorithm, objective=objective)
        if not res.assigned_people():
            break
        waves.append(res)
        remaining -= res.assigned_people()
        fleet = [
            RobotSpec(r.robot_id, t.exit, max(0.0, r.battery - t.total_cost), r.available)
            if (t := res.for_robot(r.robot_id)) is not None
            else r
            for r in fleet
        ]

    stranded = sorted(remaining, key=lambda p: (-env.get_priority(*p), p))
    return waves, stranded
