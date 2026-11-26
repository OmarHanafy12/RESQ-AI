"""Myopic greedy nearest-person baseline for benchmarking against CSP allocation."""
from collections.abc import Iterable, Sequence

from csp.allocator import _prepare
from csp.models import AllocationResult, Coord, RobotId, RobotSpec, TaskAssignment


def greedy_allocate_tasks(
    env,
    robots: Sequence[RobotSpec] | None = None,
    people: Iterable[Coord] | None = None,
    excluded_people: Iterable[Coord] | None = None,
    algorithm: str = "astar",
) -> AllocationResult:
    """Assign each available robot its nearest affordable unclaimed person."""
    ctx = _prepare(env, robots, people, excluded_people, algorithm)
    assignments: dict[RobotId, TaskAssignment | None] = {r.robot_id: None for r in ctx.robots}
    claimed: set[Coord] = set()

    for r in ctx.robots:
        options = [p for p in ctx.feasible_by_robot[r.robot_id] if p not in claimed]
        if options:
            best = min(
                options,
                key=lambda p: (ctx.to_person_costs[(r.robot_id, p)], ctx.mission_costs[(r.robot_id, p)], p),
            )
            claimed.add(best)
            assignments[r.robot_id] = ctx.make_task(r, best)

    return ctx.to_result(assignments)
