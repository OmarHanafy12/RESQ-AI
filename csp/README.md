# CSP & Task Allocation Module

Assigns rescue missions (`robot -> person -> exit`) to robots using **Constraint Satisfaction (`python-constraint`)** and multi-criteria branch-and-bound resource optimization.

## Quick Use

```python
from csp import RobotSpec, allocate_tasks, plan_all_waves

# 1) One-line allocation using robot bases from the Environment
result = allocate_tasks(env)

# 2) Or pass custom robots with battery & availability constraints
robots = [
    RobotSpec("R0", pos=(1, 8), battery=60),
    RobotSpec("R1", pos=(18, 8), battery=60),
]
result = allocate_tasks(env, robots=robots)

task_r0 = result.for_robot("R0")   # TaskAssignment(person, exit, priority, total_cost, ...) or None
```

`robots` can be `None` (uses `env.get_robot_bases()` with infinite battery) or a sequence of `RobotSpec` instances (`robot_id`, `pos`, `battery`, `available`) with unique IDs.

## Public Interface

| Function / Class | Who uses it | What it does |
|---|---|---|
| `allocate_tasks(env, robots, ...)` | **Multi-Agent, Evaluation** | Solves the single-wave CSP and returns the optimal `AllocationResult` |
| `plan_all_waves(env, robots, ...)` | **Evaluation, UI** | Computes a full multi-wave rescue schedule, updating robot positions and batteries |
| `greedy_allocate_tasks(env, robots, ...)` | **Baseline / Comparison** | Greedy nearest-person baseline without global CSP coordination |
| `precompute_mission_costs(...)` | CSP, Agents | Precomputes `person -> nearest_exit` and `robot -> person -> exit` path costs |
| `RobotSpec` | Multi-Agent, Tests | Lightweight robot specification (`robot_id`, `pos`, `battery`, `available`) |
| `TaskAssignment` | Multi-Agent, UI | Assigned mission (`robot_id`, `robot_pos`, `person`, `exit`, `priority`, `to_person_cost`, `to_exit_cost`, `total_cost`) |
| `AllocationResult` | Everyone | Wave result (`assignments`, `unassigned_people`, `unreachable_people`, `num_solutions`, `total_cost`, `makespan`, `total_priority`) |

## Constraints & Optimization

### Hard Constraints (`csp/constraints.py`)
1. **Reachability:** Both `robot -> person` and `person -> nearest_exit` must have valid finite paths (`cost < math.inf`, avoiding walls, fire, and obstacles).
2. **Battery Limit:** Total mission cost (`to_person_cost + to_exit_cost`) assigned to a robot cannot exceed its remaining `battery`.
3. **Availability:** Robots with `available=False` are not assigned tasks (`is_task_feasible`).
4. **Conflict Prevention:** Forward-checked global `NoTaskConflict` constraint ensures no two robots target the same person (`None` may repeat for idle robots).

### Optimization Objective (`csp/allocator.py`)
Among all valid CSP assignments, `BranchAndBoundSolver` selects the exact optimum ranked by:
1. **Highest total victim priority** (`sum` of rescued victim priorities).
2. **Critical triage profile** (lexicographically highest sorted priority tuple, e.g. critical `3` ahead of `2` and `1`).
3. **Maximum assigned people** in the current wave.
4. **Minimum path cost & makespan** (`objective="cost"` minimizes total cost then makespan; `objective="makespan"` minimizes makespan then total cost), respecting smoke cost (`5`) and balancing workloads.
5. **Maximum minimum fleet battery reserve** on ties.

## Dynamic Re-allocation Loop (for Member 4 — Multi-Agent)

```python
# Exclude people already carried/claimed by busy robots when re-allocating idle robots
idle_specs = [RobotSpec(r.id, pos=r.pos, battery=r.battery) for r in idle_robots]
result = allocate_tasks(env, robots=idle_specs, excluded_people=claimed_people)
for robot in idle_robots:
    mission = result.for_robot(robot.id)
    if mission is not None:
        robot.assign(mission.person, mission.exit)
```

## Demo, Benchmark & Tests

```bash
python -m csp.compare_allocation    # or: python csp/compare_allocation.py
pytest tests/test_csp.py            # or: python tests/test_csp.py
```
