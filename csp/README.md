# CSP & Task Allocation Module

Assigns rescue missions (`robot -> person -> exit`) to robots using **Constraint Satisfaction (`python-constraint`)** and multi-criteria resource optimization.

## Quick Use

```python
from csp import RobotSpec, allocate_tasks, plan_all_waves

# 1) One-line allocation using robot bases from the Environment
result = allocate_tasks(env)

# 2) Or pass custom robots with battery & capacity constraints
robots = [
    RobotSpec("R0", pos=(1, 8), battery=60, capacity=1),
    RobotSpec("R1", pos=(18, 8), battery=60, capacity=1),
]
result = allocate_tasks(env, robots=robots)

task_r0 = result.for_robot("R0")   # TaskAssignment(person, exit, priority, total_cost, ...) or None
```

`robots` can be `None` (uses `env.get_robot_bases()`), `(x, y)` tuples, `dict`s, `RobotSpec` instances, or Member 4's `Robot` objects (`.id`/`.robot_id`, `.pos`/`.position`, `.battery`, `.capacity`, `.available`).

## Public Interface

| Function / Class | Who uses it | What it does |
|---|---|---|
| `allocate_tasks(env, robots, ...)` | **Multi-Agent, Evaluation** | Solves the CSP for the current wave and returns the optimal `AllocationResult` |
| `plan_all_waves(env, robots, ...)` | **Evaluation, UI** | Computes a full multi-wave rescue schedule, updating robot positions and batteries |
| `greedy_allocate_tasks(env, robots, ...)` | **Baseline / Comparison** | Greedy nearest-person baseline without global CSP coordination |
| `precompute_mission_costs(...)` | CSP, Agents | Precomputes `robot -> person` and `person -> nearest_exit` path costs |
| `RobotSpec` | Multi-Agent, Tests | Lightweight robot specification (`robot_id`, `pos`, `battery`, `capacity`, `available`) |
| `TaskAssignment` | Multi-Agent, UI | Assigned mission (`person`, `exit`, `priority`, `to_person_cost`, `to_exit_cost`, `total_cost`) |
| `AllocationResult` | Everyone | Full wave result (`assignments`, `unassigned_people`, `unreachable_people`, `total_cost`, `makespan`) |

## Constraints & Optimization

### Hard Constraints (`csp/constraints.py`)
1. **Reachability:** Both `robot -> person` and `person -> nearest_exit` must have valid finite paths (`cost < math.inf`, avoiding walls, fire, and obstacles).
2. **Battery Limit:** Total mission cost assigned to a robot cannot exceed its remaining `battery`.
3. **Capacity Limit:** Number of people assigned to a robot cannot exceed its `capacity`.
4. **Availability:** Robots with `available=False` are not assigned new tasks.
5. **Conflict Prevention:** No two robots (or task slots) can ever target the same person.
6. **Work Conservation:** A robot cannot stay idle while an unassigned feasible person fits within its remaining battery and capacity.

### Optimization Objective (`csp/allocator.py`)
Among all valid CSP assignments, the solver selects the solution that optimizes (in priority order):
1. **Maximum assigned people** in the current wave.
2. **Highest victim priority** (`env.get_priority(x, y)`, critical `3` before `2` before `1`).
3. **Minimum path cost & makespan** (`objective="cost"` or `objective="makespan"`), naturally respecting smoke cost (`5`) and balancing robot workloads.
4. **Maximum fleet battery reserve** on ties.

## Dynamic Re-allocation Loop (for Member 4 — Multi-Agent)

```python
# Exclude people already carried/claimed by busy robots when re-allocating idle robots
result = allocate_tasks(env, robots=idle_robots, excluded_people=claimed_people)
for robot in idle_robots:
    mission = result.for_robot(robot.id)
    if mission is not None:
        robot.assign(mission.person, mission.exit)
```

## Demo & Benchmark (Greedy vs CSP)

```bash
python -m csp.compare_allocation    # prints hospital schedule + saves outputs/csp_comparison.png
```
