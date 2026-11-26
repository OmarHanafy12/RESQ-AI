"""Public API for the RESQ-AI CSP task allocation module."""
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from csp.allocator import allocate_tasks, plan_all_waves, precompute_mission_costs
from csp.baseline import greedy_allocate_tasks
from csp.models import AllocationResult, RobotSpec, TaskAssignment

__all__ = [
    "RobotSpec",
    "TaskAssignment",
    "AllocationResult",
    "allocate_tasks",
    "greedy_allocate_tasks",
    "plan_all_waves",
    "precompute_mission_costs",
]
