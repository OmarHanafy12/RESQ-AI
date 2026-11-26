"""Hard CSP constraints for task feasibility and conflict prevention."""
import math
from constraint import Constraint

if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from csp.models import Coord, RobotSpec


def is_task_feasible(mission_cost: float, robot: RobotSpec) -> bool:
    """True when `robot` is available and has enough battery for a finite mission."""
    return robot.available and math.isfinite(mission_cost) and mission_cost <= robot.battery


def no_task_conflict(person_a: Coord | None, person_b: Coord | None) -> bool:
    """True when two robots do not target the same person (`None` may repeat)."""
    return person_a is None or person_b is None or person_a != person_b


class NoTaskConflict(Constraint):
    """Global CSP constraint ensuring every non-`None` person is assigned at most once."""

    def __call__(self, variables, domains, assignments, forwardcheck=False) -> bool:
        seen = set()
        for var in variables:
            val = assignments.get(var)
            if val is not None:
                if val in seen:
                    return False
                seen.add(val)

        if forwardcheck and seen:
            for var in variables:
                if var not in assignments:
                    domain = domains[var]
                    for val in seen:
                        if val in domain:
                            domain.hideValue(val)
                            if not domain:
                                return False
        return True
