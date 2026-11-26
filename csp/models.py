"""Data models for single-wave CSP rescue task allocation."""
from dataclasses import dataclass
import math

Coord = tuple[int, int]
RobotId = str | int


@dataclass(frozen=True)
class RobotSpec:
    """Rescue robot state and battery limit for one allocation wave."""

    robot_id: RobotId
    pos: Coord
    battery: float = math.inf
    available: bool = True

    def __post_init__(self) -> None:
        if (
            not isinstance(self.pos, tuple)
            or len(self.pos) != 2
            or not all(isinstance(v, int) and not isinstance(v, bool) for v in self.pos)
        ):
            raise TypeError(f"Robot {self.robot_id!r} pos must be an (x, y) int tuple.")
        if isinstance(self.battery, bool) or not isinstance(self.battery, (int, float)):
            raise TypeError(f"Robot {self.robot_id!r} battery must be a number.")
        if math.isnan(self.battery) or self.battery < 0:
            raise ValueError(f"Robot {self.robot_id!r} battery cannot be negative or NaN.")


@dataclass(frozen=True)
class TaskAssignment:
    """Single rescue mission: robot -> person -> nearest reachable exit."""

    robot_id: RobotId
    robot_pos: Coord
    person: Coord
    exit: Coord
    priority: int
    to_person_cost: float
    to_exit_cost: float
    total_cost: float


@dataclass(frozen=True)
class AllocationResult:
    """Result of one task-allocation wave across all known robots."""

    assignments: dict[RobotId, TaskAssignment | None]
    unassigned_people: list[Coord]
    unreachable_people: list[Coord]
    num_solutions: int | None = None

    @property
    def tasks(self) -> list[TaskAssignment]:
        return [t for t in self.assignments.values() if t is not None]

    @property
    def total_cost(self) -> float:
        return sum(t.total_cost for t in self.tasks)

    @property
    def makespan(self) -> float:
        return max((t.total_cost for t in self.tasks), default=0.0)

    @property
    def total_priority(self) -> int:
        return sum(t.priority for t in self.tasks)

    def for_robot(self, robot_id: RobotId) -> TaskAssignment | None:
        return self.assignments.get(robot_id)

    def robot_to_person(self) -> dict[RobotId, Coord]:
        return {r_id: t.person for r_id, t in self.assignments.items() if t is not None}

    def assigned_people(self) -> set[Coord]:
        return {t.person for t in self.tasks}
