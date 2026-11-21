# environment.py
# The main interface of the Environment module.
# Convention: grid[y][x], coordinates are (x, y), 4-directional movement only.

import copy
import random

from environment.constants import FLOOR, EXIT, DIRECTIONS
from environment.grid import Grid

# Cost of stepping into a normal cell.
# IMPORTANT: no cost may be lower than 1, because the A* heuristic
# assumes the cheapest step costs 1. Lower costs would break A*.
NORMAL_MOVE_COST = 1

# Cost of stepping into a smoky cell (walkable, but we prefer to avoid it).
SMOKE_MOVE_COST = 5


class Environment:
    def __init__(self, width, height, seed=None):
        self.grid = Grid(width, height)
        self.people = set()      # set of (x, y) positions of people who need rescue
        self.fire = set()        # set of (x, y) cells on fire
        self.obstacles = set()   # set of (x, y) cells blocked by debris
        self.smoke = set()       # set of (x, y) cells filled with smoke (walkable, costs more)

        # --- Added for integration with the other modules ---
        self.priorities = {}     # {(x, y): priority} for each person (higher = more urgent)
        self.robot_bases = []    # list of (x, y) cells where rescue robots start
        self.casualties = set()  # people the fire reached before they were rescued

        # Own random generator, so a fixed seed gives the same fire spread every run.
        self.rng = random.Random(seed)

    # ---------- Exits ----------
    def add_exit(self, x, y):
        """Place an emergency exit at (x, y). Return True if it worked."""
        if not self.grid.in_bounds(x, y):
            print(f"Cannot add exit: ({x}, {y}) is outside the grid.")
            return False
        self.grid.set_cell(x, y, EXIT)
        return True

    def get_exits(self):
        """Return a list of all exit positions as (x, y)."""
        exits = []
        for y in range(self.grid.height):
            for x in range(self.grid.width):
                if self.grid.get_cell(x, y) == EXIT:
                    exits.append((x, y))
        return exits

    # ---------- People ----------
    def add_person(self, x, y, priority=1):
        """Place a person at (x, y). Return True if it worked.
        priority: higher number = more urgent (used by the task allocation module)."""
        if not self.is_walkable(x, y):
            print(f"Cannot add person: ({x}, {y}) is not a walkable cell.")
            return False
        self.people.add((x, y))
        self.priorities[(x, y)] = priority
        return True

    def remove_person(self, x, y):
        """Remove a person (for example, after being rescued)."""
        self.people.discard((x, y))
        self.priorities.pop((x, y), None)

    def get_people(self):
        """Return a list of all people positions as (x, y)."""
        return list(self.people)

    def get_priority(self, x, y):
        """Priority of the person at (x, y). Default is 1."""
        return self.priorities.get((x, y), 1)

    def get_people_by_priority(self):
        """People sorted from the most urgent to the least urgent (ties: by position)."""
        return sorted(self.people, key=lambda p: (-self.get_priority(*p), p))

    def get_people_in_fire(self):
        """People standing in a burning cell right now."""
        return [p for p in self.people if p in self.fire]

    def mark_casualty(self, x, y):
        """The fire reached this person: remove from people and record as casualty."""
        if (x, y) in self.people:
            self.remove_person(x, y)
            self.casualties.add((x, y))

    # ---------- Robot bases ----------
    def add_robot_base(self, x, y):
        """Set a start cell for a rescue robot. Return True if it worked."""
        if not self.is_walkable(x, y):
            print(f"Cannot add robot base: ({x}, {y}) is not a free walkable cell.")
            return False
        self.robot_bases.append((x, y))
        return True

    def get_robot_bases(self):
        """Return the list of robot start cells as (x, y)."""
        return list(self.robot_bases)

    # ---------- Accessibility ----------
    def is_walkable(self, x, y):
        """True if (x, y) is inside the grid, walkable, with no fire or obstacle.
        Smoke does NOT block movement, it only makes it more expensive."""
        if not self.grid.is_walkable(x, y):
            return False
        return (x, y) not in self.fire and (x, y) not in self.obstacles

    def get_neighbors(self, x, y):
        """Return the (x, y) cells you can step to from (x, y), 4 directions only."""
        neighbors = []
        for dx, dy in DIRECTIONS:
            nx, ny = x + dx, y + dy
            if self.is_walkable(nx, ny):
                neighbors.append((nx, ny))
        return neighbors

    # ---------- Fire ----------
    def add_fire(self, x, y):
        """Start a fire at (x, y). Return True if it worked."""
        if not self.is_walkable(x, y):
            print(f"Cannot add fire: ({x}, {y}) is not a free walkable cell.")
            return False
        if (x, y) in self.people:
            print(f"Cannot add fire: a person is at ({x}, {y}).")
            return False
        self.fire.add((x, y))
        return True

    def remove_fire(self, x, y):
        """Put out the fire at (x, y)."""
        self.fire.discard((x, y))

    def get_fire_zones(self):
        """Return a list of all fire positions as (x, y)."""
        return list(self.fire)

    def _can_catch_fire(self, x, y):
        """True if fire is allowed to spread into (x, y).
        Only plain FLOOR cells without fire or obstacle (no walls, no exits)."""
        if not self.grid.in_bounds(x, y):
            return False
        if self.grid.get_cell(x, y) != FLOOR:
            return False
        return (x, y) not in self.fire and (x, y) not in self.obstacles

    def spread_fire(self, chance=0.3):
        """Spread the fire by ONE step. Return a list of the new fire cells.

        Every free cell next to a burning cell catches fire with probability chance.
        A person standing in a new fire cell is NOT removed here: the engine
        can check has_person() on the returned cells.
        """
        # First collect every cell that COULD catch fire, so the fire
        # moves only one step each time (no chain reaction inside one call).
        candidates = set()
        for x, y in self.fire:
            for dx, dy in DIRECTIONS:
                nx, ny = x + dx, y + dy
                if self._can_catch_fire(nx, ny):
                    candidates.add((nx, ny))

        new_fire = []
        for cell in sorted(candidates):
            if self.rng.random() < chance:
                self.fire.add(cell)
                new_fire.append(cell)
        return new_fire

    # ---------- Smoke ----------
    def add_smoke(self, x, y):
        """Fill (x, y) with smoke. Return True if it worked."""
        if not self.grid.is_walkable(x, y):
            print(f"Cannot add smoke: ({x}, {y}) is not a walkable cell.")
            return False
        self.smoke.add((x, y))
        return True

    def remove_smoke(self, x, y):
        """Clear the smoke at (x, y)."""
        self.smoke.discard((x, y))

    def get_smoke_zones(self):
        """Return a list of all smoke positions as (x, y)."""
        return list(self.smoke)

    def spread_smoke(self, chance=0.5):
        """Smoke appears in free walkable cells next to the fire. Return the new smoke cells."""
        candidates = set()
        for x, y in self.fire:
            for dx, dy in DIRECTIONS:
                nx, ny = x + dx, y + dy
                if (self.grid.is_walkable(nx, ny)
                        and (nx, ny) not in self.fire
                        and (nx, ny) not in self.smoke):
                    candidates.add((nx, ny))

        new_smoke = []
        for cell in sorted(candidates):
            if self.rng.random() < chance:
                self.smoke.add(cell)
                new_smoke.append(cell)
        return new_smoke

    # ---------- One simulation step ----------
    def step(self, fire_chance=0.3, smoke_chance=0.5):
        """Advance the hazards by ONE time step (fire, then smoke).

        Returns a dict:
          new_fire    : cells that caught fire this step
          new_smoke   : cells that filled with smoke this step
          people_hit  : people standing in a NEW fire cell (the engine decides what to do;
                        call mark_casualty(x, y) to record them as casualties)
        """
        new_fire = self.spread_fire(fire_chance)
        new_smoke = self.spread_smoke(smoke_chance)
        people_hit = [c for c in new_fire if c in self.people]
        return {"new_fire": new_fire, "new_smoke": new_smoke, "people_hit": people_hit}

    # ---------- Obstacles ----------
    def add_obstacle(self, x, y):
        """Place an obstacle at (x, y). Return True if it worked."""
        if not self.is_walkable(x, y):
            print(f"Cannot add obstacle: ({x}, {y}) is not a free walkable cell.")
            return False
        if (x, y) in self.people:
            print(f"Cannot add obstacle: a person is at ({x}, {y}).")
            return False
        self.obstacles.add((x, y))
        return True

    def remove_obstacle(self, x, y):
        """Clear the obstacle at (x, y)."""
        self.obstacles.discard((x, y))

    def get_obstacles(self):
        """Return a list of all obstacle positions as (x, y)."""
        return list(self.obstacles)

    # ---------- Copy (same scenario for the baseline and the AI system) ----------
    def copy(self):
        """Independent deep copy, including the random generator state."""
        return copy.deepcopy(self)

    # ---------- Public interface for teammates ----------
    @property
    def width(self):
        return self.grid.width

    @property
    def height(self):
        return self.grid.height

    @property
    def min_move_cost(self):
        """Cheapest possible step. Pathfinding uses it to keep the A* heuristic admissible."""
        return min(NORMAL_MOVE_COST, SMOKE_MOVE_COST)

    def in_bounds(self, x, y):
        """True if (x, y) is inside the grid."""
        return self.grid.in_bounds(x, y)

    def get_cell_type(self, x, y):
        """Return the base cell type (FLOOR, WALL, EXIT). Fire, smoke and people are NOT included."""
        return self.grid.get_cell(x, y)

    def is_exit(self, x, y):
        return self.in_bounds(x, y) and self.grid.get_cell(x, y) == EXIT

    def has_fire(self, x, y):
        return (x, y) in self.fire

    def has_smoke(self, x, y):
        return (x, y) in self.smoke

    def has_obstacle(self, x, y):
        return (x, y) in self.obstacles

    def has_person(self, x, y):
        return (x, y) in self.people

    def move_cost(self, x, y):
        """Cost of stepping INTO (x, y): 5 in smoke, 1 otherwise. Used by UCS and A*."""
        if (x, y) in self.smoke:
            return SMOKE_MOVE_COST
        return NORMAL_MOVE_COST
