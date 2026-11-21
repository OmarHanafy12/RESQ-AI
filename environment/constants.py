# constants.py
# Shared values used by the whole RESQ-AI team.
# Convention: grid[y][x], coordinates are (x, y), 4-directional movement only.

# ---------- Directions ----------
# Each direction is (dx, dy): how much x and y change when you move.
UP = (0, -1)      # y decreases (row above)
DOWN = (0, 1)     # y increases (row below)
LEFT = (-1, 0)    # x decreases (column to the left)
RIGHT = (1, 0)    # x increases (column to the right)

# All four directions in one list, so we can loop over them later.
DIRECTIONS = [UP, DOWN, LEFT, RIGHT]

# ---------- Cell types ----------
# Each kind of cell gets a name that stands for a number.
FLOOR = 0     # empty walkable space (rooms and corridors)
WALL = 1      # blocked, nobody can pass
EXIT = 2      # emergency exit, the goal for evacuation