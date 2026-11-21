# grid.py
# The 2D grid. Convention: grid[y][x], coordinates are (x, y).

from environment.constants import FLOOR, DIRECTIONS
from environment.cell import is_walkable_type, cell_symbol

class Grid:
    def __init__(self, width, height):
        """Create a grid full of FLOOR cells.

        width  = number of columns (x goes from 0 to width - 1)
        height = number of rows    (y goes from 0 to height - 1)
        """
        self.width = width
        self.height = height
        # One inner list per row, each row has `width` cells.
        self.cells = [[FLOOR for _ in range(width)] for _ in range(height)]

    def get_cell(self, x, y):
        """Return the cell type at (x, y)."""
        return self.cells[y][x]

    def set_cell(self, x, y, cell_type):
        """Change the cell type at (x, y)."""
        self.cells[y][x] = cell_type
    def in_bounds(self, x, y):
        """Return True if (x, y) is inside the grid."""
        return 0 <= x < self.width and 0 <= y < self.height

    def is_walkable(self, x, y):
        """Return True if (x, y) is inside the grid AND can be walked on."""
        if not self.in_bounds(x, y):
            return False
        return is_walkable_type(self.cells[y][x])

    def get_neighbors(self, x, y):
        """Return a list of (x, y) cells you can step to from (x, y).

        Only 4 directions (UP, DOWN, LEFT, RIGHT). Walls and
        cells outside the grid are not included.
        """
        neighbors = []
        for dx, dy in DIRECTIONS:
            nx, ny = x + dx, y + dy
            if self.is_walkable(nx, ny):
                neighbors.append((nx, ny))
        return neighbors

    def __str__(self):
        """Draw the grid as text so we can see it."""
        rows = []
        for y in range(self.height):
            rows.append(" ".join(cell_symbol(self.cells[y][x]) for x in range(self.width)))
        return "\n".join(rows)