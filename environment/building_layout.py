# building_layout.py
# Helper functions that draw a building on a Grid.
# Convention: coordinates are (x, y), grid[y][x].

from environment.constants import FLOOR, WALL


def draw_hline(grid, y, x_start, x_end, cell_type=WALL):
    """Draw a horizontal line on row y, from x_start to x_end (both included)."""
    for x in range(x_start, x_end + 1):
        if grid.in_bounds(x, y):
            grid.set_cell(x, y, cell_type)


def draw_vline(grid, x, y_start, y_end, cell_type=WALL):
    """Draw a vertical line on column x, from y_start to y_end (both included)."""
    for y in range(y_start, y_end + 1):
        if grid.in_bounds(x, y):
            grid.set_cell(x, y, cell_type)


def draw_rect_walls(grid, x1, y1, x2, y2):
    """Draw the walls (only the border) of a rectangle.

    (x1, y1) = top-left corner, (x2, y2) = bottom-right corner.
    """
    draw_hline(grid, y1, x1, x2)   # top side
    draw_hline(grid, y2, x1, x2)   # bottom side
    draw_vline(grid, x1, y1, y2)   # left side
    draw_vline(grid, x2, y1, y2)   # right side


def make_door(grid, x, y):
    """Open a doorway: turn the wall at (x, y) back into FLOOR."""
    if grid.in_bounds(x, y):
        grid.set_cell(x, y, FLOOR)


def build_sample_hospital(grid):
    """Build a small test building on a 20 x 12 grid."""
    # Outer wall around the whole building
    draw_rect_walls(grid, 0, 0, grid.width - 1, grid.height - 1)

    # A horizontal wall at y=5 separates top rooms from the corridor/bottom rooms
    draw_hline(grid, 5, 0, grid.width - 1)

    # Vertical walls split the top part into 3 rooms
    draw_vline(grid, 6, 0, 5)
    draw_vline(grid, 13, 0, 5)

    # Doors from each top room into the lower area
    make_door(grid, 3, 5)
    make_door(grid, 9, 5)
    make_door(grid, 16, 5)

    # Vertical walls split the bottom part into 2 rooms
    draw_vline(grid, 10, 6, 11)
    make_door(grid, 10, 8)