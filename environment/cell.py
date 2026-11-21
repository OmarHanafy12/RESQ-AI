# cell.py
# Rules about what each cell type means.

from environment.constants import FLOOR, WALL, EXIT

# A set of cell types that people and robots can walk on.
WALKABLE_TYPES = {FLOOR, EXIT}

# How each cell type looks when we print the grid in the console.
SYMBOLS = {
    FLOOR: ".",
    WALL: "#",
    EXIT: "E",
}

def is_walkable_type(cell_type):
    """Return True if this cell type can be walked on."""
    return cell_type in WALKABLE_TYPES


def cell_symbol(cell_type):
    """Return a single character used to draw this cell type."""
    return SYMBOLS.get(cell_type, "?")