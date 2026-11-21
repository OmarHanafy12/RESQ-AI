"""Fake grids for testing, until Member 1 delivers the real environment."""
import random
from pathfinding.search import EMPTY, WALL, FIRE, SMOKE


def open_grid(w=10, h=10):
    return [[EMPTY] * w for _ in range(h)]


def wall_with_gap(w=10, h=10, gap_y=5):
    """A vertical wall in the middle with one gap."""
    g = open_grid(w, h)
    for y in range(h):
        g[y][w // 2] = WALL
    g[gap_y][w // 2] = EMPTY
    return g


def blocked_grid(w=10, h=10):
    """Fully sealed wall: no path from left to right."""
    g = open_grid(w, h)
    for y in range(h):
        g[y][w // 2] = WALL
    return g


def smoky_corridor():
    """Short route through smoke vs a longer clear route."""
    g = open_grid(7, 5)
    for x in range(1, 6):
        g[2][x] = SMOKE
    for x in range(1, 6):
        g[1][x] = WALL
    return g


def random_grid(w=30, h=30, wall_p=0.2, smoke_p=0.1, fire_p=0.03, seed=0):
    rng = random.Random(seed)
    g = open_grid(w, h)
    for y in range(h):
        for x in range(w):
            r = rng.random()
            if r < wall_p:
                g[y][x] = WALL
            elif r < wall_p + fire_p:
                g[y][x] = FIRE
            elif r < wall_p + fire_p + smoke_p:
                g[y][x] = SMOKE
    g[0][0] = EMPTY
    g[h - 1][w - 1] = EMPTY
    return g
