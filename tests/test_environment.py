# test_environment.py
# Automatic tests for the Environment module.

import unittest

from environment.constants import UP, DOWN, LEFT, RIGHT, FLOOR, WALL, EXIT
from environment.grid import Grid
from environment.environment import Environment
from environment.building_layout import build_sample_hospital
from scenarios import build_random_scenario


class TestGrid(unittest.TestCase):
    def setUp(self):
        self.grid = Grid(5, 4)

    def test_size(self):
        self.assertEqual(self.grid.width, 5)
        self.assertEqual(self.grid.height, 4)

    def test_set_and_get_cell_uses_x_y(self):
        self.grid.set_cell(2, 1, WALL)            # x=2 (column), y=1 (row)
        self.assertEqual(self.grid.get_cell(2, 1), WALL)
        self.assertEqual(self.grid.cells[1][2], WALL)   # grid[y][x]

    def test_in_bounds(self):
        self.assertTrue(self.grid.in_bounds(0, 0))
        self.assertTrue(self.grid.in_bounds(4, 3))
        self.assertFalse(self.grid.in_bounds(5, 0))
        self.assertFalse(self.grid.in_bounds(0, 4))
        self.assertFalse(self.grid.in_bounds(-1, 2))

    def test_is_walkable(self):
        self.grid.set_cell(2, 1, WALL)
        self.assertTrue(self.grid.is_walkable(0, 0))
        self.assertFalse(self.grid.is_walkable(2, 1))
        self.assertFalse(self.grid.is_walkable(99, 99))

    def test_neighbor_counts(self):
        self.assertEqual(len(self.grid.get_neighbors(0, 0)), 2)   # corner
        self.assertEqual(len(self.grid.get_neighbors(0, 2)), 3)   # edge
        self.assertEqual(len(self.grid.get_neighbors(3, 2)), 4)   # middle

    def test_wall_is_not_a_neighbor(self):
        self.grid.set_cell(2, 1, WALL)
        self.assertNotIn((2, 1), self.grid.get_neighbors(1, 1))
        self.assertEqual(len(self.grid.get_neighbors(1, 1)), 3)


class TestDirections(unittest.TestCase):
    def test_direction_values(self):
        self.assertEqual(UP, (0, -1))
        self.assertEqual(DOWN, (0, 1))
        self.assertEqual(LEFT, (-1, 0))
        self.assertEqual(RIGHT, (1, 0))


class TestEnvironment(unittest.TestCase):
    def setUp(self):
        self.env = Environment(20, 12)
        build_sample_hospital(self.env.grid)

    def test_exit_rules(self):
        self.assertTrue(self.env.add_exit(0, 8))
        self.assertFalse(self.env.add_exit(50, 50))
        self.assertEqual(self.env.get_exits(), [(0, 8)])
        self.assertTrue(self.env.is_exit(0, 8))
        self.assertFalse(self.env.is_exit(99, 99))

    def test_person_rules(self):
        self.assertTrue(self.env.add_person(3, 2))
        self.assertFalse(self.env.add_person(0, 0))      # wall
        self.assertFalse(self.env.add_person(99, 99))    # outside
        self.assertEqual(self.env.get_people(), [(3, 2)])
        self.env.remove_person(3, 2)
        self.assertEqual(self.env.get_people(), [])

    def test_fire_blocks_neighbors(self):
        self.assertIn((3, 1), self.env.get_neighbors(3, 2))
        self.assertTrue(self.env.add_fire(3, 1))
        self.assertNotIn((3, 1), self.env.get_neighbors(3, 2))
        self.env.remove_fire(3, 1)
        self.assertIn((3, 1), self.env.get_neighbors(3, 2))

    def test_obstacle_blocks_neighbors(self):
        self.assertTrue(self.env.add_obstacle(4, 2))
        self.assertNotIn((4, 2), self.env.get_neighbors(3, 2))
        self.env.remove_obstacle(4, 2)
        self.assertIn((4, 2), self.env.get_neighbors(3, 2))

    def test_invalid_fire_and_obstacle(self):
        self.assertFalse(self.env.add_fire(0, 0))          # wall
        self.env.add_person(3, 2)
        self.assertFalse(self.env.add_obstacle(3, 2))      # person there

    def test_public_interface(self):
        self.env.add_fire(3, 1)
        self.assertEqual((self.env.width, self.env.height), (20, 12))
        self.assertTrue(self.env.has_fire(3, 1))
        self.assertFalse(self.env.has_obstacle(5, 5))
        self.assertEqual(self.env.move_cost(3, 3), 1)


class TestIntegrationFeatures(unittest.TestCase):
    """Features added for the other modules: smoke, priorities, robot bases, step, copy."""

    def setUp(self):
        self.env = Environment(20, 12, seed=1)
        build_sample_hospital(self.env.grid)

    def test_smoke_costs_more_but_is_walkable(self):
        self.assertTrue(self.env.add_smoke(3, 3))
        self.assertEqual(self.env.move_cost(3, 3), 5)
        self.assertIn((3, 3), self.env.get_neighbors(3, 2))
        self.assertFalse(self.env.add_smoke(0, 0))             # wall

    def test_min_move_cost_is_one(self):
        self.assertEqual(self.env.min_move_cost, 1)

    def test_person_priority(self):
        self.env.add_person(3, 2, priority=1)
        self.env.add_person(4, 2, priority=3)
        self.assertEqual(self.env.get_priority(4, 2), 3)
        self.assertEqual(self.env.get_people_by_priority(), [(4, 2), (3, 2)])
        self.env.remove_person(4, 2)
        self.assertEqual(self.env.get_priority(4, 2), 1)       # back to default

    def test_robot_bases(self):
        self.assertTrue(self.env.add_robot_base(1, 8))
        self.assertFalse(self.env.add_robot_base(0, 0))        # wall
        self.assertEqual(self.env.get_robot_bases(), [(1, 8)])

    def test_fire_never_enters_walls_or_exits(self):
        self.env.add_exit(0, 8)
        self.env.add_fire(1, 8)
        for _ in range(60):
            self.env.step(fire_chance=1.0)
        for x, y in self.env.get_fire_zones():
            self.assertEqual(self.env.get_cell_type(x, y), FLOOR)

    def test_step_reports_people_hit_and_casualty(self):
        self.env.add_person(4, 2)
        self.env.add_fire(3, 2)
        result = self.env.step(fire_chance=1.0)
        self.assertIn((4, 2), result["people_hit"])
        self.assertEqual(self.env.get_people_in_fire(), [(4, 2)])
        self.env.mark_casualty(4, 2)
        self.assertEqual(self.env.get_people(), [])
        self.assertIn((4, 2), self.env.casualties)

    def test_step_creates_smoke_next_to_fire(self):
        self.env.add_fire(3, 2)
        result = self.env.step(fire_chance=0.0, smoke_chance=1.0)
        self.assertTrue(result["new_smoke"])
        for cell in result["new_smoke"]:
            self.assertTrue(self.env.has_smoke(*cell))

    def test_same_seed_same_fire(self):
        a = Environment(20, 12, seed=7); build_sample_hospital(a.grid); a.add_fire(3, 2)
        b = Environment(20, 12, seed=7); build_sample_hospital(b.grid); b.add_fire(3, 2)
        for _ in range(5):
            a.step(); b.step()
        self.assertEqual(sorted(a.fire), sorted(b.fire))

    def test_copy_is_independent_and_keeps_rng(self):
        self.env.add_fire(3, 2)
        clone = self.env.copy()
        self.env.step(fire_chance=1.0)
        self.assertEqual(len(clone.fire), 1)                   # clone not affected
        clone.step(fire_chance=1.0)
        self.assertEqual(sorted(clone.fire), sorted(self.env.fire))

    def test_random_scenario_is_reproducible(self):
        a = build_random_scenario(seed=5)
        b = build_random_scenario(seed=5)
        self.assertEqual(sorted(a.get_people()), sorted(b.get_people()))
        self.assertEqual(sorted(a.get_fire_zones()), sorted(b.get_fire_zones()))
        self.assertEqual(a.get_robot_bases(), b.get_robot_bases())
        self.assertEqual(len(a.get_people()), 6)

    def test_random_scenario_does_not_block_doors(self):
        for seed in range(20):
            env = build_random_scenario(seed=seed)
            for door in [(3, 5), (9, 5), (16, 5), (10, 8)]:
                self.assertTrue(env.is_walkable(*door))


if __name__ == "__main__":
    unittest.main()