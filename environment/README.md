# RESQ-AI – Environment Module

## 1. What is this module?

The Environment module represents the building where the rescue agent operates.

It manages:

- Building layout and grid.
- Walls and free spaces.
- Emergency exits.
- People who need rescue.
- Fire zones and obstacles.

Other modules can use this environment to get information about the building and available movements.

## 2. Project Files

Inside the `environment/` folder:

- `cell.py`: Defines cell properties.
- `constants.py`: Stores constants and movement directions.
- `grid.py`: Manages the grid.
- `building_layout.py`: Creates the hospital layout.
- `environment.py`: Handles the environment and its main functions.

Other important files:

- `main.py`: Contains a sample hospital scenario.
- `tests/test_environment.py`: Contains tests for the module.

## 3. Important Rules
Rules when using the module:

- Coordinates are written as `(x, y)`.
- Grid access uses `grid[y][x]`.
- `x` represents the column, and `y` represents the row.
- Row 0 is at the top.
- Movement is limited to UP, DOWN, LEFT, and RIGHT.

## 4. How to Run

First, open the complete project folder in PyCharm.

**Run the demo:**

1. Open `main.py`.
2. Right-click inside the file.
3. Select `Run 'main'`.

You will see the hospital map and the positions of people, fire, obstacles, and exits in the Run window.

**Run the tests:**

1. Open `tests/test_environment.py`.
2. Right-click inside the file.
3. Select `Run 'test_environment'` or the available Unittest run option.

## 5. How to Use the Environment

Create an environment:

```python
from environment.environment import Environment

env = Environment(20, 12)
```

### Map Symbols

When you run `main.py`, the hospital map uses these symbols:

- `F` = Fire
- `O` = Obstacle
- `P` = Person who needs rescue
- `E` = Emergency Exit
- `#` = Wall
- `.` = Free Floor
- `?` = Unknown cell type

These symbols help you understand the hospital layout in the console.


Useful methods:

| Method                    | Purpose                           |
| ------------------------- | --------------------------------- |
| `env.is_walkable(x, y)`   | Checks if a cell is accessible.   |
| `env.get_neighbors(x, y)` | Gets available neighboring cells. |
| `env.get_exits()`         | Gets exit positions.              |
| `env.get_people()`        | Gets people positions.            |
| `env.get_fire_zones()`    | Gets fire positions.              |
| `env.get_obstacles()`     | Gets obstacle positions.          |
| `env.move_cost(x, y)`     | Gets movement cost.               |

You can also update the environment using methods like `add_fire()`, `add_obstacle()`, and `add_person()`.

## 6. Current Implementation

- Fire and obstacles cannot be crossed.
- Movement cost is currently `1`.
- Only four-directional movement is supported.
- Demo data is written directly in `main.py`.
- No external dataset is required.

## 7. What Can Be Added Later?

Possible improvements:

- Different movement costs.
- Fire spreading simulation.
- More building layouts.
- Integration with the rescue agent and pathfinding algorithms.

Note: The Environment module provides the building and its information. Agent movement and the actual rescue simulation can be implemented by other modules.

## 4. Added for integration with the other modules

- **Priorities:** `add_person(x, y, priority)`, `get_priority`, `get_people_by_priority()` (for CSP).
- **Robot start cells:** `add_robot_base(x, y)`, `get_robot_bases()` (for Multi-Agent).
- **Smoke:** `add_smoke`, `spread_smoke`; `move_cost` is 5 inside smoke.
- **One simulation step:** `env.step()` spreads fire and smoke and returns `{"new_fire", "new_smoke", "people_hit"}`.
- **Casualties:** `get_people_in_fire()`, `mark_casualty(x, y)`.
- **Same scenario for both systems:** `env.copy()` and `scenarios.build_random_scenario(seed, ...)`.
- `env.min_move_cost` tells A* the cheapest possible step (keep every cost >= 1).
