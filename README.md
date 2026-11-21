# 🚨 RESQ-AI

**An AI-powered rescue management system for disaster scenarios.**

RESQ-AI simulates an emergency (such as a fire) inside a building like a hospital or university. People need to be evacuated, corridors get blocked, hazards spread dynamically, and rescue robots must be coordinated in real time. The system uses AI techniques to plan safe routes, allocate rescue tasks under constraints, coordinate multiple robots, and replan when the environment changes.

---

## 📌 Problem

During an emergency, evacuation decisions are made under pressure, with incomplete information, and in a constantly changing environment. Traditional approaches (fixed routes, nearest-exit rules, manual coordination) fail when corridors become blocked, fire spreads, or robot resources are limited.

## 💡 Our Solution

A multi-agent simulation where:

- **Environment (`environment/`)** models the building as a 2D grid (rooms, corridors, walls, exits, people with priorities, obstacles, fire, and smoke).
- **Pathfinding (`pathfinding/`)** uses **A\*** and **Uniform Cost Search (UCS)** to compute optimal, hazard-aware routes.
- **CSP (`csp/` — planned)** assigns rescue tasks to robots under constraints (battery, capacity, priority).
- **Multi-Agent Coordination (`agents/` — planned)** moves robots concurrently without collisions and triggers dynamic replanning.
- **Visualization (`visualization/` — planned)** renders a live top-down Pygame dashboard.
- **Evaluation (`evaluation/` — planned)** compares a traditional baseline system against our AI system on identical seeded scenarios.

---

## 🗂️ Project Structure

### Current Repository Layout

```text
RESQ-AI/
├── environment/                # Member 1: Grid, building layout, fire & smoke simulation
│   ├── __init__.py
│   ├── building_layout.py      # Building/hospital map builders
│   ├── cell.py                 # Cell walkability & console symbols
│   ├── constants.py            # Shared directions (UP/DOWN/LEFT/RIGHT) & cell types
│   ├── environment.py          # Main Environment class & step() simulation
│   ├── grid.py                 # 2D grid container
│   └── README.md               # Environment module documentation
│
├── pathfinding/                # Member 2: UCS, A*, path helpers & benchmarks
│   ├── __init__.py             # Public pathfinding exports
│   ├── search.py               # find_path_env, nearest_exit, cost_matrix, is_path_valid, etc.
│   ├── mock_grids.py           # Standalone test grids
│   ├── compare_algorithms.py   # UCS vs A* benchmark & chart generator
│   └── README.md               # Pathfinding module documentation
│
├── scenarios/                  # Built-in demos & reproducible seeded scenarios
│   ├── __init__.py             # Public scenario exports
│   ├── demo_environment.py     # Runnable sample hospital scenario & console viewer
│   └── scenarios.py            # Reproducible seeded random scenarios (build_random_scenario)
│
├── tests/                      # Automated test suite for all modules
│   ├── test_environment.py     # Environment & scenario tests
│   └── test_pathfinding.py     # Pathfinding & integration tests
│
├── outputs/                    # Committed charts and experiment outputs
│   └── comparison.png          # UCS vs A* benchmark chart
│
├── requirements.txt            # Python dependencies for the project
├── .gitignore                  # Standard Python, IDE, and OS ignore rules
└── README.md                   # This file
```

### Planned Module Folders (For Teammates Adding New Work)

When you implement your assigned module, add it as a top-level package right alongside `environment/` and `pathfinding/`:

| Folder | Owner | Responsibility | Tests Location |
|---|---|---|---|
| `environment/` | Member 1 | Grid, rooms, walls, exits, people, hazards, fire/smoke spread | `tests/test_environment.py` |
| `pathfinding/` | Member 2 | A\*, UCS, path cost helpers, algorithm comparison | `tests/test_pathfinding.py` |
| `csp/` | Member 3 | Task allocation via Constraint Satisfaction (CSP) | `tests/test_csp.py` |
| `agents/` | Member 4 | Rescue robots, movement, coordination, dynamic replanning | `tests/test_agents.py` |
| `visualization/` | Member 5 | Pygame simulation UI and live dashboard | `tests/test_visualization.py` |
| `evaluation/` | Member 6 | Baseline system, experiments, metrics, charts, main simulation loop | `tests/test_evaluation.py` |

> **Team Guidelines:**
> 1. **Keep each module self-contained** in its own top-level folder with an `__init__.py` and a short `README.md` explaining its public interface.
> 2. **Put all automated tests inside `tests/`** named `test_<module>.py` so `python -m pytest` discovers and runs everything in one command.
> 3. **Put generated charts/results you want to share inside `outputs/`** (this folder is tracked in Git).
> 4. **Always run commands from the `RESQ-AI/` root directory** so cross-module imports (`from environment...`, `from pathfinding...`, `from scenarios...`) resolve consistently.
> 5. **Add any new Python libraries** your module needs to the root `requirements.txt`.

---

## 🤝 Shared Conventions Across All Modules

To ensure seamless integration across teammates, follow these rules:

- **Coordinates:** Always `(x, y)` where `x` is the column and `y` is the row (`y = 0` is the top row).
- **Grid indexing:** Internally stored as `grid[y][x]`. Prefer calling `Environment` methods instead of indexing the grid directly.
- **Movement:** 4-directional only (`UP`, `DOWN`, `LEFT`, `RIGHT` from `environment.constants`).
- **Move costs:** Every walkable cell must have `move_cost >= 1` (`env.min_move_cost == 1`) so A\*'s Manhattan heuristic remains admissible. Smoke is walkable with cost `5`.
- **Reproducibility:** Use `scenarios.build_random_scenario(seed=...)` and `env.copy()` so the baseline and AI systems are evaluated on the exact same emergency.

---

## 🧠 Where is the AI?

| Component | Technique | Role |
|---|---|---|
| Route planning | A\*, Uniform Cost Search (UCS) | Optimal paths with hazard-aware costs |
| Task allocation | Constraint Satisfaction (CSP) | Assigns robots to victims under constraints |
| Coordination | Multi-Agent Systems | Cooperative, collision-free movement |
| Adaptation | Dynamic Replanning | Recomputes routes when fire or obstacles block paths |

---

## ⚙️ Setup & Installation

```bash
git clone https://github.com/OmarHanafy12/RESQ-AI.git
cd RESQ-AI

# (Recommended) Create and activate a virtual environment
python -m venv .venv
source .venv/bin/activate        # Linux / macOS
# .venv\Scripts\activate         # Windows

# Install dependencies
pip install -r requirements.txt
```

---

## ▶️ How to Run

Run all commands from the project root (`RESQ-AI/`):

```bash
# Run all automated tests (Environment + Pathfinding)
python -m pytest

# Run only one module's tests
python -m pytest tests/test_environment.py
python -m pytest tests/test_pathfinding.py

# Run the demo hospital environment in the console
python -m scenarios.demo_environment

# Benchmark and compare UCS vs A*
python -m pathfinding.compare_algorithms
```

### Quick Code Examples

**Environment & Scenarios (`environment/`, `scenarios/`):**
```python
from scenarios.demo_environment import build_demo_environment
from scenarios import build_random_scenario

env = build_demo_environment()
# or reproducible random emergency:
env = build_random_scenario(seed=42)
```
Full documentation: [environment/README.md](environment/README.md)

**Pathfinding (`pathfinding/`):**
```python
from pathfinding import find_path_env, nearest_exit, cost_matrix, is_path_valid

path, cost, nodes = find_path_env(env, start=(3, 2), goal=(0, 8), algorithm="astar")
exit_cell, exit_path, exit_cost = nearest_exit(env, start=(3, 2))
```
Full documentation: [pathfinding/README.md](pathfinding/README.md)

---

## 📊 Evaluation Metrics

Both the baseline system and the AI system will be run on the **same seeded scenarios** and compared on:

- Total evacuation time (simulation steps)
- Average and maximum rescue time per person
- Total path cost
- Number of casualties (people reached by fire before rescue)
- Robot utilization

---

## 🛠️ Tech Stack

- **Python 3**
- **Pygame** — visualization
- **python-constraint** — CSP task allocation
- **Matplotlib** — benchmark & evaluation charts
- **pytest / unittest** — automated testing

---

## 🗓️ Roadmap

- [x] Setup, repository structure, and interface agreement
- [x] Environment & fire/smoke simulation (`environment/`)
- [x] Pathfinding with UCS & A\* (`pathfinding/`)
- [ ] CSP task allocation (`csp/`)
- [ ] Multi-agent coordination & dynamic replanning (`agents/`)
- [ ] Pygame visualization & live dashboard (`visualization/`)
- [ ] Baseline system, evaluation experiments & metrics (`evaluation/`)
- [ ] Final testing, documentation, and presentation

---

## 📄 License

Educational project. Add a license of your choice (e.g., MIT).
