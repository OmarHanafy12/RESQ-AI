# RESQ-AI

## Project Structure

```text
RESQ-AI/
├── environment/                # Grid, building layouts, fire & smoke simulation
│   ├── __init__.py
│   ├── building_layout.py      # Building/hospital map builders
│   ├── cell.py                 # Cell walkability & console symbols
│   ├── constants.py            # Directions (UP/DOWN/LEFT/RIGHT) & cell types
│   ├── environment.py          # Main Environment class & simulation step
│   ├── grid.py                 # 2D grid container
│   └── README.md               # Environment module documentation
│
├── pathfinding/                # Pathfinding algorithms (UCS & A*) & benchmarks
│   ├── __init__.py             # Public pathfinding exports
│   ├── search.py               # UCS, A*, nearest exit/target, cost matrix, validation
│   ├── mock_grids.py           # Standalone grids for unit testing
│   ├── compare_algorithms.py   # UCS vs A* comparison benchmark
│   └── README.md               # Pathfinding module documentation
│
├── csp/                        # CSP rescue task allocation & resource optimization
│   ├── __init__.py             # Public CSP exports
│   ├── models.py               # RobotSpec, TaskAssignment & AllocationResult models
│   ├── constraints.py          # Feasibility & forward-checked NoTaskConflict constraints
│   ├── allocator.py            # Branch-and-bound CSP solver & multi-wave planner
│   ├── baseline.py             # Greedy nearest-person baseline allocator
│   ├── compare_allocation.py   # Demo hospital schedule & Greedy vs CSP benchmark
│   └── README.md               # CSP module documentation
│
├── scenarios/                  # Built-in demos & reproducible seeded scenarios
│   ├── __init__.py             # Exports build_random_scenario
│   ├── demo_environment.py     # Sample hospital scenario & console viewer
│   └── scenarios.py            # Reproducible seeded random scenario generator
│
├── tests/                      # Automated tests for all modules
│   ├── test_environment.py     # Environment & scenario tests
│   ├── test_pathfinding.py     # Pathfinding & integration tests
│   └── test_csp.py             # CSP task allocation & constraint tests
│
├── outputs/                    # Committed charts & experiment outputs
│   ├── comparison.png          # UCS vs A* comparison chart
│   └── csp_comparison.png      # Greedy vs CSP allocation benchmark chart
│
├── requirements.txt            # Project Python dependencies
├── .gitignore                  # Standard Python, IDE & OS ignore rules
└── README.md                   # Project structure overview
```
