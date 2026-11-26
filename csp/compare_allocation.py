"""Demo hospital schedule and Greedy vs CSP benchmark (`python -m csp.compare_allocation`)."""
from pathlib import Path
from statistics import mean
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from csp import RobotSpec, allocate_tasks, greedy_allocate_tasks, plan_all_waves
from scenarios import build_random_scenario
from scenarios.demo_environment import build_demo_environment

N_SCENARIOS = 30
FLEET_BATTERIES = (34, 18, 14)


def print_demo_hospital() -> None:
    """Print Wave-1 and full multi-wave CSP schedules on the sample hospital."""
    env = build_demo_environment()
    robots = [
        RobotSpec(f"R{i}", pos=pos, battery=75)
        for i, pos in enumerate(env.get_robot_bases())
    ]

    print("=== Demo Hospital: Wave 1 CSP Allocation ===")
    res = allocate_tasks(env, robots=robots)
    print(f"{'Robot':<8}{'Start':<10}{'Person':<10}{'Prio':<6}{'Exit':<10}{'Cost':<6}")
    for task in res.tasks:
        print(
            f"{str(task.robot_id):<8}{str(task.robot_pos):<10}"
            f"{str(task.person):<10}{task.priority:<6}"
            f"{str(task.exit):<10}{task.total_cost:<6.0f}"
        )
    print(f"Incumbent CSP solutions evaluated: {res.num_solutions}")
    print(
        f"Wave 1 -> Priority sum: {res.total_priority}, "
        f"Total cost: {res.total_cost:.0f}, Makespan: {res.makespan:.0f}"
    )
    print(f"Queued for next waves: {res.unassigned_people}")

    print("\n=== Demo Hospital: Full Multi-Wave Schedule ===")
    waves, stranded = plan_all_waves(env, robots=robots)
    for idx, wave in enumerate(waves, start=1):
        pairs = ", ".join(
            f"{t.robot_id}->{t.person} (p={t.priority}, cost={t.total_cost:.0f})"
            for t in wave.tasks
        )
        print(f"Wave {idx}: {pairs}")
    print(f"Stranded people: {stranded}")


def run_benchmark() -> None:
    """Compare myopic Greedy vs CSP across seeded scarce-battery scenarios."""
    greedy_assigned, greedy_priority, greedy_makespan = [], [], []
    csp_assigned, csp_priority, csp_makespan = [], [], []

    for seed in range(N_SCENARIOS):
        env = build_random_scenario(
            seed=seed,
            n_people=8,
            n_fires=2,
            n_obstacles=3,
            n_smoke=3,
            n_robots=len(FLEET_BATTERIES),
        )
        robots = [
            RobotSpec(f"R{i}", pos=pos, battery=battery)
            for i, (pos, battery) in enumerate(zip(env.get_robot_bases(), FLEET_BATTERIES))
        ]

        g_res = greedy_allocate_tasks(env, robots=robots)
        c_res = allocate_tasks(env, robots=robots)

        greedy_assigned.append(len(g_res.assigned_people()))
        greedy_priority.append(g_res.total_priority)
        greedy_makespan.append(g_res.makespan)

        csp_assigned.append(len(c_res.assigned_people()))
        csp_priority.append(c_res.total_priority)
        csp_makespan.append(c_res.makespan)

    print(
        f"\n=== Benchmark across {N_SCENARIOS} seeded scenarios "
        f"(batteries={FLEET_BATTERIES}) ==="
    )
    print(f"{'Allocator':<12}{'Avg Assigned':>14}{'Avg Priority':>14}{'Avg Makespan':>14}")
    print(
        f"{'Greedy':<12}{mean(greedy_assigned):>14.2f}"
        f"{mean(greedy_priority):>14.2f}{mean(greedy_makespan):>14.2f}"
    )
    print(
        f"{'CSP':<12}{mean(csp_assigned):>14.2f}"
        f"{mean(csp_priority):>14.2f}{mean(csp_makespan):>14.2f}"
    )

    out_dir = Path(__file__).resolve().parent.parent / "outputs"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / "csp_comparison.png"

    fig, axes = plt.subplots(1, 3, figsize=(11, 3.5))
    panels = [
        ("Wave-1 Assigned Tasks", greedy_assigned, csp_assigned),
        ("Wave-1 Priority Score", greedy_priority, csp_priority),
        ("Wave-1 Makespan Cost", greedy_makespan, csp_makespan),
    ]
    for ax, (title, g_vals, c_vals) in zip(axes, panels):
        ax.bar(["Greedy", "CSP"], [mean(g_vals), mean(c_vals)], color=["#888", "#2a9d8f"])
        ax.set_title(title)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Saved chart: {out_path}")


if __name__ == "__main__":
    print_demo_hospital()
    run_benchmark()
