from unified_planning.io import PDDLReader
from unified_planning.shortcuts import OneshotPlanner
import glob, pathlib

DOMAIN = "pddl/domain.pddl"
reader = PDDLReader()

for PROBLEM in sorted(glob.glob("pddl/problems/problem_*.pddl")):
    # quick guard: skip files with empty objects
    text = pathlib.Path(PROBLEM).read_text()
    if "(:objects  - block)" in text:
        print("Skipping (empty objects):", PROBLEM)
        continue

    print("Checking", PROBLEM)
    problem = reader.parse_problem(DOMAIN, PROBLEM)
    with OneshotPlanner(name="pyperplan") as planner:
        if planner is None:
            raise RuntimeError("Pyperplan not available. pip install up-pyperplan")
        res = planner.solve(problem)
    print("Status:", res.status)
    if res.plan:
        actions = getattr(res.plan, "actions", None)
        print("Plan length:", len(actions) if actions is not None else res.plan)
