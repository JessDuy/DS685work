import json, os, glob
from pathlib import Path

STATE_DIR = "outputs/states"
OUT_DIR = "pddl/problems"
os.makedirs(OUT_DIR, exist_ok=True)

def pred_to_pddl(p):
    if p[0] == "handempty":
        return "(handempty)"
    if p[0] in ("on-table","clear"):
        return f"({p[0]} {p[1]})"
    if p[0] in ("on","holding"):
        return f"({p[0]} {p[1]} {p[2]})" if len(p)==3 else f"({p[0]} {p[1]})"
    return None

def write_problem(episode, init, goal):
    objs = sorted(set(init["objects"]) | set(goal["objects"]))
    def filter_preds(preds):
        keep = []
        for p in preds:
            names = p[1:]
            if all(n in objs for n in names):
                keep.append(p)
        return keep

    init_preds = filter_preds(init["predicates"])
    goal_preds = filter_preds(goal["predicates"])

    seen = set()
    for p in init_preds:
        for n in p[1:]:
            seen.add(n)
    for b in objs:
        if b not in seen:
            init_preds.append(("on-table", b))
            init_preds.append(("clear", b))

    init_str = "\n      ".join(sorted(set(pred_to_pddl(p) for p in init_preds if pred_to_pddl(p))))
    goal_str = "\n      ".join(sorted(set(pred_to_pddl(p) for p in goal_preds if pred_to_pddl(p))))

    txt = f"""(define (problem {episode})
  (:domain blocks)
  (:objects {' '.join(objs)} - block)
  (:init
      {init_str}
  )
  (:goal (and
      {goal_str}
  ))
)
"""
    out = Path(OUT_DIR) / f"problem_{episode}.pddl"
    out.write_text(txt)
    print("wrote", out)

files = sorted(glob.glob(f"{STATE_DIR}/*.json"))[:10]
for fp in files:
    data = json.loads(Path(fp).read_text())
    write_problem(data["episode"], data["init"], data["goal"])
