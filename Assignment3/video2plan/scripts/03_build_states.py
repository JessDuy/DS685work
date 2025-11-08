import json, os, glob
from pathlib import Path

DET_DIR = "outputs/detections"
OUT_DIR = "outputs/states"
os.makedirs(OUT_DIR, exist_ok=True)

def is_on(top, bottom):
    tx1, ty1, tx2, ty2 = top
    bx1, by1, bx2, by2 = bottom
    t_cx = (tx1+tx2)/2; b_cx = (bx1+bx2)/2
    horiz_ok = abs(t_cx - b_cx) <= 0.6*((bx2-bx1)/2 + (tx2-tx1)/2)
    vertical_ok = ty2 <= by2 and (by2 - ty2) <= 0.35*(by2-by1 + ty2-ty1)
    overlap_x = max(0, min(tx2,bx2)-max(tx1,bx1)) > 0
    return horiz_ok and vertical_ok and overlap_x

def on_table(box, H):
    _, _, _, y2 = box
    return y2 >= 0.85 * H

for det_file in glob.glob(f"{DET_DIR}/*.jsonl"):
    episode = Path(det_file).stem
    with open(det_file) as f:
        frames = [json.loads(x) for x in f]
    if not frames: 
        continue

    first, last = frames[0], frames[-1]

    def build_state(frame):
        boxes = [d["box"] for d in frame["detections"] if "block" in d["label"]]
        with_idx = sorted([(i,b) for i,b in enumerate(boxes)], key=lambda t:(t[1][0], t[1][1]))
        obj_names = {i: f"b{k+1}" for k,(i,_) in enumerate(with_idx)}
        objects = list(obj_names.values())

        preds = set()
        for i,b in enumerate(boxes):
            if on_table(b, frame["height"]):
                preds.add(("on-table", obj_names[i]))

        for i, bi in enumerate(boxes):
            for j, bj in enumerate(boxes):
                if i==j: continue
                if is_on(bi, bj):
                    preds.add(("on", obj_names[i], obj_names[j]))

        has_top = set(p[2] for p in preds if p[0]=="on")
        for i,_ in enumerate(boxes):
            name = obj_names[i]
            if name not in has_top:
                preds.add(("clear", name))

        preds.add(("handempty",))
        return {"objects": objects, "predicates": sorted(list(preds))}

    init_state = build_state(first)
    goal_state = build_state(last)

    out_path = Path(OUT_DIR) / f"{episode}.json"
    with open(out_path, "w") as f:
        json.dump({"episode": episode, "init": init_state, "goal": goal_state}, f, indent=2)
    print("wrote", out_path)
