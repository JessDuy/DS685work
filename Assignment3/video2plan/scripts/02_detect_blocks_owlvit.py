import os, glob, json
from pathlib import Path
from PIL import Image
from tqdm import tqdm
from transformers import pipeline

FRAMES_DIR = "outputs/frames"
OUT_DIR = "outputs/detections"
os.makedirs(OUT_DIR, exist_ok=True)

labels = ["red block","blue block","green block","yellow block","wooden block","toy block","plastic block","building block","cube","lego brick","brick","colored cube","stacking block","block"]

detector = pipeline("zero-shot-object-detection",
                    model="google/owlvit-base-patch32", device=-1)

def norm_box(box, w, h):
    x1, y1, x2, y2 = box["xmin"], box["ymin"], box["xmax"], box["ymax"]
    return [max(0,int(x1)), max(0,int(y1)), min(w,int(x2)), min(h,int(y2))]

for ep in sorted(glob.glob(f"{FRAMES_DIR}/*")):
    if not os.path.isdir(ep): continue
    episode = Path(ep).name
    ep_out = Path(OUT_DIR) / f"{episode}.jsonl"
    if ep_out.exists():
        print("skip", episode); continue

    with open(ep_out, "w") as f:
        for jf in tqdm(sorted(glob.glob(f"{ep}/*.jpg"))):
            img = Image.open(jf).convert("RGB")
            w, h = img.size
            results = detector(img, candidate_labels=labels)
            dets = []
            for r in results:
                if r["score"] < 0.05:
                    continue
                dets.append({
                    "label": r["label"],
                    "score": float(r["score"]),
                    "box": norm_box(r["box"], w, h)
                })
            f.write(json.dumps({"frame": os.path.basename(jf), "width": w, "height": h, "detections": dets})+"\n")
    print("wrote", ep_out)
