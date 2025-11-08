import cv2, os, glob
from pathlib import Path

SRC_DIR = "data/droid"
OUT_DIR = "outputs/frames"
FPS_SAMPLE = 2.0  # frames per second to sample

os.makedirs(OUT_DIR, exist_ok=True)

for vid in glob.glob(f"{SRC_DIR}/*.mp4"):
    cap = cv2.VideoCapture(vid)
    if not cap.isOpened():
        print("Cannot open:", vid); continue

    base = Path(vid).stem
    ep_dir = Path(OUT_DIR) / base
    ep_dir.mkdir(parents=True, exist_ok=True)

    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    stride = int(max(1, round(fps / FPS_SAMPLE)))
    i = 0; saved = 0

    while True:
        ok, frame = cap.read()
        if not ok: break
        if i % stride == 0:
            out = ep_dir / f"{saved:06d}.jpg"
            cv2.imwrite(str(out), frame)
            saved += 1
        i += 1
    cap.release()
    print(f"{base}: saved {saved} frames")
