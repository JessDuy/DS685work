# Assignment 3 — Video2Plan (DROID → PDDL)

This pipeline extracts **PDDL** specs from robot demonstration videos. It uses a Visual Language Model (OWL-ViT) to detect blocks, lifts those into symbolic predicates, and auto-writes a **Blocks World** domain + multiple problem files. Problems are validated with **Unified Planning** (Pyperplan).

## Project Layout
## Setup (macOS, Python venv)
```bash
cd Assignment3/video2plan
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
# if needed:
pip install "unified-planning==1.2.0" up-pyperplan
# add a demo video
cp ~/Downloads/test.mp4 data/droid/

# pipeline
python scripts/01_extract_frames.py
python scripts/02_detect_blocks_owlvit.py
python scripts/03b_build_states_multi.py
python scripts/04_generate_problems.py

# validate all problems with UP + Pyperplan
python scripts/05_validate_with_up.py
