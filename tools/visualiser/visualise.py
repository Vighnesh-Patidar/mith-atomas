#!/usr/bin/env python3
"""
2D matplotlib visualiser for the MithAtomas flocking demo (§9.3).

Reads JSON-line snapshots from stdin (one per tick), each of shape:
    {"tick": <int>, "robots": [{"id": "...", "x": <float>, "y": <float>}, ...]}

Usage:
    ./build/examples/flocking_demo/flocking_demo | python3 tools/visualiser/visualise.py

Requires: matplotlib. No other deps.
"""

from __future__ import annotations

import json
import sys

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
except ImportError:
    print("This visualiser needs matplotlib. Install with: pip install matplotlib",
          file=sys.stderr)
    sys.exit(1)


def read_frames(stream):
    frames = []
    for line in stream:
        line = line.strip()
        if not line:
            continue
        try:
            frames.append(json.loads(line))
        except json.JSONDecodeError as e:
            print(f"skipping malformed line: {e}", file=sys.stderr)
    return frames


def main() -> int:
    frames = read_frames(sys.stdin)
    if not frames:
        print("No frames received on stdin.", file=sys.stderr)
        return 1

    # World bounds across the entire run, with padding.
    all_x = [r["x"] for f in frames for r in f["robots"]]
    all_y = [r["y"] for f in frames for r in f["robots"]]
    pad = 3.0
    xlim = (min(all_x) - pad, max(all_x) + pad)
    ylim = (min(all_y) - pad, max(all_y) + pad)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.grid(True, alpha=0.3)

    first = frames[0]
    xs = [r["x"] for r in first["robots"]]
    ys = [r["y"] for r in first["robots"]]
    scatter = ax.scatter(xs, ys, s=40)
    title = ax.set_title(f"tick {first['tick']} / {frames[-1]['tick']}")

    def update(idx):
        f = frames[idx]
        coords = [(r["x"], r["y"]) for r in f["robots"]]
        scatter.set_offsets(coords)
        title.set_text(f"tick {f['tick']} / {frames[-1]['tick']}")
        return [scatter, title]

    _anim = animation.FuncAnimation(
        fig, update, frames=len(frames), interval=50, blit=False, repeat=True,
    )
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
