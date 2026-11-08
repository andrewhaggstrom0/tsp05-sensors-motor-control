#!/usr/bin/env python3
"""Draw the Track 5 software architecture: Uno <-> tsp05_bridge <-> foxglove_bridge <-> Foxglove.

    python3 tools/draw_bridge_diagram.py      # -> docs/figures/bridge_architecture.png

Needs matplotlib (pip install -r tools/requirements.txt).
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "docs" / "figures" / "bridge_architecture.png"

INK = "#1f2328"
MUTED = "#5f6b7a"
BLUE = "#1a73e8"
ORANGE = "#e8710a"
FILL = {"uno": "#fdf1e6", "ros": "#e8f0fe", "gui": "#e6f4ea", "docker": "#f5f6f8"}


def box(ax, x, y, w, h, title, lines, fill, edge=INK):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12",
                                fc=fill, ec=edge, lw=1.4))
    ax.text(x + w / 2, y + h - 0.28, title, ha="center", va="top", fontsize=11.5, weight="bold", color=INK)
    for i, line in enumerate(lines):
        ax.text(x + 0.18, y + h - 0.78 - i * 0.36, line, ha="left", va="top", fontsize=8.6, color=INK)


def arrow(ax, x0, y0, x1, y1, label, color, above=True):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>", mutation_scale=14, lw=1.6, color=color))
    ax.text((x0 + x1) / 2, y0 + (0.14 if above else -0.14), label, ha="center",
            va="bottom" if above else "top", fontsize=8, color=color)


def main() -> None:
    fig, ax = plt.subplots(figsize=(17, 7))
    ax.set_xlim(0, 17.2)
    ax.set_ylim(0, 7.0)
    ax.axis("off")

    # Docker / laptop boundary
    ax.add_patch(FancyBboxPatch((4.55, 1.0), 8.0, 5.6, boxstyle="round,pad=0.02,rounding_size=0.15",
                                fc=FILL["docker"], ec=MUTED, lw=1.2, ls="--"))
    ax.text(4.75, 6.45, "Laptop: docker/ (ros:jazzy) or native Ubuntu + ROS 2", fontsize=9, color=MUTED, va="top")

    box(ax, 0.2, 2.0, 3.3, 3.6, "Arduino Uno", [
        "firmware/tsp05 (one loop)",
        "4 sensors, 3 motors",
        "",
        "or, with no hardware:",
        "tools/fake_arduino.py",
        "  pty, same protocol,",
        "  simulated sensors + motors",
    ], FILL["uno"])

    box(ax, 4.9, 1.55, 3.6, 4.3, "tsp05_bridge (rclpy)", [
        "reader thread: readline",
        "  -> parse_telemetry()",
        "  -> 18 fields, 26 topics",
        "malformed line: count, skip",
        "port lost: reopen every 1 s",
        "",
        "/tsp05/cmd/* callbacks",
        "  -> cmd_*() -> serial write",
        "  NaN / inf rejected",
    ], FILL["ros"])

    box(ax, 9.7, 2.45, 2.55, 2.5, "foxglove_bridge", [
        "websocket :8765",
        "topics /tsp05/*,",
        "  /rosout",
        "client publish",
    ], FILL["ros"])

    box(ax, 13.5, 1.2, 3.5, 4.9, "Foxglove app", [
        "foxglove/tsp05_layout.json",
        "  TOF, flex: raw vs filtered",
        "  DC RPM: measured vs cmd",
        "  stepper actual vs target",
        "  mode, interlock, connected",
        "",
        "TSP05 Controls panel",
        "  (foxglove/tsp05-control-panel)",
        "  mode, servo, DC, stepper,",
        "  STOP (works in both modes)",
    ], FILL["gui"])

    # Serial link
    arrow(ax, 3.5, 4.3, 4.9, 4.3, "", ORANGE)
    arrow(ax, 4.9, 3.1, 3.5, 3.1, "", BLUE)
    ax.text(4.0, 4.45, "T,... 20 Hz", ha="center", fontsize=8, color=ORANGE)
    ax.text(4.0, 2.75, "M S D P", ha="center", fontsize=8, color=BLUE)
    ax.text(4.0, 5.2, "USB serial\n115200", ha="center", fontsize=8.5, color=MUTED)

    # ROS topics
    arrow(ax, 8.5, 4.3, 9.7, 4.3, "", ORANGE)
    arrow(ax, 9.7, 3.1, 8.5, 3.1, "", BLUE)
    ax.text(9.1, 4.55, "ROS 2\ntopics", ha="center", fontsize=8.5, color=MUTED)

    # WebSocket
    arrow(ax, 12.25, 4.3, 13.5, 4.3, "", ORANGE)
    arrow(ax, 13.5, 3.1, 12.25, 3.1, "", BLUE)
    ax.text(12.95, 4.55, "Web-\nSocket", ha="center", fontsize=8.5, color=MUTED)

    ax.text(0.2, 0.6, "orange = telemetry (Uno -> GUI)    blue = commands (GUI -> Uno)", fontsize=9, color=MUTED)
    ax.text(0.2, 0.15, "Measured with the fake Arduino: telemetry 19.99 Hz on every topic; "
            "command -> telemetry round trip median 31 ms, p95 52 ms (n = 200).",
            fontsize=9, color=INK)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=160, bbox_inches="tight", facecolor="white")
    print(f"wrote {OUT.relative_to(REPO)}")


if __name__ == "__main__":
    main()
