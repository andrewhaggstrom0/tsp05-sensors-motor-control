"""Checks foxglove/tsp05_layout.json against the bridge's topics (no ROS needed).

A renamed topic would otherwise only show up as an empty plot in the demo.
"""
import json
import re
import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))
from tsp05_bridge import topics as t  # noqa: E402

LAYOUT = PKG.parent.parent.parent / "foxglove" / "tsp05_layout.json"
pytestmark = pytest.mark.skipif(not LAYOUT.exists(), reason="foxglove/ not in this checkout")

KNOWN = {f"{t.NS}/{name}" for name in (
    *t.FLOAT_TOPICS.values(), *t.FLAG_TOPICS.values(), *t.FEEDBACK_TOPICS,
    *t.STATUS_TOPICS, *t.COMMAND_TOPICS)}


@pytest.fixture(scope="module")
def layout():
    return json.loads(LAYOUT.read_text())


def _leaves(node):
    if isinstance(node, str):
        yield node
    else:
        yield from _leaves(node["first"])
        yield from _leaves(node["second"])


def test_every_path_is_a_bridge_topic(layout):
    text = json.dumps(layout["configById"])
    paths = set(re.findall(r'"(/tsp05/[^".]+)\.data"', text))
    assert paths, "layout references no /tsp05 topics"
    assert paths <= KNOWN, f"unknown topics: {sorted(paths - KNOWN)}"


def test_mosaic_matches_panel_configs(layout):
    leaves = list(_leaves(layout["layout"]))
    assert len(leaves) == len(set(leaves)), "a panel appears twice"
    assert set(leaves) == set(layout["configById"])


def test_graded_plots_present(layout):
    """Raw vs filtered for TOF and flex is a graded item."""
    plotted = {p["value"] for c in layout["configById"].values() for p in c.get("paths", [])}
    for pair in (("tof/raw_mm", "tof/filt_mm"), ("flex/deg", "flex/deg_filt"), ("dc/rpm", "dc/target_rpm"),
                 ("dc/deg", "dc/target_deg"), ("thermistor/temp_c", "interlock/threshold_c")):
        assert {f"{t.NS}/{x}.data" for x in pair} <= plotted, pair


PANEL_SRC = LAYOUT.parent / "tsp05-control-panel" / "src" / "commands.ts"


@pytest.mark.skipif(not PANEL_SRC.exists(), reason="control panel source not in this checkout")
def test_control_panel_topics_match_bridge():
    """The TypeScript panel builds topics as `${NS}/...`; each must be a bridge topic,
    and it must cover every command topic the bridge subscribes to."""
    src = PANEL_SRC.read_text()
    ns = re.search(r'export const NS = "([^"]+)"', src).group(1)
    assert ns == t.NS
    used = {f"{ns}/{name}" for name in re.findall(r"`\$\{NS\}/([a-z_/]+)`", src)}
    assert used, "no topics found in commands.ts"
    assert used <= KNOWN, f"unknown topics: {sorted(used - KNOWN)}"
    commands = {f"{t.NS}/{c}" for c in t.COMMAND_TOPICS}
    assert commands <= used, f"panel cannot send: {sorted(commands - used)}"


CONFIG_H = LAYOUT.parent.parent / "firmware" / "tsp05" / "config.h"


@pytest.mark.skipif(not (PANEL_SRC.exists() and CONFIG_H.exists()), reason="panel or firmware not in this checkout")
@pytest.mark.parametrize("key,define", [
    ("flexDeadzoneDeg", "MAP_FLEX_DEADZONE_DEG"), ("flexFullDeg", "MAP_FLEX_FULL_DEG"),
    ("dcMaxRpm", "MAP_DC_MAX_RPM"), ("swipeServoDegS", "GESTURE_SERVO_DEG_S"),
    ("slotJogDeg", "MAP_SLOT_JOG_DEG"),
])
def test_panel_sensor_map_matches_config_h(key, define):
    """The panel's SENSOR-mode mapping text must match the firmware's constants."""
    panel = re.search(rf"\b{key}: (-?[\d.]+)", PANEL_SRC.read_text())
    firmware = re.search(rf"^\s*#define\s+{define}\s+(-?[\d.]+)", CONFIG_H.read_text(), re.MULTILINE)
    assert panel and firmware, (key, define)
    assert float(panel.group(1)) == float(firmware.group(1))


def test_layout_includes_the_control_panel(layout):
    """Foxglove names extension panels "<extension name>.<panel name>!<id>"."""
    assert any(k.startswith("tsp05-control-panel.TSP05 Controls!") for k in layout["configById"])


def test_every_plot_has_a_header_title(layout):
    """Foxglove shows foxglovePanelTitle in the panel header; without it every plot reads "Plot"."""
    for key, cfg in layout["configById"].items():
        if key.startswith("Plot!"):
            assert cfg.get("foxglovePanelTitle"), key
