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
    *t.FLOAT_TOPICS.values(), *t.FLAG_TOPICS.values(), *t.STATUS_TOPICS, *t.COMMAND_TOPICS)}


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
    for pair in (("tof/raw_mm", "tof/filt_mm"), ("flex/deg", "flex/deg_filt"), ("dc/rpm", "cmd/dc_rpm")):
        assert {f"{t.NS}/{x}.data" for x in pair} <= plotted, pair
