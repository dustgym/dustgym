"""Guarded tests for the rclpy nodes — skipped on a bare CPU runner, exercised in the ROS container."""
from __future__ import annotations

import importlib
import os
import sys

import pytest

pytest.importorskip("rclpy", reason="ROS 2 (rclpy) only present in the container")

# make the nodes/ dir importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "nodes"))


def test_node_modules_import():
    # importing proves the rclpy/message APIs the nodes use resolve in this environment
    assert importlib.import_module("ccsds_bridge_node") is not None
    assert importlib.import_module("rover_executive_node") is not None
