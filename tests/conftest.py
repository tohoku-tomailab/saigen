"""pytest common setup: put the repository root on the import path.

Run the suite from anywhere with:  pytest tests/
"""
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import pytest  # noqa: E402


@pytest.fixture(autouse=True)
def _clear_li5640_instrument_locks():
    """The LI5640 driver keeps one lock per instrument for the whole process (a release
    still running blocks the next open). A test that leaves a fake instrument open must
    not block the next test, so the registry is emptied after each test."""
    yield
    from src.devices.li5640 import li5640
    with li5640._REGISTRY_LOCK:
        li5640._RESOURCE_LOCKS.clear()
