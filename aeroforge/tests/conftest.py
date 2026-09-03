"""Shared test configuration.

The API's eager preview warm (api._warm_previews, speed pass 2026-08-21)
starts real CAD builds in background worker processes the moment a design is
generated - exactly right for the live app, exactly wrong under pytest,
where TestClient generates dozens of designs whose previews nothing will
ever look at. Off for every test; the endpoints' on-demand paths are what
the suites exercise, and they are byte-identical to what the warm builds.
"""
import os

os.environ.setdefault("AEROFORGE_EAGER_PREVIEWS", "0")
