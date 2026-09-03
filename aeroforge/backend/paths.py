"""Filesystem locations shared by the API and the CAD exporters.

Kept in a module with NO heavy imports so `backend.api` can know where the
exports live without paying for the cadquery/OpenCASCADE import chain at
server boot (the CAD itself is built in worker processes - see
`backend.cadjobs`).
"""
from __future__ import annotations

from pathlib import Path

#: <repo>/aeroforge/exports - same value exporters.EXPORT_DIR always had
EXPORT_DIR = Path(__file__).resolve().parent.parent / "exports"
