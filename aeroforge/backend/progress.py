"""Build progress + ETA bookkeeping for the long CAD jobs (exports) and the
multi-type generate.

Two halves, both stdlib-only so the server process and the spawned CAD
workers can import this without paying for cadquery:

* **Worker side** - `begin(path)` names a per-job JSON progress file and
  `report(stage)` appends a (stage, timestamp) mark to it. The builders call
  `report` at their stage boundaries; when no job is active (previews, tests,
  scripts) `report` is a no-op, so the builders' behaviour is unchanged.

* **Server side** - `TimingStore` keeps a rolling history of finished
  durations per kind (`"<airplane_type>:<format>"` for exports,
  `"generate:<airplane_type>"` for the optimizer) in `exports/timing.json`,
  and `status_from_file()` turns a progress file plus that history into the
  `{stage, progress, eta_s}` triple `/api/export/status` returns. The bar is
  proportional to REAL time: each stage's share of the bar is the median
  share it took in the last runs, not an equal weight per stage. With no
  history the bar still advances stage by stage and the ETA is `None`.
"""
from __future__ import annotations

import json
import os
import statistics
import tempfile
import threading
import time
from pathlib import Path

__all__ = ["begin", "end", "report", "STAGES", "STAGE_LABELS",
           "TimingStore", "read_marks", "status_from_file",
           "stage_durations"]

# Build order of the flying-wing path (geometry._build_parts and the
# exporters). Other type modules report whichever of these they reach; a
# stage name not in this list still shows its label and simply advances the
# bar by time. "queued" is the wait for the per-design CAD lock / a free
# worker; "write" is tessellation + file output.
STAGES = ["queued", "loft", "bay", "fins", "hinges", "servos", "fuse", "split",
          "write"]
STAGE_LABELS = {
    "queued": "Waiting for a CAD worker",
    "loft": "Lofting the airframe",
    "bay": "Hollowing the equipment bay",
    "fins": "Building the vertical surfaces",
    "hinges": "Cutting the hinges",
    "servos": "Placing servos and routing wire runs",
    "fuse": "Fusing and healing the solid",
    "split": "Separating the parts",
    "write": "Writing the file",
    "done": "Done",
}
HISTORY_N = 5          # rolling window per kind (median of the last N)

# ---------------------------------------------------------------------------
# worker side
# ---------------------------------------------------------------------------
_SINK: dict = {"path": None}
_SINK_LOCK = threading.Lock()


def begin(path: str | os.PathLike | None) -> None:
    """Start reporting to `path` (None disables). Called by the job function
    inside the worker process; the file is (re)created with a "queued" mark
    so a status poll never sees a half-written file."""
    with _SINK_LOCK:
        _SINK["path"] = Path(path) if path else None
    if path:
        _write_marks(Path(path), [{"stage": "queued", "t": time.time()}])


def end() -> None:
    """Stop reporting (the worker is reused for previews afterwards)."""
    report("done")
    with _SINK_LOCK:
        _SINK["path"] = None


def report(stage: str) -> None:
    """Mark the start of `stage`. No-op unless a job began reporting."""
    with _SINK_LOCK:
        path = _SINK["path"]
    if path is None:
        return
    try:
        marks = read_marks(path)
        marks.append({"stage": str(stage), "t": time.time()})
        _write_marks(path, marks)
    except Exception:
        pass                      # progress is cosmetic; never fail a build


def _write_marks(path: Path, marks: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, suffix=".tmp")
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        json.dump({"marks": marks}, fh)
    os.replace(tmp, path)


def read_marks(path: str | os.PathLike) -> list[dict]:
    try:
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
        marks = data.get("marks") or []
        return [m for m in marks if "stage" in m and "t" in m]
    except (OSError, ValueError):
        return []


def stage_durations(marks: list[dict], t_end: float | None = None
                    ) -> dict[str, float]:
    """Seconds spent in each stage: a stage runs from its mark to the next
    one (or `t_end` / now for the last). Repeated stage names accumulate."""
    out: dict[str, float] = {}
    for i, m in enumerate(marks):
        if m["stage"] == "done":
            continue
        t1 = marks[i + 1]["t"] if i + 1 < len(marks) else (
            t_end if t_end is not None else time.time())
        out[m["stage"]] = out.get(m["stage"], 0.0) + max(0.0, t1 - m["t"])
    return out


# ---------------------------------------------------------------------------
# server side
# ---------------------------------------------------------------------------
class TimingStore:
    """Rolling per-kind history of finished build durations, persisted as
    JSON. Thread-safe; every write goes to disk atomically."""

    def __init__(self, path: str | os.PathLike, history: int = HISTORY_N):
        self.path = Path(path)
        self.history = history
        self._lock = threading.Lock()
        self._data: dict[str, list[dict]] = self._load()

    def _load(self) -> dict[str, list[dict]]:
        try:
            with open(self.path, encoding="utf-8") as fh:
                data = json.load(fh)
            if isinstance(data, dict):
                return {k: list(v) for k, v in data.items()
                        if isinstance(v, list)}
        except (OSError, ValueError):
            pass
        return {}

    def _save(self) -> None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=self.path.parent, suffix=".tmp")
            with os.fdopen(fd, "w", encoding="utf-8") as fh:
                json.dump(self._data, fh, indent=1)
            os.replace(tmp, self.path)
        except OSError:
            pass

    def record(self, kind: str, duration_s: float,
               stages: dict[str, float] | None = None) -> None:
        with self._lock:
            runs = self._data.setdefault(kind, [])
            runs.append({"duration_s": round(float(duration_s), 3),
                         "stages": {k: round(float(v), 3)
                                    for k, v in (stages or {}).items()},
                         "at": time.time()})
            del runs[:-self.history]
            self._save()

    def runs(self, kind: str) -> list[dict]:
        with self._lock:
            return list(self._data.get(kind, []))

    def expected_s(self, kind: str) -> float | None:
        """Median duration of the last runs of this kind, or None."""
        runs = self.runs(kind)
        if not runs:
            return None
        return float(statistics.median(r["duration_s"] for r in runs))

    def stage_fractions(self, kind: str) -> dict[str, float] | None:
        """Each stage's share of a run's wall time, learned from history
        (median seconds per stage, normalised). None without history."""
        runs = [r for r in self.runs(kind) if r.get("stages")]
        if not runs:
            return None
        names = list(dict.fromkeys(s for r in runs for s in r["stages"]))
        med = {n: float(statistics.median(r["stages"].get(n, 0.0)
                                          for r in runs)) for n in names}
        total = sum(med.values())
        if total <= 0:
            return None
        return {n: v / total for n, v in med.items()}

    def summary(self) -> dict:
        """Everything the frontend needs to draw an ETA before a job starts:
        {kind: {expected_s, n}}."""
        with self._lock:
            kinds = list(self._data)
        return {k: {"expected_s": self.expected_s(k), "n": len(self.runs(k))}
                for k in kinds}


def _stage_index(stage: str) -> int:
    try:
        return STAGES.index(stage)
    except ValueError:
        return len(STAGES) - 1


def status_from_file(path: str | os.PathLike | None, kind: str,
                     store: TimingStore | None, started_at: float,
                     now: float | None = None) -> dict:
    """The live `{stage, stage_label, progress, elapsed_s, eta_s}` for a
    running job.

    progress is 0-1. With history the bar is proportional to real time:
    the completed stages' learned shares plus the running stage's share
    scaled by its elapsed/expected time (never past its own end, so a slow
    run stalls at the stage boundary instead of lying). Without history each
    completed stage advances the bar by an equal step and eta_s is None.
    """
    now = time.time() if now is None else now
    marks = read_marks(path) if path else []
    stage = marks[-1]["stage"] if marks else "queued"
    elapsed = max(0.0, now - started_at)
    expected = store.expected_s(kind) if store else None
    fracs = store.stage_fractions(kind) if store else None
    stage_t0 = marks[-1]["t"] if marks else started_at

    if stage == "done":
        progress = 1.0
    elif fracs and expected:
        seen = list(dict.fromkeys(m["stage"] for m in marks))
        before = sum(fracs.get(s, 0.0) for s in seen if s != stage)
        share = fracs.get(stage)
        if share is None:                      # a stage history never saw
            share = max(0.0, 1.0 - before) * 0.5
        in_stage = max(0.0, now - stage_t0) / max(1e-6, share * expected)
        progress = before + share * min(0.98, in_stage)
        progress = min(0.99, max(0.0, progress))
    else:
        idx = _stage_index(stage)
        progress = min(0.99, idx / len(STAGES))

    if expected and stage == "queued":
        # waiting for a worker / the per-design lock: nothing of the build
        # has run yet, so the ETA must not count down (audit 2026-08-28 saw
        # 1871 -> 1153 s over 12 min of pure queueing)
        eta = expected
        eta_s = round(eta, 1)
    elif expected:
        eta = max(0.0, expected - elapsed)
        if elapsed > expected:        # overdue: estimate from the bar instead
            eta = max(0.0, (1.0 - progress) * elapsed / max(progress, 0.05))
        eta_s = round(eta, 1)
    else:
        eta_s = None
    return {"stage": stage,
            "stage_label": STAGE_LABELS.get(stage, stage.replace("_", " ")),
            "progress": round(progress, 4),
            "elapsed_s": round(elapsed, 1),
            "eta_s": eta_s,
            "expected_s": round(expected, 1) if expected else None}
