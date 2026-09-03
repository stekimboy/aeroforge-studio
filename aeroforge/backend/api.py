"""FastAPI endpoints (spec section 7).

POST /api/generate         -> physics for all 5 design variants + the primary
                              variant's preview STL
GET  /api/preview/{id}.stl -> the preview STL for any variant, BUILT LAZILY on
                              first request and cached afterwards
POST /api/export           -> write STEP/STL to exports\\ and return the file
GET  /api/options          -> planforms/missions/vstabs/airfoils/materials/
                              variants/presets
GET  /api/learn            -> static knowledge base

Why lazy previews: the physics for one variant takes ~0.4 s but its CAD takes
2-6 s. Building all five up front would put a ~25 s wall in front of the user.
So /api/generate runs the physics for all five (fast) and builds ONLY the
primary variant's solid, so the main 3D view renders immediately; the frontend
then prefetches the other four previews in the background, and each is built
once, under a per-id lock, on its first request.
"""
from __future__ import annotations

import json
import threading
import time
import traceback
from pathlib import Path

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, JSONResponse

from . import cadjobs
from .cad import brepcache
from . import progress as _progress
from .paths import EXPORT_DIR
from .physics.airfoils import LIBRARY, WING_AIRFOILS
from .physics.config_defs import (
    AIRPLANE_TYPES, CONV_SM_BAND, CONV_STYLES, MISSIONS, PLANFORMS, SM_BAND,
    VSTABS,
)
from .physics.guidance import LEARN_ARTICLES
from .physics.variants import (
    PRIMARY_KEY, generate_variants, primary_key_for, variant_catalog,
)
from .physics.weights import MATERIALS
from .schemas import ExportRequest, GenerateRequest

router = APIRouter(prefix="/api")

# Preview artifacts are content-addressed by design id: an id is a fresh
# uuid4 per generate, is never reused, and nothing about a design changes
# after it exists - so the browser may cache its preview files forever
# instead of re-downloading megabytes of STL on every gallery revisit.
# (The app shell stays no-store in main.py; only per-design artifacts are
# immutable.)
IMMUTABLE = {"Cache-Control": "public, max-age=31536000, immutable"}

# in-memory design store for this local, single-user app
_DESIGNS: dict[str, dict] = {}
_META: dict[str, dict] = {}
_PART_META: dict[str, list] = {}
_PART_REFUSALS: dict[str, list] = {}   # build refusals per design (sidecar)
PREVIEW_DIR = EXPORT_DIR / "previews"

# One lock per design id so the frontend can prefetch the four non-primary
# previews concurrently without two threads meshing the same solid (or, worse,
# one thread serving a half-written STL). _LOCKS itself is guarded by _REGISTRY.
_LOCKS: dict[str, threading.Lock] = {}
_REGISTRY = threading.Lock()


def _lock_for(design_id: str) -> threading.Lock:
    with _REGISTRY:
        lock = _LOCKS.get(design_id)
        if lock is None:
            lock = _LOCKS[design_id] = threading.Lock()
        return lock


PRESETS = [
    {
        # Skywalker X5 class: 1280 mm span, 717 mm root length
        "name": "Sport wing (X5 class)",
        "values": {"planform": "swept", "mission": "sport",
                   "cruise_speed_ms": 18.0, "payload_g": 0,
                   "box": {"length_mm": 800, "width_mm": 1300, "height_mm": 300},
                   "n_motors": 1, "vstab": "winglets",
                   "build_method": "3d_printed", "material": "lw_pla"},
    },
    {
        "name": "Long-range FPV cruiser",
        "values": {"planform": "bwb", "mission": "fpv_cruiser",
                   "cruise_speed_ms": 16.0, "payload_g": 250,
                   "box": {"length_mm": 900, "width_mm": 1500, "height_mm": 350},
                   "n_motors": 1, "vstab": "winglets",
                   "build_method": "3d_printed", "material": "lw_pla"},
    },
    {
        # PW-51 / Zagi-plank class park flyer
        "name": "Plank park flyer",
        "values": {"planform": "plank", "mission": "park_flyer",
                   "cruise_speed_ms": 10.0, "payload_g": 0,
                   "box": {"length_mm": 500, "width_mm": 1000, "height_mm": 250},
                   "n_motors": 1, "vstab": "center_fin",
                   "build_method": "foamboard", "material": "foam"},
    },
    {
        # NASA Prandtl-D / Horten class bell-spanload floater
        "name": "Bell-spanload floater",
        "values": {"planform": "bell", "mission": "thermal_floater",
                   "cruise_speed_ms": 10.0, "payload_g": 0,
                   "box": {"length_mm": 700, "width_mm": 1800, "height_mm": 250},
                   "n_motors": 1, "vstab": "none",
                   "build_method": "3d_printed", "material": "lw_pla"},
    },
]


def _airplane_types() -> list[dict]:
    """The type registry as the UI sees it (v2 additive keys; v3 axes; task-5
    `recommended_min_box_mm` [L, W, H], advisory only - never enforced)."""
    return [{"name": name, "label": t["label"],
             "description": t["description"],
             "axes": t.get("axes", {}),
             "recommended_min_box_mm": list(t.get("recommended_min_box_mm", []))}
            for name, t in AIRPLANE_TYPES.items()]


@router.get("/types")
def get_types():
    """Just the aircraft-type registry - what /api/options carries under
    `airplane_types`, without the rest of the option payload."""
    return {"airplane_types": _airplane_types()}


@router.get("/options")
def get_options():
    return {
        "planforms": [{"name": p.name, "label": p.label,
                       "description": p.description, "blurb": p.blurb,
                       "sweep_band": list(p.sweep_band),
                       "taper_band": list(p.taper_band),
                       "ar_band": list(p.ar_band),
                       "washout_band": list(p.washout_band),
                       "root_chord_frac_band": list(p.root_chord_frac_band),
                       "bell_spanload": p.bell_spanload,
                       "vstab_options": list(p.vstab_options)}
                      for p in PLANFORMS.values()],
        "missions": [{"name": m.name, "label": m.label, "blurb": m.blurb,
                      "static_margin": m.static_margin,
                      "wl_band": list(m.wl_band),
                      "stall_factor": m.stall_factor,
                      "default_cruise_ms": m.default_cruise_ms,
                      "preferred_planforms": list(m.preferred_planforms)}
                     for m in MISSIONS.values()],
        "vstabs": [{"name": v.name, "label": v.label, "count": v.count,
                    "description": v.description} for v in VSTABS.values()],
        "static_margin_band": list(SM_BAND),
        "airfoils": [{"name": a.name, "description": a.description,
                      "reflexed": a.reflexed, "thickness": a.thickness}
                     for a in LIBRARY.values() if a.reflexed],
        "fin_airfoils": [{"name": a.name, "description": a.description}
                         for a in LIBRARY.values() if not a.reflexed],
        "wing_airfoils": list(WING_AIRFOILS),
        "materials": [{"name": m.name, "label": m.label}
                      for m in MATERIALS.values()],
        "variants": variant_catalog(),
        "presets": PRESETS,
        # v2 (V2_PLAN.md): the aircraft-type dropdown at the first step of the
        # UI, and the conventional style roster its panel shows. Additive keys
        # only - nothing above changed.
        "airplane_types": _airplane_types(),
        "conv_styles": [{"name": s.name, "label": s.label,
                         "description": s.description, "blurb": s.blurb,
                         "ar_band": list(s.ar_band),
                         "wl_band": list(s.wl_band),
                         "wing_position": s.wing_position,
                         "dihedral_deg": s.dihedral_deg,
                         "vh": s.vh, "vv": s.vv}
                        for s in CONV_STYLES.values()],
        "conv_static_margin_band": list(CONV_SM_BAND),
    }


def _build_preview(design: dict) -> tuple[Path, dict]:
    """Build (or reuse) the preview STL for one design. Caller holds the lock.

    The build itself runs in a CAD worker process (see `backend.cadjobs`) so
    two different designs can mesh at the same time - OCC is not thread-safe,
    so in-process threads never could. Same builder, same fine tessellation
    (0.05 / 0.12: the preview is the product the user stares at), verified for
    full face coverage and published atomically, exactly as before.
    """
    path = PREVIEW_DIR / f"{design['id']}.stl"
    meta = _META.get(design["id"])
    if path.exists() and path.stat().st_size > 0 and meta is not None:
        return path, meta
    meta = cadjobs.run_job(cadjobs.job_preview_stl, design, str(PREVIEW_DIR))
    _META[design["id"]] = meta
    return path, meta


# What each named part IS, for the viewer's colour key. Kept next to the API
# rather than in the CAD so the browser and the STEP export agree on the story:
# white is structure, mid grey moves, dark grey comes off.
PART_ROLES: dict[str, str] = {
    "centre_body": "structure", "wing_left": "structure",
    "wing_right": "structure", "airframe": "structure",
    "fin": "structure", "fin_left": "structure", "fin_right": "structure",
    "winglet_left": "structure", "winglet_right": "structure",
    "elevon_left": "control", "elevon_right": "control",
    # v2 conventional parts: ailerons, elevator halves and the rudder are
    # servo-driven moving surfaces, same story as the elevons.
    "aileron_left": "control", "aileron_right": "control",
    "elevator_left": "control", "elevator_right": "control",
    "rudder": "control",
    "hatch_lid": "hatch", "cg_marker": "marker",
}
# Stealth-graphite part key (builder's decision, round 2 - the white/grey
# scheme read washed out): the airframe is dark graphite like the aircraft
# this app is imitating, the moving surfaces are a clearly lighter titanium
# grey so control surfaces read at a glance, the hatch is a darker inset
# panel, and red stays on the CG. Mirrored in exporters._PART_COLORS.
ROLE_COLORS = {"structure": "#3d4148", "control": "#7d848b",
               "hatch": "#25282c", "marker": "#d94b4b"}
ROLE_LABELS = {"structure": "Fixed structure",
               "control": "Moving control surface (servo)",
               "hatch": "Removable hatch", "marker": "CG marker"}


def _part_role(name: str) -> str:
    if name in PART_ROLES:
        return PART_ROLES[name]
    if ("elevon" in name or "aileron" in name or "elevator" in name
            or "rudder" in name):
        return "control"
    if "hatch" in name or "lid" in name:
        return "hatch"
    return "structure"


def _build_preview_parts(design: dict) -> list[dict]:
    """Build (or reuse) one preview STL per NAMED PART.

    The single-solid preview cannot show what comes apart: the elevons are
    fused into it, the hatch is only scribed, and the hinges are invisible.
    Serving the assembly part-by-part lets the viewer colour each one by what
    it does, which is the same story the STEP export tells.
    """
    did = design["id"]
    manifest = _PART_META.get(did)
    if manifest:
        return manifest
    built = cadjobs.run_job(cadjobs.job_parts_previews, design,
                            str(PREVIEW_DIR))
    try:
        # the honest-refusal sidecar the job wrote (cadjobs.build_refusals)
        _PART_REFUSALS[did] = json.loads(
            (PREVIEW_DIR / f"{did}__refusals.json").read_text(
                encoding="utf-8"))
    except Exception:
        _PART_REFUSALS[did] = []
    out: list[dict] = []
    for entry in built:
        name = entry["name"]
        role = _part_role(name)
        out.append({
            "name": name, "role": role,
            "label": name.replace("_", " "),
            "color": ROLE_COLORS[role], "role_label": ROLE_LABELS[role],
            "url": f"/api/preview/{did}/part/{name}.stl",
            "volume_mm3": entry["volume_mm3"],
        })
    _PART_META[did] = out
    return out


@router.get("/preview/{design_id}/parts.json")
def preview_parts(design_id: str):
    """Manifest of the assembly: one entry per part, with its colour and what
    the colour means."""
    design = _DESIGNS.get(design_id)
    if design is None:
        raise HTTPException(404, f"No design with id '{design_id}'.")
    try:
        with _lock_for(design_id + ":parts"):
            manifest = _build_preview_parts(design)
    except Exception as exc:
        raise HTTPException(500, f"Could not build the exploded preview: {exc}")
    return JSONResponse(
        {"parts": manifest,
         # features the builder refused, one sentence each - the viewer
         # shows them in the banner instead of a silently solid body
         "refusals": list(_PART_REFUSALS.get(design_id) or []),
         "legend": [{"role": r, "color": ROLE_COLORS[r],
                     "label": ROLE_LABELS[r]}
                    for r in ("structure", "control", "hatch")]},
        headers=IMMUTABLE)


@router.get("/preview/{design_id}/part/{part}.stl")
def preview_part(design_id: str, part: str):
    design = _DESIGNS.get(design_id)
    if design is None:
        raise HTTPException(404, f"No design with id '{design_id}'.")
    if not part.replace("_", "").isalnum():
        raise HTTPException(400, "bad part name")
    path = PREVIEW_DIR / f"{design_id}__{part}.stl"
    if not path.exists():
        with _lock_for(design_id + ":parts"):
            _build_preview_parts(design)
    if not path.exists():
        raise HTTPException(404, f"No part '{part}' on this design.")
    return FileResponse(path, media_type="model/stl", filename=path.name,
                        headers=IMMUTABLE)


# Monotonic generation stamp for the eager preview warm: a warm thread from
# an older generate exits before building, so a user who re-generates never
# has the pool's two workers grinding previews of designs no longer on
# screen ahead of the ones that are.
_WARM_GEN = 0


def _warm_previews(variants: list[dict], primary: dict) -> None:
    """Queue the builds the frontend is about to ask for, in the order it
    asks for them: primary parts (the viewer's preferred representation),
    primary one-piece (its fallback and the gallery mini), then the other
    minis in gallery order. Each thread takes the same per-id lock the GET
    endpoints take, so there is exactly one build per artifact no matter who
    gets there first. Failures stay silent here on purpose - the on-demand
    path rebuilds and reports the real error to the user.

    AEROFORGE_EAGER_PREVIEWS=0 disables the warm (tests set it: a TestClient
    generate must not start real CAD builds in the background)."""
    import os
    if os.environ.get("AEROFORGE_EAGER_PREVIEWS", "1") == "0":
        return
    global _WARM_GEN
    _WARM_GEN += 1
    gen = _WARM_GEN

    def _warm(did: str, parts: bool) -> None:
        if gen != _WARM_GEN:
            return                       # a newer generate superseded us
        design = _DESIGNS.get(did)
        if design is None:
            return
        try:
            if parts:
                with _lock_for(did + ":parts"):
                    if gen == _WARM_GEN:
                        _build_preview_parts(design)
            else:
                with _lock_for(did):
                    if gen == _WARM_GEN:
                        _build_preview(design)
        except Exception:
            pass

    def _exports_pending() -> bool:
        return any(j.get("status") == "building"
                   for j in _EXPORT_JOBS.values())

    def _warm_minis(dids: list[str]) -> None:
        # The gallery minis go ONE AT A TIME, and never while an export is
        # building. Submitting all four at once put them ahead of the user's
        # export in the pool's FIFO queue (measured 2026-08-28: a STEP click
        # sat "queued" behind two rounds of minis, hours on this machine) and
        # ran two of them beside the primary's builds. Serialized, they take
        # one worker and the other stays free for what the user asked for.
        for did in dids:
            while gen == _WARM_GEN and _exports_pending():
                time.sleep(2.0)
            if gen != _WARM_GEN:
                return
            _warm(did, False)

    for parts in (True, False):
        threading.Thread(target=_warm, args=(primary["id"], parts),
                         name=f"preview-warm-{primary['id'][:8]}",
                         daemon=True).start()
    minis = [v["id"] for v in variants if v is not primary]
    threading.Thread(target=_warm_minis, args=(minis,),
                     name="preview-warm-minis", daemon=True).start()


@router.post("/generate")
def generate(req: GenerateRequest):
    t0 = time.time()
    try:
        variants = generate_variants(req.to_optimizer_input())
        _TIMING.record(
            f"generate:{getattr(req, 'airplane_type', None) or 'flying_wing'}",
            time.time() - t0)
    except Exception as exc:  # never a bare 500 (spec 7)
        raise HTTPException(
            status_code=422,
            detail=f"The optimizer could not produce a design: {exc}. "
                   "Try a larger box or a lower cruise speed.") from exc

    for v in variants:
        _DESIGNS[v["id"]] = v["design"]
        v["preview_stl_url"] = f"/api/preview/{v['id']}.stl"

    # the character whose planform family the user asked for leads the gallery
    primary = next((v for v in variants if v.get("primary")), None)
    if primary is None:
        want = primary_key_for(req.planform)
        primary = next((v for v in variants if v["key"] == want),
                       next((v for v in variants if v["key"] == PRIMARY_KEY),
                            variants[0]))

    # NO CAD is built synchronously here: returning as soon as the numbers
    # exist puts the variant cards, trait bars and spec sheet on screen
    # immediately. But the frontend's very next act is deterministic - it
    # fetches the primary's part previews, then the gallery minis - so the
    # same builds are STARTED here in the worker pool (speed pass,
    # 2026-08-21) instead of waiting for those requests to arrive. The
    # per-id locks make the eventual GETs join the in-flight build rather
    # than duplicate it; the builders, tolerances and artifacts are the
    # on-demand path's own, so this moves wall-clock, not geometry. A newer
    # generate invalidates the queue: stale warms exit before building, so
    # the pool's two workers go to the design the user is actually viewing.
    _warm_previews(variants, primary)

    return {
        "variants": variants,
        "primary_id": primary["id"],
        # back-compat mirror of the primary variant
        "design": primary["design"],
        "preview_stl_url": primary["preview_stl_url"],
        "meta": None,
    }


@router.get("/learn")
def learn():
    """Static knowledge base of RC design articles for the Learn tab."""
    return {"articles": LEARN_ARTICLES}


@router.get("/preview/{design_id}.stl")
def preview_stl(design_id: str):
    design = _DESIGNS.get(design_id)
    if design is None:
        raise HTTPException(
            404, f"No design with id '{design_id}' - it was never generated, or "
                 "the server has restarted since. Generate a design first.")
    try:
        with _lock_for(design_id):
            path, _meta = _build_preview(design)
    except Exception as exc:
        raise HTTPException(
            500, f"Preview generation failed for this variant: {exc}. The "
                 "numeric design is still valid.") from exc
    return FileResponse(path, media_type="model/stl", filename=path.name,
                        headers=IMMUTABLE)


# One built file per (design, format). A STEP of the whole aircraft is a full
# CAD build - about 2.5 minutes - and nothing about a design changes after it is
# generated, so the second click must not pay for it again. Without this a user
# who clicks twice gets two concurrent builds fighting for the same cores, which
# is slower than one and looks exactly like a hung button.
_EXPORTS: dict[tuple[str, str], Path] = {}
# {status, stage, progress, started_at, eta_s, error?} per running job - the
# live part comes from the worker's progress file (backend.progress) joined
# with the rolling duration history in exports/timing.json.
_EXPORT_JOBS: dict[tuple[str, str], dict] = {}
PROGRESS_DIR = EXPORT_DIR / "progress"
_TIMING = _progress.TimingStore(EXPORT_DIR / "timing.json")

_EXPORT_MEDIA = {"step": "application/step", "stl_parts": "application/zip",
                 "stl": "model/stl"}


def _export_key(design_id: str, fmt: str) -> tuple[str, str]:
    return (design_id, fmt if fmt in _EXPORT_MEDIA else "stl")


def _timing_kind(design: dict, fmt: str) -> str:
    """History bucket for the ETA: one per (airplane type, format)."""
    return f"{design.get('airplane_type', 'flying_wing')}:{fmt}"


def _progress_path(key: tuple[str, str]) -> Path:
    return PROGRESS_DIR / f"{key[0]}_{key[1]}.json"


def _export_build(design: dict, key: tuple[str, str]) -> Path:
    """Build (or reuse) the export file for one (design, format). Blocking;
    takes the same lock as the exploded preview, because `build_design_parts`
    backs both STEP and the parts zip - one CAD build of a design at a time."""
    design_id, fmt = key
    lock = _lock_for(design_id + (":parts" if fmt != "stl" else ""))
    with lock:
        path = _EXPORTS.get(key)
        if path is None or not path.exists() or path.stat().st_size == 0:
            pfile = _progress_path(key)
            # A design whose parts/solid the previews already built is only
            # loaded and written (backend.cad.brepcache): a few seconds, and
            # not an ETA data point for a cold build - keep the history
            # honest by filing it under its own kind.
            cached = (brepcache.has_solid(design) if fmt == "stl"
                      else brepcache.has_parts(design))
            t0 = time.time()
            path = Path(cadjobs.run_job(cadjobs.job_export, design, fmt,
                                        str(pfile)))
            _EXPORTS[key] = path
            # learn from it: total duration + per-stage seconds feed the
            # next run's bar and ETA (median of the last few)
            t1 = time.time()
            marks = _progress.read_marks(pfile)
            _TIMING.record(_timing_kind(design, fmt)
                           + (":cached" if cached else ""), t1 - t0,
                           _progress.stage_durations(marks, t1))
            pfile.unlink(missing_ok=True)
    return path


def _export_state(key: tuple[str, str]) -> dict:
    """JSON-ready status for one export job."""
    path = _EXPORTS.get(key)
    if path is not None and path.exists() and path.stat().st_size > 0:
        return {"status": "ready",
                "url": f"/api/export/file/{key[0]}/{key[1]}",
                "filename": path.name,
                "bytes": path.stat().st_size}
    job = _EXPORT_JOBS.get(key)
    if job is not None:
        out = dict(job)
        if out.get("status") == "building":
            design = _DESIGNS.get(key[0]) or {}
            out.update(_progress.status_from_file(
                _progress_path(key), _timing_kind(design, key[1]), _TIMING,
                float(out.get("started_at") or time.time())))
        return out
    return {"status": "none"}


@router.post("/export")
def export(req: ExportRequest):
    """Synchronous export: returns the file in the response. Fine for scripts
    and tests; the BROWSER must not use it for a fresh design - the CAD build
    holds the request open for minutes, and Chrome kills a fetch whose response
    headers do not arrive in time, which reads as 'the button does nothing'.
    The frontend uses /export/start + /export/status + /export/file instead."""
    design = _DESIGNS.get(req.design_id)
    if design is None:
        raise HTTPException(404, "Unknown design id - generate a design first.")
    key = _export_key(req.design_id, req.format)
    try:
        path = _export_build(design, key)
    except Exception as exc:
        raise HTTPException(500, f"Export failed: {exc}") from exc
    return FileResponse(path, media_type=_EXPORT_MEDIA[key[1]],
                        filename=path.name)


@router.post("/export/start")
def export_start(req: ExportRequest):
    """Kick off (or join) a background export build and return at once.

    The response is always JSON and always fast: {"status": "ready", "url",
    "filename", "bytes"} if the file is already built, else
    {"status": "building"}. The build itself runs in a daemon thread and
    survives the browser tab; every later /export/status call joins the same
    job rather than starting another.
    """
    design = _DESIGNS.get(req.design_id)
    if design is None:
        raise HTTPException(404, "Unknown design id - generate a design first.")
    key = _export_key(req.design_id, req.format)
    with _REGISTRY:
        state = _export_state(key)
        if state["status"] in ("ready", "building"):
            return state
        started = time.time()
        _EXPORT_JOBS[key] = {"status": "building", "started_at": started}
        _progress_path(key).unlink(missing_ok=True)   # no stale marks

    def _run():
        try:
            _export_build(design, key)
            _EXPORT_JOBS[key] = {"status": "ready", "started_at": started}
        except Exception as exc:            # reported via /export/status
            _EXPORT_JOBS[key] = {
                "status": "failed", "started_at": started,
                "error": f"{type(exc).__name__}: {exc}"}
            _progress_path(key).unlink(missing_ok=True)

    threading.Thread(target=_run, daemon=True,
                     name=f"export-{key[0]}-{key[1]}").start()
    return _export_state(key)


@router.get("/timing")
def timing():
    """Rolling build-duration history (median of the last runs) per kind:
    "<airplane_type>:<format>" for exports, "generate:<airplane_type>" for
    the optimizer. The frontend draws its ETA from this before a job has
    reported any stage; `stages` lists the export build order."""
    return {"kinds": _TIMING.summary(), "stages": _progress.STAGES,
            "labels": _progress.STAGE_LABELS}


@router.get("/export/status/{design_id}/{fmt}")
def export_status(design_id: str, fmt: str):
    if design_id not in _DESIGNS:
        raise HTTPException(404, "Unknown design id - generate a design first.")
    return _export_state(_export_key(design_id, fmt))


@router.get("/export/file/{design_id}/{fmt}")
def export_file(design_id: str, fmt: str):
    key = _export_key(design_id, fmt)
    path = _EXPORTS.get(key)
    if path is None or not path.exists() or path.stat().st_size == 0:
        raise HTTPException(404, "Not built yet - POST /api/export/start first.")
    return FileResponse(path, media_type=_EXPORT_MEDIA[key[1]],
                        filename=path.name)
