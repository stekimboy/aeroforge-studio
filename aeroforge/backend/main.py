"""AeroForge FastAPI app: local API + static frontend at http://127.0.0.1:8000."""
from __future__ import annotations

import contextlib
from pathlib import Path

from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.gzip import GZipMiddleware

from . import cadjobs
from .api import router

FRONTEND_DIR = Path(__file__).resolve().parent.parent / "frontend"


@contextlib.asynccontextmanager
async def _lifespan(app: FastAPI):
    # Spawn and warm the CAD worker pool in the background: each worker pays
    # its cadquery/neuralfoil import bill now, while the user is still looking
    # at an empty sidebar, instead of in front of their first preview.
    cadjobs.warm_pool_async()
    yield


app = FastAPI(title="AeroForge", version="1.0.0",
              description="Physics-based RC airplane design & optimization studio",
              lifespan=_lifespan)
app.include_router(router)


@app.get("/health")
def health():
    return {"status": "ok"}


# The frontend is edited between runs and served from disk, so browsers must
# never reuse a cached copy: a stale app.js against a newer API silently breaks
# the whole page (empty dropdowns, dead Generate button).
NO_CACHE = {"Cache-Control": "no-store, must-revalidate", "Pragma": "no-cache",
            "Expires": "0"}


@app.get("/")
def index():
    return FileResponse(FRONTEND_DIR / "index.html", headers=NO_CACHE)


class NoCacheStatic(StaticFiles):
    """StaticFiles that forbids client-side caching of the app shell."""

    def file_response(self, *args, **kwargs):
        resp = super().file_response(*args, **kwargs)
        resp.headers.update(NO_CACHE)
        return resp


@app.exception_handler(Exception)
async def unhandled(request: Request, exc: Exception):
    # never a bare 500 without a readable message (spec 7)
    return JSONResponse(
        status_code=500,
        content={"detail": f"Unexpected server error: {type(exc).__name__}: {exc}"},
    )


app.mount("/", NoCacheStatic(directory=FRONTEND_DIR), name="frontend")


class SelectiveGZip:
    """gzip for JSON and the app shell, NEVER for the model artifacts.

    A /api/generate response is ~0.5 MB of JSON and compresses ~10x, which is
    worth having even on loopback. The STL/STEP/zip artifacts are megabytes of
    binary the browser either caches (previews are immutable) or writes
    straight to disk (downloads); starlette's gzip (compresslevel 9) would
    stall each of those responses for CPU that saves nothing on 127.0.0.1 -
    so requests for them bypass the compressor entirely. The bytes served are
    identical either way; only the transfer encoding differs.
    """

    _SKIP_SUFFIXES = (".stl", ".step", ".zip")
    _SKIP_PREFIXES = ("/api/export/file/",)

    def __init__(self, app):
        self.plain = app
        self.gzipped = GZipMiddleware(app, minimum_size=1024)

    async def __call__(self, scope, receive, send):
        if scope["type"] == "http":
            path = scope.get("path", "")
            if path.endswith(self._SKIP_SUFFIXES) or \
                    path.startswith(self._SKIP_PREFIXES):
                await self.plain(scope, receive, send)
                return
        await self.gzipped(scope, receive, send)


app.add_middleware(SelectiveGZip)
