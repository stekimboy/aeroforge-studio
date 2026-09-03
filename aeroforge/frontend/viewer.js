// three.js workspace: STL preview + CG / NP / MAC / motor markers + size box.
// Dark-studio product rendering: ACES tone mapping, RoomEnvironment IBL,
// soft contact shadows over a radial floor glow.
//
// Also exports MiniPreviewPool: many small auto-rotating previews driven by a
// SINGLE offscreen WebGL context, blitted into per-card 2D canvases.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import { mergeVertices, toCreasedNormals } from "three/addons/utils/BufferGeometryUtils.js";

/* Marker colours — kept in sync with the --m-* custom properties in style.css.
   Monochrome discipline: red is reserved for what is hardware-critical (the
   CG and the motors); everything else reads in greys and white. */
export const MARKERS = {
  cg: 0xff3b30,
  np: 0xf2f4f6,
  mac: 0x8d939a,
  motor: 0xff8078,
  box: 0x565c63,
};
const HEX = (n) => "#" + n.toString(16).padStart(6, "0");

/* ------------------------------------------------------------------ */
/* Shared, memoised STL -> BufferGeometry cache.                        */
/* The main viewer and every mini preview pull from here, so selecting  */
/* a variant whose card already rendered is instant and costs no extra  */
/* download or normal-recompute.                                        */
/* ------------------------------------------------------------------ */
const _loader = new STLLoader();
const _geoCache = new Map(); // url -> Promise<BufferGeometry>

async function _load(url) {
  const raw = await _loader.loadAsync(url);
  // STL triangles share no vertices, so computeVertexNormals() alone gives
  // flat faceted shading. Weld coincident vertices, then compute creased
  // normals: smooth across gentle curvature, crisp at real edges (~40 deg).
  let geo = mergeVertices(raw, 1e-4);
  geo = toCreasedNormals(geo, Math.PI * 0.22);
  geo.computeBoundingBox();
  geo.computeBoundingSphere();
  return geo;
}

export function loadGeometry(url) {
  const hit = _geoCache.get(url);
  if (hit) return hit;
  const p = _load(url);
  // A failed load must not poison the cache — drop it so a retry can work.
  p.catch(() => { if (_geoCache.get(url) === p) _geoCache.delete(url); });
  _geoCache.set(url, p);
  return p;
}

/* Release every cached geometry. Callers MUST first detach the geometries
   from any live scene (Viewer.clear(), MiniPreviewPool.reset()). */
export function clearGeometryCache() {
  for (const p of _geoCache.values()) {
    p.then((g) => g.dispose()).catch(() => {});
  }
  _geoCache.clear();
}

/* Shared surface look for the printed airframe: dark graphite with a satin
   composite sheen - the stealth-hardware read. Definition against the
   near-black stage comes from the rim light and the environment reflections,
   not from albedo. */
function airframeMaterial(color = 0x3d4148) {
  return new THREE.MeshPhysicalMaterial({
    color,
    metalness: 0.12,
    roughness: 0.52,
    clearcoat: 0.22,
    clearcoatRoughness: 0.42,
    envMapIntensity: 1.15,
    side: THREE.DoubleSide,
  });
}

/* Screen-space vertical gradient used as the scene background. */
function gradientTexture(top, bottom) {
  const c = document.createElement("canvas");
  c.width = 16; c.height = 512;
  const g = c.getContext("2d");
  const gr = g.createLinearGradient(0, 0, 0, 512);
  gr.addColorStop(0, top);
  gr.addColorStop(1, bottom);
  g.fillStyle = gr;
  g.fillRect(0, 0, 16, 512);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

/* Soft radial pool of light for the studio floor, so contact shadows have
   something slightly brighter than the background to darken. */
function floorTexture() {
  const S = 256;
  const c = document.createElement("canvas");
  c.width = c.height = S;
  const g = c.getContext("2d");
  const gr = g.createRadialGradient(S / 2, S / 2, 0, S / 2, S / 2, S / 2);
  gr.addColorStop(0.00, "rgba(88,94,101,0.42)");
  gr.addColorStop(0.35, "rgba(52,57,62,0.26)");
  gr.addColorStop(0.70, "rgba(26,29,32,0.10)");
  gr.addColorStop(1.00, "rgba(6,7,8,0.0)");
  g.fillStyle = gr;
  g.fillRect(0, 0, S, S);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

export class Viewer {
  constructor(canvas) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.0;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    this.scene = new THREE.Scene();
    this.scene.background = gradientTexture("#131518", "#050607");

    // Image-based lighting from the bundled RoomEnvironment (no external HDRs).
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    this.scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    pmrem.dispose();

    this.camera = new THREE.PerspectiveCamera(45, 1, 1, 30000);
    this.camera.position.set(900, -1200, 700);
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxPolarAngle = Math.PI * 0.52; // keep camera above the ground

    // Key light drives the contact shadow; the environment map provides fill.
    this.keyLight = new THREE.DirectionalLight(0xffffff, 2.1);
    this.keyLight.position.set(600, -800, 1000);
    this.keyLight.castShadow = true;
    this.keyLight.shadow.mapSize.set(2048, 2048);
    this.keyLight.shadow.bias = -0.0001;
    this.scene.add(this.keyLight);
    this.scene.add(this.keyLight.target);

    // Neutral rim from behind carves the dark airframe out of the darker
    // sky - with graphite hardware the silhouette IS the rim light.
    const rim = new THREE.DirectionalLight(0xd6dade, 1.25);
    rim.position.set(-700, 700, 420);
    this.scene.add(rim);

    // Very low near-neutral bounce so the underside never goes fully black.
    const bounce = new THREE.DirectionalLight(0xd8d4cd, 0.18);
    bounce.position.set(200, 400, -900);
    this.scene.add(bounce);

    // Shadow-catching ground + floor glow + subtle grid, rebuilt per design.
    this.groundGroup = new THREE.Group();
    this.scene.add(this.groundGroup);
    this._floorTex = floorTexture();
    this._buildGround(2200, 0);

    this.designGroup = new THREE.Group();
    this.scene.add(this.designGroup);

    this._resize();
    window.addEventListener("resize", () => this._resize());
    // The stage also resizes when the variant gallery opens/collapses.
    if (typeof ResizeObserver !== "undefined") {
      this._ro = new ResizeObserver(() => this._resize());
      this._ro.observe(canvas.parentElement || canvas);
    }
    this._animate();
  }

  _resize() {
    const w = this.canvas.clientWidth ||
      (this.canvas.parentElement && this.canvas.parentElement.clientWidth) || 1;
    const h = this.canvas.clientHeight ||
      (this.canvas.parentElement && this.canvas.parentElement.clientHeight) || 1;
    if (w < 1 || h < 1) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  _animate() {
    requestAnimationFrame(() => this._animate());
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  /* Ground plane (shadow catcher) + floor glow + fine grid at height z. */
  _buildGround(size, z) {
    this.groundGroup.clear();

    const glow = new THREE.Mesh(
      new THREE.PlaneGeometry(size * 3.2, size * 3.2),
      new THREE.MeshBasicMaterial({
        map: this._floorTex, transparent: true, depthWrite: false,
      }));
    glow.position.z = z;
    glow.renderOrder = -2;
    this.groundGroup.add(glow);

    const catcher = new THREE.Mesh(
      new THREE.PlaneGeometry(size * 4, size * 4),
      new THREE.ShadowMaterial({ opacity: 0.42 }));
    catcher.receiveShadow = true;
    catcher.position.z = z + size * 0.0002;
    catcher.renderOrder = -1;
    this.groundGroup.add(catcher);

    const grid = new THREE.GridHelper(size * 2, 48, 0x33383d, 0x191c1f);
    grid.rotation.x = Math.PI / 2; // grid in the XY (ground) plane, z-up
    grid.position.z = z + size * 0.0005;
    grid.material.transparent = true;
    grid.material.opacity = 0.5;
    grid.material.depthWrite = false;
    this.groundGroup.add(grid);
  }

  _roundedRect(g, x, y, w, h, r) {
    g.beginPath();
    g.moveTo(x + r, y);
    g.arcTo(x + w, y, x + w, y + h, r);
    g.arcTo(x + w, y + h, x, y + h, r);
    g.arcTo(x, y + h, x, y, r);
    g.arcTo(x, y, x + w, y, r);
    g.closePath();
  }

  /* Crisp, DPR-aware pill label rendered to a canvas sprite.
     heightMM sets the on-model height so labels scale with aircraft size. */
  _label(text, color, pos, heightMM) {
    const dpr = Math.min(window.devicePixelRatio || 1, 2) * 2; // supersampled
    const fontPx = 48, padX = 26, padY = 13;
    const font =
      `600 ${fontPx}px "Cascadia Mono", Consolas, "Bahnschrift", monospace`;
    const meas = document.createElement("canvas").getContext("2d");
    meas.font = font;
    const w = Math.ceil(meas.measureText(text).width) + padX * 2;
    const h = fontPx + padY * 2;

    const c = document.createElement("canvas");
    c.width = w * dpr; c.height = h * dpr;
    const g = c.getContext("2d");
    g.scale(dpr, dpr);
    this._roundedRect(g, 1.5, 1.5, w - 3, h - 3, 6);
    g.fillStyle = "rgba(10,11,12,0.88)";  // dark tag reads on a dark scene
    g.fill();
    g.strokeStyle = color;
    g.globalAlpha = 0.65;
    g.lineWidth = 2.5;
    g.stroke();
    g.globalAlpha = 1;
    g.font = font;
    g.fillStyle = color;
    g.textAlign = "center";
    g.textBaseline = "middle";
    g.fillText(text, w / 2, h / 2 + 2);

    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.anisotropy = this.renderer.capabilities.getMaxAnisotropy();
    const sp = new THREE.Sprite(new THREE.SpriteMaterial({
      map: tex, depthTest: false, transparent: true,
    }));
    sp.scale.set(heightMM * (w / h), heightMM, 1);
    sp.position.copy(pos);
    sp.renderOrder = 11;
    return sp;
  }

  _marker(color, pos, r) {
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(r, 32, 20),
      new THREE.MeshBasicMaterial({ color, depthTest: false }));
    m.position.copy(pos);
    m.renderOrder = 10;
    return m;
  }

  /* Drop the current aircraft and free everything this viewer created for it
     (marker meshes, MAC lines, box edges, label sprites + their canvas
     textures). The airframe geometry is owned by the shared cache, so it is
     deliberately left alone — clearGeometryCache() releases that.
     Call before clearGeometryCache() so no live mesh points at a dead buffer. */
  clear() {
    this.designGroup.traverse((o) => {
      if (o.geometry && !(o.userData && o.userData.sharedGeometry)) {
        o.geometry.dispose();
      }
      const mats = Array.isArray(o.material) ? o.material
        : (o.material ? [o.material] : []);
      for (const m of mats) {
        if (m.map) m.map.dispose();
        m.dispose();
      }
    });
    this.designGroup.clear();
  }

  /* Load the aircraft PART BY PART, each in the colour of what it does:
     white is structure, mid grey moves, dark grey comes off. The single-solid preview
     cannot show any of that - the elevons are fused into it, the hatch is only
     scribed and the hinges are invisible - so when a parts manifest is
     available this is what gets drawn. Falls back to the one-piece STL. */
  async showDesignParts(manifest, design, boxMM) {
    const token = (this._loadToken = (this._loadToken || 0) + 1);
    const parts = (manifest && manifest.parts) || [];
    if (!parts.length) throw new Error("no parts in the manifest");
    const loaded = await Promise.all(parts.map(async (p) => {
      try { return { p, geo: await loadGeometry(p.url) }; }
      catch { return null; }
    }));
    if (token !== this._loadToken) return;
    this.clear();
    let any = null;
    for (const item of loaded) {
      if (!item) continue;
      const mesh = new THREE.Mesh(
        item.geo, airframeMaterial(new THREE.Color(item.p.color).getHex()));
      mesh.userData.sharedGeometry = true;
      mesh.userData.partName = item.p.name;
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.designGroup.add(mesh);
      any = mesh;
    }
    if (!any) throw new Error("no part geometry loaded");
    this._decorate(design, boxMM);
  }

  async showDesign(stlUrl, design, boxMM) {
    // Guard against out-of-order loads when the user clicks variants quickly.
    const token = (this._loadToken = (this._loadToken || 0) + 1);
    const geo = await loadGeometry(stlUrl);
    if (token !== this._loadToken) return; // superseded by a newer selection
    this.clear();

    const mesh = new THREE.Mesh(geo, airframeMaterial());
    mesh.userData.sharedGeometry = true;   // owned by the module-level cache
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    this.designGroup.add(mesh);
    this._decorate(design, boxMM);
  }

  /* Markers, MAC lines, motor dots, size box, ground and camera framing.
     Shared by the one-piece and the part-by-part loaders. */
  _decorate(design, boxMM) {
    // Model-relative scale so overlays fit both tiny and huge aircraft.
    const bb = new THREE.Box3().setFromObject(this.designGroup);
    const center = bb.getCenter(new THREE.Vector3());
    const size = bb.getSize(new THREE.Vector3()).length();
    const markerR = Math.max(size * 0.007, 2.5);   // mm
    const labelH = Math.max(size * 0.042, 14);     // mm
    const labelDz = Math.max(size * 0.085, 30);    // mm above/below datum
    const lineLift = Math.max(size * 0.003, 2);    // mm above the wing skin

    const s = (design && design.stability) || {};
    if (Number.isFinite(s.x_cg_m) && Number.isFinite(s.x_np_m)) {
      const xcg = s.x_cg_m * 1000, xnp = s.x_np_m * 1000;

      // CG (signal red) and NP (white) markers with labels
      this.designGroup.add(
        this._marker(MARKERS.cg, new THREE.Vector3(xcg, 0, 0), markerR));
      this.designGroup.add(this._label(
        `CG ${Number(s.cg_pct_mac ?? 0).toFixed(0)}%MAC · ${xcg.toFixed(0)}mm`,
        HEX(MARKERS.cg), new THREE.Vector3(xcg, 0, labelDz), labelH));
      this.designGroup.add(
        this._marker(MARKERS.np, new THREE.Vector3(xnp, 0, 0), markerR));
      this.designGroup.add(this._label("NP", HEX(MARKERS.np),
        new THREE.Vector3(xnp, 0, -labelDz), labelH));
    }

    // MAC chord lines (grey) at +/- y_MAC
    if (Number.isFinite(s.mac_m) && Number.isFinite(s.x_le_mac_m)) {
      const mac = s.mac_m * 1000, xle = s.x_le_mac_m * 1000;
      const ymac = Number(s.y_mac_m ?? 0) * 1000;
      const macMat = new THREE.LineBasicMaterial({ color: MARKERS.mac });
      for (const sign of [1, -1]) {
        const pts = [
          new THREE.Vector3(xle, sign * ymac, lineLift),
          new THREE.Vector3(xle + mac, sign * ymac, lineLift),
        ];
        this.designGroup.add(new THREE.Line(
          new THREE.BufferGeometry().setFromPoints(pts), macMat));
      }
      this.designGroup.add(this._label("MAC", HEX(MARKERS.mac),
        new THREE.Vector3(xle + mac / 2, ymac, labelDz * 0.75), labelH * 0.9));
    }

    // Motor positions (red) — design.geometry.motors is in METRES.
    const motors = (design && design.geometry &&
      Array.isArray(design.geometry.motors)) ? design.geometry.motors : [];
    motors.forEach((mo, i) => {
      if (!mo) return;
      const p = new THREE.Vector3(
        Number(mo.x || 0) * 1000, Number(mo.y || 0) * 1000,
        Number(mo.z || 0) * 1000);
      if (!Number.isFinite(p.x) || !Number.isFinite(p.y) ||
          !Number.isFinite(p.z)) return;
      this.designGroup.add(this._marker(MARKERS.motor, p, markerR * 0.9));
      if (motors.length <= 4) {
        this.designGroup.add(this._label(
          `M${i + 1}`, HEX(MARKERS.motor),
          new THREE.Vector3(p.x, p.y, p.z + labelDz * 0.55), labelH * 0.72));
      }
    });

    // User's size box (wireframe), aligned to the aircraft exactly the way the
    // optimizer's box constraints measure it: length from the nose datum
    // (x = 0) aft, span centred on the centreline, and HEIGHT FROM THE BELLY
    // UP — height_total is measured from the lowest point of the airframe, not
    // from the z = 0 centreline. Centring the box on z = 0 (as it used to) put
    // half of it underground and made any tall fin appear to burst out through
    // the top of a box the aircraft genuinely fits inside.
    if (boxMM && Number.isFinite(boxMM.length_mm)) {
      const L = boxMM.length_mm, W = boxMM.width_mm, H = boxMM.height_mm;
      const boxEdges = new THREE.LineSegments(
        new THREE.EdgesGeometry(new THREE.BoxGeometry(L, W, H)),
        new THREE.LineBasicMaterial({
          color: MARKERS.box, transparent: true, opacity: 0.5,
        }));
      boxEdges.position.set(L / 2, 0, bb.min.z + H / 2);
      this.designGroup.add(boxEdges);
    }

    // ground + grid under the model
    this._buildGround(size, bb.min.z - size * 0.0015);

    // fit the shadow camera tightly around the model for crisp contact shadows
    const kl = this.keyLight;
    kl.position.set(center.x + size * 0.45, center.y - size * 0.55, center.z + size * 0.9);
    kl.target.position.copy(center);
    kl.target.updateMatrixWorld();
    const sc = kl.shadow.camera;
    sc.left = sc.bottom = -size * 0.8;
    sc.right = sc.top = size * 0.8;
    sc.near = size * 0.05;
    sc.far = size * 3.5;
    sc.updateProjectionMatrix();
    kl.shadow.normalBias = size * 0.0015;

    // frame the model
    this.controls.target.copy(center);
    const dir = new THREE.Vector3(0.55, -1.0, 0.55).normalize();
    this.camera.position.copy(center.clone().add(dir.multiplyScalar(size * 1.15)));
    this.camera.near = Math.max(size / 200, 0.1);
    this.camera.far = size * 30;
    this.camera.updateProjectionMatrix();
    this.controls.minDistance = size * 0.25;
    this.controls.maxDistance = size * 5;
    this.controls.update();
  }
}

/* ------------------------------------------------------------------ */
/* MiniPreviewPool                                                      */
/*                                                                      */
/* One offscreen WebGL context renders every card in turn and blits the */
/* result into each card's own 2D canvas — so N cards cost 1 GPU        */
/* context, not N. Geometry loads run through a small bounded queue     */
/* (two in flight — the server builds lazily in a two-worker CAD pool,  */
/* so a pair of requests actually builds in parallel; more would just   */
/* queue on the server's per-design locks).                             */
/* ------------------------------------------------------------------ */
export class MiniPreviewPool {
  constructor({ fps = 30, maxRatio = 1.5, maxInFlight = 2 } = {}) {
    this.entries = new Map();   // id -> entry
    this.queue = [];
    this.busy = 0;
    this.maxInFlight = Math.max(1, maxInFlight);
    this.onState = null;        // (id, state) => void
    this.maxRatio = maxRatio;
    this._minDelta = 1 / Math.max(fps, 1);
    this._last = 0;
    this._w = 0; this._h = 0;
    this._gl = null;
  }

  /* The GL context is built on first use, so a page that never generates a
     design never pays for it. */
  _ensureGL() {
    if (this._gl) return this._gl;
    const canvas = document.createElement("canvas");
    const renderer = new THREE.WebGLRenderer({
      canvas, antialias: true, alpha: true, powerPreference: "low-power",
    });
    renderer.setPixelRatio(1); // we size the drawing buffer ourselves
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.0;

    const scene = new THREE.Scene();
    const pmrem = new THREE.PMREMGenerator(renderer);
    scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    pmrem.dispose();

    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(600, -800, 900);
    scene.add(key);
    const rim = new THREE.DirectionalLight(0xd6dade, 1.05);
    rim.position.set(-600, 600, 300);
    scene.add(rim);

    const pivot = new THREE.Group();
    scene.add(pivot);
    const mesh = new THREE.Mesh(new THREE.BufferGeometry(), airframeMaterial());
    pivot.add(mesh);

    const camera = new THREE.PerspectiveCamera(34, 1.6, 1, 200000);
    camera.up.set(0, 0, 1);

    this._gl = { canvas, renderer, scene, camera, pivot, mesh };
    this._loop();
    return this._gl;
  }

  /* Register a card. `canvas` is the card's own 2D canvas. */
  attach(id, canvas, url) {
    if (!id || !canvas) return;
    this._ensureGL();
    this.entries.set(id, {
      id, url, canvas, ctx: canvas.getContext("2d"),
      geo: null, center: new THREE.Vector3(), radius: 1,
      yaw: Math.PI * 0.12, state: "idle", visible: false,
    });
  }

  setVisible(id, visible) {
    const e = this.entries.get(id);
    if (e) e.visible = !!visible;
  }

  /* Ask for a card's geometry. Idempotent and safe to call repeatedly. */
  request(id) {
    const e = this.entries.get(id);
    if (!e || e.state !== "idle" || !e.url) return;
    e.state = "queued";
    this.queue.push(id);
    this._pump();
  }

  /* Prefetch everything, active card first, one at a time. */
  requestAll(firstId) {
    if (firstId) this.request(firstId);
    for (const id of this.entries.keys()) this.request(id);
  }

  async _pump() {
    if (this.busy >= this.maxInFlight) return;
    const id = this.queue.shift();
    if (id === undefined) return;
    const e = this.entries.get(id);
    if (!e) { this._pump(); return; }
    this.busy++;
    this._setState(e, "loading");
    this._pump();               // let a second load start alongside this one
    try {
      const geo = await loadGeometry(e.url);
      if (this.entries.get(id) === e) {   // not reset while we were loading
        e.geo = geo;
        const bb = geo.boundingBox;
        if (bb) {
          bb.getCenter(e.center);
          e.radius = Math.max(bb.getSize(new THREE.Vector3()).length() / 2, 1e-3);
        }
        this._setState(e, "ready");
      }
    } catch {
      if (this.entries.get(id) === e) this._setState(e, "error");
    } finally {
      this.busy--;
      this._pump();
    }
  }

  _setState(e, state) {
    e.state = state;
    if (this.onState) {
      try { this.onState(e.id, state); } catch { /* UI callback must not stop the queue */ }
    }
  }

  /* Drop all cards. Detaches the shared mesh geometry so the caller can
     safely dispose the geometry cache afterwards. */
  reset() {
    this.entries.clear();
    this.queue.length = 0;
    if (this._gl) this._gl.mesh.geometry = new THREE.BufferGeometry();
  }

  _sizeFor(entry) {
    const w = entry.canvas.clientWidth, h = entry.canvas.clientHeight;
    if (w < 2 || h < 2) return null;
    const r = Math.min(window.devicePixelRatio || 1, this.maxRatio);
    return { w, h, bw: Math.round(w * r), bh: Math.round(h * r) };
  }

  _loop() {
    requestAnimationFrame((t) => this._loop(t));
    const now = performance.now() / 1000;
    const raw = now - this._last;
    if (raw < this._minDelta) return;
    this._last = now;
    const dt = Math.min(raw, 0.1);   // clamp the first frame / tab-switch gaps
    if (document.hidden || !this._gl) return;

    const { renderer, scene, camera, pivot, mesh, canvas } = this._gl;
    for (const e of this.entries.values()) {
      if (!e.visible || e.state !== "ready" || !e.geo) continue;
      const s = this._sizeFor(e);
      if (!s) continue;

      if (e.canvas.width !== s.bw || e.canvas.height !== s.bh) {
        e.canvas.width = s.bw; e.canvas.height = s.bh;
      }
      if (this._w !== s.bw || this._h !== s.bh) {
        renderer.setSize(s.bw, s.bh, false);
        camera.aspect = s.bw / s.bh;
        camera.updateProjectionMatrix();
        this._w = s.bw; this._h = s.bh;
      }

      e.yaw += dt * 0.35;                       // slow auto-rotate
      mesh.geometry = e.geo;
      mesh.position.copy(e.center).multiplyScalar(-1);
      pivot.rotation.z = e.yaw;

      const dist = e.radius * 2.65;
      camera.position.set(dist * 0.28, -dist * 0.90, dist * 0.34);
      camera.lookAt(0, 0, 0);
      camera.near = Math.max(e.radius / 50, 0.05);
      camera.far = e.radius * 40;
      camera.updateProjectionMatrix();

      renderer.render(scene, camera);
      // Blit while the drawing buffer is still valid (same task as render).
      e.ctx.clearRect(0, 0, s.bw, s.bh);
      e.ctx.drawImage(canvas, 0, 0, s.bw, s.bh);
    }
  }
}
