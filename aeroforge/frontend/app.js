// AeroForge UI logic: sidebar inputs -> /api/generate -> variant gallery,
// 3D viewer, spec sheet, design notes and exports.
//
// v2 (V2_PLAN.md): an aircraft-type dropdown sits at the FIRST step. With
// "Flying wing" selected the sidebar below it is the v1 panel, exactly as it
// always was; "Conventional" and "Any" swap in their own panels beside it,
// never inside it.
import { Viewer, MiniPreviewPool, clearGeometryCache } from "/viewer.js";

const $ = (id) => document.getElementById(id);

/* The 3D viewer needs WebGL, which can fail (no GPU, hardware acceleration
   off, driver blocklist, three.js CDN blocked). That must NOT take the whole
   UI down with it: the sidebar, presets and Generate have to keep working, so
   construction is guarded and every later use is null-checked. */
let viewer = null;
let minis = null;
let viewerError = "";
try {
  viewer = new Viewer($("viewport"));
  minis = new MiniPreviewPool();
} catch (e) {
  viewerError = (e && e.message) || String(e);
  console.error("3D viewer unavailable:", e);
}

/* Surface any unexpected script error instead of dying silently. */
function showError(msg) {
  const b = $("banner");
  if (!b) return;
  b.textContent = msg;
  b.className = "warn";
}
/* "ResizeObserver loop completed with undelivered notifications" is a benign
   browser notice, not a fault: it fires whenever observations arrive faster
   than they can be delivered, which the 3D stage does on every layout change
   (opening the data panel, collapsing the gallery). Nothing is broken and
   nothing is dropped, so it must not raise an alarm banner. */
const BENIGN = /^ResizeObserver loop/;
window.addEventListener("error", (e) => {
  const msg = (e.error && e.error.message) || e.message || "";
  if (BENIGN.test(msg)) return;
  showError("Page error: " + msg);
});
window.addEventListener("unhandledrejection", (e) => {
  const r = e.reason;
  const msg = (r && r.message) || String(r);
  if (BENIGN.test(msg)) return;
  showError("Page error: " + msg);
});

/* active-design state */
let variants = [];        // [{id, key, name, tagline, traits[], design, preview_stl_url}]
let activeId = null;
let currentDesignId = null;
let lastBox = null;       // box used for the last generate (for the overlay)

const esc = (t) => String(t ?? "").replace(/[&<>"']/g,
  (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const num = (v) => (v == null || !Number.isFinite(Number(v)) ? null : Number(v));
const fmt = (v, d = 1) => (num(v) == null ? "—" : num(v).toFixed(d));
/* scale a possibly-missing SI value (null stays null) */
const mul = (v, k) => (num(v) == null ? null : num(v) * k);
/* value + unit, or a lone em-dash when the field is absent */
const u = (v, unit, d = 1) => (num(v) == null ? "—" : `${num(v).toFixed(d)} ${unit}`);
/* same, but with the unit tight against the number (degrees, percent, ×) */
const ut = (v, unit, d = 1) => (num(v) == null ? "—" : `${num(v).toFixed(d)}${unit}`);
/* "a → b unit", em-dash unless BOTH ends are real numbers */
const span2 = (a, b, unit, d = 0) =>
  (num(a) == null || num(b) == null ? "—"
    : `${num(a).toFixed(d)} → ${num(b).toFixed(d)} ${unit}`);
/* sub-line flavour of the above: drops out entirely when incomplete */
const span2n = (a, b, unit, d = 0) =>
  (num(a) == null || num(b) == null ? null : span2(a, b, unit, d));
/* sub-line fragment: null (i.e. omitted) when the number is missing */
const frag = (v, before, after = "", d = 1) =>
  (num(v) == null ? null : `${before}${num(v).toFixed(d)}${after}`);
/* join the sub-line fragments that actually have content */
const sub = (...parts) => parts.filter((p) => p != null && p !== "").join(" · ");

/* ====================================================================== */
/* Domain model (SPEC_FLYING_WING.md §3). The backend is the source of     */
/* truth and its /api/options payload wins on every field it supplies;     */
/* these built-ins are the labels, one-line descriptions and vstab         */
/* filtering the UI falls back to so the studio is usable even when the    */
/* server sends a bare list.                                               */
/* ====================================================================== */
const ANY_PLANFORM = "any";

const PLANFORM_META = {
  [ANY_PLANFORM]: {
    label: "Any (let the variants choose)",
    description: "Leave the family open — the gallery spans several wing " +
      "shapes so you can compare them side by side.",
    vstab_options: ["winglets", "twin_fin", "center_fin", "none"],
  },
  swept: {
    label: "Swept sport wing",
    description: "The classic X5 / AR Wing shape: 18–32° of leading-edge " +
      "sweep, taper 0.35–0.60, AR 3.5–5.2, a deep root that is the fuselage.",
    vstab_options: ["winglets", "twin_fin", "center_fin"],
  },
  bwb: {
    label: "Blended wing body",
    description: "Ritewing-Drak class long-range cruiser: 16–28° sweep, a " +
      "deep blended centre body carrying the payload bay, big canted winglets.",
    vstab_options: ["winglets", "twin_fin", "center_fin"],
  },
  plank: {
    label: "Plank",
    description: "Almost unswept (0–8°), nearly rectangular, AR 5–8 — a " +
      "strongly reflexed section does all the trimming. It looks like a door.",
    vstab_options: ["center_fin", "winglets"],
  },
  bell: {
    label: "Bell-distribution (Horten)",
    description: "Horten / NASA Prandtl-D spanload: 20–34° sweep, 8–13° of " +
      "washout, and no vertical surfaces at all — the tips make their own yaw.",
    vstab_options: ["none"],
    bell_spanload: true,
  },
};
const PLANFORM_ORDER = [ANY_PLANFORM, "swept", "bwb", "plank", "bell"];

const MISSION_META = {
  sport: {
    label: "Sport",
    description: "Crisp and quick: higher wing loading, tighter static margin.",
    default_cruise_ms: 18,
    preferred_planforms: ["swept", "bwb", "plank", "bell"],
  },
  fpv_cruiser: {
    label: "FPV cruiser",
    description: "Steady camera platform — mid loading, long legs, payload bay.",
    default_cruise_ms: 16,
    preferred_planforms: ["bwb", "swept", "bell", "plank"],
  },
  thermal_floater: {
    label: "Thermal floater",
    description: "Light loading and low sink for slope soaring and thermals.",
    default_cruise_ms: 10,
    preferred_planforms: ["bell", "plank", "swept", "bwb"],
  },
  park_flyer: {
    label: "Park flyer",
    description: "Small, slow and forgiving for tight fields.",
    default_cruise_ms: 9,
    preferred_planforms: ["plank", "swept", "bell", "bwb"],
  },
};
const MISSION_ORDER = ["sport", "fpv_cruiser", "thermal_floater", "park_flyer"];

const VSTAB_META = {
  winglets: {
    label: "2 × wingtip winglets",
    description: "Canted 8–20° outboard at the tips — the usual sport-wing fit.",
  },
  twin_fin: {
    label: "2 × inboard fins",
    description: "Vertical fins at 50–65% of the semi-span, Skywalker X5 style.",
  },
  center_fin: {
    label: "1 × centre fin",
    description: "A single vertical stabilizer on the centre-body spine. " +
      "The only workable choice on an unswept plank, and a good one on any " +
      "swept wing: it keeps the tips clean and survives a cartwheel.",
  },
  none: {
    label: "None",
    description: "None. A bell spanload loads the tips so lightly that they " +
      "produce induced THRUST, which gives proverse yaw — the wing yaws into " +
      "the turn by itself. Fins are unnecessary here, and adding them cancels " +
      "the effect the configuration exists for.",
  },
};
/* ====================================================================== */
/* v2 airplane types (V2_PLAN.md). Server /api/options wins on every field  */
/* it supplies; these built-ins keep the dropdown usable against an older   */
/* or partial server. "any" is a REQUEST mode, not a type — it is appended  */
/* client-side because the registry only lists concrete types.             */
/* ====================================================================== */
const AIRPLANE_TYPE_META = {
  flying_wing: {
    label: "Flying wing",
    description: "Tailless blended wing — one continuous lifting surface " +
      "tip to tip, trimmed by a reflexed section and washout.",
  },
  conventional: {
    label: "Conventional",
    description: "Wing + fuselage + horizontal and vertical stabilizers " +
      "with a tractor motor on the nose — the classic trainer/sport layout.",
  },
};
const ANY_TYPE_OPTION = {
  name: "any",
  label: "Any (optimize across types)",
  description: "Not committed to a configuration — every type is solved " +
    "from your bare parameters and the best five span at least three of " +
    "them. Expect the solve to take about a minute.",
};

/* v3 configuration axes (RESEARCH_TYPES_V3.md §10.4 via /api/options
   `axes`). One row per axis; the select renders only when the current type
   offers a real choice. Labels are the UI's own; the values are the
   schema's literals. n_motors rides the existing Motors select instead. */
const AXIS_DEFS = [
  ["motor_layout", {
    tractor: "Tractor (nose)", pusher: "Pusher" }],
  ["tail_type", {
    conventional: "Conventional tail", t_tail: "T-tail", v_tail: "V-tail" }],
  ["wing_position", {
    high: "High wing", mid: "Mid wing", low: "Low wing" }],
];

const CONV_STYLE_META = {
  trainer: { label: "High-wing trainer",
    blurb: "Docile, self-recovering, lands at a walk - the first aeroplane." },
  sport: { label: "Low-wing sport",
    blurb: "Crisper than the trainer without giving up the manners." },
  aerobatic: { label: "Aerobat",
    blurb: "Neutral, symmetric and eager - it goes exactly where pointed." },
  floater: { label: "High-wing floater",
    blurb: "Slow, floaty and forgiving - thermals and calm evenings." },
  speed: { label: "Sport speedster",
    blurb: "Wind penetration and pace - fly the approach with power on." },
};
const CONV_STYLE_ORDER = ["trainer", "sport", "aerobatic", "floater", "speed"];

const vstabLabel = (v) =>
  (VSTAB_META[v] && VSTAB_META[v].label) || (v ? String(v) : "—");
const vstabNote = (v) => (VSTAB_META[v] && VSTAB_META[v].description) || "";
const planformLabel = (k) =>
  (PLANFORM_META[k] && PLANFORM_META[k].label) || (k ? String(k) : "—");
const missionLabel = (k) =>
  (MISSION_META[k] && MISSION_META[k].label) || (k ? String(k) : "—");

/* Fallback presets — used only when the server sends none it can vouch for.
   All five are flying wings; there is nothing else to build. */
const FALLBACK_PRESETS = [
  { name: "FPV wing — 900 mm",
    values: { planform: "swept", mission: "fpv_cruiser", cruise_speed_ms: 18,
      payload_g: 150, box: { length_mm: 520, width_mm: 950, height_mm: 260 },
      n_motors: 1, build_method: "3d_printed", material: "lw_pla" } },
  { name: "Sport wing — 1200 mm",
    values: { planform: "swept", mission: "sport", cruise_speed_ms: 22,
      payload_g: 0, box: { length_mm: 700, width_mm: 1250, height_mm: 300 },
      n_motors: 1, build_method: "3d_printed", material: "lw_pla" } },
  { name: "Long-range BWB cruiser",
    values: { planform: "bwb", mission: "fpv_cruiser", cruise_speed_ms: 17,
      payload_g: 250, box: { length_mm: 820, width_mm: 1600, height_mm: 330 },
      n_motors: 1, build_method: "3d_printed", material: "lw_pla" } },
  { name: "Bell-distribution floater",
    values: { planform: "bell", mission: "thermal_floater", cruise_speed_ms: 10,
      payload_g: 0, box: { length_mm: 620, width_mm: 1800, height_mm: 260 },
      n_motors: 1, build_method: "3d_printed", material: "lw_pla" } },
  { name: "Plank park flyer",
    values: { planform: "plank", mission: "park_flyer", cruise_speed_ms: 9,
      payload_g: 0, box: { length_mm: 480, width_mm: 900, height_mm: 220 },
      n_motors: 1, build_method: "3d_printed", material: "lw_pla" } },
];

/* ---------------- unit-linked cruise speed (m/s <-> mph) ---------------- */
const MPH = 2.236936;
let cruiseTouched = false;   // stop the mission default from stomping on typing
$("cruise_ms").addEventListener("input", () => {
  cruiseTouched = true;
  $("cruise_mph").value = (parseFloat($("cruise_ms").value || 0) * MPH).toFixed(1);
});
$("cruise_mph").addEventListener("input", () => {
  cruiseTouched = true;
  $("cruise_ms").value = (parseFloat($("cruise_mph").value || 0) / MPH).toFixed(1);
});
function setCruise(ms) {
  const v = num(ms);
  if (v == null) return;
  $("cruise_ms").value = v;
  $("cruise_mph").value = (v * MPH).toFixed(1);
}

/* ---------------- auto/set toggles ---------------- */
for (const [modeId, inputId] of [
  ["stall_mode", "stall_ms"], ["ar_mode", "ar"], ["sm_mode", "sm"],
]) {
  const mode = $(modeId), input = $(inputId);
  if (!mode || !input) continue;
  mode.addEventListener("change", () => {
    input.disabled = mode.value === "auto";
  });
}

/* ---------------- motor-mount overrides ---------------- */
/* One toggle gates all three, because they describe a single object: the bolt
   pattern on the back of a motor you either already own or have not chosen
   yet. Whatever is showing here is what the CAD drills. */
const MOUNT_FIELDS = ["mount_screws", "mount_screw_d", "mount_bolt_circle"];
function syncMountMode() {
  const custom = $("mount_mode") && $("mount_mode").value === "set";
  for (const id of MOUNT_FIELDS) if ($(id)) $(id).disabled = !custom;
  const note = $("mount_note");
  if (!note) return;
  const r = parseFloat($("mount_bolt_circle") ? $("mount_bolt_circle").value : "");
  note.textContent = custom && Number.isFinite(r)
    // a square pattern's bolt circle runs through its corners: s = R x sqrt(2)
    ? `That is a ${(r * Math.SQRT2).toFixed(1)} × ${(r * Math.SQRT2).toFixed(1)} mm `
      + "square pattern — the size quoted on a motor's spec sheet."
    : "Measure your motor's bolt spacing across the flats and divide by √2, or "
      + "read the pattern off its spec sheet. These numbers are what gets "
      + "drilled in the exported model.";
}
if ($("mount_mode")) $("mount_mode").addEventListener("change", syncMountMode);
if ($("mount_bolt_circle"))
  $("mount_bolt_circle").addEventListener("input", syncMountMode);
syncMountMode();

/* ---------------- options + presets from the backend ---------------- */
let planforms = [];   // [{name,label,description,vstab_options,bell_spanload}]
let missions = [];    // [{name,label,description,default_cruise_ms,preferred_planforms}]

/* Merge one server record with the built-in copy for that key. */
function mergePlanform(p) {
  const name = String((p && p.name) || "");
  const meta = PLANFORM_META[name] || {};
  const opts = (p && Array.isArray(p.vstab_options) && p.vstab_options.length)
    ? p.vstab_options.slice()
    : (meta.vstab_options || ["none"]).slice();
  return {
    name,
    label: (p && p.label) || meta.label || name,
    description: (p && (p.description || p.blurb)) || meta.description || "",
    vstab_options: opts,
    bell_spanload: !!((p && p.bell_spanload) || meta.bell_spanload),
  };
}
function mergeMission(m) {
  const name = String((m && m.name) || "");
  const meta = MISSION_META[name] || {};
  return {
    name,
    label: (m && m.label) || meta.label || name,
    description: (m && (m.description || m.blurb)) || meta.description || "",
    default_cruise_ms: num(m && m.default_cruise_ms) ?? meta.default_cruise_ms ?? null,
    preferred_planforms: Array.isArray(m && m.preferred_planforms)
      ? m.preferred_planforms.slice() : (meta.preferred_planforms || []).slice(),
  };
}

const planformByName = (n) =>
  planforms.find((p) => p.name === n) || planforms[0] || mergePlanform({ name: n });
const missionByName = (n) =>
  missions.find((m) => m.name === n) || missions[0] || mergeMission({ name: n });

/* ---------------- v2 aircraft-type switching ---------------- */
let airplaneTypes = [];   // [{name,label,description}] + the "any" mode
let convStyles = [];      // [{name,label,description,blurb}]

/* The flying-wing copy is captured from the DOM at load so switching back
   restores it EXACTLY — the v1 panel must never drift. */
const FW_BRAND_SUB = $("brand_sub") ? $("brand_sub").textContent : "";
const FW_BOX_NOTE = $("box_note") ? $("box_note").textContent : "";
const FW_HINT_HTML = $("hint") ? $("hint").innerHTML : "";
const ALT_COPY = {
  conventional: {
    sub: "Physics-based RC aircraft design studio",
    box: "The assembled aircraft must fit inside this box — span across " +
      "the width, fuselage along the length.",
    hint: "Pick a style and a mission on the right, then press " +
      "<b>Generate</b> to forge five aeroplanes.",
  },
  any: {
    sub: "Physics-based RC aircraft design studio",
    box: "The assembled aircraft must fit inside this box, whichever " +
      "configuration wins.",
    hint: "Set your mission targets on the right, then press " +
      "<b>Generate</b> to forge five designs across the types.",
  },
  // v3: the six new types share one copy set — each type's own description
  // already rides the dropdown note directly above.
  generic: {
    sub: "Physics-based RC aircraft design studio",
    box: "The assembled aircraft must fit inside this box — span across " +
      "the width, fuselage along the length.",
    hint: "Pick a mission on the right, then press <b>Generate</b> to " +
      "forge five designs.",
  },
};

function currentAirplaneType() {
  const el = $("airplane_type");
  const v = el ? el.value : "flying_wing";
  // any registry type (or the client-side "any" mode) is its own panel key;
  // anything unknown falls back to the v1 flying-wing panel.
  return airplaneTypes.some((t) => t.name === v) ? v : "flying_wing";
}

const axesFor = (type) => {
  const rec = airplaneTypes.find((t) => t.name === type);
  return (rec && rec.axes) || null;
};

/* v3: render one select per configuration axis that offers a REAL choice for
   this type. "Type default" (value "") sends nothing, so the type's own
   researched default applies server-side and a bare request stays
   byte-identical to before. Rebuilt on every type switch, which also resets
   stale choices from the previous type. */
function renderConfigRow(type) {
  const block = $("cfg_block");
  if (!block) return;
  const axes = axesFor(type);
  let shown = 0;
  for (const [axis, labels] of AXIS_DEFS) {
    const row = $("cfg_row_" + axis);
    const sel = $("cfg_" + axis);
    if (!row || !sel) continue;
    const opts = (axes && Array.isArray(axes[axis])) ? axes[axis] : [];
    const usable = opts.length >= 2;   // 0/1 legal values = nothing to choose
    row.hidden = !usable;
    if (!usable) { sel.innerHTML = ""; continue; }
    shown += 1;
    const name = (v) => labels[v] || String(v);
    sel.innerHTML =
      `<option value="">Type default (${esc(name(opts[0]))})</option>` +
      opts.map((v) =>
        `<option value="${esc(v)}">${esc(name(v))}</option>`).join("");
  }
  block.hidden = shown === 0;
}

/* v3: the shared Motors select follows the type's n_motors band. The v2
   panels (conventional / any) are FROZEN: they keep the shipped 1–4 roster
   and note verbatim; only the six new types get band-fitted options —
   including 0 ("None — pure sailplane") for the glider. */
const V2_MOTOR_TYPES = new Set(["flying_wing", "conventional", "any"]);
const V2_MOTORS_NOTE =
  "A conventional model flies a single tractor motor on the nose. Motor " +
  "positions are shown in the 3D view; AeroForge does not size props, Kv " +
  "or batteries.";
function syncAltMotors(type) {
  const sel = $("alt_n_motors");
  if (!sel) return;
  const note = $("alt_motors_note");
  if (V2_MOTOR_TYPES.has(type)) {
    sel.innerHTML =
      "<option>1</option><option>2</option><option>3</option><option>4</option>";
    sel.value = "1";
    sel.disabled = false;
    if (note) note.textContent = V2_MOTORS_NOTE;
    return;
  }
  const axes = axesFor(type) || {};
  const band = Array.isArray(axes.n_motors) ? axes.n_motors : [1, 1];
  const lo = Math.max(0, Math.round(num(band[0]) ?? 1));
  const hi = Math.max(lo, Math.round(num(band[1]) ?? lo));
  const opts = [];
  for (let n = lo; n <= hi; n++) {
    opts.push(`<option value="${n}">` +
      (n === 0 ? "None — pure sailplane" : String(n)) + "</option>");
  }
  sel.innerHTML = opts.join("");
  sel.value = String(Math.min(Math.max(1, lo), hi));  // researched default: 1
  sel.disabled = hi === lo;
  if (note) {
    const rec = airplaneTypes.find((t) => t.name === type);
    const layout = (Array.isArray(axes.motor_layout) && axes.motor_layout[0])
      ? ` ${axes.motor_layout[0]}` : "";
    note.textContent =
      (hi === lo
        ? `A ${((rec && rec.label) || type).toLowerCase()} flies ` +
          `${lo === 1 ? "one" : lo}${layout} motor${lo === 1 ? "" : "s"}. `
        : `This type flies ${lo}–${hi} motors. `) +
      "Motor positions are shown in the 3D view; AeroForge does not size " +
      "props, Kv or batteries.";
  }
}

function syncAirplaneType() {
  const t = currentAirplaneType();
  const sidebar = $("sidebar");
  if (sidebar) sidebar.dataset.atype = t;
  const rec = airplaneTypes.find((x) => x.name === t);
  const note = $("airplane_type_note");
  if (note) note.textContent = (rec && rec.description) || "";
  const copy = ALT_COPY[t] ||           // undefined for flying_wing ->
    (t !== "flying_wing" ? ALT_COPY.generic : undefined);  // restore v1 copy
  if ($("brand_sub")) $("brand_sub").textContent = copy ? copy.sub : FW_BRAND_SUB;
  if ($("box_note")) $("box_note").textContent = copy ? copy.box : FW_BOX_NOTE;
  if ($("hint")) $("hint").innerHTML = copy ? copy.hint : FW_HINT_HTML;
  renderConfigRow(t);
  syncAltMotors(t);
  syncBoxReco();
}

function syncConvStyle() {
  const sel = $("conv_style");
  if (!sel || !$("conv_style_note")) return;
  const s = convStyles.find((x) => x.name === sel.value);
  $("conv_style_note").textContent = s
    ? (s.description || s.blurb || "")
    : "Every run generates all five characters; the mission's own pick " +
      "leads the gallery and the cards let you compare.";
}

function syncAltMission() {
  const m = missionByName($("alt_mission").value);
  $("alt_mission_note").textContent = m.description || "";
  if (!cruiseTouched && m.default_cruise_ms != null) setCruise(m.default_cruise_ms);
}

/* Planform options, ordered so the selected mission's preferred families come
   first (§3.2: preferred_planforms orders the UI, it never restricts it).
   "Any" is pinned to the top and the current choice is preserved. */
function renderPlanformOptions() {
  const sel = $("planform");
  const keep = sel.value;
  const pref = missionByName($("mission").value).preferred_planforms;
  const rank = (p) => {
    if (p.name === ANY_PLANFORM) return -1;
    const i = pref.indexOf(p.name);
    return i >= 0 ? i : pref.length + PLANFORM_ORDER.indexOf(p.name);
  };
  const ordered = planforms.slice().sort((a, b) => rank(a) - rank(b));
  sel.innerHTML = ordered.map((p) =>
    `<option value="${esc(p.name)}" title="${esc(p.description)}">${esc(p.label)}</option>`
  ).join("");
  if (ordered.some((p) => p.name === keep)) sel.value = keep;
}

function syncPlanform() {
  const p = planformByName($("planform").value);
  $("planform_note").textContent = p.description || "";

  // vertical surfaces are filtered per planform, exactly as vstab used to be
  // filtered per config: a bell spanload offers "none" and nothing else.
  const sel = $("vstab");
  const keep = sel.value;
  const opts = p.vstab_options.length ? p.vstab_options : ["none"];
  sel.innerHTML =
    `<option value="auto">Auto (${esc(vstabLabel(opts[0]))})</option>` +
    opts.map((v) =>
      `<option value="${esc(v)}" title="${esc(vstabNote(v))}">${esc(vstabLabel(v))}</option>`
    ).join("");
  sel.value = opts.includes(keep) ? keep : "auto";
  sel.disabled = opts.length < 2;

  const forced = opts.length === 1;
  const note = forced ? vstabNote(opts[0])
    : (p.name === ANY_PLANFORM
      ? "Auto lets each variant fit what its own planform wants."
      : `Available on the ${p.label.toLowerCase()}: ` +
        opts.map(vstabLabel).join(", ") + ".");
  $("vstab_note").textContent = note;
  // the tooltip carries the same reasoning for hover users
  $("vstab_row").title = forced
    ? `${p.label}: ${vstabLabel(opts[0])} only. ${vstabNote(opts[0])}`
    : "Directional-stability surfaces available for this planform";
}

function syncMission() {
  const m = missionByName($("mission").value);
  $("mission_note").textContent = m.description || "";
  if (!cruiseTouched && m.default_cruise_ms != null) setCruise(m.default_cruise_ms);
  renderPlanformOptions();
  syncPlanform();
}

function applyPreset(v) {
  if (!v) return;
  const setIf = (id, val) => {
    const el = $(id);
    if (!el || val == null) return;
    if (el.tagName === "SELECT" &&
        ![...el.options].some((o) => o.value === String(val))) return;
    el.value = val;
  };
  // `config` / `style` are the v1 names; accept them so a half-migrated
  // server preset still lands somewhere sensible.
  setIf("mission", v.mission ?? v.style);
  syncMission();
  setIf("planform", v.planform ?? v.config);
  syncPlanform();
  if (v.vstab) setIf("vstab", v.vstab);
  if (v.cruise_speed_ms != null) { setCruise(v.cruise_speed_ms); cruiseTouched = true; }
  setIf("payload_g", v.payload_g ?? 0);
  if (v.box) {
    setIf("box_l", v.box.length_mm);
    setIf("box_w", v.box.width_mm);
    setIf("box_h", v.box.height_mm);
  }
  setIf("n_motors", v.n_motors ?? 1);
  setIf("build_method", v.build_method ?? "3d_printed");
  setIf("material", v.material ?? "lw_pla");
}

async function loadOptions() {
  let opt = {};
  let failure = null;
  try {
    const res = await fetch("/api/options");
    if (!res.ok) throw new Error("HTTP " + res.status);
    opt = await res.json();
  } catch (e) {
    // The server is mid-flight or gone. Still build the studio from the
    // built-in domain model so the user sees a working sidebar, then report.
    opt = {};
    failure = e;
  }
  {
    // Build the selectors from whatever arrived; the built-in domain model
    // fills every gap so the studio still works against a partial server.
    const rawP = Array.isArray(opt.planforms) && opt.planforms.length
      ? opt.planforms.map(mergePlanform)
      : PLANFORM_ORDER.filter((k) => k !== ANY_PLANFORM)
          .map((k) => mergePlanform({ name: k }));
    planforms = [mergePlanform({ name: ANY_PLANFORM })].concat(
      rawP.filter((p) => p.name && p.name !== ANY_PLANFORM));

    missions = (Array.isArray(opt.missions) && opt.missions.length
      ? opt.missions.map(mergeMission)
      : MISSION_ORDER.map((k) => mergeMission({ name: k })));

    $("mission").innerHTML = missions.map((m) =>
      `<option value="${esc(m.name)}" title="${esc(m.description)}">${esc(m.label)}</option>`
    ).join("");
    $("mission").value = missions.some((m) => m.name === "sport")
      ? "sport" : missions[0].name;
    renderPlanformOptions();
    $("planform").value = ANY_PLANFORM;

    $("planform").addEventListener("change", syncPlanform);
    $("mission").addEventListener("change", syncMission);
    syncMission();

    $("material").innerHTML = (opt.materials || [])
      .map((m) => `<option value="${esc(m.name)}">${esc(m.label)}</option>`).join("");

    // A tailless wing needs a positive-Cm0 section, so only reflexed airfoils
    // are offerable. Fall back to the whole library if nothing is flagged.
    const airfoils = Array.isArray(opt.airfoils) ? opt.airfoils : [];
    const reflexed = airfoils.filter((a) => a && a.reflexed);
    $("airfoil").innerHTML = '<option value="auto">Auto</option>' +
      (reflexed.length ? reflexed : airfoils).map((a) =>
        `<option value="${esc(a.name)}" title="${esc(a.description)}">${esc(a.name)}</option>`
      ).join("");

    const served = Array.isArray(opt.presets) ? opt.presets : [];
    const usable = served.filter((p) => p && p.values &&
      (p.values.planform || p.values.mission));
    const presets = usable.length ? usable : FALLBACK_PRESETS;
    $("preset").innerHTML = '<option value="">— choose a preset —</option>' +
      presets.map((p, i) => `<option value="${i}">${esc(p.name)}</option>`).join("");
    $("preset").addEventListener("change", () => {
      const p = presets[$("preset").value];
      if (p) applyPreset(p.values);
    });

    // v2: aircraft-type dropdown + the conventional panel's own selectors.
    // Server records win field-by-field; built-ins fill every gap. "any" is
    // appended client-side — it is a request mode, not a registry entry.
    airplaneTypes = (Array.isArray(opt.airplane_types) && opt.airplane_types.length
      ? opt.airplane_types
      : Object.entries(AIRPLANE_TYPE_META).map(([name, m]) => ({ name, ...m })))
      .map((t) => {
        const name = String((t && t.name) || "");
        const meta = AIRPLANE_TYPE_META[name] || {};
        return { name, label: (t && t.label) || meta.label || name,
                 description: (t && t.description) || meta.description || "",
                 // v3: server-declared configuration axes (§10.4 matrix);
                 // null against an older server -> no config row renders.
                 axes: (t && t.axes) || null,
                 // task 5: advisory [L, W, H] mm floor; [] = none known
                 recommended_min_box_mm: Array.isArray(t && t.recommended_min_box_mm)
                   ? t.recommended_min_box_mm.map(Number) : [] };
      }).filter((t) => t.name && t.name !== "any");
    airplaneTypes.push({ ...ANY_TYPE_OPTION });
    if ($("airplane_type")) {
      $("airplane_type").innerHTML = airplaneTypes.map((t) =>
        `<option value="${esc(t.name)}" title="${esc(t.description)}">${esc(t.label)}</option>`
      ).join("");
      $("airplane_type").value = "flying_wing";
      $("airplane_type").addEventListener("change", syncAirplaneType);
    }

    convStyles = (Array.isArray(opt.conv_styles) && opt.conv_styles.length
      ? opt.conv_styles
      : CONV_STYLE_ORDER.map((k) => ({ name: k })))
      .map((s) => {
        const name = String((s && s.name) || "");
        const meta = CONV_STYLE_META[name] || {};
        return { name, label: (s && s.label) || meta.label || name,
                 description: (s && s.description) || "",
                 blurb: (s && s.blurb) || meta.blurb || "" };
      }).filter((s) => s.name);
    if ($("conv_style")) {
      $("conv_style").innerHTML =
        '<option value="">Mission default (all five generated)</option>' +
        convStyles.map((s) =>
          `<option value="${esc(s.name)}" title="${esc(s.blurb || s.description)}">${esc(s.label)}</option>`
        ).join("");
      $("conv_style").addEventListener("change", syncConvStyle);
      syncConvStyle();
    }

    if ($("alt_mission")) {
      $("alt_mission").innerHTML = missions.map((m) =>
        `<option value="${esc(m.name)}" title="${esc(m.description)}">${esc(m.label)}</option>`
      ).join("");
      $("alt_mission").value = missions.some((m) => m.name === "sport")
        ? "sport" : missions[0].name;
      $("alt_mission").addEventListener("change", syncAltMission);
      // note only at load — the shared cruise field already carries the v1
      // mission default, and stomping it here would be a v1 behaviour change
      $("alt_mission_note").textContent =
        missionByName($("alt_mission").value).description || "";
    }
    syncAirplaneType();
  }
  if (failure) throw failure;
}
loadOptions()
  .then(() => {
    // the sidebar is alive; warn only if 3D itself is unavailable
    if (viewerError) {
      showError("3D preview unavailable (" + viewerError +
                "). Design generation and exports still work.");
    }
  })
  .catch((e) => {
    console.error("options load failed", e);
    showError("Could not load design options from the server: " +
              ((e && e.message) || e) + " — is the AeroForge server still running?");
  });

/* ---------------- request assembly ---------------- */
function buildRequest() {
  const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
  return {
    airplane_type: "flying_wing",       // the default; explicit for clarity
    planform: $("planform").value,      // "any" = let the variants choose
    mission: $("mission").value,
    cruise_speed_ms: parseFloat($("cruise_ms").value),
    stall_speed_target_ms:
      $("stall_mode").value === "target" ? parseFloat($("stall_ms").value) : null,
    payload_g: parseFloat($("payload_g").value || 0),
    box: {
      length_mm: parseFloat($("box_l").value),
      width_mm: parseFloat($("box_w").value),
      height_mm: parseFloat($("box_h").value),
    },
    allow_folding: $("allow_folding").checked,
    vstab: $("vstab") ? $("vstab").value : "auto",
    n_motors: parseInt($("n_motors").value, 10),
    build_method: $("build_method").value,
    material: $("build_method").value === "foamboard"
      ? "foam" : ($("material").value || "lw_pla"),
    airfoil_override: $("airfoil").value === "auto" ? null : $("airfoil").value,
    aspect_ratio_target:
      $("ar_mode").value === "set" ? parseFloat($("ar").value) : null,
    // tailless band is 3-15% of MAC, not the 4-20% of a tailed model
    static_margin_override:
      $("sm_mode").value === "set"
        ? clamp(parseFloat($("sm").value), 3, 15) / 100 : null,
    // motor mount: null on every field = size the pattern from all-up mass
    ...(($("mount_mode") && $("mount_mode").value === "set")
      ? {
        mount_screws: parseInt($("mount_screws").value, 10),
        mount_screw_d_mm: clamp(parseFloat($("mount_screw_d").value), 1.5, 8),
        mount_bolt_circle_mm:
          clamp(parseFloat($("mount_bolt_circle").value), 4, 40),
      }
      : { mount_screws: null, mount_screw_d_mm: null,
          mount_bolt_circle_mm: null }),
  };
}

/* v2: the conventional / any panels send only the fields the backend accepts
   (schemas.GenerateRequest) — planform/vstab/airfoil/AR/SM are flying-wing
   knobs and are omitted so their server defaults apply. There is no style
   field in the request: every conventional run generates all five characters
   and the mission picks the lead, so the Style selector steers the GALLERY
   (which variant is shown first), not the solver.
   v3: every non-flying-wing type sends through here; a visible configuration
   axis with a non-default choice adds motor_layout / tail_type /
   wing_position — "Type default" adds NOTHING, keeping a bare request
   byte-identical to v2's. */
function buildRequestAlt(type) {
  const req = {
    airplane_type: type,   // "conventional" | "any" | one of the v3 types
    mission: $("alt_mission").value,
    cruise_speed_ms: parseFloat($("cruise_ms").value),
    stall_speed_target_ms:
      $("stall_mode").value === "target" ? parseFloat($("stall_ms").value) : null,
    payload_g: parseFloat($("payload_g").value || 0),
    box: {
      length_mm: parseFloat($("box_l").value),
      width_mm: parseFloat($("box_w").value),
      height_mm: parseFloat($("box_h").value),
    },
    allow_folding: $("allow_folding").checked,
    n_motors: parseInt($("alt_n_motors").value, 10),
    build_method: $("build_method").value,
    material: $("build_method").value === "foamboard"
      ? "foam" : ($("material").value || "lw_pla"),
  };
  for (const [axis] of AXIS_DEFS) {
    const row = $("cfg_row_" + axis);
    const sel = $("cfg_" + axis);
    if (row && sel && !row.hidden && sel.value) req[axis] = sel.value;
  }
  return req;
}

/* ================= design data panel (specs + notes tabs) ================= */
/* Both readouts are docked in their own column rather than floating over the
   3D stage, so the aircraft is never covered. */
function setPane(name) {
  for (const tab of document.querySelectorAll(".dp-tab")) {
    const on = tab.dataset.pane === name && !tab.disabled;
    tab.classList.toggle("is-active", on);
    tab.setAttribute("aria-selected", on ? "true" : "false");
  }
  $("specs").classList.toggle("is-active", name === "specs");
  $("guidance_body").classList.toggle("is-active", name === "notes");
  const body = document.querySelector(".dp-body");
  if (body) body.scrollTop = 0;
}
document.querySelector(".dp-tabs").addEventListener("click", (e) => {
  const tab = e.target.closest(".dp-tab");
  if (tab && !tab.disabled) setPane(tab.dataset.pane);
});
$("dp_collapse").addEventListener("click", () =>
  $("datapanel").classList.add("collapsed"));
$("dp_expand").addEventListener("click", () =>
  $("datapanel").classList.remove("collapsed"));

/* ---------------- spec readout ---------------- */
/* Every key here is read exactly as SPEC_FLYING_WING.md §4 names it. There are
   no tail-volume rows: a flying wing has no tail. */
function showSpecs(d) {
  if (!d) { $("datapanel").classList.add("hidden"); return; }
  const g = d.geometry || {};
  const b = g.body || {};
  const vs = g.vstab || {};
  const el = g.elevons || {};
  const mm = g.motor_mount || {};
  const a = d.aero || {};
  const s = d.stability || {};
  const ps = d.power_system || {};
  const mass = d.mass || {};

  const motors = Array.isArray(g.motors) ? g.motors : [];
  const nMotors = ps.n_motors ?? (motors.length || null);
  const motorType = motors.length && motors[0] && motors[0].type
    ? String(motors[0].type) : "";

  // Fin area as a fraction of wing area — the number that says at a glance
  // whether this is a real fin or a sail (§8: 2–9% of wing area).
  const finPct = (num(vs.area_total_m2) != null && num(g.area_m2) > 0)
    ? (vs.area_total_m2 / g.area_m2) * 100 : null;
  const noFins = String(vs.type || "") === "none" ||
    (num(vs.area_total_m2) === 0 && !vs.type);

  const vstabRows = noFins
    ? [["Vertical surfaces", esc(vs.label || "None"),
        "bell spanload — the tips give proverse yaw, so fins are neither " +
        "needed nor wanted"]]
    : [
      ["Vertical surfaces", esc(vs.label || vstabLabel(vs.type)),
        sub(frag(vs.count, "", " ×", 0),
            frag(mul(vs.area_total_m2, 1e4), "", " cm²", 0),
            frag(finPct, "", "% of wing area", 1))],
      ["Fin height", u(mul(vs.height_m, 1000), "mm", 0),
        sub(span2n(mul(vs.root_chord_m, 1000), mul(vs.tip_chord_m, 1000), "mm chord"),
            frag(vs.sweep_le_deg, "", "° sweep", 0),
            frag(vs.cant_deg, "", "° cant", 0),
            frag(mul(vs.y_frac, 100), "at ", "% semi-span", 0))],
    ];

  /* v2: conventional designs carry geometry.fuselage + geometry.tail blocks
     (V2_PLAN.md schema). When the fuselage exists the sheet gets its own
     tailed groups; a flying-wing (or delta — it rides the tailless dict
     shape) never enters this branch and renders through the untouched v1
     groups below, pixel-identical to before.
     v3: the type-specific blocks (canard / wing2 / booms / spar /
     polyhedral / fins) each add a group ONLY when the geometry carries
     them, so a conventional dict still renders exactly the v2 groups. Key
     names are read from the physics dicts as generated, never guessed. */
  const fus = g.fuselage || null;
  const tail = g.tail || null;
  const bay = (fus && fus.bay) || null;
  const ail = g.ailerons || {};
  const can = g.canard || null;      // canard foreplane block
  const w2 = g.wing2 || null;        // tandem front wing / biplane upper wing
  const boomsG = g.booms || null;    // twin-boom tail carriers
  const aftFins = g.fins || null;    // canard/tandem fins (no tail block)
  const spar = g.spar || null;       // glider carbon-spar callout
  const poly = g.polyhedral || null; // glider polyhedral break
  const ballastG = mul(g.nose_ballast_provision_kg, 1000);
  const evSrc = can || w2 || {};     // where the elevator lives sans tail
  const convGroups = fus ? [
    ["Aircraft", [
      ["Style", esc(d.planform_label || d.planform || "—"),
        missionLabel(d.mission) + " mission"],
      ["Span", u(mul(g.span_m, 1000), "mm", 0), "tip to tip"],
      ["Wing area", u(mul(g.area_m2, 1e4), "cm²", 0), `AR ${fmt(g.aspect_ratio, 2)}`],
      ["Root / tip chord",
        span2(mul(g.root_chord_m, 1000), mul(g.tip_chord_m, 1000), "mm"),
        `taper ${fmt(g.taper, 2)}`],
      ["Dihedral", ut(g.dihedral_deg, "°"),
        sub(frag(g.sweep_le_deg, "LE sweep ", "°"),
            g.wing_position ? `${esc(g.wing_position)} wing` : null)],
      ["Washout", ut(g.washout_deg, "°"),
        sub(frag(g.root_incidence_deg, "root incidence ", "°"))],
      ["Airfoil", esc(g.airfoil || "—"),
        sub("plain section — the tail does the trimming",
            g.fin_airfoil ? `tail ${esc(g.fin_airfoil)}` : null)],
    ]],
    ["Fuselage", [
      ["Length", u(mul(fus.length_m, 1000), "mm", 0),
        sub(frag((num(fus.length_m) != null && num(g.span_m) > 0)
          ? (fus.length_m / g.span_m) * 100 : null, "", "% of the span", 0))],
      ["Cross-section",
        (num(fus.width_m) == null || num(fus.height_m) == null) ? "—"
          : `${fmt(mul(fus.width_m, 1000), 0)} × ` +
            `${fmt(mul(fus.height_m, 1000), 0)} mm`,
        "width × height"],
      ["Wing LE station", u(mul(fus.x_wing_le_m, 1000), "mm", 0),
        "from the nose datum"],
      ["Equipment bay",
        (bay == null || num(bay.bay_length_m) == null ||
         num(bay.bay_width_m) == null) ? "—"
          : `${fmt(mul(bay.bay_length_m, 1000), 0)} × ` +
            `${fmt(mul(bay.bay_width_m, 1000), 0)} mm`,
        bay == null ? "" :
          sub(frag(mul(bay.bay_start_m, 1000), "starts ", " mm from the nose", 0),
              frag(mul(bay.bay_depth_m, 1000), "", " mm deep", 0))],
      ["Overall size",
        (num(g.length_total_m) == null || num(g.height_total_m) == null) ? "—"
          : `${fmt(mul(g.length_total_m, 1000), 0)} × ` +
            `${fmt(mul(g.height_total_m, 1000), 0)} mm`,
        "length × height from the nose datum"],
    ]],
    // v3: canard foreplane (geometry.canard) — area/volume/incidence plus
    // the stall-first loading margin the physics records (LEN-CAN Eq. 2-7).
    ...(can ? [["Canard (foreplane)", [
      ["Canard area", u(mul(can.area_m2, 1e4), "cm²", 0),
        sub(frag(mul(can.sc_s, 100), "", "% of the wing", 0),
            frag(can.aspect_ratio, "AR ", "", 1))],
      ["Canard span", u(mul(can.span_m, 1000), "mm", 0),
        span2n(mul(can.c_root_m, 1000), mul(can.c_tip_m, 1000), "mm chord")],
      ["Volume V_C", fmt(can.V_C, 2),
        "lifting-canard band 0.5–0.9 — far above a control canard's"],
      ["Incidence", ut(can.incidence_deg, "°"),
        sub("rigged to load the foreplane harder than the wing",
            frag(s.decalage_deg, "decalage ", "°"))],
      ["Stall-first margin", ut(s.stall_first_margin_deg, "°"),
        sub(frag(s.cl_ratio_fr, "CLf/CLr ", "", 2),
            s.canard_stalls_first === false
              ? "CHECK: the canard does NOT stall first"
              : "the canard lets go before the wing ever can")],
    ]]] : []),
    // v3: second lifting surface (geometry.wing2) — tandem front wing
    // (gap/stagger in MAC, lift share) or biplane upper wing (gap/stagger
    // in chords, area ratio); the keys tell the two apart.
    ...(w2 ? [[w2.role === "upper" ? "Upper wing" : "Front wing", [
      ["Span", u(mul(w2.span_m, 1000), "mm", 0),
        sub(frag(mul(w2.area_m2, 1e4), "", " cm²", 0),
            span2n(mul(w2.c_root_m, 1000), mul(w2.c_tip_m, 1000), "mm chord"))],
      ["Gap", u(mul(w2.gap_m, 1000), "mm", 0),
        num(w2.gap_chord) != null
          ? `${fmt(w2.gap_chord, 2)}× chord · biplane band 0.8–1.25`
          : sub(frag(w2.gap_mac, "",
              "× MAC — the rear wing rides clear of the front wake", 2))],
      ["Stagger",
        num(w2.stagger_m) != null
          ? u(mul(w2.stagger_m, 1000), "mm", 0)
          : ut(w2.stagger_mac, "× MAC", 1),
        num(w2.stagger_chord) != null
          ? `${fmt(w2.stagger_chord, 2)}× chord, upper wing ahead`
          : "AC to AC, in rear-wing MAC"],
      ["Decalage", ut(w2.decalage_deg, "°"),
        sub(frag(w2.incidence_deg, "incidence ", "°"))],
      num(w2.lift_share) != null
        ? ["Lift share", ut(mul(w2.lift_share, 100), "%"),
            sub("carried by the front wing",
                frag(s.cl_ratio_fr, "CLf/CLr ", "", 2),
                s.front_stalls_first === false
                  ? "CHECK: the front wing does NOT stall first" : null)]
        : ["Area ratio", fmt(w2.lower_upper_area_ratio, 2),
            "lower / upper — equal wings are the efficient arrangement"],
    ]]] : []),
    ...(tail ? [["Tail", [
      // v3: non-default arrangements (H-tail between booms, V-tail, T-tail)
      // announce themselves; a plain conventional tail adds no row.
      ...((tail.arrangement && tail.arrangement !== "conventional")
        ? [["Arrangement",
            esc(String(tail.arrangement).replace(/_/g, " ")),
            frag(tail.n_fins, "", " fins, one per boom", 0)]] : []),
      ["Tail volumes",
        (num(tail.V_H) == null || num(tail.V_V) == null) ? "—"
          : `V_H ${fmt(tail.V_H, 2)} · V_V ${fmt(tail.V_V, 3)}`,
        "tail-aft bands: horizontal 0.40–0.65 · vertical 0.025–0.05"],
      ["Stab span", u(mul(tail.span_h_m, 1000), "mm", 0),
        sub(frag(mul(tail.area_h_m2, 1e4), "", " cm²", 0),
            span2n(mul(tail.c_root_h_m, 1000), mul(tail.c_tip_h_m, 1000),
                   "mm chord"))],
      ["Fin height", u(mul(tail.height_v_m, 1000), "mm", 0),
        sub(frag(mul(tail.area_v_m2, 1e4), "", " cm²", 0),
            span2n(mul(tail.c_root_v_m, 1000), mul(tail.c_tip_v_m, 1000),
                   "mm chord"))],
      ["Tail arm", u(mul(s.l_h_m, 1000), "mm", 0),
        sub(frag((num(s.l_h_m) != null && num(s.mac_m) > 0)
          ? s.l_h_m / s.mac_m : null, "", "× MAC · CG to stab quarter-chord", 1))],
      ["Stab incidence", ut(tail.incidence_h_deg, "°"),
        "rigged so the elevator sits neutral in cruise"],
    ]]] : (aftFins ? [["Fins", [
      // v3: canard/tandem carry geometry.fins instead of a tail block —
      // the vertical surfaces are all the "tail" these types have.
      ["Arrangement",
        esc(String(aftFins.arrangement || "fins").replace(/_/g, " ")),
        frag(aftFins.count, "", " ×", 0)],
      ["Fin area", u(mul(aftFins.area_total_m2, 1e4), "cm²", 0),
        frag((num(aftFins.area_total_m2) != null && num(g.area_m2) > 0)
          ? (aftFins.area_total_m2 / g.area_m2) * 100 : null,
          "", "% of wing area", 1)],
      ["Fin height", u(mul(aftFins.height_m, 1000), "mm", 0),
        sub(span2n(mul(aftFins.c_root_m, 1000), mul(aftFins.c_tip_m, 1000),
                   "mm chord"),
            frag(mul(aftFins.y_m, 1000), "at y = ", " mm", 0))],
      ["Lateral-area balance", ut(aftFins.cla_moment_ratio, "×", 2),
        "aft / forward side-area moment — needs ≥ 1.25"],
    ]]] : [])),
    // v3: twin-boom carriers (geometry.booms) — spacing vs prop disk and
    // the recorded stiffness-proxy section.
    ...(boomsG ? [["Tail booms", [
      ["Spacing", u(mul(boomsG.spacing_m, 1000), "mm", 0),
        sub(frag(boomsG.spacing_over_prop_d, "", "× prop Ø — needs ≥ 1.15", 2),
            frag(mul(boomsG.prop_diameter_est_m, 1000),
                 "prop est. ", " mm", 0))],
      ["Boom length", u(mul(boomsG.length_m, 1000), "mm", 0),
        frag(mul(boomsG.x_start_m, 1000), "starts ", " mm from the nose", 0)],
      ["Boom section",
        (boomsG.section_mm && num(boomsG.section_mm.tube_od_mm) != null)
          ? `Ø${fmt(boomsG.section_mm.tube_od_mm, 0)} × ` +
            `${fmt(boomsG.section_mm.tube_wall_mm, 0)} mm`
          : "—",
        boomsG.section_mm
          ? sub(boomsG.section_mm.type
                  ? esc(String(boomsG.section_mm.type).replace(/_/g, " "))
                  : null,
                (num(boomsG.section_mm.I_mm4) != null &&
                 num(boomsG.section_mm.I_required_mm4) != null)
                  ? `I ${fmt(boomsG.section_mm.I_mm4, 0)} ≥ ` +
                    `${fmt(boomsG.section_mm.I_required_mm4, 0)} mm⁴ required`
                  : null)
          : ""],
    ]]] : []),
    // v3: glider structure — spar callout, polyhedral break, nose-ballast
    // provision (n_motors = 0).
    ...((spar || poly || (ballastG != null && ballastG > 0))
      ? [["Glider structure", [
        ...(spar ? [["Wing spar",
          spar.required
            ? `Ø${fmt(spar.d_root_mm, 0)} mm carbon`
            : "not required",
          spar.required
            ? "round tube socket at the root, taper allowed outboard — " +
              "the printed skin carries torsion only"
            : "span and AR sit inside what the printed skin carries alone"]]
          : []),
        ...(poly ? [["Polyhedral",
          `${fmt(poly.inner_deg, 0)}° / ${fmt(poly.outer_deg, 0)}°`,
          sub(frag(mul(poly.break_frac, 100), "break at ", "% semi-span", 0),
              frag(poly.equivalent_dihedral_deg, "≈ ",
                   "° equivalent dihedral", 1))]] : []),
        ...((ballastG != null && ballastG > 0) ? [["Nose ballast",
          u(ballastG, "g", 0),
          "provision in place of the power system — pure sailplane"]] : []),
      ]]] : []),
    ["Handling", [
      ["Yaw stiffness", u(s.cn_beta, "/rad", 4),
        sub("Cn_β · positive = it holds heading",
            frag(s.cn_beta_fin, "fin ", "", 4),
            frag(s.cn_beta_fus, "fuselage ", "", 4))],
      ["Roll stability", u(s.cl_beta, "/rad", 4),
        sub("Cl_β · negative = a sideslip rolls it level",
            frag(s.cl_beta_dihedral, "dihedral ", "", 4))],
      ["Roll : yaw balance",
        num(s.roll_yaw_ratio) == null || !Number.isFinite(s.roll_yaw_ratio)
          ? "—" : `${fmt(s.roll_yaw_ratio, 1)} : 1`,
        "high = Dutch-roll wag, low = spiral tendency"],
      ["Elevator trim used", u(mul(s.elevator_trim_used, 100), "%", 0),
        "of full travel held for trim; the rest is your flare"],
    ]],
    ["Motor mount", [
      ["Screw holes", num(mm.n_screws) == null ? "—" : `${mm.n_screws} ×`,
        sub(mm.pattern ? `${esc(mm.pattern)} pattern` : null,
            frag(mm.pattern_side_mm, "", " mm square", 1))],
      ["Screw hole Ø", u(mm.screw_hole_d_mm, "mm", 1),
        num(mm.screw_hole_d_mm) == null ? ""
          : `clearance for M${Math.max(2, Math.round(mm.screw_hole_d_mm - 0.2))}`],
      ["Bolt-circle radius", u(mm.bolt_circle_radius_mm, "mm", 2),
        "from the motor centre to each hole"],
      ["Centre bore", u(mm.shaft_hole_d_mm, "mm", 1),
        "shaft / wire clearance"],
      ["Mount plate", u(mm.plate_thickness_mm, "mm", 1),
        sub(frag(mm.plate_radius_mm, "", " mm radius", 1),
            mm.type ? `${esc(mm.type)}, face flush with the ` +
              (mm.type === "tractor" ? "nose" : "trailing edge") : null)],
    ]],
    ["Controls & power", [
      ["Ailerons",
        (num(ail.inner_frac) == null || num(ail.outer_frac) == null) ? "—"
          : `${fmt(mul(ail.inner_frac, 100), 0)}–${fmt(mul(ail.outer_frac, 100), 0)}% semi-span`,
        num(ail.chord_frac) == null ? ""
          : `${fmt(mul(ail.chord_frac, 100), 0)}% of the local chord`],
      tail
        ? ["Elevator / rudder",
            (num(tail.elevator_chord_frac) == null ||
             num(tail.rudder_chord_frac) == null) ? "—"
              : `${fmt(mul(tail.elevator_chord_frac, 100), 0)}% / ` +
                `${fmt(mul(tail.rudder_chord_frac, 100), 0)}%`,
            "of the stab / fin chord"]
        // v3: no tail block — the elevator rides the canard / front wing
        : ["Elevator",
            num(evSrc.elevator_chord_frac) == null ? "—"
              : `${fmt(mul(evSrc.elevator_chord_frac, 100), 0)}%`,
            can ? "of the canard chord — full-span on the foreplane"
                : "of the front-wing chord — full-span elevator forward"],
      ["Motors",
        nMotors == null ? "—" : (nMotors === 0 ? "none" : `${nMotors} ×`),
        nMotors === 0
          ? "pure sailplane — nose ballast holds the CG instead"
          : (motorType ? `${esc(motorType)} layout` : "")],
      ["Pack position", u(mul(g.battery_x_m, 1000), "mm", 0),
        sub(frag(mul(g.battery_x_required_m, 1000), "", " mm needed for balance", 0))],
      ["Pack mass", u(mul(ps.pack_mass_kg, 1000), "g", 0),
        "allowance in the weight budget"],
    ]],
    ["Aerodynamics", [
      ["All-up weight", u(mul(mass.total_kg, 1000), "g", 0),
        sub(g.build_method
              ? esc(String(g.build_method).replace(/_/g, " ").replace("3d", "3D"))
              : null,
            frag(g.wall_mm, "", " mm wall"))],
      ["Wing loading", u(a.wing_loading_kgm2, "kg/m²"), ""],
      ["Cruise / stall", `${fmt(a.v_cruise_ms)} / ${u(a.v_stall_ms, "m/s")}`,
        `margin ${fmt(a.stall_margin, 2)}×`],
      ["L/D cruise", fmt(a.ld_cruise), `CL ${fmt(a.cl_cruise, 2)}`],
      ["Reynolds (MAC)", u(mul(a.re_mac, 1e-3), "k", 0), ""],
    ]],
    ["Stability", [
      ["Static margin", ut(mul(s.static_margin, 100), "%"),
        // v3: canard/tandem reference the margin to the REAR wing's MAC
        // alone (LEN-CAN §2), so the band reads differently.
        s.static_margin_ref === "MAC_rear"
          ? "CG ahead of NP · % of the rear wing's MAC · canard band 20–25%"
          : "CG ahead of NP · tailed band 5–15%"],
      ["CG", ut(s.cg_pct_mac, "% MAC"),
        sub(frag(s.cg_mm_from_nose, "", " mm from the nose datum", 0))],
      ["dCm/dα", fmt(s.dcm_dalpha, 3), "negative = pitch-stable"],
    ]],
  ] : null;

  const groups = convGroups || [
    ["Planform", [
      ["Family", esc(d.planform_label || planformLabel(d.planform)),
        missionLabel(d.mission) + " mission"],
      ["Span", u(mul(g.span_m, 1000), "mm", 0), "tip to tip"],
      ["Wing area", u(mul(g.area_m2, 1e4), "cm²", 0), `AR ${fmt(g.aspect_ratio, 2)}`],
      ["Root / tip chord",
        span2(mul(g.root_chord_m, 1000), mul(g.tip_chord_m, 1000), "mm"),
        `taper ${fmt(g.taper, 2)}`],
      ["LE sweep", ut(g.sweep_le_deg, "°"),
        sub(frag(g.dihedral_deg, "dihedral ", "°"))],
      ["Washout", ut(g.washout_deg, "°"),
        sub(frag(g.root_incidence_deg, "root incidence ", "°"),
            s.bell_spanload ? "bell spanload" : null)],
      ["Airfoil", esc(g.airfoil || "—"),
        sub(g.airfoil ? "reflexed section" : null,
            (!noFins && g.fin_airfoil) ? `fins ${esc(g.fin_airfoil)}` : null)],
    ]],
    ["Centre body", [
      ["Body depth", u(b.depth_scale, "× wing t/c", 2),
        sub(frag(b.chord_scale, "chord ×", "", 2),
            frag(mul(b.half_width_m, 1000), "blends out by ", " mm", 0))],
      ["Equipment bay",
        (num(b.bay_length_m) == null || num(b.bay_width_m) == null) ? "—"
          : `${fmt(mul(b.bay_length_m, 1000), 0)} × ` +
            `${fmt(mul(b.bay_width_m, 1000), 0)} mm`,
        sub("hollow, opened by a removable lid",
            frag(mul(b.bay_wall_m, 1000), "", " mm walls", 1))],
      ["Hatch lid", "separate part",
        "exported as its own body in STEP and its own file in Export parts"],
      ["Overall size",
        (num(g.length_total_m) == null || num(g.height_total_m) == null) ? "—"
          : `${fmt(mul(g.length_total_m, 1000), 0)} × ` +
            `${fmt(mul(g.height_total_m, 1000), 0)} mm`,
        "length × height from the nose datum"],
    ]],
    ["Vertical surfaces", vstabRows],
    // The other two axes. Longitudinal balance says whether it flies at all;
    // these say how it will feel doing it.
    ["Handling", [
      ["Yaw stiffness", u(s.cn_beta, "/rad", 4),
        sub("Cn_β · positive = it holds heading",
            frag(s.cn_beta_fin, "fins ", "", 4),
            frag(s.cn_beta_sweep, "sweep ", "", 4))],
      ["Roll stability", u(s.cl_beta, "/rad", 4),
        sub("Cl_β · negative = a sideslip rolls it level",
            frag(s.cl_beta_dihedral, "dihedral ", "", 4))],
      ["Roll : yaw balance",
        num(s.roll_yaw_ratio) == null || !Number.isFinite(s.roll_yaw_ratio)
          ? "—" : `${fmt(s.roll_yaw_ratio, 1)} : 1`,
        "high = Dutch-roll wag, low = spiral tendency"],
      ["Yaw on roll input",
        num(s.proverse_yaw) == null ? "—"
          : (s.proverse_yaw > 0.02 ? "proverse" :
            (s.proverse_yaw < -0.02 ? "adverse" : "neutral")),
        num(s.proverse_yaw) == null ? ""
          : (s.proverse_yaw > 0.02
            ? "nose swings INTO the turn — co-ordinated on elevons alone"
            : "nose swings out of the turn — dial in elevon differential")],
      ["Elevon trim used", u(mul(s.elevon_trim_used, 100), "%", 0),
        "of full travel held for trim; the rest is your flare"],
    ]],
    // Everything needed to drill/print the mount and bolt a motor to it. These
    // are the numbers the CAD actually uses, so what is printed matches.
    ["Motor mount", [
      ["Screw holes", num(mm.n_screws) == null ? "—" : `${mm.n_screws} ×`,
        sub(mm.pattern ? `${esc(mm.pattern)} pattern` : null,
            frag(mm.pattern_side_mm, "", " mm square", 1))],
      ["Screw hole Ø", u(mm.screw_hole_d_mm, "mm", 1),
        num(mm.screw_hole_d_mm) == null ? ""
          : `clearance for M${Math.max(2, Math.round(mm.screw_hole_d_mm - 0.2))}`],
      ["Bolt-circle radius", u(mm.bolt_circle_radius_mm, "mm", 2),
        "from the motor centre to each hole"],
      ["Centre bore", u(mm.shaft_hole_d_mm, "mm", 1),
        "shaft / wire clearance"],
      ["Mount plate", u(mm.plate_thickness_mm, "mm", 1),
        sub(frag(mm.plate_radius_mm, "", " mm radius", 1),
            mm.type ? `${esc(mm.type)}, face flush with the ` +
              (mm.type === "tractor" ? "nose" : "trailing edge") : null)],
    ]],
    ["Controls & power", [
      ["Elevons",
        (num(el.inner_frac) == null || num(el.outer_frac) == null) ? "—"
          : `${fmt(mul(el.inner_frac, 100), 0)}–${fmt(mul(el.outer_frac, 100), 0)}% semi-span`,
        num(el.chord_frac) == null ? ""
          : `${fmt(mul(el.chord_frac, 100), 0)}% of the local chord`],
      ["Motors", nMotors ? `${nMotors} ×` : "—",
        motorType ? `${esc(motorType)} layout` : ""],
      ["Pack position", u(mul(g.battery_x_m, 1000), "mm", 0),
        sub(frag(mul(g.battery_x_required_m, 1000), "", " mm needed for balance", 0))],
      ["Pack mass", u(mul(ps.pack_mass_kg, 1000), "g", 0),
        "allowance in the weight budget"],
    ]],
    ["Aerodynamics", [
      ["All-up weight", u(mul(mass.total_kg, 1000), "g", 0),
        sub(g.build_method
              ? esc(String(g.build_method).replace(/_/g, " ").replace("3d", "3D"))
              : null,
            frag(g.wall_mm, "", " mm wall"))],
      ["Wing loading", u(a.wing_loading_kgm2, "kg/m²"), ""],
      ["Cruise / stall", `${fmt(a.v_cruise_ms)} / ${u(a.v_stall_ms, "m/s")}`,
        `margin ${fmt(a.stall_margin, 2)}×`],
      ["L/D cruise", fmt(a.ld_cruise), `CL ${fmt(a.cl_cruise, 2)}`],
      ["Reynolds (MAC)", u(mul(a.re_mac, 1e-3), "k", 0), ""],
    ]],
    ["Stability", [
      ["Static margin", ut(mul(s.static_margin, 100), "%"),
        "CG ahead of NP · tailless band 3–15%"],
      ["CG", ut(s.cg_pct_mac, "% MAC"),
        sub(frag(s.cg_mm_from_nose, "", " mm from the nose datum", 0))],
      ["dCm/dα", fmt(s.dcm_dalpha, 3), "negative = pitch-stable"],
    ]],
  ];
  $("specs").innerHTML = groups.map(([name, rows]) =>
    `<section class="spec-group"><h4>${name}</h4>` +
    rows.map(([k, v, s2]) =>
      `<div class="item"><span>${k}</span><b>${v}</b><span>${s2 ?? ""}</span></div>`).join("") +
    `</section>`).join("");
  $("datapanel").classList.remove("hidden");
}

/* ---------------- plain text -> HTML ("- " bullets, blank-line paragraphs) -- */
function textToHtml(body) {
  const out = [];
  let list = null, para = [];
  const flushPara = () => { if (para.length) { out.push(`<p>${para.join(" ")}</p>`); para = []; } };
  const flushList = () => { if (list) { out.push(`<ul>${list.join("")}</ul>`); list = null; } };
  for (const raw of String(body ?? "").split(/\r?\n/)) {
    const line = raw.trim();
    if (!line) { flushPara(); flushList(); continue; }
    if (line.startsWith("- ")) {
      flushPara();
      (list = list || []).push(`<li>${esc(line.slice(2))}</li>`);
    } else {
      flushList();
      para.push(esc(line));
    }
  }
  flushPara(); flushList();
  return out.join("");
}

/* ---------------- design notes (guidance) tab ---------------- */
function showGuidance(sections) {
  const tab = document.querySelector('.dp-tab[data-pane="notes"]');
  const has = Array.isArray(sections) && sections.length > 0;
  $("guidance_body").innerHTML = has ? sections.map((sec) =>
    `<section class="guidance-section"><h3>${esc(sec && sec.title)}</h3>` +
    `${textToHtml(sec && sec.body)}</section>`).join("") : "";
  tab.disabled = !has;
  tab.title = has ? "" : "No notes for this design";
  // a variant with no notes must not leave an empty pane showing
  if (!has && tab.classList.contains("is-active")) setPane("specs");
}

/* ---------------- Learn overlay (knowledge browser) ---------------- */
let learnArticles = null; // null = not fetched yet
function renderLearnList() {
  $("learn_list").innerHTML = learnArticles.map((a, i) =>
    `<li data-i="${i}"><b>${esc(a.title)}</b><span>${esc(a.summary ?? "")}</span></li>`
  ).join("");
  $("learn_article").innerHTML = learnArticles.length
    ? '<p class="placeholder">Select an article on the left.</p>'
    : '<p class="placeholder">No articles yet.</p>';
}
async function openLearn() {
  $("learn_modal").classList.remove("hidden");
  if (learnArticles !== null) return; // fetched once, keep it
  try {
    const res = await fetch("/api/learn");
    if (!res.ok) throw new Error(String(res.status));
    const body = await res.json();
    learnArticles = Array.isArray(body.articles) ? body.articles : [];
    renderLearnList();
  } catch {
    $("learn_list").innerHTML = "";
    $("learn_article").innerHTML =
      '<p class="placeholder">Knowledge base not available yet.</p>';
    learnArticles = null; // allow a retry on next open
  }
}
$("learn_btn").addEventListener("click", openLearn);
$("learn_close").addEventListener("click", () =>
  $("learn_modal").classList.add("hidden"));
$("learn_modal").addEventListener("click", (e) => {
  if (e.target === $("learn_modal")) $("learn_modal").classList.add("hidden");
});
document.addEventListener("keydown", (e) => {
  if (e.key === "Escape") $("learn_modal").classList.add("hidden");
});
$("learn_list").addEventListener("click", (e) => {
  const li = e.target.closest("li[data-i]");
  if (!li || !learnArticles) return;
  const a = learnArticles[Number(li.dataset.i)];
  if (!a) return;
  for (const el of $("learn_list").children)
    el.classList.toggle("active", el === li);
  $("learn_article").innerHTML =
    `<h2>${esc(a.title)}</h2>${textToHtml(a.body)}`;
  $("learn_article").scrollTop = 0;
});

/* ---------------- banners ---------------- */
function showBanner(d) {
  const b = $("banner");
  if (!d || d.feasible !== false) { b.className = "hidden"; b.textContent = ""; return; }
  const bad = (Array.isArray(d.constraints) ? d.constraints : []).filter((c) => c && !c.ok);
  b.textContent = "Closest feasible wing — " +
    (bad[0] && bad[0].message ? bad[0].message : "a constraint is binding.");
  b.className = "warn";
}
/* Features the CAD build REFUSED (a bay that would not carve, a servo
   pocket the section cannot bury, a wire run that cannot stay inside the
   skin): the server names each one in parts.json `refusals`; a solid body
   with nothing said about it is exactly what the user must never see. */
function showRefusals(list) {
  if (!Array.isArray(list) || list.length === 0) return;
  const b = $("banner");
  const head = "Built with refusals — ";
  const text = head + list.slice(0, 4).join(" · ") +
    (list.length > 4 ? ` · +${list.length - 4} more` : "");
  b.textContent = (b.className === "warn" && b.textContent && !b.textContent.startsWith(head))
    ? b.textContent + "  |  " + text : text;
  b.className = "warn";
}
function failBanner(msg) {
  const b = $("banner");
  b.textContent = msg;
  b.className = "bad";
}

/* ================= variant gallery ================= */
const vid = (v) => (v && (v.id ?? (v.design && v.design.id))) || null;
const exportIdOf = (v) => (v && ((v.design && v.design.id) ?? v.id)) || null;

const cardObserver = ("IntersectionObserver" in window)
  ? new IntersectionObserver((entries) => {
      for (const en of entries) {
        const id = en.target.dataset.id;
        if (!minis) return;
        minis.setVisible(id, en.isIntersecting);
        if (en.isIntersecting) minis.request(id);
      }
    }, { threshold: 0.05 })
  : null;

if (minis) minis.onState = (id, state) => {
  const view = $("variant_rail").querySelector(`[data-id="${CSS.escape(id)}"] .vc-view`);
  if (!view) return;
  view.className = "vc-view is-" + state;
  const label = view.querySelector(".vc-state");
  if (!label) return;
  if (state === "loading") label.innerHTML = '<i class="spinner"></i>';
  else if (state === "error") label.textContent = "Preview unavailable";
  else if (state === "ready") label.textContent = "";
};

function traitRow(t) {
  const raw = Number(t && t.value);
  const pct = Math.round(Math.max(0, Math.min(1, Number.isFinite(raw) ? raw : 0)) * 100);
  const detail = esc(t && t.detail);
  return `<span class="trait" title="${esc(t && t.label)}${detail ? ": " + detail : ""}">` +
    `<span class="t-label">${esc(t && t.label)}</span>` +
    `<span class="t-bar"><i style="--fill:${pct}%"></i></span>` +
    `<span class="t-detail">${detail}</span></span>`;
}

function renderVariants() {
  const rail = $("variant_rail");
  const section = $("variants");
  if (minis) minis.reset();
  if (cardObserver) cardObserver.disconnect();   // release the old cards

  if (variants.length < 2) {   // nothing to choose between — hide the gallery
    rail.innerHTML = "";
    section.classList.add("hidden");
    return;
  }
  section.classList.remove("hidden");
  const families = new Set(variants
    .map((v) => v.design && v.design.planform).filter(Boolean));
  // v2/v3: a gallery that contains any non-flying-wing design is not
  // "wings". The flying-wing-only strings are exactly the v1 ones.
  const hasConv = variants.some((v) => {
    const t = v.airplane_type || (v.design && v.design.airplane_type);
    return t && t !== "flying_wing";
  });
  const title = document.querySelector(".variants-title");
  if (title) title.textContent = hasConv ? "Design variants" : "Wing variants";
  $("variants_sub").textContent = hasConv
    ? `${variants.length} designs from the same mission` +
      (families.size > 1 ? ` across ${families.size} characters` : "") +
      " — click to compare"
    : `${variants.length} wings from the same mission` +
      (families.size > 1 ? ` across ${families.size} planform families` : "") +
      " — click to compare";

  rail.innerHTML = variants.map((v, i) => {
    const id = vid(v);
    const traits = Array.isArray(v.traits) ? v.traits.slice(0, 4) : [];
    const fam = (v.design && (v.design.planform_label ||
      (v.design.planform ? planformLabel(v.design.planform) : ""))) || "";
    return `<button type="button" class="variant-card" data-id="${esc(id)}"
        aria-pressed="false" style="--d:${i * 45}ms">
      <span class="vc-view is-idle">
        <canvas class="vc-canvas"></canvas>
        <span class="vc-state"></span>
      </span>
      <span class="vc-name">${esc(v.name || v.key || "Variant")}</span>
      <span class="vc-plan">${esc(fam)}</span>
      <span class="vc-tag">${esc(v.tagline || "")}</span>
      <span class="vc-traits">${traits.map(traitRow).join("")}</span>
    </button>`;
  }).join("");

  for (const card of rail.children) {
    const id = card.dataset.id;
    const v = variants.find((x) => String(vid(x)) === id);
    if (!v) continue;
    if (v.preview_stl_url) {
      if (minis) minis.attach(id, card.querySelector(".vc-canvas"), v.preview_stl_url);
    } else {
      const view = card.querySelector(".vc-view");
      view.className = "vc-view is-error";
      view.querySelector(".vc-state").textContent = "Preview unavailable";
    }
    if (cardObserver) cardObserver.observe(card);
    else if (minis) minis.setVisible(id, true);
  }
  // let the browser paint the 0-width bars, then animate them in
  requestAnimationFrame(() => requestAnimationFrame(() => {
    for (const card of rail.children) card.classList.add("in");
  }));
}

function markActiveCard() {
  for (const card of $("variant_rail").children) {
    const on = card.dataset.id === String(activeId);
    card.classList.toggle("active", on);
    card.setAttribute("aria-pressed", on ? "true" : "false");
  }
}

let selectToken = 0;
async function selectVariant(id) {
  const v = variants.find((x) => String(vid(x)) === String(id));
  if (!v) return;
  const token = ++selectToken;
  activeId = String(vid(v));
  markActiveCard();

  const design = v.design || null;
  currentDesignId = exportIdOf(v);
  showSpecs(design);
  showBanner(design);
  showGuidance(design && design.guidance);
  $("export_step").disabled = !currentDesignId;
  $("export_stl").disabled = !currentDesignId;
  $("export_parts").disabled = !currentDesignId;

  if (!v.preview_stl_url) return;
  $("stage_busy").classList.remove("hidden");
  try {
    if (viewer) {
      // Prefer the PART-BY-PART view: it is the only one that shows what
      // actually comes apart - grey moving surfaces, the darker-grey hatch,
      // the hinges between them. The one-piece STL has the elevons fused in and
      // the hatch only scribed, so it cannot show any of that. Fall back to it
      // if the assembly cannot be built.
      let drew = false;
      try {
        const r = await fetch(`/api/preview/${encodeURIComponent(activeId)}/parts.json`);
        if (r.ok) {
          const manifest = await r.json();
          if (token !== selectToken) return;
          await viewer.showDesignParts(manifest, design, lastBox);
          showPartLegend(manifest.legend);
          showRefusals(manifest.refusals);
          drew = true;
        }
      } catch { /* fall through to the single solid */ }
      if (!drew) {
        await viewer.showDesign(v.preview_stl_url, design, lastBox);
        showPartLegend(null);
      }
    }
  } catch (e) {
    if (token === selectToken) failBanner("Could not load the 3D preview for this variant.");
  } finally {
    if (token === selectToken) $("stage_busy").classList.add("hidden");
  }
  if (token !== selectToken) return;
  // the active model is on screen — now quietly prefetch the other cards
  if (minis) minis.requestAll(activeId);
}

$("variant_rail").addEventListener("click", (e) => {
  const card = e.target.closest(".variant-card");
  if (!card || card.dataset.id === String(activeId)) return;
  selectVariant(card.dataset.id);
});

$("variants_toggle").addEventListener("click", () => {
  const collapsed = $("variants").classList.toggle("collapsed");
  $("variants_toggle").textContent = collapsed ? "+" : "−";
  $("variants_toggle").title = collapsed ? "Show the gallery" : "Collapse the gallery";
});

/* ---------------- generate ---------------- */
async function generate() {
  const btn = $("generate");
  btn.disabled = true; btn.textContent = "Optimizing…";
  $("hint").classList.add("hidden");
  $("banner").className = "hidden";
  $("stage_busy").classList.remove("hidden");
  let tick = null;
  try {
    const atype = currentAirplaneType();
    if (atype !== "flying_wing") {
      // v3: "any" competes EVERY type server-side (~a minute of physics);
      // an elapsed count keeps the pending state visibly alive the whole
      // way. The flying-wing panel keeps its v1 label untouched.
      // Replaced the bare seconds counter (2026-08-27) with the shared
      // progress bar: the ETA is the median of this type's last generate
      // runs (GET /api/timing), indeterminate until there is history.
      const t0 = Date.now();
      const expected = timingExpected(`generate:${atype}`);
      showProgress("Optimizing", expected ? 0 : null, expected);
      tick = setInterval(() => {
        const el = (Date.now() - t0) / 1000;
        showProgress("Optimizing",
                     expected ? Math.min(0.97, el / expected) : null,
                     expected ? Math.max(0, expected - el) : null);
      }, 1000);
    }
    const req = atype === "flying_wing" ? buildRequest() : buildRequestAlt(atype);
    const res = await fetch("/api/generate", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(req),
    });
    if (!res.ok) {
      const err = await res.json().catch(() => ({}));
      // a string detail is the server's own sentence (an honest refusal
      // naming the constraint) - show it as written, not JSON-quoted
      const detail = err.detail;
      throw new Error(detail
        ? (typeof detail === "string" ? detail : JSON.stringify(detail))
        : res.statusText);
    }
    const body = await res.json();
    lastBox = req.box;

    // Preferred shape: {variants:[...], primary_id}. Fall back to a lone design.
    let list = Array.isArray(body.variants)
      ? body.variants.filter((v) => v && vid(v)) : [];
    if (list.length === 0 && body.design) {
      list = [{
        id: body.design.id, key: "primary", name: "Design", tagline: "",
        traits: [], design: body.design,
        preview_stl_url: body.preview_stl_url,
      }];
    }
    if (list.length === 0) throw new Error("the server returned no design.");
    variants = list;

    // Drop live references BEFORE freeing the previous run's GPU buffers.
    if (viewer) viewer.clear();
    if (minis) minis.reset();
    clearGeometryCache();

    renderVariants();
    let primary = list.find((v) => String(vid(v)) === String(body.primary_id));
    if (atype === "conventional" && $("conv_style") && $("conv_style").value) {
      // The Style selector's promise: that character leads the gallery. The
      // backend generates all five and marks the mission's own pick primary;
      // the chosen style's variant (its design carries the style name on the
      // planform key) is selected here instead.
      const styled = list.find((v) =>
        v.design && v.design.planform === $("conv_style").value);
      if (styled) primary = styled;
    }
    await selectVariant(vid(primary || list[0]));
  } catch (e) {
    failBanner("Generation failed: " + e.message);
    $("stage_busy").classList.add("hidden");
  } finally {
    if (tick) { clearInterval(tick); hideProgress(); refreshTiming(); }
    btn.disabled = false; btn.textContent = "Generate";
  }
}
/* ================= task 5: UI safety ================= */
/* Three guards, all built on ONE fact: the canonical JSON of the request the
   form WOULD send right now (buildRequest / buildRequestAlt, exactly what
   generate() sends) versus the JSON of the request last sent successfully.
   1. Generate is disabled while the two are equal and a design exists - a
      re-run would only reproduce the gallery on screen.
   2. Re-generating over an existing design, with changes, asks first and
      lists the changed fields (sidebar labels, old -> new). While a
      generation is in flight the click is BLOCKED instead: the running
      request cannot be cancelled server-side, so the safe behaviour is to
      wait for it rather than race two galleries.
   3. Exporting with a dirty form asks too: the export is of the LAST
      generated design; the edits on the sidebar are not in it.
   The v1 flying-wing panel's labels and request shape are untouched. */
let lastSentJson = null;     // canonical JSON of the last request that produced a gallery
let lastSentReq = null;      // the same, parsed (for the change list)
let generating = false;      // one generation at a time

function currentRequest() {
  const atype = currentAirplaneType();
  return atype === "flying_wing" ? buildRequest() : buildRequestAlt(atype);
}
// key-sorted JSON so field order can never make an unchanged form look dirty
function canonical(obj) {
  const sortKeys = (v) => {
    if (Array.isArray(v)) return v.map(sortKeys);
    if (v && typeof v === "object") {
      return Object.fromEntries(Object.keys(v).sort().map((k) => [k, sortKeys(v[k])]));
    }
    return typeof v === "number" && Number.isNaN(v) ? null : v;
  };
  return JSON.stringify(sortKeys(obj));
}
function formIsDirty() {
  return lastSentJson === null || canonical(currentRequest()) !== lastSentJson;
}
function refreshDirty() {
  const btn = $("generate");
  if (!btn || generating) return;
  const clean = variants.length > 0 && !formIsDirty();
  btn.disabled = clean;
  btn.classList.toggle("is-clean", clean);
  btn.title = clean ? "No changes since the last generation" : "";
}

/* request key -> the sidebar control that owns it, so the change list can
   speak in the sidebar's own labels and option texts. The mission / motors
   keys are shared by two panels; the one that is visible wins. */
const REQ_FIELD_IDS = {
  airplane_type: "airplane_type", planform: "planform", vstab: "vstab",
  cruise_speed_ms: "cruise_ms", stall_speed_target_ms: "stall_ms",
  payload_g: "payload_g", "box.length_mm": "box_l", "box.width_mm": "box_w",
  "box.height_mm": "box_h", allow_folding: "allow_folding",
  build_method: "build_method", material: "material",
  airfoil_override: "airfoil", aspect_ratio_target: "ar",
  static_margin_override: "sm", mount_screws: "mount_screws",
  mount_screw_d_mm: "mount_screw_d", mount_bolt_circle_mm: "mount_bolt_circle",
  motor_layout: "cfg_motor_layout", tail_type: "cfg_tail_type",
  wing_position: "cfg_wing_position",
};
function fieldIdFor(key) {
  if (key === "mission") return currentAirplaneType() === "flying_wing" ? "mission" : "alt_mission";
  if (key === "n_motors") return currentAirplaneType() === "flying_wing" ? "n_motors" : "alt_n_motors";
  return REQ_FIELD_IDS[key] || null;
}
function labelFor(key) {
  const id = fieldIdFor(key);
  const el = id && $(id);
  const row = el && el.closest("label.row");
  const span = row && row.querySelector("span:first-child");
  let text = span ? span.textContent.trim() : key.replace(/_/g, " ");
  if (key.startsWith("box.")) text = "Box " + text.toLowerCase();
  return text;
}
function valueText(key, v) {
  if (v === null || v === undefined) return "auto";
  if (typeof v === "boolean") return v ? "yes" : "no";
  const id = fieldIdFor(key);
  const el = id && $(id);
  if (el && el.tagName === "SELECT") {
    const opt = Array.from(el.options).find((o) => o.value === String(v));
    if (opt) return opt.textContent.trim();
  }
  if (key === "static_margin_override") return (v * 100).toFixed(1) + " %";
  if (key.startsWith("box.")) return v + " mm";
  if (key === "cruise_speed_ms" || key === "stall_speed_target_ms") return v + " m/s";
  if (key === "payload_g") return v + " g";
  return String(v);
}
function flatten(obj, prefix = "", out = {}) {
  for (const [k, v] of Object.entries(obj || {})) {
    const key = prefix ? prefix + "." + k : k;
    if (v && typeof v === "object" && !Array.isArray(v)) flatten(v, key, out);
    else out[key] = v;
  }
  return out;
}
function requestChanges(prev, next) {
  const a = flatten(prev), b = flatten(next);
  const keys = Array.from(new Set([...Object.keys(a), ...Object.keys(b)]));
  return keys
    .filter((k) => canonical(a[k] ?? null) !== canonical(b[k] ?? null))
    .map((k) => ({ key: k, label: labelFor(k),
                   from: valueText(k, a[k]), to: valueText(k, b[k]) }));
}
function changesHtml(changes) {
  return "<ul>" + changes.map((c) =>
    `<li>${esc(c.label)}: <span class="chg-old">${esc(c.from)}</span>` +
    `<span class="chg-arrow">&rarr;</span><span class="chg-new">${esc(c.to)}</span></li>`
  ).join("") + "</ul>";
}

/* the in-page dialog: resolves true on the primary button, false otherwise */
let confirmResolve = null;
function closeConfirm(result) {
  $("confirm_modal").classList.add("hidden");
  const r = confirmResolve; confirmResolve = null;
  if (r) r(result);
}
function openConfirm({ title, html, okLabel, cancelLabel }) {
  $("confirm_title").textContent = title;
  $("confirm_body").innerHTML = html;
  const ok = $("confirm_ok"), cancel = $("confirm_cancel");
  ok.textContent = okLabel || "OK";
  ok.hidden = !okLabel;
  cancel.textContent = cancelLabel || "Cancel";
  $("confirm_modal").classList.remove("hidden");
  return new Promise((resolve) => {
    confirmResolve = resolve;
    (okLabel ? ok : cancel).focus();
  });
}
$("confirm_ok").addEventListener("click", () => closeConfirm(true));
$("confirm_cancel").addEventListener("click", () => closeConfirm(false));
$("confirm_modal").addEventListener("click", (e) => {
  if (e.target === $("confirm_modal")) closeConfirm(false);
});
document.addEventListener("keydown", (e) => {
  if (e.key === "Escape" && confirmResolve) closeConfirm(false);
});

function describeCurrentDesign() {
  const v = variants.find((x) => String(vid(x)) === String(activeId)) || variants[0];
  const d = v && v.design;
  if (!d) return "the last generated design";
  const g = d.geometry || {};
  const bits = [v.name || d.planform_label || d.planform];
  if (d.airplane_type) bits.push(d.airplane_type.replace(/_/g, " "));
  if (g.span_m) bits.push((g.span_m * 1000).toFixed(0) + " mm span");
  if (d.mission) bits.push(d.mission.replace(/_/g, " "));
  return bits.filter(Boolean).join(", ");
}

async function runGenerate(req) {
  generating = true;
  const before = variants;          // generate() reassigns on success only
  try {
    await generate();
  } finally {
    generating = false;
    if (variants !== before) {
      lastSentJson = canonical(req);
      lastSentReq = req;
    }
    refreshDirty();
  }
}
async function guardedGenerate() {
  if (generating) {
    await openConfirm({
      title: "Generation in progress",
      html: "<p>A design is still being generated. Wait for it to finish, " +
            "then press Generate again with your changes - two runs at once " +
            "would race for the same gallery.</p>",
      okLabel: null, cancelLabel: "OK",
    });
    return;
  }
  const req = currentRequest();
  if (variants.length && lastSentReq) {
    const changes = requestChanges(lastSentReq, req);
    if (changes.length === 0) { refreshDirty(); return; }   // nothing to do
    const go = await openConfirm({
      title: "Re-generate?",
      html: "<p>This replaces the current five designs with a new run using " +
            "these changes:</p>" + changesHtml(changes),
      okLabel: "Re-generate", cancelLabel: "Cancel",
    });
    if (!go) return;
  }
  await runGenerate(req);
}
async function guardedExport(format) {
  if (!currentDesignId) return;
  if (variants.length && formIsDirty()) {
    const changes = lastSentReq ? requestChanges(lastSentReq, currentRequest()) : [];
    const go = await openConfirm({
      title: "Export the last generated design?",
      html: `<p>The export is of the <b>last generated</b> design ` +
            `(${esc(describeCurrentDesign())}). Your current sidebar changes ` +
            `are <b>not</b> included:</p>` +
            (changes.length ? changesHtml(changes) : "") +
            `<p class="muted">Press Generate first to design with them.</p>`,
      okLabel: "Export anyway", cancelLabel: "Cancel",
    });
    if (!go) return;
  }
  return doExport(format);
}
$("generate").addEventListener("click", guardedGenerate);
// every input / change on the sidebar re-evaluates the dirty state (delegated,
// so rows rendered later - the v3 config axes - are covered too)
for (const ev of ["input", "change"]) {
  $("sidebar").addEventListener(ev, () => { refreshDirty(); syncBoxReco(); });
}

/* recommended minimum box for the selected type - advisory only */
function recommendedBoxFor(type) {
  if (type === "any") {
    // whichever configuration wins must fit, so the hint is the element-wise
    // envelope of every registry type's floor
    const all = airplaneTypes.map((t) => t.recommended_min_box_mm || [])
      .filter((b) => b.length === 3);
    return all.length ? [0, 1, 2].map((i) => Math.max(...all.map((b) => b[i]))) : null;
  }
  const rec = airplaneTypes.find((t) => t.name === type);
  const b = rec && rec.recommended_min_box_mm;
  return b && b.length === 3 ? b : null;
}
function syncBoxReco() {
  const el = $("box_reco");
  if (!el) return;
  const box = recommendedBoxFor(currentAirplaneType());
  if (!box) { el.hidden = true; el.textContent = ""; return; }
  const cur = [$("box_l"), $("box_w"), $("box_h")].map((i) => parseFloat(i.value));
  const low = cur.some((v, i) => Number.isFinite(v) && v < box[i]);
  el.hidden = false;
  el.classList.toggle("is-low", low);
  el.textContent = `Recommended ≥ ${box[0]} × ${box[1]} × ${box[2]} mm ` +
    `(L × W × H) for a clean build (servo pockets, bay, wire pipes)` +
    (low ? " — this box is smaller; the optimizer will try, but expect " +
           "servo or bay constraints to bind." : ".");
}

/* ---------------- build progress bar + ETA ---------------- */
/* One bar under the export buttons, shared by doExport and the multi-type
   generate. progress: 0-1, or null for an indeterminate sweep; eta: seconds
   or null ("ETA unknown"). The server learns durations per (type, format)
   in exports/timing.json and serves the medians at GET /api/timing. */
let timingTable = {};
async function refreshTiming() {
  try {
    const r = await fetch("/api/timing");
    if (r.ok) timingTable = (await r.json()).kinds || {};
  } catch (_e) { /* cosmetic */ }
}
function timingExpected(kind) {
  const k = timingTable[kind];
  return k && k.expected_s ? Number(k.expected_s) : null;
}
function fmtEta(s) {
  if (s == null || !isFinite(s)) return "ETA unknown";
  if (s < 5) return "almost done";
  if (s < 90) return `ETA ~${Math.max(5, Math.round(s / 5) * 5)} s`;
  return `ETA ~${Math.max(1, Math.round(s / 60))} min`;
}
function showProgress(stage, progress, eta) {
  const box = $("build_progress");
  if (!box) return;
  box.classList.remove("hidden");
  box.classList.toggle("indeterminate", progress == null);
  const pct = progress == null ? 0 : Math.round(progress * 100);
  box.querySelector(".bp-fill").style.width = progress == null ? "" : `${pct}%`;
  box.setAttribute("aria-valuenow", String(pct));
  box.querySelector(".bp-stage").textContent =
    progress == null ? `${stage}…` : `${stage}… ${pct} %`;
  box.querySelector(".bp-eta").textContent = fmtEta(eta);
}
function hideProgress() {
  const box = $("build_progress");
  if (box) box.classList.add("hidden");
}
refreshTiming();

/* ---------------- part colour legend ---------------- */
/* The colours in the viewer mean the same thing as the colours in the STEP
   export, so the legend is the key to both. */
function showPartLegend(legend) {
  const el = $("part_legend");
  if (!el) return;
  if (!legend || !legend.length) { el.className = "hidden"; el.innerHTML = ""; return; }
  el.innerHTML = legend.map((r) =>
    `<span><i class="swatch" style="background:${esc(r.color)}"></i>${esc(r.label)}</span>`
  ).join("");
  el.className = "";
}

/* ---------------- export ---------------- */
/* A STEP of a full aircraft is ~28 MB and the first one costs a CAD build of
   MINUTES - longer still while the variant previews are building alongside it.
   So the build must never ride on one long fetch: Chrome kills a request whose
   response headers take too long, and the failure reads as "the button does
   nothing". Instead POST /export/start returns immediately, short status polls
   carry the wait (each one fast, none killable by a timeout), and the finished
   file is downloaded by plain navigation so the browser's own download manager
   owns it - no blob, no object URL, nothing to revoke. */
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function saveUrl(url) {
  const a = document.createElement("a");
  a.href = url; a.download = ""; a.rel = "noopener";
  a.style.display = "none";
  document.body.appendChild(a);
  a.click();
  setTimeout(() => a.remove(), 60000);
}

async function doExport(format) {
  if (!currentDesignId) return;
  const btn = { step: $("export_step"), stl: $("export_stl"),
                stl_parts: $("export_parts") }[format];
  if (!btn) return;
  const label = btn.dataset.label || (btn.dataset.label = btn.textContent);
  const id = currentDesignId;
  const t0 = Date.now();
  btn.disabled = true;
  btn.textContent = "Building…";
  // The bar: server stage + progress + ETA from /export/status (learned per
  // type/format); before the first poll answers, the type's median from
  // /api/timing gives an ETA, else the bar sweeps indeterminately.
  const dsn = (variants.find((v) => String(vid(v)) === String(id)) || {}).design;
  const kind = `${(dsn && dsn.airplane_type) || "flying_wing"}:${format}`;
  const paint = (st) => {
    const eta = st.eta_s != null ? st.eta_s : timingExpected(kind);
    const prog = st.progress != null ? st.progress : (eta ? 0 : null);
    showProgress(st.stage_label || "Building", prog, eta);
  };
  paint({ stage_label: "Starting the build" });
  try {
    const start = await fetch("/api/export/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ design_id: id, format }),
    });
    if (!start.ok) {
      const err = await start.json().catch(() => ({}));
      failBanner("Export failed: " + (err.detail || start.statusText));
      return;
    }
    let st = await start.json();
    while (st.status === "building") {
      paint(st);
      await sleep(2000);
      try {
        const rs = await fetch(`/api/export/status/${id}/${format}`);
        if (rs.ok) st = await rs.json();
      } catch (_e) { /* transient - keep polling */ }
    }
    if (st.status === "ready") showProgress("Done", 1, 0);
    if (st.status !== "ready") {
      failBanner("Export failed: " + (st.error || "the CAD build failed"));
      return;
    }
    saveUrl(st.url);
    const b = $("banner");
    b.textContent = `Saved ${st.filename} — ${(st.bytes / 1e6).toFixed(1)} MB `
      + `in ${Math.round((Date.now() - t0) / 1000)}s. Check your downloads `
      + `folder.`;
    b.className = "ok";
  } catch (e) {
    failBanner("Export failed: " + e.message);
  } finally {
    hideProgress();
    refreshTiming();
    btn.disabled = false; btn.textContent = label;
  }
}
// task 5: the guard asks first when the sidebar has changed since the design
// was generated (the export is of the LAST generated design)
$("export_step").addEventListener("click", () => guardedExport("step"));
$("export_stl").addEventListener("click", () => guardedExport("stl"));
$("export_parts").addEventListener("click", () => guardedExport("stl_parts"));
