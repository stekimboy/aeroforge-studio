"""API tests: generate returns five flying-wing variants, the response carries
the physics numbers, previews build lazily, export works."""
import concurrent.futures as cf

import pytest
from fastapi.testclient import TestClient

from backend.main import app
from backend.physics.config_defs import PLANFORMS
from backend.physics.guidance import LEARN_ARTICLES

client = TestClient(app, raise_server_exceptions=False)

SMALL_BOX = {"length_mm": 900, "width_mm": 1200, "height_mm": 300}


def _req(**over):
    body = {"planform": "swept", "mission": "sport", "box": dict(SMALL_BOX)}
    body.update(over)
    return body


@pytest.fixture(scope="module")
def generated():
    r = client.post("/api/generate", json=_req())
    assert r.status_code == 200, r.text
    return r.json()


def test_health_and_index():
    assert client.get("/health").json() == {"status": "ok"}
    r = client.get("/")
    assert r.status_code == 200
    assert b"AeroForge" in r.content


def test_options_advertises_flying_wings_only():
    r = client.get("/api/options")
    assert r.status_code == 200
    data = r.json()
    assert "configs" not in data, "the aircraft-type selector is gone"
    names = {p["name"] for p in data["planforms"]}
    assert names == set(PLANFORMS) == {"swept", "bwb", "plank", "bell"}
    for p in data["planforms"]:
        assert p["label"] and p["description"]
        assert p["vstab_options"], p["name"]
    assert {m["name"] for m in data["missions"]} == {
        "sport", "fpv_cruiser", "thermal_floater", "park_flyer"}
    assert len(data["presets"]) >= 3
    assert len(data["variants"]) == 5
    for v in data["variants"]:
        assert v["key"] and v["name"] and v["tagline"]


def test_bell_planform_offers_no_vertical_surfaces():
    data = client.get("/api/options").json()
    bell = next(p for p in data["planforms"] if p["name"] == "bell")
    assert list(bell["vstab_options"]) == ["none"], bell["vstab_options"]


def test_options_advertises_airplane_types_and_conv_styles():
    """v2: the aircraft-type dropdown and the conventional style roster are
    served ADDITIVELY - every v1 key is untouched (previous tests).
    v3: the type roster grows to eight (V3_PLAN.md)."""
    data = client.get("/api/options").json()
    types = {t["name"]: t for t in data["airplane_types"]}
    # biplane and glider removed by the builder (2026-08-21) - six types now
    assert set(types) == {"flying_wing", "conventional", "delta", "canard",
                          "tandem", "twin_boom"}
    for t in types.values():
        assert t["label"] and t["description"]
    # v3 wave 4: every type carries its configuration-axes dict, transcribed
    # from RESEARCH_TYPES_V3.md s.10.4 (first list entry = type default;
    # single-entry list = fixed; empty list = axis meaningless; n_motors is
    # a [min, max] band).
    for t in types.values():
        assert set(t["axes"]) == {"motor_layout", "n_motors", "tail_type",
                                  "wing_position"}, t
    assert types["flying_wing"]["axes"]["motor_layout"] == ["pusher"]
    assert types["flying_wing"]["axes"]["tail_type"] == []
    assert types["conventional"]["axes"]["tail_type"] == [
        "conventional", "t_tail", "v_tail"]
    assert types["conventional"]["axes"]["wing_position"] == [
        "high", "mid", "low"]
    assert types["delta"]["axes"]["tail_type"] == []
    assert types["canard"]["axes"]["motor_layout"] == ["pusher"]
    assert types["tandem"]["axes"]["wing_position"] == ["high"]
    assert types["twin_boom"]["axes"]["motor_layout"][0] == "pusher"
    assert types["twin_boom"]["axes"]["n_motors"] == [1, 2]
    styles = {s["name"]: s for s in data["conv_styles"]}
    assert set(styles) == {"trainer", "sport", "aerobatic", "floater", "speed"}
    for s in styles.values():
        assert s["label"] and s["description"] and s["blurb"]
        assert len(s["ar_band"]) == 2 and len(s["wl_band"]) == 2
    lo, hi = data["conv_static_margin_band"]
    assert 0 < lo < hi <= 0.2


@pytest.mark.parametrize("planform", list(PLANFORMS))
def test_generate_each_planform(planform):
    r = client.post("/api/generate", json=_req(planform=planform))
    assert r.status_code == 200, r.text
    d = r.json()["design"]
    assert d["planform"] == planform
    assert "cg_pct_mac" in d["stability"]
    assert 0.03 <= d["stability"]["static_margin"] <= 0.15
    assert d["aero"]["v_stall_ms"] > 0
    assert 5.0 <= d["aero"]["ld_cruise"] <= 24.0
    assert "propulsion" not in d
    ps = d["power_system"]
    assert ps["n_motors"] >= 1 and ps["pack_mass_kg"] > 0
    assert len(ps["motor_positions"]) == ps["n_motors"]
    # the geometry the CAD consumes
    g = d["geometry"]
    for key in ("span_m", "area_m2", "root_chord_m", "tip_chord_m", "taper",
                "sweep_le_deg", "washout_deg", "body", "vstab", "elevons",
                "length_total_m", "height_total_m"):
        assert key in g, f"{planform}: geometry missing {key}"


def test_generate_returns_five_variants(generated):
    variants = generated["variants"]
    assert len(variants) == 5
    ids = set()
    for v in variants:
        assert v["id"] and v["key"] and v["name"] and v["tagline"]
        assert len(v["tagline"]) <= 80
        assert len(v["traits"]) >= 3
        for t in v["traits"]:
            assert t["label"]
            assert 0.05 <= t["value"] <= 1.0
            assert isinstance(t["detail"], str) and t["detail"]
        assert v["preview_stl_url"] == f"/api/preview/{v['id']}.stl"
        assert v["design"]["character"]["key"] == v["key"]
        ids.add(v["id"])
    assert len(ids) == 5
    assert generated["primary_id"] in ids
    primary = next(v for v in variants if v["id"] == generated["primary_id"])
    assert generated["design"]["id"] == generated["primary_id"]
    assert generated["preview_stl_url"] == primary["preview_stl_url"]


def test_variants_are_visibly_different_wings(generated):
    """Five characters, not one wing at five sizes."""
    designs = [v["design"] for v in generated["variants"]]
    assert len({d["planform"] for d in designs}) >= 3
    assert len({round(d["geometry"]["taper"], 2) for d in designs}) >= 3
    assert len({round(d["geometry"]["sweep_le_deg"], 1) for d in designs}) >= 3


def test_every_variant_is_safe(generated):
    for v in generated["variants"]:
        d = v["design"]
        st, a, g = d["stability"], d["aero"], d["geometry"]
        assert st["x_cg_m"] < st["x_np_m"], v["key"]
        assert 0.03 <= st["static_margin"] <= 0.15, v["key"]
        assert a["v_cruise_ms"] >= 1.19 * a["v_stall_ms"], v["key"]
        assert g["span_m"] <= SMALL_BOX["width_mm"] / 1000 + 1e-6, v["key"]
        assert g["length_total_m"] <= SMALL_BOX["length_mm"] / 1000 + 1e-6, v["key"]
        assert g["height_total_m"] <= SMALL_BOX["height_mm"] / 1000 + 1e-6, v["key"]


def test_lazy_preview_for_every_variant(generated):
    for v in generated["variants"]:
        r = client.get(v["preview_stl_url"])
        assert r.status_code == 200, (v["key"], r.text)
        assert len(r.content) > 10_000, v["key"]
        again = client.get(v["preview_stl_url"])
        assert again.status_code == 200
        assert len(again.content) == len(r.content)


def test_preview_is_concurrency_safe(generated):
    """The frontend prefetches the non-primary previews in parallel."""
    urls = [v["preview_stl_url"] for v in generated["variants"]]
    with cf.ThreadPoolExecutor(max_workers=5) as pool:
        results = list(pool.map(lambda u: client.get(u), urls))
    for r in results:
        assert r.status_code == 200, r.text
        assert len(r.content) > 10_000


def test_preview_unknown_id_404():
    r = client.get("/api/preview/deadbeefcafe.stl")
    assert r.status_code == 404
    detail = r.json()["detail"].lower()
    assert "deadbeefcafe" in detail and "generate" in detail


def test_export_step_and_stl(generated):
    did = generated["primary_id"]
    for fmt, sig in (("step", b"ISO-10303"), ("stl", None)):
        e = client.post("/api/export", json={"design_id": did, "format": fmt})
        assert e.status_code == 200, e.text
        assert len(e.content) > 10_000
        if sig:
            assert sig in e.content[:100]


def test_export_works_for_a_non_primary_variant(generated):
    other = next(v for v in generated["variants"]
                 if v["id"] != generated["primary_id"])
    e = client.post("/api/export", json={"design_id": other["id"], "format": "stl"})
    assert e.status_code == 200, e.text
    assert len(e.content) > 10_000


def test_rejected_planform_is_a_readable_422():
    r = client.post("/api/generate", json=_req(planform="conventional"))
    assert r.status_code == 422


def test_planform_any_lets_the_variants_choose():
    """The sidebar's default state. 'any' must not 422, and it must produce a
    gallery that actually spans families rather than five of one shape."""
    r = client.post("/api/generate", json=_req(planform="any"))
    assert r.status_code == 200, r.text
    body = r.json()
    assert len({v["design"]["planform"] for v in body["variants"]}) >= 3


@pytest.mark.parametrize("vstab", ["winglets", "twin_fin", "center_fin"])
def test_vstab_wire_values_are_accepted(vstab):
    r = client.post("/api/generate", json=_req(planform="swept", vstab=vstab))
    assert r.status_code == 200, r.text
    assert r.json()["design"]["geometry"]["vstab"]["type"] == vstab


def test_unknown_design_export_404():
    r = client.post("/api/export", json={"design_id": "nope", "format": "stl"})
    assert r.status_code == 404
    assert "generate" in r.json()["detail"].lower()


# ---------------------------------------------------------------------------
# Expert guidance + Learn knowledge base
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("planform", ["swept", "bell"])
def test_generate_includes_guidance(planform):
    r = client.post("/api/generate", json=_req(planform=planform))
    assert r.status_code == 200, r.text
    d = r.json()["design"]
    guidance = d.get("guidance")
    assert isinstance(guidance, list) and len(guidance) > 0
    for sec in guidance:
        assert isinstance(sec["title"], str) and sec["title"]
        assert isinstance(sec["body"], str) and sec["body"]
    text = " ".join(s["title"] + " " + s["body"] for s in guidance)
    assert [b for b in (s["body"] for s in guidance) if "CG" in b]
    lowered = text.lower()
    # No power-system SIZING and no tailed-aircraft concepts. Naming the pack's
    # constituents ("motor, ESC and flight pack together are ~24% of all-up
    # weight") is the lumped mass allowance the app does model, and is exactly
    # the sort of thing a builder needs - it is specifying hardware that is out
    # of scope.
    for banned in ("kv", "lipo", "mah", "thrust-to-weight", "prop diameter",
                   "horizontal stabilizer", "canard", "ruddervator"):
        assert banned not in lowered, banned


def test_bell_guidance_says_no_fins():
    r = client.post("/api/generate", json=_req(planform="bell"))
    text = " ".join(s["body"] for s in r.json()["design"]["guidance"]).lower()
    assert "proverse" in text
    assert "washout" in text


def test_every_variant_has_its_own_guidance(generated):
    for v in generated["variants"]:
        guidance = v["design"]["guidance"]
        assert isinstance(guidance, list) and guidance
        why = [s for s in guidance if s["title"].startswith("Why this variant")]
        assert len(why) == 1, v["key"]
        assert v["name"] in why[0]["title"]
        assert any(ch.isdigit() for ch in why[0]["body"])


def test_infeasible_design_leads_with_compromises():
    r = client.post("/api/generate", json=_req(
        cruise_speed_ms=28.0,
        box={"length_mm": 260, "width_mm": 320, "height_mm": 110}))
    assert r.status_code == 200, r.text
    body = r.json()
    assert len(body["variants"]) == 5
    d = body["design"]
    assert d["feasible"] is False
    guidance = d["guidance"]
    assert guidance, "infeasible design must still carry guidance"
    assert "compromise" in guidance[0]["title"].lower()


def test_learn_articles_are_flying_wing_material():
    r = client.get("/api/learn")
    assert r.status_code == 200
    articles = r.json()["articles"]
    assert len(articles) == len(LEARN_ARTICLES)
    assert 8 <= len(articles) <= 10
    for a in articles:
        for fld in ("title", "summary", "body"):
            assert isinstance(a[fld], str) and a[fld], f"missing {fld}"
    blob = " ".join(a["title"] + " " + a["body"] for a in articles).lower()
    for topic in ("reflex", "elevon", "washout", "sweep", "static margin"):
        assert topic in blob, f"knowledge base never covers {topic}"
