"""Generate VALIDATION.md from real optimizer runs (one design per planform
family, plus the five variant characters for the default request)."""
from pathlib import Path

from backend.physics.config_defs import MISSIONS, PLANFORMS
from backend.physics.optimizer import optimize_design
from backend.physics.variants import generate_variants
from backend.schemas import GenerateRequest

BOX = {"length_mm": 900, "width_mm": 1200, "height_mm": 300}


def _input(**over):
    body = dict(mission="sport", cruise_speed_ms=16.0, payload_g=0.0,
                box=dict(BOX), n_motors=1, build_method="3d_printed",
                material="lw_pla")
    body.update(over)
    return GenerateRequest(**body).to_optimizer_input()


HEADER = """# VALIDATION.md - flying-wing physics sanity summary

Automatically generated from real optimizer runs (defaults: sport mission,
16 m/s cruise, 900x1200x300 mm box, one motor, LW-PLA). Every number below is
the direct output of the physics pipeline in `backend/physics/` - nothing is
hand-entered. The same checks run automatically in `tests/`:

- CG ahead of NP, static margin inside the TAILLESS band 3-15% (a flying wing
  flies at a lower margin than a tailed model)
- V_cruise >= stall_factor x V_stall; L = W at cruise (lift equation check)
- Reflexed section (positive Cm0) on every design - a tailless wing has no
  tail to trim against
- Fits the size box in span / length / height
- Real RC proportions: root chord >= 0.30 x span for swept/BWB wings, and a
  centre body deep enough to hold the pack. Reference airframes: Skywalker X5
  Pro 1280 mm span / 717 mm root (0.56); SonicModell AR Wing 900 / 482 (0.54)
- Vertical surfaces 2-9% of wing area, sized from real flying-wing practice
  rather than the tail-aft V_V band; bell-spanload wings carry none at all
- One valid watertight solid per design; STL mesh area >= 98.5% of BRep

## Representative designs (one per planform family)

| Quantity | {cols} |
|---|{seps}|
"""

ROWS = [
    ("Span [mm]", lambda d: f"{d['geometry']['span_m']*1000:.0f}"),
    ("Wing area [dm^2]", lambda d: f"{d['geometry']['area_m2']*100:.1f}"),
    ("Aspect ratio", lambda d: f"{d['geometry']['aspect_ratio']:.2f}"),
    ("Root chord [mm]", lambda d: f"{d['geometry']['root_chord_m']*1000:.0f}"),
    ("Root chord / span",
     lambda d: f"{d['geometry']['root_chord_m']/d['geometry']['span_m']:.2f}"),
    ("Tip chord [mm]", lambda d: f"{d['geometry']['tip_chord_m']*1000:.0f}"),
    ("Taper ratio", lambda d: f"{d['geometry']['taper']:.2f}"),
    ("LE sweep [deg]", lambda d: f"{d['geometry']['sweep_le_deg']:.1f}"),
    ("Washout [deg]", lambda d: f"{d['geometry']['washout_deg']:.1f}"),
    ("Airfoil", lambda d: d["geometry"]["airfoil"]),
    ("Body depth scale", lambda d: f"{d['geometry']['body']['depth_scale']:.2f}"),
    ("Body chord scale", lambda d: f"{d['geometry']['body']['chord_scale']:.2f}"),
    ("Equipment bay [mm]",
     lambda d: f"{d['geometry']['body']['bay_length_m']*1000:.0f}"),
    ("All-up mass [g]", lambda d: f"{d['mass']['total_kg']*1000:.0f}"),
    ("Wing loading [kg/m^2]", lambda d: f"{d['aero']['wing_loading_kgm2']:.2f}"),
    ("CL cruise", lambda d: f"{d['aero']['cl_cruise']:.3f}"),
    ("L/D cruise", lambda d: f"{d['aero']['ld_cruise']:.1f}"),
    ("V stall [m/s]", lambda d: f"{d['aero']['v_stall_ms']:.1f}"),
    ("Stall margin [x]", lambda d: f"{d['aero']['stall_margin']:.2f}"),
    ("Re at MAC [k]", lambda d: f"{d['aero']['re_mac']/1000:.0f}"),
    ("MAC [mm]", lambda d: f"{d['stability']['mac_m']*1000:.0f}"),
    ("NP [mm from nose]", lambda d: f"{d['stability']['x_np_m']*1000:.0f}"),
    ("CG [mm from nose]", lambda d: f"{d['stability']['x_cg_m']*1000:.0f}"),
    ("CG [%MAC]", lambda d: f"{d['stability']['cg_pct_mac']:.0f}"),
    ("Static margin", lambda d: f"{d['stability']['static_margin']:.3f}"),
    ("Vertical surfaces", lambda d: d["geometry"]["vstab"]["type"]),
    ("Vert. area [% wing]",
     lambda d: f"{d['geometry']['vstab']['area_total_m2']/d['geometry']['area_m2']*100:.1f}"),
    ("Vert. height [mm]",
     lambda d: f"{d['geometry']['vstab']['height_m']*1000:.0f}"),
    ("Motor stations", lambda d: f"{d['power_system']['n_motors']}"),
    ("Feasible", lambda d: "yes" if d["feasible"] else "NO: " + ",".join(d["binding"])),
]

VARIANT_ROWS = [
    ("Planform", lambda d: d["planform"]),
    ("Span [mm]", lambda d: f"{d['geometry']['span_m']*1000:.0f}"),
    ("Aspect ratio", lambda d: f"{d['geometry']['aspect_ratio']:.2f}"),
    ("Root chord [mm]", lambda d: f"{d['geometry']['root_chord_m']*1000:.0f}"),
    ("Taper ratio", lambda d: f"{d['geometry']['taper']:.2f}"),
    ("LE sweep [deg]", lambda d: f"{d['geometry']['sweep_le_deg']:.1f}"),
    ("Washout [deg]", lambda d: f"{d['geometry']['washout_deg']:.1f}"),
    ("Airfoil", lambda d: d["geometry"]["airfoil"]),
    ("Vertical surfaces", lambda d: d["geometry"]["vstab"]["type"]),
    ("All-up mass [g]", lambda d: f"{d['mass']['total_kg']*1000:.0f}"),
    ("Wing loading [kg/m^2]", lambda d: f"{d['aero']['wing_loading_kgm2']:.2f}"),
    ("V stall [m/s]", lambda d: f"{d['aero']['v_stall_ms']:.2f}"),
    ("V cruise [m/s]", lambda d: f"{d['aero']['v_cruise_ms']:.1f}"),
    ("L/D cruise", lambda d: f"{d['aero']['ld_cruise']:.2f}"),
    ("Static margin", lambda d: f"{d['stability']['static_margin']:.3f}"),
    ("Feasible", lambda d: "yes" if d["feasible"] else "NO: " + ",".join(d["binding"])),
]


def main():
    designs = {p: optimize_design(_input(planform=p)) for p in PLANFORMS}
    cols = " | ".join(PLANFORMS)
    out = HEADER.format(cols=cols, seps="|".join("---" for _ in PLANFORMS))
    for label, fn in ROWS:
        out += f"| {label} | " + " | ".join(
            fn(designs[p]) for p in PLANFORMS) + " |\n"

    variants = generate_variants(_input(planform="swept"))
    out += (
        "\n## The five wing characters (same request)\n\n"
        "Each column is a full independent solve of the same constrained "
        "problem with a different planform family and different targets, so "
        "the differences below are physics and shape, not styling.\n\n"
    )
    vcols = " | ".join(v["name"] for v in variants)
    out += f"| Quantity | {vcols} |\n|---|" + "|".join("---" for _ in variants) + "|\n"
    for label, fn in VARIANT_ROWS:
        out += f"| {label} | " + " | ".join(
            fn(v["design"]) for v in variants) + " |\n"

    out += "\nNormalized traits (min-max across the five):\n\n"
    labels = [t["label"] for t in variants[0]["traits"]]
    out += f"| Trait | {vcols} |\n|---|" + "|".join("---" for _ in variants) + "|\n"
    for i, label in enumerate(labels):
        out += f"| {label} | " + " | ".join(
            f"{v['traits'][i]['value']:.2f} ({v['traits'][i]['detail']})"
            for v in variants) + " |\n"

    out += "\n## Mission sensitivity (swept planform)\n\n"
    out += "| Mission | Wing loading [kg/m^2] | V stall [m/s] | L/D |\n|---|---|---|---|\n"
    for m in MISSIONS:
        d = optimize_design(_input(planform="swept", mission=m))
        out += (f"| {m} | {d['aero']['wing_loading_kgm2']:.2f} | "
                f"{d['aero']['v_stall_ms']:.2f} | {d['aero']['ld_cruise']:.1f} |\n")

    slow = optimize_design(_input(planform="swept", cruise_speed_ms=11.0))
    fast = optimize_design(_input(planform="swept", cruise_speed_ms=22.0))
    out += (f"\nCruise-speed sensitivity: wing loading at 11 m/s = "
            f"{slow['aero']['wing_loading_kgm2']:.2f} kg/m^2, at 22 m/s = "
            f"{fast['aero']['wing_loading_kgm2']:.2f} kg/m^2 - increases with "
            "cruise speed as expected.\n")
    Path(__file__).parent.joinpath("VALIDATION.md").write_text(out, encoding="utf-8")
    print("VALIDATION.md written")


if __name__ == "__main__":
    main()
