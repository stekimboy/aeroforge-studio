"""Pydantic request/response models for the API (spec section 7).

AeroForge designs FLYING WINGS ONLY: the request carries a PLANFORM family and
a MISSION where v1 carried an aircraft configuration and a flight style.

UI units: box in mm, speeds in m/s, payload in grams. The optimizer works in
SI (m, kg), so `to_optimizer_input` does the conversion in one place.
"""
from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field, field_validator

# "any" is the sidebar's default: the user has not committed to a family, so
# the five characters span several of them and the gallery becomes the choice.
# It is normalized away in `to_optimizer_input` - the optimizer itself only
# ever sees a concrete family.
PlanformName = Literal["any", "swept", "bwb", "plank", "bell"]
MissionName = Literal["sport", "fpv_cruiser", "thermal_floater", "park_flyer"]
VstabName = Literal["auto", "winglets", "twin_fin", "center_fin", "none"]
MaterialName = Literal["lw_pla", "pla", "foam"]


class BoxMM(BaseModel):
    """Hard size constraint: assembled aircraft must fit this box (mm)."""
    length_mm: float = Field(800, gt=100, le=5000)
    width_mm: float = Field(1200, gt=100, le=5000)
    height_mm: float = Field(300, gt=50, le=2000)


class GenerateRequest(BaseModel):
    # v2 (V2_PLAN.md): which airplane TYPE to design. "flying_wing" is the
    # default and preserves v1 behaviour exactly; "conventional" routes to the
    # tailed solver; "any" is a request mode - both types are generated and
    # the best five span both. Validated against the AIRPLANE_TYPES registry.
    airplane_type: str = "flying_wing"
    planform: PlanformName = "any"
    mission: MissionName = "sport"
    cruise_speed_ms: float = Field(16.0, ge=5.0, le=60.0)
    stall_speed_target_ms: Optional[float] = Field(None, ge=3.0, le=30.0)
    payload_g: float = Field(0.0, ge=0.0, le=5000.0)
    box: BoxMM = BoxMM()
    allow_folding: bool = False  # informational; optimizer always fits assembled
    # v3: 0 motors was legal only for the glider, which was removed
    # (builder, 2026-08-21); every current type needs at least one. The
    # validator below now rejects 0 outright.
    n_motors: int = Field(1, ge=0, le=4)
    # v3 (V3_PLAN.md) configuration axes. None = the type's researched
    # default (twin_boom -> pusher, tailless -> no tail_type, etc.); the
    # per-type physics modules consume what applies and ignore the rest.
    motor_layout: Optional[Literal["tractor", "pusher"]] = None
    tail_type: Optional[Literal["conventional", "t_tail", "v_tail"]] = None
    wing_position: Optional[Literal["high", "mid", "low"]] = None
    vstab: VstabName = "auto"
    build_method: Literal["3d_printed", "foamboard"] = "3d_printed"
    material: MaterialName = "lw_pla"
    airfoil_override: Optional[str] = None
    aspect_ratio_target: Optional[float] = Field(None, ge=3.0, le=12.0)
    # tailless static-margin band (spec 3.2) - NOT the tailed 0.04-0.20
    static_margin_override: Optional[float] = Field(None, ge=0.03, le=0.15)

    # ---- motor mount ------------------------------------------------------
    # Left as None the bolt pattern is chosen from all-up mass using the
    # standard RC outrunner patterns (16/19/25 mm square, M3). Set them when
    # you already know which motor is going on it - these numbers are what the
    # CAD drills, so the printed mount matches the motor you actually own.
    mount_screws: Optional[int] = Field(None, ge=2, le=8)
    mount_screw_d_mm: Optional[float] = Field(None, ge=1.5, le=8.0)
    mount_bolt_circle_mm: Optional[float] = Field(None, ge=4.0, le=40.0)

    @field_validator("airplane_type")
    @classmethod
    def _known_airplane_type(cls, v):
        from .physics.config_defs import AIRPLANE_TYPES
        vv = (v or "flying_wing").strip() or "flying_wing"
        if vv != "any" and vv not in AIRPLANE_TYPES:
            raise ValueError(
                f"unknown airplane_type {vv!r}; options: "
                f"{list(AIRPLANE_TYPES)} or 'any'")
        return vv

    @field_validator("n_motors")
    @classmethod
    def _at_least_one_motor(cls, v, info):
        # 0 motors was the pure-sailplane glider's privilege; the glider was
        # removed (builder, 2026-08-21) and no current type flies unpowered.
        if v == 0:
            raise ValueError(
                "n_motors=0 was only valid for the removed 'glider' type; "
                "every current type needs at least one motor")
        return v

    @field_validator("airfoil_override")
    @classmethod
    def _known_airfoil(cls, v, info):
        if v in (None, "", "auto"):
            return None
        from .physics.airfoils import LIBRARY, WING_AIRFOILS
        # v2: a conventional (or "any") request may name a NON-reflexed
        # cambered section - the tail trims there. Importing the conventional
        # module registers those sections; each solver still resolves/ignores
        # an override that is wrong for its own configuration.
        if (info.data.get("airplane_type") or "flying_wing") != "flying_wing":
            from .physics.conventional import CONV_AIRFOILS
            if v not in LIBRARY:
                raise ValueError(
                    f"unknown airfoil {v!r}; options: "
                    f"{list(WING_AIRFOILS) + list(CONV_AIRFOILS)}")
            return v
        if v not in LIBRARY:
            raise ValueError(f"unknown airfoil {v!r}; options: {list(WING_AIRFOILS)}")
        if not LIBRARY[v].reflexed:
            raise ValueError(
                f"{v!r} is not reflexed. A flying wing needs a positive-Cm0 "
                f"reflexed section to trim; options: {list(WING_AIRFOILS)}")
        return v

    def to_optimizer_input(self) -> dict:
        material = "foam" if self.build_method == "foamboard" else self.material
        return {
            # v2: threads through untouched flying-wing code (which never
            # reads it) and dispatches the conventional / "any" solvers
            "airplane_type": self.airplane_type,
            # "any" -> None: the optimizer falls back to its default family and
            # `variants.primary_key_for(None)` lets the reference character lead
            "planform": None if self.planform == "any" else self.planform,
            "mission": self.mission,
            "v_cruise": self.cruise_speed_ms,
            "v_stall_target": self.stall_speed_target_ms,
            "payload_kg": self.payload_g / 1000.0,
            "box_l": self.box.length_mm / 1000.0,
            "box_w": self.box.width_mm / 1000.0,
            "box_h": self.box.height_mm / 1000.0,
            "vstab": self.vstab,
            "n_motors": self.n_motors,
            "material": material,
            "build_method": self.build_method,
            "airfoil_override": self.airfoil_override,
            "ar_target": self.aspect_ratio_target,
            "sm_override": self.static_margin_override,
            "mount_screws": self.mount_screws,
            "mount_screw_d_mm": self.mount_screw_d_mm,
            "mount_bolt_circle_mm": self.mount_bolt_circle_mm,
            # v3 configuration axes: None entries dropped so a bare request
            # is byte-identical to a v2 one and each type applies its own
            # researched defaults.
            "config": {k: v for k, v in {
                "motor_layout": self.motor_layout,
                "tail_type": self.tail_type,
                "wing_position": self.wing_position,
            }.items() if v is not None} or None,
        }


class ExportRequest(BaseModel):
    design_id: str
    # "stl_parts" = a zip of one STL per named part (the hatch lid is a
    # separate piece of plastic on the real aircraft, so it must print alone)
    format: Literal["step", "stl", "stl_parts"]


class GenerateResponse(BaseModel):
    """POST /api/generate returns FIVE flying-wing characters (variants).

    `design` / `preview_stl_url` / `meta` mirror the primary variant - the one
    whose planform family the user asked for - so simple clients can ignore the
    gallery entirely.
    """
    variants: list[dict]
    primary_id: str
    design: dict
    preview_stl_url: str
    meta: dict
