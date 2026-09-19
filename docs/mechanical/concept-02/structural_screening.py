"""Reproducible analytical screening, not FEA or structural certification.

Units: N, mm, MPa. Beam models intentionally expose their idealized supports.
Uses mechanical-audit.json when available; regenerate that independent audit
after any change to the four CNC parts. Otherwise uses export volume estimates.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

OUT = Path(__file__).resolve().parent
E = 68300.0  # Kaiser 6061 typical elastic modulus, MPa.
RHO = 2.7e-6  # Assumed 6061 density, kg/mm3.
G = 9.80665
P = 22 * G  # 20kg payload plus 2kg moving-assembly design allowance.


def cantilever(width, thickness, length, force, probe=None):
    inertia = width * thickness**3 / 12
    x = length if probe is None else probe
    return {
        "width_mm": width,
        "thickness_mm": thickness,
        "length_mm": length,
        "force_N": force,
        "root_moment_Nm": force * length / 1000,
        "root_stress_MPa": force * length * thickness / (2 * inertia),
        "tip_deflection_mm": force * length**3 / (3 * E * inertia),
        "probe_x_mm": x,
        "probe_deflection_mm": force * x*x * (3*length-x) / (6*E*inertia),
    }


def beam_with_eccentric_cell(span, support_left, attachment, load_coordinate, width, thickness):
    """Simply supported full-width beam; cell contributes point force AND couple."""
    a = attachment - support_left
    couple = P * (load_coordinate - attachment)  # Clockwise applied couple.
    right_reaction = (P * a + couple) / span
    left_reaction = P - right_reaction
    inertia = width * thickness**3 / 12
    integration_c = -(left_reaction*span**3/6 - P*(span-a)**3/6 + couple*(span-a)**2/2) / span
    ys = []
    moments = []
    for i in range(10001):
        x = span*i/10000
        positive = max(x-a, 0)
        y = (left_reaction*x**3/6 - P*positive**3/6 + couple*positive**2/2 + integration_c*x)/(E*inertia)
        moment = left_reaction*x - P*positive + (couple if x >= a else 0)
        ys.append((abs(y), x, y))
        moments.append(abs(moment))
    moments += [abs(left_reaction*a), abs(left_reaction*a+couple)]
    largest = max(ys)
    return {
        "span_mm": span,
        "attachment_a_mm": a,
        "full_beam_width_mm": width,
        "thickness_mm": thickness,
        "force_N": P,
        "applied_couple_Nm": couple/1000,
        "left_reaction_N": left_reaction,
        "right_reaction_N": right_reaction,
        "max_moment_Nm": max(moments)/1000,
        "max_stress_MPa": max(moments)*thickness/(2*inertia),
        "max_absolute_deflection_mm": largest[0],
        "deflection_location_relative_to_left_support_mm": largest[1],
        "note": "Continuous support lines, not four point feet. Negative reaction omits stabilizing dead weight."
    }


def main():
    data = {
        "status": "ANALYTICAL_SCREENING_NOT_FEA_NOT_CAPACITY_CERTIFICATION",
        "material_assumptions": {"E_MPa": E, "density_kg_per_mm3": RHO, "yield_typical_MPa": 276},
        "gravity_m_per_s2": G,
        "structural_force_N": P,
        "equivalent_structure_load_kg": 22,
        "carrier_ideal_fixed_strip": {
            "X_full_width": cantilever(196, 8, 128, P, 100),
            "X_contact_width": cantilever(30, 8, 128, P, 100),
            "Y_full_width": cantilever(276, 8, 85, P, 55),
            "Y_contact_width": cantilever(25, 8, 85, P, 55),
        },
        "base_full_width_line_supported_beam": {
            "X_load_platform_center": beam_with_eccentric_cell(264, -132, -106, 0, 320, 6),
            "X_load_right_edge": beam_with_eccentric_cell(264, -132, -106, 140, 320, 6),
            "Y_load_rear_edge": beam_with_eccentric_cell(284, -142, 50, 150, 300, 6),
        },
        "cell_elastic_translation_linear_assumption_mm_at_22kg": [0.6*22/30, 0.8*22/30],
        "fastener_engagement_mm": {"fixed_M6x25": 25-6-6-1.6, "moving_M6x45":45-8-22, "stop_M6x60":6},
    }
    # Actual rounded pan outline against the hull of four complete circular pads.
    points = []
    for cx, cy, start in [(120,130,0),(-120,130,90),(-120,-30,180),(120,-30,270)]:
        for i in range(901):
            a = math.radians(start+i/10)
            points.append((cx+20*math.cos(a),cy+20*math.sin(a)))
    distance_outside_centers = [math.hypot(max(abs(x)-132,0),max(abs(y)-142,0)) for x,y in points]
    data["stability"] = {
        "foot_centers": [[x,y] for x in (-132,132) for y in (-142,142)],
        "assumed_contact_radius_mm": 15,
        "actual_rounded_pan_margin_mm": 15-max(distance_outside_centers),
        "bounding_rectangle_corner_margin_mm": 15-math.sqrt(8**2+8**2),
        "conditions": "Coplanar feet, complete circular pad contact, vertical static loads, no push/impact."
    }
    verification_path = OUT / "verification.json"
    audit_path = OUT / "mechanical-audit.json"
    audit = json.loads(audit_path.read_text(encoding="utf-8-sig")) if audit_path.exists() else None
    exact_audit = audit is not None
    if verification_path.exists() or exact_audit:
        verified = audit if exact_audit else json.loads(verification_path.read_text(encoding="utf-8-sig"))
        metal = []
        for name in ("base_aluminium","plate_carrier_aluminium","cell_fixed_spacer","cell_moving_spacer"):
            stats = verified["parts"][name]
            mass = stats["volume_mm3"]*RHO
            if exact_audit:
                x, y = stats["centroid_mm"][:2]
            else:
                # Envelope center is only a fallback estimate.
                lo, hi = stats["bounds_mm"]
                x, y = (lo[0]+hi[0])/2, (lo[1]+hi[1])/2
            metal.append({"name":name,"mass_kg":mass,"centroid_xy_mm":[x,y]})
        m = sum(p["mass_kg"] for p in metal)
        mx = sum(p["mass_kg"]*p["centroid_xy_mm"][0] for p in metal)
        my = sum(p["mass_kg"]*p["centroid_xy_mm"][1] for p in metal)
        data["stability"]["cad_metal_mass_only"] = metal
        data["stability"]["cad_metal_mass_total_kg"] = m
        data["stability"]["metal_centroid_xy_mm"] = [mx/m,my/m]
        data["stability"]["centroid_source"] = "CAD center of mass from mechanical-audit snapshot" if exact_audit else "Envelope center approximation"
        if exact_audit:
            data["stability"]["audited_builder_sha256"] = audit["builder_sha256"]
        cases = []
        for label,x,y in [("right_edge",140,50),("rear_edge",0,150),("bounding_box_corner",140,150)]:
            total = m+20
            rx, ry = (mx+20*x)/total,(my+20*y)/total
            cases.append({"label":label,"payload_xy_mm":[x,y],"resultant_xy_mm":[rx,ry],
                          "margin_to_point_foot_rectangle_x_mm":132-abs(rx),
                          "margin_to_point_foot_rectangle_y_mm":142-abs(ry)})
        data["stability"]["twenty_kg_plus_metal_only"] = cases
        data["stability"]["point_foot_model_rear_restoring_to_payload_overturning_moment_ratio"] = (m*142-my)/(20*8)
        data["stability"]["point_foot_model_right_restoring_to_payload_overturning_moment_ratio"] = (m*132-mx)/(20*8)
        data["stability"]["mass_note"] = "Only four custom aluminium parts; assumed density. Not a complete measured mass/CG."
    (OUT/"structural-screening.json").write_text(json.dumps(data,indent=2),encoding="utf-8")
    print(json.dumps(data,indent=2))


if __name__ == "__main__":
    main()
