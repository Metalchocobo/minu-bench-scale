"""Parametric visual enclosure study, in millimetres (CadQuery 2.8).

This is a printable FORM prototype, not a released mechanical design. No load
capacity, cell compatibility, fastener detail, overtravel stop or assembly fit
is claimed. The three printable solids are intentionally separate. The plate
must ultimately transfer its load through a selected cell, never the cover.

Install dependencies outside the repository:
    python -m pip install cadquery==2.8.0 trimesh
Run:
    python generate_concept.py

Coordinates: X width, Y depth (negative = front), Z up. All CAD and meshes use
the assembled coordinate frame; slicers may place each part on their build bed.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from itertools import combinations
from pathlib import Path
import json
import math
import os
import sys

import cadquery as cq
import trimesh


@dataclass(frozen=True)
class Dimensions:
    width: float = 280.0
    depth: float = 300.0
    height: float = 110.0
    corner_radius: float = 20.0
    wall: float = 3.2
    base_width: float = 272.0
    base_depth: float = 292.0
    base_floor_z: float = 8.0
    base_floor_thickness: float = 6.0
    shell_bottom: float = 16.8
    shell_front_roof: float = 46.0
    shell_rear_roof: float = 71.0
    slope_end_y: float = -50.0
    plate_depth: float = 200.0
    plate_center_y: float = 50.0
    plate_skin: float = 6.0
    plate_rib_height: float = 30.0
    plate_rib_thickness: float = 6.0
    base_rib_thickness: float = 4.0
    base_rib_height: float = 25.0


D = Dimensions()
OUT = Path(__file__).resolve().parent


def rounded_prism(width, depth, radius, z, height, x=0, y=0):
    """A rounded rectangle extruded in Z, with constant, explicit wall geometry."""
    return (cq.Workplane("XY").box(width, depth, height,
                                  centered=(True, True, False))
            .edges("|Z").fillet(radius).translate((x, y, z)))


def block(width, depth, z, height, x=0, y=0):
    return (cq.Workplane("XY").box(width, depth, height,
                                  centered=(True, True, False))
            .translate((x, y, z)))


def rib_between(a, b, width, z, height):
    """In-plane rib with rectangular section; coordinates are two XY points."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    rib = block(length, width, z, height)
    return rib.rotate((0, 0, 0), (0, 0, 1), math.degrees(math.atan2(dy, dx))).translate(
        ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, 0))


def roof_clip(offset=0.0):
    """Piecewise planar envelope: a smooth blank sloped front and rear deck."""
    y0, y1 = -D.depth / 2, D.slope_end_y
    rise = (D.shell_rear_roof - D.shell_front_roof) / (y1 - y0)
    extended_front = D.shell_front_roof - rise * 10
    points = [(y0 - 10, 0), (D.depth / 2 + 10, 0),
              (D.depth / 2 + 10, D.shell_rear_roof - offset),
              (y1, D.shell_rear_roof - offset),
              (y0 - 10, extended_front - offset)]
    return cq.Workplane("YZ").polyline(points).close().extrude(D.width, both=True)


def build_shell():
    outer = rounded_prism(D.width, D.depth, D.corner_radius,
                          D.shell_bottom, D.shell_rear_roof - D.shell_bottom)
    outer = outer.intersect(roof_clip())
    inner = rounded_prism(D.width - 2 * D.wall, D.depth - 2 * D.wall,
                          D.corner_radius - D.wall, 0, D.height + 10)
    inner = inner.intersect(roof_clip(D.wall))
    shell = outer.cut(inner)
    # This large service/open movement aperture is NOT a selected cell interface.
    opening = rounded_prism(244, 174, 14, 65, 50, y=D.plate_center_y)
    return shell.cut(opening).clean()


def build_base():
    z = D.base_floor_z + D.base_floor_thickness
    base = rounded_prism(D.base_width, D.base_depth, 18,
                         D.base_floor_z, D.base_floor_thickness)
    perimeter = rounded_prism(D.base_width, D.base_depth, 18, z, 7)
    perimeter = perimeter.cut(rounded_prism(D.base_width - 6.4,
                                           D.base_depth - 6.4, 14.8, z - 1, 9))
    base = base.union(perimeter)
    # Main rails reach the illustrative feet; the surrounding cover is cosmetic.
    for x in (-103, 103):
        base = base.union(block(D.base_rib_thickness, 263, z,
                                D.base_rib_height, x=x))
    for y in (-114, 114):
        base = base.union(block(255, D.base_rib_thickness, z,
                                D.base_rib_height, y=y))
    # Fixed load island is a PLACEHOLDER. No mounting holes are invented here.
    # A hollow pedestal and four radial ribs illustrate the intended load path.
    pedestal = block(56, 44, 39, 5, y=-36)
    for x in (-24, 24):
        for y in (-54, -18):
            pedestal = pedestal.union(block(8, 8, z, 39 - z, x=x, y=y))
    pedestal = pedestal.union(block(4, 44, z, 39 - z, y=-36))
    base = base.union(pedestal)
    for sx in (-1, 1):
        for sy in (-1, 1):
            base = base.union(rib_between((sx * 20, -36 + sy * 14),
                                          (sx * 103, sy * 114),
                                          D.base_rib_thickness, z,
                                          D.base_rib_height))
    return base.clean()


def build_plate():
    skin_z = D.height - D.plate_skin
    plate = rounded_prism(D.width, D.plate_depth, D.corner_radius,
                          skin_z, D.plate_skin, y=D.plate_center_y)
    rib_z = skin_z - D.plate_rib_height
    perimeter = rounded_prism(D.width - 4, D.plate_depth - 4, 18,
                              rib_z + 1, D.plate_rib_height - 1,
                              y=D.plate_center_y)
    inner = rounded_prism(D.width - 12, D.plate_depth - 12, 14,
                          rib_z, D.plate_rib_height + 1,
                          y=D.plate_center_y)
    plate = plate.union(perimeter.cut(inner))
    rib_limit = rounded_prism(D.width - 4, D.plate_depth - 4, 18,
                              rib_z, D.plate_rib_height,
                              y=D.plate_center_y)
    for x in (-100, -60, -20, 20, 60, 100):
        rib = block(D.plate_rib_thickness, D.plate_depth - 6,
                    rib_z, D.plate_rib_height, x=x, y=D.plate_center_y)
        plate = plate.union(rib.intersect(rib_limit))
    for y in (-10, 30, 70, 110):
        rib = block(D.width - 8, D.plate_rib_thickness,
                    rib_z, D.plate_rib_height, y=y)
        plate = plate.union(rib.intersect(rib_limit))
    # Moving load island, also a placeholder, is part of the floating plate.
    # It is spatially separated from the fixed pedestal and from the cover.
    moving_island = block(80, 64, 70, 6, y=50)
    for x in (-36, 36):
        for y in (22, 78):
            moving_island = moving_island.union(block(8, 8, 76,
                                                      skin_z - 76, x=x, y=y))
    for x in (-20, 20):
        moving_island = moving_island.union(block(6, 64, 76,
                                                  skin_z - 76, x=x, y=50))
    # Internal powder/drainage relief only; this is not a cell mounting hole.
    relief = cq.Workplane("XY").center(0, 50).circle(12).extrude(77).translate((0, 0, -1))
    return plate.union(moving_island).cut(relief).clean()


def shape_stats(shape):
    s = shape.val()
    bb = s.BoundingBox()
    return {
        "brep_valid": s.isValid(), "solid_count": len(s.Solids()),
        "volume_mm3": round(s.Volume(), 3),
        "bounds_mm": [[round(bb.xmin, 3), round(bb.ymin, 3), round(bb.zmin, 3)],
                      [round(bb.xmax, 3), round(bb.ymax, 3), round(bb.zmax, 3)]],
        "size_mm": [round(bb.xlen, 3), round(bb.ylen, 3), round(bb.zlen, 3)],
    }


def mesh_preview(name, label, shape, color, printable):
    vertices, faces = shape.val().tessellate(0.5, 0.2)
    return {"name": name, "label": label, "color": color,
            "printable_form_prototype": printable,
            "vertices": [[round(v.x, 3), round(v.y, 3), round(v.z, 3)] for v in vertices],
            "faces": [list(f) for f in faces]}


def main():
    stl_dir = OUT / "stl-forma"
    step_dir = OUT / "step"
    stl_dir.mkdir(exist_ok=True)
    step_dir.mkdir(exist_ok=True)
    parts = {
        "base": ("Basamento nervato", build_base(), "#26332f"),
        "guscio": ("Scocca con frontale inclinato", build_shell(), "#738375"),
        "piatto": ("Piatto indipendente nervato", build_plate(), "#e5e6df"),
    }
    summary = {
        "status": "CONCEPT VOLUMETRICO - prototipo di forma; montaggio e portata non validati",
        "units": "mm", "parameters": asdict(D), "parts": {},
        "target_load_kg": 20, "load_capacity_verified": False,
        "cell_reference_is_selected_hardware": False,
        "assembly_ready": False, "fastener_and_cell_interfaces_defined": False,
        "minimum_nominal_plate_rib_to_shell_roof_gap_mm": 3,
        "notes": [
            "I tre STL sono solidi di forma separati, non un insieme montabile definitivo.",
            "20 kg e un obiettivo di progetto, non una portata dimostrata dal CAD.",
            "Gli ingombri cella e piedi sono riferimenti esclusi dagli STL.",
            "Nessuna selezione materiale, tolleranza, fissaggio o finecorsa di sovraccarico e validata.",
            "Le quote sono assolute nell'assetto assemblato; gli STL non codificano unita, usare millimetri.",
        ],
    }
    assembly = cq.Assembly(name="MINU_concept_01_FORM_ONLY")
    preview = {"units": "mm", "axes": {"x": "width", "y": "depth; front negative", "z": "up"},
               "bounds_mm": [[-140, -150, 0], [140, 150, 110]], "parts": []}
    for name, (label, shape, color) in parts.items():
        stats = shape_stats(shape)
        assert stats["brep_valid"] and stats["solid_count"] == 1, (name, stats)
        stl_path = stl_dir / (name + "_FORMA_NON_MONTAGGIO.stl")
        cq.exporters.export(shape, str(stl_path), tolerance=0.12, angularTolerance=0.12)
        cq.exporters.export(shape, str(step_dir / (name + ".step")))
        mesh = trimesh.load_mesh(stl_path, process=True)
        stats.update({"stl_watertight": bool(mesh.is_watertight),
                      "stl_winding_consistent": bool(mesh.is_winding_consistent),
                      "stl_components": len(mesh.split()),
                      "stl_triangles": len(mesh.faces)})
        assert stats["stl_watertight"] and stats["stl_winding_consistent"]
        assert stats["stl_components"] == 1
        summary["parts"][name] = stats
        assembly.add(shape, name=name, color=cq.Color(color))
        preview["parts"].append(mesh_preview(name, label, shape, color, True))
    # This plain translucent box represents a RESERVED VOLUME, not a real cell.
    cell = block(38, 100, 45, 22, y=10)
    assembly.add(cell, name="REFERENCE_ONLY_reserved_cell_volume",
                 color=cq.Color(0.7, 0.8, 0.85, 0.45))
    preview["parts"].append(mesh_preview("cella", "Volume cella indicativo", cell, "#acbcc6", False))
    summary["cell_reserved_volume_mm"] = shape_stats(cell)
    for i, (x, y) in enumerate(((-103, -114), (103, -114), (-103, 114), (103, 114)), 1):
        foot = cq.Workplane("XY").circle(13.5).extrude(8).translate((x, y, 0))
        assembly.add(foot, name=f"REFERENCE_ONLY_foot_{i}", color=cq.Color("#171b19"))
        preview["parts"].append(mesh_preview(f"piede_{i}", "Piedino indicativo", foot, "#171b19", False))
    interference = {}
    for a, b in combinations(parts, 2):
        overlap = parts[a][1].val().intersect(parts[b][1].val()).Volume()
        interference[f"{a}/{b}"] = round(overlap, 6)
        assert overlap < 0.001, (a, b, overlap)
    summary["plastic_part_intersections_mm3"] = interference
    cell_interference = {}
    for name, (_, shape, _) in parts.items():
        overlap = shape.val().intersect(cell.val()).Volume()
        cell_interference[name] = round(overlap, 6)
        assert overlap < 0.001, ("reserved_cell", name, overlap)
    summary["reserved_cell_intersections_mm3"] = cell_interference
    # Assembly retains separate names and solids; the reference-only objects are named explicitly.
    assembly.export(str(step_dir / "MINU_concept_01_assembly_WITH_REFERENCES.step"))
    summary["step_roundtrip"] = {}
    for step_path in sorted(step_dir.glob("*.step")):
        imported = cq.importers.importStep(str(step_path))
        solid_count = len(imported.solids().vals())
        expected = 8 if "assembly" in step_path.name else 1
        assert imported.val().isValid() and solid_count == expected
        summary["step_roundtrip"][step_path.name] = {
            "brep_valid": True, "solid_count": solid_count}
    (OUT / "verification.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (OUT / "preview-mesh.json").write_text(json.dumps(preview, separators=(",", ":"), ensure_ascii=False), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False))
    print(f"Preview bytes: {(OUT / 'preview-mesh.json').stat().st_size}")
    if sys.platform == "win32":
        # OCP 7.9.3.1.1 on this Windows runtime crashes during interpreter
        # finalization after successful STEPCAF export. All files are already
        # closed and all assertions passed. Skip only the native finalizers in
        # this stand-alone batch process; exceptions above are never bypassed.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()




