"""Minu concept 02: nominal, serviceable mechanical assembly, millimetres.

Run with CadQuery 2.8 + trimesh. Purchased parts are simplified envelopes with
documented mounting dimensions; threads are nominal cylinders, not helices.
This CAD is NOT a strength, food-contact, charging or metrology certification.
The exact owned display and the printed/service tolerances require validation.
"""
from __future__ import annotations

import csv
import json
import hashlib
import math
import os
from pathlib import Path
import sys
from collections import Counter
from itertools import combinations

import cadquery as cq
import trimesh

OUT = Path(__file__).resolve().parent
WIDTH, DEPTH = 300.0, 320.0
BASE_Z, BASE_T = 13.0, 6.0
ROOF_FRONT, ROOF_BACK = 48.0, 65.0
PLATE_Y, CARRIER_Z, CARRIER_T = 50.0, 69.0, 8.0
PLATE_Z, PLATE_T = 77.0, 4.0
ANGLE = math.degrees(math.atan((ROOF_BACK - ROOF_FRONT) / 110.0))
DISPLAY_X, DISPLAY_Y = -67.0, -112.0
PARTS = []
MATES = []
REFERENCES = {}
CABLE_PATHS = {}
SHELL_FASTENERS = [(x, y) for x in (-142., 142.) for y in (-118., -10., 105.)]
SHELL_FASTENERS += [(-88., -152.), (88., -152.)]
FEET = [(x, y) for x in (-132., 132.) for y in (-142., 142.)]
CELL_FIXED = [(-106., y) for y in (42.5, 57.5)]
CELL_MOVING = [(0., y) for y in (42.5, 57.5)]
PAN_FIX = [(x, y) for x in (-113., 113.) for y in (-24., 50., 124.)]
POWER_FIX = [(x, y) for x in (45., 118.) for y in (22., 88.)]
ELECTRONICS_FIX = [(x, y) for x in (-107., 26.) for y in (-53., -3.)]
CABLE_CLIP_FIX = [(126., -28.), (138., -28.), (16., 18.), (28., 18.)]


def block(w, d, z, h, x=0, y=0):
    return cq.Workplane('XY').box(w, d, h, centered=(True, True, False)).translate((x, y, z))


def rr(w, d, r, z, h, x=0, y=0):
    return block(w, d, z, h).edges('|Z').fillet(r).translate((x, y, 0))


def cylinder(d, z, h, x=0, y=0):
    return cq.Workplane('XY').circle(d / 2).extrude(h).translate((x, y, z))


def ring(od, id_, z, h, x=0, y=0):
    return cq.Workplane('XY').circle(od / 2).circle(id_ / 2).extrude(h).translate((x, y, z))


def hexagon(af, z, h, x=0, y=0):
    return cq.Workplane('XY').polygon(6, af / math.cos(math.pi / 6)).extrude(h).translate((x, y, z))


def hole(shape, pts, dia, z, h):
    cutter = cq.Workplane('XY').pushPoints(pts).circle(dia / 2).extrude(h).translate((0, 0, z))
    return shape.cut(cutter).clean()


def countersink(shape, pts, d, head_d, surface, depth):
    depth = (head_d - d) / 2.0  # True 90 degree included angle.
    for x, y in pts:
        cone = cq.Solid.makeCone(d / 2, head_d / 2, depth,
                                 cq.Vector(x, y, surface - depth), cq.Vector(0, 0, 1))
        shape = shape.cut(cone)
    return shape.clean()


def frame(w, d, iw, id_, z, h, x=0, y=0):
    return block(w, d, z, h, x, y).cut(block(iw, id_, z - 1, h + 2, x, y)).clean()


def plane_height(y):
    return ROOF_FRONT + (y + DEPTH / 2) * (ROOF_BACK - ROOF_FRONT) / 110.


def on_front(shape, x=DISPLAY_X, y=DISPLAY_Y):
    return shape.rotate((0, 0, 0), (1, 0, 0), ANGLE).translate((x, y, plane_height(y)))


def on_right(shape):
    # Local +Z becomes global +X. Local origin is the outside wall surface.
    return shape.rotate((0, 0, 0), (0, 1, 0), 90).translate((150, 102, 40))


def add(name, label, shape, category, material, color, bom, notes='', source='', printable=False):
    s = shape.val() if hasattr(shape, 'val') else shape
    assert s.isValid() and len(s.Solids()) == 1, (name, s.isValid(), len(s.Solids()))
    PARTS.append(dict(name=name, label=label, shape=s, category=category,
                      material=material, color=color, bom=bom, notes=notes,
                      source=source, printable=printable))
    return s


def nut(name, d, af, h, z, x=0, y=0, transform=None, label=None):
    shape = hexagon(af, z, h, x, y).cut(cylinder(d, z - 1, h + 2, x, y))
    if transform:
        shape = transform(shape)
    return add(name, label or f'Dado M{d:g}', shape, 'fastener', 'steel', '#b0b5b8',
               f'NUT_M{d:g}_DIN934', 'Nominal thread; no helical geometry.')


def socket_screw(name, d, length, seat, x=0, y=0, up=True, transform=None):
    hd, hh, key = {3: (5.5, 3., 2.5), 6: (10., 6., 5.)}[d]
    # Positive local Z is insertion direction; seat is the under-head plane.
    sh = cylinder(d, 0, length).union(cylinder(hd, -hh, hh))
    sh = sh.cut(hexagon(key, -hh - .1, hh * .55))
    if not up:
        sh = sh.rotate((0, 0, 0), (1, 0, 0), 180)
    sh = sh.translate((x, y, seat))
    if transform:
        sh = transform(sh)
    add(name, f'Vite M{d} x {length:g} testa cilindrica', sh, 'fastener', 'steel', '#a6afb4',
        f'SCREW_M{d}x{length:g}_ISO4762', 'Simplified thread; nominal shank diameter.')


def csk_screw(name, d, length, top, x=0, y=0, transform=None):
    hd, hh, key = {3: (6., 1.7, 2.), 6: (12., 3.3, 4.)}[d]
    cone_h = (hd - d) / 2.0
    sh = cylinder(d, top - length, length - cone_h, x, y)
    sh = sh.union(cq.Solid.makeCone(d / 2, hd / 2, cone_h,
                                   cq.Vector(x, y, top - cone_h), cq.Vector(0, 0, 1)))
    sh = sh.cut(hexagon(key, top - hh * .6, hh, x, y))
    if transform:
        sh = transform(sh)
    add(name, f'Vite svasata M{d} x {length:g}', sh, 'fastener', 'steel', '#a6afb4',
        f'SCREW_M{d}x{length:g}_DIN7991', 'M6 head D12 x 3.3; nominal simplified threads.')


def build_base():
    s = rr(WIDTH, DEPTH, 18, BASE_Z, BASE_T)
    s = hole(s, SHELL_FASTENERS, 3.4, 12, 8)
    s = hole(s, CELL_FIXED, 6.4, 12, 8)
    s = hole(s, FEET, 6., 12, 8)
    s = hole(s, POWER_FIX, 3., 12, 8)
    s = hole(s, ELECTRONICS_FIX, 3., 12, 8)
    s = hole(s, CABLE_CLIP_FIX, 3., 12, 8)
    # Four independently adjustable overload screws are anchored in metal.
    s = hole(s, [(x, y) for x in (-112., 112.) for y in (-20., 120.)], 6., 12, 8)
    return s


def roof_clip(offset=0):
    p = [(-170., 0), (170., 0), (170., ROOF_BACK - offset),
         (-50., ROOF_BACK - offset), (-170., plane_height(-170.) - offset)]
    return cq.Workplane('YZ').polyline(p).close().extrude(180, both=True)


def build_shell():
    outside = rr(300, 320, 18, 19, 46).intersect(roof_clip())
    inside = rr(294, 314, 15, 18, 70).intersect(roof_clip(3))
    s = outside.cut(inside)
    # A floating spacer passes through this opening; the shell carries no load.
    s = s.cut(rr(49, 54, 5, 58, 15, x=-.5, y=50))
    s = hole(s, [(x, y) for x in (-112., 112.) for y in (-20., 120.)], 8., 58, 12)
    s = s.cut(on_front(block(118, 46, -20, 40)))
    s = s.cut(on_front(cq.Workplane('XY').pushPoints([(x, y) for x in (-65., 65.) for y in (-20., 20.)]).circle(1.7).extrude(30).translate((0, 0, -15))))
    for ky in (-129., -98.):
        # Adhesive keypads need cable slots, not switch holes.
        s = s.cut(on_front(block(16, 6, -8, 14), 77, ky + 10))
        s = s.cut(on_front(rr(71, 21, 1, -.35, 1), 77, ky))
    s = s.cut(on_right(cylinder(12.4, -20, 40)))
    for x, y in SHELL_FASTENERS:
        boss = cylinder(12, 19, 12, x, y)
        boss = hole(boss, [(x, y)], 3.4, 18, 14)
        boss = boss.cut(hexagon(5.7, 26.5, 2.6, x, y))
        # Open lateral slot permits inserting a standard M3 hex nut from inside.
        if abs(x) > 130:
            boss = boss.cut(block(8, 5.7, 26.5, 2.6, x - math.copysign(4., x), y))
        else:
            boss = boss.cut(block(5.7, 8, 26.5, 2.6, x, y + 4))
        s = s.union(boss)
    return s.clean()


def build_carrier():
    s = rr(276, 196, 16, CARRIER_Z, CARRIER_T, y=PLATE_Y)
    s = hole(s, CELL_MOVING, 6.4, 68, 10)
    s = countersink(s, CELL_MOVING, 6.4, 12.4, 77, 3.3)
    s = hole(s, PAN_FIX, 3., 68, 10)
    return s


def build_pan():
    s = rr(280, 200, 20, PLATE_Z, PLATE_T, y=PLATE_Y)
    s = hole(s, PAN_FIX, 3.4, 76, 6)
    return countersink(s, PAN_FIX, 3.4, 6.4, 81, 1.7)


def build_front():
    # Replaceable universal bezel: no unverified PCB drilling pattern is used.
    bezel = rr(142, 58, 5, 0, 3)
    bezel = bezel.cut(block(80, 22, -1, 6))
    bezel = bezel.cut(block(84, 26, -.1, 1.7))
    support = frame(116, 44, 101.5, 34.5, -11.3, 11.3)
    for x in (-54.5, 54.5):
        for y in (-12., 12.):
            support = support.cut(hexagon(5.7, -6.8, 2.6, x, y))
            support = support.cut(cylinder(3.4, -12, 13, x, y))
            # Captive nut inserted from inside before fitting the display.
            support = support.cut(block(7, 5.7, -6.8, 2.6, x - math.copysign(3, x), y))
    bezel = bezel.union(support)
    bezel = hole(bezel, [(x, y) for x in (-65., 65.) for y in (-20., 20.)], 3.4, -1, 5)
    bezel = countersink(bezel, [(x, y) for x in (-65., 65.) for y in (-20., 20.)], 3.4, 6.4, 3, 1.7)
    add('display_bezel', 'Cornice OLED sostituibile con sede universale', on_front(bezel),
        'printed', 'PA12 proposed', '#26352e', 'PRINT_DISPLAY_BEZEL',
        'Pocket 101.5 x 34.5 x 11.3; owned SSD1322 module must be measured.', printable=True)
    add('display_window', 'Vetrino piano OLED 82 x 24 x 1.5', on_front(block(82, 24, 0, 1.5)),
        'custom', 'clear acrylic', '#233837', 'CUT_DISPLAY_WINDOW', 'Fit with thin perimeter adhesive; no food-contact claim.')
    display = block(100.5, 33.5, -11.3, 11.3)
    add('oled_reference', 'OLED SSD1322 - inviluppo da confermare', on_front(display),
        'electronics', 'reference envelope', '#12251a', 'REUSE_OLED_SSD1322',
        'Unverified owned module envelope; no invented board hole pattern.')
    for j, y in enumerate((-12., 12.), 1):
        bar = hole(block(114, 5, -14.3, 3, y=y), [(x, y) for x in (-54.5, 54.5)], 3.4, -15, 5)
        add(f'display_rear_bar_{j}', 'Traversino fermo OLED', on_front(bar), 'printed',
            'PA12 proposed', '#506459', 'PRINT_DISPLAY_REAR_BAR', printable=True)
        for k, x in enumerate((-54.5, 54.5), 1):
            socket_screw(f'display_bar_screw_{j}_{k}', 3, 10, -14.3, x, y, transform=on_front)
            nut(f'display_bar_nut_{j}_{k}', 3, 5.5, 2.4, -6.7, x, y, transform=on_front)
    for j, (x, y) in enumerate([(x, y) for x in (-65., 65.) for y in (-20., 20.)], 1):
        csk_screw(f'display_bezel_screw_{j}', 3, 10, 3, x, y, transform=on_front)
        nut(f'display_bezel_nut_{j}', 3, 5.5, 2.4, -5.4, x, y, transform=on_front)
    for j, ky in enumerate((-129., -98.), 1):
        membrane = rr(70, 20, 1, -.35, 1)
        add(f'keypad_strip_{j}', f'Tastiera adesiva 1x4 - striscia {j}', on_front(membrane, 77, ky),
            'controls', 'membrane purchased', '#537b64', 'KEYPAD_1x4_BERRYBASE_B_SM4T',
            '70x20x1 mm blank membrane; final function overlay is a custom print. Tail exit must be checked on purchased sample.',
            'https://www.berrybase.de/en/membrane-keypad-4-keys-without-labelling-with-adhesive-layer')


def build_power_tray():
    tray = rr(82, 96, 3, 19, 2, x=81.5, y=55)
    tray = hole(tray, POWER_FIX, 3.4, 18, 4)
    # Removable perimeter stops, not ribs in the smooth main electronics floor.
    for y in (8., 102.):
        tray = tray.union(block(78, 1., 21, 3, x=81.5, y=y))
    return tray.clean()


def build_usb():
    # Verified Premier PCM-0726 / Adafruit 6069 mounting orientation:
    # flange inside, M12x1 through the panel, nut and cap outside.
    body = cylinder(11, -19, 10).union(cylinder(18, -9, 6)).union(cylinder(12, -3, 15))
    body = body.cut(ring(14, 11, -4.5, 1.5))
    body = body.cut(rr(8.8, 3.4, 1.6, 2, 12))
    add('usb_c_panel_body', 'USB-C pannello PCM-0726 / Adafruit 6069', on_right(body),
        'controls', 'purchased connector', '#3c4546', 'USB_C_PANEL_6069',
        'M12x1 x15; flange D18x6; rear D11x10. Internal flange; external nut.',
        'https://www.adafruit.com/product/6069')
    add('usb_c_panel_nut', 'Dado plastico USB-C - ingombro provvisorio', on_right(ring(20, 12, 0, 4)),
        'fastener', 'ABS supplied', '#2b3030', 'INCLUDED_USB_NUT', 'Outer D20 x4 assumed; maker does not dimension nut.')
    add('usb_c_panel_seal', 'O-ring USB-C incluso', on_right(ring(14, 11, -4.5, 1.5)),
        'fastener', 'elastomer supplied', '#242827', 'INCLUDED_USB_ORING',
        'Ring simplified as rectangular section; actual 11 ID x1.5 circular cross-section.')
    # The cap is shown parked beside the socket so the USB-C opening remains visible.
    cap = cylinder(18, 0, 4, y=25)
    add('usb_c_protective_cap_parked', 'Tappo USB-C parcheggiato - ingombro', on_right(cap),
        'controls', 'elastomer supplied', '#35483d', 'INCLUDED_USB_CAP',
        'D18x4 assumed. Parked display position only; flexible tether is not dimensioned.')


def import_ups():
    path = OUT / 'sources' / 'waveshare-ups-hat-d' / 'UPS-HAT_D.stp'
    vendor = cq.importers.importStep(str(path))
    solids = vendor.solids().vals()
    assert len(solids) == 297, len(solids)
    for i, s in enumerate(solids, 1):
        b = s.BoundingBox()
        if b.xlen > 60 and 19 < b.ylen < 23 and 19 < b.zlen < 23:
            color, label = '#558a9a', 'Cella 21700 fornita nel kit'
        elif b.xlen > 50 and b.ylen > 30 and b.zlen < 2:
            color, label = '#387354', 'PCB UPS HAT(D)'
        elif b.xlen > 50 and b.ylen > 30 and b.zlen < 4:
            color, label = '#b0c9c3', 'Protezione acrilica UPS HAT(D)'
        else:
            color, label = '#616b65', 'Componente incluso UPS HAT(D)'
        moved = s.rotate((0, 0, 0), (0, 0, 1), 90).translate((112.732, 97.5, 49.1))
        add(f'ups_vendor_{i:03d}', f'{label} {i:03d}', moved, 'electronics_detail', 'vendor STEP geometry',
            color, 'INCLUDED_UPS_VENDOR_GEOMETRY',
            'One of 297 named solids from the official supplied assembly. Included in one CN kit SKU25507, not a separate purchase.',
            'https://www.waveshare.net/shop/UPS-HAT-D.htm')
    REFERENCES['ups_hat_d_reference'] = block(62.464, 85, 21, 38.6, x=81.5, y=55).val()


def cable_route(name, label, points, diameter, bom, nominal_length):
    # The route is a swept clearance envelope. Smooth spline bends are not a
    # claim about the actual selected cable's minimum bend radius.
    pts = [cq.Vector(*p) for p in points]
    path = cq.Edge.makeSpline(pts)
    normal = pts[1].sub(pts[0])
    section = cq.Workplane(cq.Plane(origin=pts[0], normal=normal)).circle(diameter / 2)
    wire = section.sweep(cq.Wire.assembleEdges([path]), isFrenet=True)
    add(name, label, wire, 'cable', 'routed cable envelope', '#353a3b', bom,
        f'Illustrative routed centreline {path.Length():.1f} mm; supplied/cut cable nominal {nominal_length} mm. Connector bodies and service slack require sample fitting.')
    CABLE_PATHS[name] = (path, pts[0], normal)
    return path.Length()


def build_wiring():
    # The real board faces -X after the official STEP placement.
    cy, cz = 47.9398, 50.7707
    ay, az = 64.7265, 52.7516
    cbody = block(16, 10, cz - 3, 6, x=37, y=cy)
    cmetal = block(11.5876, 8.1, cz - 1.2, 2.4, x=50.7938, y=cy)
    add('usb_c_internal_plug_body', 'Spina USB-C interna - inviluppo', cbody, 'controls', 'purchased connector envelope',
        '#404747', 'INCLUDED_USB_C_PLUG_BODY', 'Plug envelope assumed; board socket mouth comes from official STEP.')
    add('usb_c_internal_plug_metal', 'Innesto USB-C nella UPS', cmetal, 'controls', 'metal connector envelope',
        '#b9c0be', 'INCLUDED_USB_C_PLUG_METAL', 'Nominal envelope, electrical contact details omitted.')
    abody = block(18, 16, az - 4, 8, x=34, y=ay)
    ametal = block(14.2159, 12, az - 2.25, 4.5, x=50.10795, y=ay)
    add('usb_a_output_plug_body', 'Spina USB-A uscita 5 V - inviluppo', abody, 'controls', 'purchased connector envelope',
        '#404747', 'USB_A_OUTPUT_HARNESS_2697', 'Adafruit2697 22AWG harness cut to200mm; custom terminal end, no barrel connector remains.',
        'https://www.adafruit.com/product/2697')
    add('usb_a_output_plug_metal', 'Innesto USB-A nella UPS', ametal, 'controls', 'metal connector envelope',
        '#b9c0be', 'INCLUDED_USB_A_PLUG_METAL', 'Nominal envelope, electrical contact details omitted.')
    cable_route('usb_c_passive_cable', 'Cavo USB-C pannello 300 mm - percorso',
                [(131, 102, 40), (133, 91, 33), (133, 61, 25), (129, 25, 25),
                 (129, -10, 25), (132, -28, 25), (111, -40, 29), (71, -32, 30), (38, -2, 49),
                 (27, 24, 50.77), (27, 40, 50.77), (29, cy, cz)], 4.8,
                'INCLUDED_USB_C_CABLE_300MM', 300)
    cable_route('usb_a_5v_output_cable', 'Cablaggio uscita 5 V 200 mm - percorso',
                [(25, ay, az), (21, 51, 43), (21, 33, 28), (22, 18, 25),
                 (13, -3, 26), (-20, -15, 26), (-54, -40, 26), (-71, -34, 26),
                 (-65, -25, 26)], 4.5, 'CUSTOM_USB_A_5V_HARNESS_200MM', 200)
    # Two removable split clamps attach to tapped holes in the metal floor.
    for i, (x, y) in enumerate(((132., -28.), (22., 18.)), 1):
        route_name = 'usb_c_passive_cable' if i == 1 else 'usb_a_5v_output_cable'
        path, origin, normal = CABLE_PATHS[route_name]
        cutter = cq.Workplane(cq.Plane(origin=origin, normal=normal)).circle(2.7).sweep(cq.Wire.assembleEdges([path]), isFrenet=True)
        lower = block(20, 16, 19, 5, x=x, y=y).cut(cutter)
        upper = block(20, 10, 24, 5, x=x, y=y).cut(cutter)
        pts = [(x - 6, y), (x + 6, y)]
        lower, upper = hole(lower, pts, 3.4, 18, 7), hole(upper, pts, 3.4, 23, 7)
        add(f'cable_clamp_lower_{i}', 'Fermacavo apribile - base', lower, 'printed', 'PA12 proposed',
            '#50665a', 'PRINT_CABLE_CLAMP_LOWER', 'D5.4 cable channel; removable split clamp.', printable=True)
        add(f'cable_clamp_upper_{i}', 'Fermacavo apribile - coperchio', upper, 'printed', 'PA12 proposed',
            '#50665a', 'PRINT_CABLE_CLAMP_UPPER', printable=True)
        for j, (sx, sy) in enumerate(pts, 1):
            socket_screw(f'cable_clamp_screw_{i}_{j}', 3, 16, 29, sx, sy, up=False)
    terminals = block(12, 8, 21, 8, x=-65, y=-25)
    terminals = terminals.cut(block(8, 4, 24, 6, x=-65, y=-25))
    add('5v_terminal_reference', 'Terminazione 5 V sul vano elettronica - riferimento', terminals,
        'electronics', 'terminal envelope', '#608461', 'CUSTOM_5V_TERMINATION',
        'Interface location only. Actual existing5V rail connector and polarity must be verified before wiring.')
    cable_route('load_cell_4wire_cable', 'Cavo cella verso HX711 - percorso',
                [(-118, 50, 36), (-130, 70, 29), (-128, 112, 27), (-124, 132, 27),
                 (-87, 132, 27), (-74, 110, 27), (-86, 98, 27), (-108, 101, 27), (-128, 80, 27),
                 (-131, 35, 27), (-128, 0, 27), (-114, -41, 27), (-97, -39, 27),
                 (-90, -25, 27)], 4.5, 'INCLUDED_L6D_4WIRE_CABLE_400MM', 400)
    for i, color in enumerate(('#af433b', '#303635', '#668e64', '#d4d6cb')):
        wire = cq.Solid.makeCylinder(.5, 10, cq.Vector(-91.5 + i, -25, 27), cq.Vector(0, 1, 0))
        add(f'load_cell_wire_end_{i+1}', 'Conduttore cella verso HX711', wire, 'cable', 'wire end envelope', color,
            'INCLUDED_L6D_WIRE_END', 'Four free ends for E+/E-/A+/A-. Actual module solder/terminal location is not assumed.')


def main():
    add('base_aluminium', 'Fondo portante piano alluminio 6 mm', build_base(), 'custom', 'aluminium plate',
        '#a8afb0', 'CNC_BASE_300x320x6', 'Upper face smooth. Tapped holes shown at nominal major diameter; drawing must specify taps.')
    add('outer_shell', 'Scocca superiore rimovibile', build_shell(), 'printed', 'PA12 proposed',
        '#dadbd2', 'PRINT_OUTER_SHELL', 'Non-load-bearing shell; 3 mm walls, captive M3 nuts. Validate printed fit.', printable=True)
    fixed = hole(rr(25, 30, 1, 19, 6, x=-105.5, y=50), CELL_FIXED, 6.4, 18, 8)
    moving = hole(rr(25, 30, 1, 47, 22, x=-.5, y=50), CELL_MOVING, 6.4, 46, 24)
    add('cell_fixed_spacer', 'Distanziale fisso cella 25 x30 x6', fixed, 'custom', 'aluminium plate', '#b3babc', 'CNC_CELL_FIXED_SPACER')
    add('cell_moving_spacer', 'Distanziale mobile cella 25 x30 x22', moving, 'custom', 'aluminium bar', '#b3babc', 'CNC_CELL_MOVING_SPACER')
    cell = hole(block(130, 30, 25, 22, x=-53, y=50), CELL_FIXED + CELL_MOVING, 6., 24, 24)
    add('load_cell_l6d', 'Zemic L6D-C3-30kg - involucro meccanico', cell, 'loadcell', 'aluminium purchased',
        '#b4b9ba', 'ZEMIC_L6D_C3_30KG_0_4B',
        'Drawing envelope 130x30x22, holes M6 through 106x15. Internal strain geometry omitted. Platform axis acceptance and torque require supplier confirmation.',
        'https://www.zemiceurope.com/media/Documentation/L6D_Datasheet.pdf')
    add('plate_carrier_aluminium', 'Portapiatto piano alluminio 8 mm', build_carrier(), 'custom', 'aluminium plate',
        '#b5bdbb', 'CNC_CARRIER_276x196x8', 'Two M6 countersinks; six tapped M3. No plastic load-bearing ribs.')
    add('plastic_pan', 'Piatto plastico liscio 280 x200 x4', build_pan(), 'printed', 'PA12 proposed',
        '#f0f0e8', 'PRINT_PAN_280x200x4', 'Supported over almost all its underside by the metal carrier; no food-contact certification.', printable=True)
    for i, (x, y) in enumerate(CELL_FIXED, 1):
        add(f'cell_fixed_washer_{i}', 'Rondella M6 ISO7089', ring(12, 6.4, 11.4, 1.6, x, y),
            'fastener', 'steel 200HV', '#b0b5b8', 'WASHER_M6_ISO7089')
        socket_screw(f'cell_fixed_screw_{i}', 6, 25, 11.4, x, y)
    for i, (x, y) in enumerate(CELL_MOVING, 1):
        csk_screw(f'cell_moving_screw_{i}', 6, 45, 77, x, y)
    for i, (x, y) in enumerate(PAN_FIX, 1):
        csk_screw(f'pan_screw_{i}', 3, 10, 81, x, y)
    for i, (x, y) in enumerate(SHELL_FASTENERS, 1):
        socket_screw(f'shell_screw_{i}', 3, 16, 13, x, y)
        nut(f'shell_nut_{i}', 3, 5.5, 2.4, 26.6, x, y)
    for i, (x, y) in enumerate(FEET, 1):
        # Purchased foot split into individual physical material parts.
        add(f'foot_pad_{i}', 'Pattino antiscivolo piede Elesa', cylinder(30, 0, 1, x, y),
            'feet', 'SBR supplied', '#232b27', 'INCLUDED_FOOT_SBR')
        body = cylinder(30, 1, 6, x, y).union(cylinder(12, 7, 1, x, y)).union(hexagon(17, 8, 5, x, y))
        body = body.union(cylinder(6, 13, 23, x, y))
        add(f'adjustable_foot_{i}', 'Piede Elesa LX.30-SW17-AS-M6x22', body, 'feet', 'purchased foot simplified',
            '#444c49', 'ELESA_531051', 'Envelope D30, overall36 incl SBR. Profile simplified; stud measured nominal23 in EN drawing.',
            'https://www.elesa.com/siteassets/PDF/PDF_EN/LX.pdf')
        nut(f'foot_locknut_{i}', 6, 10, 5, 19, x, y)
    # Dedicated metal stops protect against gross overload after adjustment.
    # A 1.5 mm nominal gap is provisional; set from the assembled calibrated cell.
    for i, (x, y) in enumerate([(x, y) for x in (-112., 112.) for y in (-20., 120.)], 1):
        s = cylinder(6, 7.5, 60, x, y)
        s = s.cut(hexagon(3, 7.4, 3.1, x, y))
        add(f'overload_stop_{i}', 'Grano M6 x60 arresto regolabile', s, 'fastener', 'steel', '#8e9799',
            'SETSCREW_M6x60_DIN913_FLAT_END', 'Tip Z67.5; carrier Z69 => provisional1.5mm gap. Full6mm base engagement, locknut. Never preload the measuring plate.',
            'https://www.rwproducts.nl/din-913-stelschroef-binnenzeskant-met-afschuining-elvz-m6x60')
        nut(f'overload_locknut_{i}', 6, 10, 5, 19, x, y)
    add('power_tray', 'Vassoio potenza rimovibile', build_power_tray(), 'printed', 'PA12 proposed',
        '#596b64', 'PRINT_POWER_TRAY', 'Generic edge-guided tray; no invented UPS PCB mounting pattern.', printable=True)
    for j, y in enumerate((22., 88.), 1):
        # The real acrylic shield is BELOW the batteries, Z21..22.5.
        # Capture that sheet; never clamp the PCB's tallest pogo component.
        bridge_y = y if j == 1 else y - 1.0
        strap = block(66, 8, 22.5, 2, x=82, y=bridge_y)
        strap = strap.union(block(2, 8, 21, 1.5, x=50, y=bridge_y))
        strap = strap.union(block(2, 8, 21, 1.5, x=114, y=bridge_y))
        for x in (45., 118.):
            strap = strap.union(block(10, 8, 21, 2, x=x, y=y))
        strap = hole(strap, [(45., y), (118., y)], 3.4, 20, 5)
        add(f'power_retaining_strap_{j}', 'Staffa rimovibile ritegno verticale UPS', strap,
            'printed', 'PA12 proposed', '#42584b', 'PRINT_POWER_RETAINING_STRAP',
            'Clamps official lower acrylic sheet at Z22.5. Bridge top24.5 leaves nominal0.9mm below battery envelope. Verify printed fit; no PCB/pogo contact.', printable=True)
    for i, (x, y) in enumerate(POWER_FIX, 1):
        socket_screw(f'power_tray_screw_{i}', 3, 10, 23, x, y, up=False)
    electronics = hole(rr(145, 62, 3, 19, 2, x=-40.5, y=-28), ELECTRONICS_FIX, 3.4, 18, 4)
    electronics = hole(electronics, [(-112., -20.)], 13., 18, 4)
    add('electronics_tray', 'Piastra elettronica universale rimovibile - faccia piana', electronics,
        'printed', 'PA12 proposed', '#73877c', 'PRINT_ELECTRONICS_TRAY',
        'Flat 145x62 area for existing boards; no assumed module holes. Use removable insulating mounts after measuring boards.', printable=True)
    for i, (x, y) in enumerate(ELECTRONICS_FIX, 1):
        socket_screw(f'electronics_tray_screw_{i}', 3, 8, 21, x, y, up=False)
    # Official vendor STEP remains available separately; a bounding reference is
    # used here to avoid silently claiming an unverified purchased board revision.
    import_ups()
    build_front()
    build_usb()
    build_wiring()
    from controls_cable_details import build_extra_controls
    build_extra_controls(globals())
    baseline = OUT / 'interference-audit-full-before-usb-reroute.json' if '--incremental-usb-audit' in sys.argv else None
    audit = pair_audit(baseline)
    if audit['unexpected_overlap_count']:
        print(json.dumps([p for p in audit['overlaps'] if not p['allowed']], indent=2))
    assert audit['unexpected_overlap_count'] == 0, 'See interference-audit.json'
    export_all(audit)


def stats(s):
    b = s.BoundingBox()
    return dict(brep_valid=s.isValid(), solid_count=len(s.Solids()),
                volume_mm3=round(s.Volume(), 3),
                bounds_mm=[[round(b.xmin, 3), round(b.ymin, 3), round(b.zmin, 3)],
                           [round(b.xmax, 3), round(b.ymax, 3), round(b.zmax, 3)]])


def pair_audit(reuse_baseline_path=None):
    """Check all project parts against each other and vendor parts via AABB.

    The vendor's own internal contacts are excluded; mating electrical connector
    envelopes and labelled cable terminations are kept as explicit exceptions.
    """
    bounds = {p['name']: p['shape'].BoundingBox() for p in PARTS}
    changed = {'usb_c_passive_cable', 'cable_clamp_lower_1', 'cable_clamp_upper_1'}
    overlaps, tested, rechecked = [], 0, 0
    baseline = None
    if reuse_baseline_path is not None:
        baseline = json.loads(reuse_baseline_path.read_text(encoding='utf-8'))
        assert baseline['unexpected_overlap_count'] == 1
        remaining = [p for p in baseline['overlaps'] if not p['allowed']]
        assert len(remaining) == 1 and set((remaining[0]['a'], remaining[0]['b'])) == {'outer_shell', 'usb_c_passive_cable'}
        overlaps = [p for p in baseline['overlaps'] if not ({p['a'], p['b']} & changed)]
    explicit = {
        frozenset(('usb_c_passive_cable', 'usb_c_internal_plug_body')): 'Cable enters its moulded connector envelope.',
        frozenset(('usb_c_passive_cable', 'usb_c_panel_body')): 'Cable enters its panel connector envelope.',
        frozenset(('usb_a_5v_output_cable', 'usb_a_output_plug_body')): 'Cable enters its moulded connector envelope.',
        frozenset(('usb_a_5v_output_cable', '5v_terminal_reference')): 'Cable termination envelope.',
        frozenset(('load_cell_4wire_cable', 'load_cell_l6d')): 'Cell cable exits its purchased enclosure.',
    }
    for a, b in combinations(PARTS, 2):
        an, bn = a['name'], b['name']
        if a['category'] == b['category'] == 'electronics_detail':
            continue
        aa, bb = bounds[an], bounds[bn]
        if (min(aa.xmax, bb.xmax) - max(aa.xmin, bb.xmin) < .00001 or
                min(aa.ymax, bb.ymax) - max(aa.ymin, bb.ymin) < .00001 or
                min(aa.zmax, bb.zmax) - max(aa.zmin, bb.zmin) < .00001):
            continue
        tested += 1
        if baseline is not None and not ({an, bn} & changed):
            continue
        rechecked += 1
        volume = a['shape'].intersect(b['shape']).Volume()
        if volume < .001:
            continue
        reason = explicit.get(frozenset((an, bn)))
        names = [an, bn]
        if any(n.startswith('ups_vendor_') for n in names) and any(n in ('usb_c_internal_plug_metal', 'usb_a_output_plug_metal') for n in names):
            reason = 'Simplified inserted electrical plug/contact envelope; exact purchased plug not measured.'
        for j in (1, 2):
            if set(names) == {f'keypad_tail_{j}', f'keypad_strip_{j}'}:
                reason = 'Continuous membrane tail joins the same purchased keypad.'
            if set(names) == {f'keypad_tail_{j}', f'keypad_female_connector_{j}'}:
                reason = 'Flex terminates inside its connector envelope.'
            if f'keypad_adapter_branch_{j}' in names and any(n.startswith('keypad_adapter_header_') or n.startswith('keypad_matrix_terminal_contact_') or n == 'keypad_matrix_6p_termination' for n in names):
                reason = 'Harness joins its header/termination envelope.'
        if any(n.startswith('load_cell_wire_end_') for n in names) and 'load_cell_4wire_cable' in names:
            reason = 'Individual wire exits its sheath envelope.'
        overlaps.append(dict(a=an, b=bn, intersection_mm3=round(volume, 6), allowed=bool(reason), reason=reason))
    report = dict(scope='All project solids against each other and official UPS solids; vendor-internal pairs excluded.',
                  aabb_candidate_pairs=tested, overlaps=overlaps,
                  unexpected_overlap_count=sum(not p['allowed'] for p in overlaps))
    if baseline is not None:
        report['verification_mode'] = 'Complete baseline plus focused recheck after USB-C reroute.'
        report['baseline_file'] = reuse_baseline_path.name
        report['baseline_sha256'] = hashlib.sha256(reuse_baseline_path.read_bytes()).hexdigest()
        report['recomputed_parts'] = sorted(changed)
        report['recomputed_candidate_pairs'] = rechecked
        report['unchanged_geometry_reused'] = 'All other parts and placements are unchanged from the complete baseline. Clamp channels follow the cable and are included in the focused recheck.'
    else:
        report['verification_mode'] = 'Complete current-geometry check.'
    (OUT / 'interference-audit.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print('All-pair interference audit:', report['unexpected_overlap_count'], 'unclassified overlaps; candidates', tested)
    return report


def export_all(audit):
    for directory in ('step', 'stl', 'drawings'):
        (OUT / directory).mkdir(exist_ok=True)
    assembly = cq.Assembly(name='MINU_CONCEPT_02_NOMINAL_ASSEMBLY')
    preview = dict(units='mm', axes={'x': 'width', 'y': 'depth; front negative', 'z': 'up'}, parts=[])
    verification = dict(status='NOMINAL_CAD_ASSEMBLY_PROTOTYPE_NOT_RELEASED',
                        units='mm', target_useful_load_kg=20, load_capacity_verified=False,
                        nominal_assembly_modeled=True, physical_assembly_verified=False,
                        final_print_tolerances_verified=False, threads='nominal cylinders; no helices',
                        outline_mm=[300, 320, 81], parts={}, selected_nominal_contacts=[],
                        user_display_measured=False, battery_selected_fit_verified=False,
                        minimum_nominal_carrier_to_shell_gap_mm=4,
                        minimum_nominal_ups_to_roof_gap_mm=2.4,
                        overload_stop_gap_mm=1.5,
                        cell_mount_engagement_mm={'fixed_M6x25': 11.4, 'moving_M6x45': 15.0},
                        notes=['Not a strength, print-process, assembly-fit or weighing-accuracy validation.',
                               'CNC files have nominal holes; fabrication drawing/tap and countersink callouts govern.',
                               'The exact owned display needs measurement; USB nut and cap dimensions are provisional.',
                               'UPS uses official vendor STEP solids; wiring, charging implementation and physical fit are unverified.'])
    by_name = {}
    for p in PARTS:
        name, s = p['name'], p['shape']
        by_name[name] = p
        st = stats(s)
        verification['parts'][name] = st
        assembly.add(s, name=name, color=cq.Color(p['color']))
        # Separate custom manufacturing STEP and STL, plus every purchased
        # component as a named solid in the complete assembly.
        if p['category'] in ('printed', 'custom'):
            cq.exporters.export(s, str(OUT / 'step' / f'{name}.step'))
            if p['printable']:
                path = OUT / 'stl' / f'{name}.stl'
                cq.exporters.export(s, str(path), tolerance=.12, angularTolerance=.12)
                m = trimesh.load_mesh(path, process=True)
                st.update(stl_watertight=bool(m.is_watertight), stl_winding_consistent=bool(m.is_winding_consistent),
                          stl_components=len(m.split()), stl_triangles=len(m.faces))
                assert m.is_watertight and m.is_winding_consistent and len(m.split()) == 1, name
        verts, faces = s.tessellate(.6, .22)
        preview['parts'].append(dict(name=name, label=p['label'], category=p['category'],
                                     color=p['color'], material=p['material'], notes=p['notes'],
                                     printable=p['printable'],
                                     vertices=[[round(v.x, 3), round(v.y, 3), round(v.z, 3)] for v in verts],
                                     faces=[list(f) for f in faces]))
    for name, s in REFERENCES.items():
        by_name[name] = dict(shape=s)
    # Check primary load path, floating clearances and electronics envelope.
    checks = [('base_aluminium', 'outer_shell'), ('plastic_pan', 'plate_carrier_aluminium'),
              ('load_cell_l6d', 'base_aluminium'), ('load_cell_l6d', 'outer_shell'),
              ('cell_fixed_spacer', 'load_cell_l6d'), ('cell_moving_spacer', 'load_cell_l6d'),
              ('cell_moving_spacer', 'outer_shell'), ('plate_carrier_aluminium', 'outer_shell'),
              ('ups_hat_d_reference', 'outer_shell'), ('ups_hat_d_reference', 'load_cell_l6d'),
              ('ups_hat_d_reference', 'plate_carrier_aluminium'), ('ups_hat_d_reference', 'power_tray'),
              ('usb_c_panel_body', 'outer_shell'), ('display_bezel', 'outer_shell'),
              ('display_bezel', 'oled_reference')]
    checks += [(f'overload_stop_{i}', 'outer_shell') for i in range(1, 5)]
    checks += [(f'power_retaining_strap_{i}', 'outer_shell') for i in range(1, 3)]
    checks += [(f'power_retaining_strap_{i}', f'ups_vendor_{j:03d}') for i in range(1, 3) for j in range(1, 298)]
    checks += [('electronics_tray', 'overload_stop_1'), ('electronics_tray', 'outer_shell')]
    for a, b in checks:
        overlap = by_name[a]['shape'].intersect(by_name[b]['shape']).Volume()
        verification['selected_nominal_contacts'].append(dict(a=a, b=b, intersection_mm3=round(overlap, 6)))
        assert overlap < .001, (a, b, overlap)
    verification['all_pair_interference_audit'] = {k: audit[k] for k in ('scope', 'aabb_candidate_pairs', 'unexpected_overlap_count')}
    # The support polygon is the rounded rectangle generated by the four foot
    # disks. Sample the entire rounded pan boundary to verify vertical stability
    # geometry; this is not a horizontal-push or uneven-foot test.
    points = []
    for cx, cy, start in [(120, 130, 0), (-120, 130, 90), (-120, -30, 180), (120, -30, 270)]:
        for i in range(91):
            a = math.radians(start + i)
            points.append((cx + 20 * math.cos(a), cy + 20 * math.sin(a)))
    excess = [math.hypot(max(abs(x) - 132, 0), max(abs(y) - 142, 0)) for x, y in points]
    assert max(excess) <= 15
    verification['vertical_load_support_polygon'] = dict(pan_outline_inside=True, foot_centers_xy=FEET,
                                                       foot_contact_radius_mm=15, minimum_axis_margin_mm=7,
                                                       note='Four coplanar full-contact foot disks assumed; no lateral force, uneven support or dynamics.')
    asm_path = OUT / 'step' / 'MINU_concept_02_COMPLETE_NOMINAL_ASSEMBLY.step'
    assembly.export(str(asm_path))
    imported = cq.importers.importStep(str(asm_path))
    count = len(imported.solids().vals())
    assert imported.val().isValid() and count == len(PARTS), (count, len(PARTS))
    verification['assembly_step_roundtrip'] = dict(brep_valid=True, solid_count=count)
    verification['total_named_parts'] = len(PARTS)
    physical_bounds = [p['shape'].BoundingBox() for p in PARTS]
    verification['complete_assembly_bounds_mm'] = [
        [round(min(b.xmin for b in physical_bounds), 3), round(min(b.ymin for b in physical_bounds), 3), round(min(b.zmin for b in physical_bounds), 3)],
        [round(max(b.xmax for b in physical_bounds), 3), round(max(b.ymax for b in physical_bounds), 3), round(max(b.zmax for b in physical_bounds), 3)]]
    verification['vendor_ups_assembly'] = dict(included_solids=297, purchase_quantity=1,
                                             purchase_sku='Waveshare CN UPS HAT(D) kit 25507',
                                             official_step='sources/waveshare-ups-hat-d/UPS-HAT_D.stp',
                                             mounting_transform='rotate Z+90deg; translate(112.732,97.5,49.1)mm')
    verification['part_category_counts'] = dict(Counter(p['category'] for p in PARTS))
    grouped = {}
    for p in PARTS:
        if p['bom'] not in grouped:
            grouped[p['bom']] = dict(item=p['bom'], label=p['label'], quantity=0, category=p['category'],
                                    material=p['material'], notes=p['notes'], source=p['source'], cad_names=[])
        grouped[p['bom']]['quantity'] += 1
        grouped[p['bom']]['cad_names'].append(p['name'])
    (OUT / 'bom-geometry.json').write_text(json.dumps(list(grouped.values()), indent=2, ensure_ascii=False), encoding='utf-8')
    with (OUT / 'bom-geometry.csv').open('w', encoding='utf-8-sig', newline='') as f:
        fields = ['item', 'label', 'quantity', 'category', 'material', 'notes', 'source', 'cad_names']
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for item in grouped.values():
            writer.writerow({**item, 'cad_names': '; '.join(item['cad_names'])})
    (OUT / 'preview-mesh.json').write_text(json.dumps(preview, separators=(',', ':'), ensure_ascii=False), encoding='utf-8')
    (OUT / 'verification.json').write_text(json.dumps(verification, indent=2, ensure_ascii=False), encoding='utf-8')
    if 'KEYPAD_DETAILS_REPORT' in globals():
        (OUT / 'keypad-geometry-details.json').write_text(json.dumps(KEYPAD_DETAILS_REPORT, indent=2), encoding='utf-8')
    print(json.dumps({k: verification[k] for k in ['status', 'total_named_parts', 'part_category_counts', 'assembly_step_roundtrip', 'vertical_load_support_polygon']}, ensure_ascii=False))
    print('Export complete:', asm_path)
    if sys.platform == 'win32':
        # OCP 7.9 teardown crashes after successful STEPCAF export on this runtime.
        # Files are closed and all assertions have completed before bypassing it.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == '__main__':
    main()
