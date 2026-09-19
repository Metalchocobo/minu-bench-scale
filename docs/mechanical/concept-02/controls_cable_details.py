"""Additional nominal keyboard mounting and cabling for concept 02.

Call build_extra_controls(globals()) from generate_assembly after build_front.
Do not also add the old short keypad_tail objects. Connector dimensions and
bend radii are packaging assumptions, not verified BerryBase mechanical data.
"""
import math


def build_extra_controls(api):
    cq, add = api['cq'], api['add']
    block, rr, on_front = api['block'], api['rr'], api['on_front']
    tape_url = 'https://www.3m.com/3M/en_US/p/dc/v100808810/'
    report = {'units': 'mm', 'physical_fit_verified': False, 'tails': []}
    # Coordinates are local to the inclined fascia. The printed hood only
    # bonds outside the membrane footprint; no feet press the button areas.
    for j, ky in enumerate((-129., -98.), 1):
        hood = rr(24, 11, 1, 1.2, 1.2, y=13.5)
        hood = hood.union(block(24, 2, .13, 1.07, y=18.))
        for x in (-11., 11.):
            hood = hood.union(block(2, 6, .13, 1.07, x=x, y=14.))
        add(f'keypad_cable_hood_{j}', 'Ponte copricoda tastiera adesivo', on_front(hood, 77, ky),
            'printed', 'PA12 proposed', '#597264', 'PRINT_KEYPAD_CABLE_HOOD',
            '24x11x2.4 cover; adhesive feet outside 70x20 membrane. No claim of liquid sealing; fit bend on sample.', printable=True)
        pads = [(24, 2, 0., 18.), (2, 6, -11., 14.), (2, 6, 11., 14.)]
        for k, (w, d, x, y) in enumerate(pads, 1):
            pad = block(w, d, 0, .13, x=x, y=y)
            add(f'keypad_hood_adhesive_{j}_{k}', 'Adesivo copricoda 0.13 mm', on_front(pad, 77, ky),
                'custom', '3M468MP proposed', '#ba9d69', 'CUT_3M468MP_KEYPAD_HOOD_PAD',
                'Manufacturer tape thickness0.13; adhesion to finished PA12 must be tested. Cut piece, not a full roll.', tape_url)

        # A 76 mm flexible centreline plus a provisional12 mm connector yields
        # the selected BerryBase88 mm tail/connector envelope. The two arcs
        # avoid a right-angle crease at the panel slot.
        # Flat membrane ribbon bends in its own YZ plane; separate the first
        # tail by5mm vertically instead of an impossible in-plane lateral bend.
        drop = 5. if j == 1 else 0.
        straight = 76. - 1. - math.pi - 18. - 2. * math.pi - drop
        end_x, end_y, end_z = 0., 16. + straight, -24.5 - drop
        route = (cq.Workplane('YZ').moveTo(9., -.5).lineTo(10., -.5)
                 .threePointArc((11.41421356, -1.08578644), (12., -2.5))
                 .lineTo(12., -20.5 - drop)
                 .threePointArc((13.17157288, -23.32842712 - drop), (16., end_z))
                 .lineTo(end_y, end_z).wire().val())
        section = cq.Workplane(cq.Plane(origin=(0, 9., -.5), xDir=(1, 0, 0), normal=(0, 1, 0))).rect(14.28, .3)
        ribbon = section.sweep(route, isFrenet=True)
        add(f'keypad_tail_{j}', 'Coda tastiera curva - inviluppo 88 mm con connettore', on_front(ribbon, 77, ky),
            'cable', 'flex envelope', '#b99b54', 'INCLUDED_KEYPAD_TAIL',
            '76mm routed flex +12mm provisional connector =88mm. Width14.28 and thickness0.3 assumed; BerryBase only publishes total length. Bend radii2/4mm require sample validation.')
        female = block(14.28, 12, end_z - 2, 4, x=end_x, y=end_y + 6)
        for x in (-5.08, -2.54, 0., 2.54, 5.08):
            female = female.cut(block(1.0, 5, end_z - .5, 1, x=end_x + x, y=end_y + 10))
        add(f'keypad_female_connector_{j}', 'Connettore femmina 5 pin incluso nella membrana', on_front(female, 77, ky),
            'controls', 'purchased connector envelope', '#282e2d', 'INCLUDED_KEYPAD_5P_FEMALE',
            'Pitch2.54 documented; 14.28x12x4 body and cavity details are provisional, measure selected BerryBase sample.')
        male = block(12.7, 2.54, end_z - 1.27, 2.54, x=end_x, y=end_y + 13.27)
        for x in (-5.08, -2.54, 0., 2.54, 5.08):
            male = male.cut(block(.64, 3, end_z - .32, .64, x=end_x + x, y=end_y + 13.27))
        add(f'keypad_adapter_header_body_{j}', 'Pettine maschio 1x5 adattatore tastiera', on_front(male, 77, ky),
            'controls', 'header body envelope', '#313b34', 'HEADER_1x5_2p54',
            'Nominal1x5 cut header; assembled adapter, verify pin assignment by continuity.')
        for k, x in enumerate((-5.08, -2.54, 0., 2.54, 5.08), 1):
            pin = block(.64, 10, end_z - .32, .64, x=end_x + x, y=end_y + 13)
            add(f'keypad_adapter_header_pin_{j}_{k}', 'Contatto pettine tastiera', on_front(pin, 77, ky),
                'fastener', 'header contact', '#c4b576', 'INCLUDED_HEADER_CONTACT', 'Nominal square0.64mm contact, included with1x5 header.')
        point = on_front(cq.Vertex.makeVertex(end_x, end_y + 18., end_z), 77, ky).Center()
        start = (point.x, point.y, point.z)
        # These two branches join a common6-way termination above the removable
        # electronics tray. The six-way termination is not an assumed PCB socket.
        destination = (-44.5 if j == 1 else -36.5, -51., 25.)
        points = ([start, (77., -46., 36.), (71., -43., 27.), (60., -53., 30.),
                   (0., -65., 30.), (-44.5, -60., 25.), destination] if j == 1 else
                  [start, (60., -13., 35.), (12., -30., 37.), (-25., -40., 34.),
                   (-36.5, -55., 25.), destination])
        branch_length = api['cable_route'](f'keypad_adapter_branch_{j}', 'Cablaggio su misura matrice tastiera - ramo', points, 4.2,
                                          'CUSTOM_KEYPAD_MATRIX_BRANCH', 200)
        report['tails'].append({'part': f'keypad_tail_{j}', 'flex_centerline_mm': round(route.Length(), 4),
                                'connector_length_assumed_mm': 12, 'combined_target_mm': 88,
                                'jumper_start': list(start), 'jumper_end': list(destination),
                                'jumper_centerline_mm': round(branch_length, 3), 'jumper_nominal_cut_length_mm': 200})

    termination = block(15.24, 6, 23., 4., x=-40.5, y=-48.)
    for x in (-46.85, -44.31, -41.77, -39.23, -36.69, -34.15):
        termination = termination.cut(block(1., 4., 24.5, 1., x=x, y=-46.))
    add('keypad_matrix_6p_termination', 'Terminazione adattatore tastiera 6 fili', termination,
        'controls', 'connector envelope', '#26362d', 'HOUSING_1x6_2p54',
        'Generic6-way termination positioned above electronics tray. Pitch2.54; body depth/height provisional. Not a verified mating socket on owned ESP32; wire lengths and pin order require assembly.',
        'https://www.berrybase.de/en/dupont-housing-1x6-pin')
    for k, x in enumerate((-46.85, -44.31, -41.77, -39.23, -36.69, -34.15), 1):
        pin = block(.64, 3.8, 24.68, .64, x=x, y=-46.)
        add(f'keypad_matrix_terminal_contact_{k}', 'Contatto terminazione tastiera 6 fili', pin,
            'fastener', 'connector contact envelope', '#c4b576', 'CRIMP_CONTACT_2p54',
            'Generic compatible crimp contact envelope; order contact and housing from one matched family.')
    api['KEYPAD_DETAILS_REPORT'] = report
    return report
