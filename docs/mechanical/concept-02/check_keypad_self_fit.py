"""Fast pair review of nominal keypad helper geometry."""
import json
import os
from itertools import combinations
from pathlib import Path
import generate_assembly as g
from controls_cable_details import build_extra_controls
build_extra_controls(vars(g))
results = []
for a, b in combinations(g.PARTS, 2):
    ab, bb = a['shape'].BoundingBox(), b['shape'].BoundingBox()
    if ab.xmax < bb.xmin or ab.xmin > bb.xmax or ab.ymax < bb.ymin or ab.ymin > bb.ymax or ab.zmax < bb.zmin or ab.zmin > bb.zmax:
        continue
    volume = a['shape'].intersect(b['shape']).Volume()
    if volume > .01:
        results.append({'a': a['name'], 'b': b['name'], 'intersection_mm3': round(volume, 5)})
report = {'clashes_requiring_interpretation': results, 'keypad_details': g.KEYPAD_DETAILS_REPORT}
Path(__file__).with_name('keypad-self-fit-check.json').write_text(json.dumps(report, indent=2), encoding='utf8')
print(json.dumps(report, indent=2), flush=True)
os._exit(0)
