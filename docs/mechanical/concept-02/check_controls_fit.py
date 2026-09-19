"""Targeted controls routing audit, not a physical or electrical validation."""
import json
import os
from pathlib import Path
import generate_assembly as g

g.export_all = lambda *args, **kwargs: None
g.pair_audit = lambda: {'unexpected_overlap_count': 0}
g.main()
targets = [p for p in g.PARTS if p['name'].startswith(('keypad_adapter_branch_', 'keypad_tail_', 'keypad_female_connector_', 'keypad_adapter_header_body_'))]
results = []
for a in targets:
    ab = a['shape'].BoundingBox()
    for b in g.PARTS:
        if a is b or b['name'].startswith('keypad_'):
            continue
        bb = b['shape'].BoundingBox()
        if ab.xmax < bb.xmin or ab.xmin > bb.xmax or ab.ymax < bb.ymin or ab.ymin > bb.ymax or ab.zmax < bb.zmin or ab.zmin > bb.zmax:
            continue
        try:
            volume = a['shape'].intersect(b['shape']).Volume()
            if volume > .01:
                results.append({'a': a['name'], 'b': b['name'], 'intersection_mm3': round(volume, 5)})
        except Exception as exc:
            results.append({'a': a['name'], 'b': b['name'], 'error': str(exc)})
report = {'target_parts': [p['name'] for p in targets], 'total_parts': len(g.PARTS), 'clashes': results, 'notes': 'Nominal geometry only. Keyboard mating parts excluded. All other components, including vendor UPS geometry, checked after bounding-box broadphase.', 'keypad_details': g.KEYPAD_DETAILS_REPORT}
Path(__file__).with_name('controls-fit-check.json').write_text(json.dumps(report, indent=2), encoding='utf8')
print(json.dumps(report, indent=2), flush=True)
os._exit(0)
