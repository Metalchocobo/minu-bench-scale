"""Pack the CAD tessellation for an inline viewer, without changing CAD files."""
from pathlib import Path
import argparse
import base64
import json
import struct
import gzip
import fast_simplification
import trimesh

p = argparse.ArgumentParser()
p.add_argument('template', type=Path)
p.add_argument('output', type=Path)
args = p.parse_args()
source = Path(__file__).resolve().parent / 'preview-mesh.json'
data = json.loads(source.read_text(encoding='utf-8'))
result = {'parts': []}
skipped = 0
for part in data['parts']:
    # The STEP remains complete. Skip only tiny vendor electronic details in the
    # lightweight viewer, preserving all project fasteners and custom parts.
    v = part['vertices']
    mins = [min(pt[i] for pt in v) for i in range(3)]
    maxs = [max(pt[i] for pt in v) for i in range(3)]
    vol = (maxs[0]-mins[0])*(maxs[1]-mins[1])*(maxs[2]-mins[2])
    if part['name'].startswith('ups_vendor_') and vol < 20:
        skipped += 1
        continue
    faces = part['faces']
    target = 1200 if part['name'].startswith('ups_vendor_') or part['category']=='cable' else 6000
    if len(faces) > target:
        mesh = trimesh.Trimesh(vertices=v, faces=faces, process=True)
        vv, ff = fast_simplification.simplify(mesh.vertices, mesh.faces, target_count=target, agg=5)
        v, faces = vv.tolist(), ff.tolist()
    unique, remap, packed = {}, [], []
    for pt in v:
        q = tuple(round(x*10) for x in pt)
        if q not in unique:
            unique[q] = len(packed)//3
            packed.extend(q)
        remap.append(unique[q])
    triangles = [remap[i] for face in faces for i in face]
    wide = len(unique) > 65535
    pv = base64.b64encode(struct.pack('<'+'h'*len(packed), *packed)).decode('ascii')
    pf = base64.b64encode(struct.pack('<'+('I' if wide else 'H')*len(triangles), *triangles)).decode('ascii')
    result['parts'].append({k: part[k] for k in ('name','label','category','color','material')} | {'v':pv,'f':pf,'i32':wide})
compressed = base64.b64encode(gzip.compress(json.dumps(result, separators=(',',':'), ensure_ascii=False).encode('utf-8'), mtime=0)).decode('ascii')
text = args.template.read_text(encoding='utf-8').replace('/*COMPRESSED_CAD_DATA*/null', json.dumps(compressed))
args.output.write_text(text, encoding='utf-8')
print(json.dumps({'viewer_bytes': args.output.stat().st_size, 'parts':len(result['parts']), 'tiny_vendor_details_omitted':skipped}))
assert args.output.stat().st_size < 1_000_000, 'Viewer exceeds inline size budget'
