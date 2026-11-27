"""Verify exported CAD geometry against manifest and URDF transforms."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[2];PKG=ROOT/'arm_ws/src/arm_description_cad'
m=json.loads((ROOT/'docs/cad_step/manifest.json').read_text());r=ET.parse(PKG/'urdf/arm_cad.urdf').getroot()
assert r.find('ros2_control') is None
assert len(r.findall('link'))==14
assert len(m['solids'])==153
parents={j.find('child').get('link'):j for j in r.findall('joint')}
assert set(x.get('name') for x in r.findall('link'))-set(parents)=={'base_link'}
base=np.array(m['frames_source_mm']['base_link'])
for link,frame in m['frames_source_mm'].items():
 pos=np.zeros(3);name=link;seen=set()
 while name in parents:
  assert name not in seen;seen.add(name);j=parents[name]
  pos+=np.array(list(map(float,j.find('origin').get('xyz').split())))
  name=j.find('parent').get('link')
 np.testing.assert_allclose(pos,(np.array(frame)-base)/1000,atol=1e-12)
for row in m['solids']:
 mesh=trimesh.load(PKG/f"meshes/solid_{row['solid']:03d}.stl",force='mesh')
 assert len(mesh.faces)==row['triangles'] and np.isfinite(mesh.vertices).all()
 world=mesh.vertices*1000+np.array(m['frames_source_mm'][row['link']])
 bounds=np.r_[world.min(axis=0),world.max(axis=0)]
 # Curved extrema differ slightly due to the explicitly selected tessellation.
 np.testing.assert_allclose(bounds,row['bounds_mm'],atol=.11,rtol=0)
 assert len(r.findall(f"link[@name='{row['link']}']/visual[@name='cad_{row['solid']}']/geometry/mesh"))==1
print('PASS: 153 meshes, units, bounds, tree and CAD reference transforms; no hardware plugin')
