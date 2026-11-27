"""Per-solid convex volumes preserve gaps between parts and gripper fingers."""
from pathlib import Path
import xml.etree.ElementTree as ET
import json
import trimesh
ROOT=Path(__file__).resolve().parents[2];p=ROOT/'arm_ws/src/arm_description_cad'
out=p/'meshes/collision';out.mkdir(exist_ok=True)
robot=ET.parse(p/'urdf/arm_cad.urdf');records=[]
for node in robot.findall('.//collision/geometry/mesh'):
    name=Path(node.get('filename')).name
    src=trimesh.load(p/'meshes'/name,force='mesh');hull=src.convex_hull
    assert hull.is_watertight and hull.volume>0
    hull.export(out/name)
    node.set('filename','package://arm_description_cad/meshes/collision/'+name)
    records.append({'mesh':name,'faces':len(hull.faces),'volume_m3':hull.volume})
ET.indent(robot);robot.write(p/'urdf/arm_cad.urdf',encoding='utf-8',xml_declaration=True)
(ROOT/'docs/cad_step/collision_volumes.json').write_text(json.dumps(records,indent=2)+'\n')
print(len(records),'closed convex volumes;',sum(x['faces'] for x in records),'triangles')
