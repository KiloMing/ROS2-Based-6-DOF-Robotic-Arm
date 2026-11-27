#!/usr/bin/env python3
"""Exact STEP tessellation and CAD-reference five-axis preview, no hardware."""
import argparse,hashlib,json,subprocess,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import cadquery as cq
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[2]
PKG=ROOT/'arm_ws/src/arm_description_cad'
FRAMES={'base_link':[2535.4728,1946.5957,-37.], 'shoulder_mount':[2535.4728,1946.5957,30.], 'upper_arm_link':[2535.744164,1946.482905,68.401159], 'forearm_link':[2535.744164,1946.482905,173.470851], 'wrist_pitch_link':[2535.744164,1946.171777,260.632449], 'wrist_rotation_link':[2535.744164,1946.171777,332.632449]}
GROUPS={'base_link':list(range(1,11))+list(range(28,50))+[17,153], 'shoulder_mount':list(range(11,17))+list(range(18,28))+[50], 'upper_arm_link':list(range(51,73))+list(range(74,85)), 'forearm_link':[73,85,86], 'wrist_pitch_link':list(range(87,100)), 'wrist_rotation_link':list(range(100,153))}

def main():
 p=argparse.ArgumentParser();p.add_argument('step',type=Path);a=p.parse_args()
 solids=cq.importers.importStep(str(a.step)).val().Solids()
 assert len(solids)==153, 'Unexpected STEP: re-audit solid grouping'
 assert sorted(sum(GROUPS.values(),[]))==list(range(1,154))
 robot=ET.Element('robot',name='am1_step_cad');records=[]
 for link,ids in GROUPS.items():
  elem=ET.SubElement(robot,'link',name=link)
  for sid in ids:
   solid=solids[sid-1];verts,faces=solid.tessellate(.10,.15)
   world=np.array([v.toTuple() for v in verts]);vertices=(world-FRAMES[link])*.001
   mesh=trimesh.Trimesh(vertices=vertices,faces=faces,process=False)
   name=f'solid_{sid:03d}.stl';mesh.export(PKG/'meshes'/name)
   for kind in ('visual','collision'):
    node=ET.SubElement(elem,kind,name=f'cad_{sid}')
    ET.SubElement(node,'origin',xyz='0 0 0',rpy='0 0 0')
    geom=ET.SubElement(node,'geometry');ET.SubElement(geom,'mesh',filename=f'package://arm_description_cad/meshes/{name}')
    if kind=='visual':
     mat=ET.SubElement(node,'material',name='cad_white')
     ET.SubElement(mat,'color',rgba='0.95 0.95 0.95 1')
   b=solid.BoundingBox();records.append({'solid':sid,'link':link,'triangles':len(faces),'bounds_mm':[b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax]})
 parents=list(FRAMES)
 for i,name in enumerate(['base_joint','shoulder_joint','elbow_joint','wrist_pitch_joint','wrist_rotation_joint'],1):
  j=ET.SubElement(robot,'joint',name=name,type='continuous')
  ET.SubElement(j,'parent',link=parents[i-1]);ET.SubElement(j,'child',link=parents[i])
  xyz=(np.array(FRAMES[parents[i]])-FRAMES[parents[i-1]])*.001
  ET.SubElement(j,'origin',xyz=' '.join(map(str,xyz)),rpy='0 0 0')
  ET.SubElement(j,'axis',xyz='0 0 1' if i in (1,5) else '1 0 0')
 for name,xyz in [('tool0',[0,0,0]),('cad_tip_reference',[0,.001,.09897794])]:
  ET.SubElement(robot,'link',name=name);j=ET.SubElement(robot,'joint',name=name+'_joint',type='fixed')
  ET.SubElement(j,'parent',link='wrist_rotation_link');ET.SubElement(j,'child',link=name)
  ET.SubElement(j,'origin',xyz=' '.join(map(str,xyz)),rpy='0 0 0')
 ET.indent(robot);ET.ElementTree(robot).write(PKG/'urdf/arm_cad.urdf',encoding='utf-8',xml_declaration=True)
 manifest={'source_sha256':hashlib.sha256(a.step.read_bytes()).hexdigest(),'source_name':a.step.name,'mesh_units':'metres','frames_source_mm':FRAMES,'solids':records,'gripper':'CAD reference shape retained; closed-loop actuation not implemented','joint_limits':'CAD supplies no physical limits. Continuous joints are preview coordinates only.','ownership':'Rigid grouping inferred from housings, horns and brackets; not imported STEP motion mates.'}
 (ROOT/'docs/cad_step/manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 print('Exported',len(records),'solids',sum(x['triangles'] for x in records),'triangles')
 subprocess.run([sys.executable,str(ROOT/'tools/cad/build_gripper.py')],check=True)
 subprocess.run([sys.executable,str(ROOT/'tools/cad/build_collision.py')],check=True)
if __name__=='__main__':main()
