"""Split source STEP meshes into six four-bar bodies, retaining zero geometry."""
from pathlib import Path
import xml.etree.ElementTree as E
import json
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[2];P=ROOT/'arm_ws/src/arm_description_cad'
r=E.parse(P/'urdf/arm_cad.urdf');root=r.getroot();m=json.loads((ROOT/'docs/cad_step/manifest.json').read_text())
if root.find("joint[@name='gripper_drive_joint']") is not None:raise RuntimeError('Already split; re-export STEP first')
w=np.array(m['frames_source_mm']['wrist_rotation_link']);y=w[1]
bodies=[
('left_drive_link','gripper_drive_joint','wrist_rotation_link',[2521.99416,y,357.63245],[105,106,134,135],0,.65),
('left_rocker_link','left_rocker_joint','wrist_rotation_link',[2531.74416,y,377.63245],[109,112,126,128],0,.65),
('left_finger_link','left_finger_joint','left_drive_link',[2500.88366,y,378.94786],[113,119,120,122,123,131,132],-.68,0),
('right_drive_link','right_drive_joint','wrist_rotation_link',[2549.49416,y,357.63245],[107,144,150,151,152],-.65,0),
('right_rocker_link','right_rocker_joint','wrist_rotation_link',[2539.74416,y,377.63245],[108,110,127,129],-.65,0),
('right_finger_link','right_finger_joint','right_drive_link',[2570.60466,y,378.94786],[111,118,121,124,125,130,133],0,.68)]
frames={'wrist_rotation_link':w};old=root.find("link[@name='wrist_rotation_link']")
for link,joint,parent,frame,ids,lo,hi in bodies:
 frame=np.array(frame);frames[link]=frame;m['frames_source_mm'][link]=frame.tolist();new=E.SubElement(root,'link',name=link)
 for sid in ids:
  name=f'solid_{sid:03d}.stl';mesh=trimesh.load(P/'meshes'/name,force='mesh');mesh.apply_translation((w-frame)/1000);mesh.export(P/'meshes'/name)
  for kind in ['visual','collision']:
   elem=old.find(f"{kind}[@name='cad_{sid}']");assert elem is not None;old.remove(elem);new.append(elem)
  for row in m['solids']:
   if row['solid']==sid:row['link']=link
 j=E.SubElement(root,'joint',name=joint,type='revolute');E.SubElement(j,'parent',link=parent);E.SubElement(j,'child',link=link)
 E.SubElement(j,'origin',xyz=' '.join(map(str,(frame-frames[parent])/1000)),rpy='0 0 0');E.SubElement(j,'axis',xyz='0 1 0')
 E.SubElement(j,'limit',lower=str(lo),upper=str(hi),velocity='.3',effort='1')
m['gripper']='Single-input linear mimic fit of CAD four-bar, 0..0.58 rad; see gripper_mimic_fit.json for approximation error.'
E.indent(r);r.write(P/'urdf/arm_cad.urdf',encoding='utf-8',xml_declaration=True)
(ROOT/'docs/cad_step/manifest.json').write_text(json.dumps(m,indent=2)+'\n')

# Recreate the standard MoveIt single-input coupling after every STEP export.
from configure_gripper_mimic import configure
configure()
