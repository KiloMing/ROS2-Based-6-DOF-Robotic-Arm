"""Generate a single-input MoveIt gripper; preserve the original STEP meshes.

URDF mimic is linear. Fit the CAD four-bar only on the collision-tested
0..0.58 rad interval; export quantitative errors instead of claiming exact closure.
"""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
from gripper_linkage import NAMES, solve, A, B, C, D

ROOT = Path(__file__).resolve().parents[2]

def configure():
    qs = np.linspace(0., .58, 1001)
    exact = np.array([solve(q)[0] for q in qs])
    factors = qs @ exact / (qs @ qs)
    factors[0] = 1.
    factors[3:] = -factors[:3]
    path = ROOT / 'arm_ws/src/arm_description_cad/urdf/arm_cad.urdf'
    tree = ET.parse(path)
    for name, factor in zip(NAMES, factors):
        joint = tree.getroot().find(f"joint[@name='{name}']")
        for old in joint.findall('mimic'):
            joint.remove(old)
        if name != NAMES[0]:
            ET.SubElement(joint, 'mimic', joint=NAMES[0], multiplier=str(factor), offset='0')
        limit = joint.find('limit')
        limit.set('lower', str(min(0., .58 * factor)))
        limit.set('upper', str(max(0., .58 * factor)))
    ET.indent(tree)
    tree.write(path, encoding='utf-8', xml_declaration=True)
    srdf = ROOT / 'arm_ws/src/arm_cad_moveit_config/config/arm_cad.srdf'
    tree = ET.parse(srdf)
    root = tree.getroot()
    for passive in list(root.findall('passive_joint')):
        if passive.get('name') in NAMES:
            root.remove(passive)
    for state in root.findall("group_state[@group='gripper']"):
        for child in list(state):
            state.remove(child)
        ET.SubElement(state, 'joint', name=NAMES[0], value='.58' if state.get('name') == 'close' else '0')
    ET.indent(tree)
    tree.write(srdf, encoding='utf-8', xml_declaration=True)
    def rot(q):
        return np.array([[np.cos(q), np.sin(q)], [-np.sin(q), np.cos(q)]])
    residuals = []
    for q in qs:
        v = q * factors
        lower = A + rot(v[0]) @ (C-A)
        residuals.append(float(np.linalg.norm(lower + rot(v[0]+v[2]) @ (D-C) - (B + rot(v[1]) @ (D-B)))))
    report = dict(range_rad=[0., .58], samples=len(qs), multipliers=dict(zip(NAMES, factors.tolist())),
                  max_angle_error_deg=float(np.rad2deg(np.max(np.abs(exact-qs[:, None]*factors)))),
                  max_four_bar_closure_error_mm=max(residuals),
                  note='Linear kinematic approximation for MoveIt; unchanged CAD meshes. Not hardware calibration.')
    (ROOT/'docs/cad_step/gripper_mimic_fit.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))

if __name__ == '__main__':
    configure()
