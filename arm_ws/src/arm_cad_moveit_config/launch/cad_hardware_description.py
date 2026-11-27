"""Build CAD ros2_control description using the existing YH-K32 driver."""
import math
import xml.etree.ElementTree as ET

MOTORS = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint',
          'wrist_rotation_joint', 'gripper_drive_joint']
FOLLOWERS = ['left_rocker_joint', 'left_finger_joint', 'right_drive_joint',
             'right_rocker_joint', 'right_finger_joint']

def build_description(path, calibration, port, enable_motion=False, limits_verified=False, commissioning_base_test=False):
    accepted = calibration.get('calibration_verified', False) or calibration.get('mapping_accepted_for_control', False)
    if enable_motion and not commissioning_base_test and not accepted:
        raise ValueError('CAD zero/direction calibration is not verified; check hardware_calibration.yaml')
    if enable_motion and not limits_verified and not commissioning_base_test:
        raise ValueError('Motion requires limits_verified:=true after checking physical ranges')
    root = ET.parse(path).getroot()
    control = ET.SubElement(root, 'ros2_control', name='CadSerial', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'arm_hardware/ArmSystem'
    params = dict(model='cad', port=port, baud='115200', timeout_ms='40',
                  max_feedback_age_ms='1200', max_step_raw='8', enable_step_check='false',
                  max_tracking_raw='80', move_time_ms='180',
                  enable_motion=str(enable_motion).lower(), limits_verified=str(limits_verified).lower(),
                  commissioning_base_test=str(commissioning_base_test).lower())
    for key, value in params.items():
        ET.SubElement(hardware, 'param', name=key).text = value
    for name in MOTORS:
        values = calibration['joints'][name]
        for key in ('raw_zero', 'radians_per_raw', 'raw_min', 'raw_max'):
            if values.get(key) is None:
                raise ValueError(f'Missing CAD calibration: {name}.{key}; current raw alone does not establish CAD zero')
            if not math.isfinite(float(values[key])):
                raise ValueError(f'Nonfinite calibration: {name}.{key}')
        zero, scale = float(values['raw_zero']), float(values['radians_per_raw'])
        low, high = int(values['raw_min']), int(values['raw_max'])
        if not (500 <= low <= zero <= high <= 2500) or low == high or abs(scale) < 1e-8:
            raise ValueError(f'Invalid raw calibration: {name}')
        joint = root.find(f"joint[@name='{name}']")
        bounds = sorted(((low-zero)*scale, (high-zero)*scale))
        limit = joint.find('limit')
        if name == 'gripper_drive_joint':
            bounds = [max(bounds[0], float(limit.get('lower'))), min(bounds[1], float(limit.get('upper')))]
        if bounds[0] >= bounds[1]:
            raise ValueError(f'No valid CAD/raw range intersection: {name}')
        joint.set('type', 'revolute')
        if limit is None:
            limit = ET.SubElement(joint, 'limit', effort='1', velocity='0.3')
        limit.set('lower', str(bounds[0])); limit.set('upper', str(bounds[1]))
        cj = ET.SubElement(control, 'joint', name=name)
        for key in ('raw_zero', 'radians_per_raw', 'raw_min', 'raw_max'):
            ET.SubElement(cj, 'param', name=key).text = str(values[key])
        ET.SubElement(cj, 'command_interface', name='position')
        ET.SubElement(cj, 'state_interface', name='position')
        ET.SubElement(cj, 'state_interface', name='velocity')
    for name in FOLLOWERS:
        cj = ET.SubElement(control, 'joint', name=name, mimic='true')
        ET.SubElement(cj, 'state_interface', name='position')
        ET.SubElement(cj, 'state_interface', name='velocity')
    return ET.tostring(root, encoding='unicode')


def build_semantic(srdf_path, description):
    """Keep gripper named targets within the calibrated hardware bounds."""
    root = ET.parse(srdf_path).getroot()
    limit = ET.fromstring(description).find("joint[@name='gripper_drive_joint']/limit")
    lower, upper = float(limit.get('lower')), float(limit.get('upper'))
    for state in root.findall("group_state[@group='gripper']"):
        for joint in state.findall("joint[@name='gripper_drive_joint']"):
            joint.set('value', str(max(lower, min(upper, float(joint.get('value'))))))
    return ET.tostring(root, encoding='unicode')
