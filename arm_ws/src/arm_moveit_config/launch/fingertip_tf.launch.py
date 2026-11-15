"""Publish approximate fingertip reference; does not alter collision geometry or plan motion."""
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    path = Path(get_package_share_directory('arm_moveit_config')) / 'config/fingertip_approx.json'
    data = json.loads(path.read_text())
    args = []
    for key, value in zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'),
                          data['translation_m'] + data['quaternion_xyzw']):
        args.extend(['--' + key, str(value)])
    args.extend(['--frame-id', data['parent_frame'], '--child-frame-id', data['child_frame']])
    return LaunchDescription([Node(package='tf2_ros', executable='static_transform_publisher',
                                   name='fingertip_approx_tf', arguments=args, output='screen')])
