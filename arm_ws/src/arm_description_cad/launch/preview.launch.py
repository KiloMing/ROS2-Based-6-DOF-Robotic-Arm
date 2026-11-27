from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    share=Path(get_package_share_directory('arm_description_cad'))
    model=(share/'urdf/arm_cad.urdf').read_text()
    remaps=[('/tf','/cad_preview/tf'),('/tf_static','/cad_preview/tf_static')]
    return LaunchDescription([
        Node(package='robot_state_publisher',executable='robot_state_publisher',namespace='cad_preview',parameters=[{'robot_description':model}],remappings=remaps),
        Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui',namespace='cad_preview',parameters=[{'robot_description':model}]),
        Node(package='rviz2',executable='rviz2',namespace='cad_preview',arguments=['-d',str(share/'config/cad.rviz')],remappings=remaps)])
