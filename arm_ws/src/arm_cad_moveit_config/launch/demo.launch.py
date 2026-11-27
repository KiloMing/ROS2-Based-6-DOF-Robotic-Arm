"""CAD-only mock execution. ROS domain 87 isolates this demo from hardware."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    path=Path(get_package_share_directory('arm_description_cad'))/'urdf/arm_cad.urdf'
    cfg=(MoveItConfigsBuilder('am1_step_cad',package_name='arm_cad_moveit_config')
         .robot_description(file_path=str(path)).robot_description_semantic(file_path='config/arm_cad.srdf')
         .robot_description_kinematics(file_path='config/kinematics.yaml')
         .joint_limits(file_path='config/joint_limits.yaml')
         .trajectory_execution(file_path='config/moveit_controllers.yaml')
         .planning_pipelines(pipelines=['ompl']).to_moveit_configs())
    root=ET.parse(path).getroot();control=ET.SubElement(root,'ros2_control',name='CadMock',type='system')
    ET.SubElement(ET.SubElement(control,'hardware'),'plugin').text='mock_components/GenericSystem'
    for joint in root.findall('joint'):
        if joint.get('type') not in ('continuous','revolute'): continue
        # Bounded UI domain enables RViz ProgressBarEditor. Mock only; not CAD-derived limits.
        if joint.get('type') == 'continuous':
            joint.set('type','revolute')
            E_LIMIT = ET.SubElement(joint,'limit',lower='-3.141592653589793',upper='3.141592653589793',effort='1.0',velocity='0.5')
        j=ET.SubElement(control,'joint',name=joint.get('name'))
        if joint.find('mimic') is None:
            ET.SubElement(j,'command_interface',name='position')
        state=ET.SubElement(j,'state_interface',name='position')
        ET.SubElement(state,'param',name='initial_value').text='0'
        ET.SubElement(j,'state_interface',name='velocity')
    cfg.robot_description={'robot_description':ET.tostring(root,encoding='unicode')}
    share=Path(get_package_share_directory('arm_cad_moveit_config'))
    params=cfg.to_dict()
    params.update(publish_robot_description=True,publish_robot_description_semantic=True,
                  publish_planning_scene=True,publish_geometry_updates=True,publish_state_updates=True,publish_transforms_updates=True)
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID','87'),
        DeclareLaunchArgument('rviz',default_value='true'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[cfg.robot_description]),
        Node(package='controller_manager',executable='ros2_control_node',parameters=[str(share/'config/ros2_controllers.yaml')],remappings=[('robot_description','/robot_description')]),
        Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','arm_trajectory_controller','gripper_trajectory_controller','--controller-manager-timeout','30']),
        Node(package='moveit_ros_move_group',executable='move_group',parameters=[params]),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'config/moveit.rviz')],parameters=[params],condition=IfCondition(LaunchConfiguration('rviz')))])
