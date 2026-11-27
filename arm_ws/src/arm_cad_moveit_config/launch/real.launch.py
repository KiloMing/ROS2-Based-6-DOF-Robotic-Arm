"""CAD model with the original YH-K32 serial driver; no automatic startup motion."""
from pathlib import Path
import importlib.util
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def setup(context):
    share = Path(get_package_share_directory('arm_cad_moveit_config'))
    helper = importlib.util.spec_from_file_location('cad_hw_description', share/'launch/cad_hardware_description.py')
    module = importlib.util.module_from_spec(helper); helper.loader.exec_module(module)
    value = lambda name: LaunchConfiguration(name).perform(context)
    path = Path(get_package_share_directory('arm_description_cad'))/'urdf/arm_cad.urdf'
    calibration = yaml.safe_load(Path(value('calibration_file')).read_text())
    xml = module.build_description(path, calibration, value('serial_port'),
                                   value('enable_motion').lower() == 'true', value('limits_verified').lower() == 'true',
                                   value('commissioning_base_test').lower() == 'true')
    cfg = (MoveItConfigsBuilder('am1_step_cad', package_name='arm_cad_moveit_config')
           .robot_description(file_path=str(path))
           .robot_description_semantic(file_path='config/arm_cad.srdf')
           .robot_description_kinematics(file_path='config/kinematics.yaml')
           .joint_limits(file_path='config/joint_limits.yaml')
           .trajectory_execution(file_path='config/moveit_controllers.yaml')
           .planning_pipelines(pipelines=['ompl']).to_moveit_configs())
    cfg.robot_description = {'robot_description': xml}
    params = cfg.to_dict()
    params.update(publish_robot_description=True, publish_robot_description_semantic=True,
                  publish_planning_scene=True, publish_geometry_updates=True,
                  publish_state_updates=True, publish_transforms_updates=True)
    controllers = ['joint_state_broadcaster']
    # Read-only inspection exposes measured joints but does not activate motion controllers.
    if value('enable_motion').lower() == 'true':
        controllers += ['arm_trajectory_controller']
        if value('commissioning_base_test').lower() != 'true':
            controllers += ['gripper_trajectory_controller']
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[cfg.robot_description]),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[str(share/'config/real_controllers.yaml')], remappings=[('robot_description', '/robot_description')]),
        Node(package='controller_manager', executable='spawner', arguments=controllers+['--controller-manager-timeout', '30']),
        Node(package='moveit_ros_move_group', executable='move_group', parameters=[params]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share/'config/moveit.rviz')],
             parameters=[params], condition=IfCondition(LaunchConfiguration('rviz')))]

def generate_launch_description():
    share = Path(get_package_share_directory('arm_cad_moveit_config'))
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        DeclareLaunchArgument('calibration_file', default_value=str(share/'config/hardware_calibration.yaml')),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('limits_verified', default_value='false'),
        DeclareLaunchArgument('commissioning_base_test', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup)])
