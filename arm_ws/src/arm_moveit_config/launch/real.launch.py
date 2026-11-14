"""MoveIt 2 + actual YH-K32 bus servos.

Read-only by default. This launch does not authorize servo movement.
Do not start the simulated demo launcher simultaneously.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description():
    config = (
        MoveItConfigsBuilder("my_robot_arm", package_name="arm_moveit_config")
        .robot_description(mappings={"hardware_mode": "real"})
        .to_moveit_configs()
    )

    control = os.path.join(
        get_package_share_directory("arm_description"), "launch", "control.launch.py"
    )
    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control),
            launch_arguments={
                "hardware_mode": "real",
                "serial_port": "/dev/ttyUSB0",
                "enable_motion": "false",
                "limits_verified": "false",
                "update_rate": "20",
                "start_rviz": "false",
            }.items(),
        ),
    ])
    for extra in (
        generate_static_virtual_joint_tfs_launch(config),
        generate_move_group_launch(config),
        generate_moveit_rviz_launch(config),
    ):
        for action in extra.entities:
            ld.add_action(action)
    return ld
