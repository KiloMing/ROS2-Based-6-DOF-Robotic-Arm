
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory("arm_description")

    xacro_file = os.path.join(
        pkg_dir, "urdf", "arm.urdf.xacro"
    )

    # 展开 Xacro，得到完整 URDF 文本
    robot_description = subprocess.check_output(
        ["xacro", xacro_file],
        text=True
    )

    return LaunchDescription([

        # 根据 URDF 和关节位置发布 TF
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": False
            }]
        ),

        # 使用滑块模拟关节角度
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen"
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen"
        ),
    ])
