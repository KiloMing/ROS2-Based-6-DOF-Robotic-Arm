from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("arm_description")
    xacro_path = os.path.join(pkg_dir, "urdf", "arm.urdf.xacro")
    controllers_path = os.path.join(pkg_dir, "config", "controllers.yaml")
    mode = LaunchConfiguration("hardware_mode")
    port = LaunchConfiguration("serial_port")
    motion = LaunchConfiguration("enable_motion")
    verified = LaunchConfiguration("limits_verified")
    update_rate = LaunchConfiguration("update_rate")

    description = ParameterValue(
        Command([
            "xacro ", xacro_path,
            " hardware_mode:=", mode,
            " serial_port:=", port,
            " enable_motion:=", motion,
            " limits_verified:=", verified,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("hardware_mode", default_value="mock"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("limits_verified", default_value="false"),
        DeclareLaunchArgument("update_rate", default_value="100"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description, "use_sim_time": False}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                controllers_path,
                {"update_rate": ParameterValue(update_rate, value_type=int)},
            ],
            remappings=[("robot_description", "/robot_description")],
            output="screen",
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager",
                       "/controller_manager", "--param-file", controllers_path],
            output="screen",
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=["arm_trajectory_controller", "--controller-manager",
                       "/controller_manager", "--param-file", controllers_path],
            output="screen",
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=["gripper_trajectory_controller", "--controller-manager",
                       "/controller_manager", "--param-file", controllers_path],
            output="screen",
        ),
        Node(
            package="rviz2", executable="rviz2",
            condition=IfCondition(LaunchConfiguration("start_rviz")),
            output="screen",
        ),
    ])
