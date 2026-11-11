
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

    controllers_file = os.path.join(
        pkg_dir, "config", "controllers.yaml"
    )

    # 将 Xacro 转换成 URDF 字符串
    robot_description = subprocess.check_output(
        ["xacro", xacro_file],
        text=True
    )

    # 1. 发布机械臂描述和 TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False
        }]
    )

    # 2. 启动 ros2_control 管理器
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_file],
        remappings=[
            ("robot_description", "/robot_description")
        ]
    )

    # 3. 发布虚拟硬件的关节状态
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_file
        ],
        output="screen"
    )

    # 4. 五个旋转关节的轨迹控制器
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_file
        ],
        output="screen"
    )

    # 5. 夹爪控制器
    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_file
        ],
        output="screen"
    )

    # 6. RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        arm_controller,
        gripper_controller,
        rviz
    ])
