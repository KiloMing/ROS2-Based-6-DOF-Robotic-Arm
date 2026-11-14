# ROS2-Based-6-DOF-Robotic-Arm

基于 ROS 2 Jazzy、MoveIt 2 与 YH-K32 总线舵机控制板的实体机械臂项目。

当前模型为五个独立旋转关节（000–004）加一个主动夹爪执行器（005）；右夹指为 mimic，并非第六个旋转轴。

## 工程结构

- `arm_ws/src/arm_description`：URDF/Xacro、控制器启动入口；默认使用 GenericSystem 虚拟硬件。
- `arm_ws/src/arm_moveit_config`：MoveIt 2、SRDF、运动规划与轨迹控制器映射。
- `arm_ws/src/arm_hardware`：YH-K32 串口真实硬件插件；默认只读，不执行运动。
- `hardware/servo_calibration.csv`：历史关节标定；000 号实物零位确认是 Raw1600。
- `docs/records/2026-11-14-real-hardware-integration.md`：实机接入流程和待验收项目。

## Ubuntu / ROS 2 Jazzy 构建

    source /opt/ros/jazzy/setup.bash
    cd arm_ws
    colcon build --symlink-install --packages-up-to arm_moveit_config arm_hardware
    source install/setup.bash

## 启动

模拟链路（与之前保持一致）：

    ros2 launch arm_moveit_config demo.launch.py

实机只读调试（先确保供电与机械支撑，且没有其他程序占用串口）：

    ros2 launch arm_description control.launch.py hardware_mode:=real serial_port:=/dev/ttyUSB0 update_rate:=20 start_rviz:=false

另开终端检查 `ros2 control list_controllers` 和 `ros2 topic echo /joint_states`。完整 MoveIt 实机可视化（仍然禁止运动）：

    ros2 launch arm_moveit_config real.launch.py serial_port:=/dev/ttyUSB0

实机运动尚未验收。历史原始范围不是实测安全限位，勿直接启用 `enable_motion:=true`。正式运动前要核对关节方向、反馈、软限位、支持机构和停机方案。不要同时启动 demo 和 real 的控制器。

详见 [进度路线](docs/roadmap.md) 和 [实机驱动记录](docs/records/2026-11-14-real-hardware-integration.md)。
