# CAD 夹爪：MoveIt 单驱动同步开合

CAD真实网格保留，六个旋转坐标中只有 `gripper_drive_joint` 独立。其余五个使用URDF mimic，从模型预览、规划到ros2_control反馈都跟随同一个驱动；不再用passive标记代替联动。控制器只接收一个驱动关节。

## RViz操作

启动 `ros2 launch arm_cad_moveit_config demo.launch.py` 后，在MotionPlanning的Planning页选择 `gripper`：

1. Start State选择 `<current>`。
2. Goal State选择 `open`（CAD原装配开口）或 `close`（已验证收拢姿态，非零间隙完全闭合）。
3. 点击Plan查看左右同步动作，再点击Execute，或直接Plan & Execute。
4. 自定义开度时，在Joints页拖动 `gripper_drive_joint` 的Value进度条；其他关节不可独立编辑，但跟随预览。
5. 控制手臂时切回 `arm` 组。

## 同一MoveIt链路的命令行入口

```bash
source /opt/ros/jazzy/setup.bash
source /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws/install/setup.bash
export ROS_DOMAIN_ID=87
ros2 run arm_cad_moveit_config gripper_move.py --close --execute
ros2 run arm_cad_moveit_config gripper_move.py --open --execute
```

去掉 `--execute` 仅规划。该脚本现在通过MoveGroup action请求OMPL规划和执行，不再绕过MoveIt发送六轴手工轨迹。速度/加速度缩放为0.3。

## 几何与拟合精度

STEP圆柱孔轴线（mm，X/Z平面）：左固定支点A=(2521.99416,357.63245)、B=(2531.74416,377.63245)，活动支点C=(2500.88366,378.94786)、D=(2510.44645,398.76081)。右侧镜像。原装配角度为零，不使用此前手工尺寸。

URDF mimic只表达线性关系，CAD四连杆并非严格平行四边形。因此使用 `configure_gripper_mimic.py` 对精确闭环解在0..0.58 rad范围拟合：左右驱动严格反向等角，左右摇杆和夹指也严格镜像同步。1001样本中最大角度偏差0.07347°、闭环位置偏差0.01952 mm。这是明确的运动学近似，不声称非零姿态仍精确闭环；网格和零位装配不变。系数和误差保存在 `gripper_mimic_fit.json`，精确几何求解器 `gripper_linkage.py` 保留作参考。

0..0.58 rad是当前CAD碰撞验证范围，并非实测机械极限或已标定毫米开口。更大开度角之前出现drive/rocker凸包碰撞，不扩大范围。未关闭这些碰撞检查。

## 验证与实物边界

`ROS_DOMAIN_ID=87 python3 tools/cad/test_gripper_moveit.py` 检查101个全机碰撞姿态，然后通过MoveGroup执行收拢、张开，并核对六个反馈坐标符合mimic关系。只运行在CAD模拟域。

后续抓取顺序是arm到预抓取位 → gripper张开 → arm接近 → gripper闭合 → arm抬起。该CAD配置目前使用GenericSystem，尚未接入真实夹爪舵机映射、实际开口/接触力标定，模拟成功不代表实物已抓住物体。
