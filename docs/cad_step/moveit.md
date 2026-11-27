# CAD MoveIt 与碰撞体积

独立功能包 `arm_cad_moveit_config`，使用白色真实CAD视觉网格。153个实体分别生成封闭凸包作为碰撞体积（137000三角形），保留零件间隙；单个实体的孔/凹槽会被凸包填充，是偏保守近似。输出详见 collision_volumes.json。

只禁用直接相邻链接碰撞，不继承旧SRDF的大量Never规则。位置IK用于5轴arm组，末端为CAD指尖参考点；无法任意独立指定六维位姿。已增加gripper组、open/close状态及独立模拟控制器。夹爪通过mimic统一为一个驱动自由度，可以使用RViz的Plan & Execute。四连杆线性拟合误差见gripper_mimic_fit.json。详见gripper.md。

## 启动

```bash
source /opt/ros/jazzy/setup.bash
source /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws/install/setup.bash
ros2 launch arm_cad_moveit_config demo.launch.py
```

在RViz选择arm组，起点current，目标可选择cad_zero或review_pose，然后Plan / Execute。Execute仅驱动GenericSystem模拟关节，绝不连接串口。

启动文件将所有子进程置于ROS_DOMAIN_ID=87，与默认域的实机隔离。另开终端查询需要：

```bash
export ROS_DOMAIN_ID=87
ros2 control list_controllers
```

默认不添加桌面/网球；可在RViz Scene Objects加入环境障碍物。底板属于机器人base_link碰撞模型。RViz展开MotionPlanning→Scene Robot，开启Show Robot Collision即可查看碰撞形状（具体显示项以本机界面为准）。

速度0.5 rad/s、加速度0.5 rad/s²及默认缩放0.1只是模拟参数；连续关节不意味着实物无限旋转。禁止直接把配置接到真实驱动，尚需核对实际限位、CAD参考姿态与舵机零位及相机外参。

## 验证

2026-11-26：两个包构建通过；原始CAD零姿态自碰撞检查有效；OMPL成功规划71个轨迹点，ExecuteTrajectory返回SUCCESS。向场景添加与底座重叠的方盒后，状态有效性检查返回碰撞，测试后移除了方盒。全部测试在独立域的模拟控制器完成，没有实机运动。

重新生成顺序：export_step.py会自动调用build_collision.py，因此重新导出不会丢失碰撞凸包引用。

## 关节编辑修正

模拟启动时将CAD的连续关节转换为有界旋转关节（±180°），用于RViz Joints页的角度拖动条；这是模拟编辑范围，不是图纸或实物限位。原CAD URDF保持不变。按住Value列左右拖动修改目标，再Plan/Execute。
