# CAD 模型接入原 YH-K32 串口驱动

状态：2026-11-26已接入真实反馈，并通过MoveIt执行一次底座+2°测试。用户确认005 raw1500为CAD夹爪零位；比例和方向仍沿用历史候选值。底座实测移动约1.215°，其他关节反馈未变；规划和执行返回成功，但精确到位验证未通过，不能将全机CAD标定记为完成。

## 保留的通信逻辑

复用 `arm_hardware/ArmSystem`，115200 8N1，PRAD读取，`#NNNPddddTmmmm!`运动帧，串口独占、超时重试、反馈时效检查和跟踪检查保持原实现。新CAD模式只改变坐标换算及从动反馈，旧配置仍使用原有米制夹爪转换。

000..004分别为base/shoulder/elbow/wrist_pitch/wrist_rotation；005改为 `gripper_drive_joint`，以弧度计。仅六个主动关节有命令接口；五个从动关节由005实际反馈和URDF mimic系数计算，绝不当作额外舵机发送。启动读取反馈初始化指令，不主动回零。

新文件：
- `arm_cad_moveit_config/config/hardware_calibration.yaml`：每个舵机raw零位、有符号rad/raw比例和raw范围；臂参数沿用历史候选值，夹爪raw零位1500为用户确认值，角度比例沿用0.00235619449 rad/raw候选值。
- `arm_cad_moveit_config/launch/real.launch.py`：CAD+串口启动，默认只读。
- `arm_cad_moveit_config/config/real_controllers.yaml`：50Hz控制循环；轮询6个舵机，不代表每个舵机反馈50Hz。5°到位容差。

换算：`q = (raw - raw_zero) * radians_per_raw`。两个已知CAD角度与实际raw读数可求比例 `(q2-q1)/(raw2-raw1)`，再求零位 `raw1-q1/比例`。不能使用两个raw数而没有对应姿态，也不能把旧毫米开口数直接当转角。

## 需要完成的对应关系

1. 对比模型与实物，明确各关节CAD零位及正方向。旧模型俯仰轴和CAD坐标轴不同，历史符号仅为候选。
2. 至少记录夹爪两个不同开合姿态：对应CAD驱动角度与005反馈raw。包含图纸零位更方便。角度可通过连杆实测或确认一致的CAD姿态获得，不能由当前raw单独推算。
3. 填写profile并核对可用范围后，再将 `calibration_verified` 改为true；不要为了绕过缺失数据直接改标记。

## 完整运动配置（全机映射核实后使用）

```bash
source /opt/ros/jazzy/setup.bash
source /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws/install/setup.bash
export ROS_DOMAIN_ID=0
# 首先只读查看CAD模型与实物反馈是否一致
ros2 launch arm_cad_moveit_config real.launch.py
```

确认映射及机械范围后，停止只读进程，再启动：

```bash
ros2 launch arm_cad_moveit_config real.launch.py enable_motion:=true limits_verified:=true
```

`serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`为默认值。不要同时启动旧real.launch占用同一个串口。CAD模拟demo在域87，与实物域0隔离。`tools/cad/gripper_move.py`仍仅允许模拟域；实物通过该real配置的RViz规划执行，不能把模拟测试脚本当实物标定。

## 验证

`tools/cad/test_hardware_description.py`：缺失/非有限标定拒绝、命令与反馈接口、符号与范围、未核实映射不启用运动。

`arm_hardware`的`cad_mapping`测试使用伪终端，验证真实驱动的CAD raw双向转换、六个舵机编号、五个从动反馈、启动不发送运动、越界拒绝、缺失映射拒绝、旧米制模型兼容及只读不发送运动。`serial_retry`继续覆盖丢包重试及断线。

## 当前底座实物测试模式

```bash
source /opt/ros/jazzy/setup.bash
source /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch arm_cad_moveit_config real.launch.py enable_motion:=true commissioning_base_test:=true
```

该模式不伪造calibration_verified或limits_verified状态。仅底座允许在每次激活时位置的±20 raw内运动（约±2.7°），其余舵机保持激活时位置，不启动夹爪控制器。不是全机自由运动模式。

同一域另开终端，首次测试：

```bash
python3 /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/tools/cad/test_real_base_moveit.py --execute
```

脚本通过MoveIt规划，并检查整个轨迹只有底座约+2°变化，然后交给MoveIt执行。它额外核对反馈位移，不以控制器成功返回作为精确到位证据。同一启动周期不要累加重复测试，否则会超出激活位置范围。

本次证据保存在 `real_base_test_2026-11-26.json`。旧控制器5°容差大于这次2°目标，执行成功但实际少走约0.785°；这证明串口运动链路可用，不证明定位精度。未追加运动补偿，也未验证夹爪或全机正方向。

## 全自由度启用（2026-11-26）

用户明确确认实物运动方向和限制与MoveIt一致。据此将hardware_calibration.yaml的calibration_verified设为true；这是用户核验结果，不是额外自动标定。底座精度测试结果保留不变。

停止旧测试进程后启动：

```bash
source /opt/ros/jazzy/setup.bash
source /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch arm_cad_moveit_config real.launch.py enable_motion:=true limits_verified:=true commissioning_base_test:=false
```

arm组控制五个机械臂关节；gripper组控制一个驱动关节，其余夹爪关节同步跟随。启动读入当前位置，不自动回零或发送规划目标。底座专用测试范围不再生效；原串口反馈检查、关节范围及碰撞检查保留。

## 2026-11-27：纠正接入实物后的反向映射

用户确认当前CAD仿真关节方向正确，接入实物后五个机械臂关节方向相反。修改hardware_calibration.yaml中000..004的有符号rad/raw比例为：000负、001正、002负、003正、004负。005夹爪开合不在本次反转范围，保留原映射。

URDF关节轴、网格、零位raw和串口raw上下限均保持。驱动反馈 `q=(raw-zero)*scale` 与指令 `raw=zero+q/scale` 同时使用新符号；real启动器按换算结果排序生成角度上下限，因此不会出现只改指令不改反馈或上下限倒置的问题。

本次只修改配置并做离线验证，不连接串口、不发送动作。此前全机verified状态已撤回，修正后尚未进行实物方向复核。现有CAD模拟无需重启，后续实物进程需要重新启动才会加载新映射。

2026-11-27：修正后的方向映射已接入真实反馈，用户随后明确要求启用运动控制。使用mapping_accepted_for_control=true记录该授权，calibration_verified仍为false，避免把授权当作独立实测标定。已启动完整arm/gripper控制器；此次启动不发送目标。启动命令沿用上面的全自由度命令。

2026-11-27：用户按上述全自由度启动命令重新启动后报告运行成功。此为用户实测反馈；此前肩关节tracking mismatch记录保留，未据此声称其根因已经通过代码修复。
