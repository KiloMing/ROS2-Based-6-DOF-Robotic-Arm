# ROS 2 实物机械臂、视觉靠近与示范复现操作手册

整理日期：2026-11-15。依据本次对话中实际代码与执行记录整理；历史速度调整请求以当前配置为准。

## 1. 当前能力与边界

已经实现：实物 MoveIt 规划执行、Orbbec 深度图与 YOLO 球检测、相机点转底座坐标、近似指尖靠近、手动示范记录、固定目标姿态复现。

尚未完成：精确 TCP 标定、独立验证相机外参、任意球位置的可靠抓取、闭环视觉伺服、抓取接触检测。
`Execution result: 1` 表示轨迹控制成功，不等于真实夹爪对准球，也不表示已抓住球。
MoveIt 只检查其规划场景里已有的碰撞物体；桌面、球、线缆、相机等未建模物体不自动受到保护。

## 2. 本机环境与编译

- 项目：`/home/kiloming/ROS2-Based-6-DOF-Robotic-Arm`
- 工作空间：项目下 `arm_ws`，ROS 2 Jazzy。
- 串口：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，115200 baud；通常映射 `/dev/ttyUSB0`。
- 相机：Orbbec SV1301S_U3；SDK `/home/kiloming/OrbbecSDK_v1.10.37/SDK/lib`。
- 相机 Python：`/home/kiloming/pyorbbecsdk/.venv/bin/python`。
- 默认 YOLO 权重：`/home/kiloming/Documents/Codex/2026-11-13/qign/outputs/tennis_test/yolo11n.pt`。
- 新电脑必须安装 ROS/MoveIt/ros2_control/相机 SDK 与 Python 依赖并调整上述路径；权重和 SDK 不在提交中。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws
colcon build --packages-select arm_description arm_hardware arm_moveit_config
source /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws/install/setup.bash
```

修改硬件库、URDF、launch 或安装配置后需要编译和重启。直接运行的 Python 工具修改后重新运行即可。

## 3. 完整启动顺序：各终端分别运行

开始前在旧程序终端按 Ctrl+C 正常退出，防止重复启动。不要同时启动 mock/demo 与实物控制。

### 终端 1：MoveIt + 实物控制

```bash
source /opt/ros/jazzy/setup.bash
source /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws/install/setup.bash
ros2 launch arm_moveit_config real.launch.py \
  serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  enable_motion:=true limits_verified:=true enable_step_check:=false
```

这是本机实物配置。`limits_verified` 是操作开关，不是程序对物理限位的认证。`enable_step_check:=false` 仅关闭单条指令增量检查，跟随误差、反馈时效和范围检查仍在。

仅查看仿真模型可以使用下列命令，但须先停掉实物启动：

```bash
ros2 launch arm_moveit_config demo.launch.py
```

### 终端 2：相机 TF

```bash
source /opt/ros/jazzy/setup.bash
source /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws/install/setup.bash
ros2 launch arm_moveit_config camera_tf.launch.py
```

### 终端 3：近似指尖 TF

```bash
source /opt/ros/jazzy/setup.bash
source /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws/install/setup.bash
ros2 launch arm_moveit_config fingertip_tf.launch.py
```

### 终端 4：相机与球识别

```bash
bash /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/tools/vision/run.sh
```

Q/Esc 关闭识别窗口；先关闭其他相机或标定程序。自定义权重：

```bash
bash /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/tools/vision/run.sh --model /absolute/path/to/model.pt
```

### 终端 5：相机点转换为底座点

```bash
source /opt/ros/jazzy/setup.bash
source /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws/install/setup.bash
python3 /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/tools/vision/ball_to_base.py
```

转换由 TF2 完成，MoveIt 用于运动规划和运动学计算，不负责替代相机外参标定。

### 终端 6：检查

```bash
source /opt/ros/jazzy/setup.bash
source /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws/install/setup.bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /vision/ball_status --once --full-length
ros2 topic echo /vision/ball_surface_point_base --once
```

三个控制器应 active，状态时间应新鲜，识别 valid 应为 true。没有有效点时 echo 可能一直等待，用 Ctrl+C 退出。

## 4. 两种运动方式：不要混用含义

### A. 固定示范姿态复现（最近验证成功）

入口 `tools/vision/approach_demonstrated_target.py`，默认目标 `tools/vision/config/demonstrated_target.json`。
不依赖球检测，不随球的位置变化，不闭合夹爪。球或底座换位置后不能假定目标仍适用。
从当前关节状态重新请求 MoveIt 规划，轨迹不一定与手动示范路径相同。

```bash
source /opt/ros/jazzy/setup.bash
source /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws/install/setup.bash
# 只规划
python3 /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/tools/vision/approach_demonstrated_target.py
# 规划并执行实物运动
python3 /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/tools/vision/approach_demonstrated_target.py --execute
```

支持 `--target /absolute/path/to/confirmed_target.json`。执行前检查控制器、反馈新鲜度及规划期间关节变化；失败不自动重试，等待超时请求取消。
目标角度（度）：底座 −6.345，肩 49.275，肘 44.955，腕俯仰 39.420，腕旋转 −0.135。
最近一次执行返回 1，肩误差 2.97°、肘误差 2.70°、底座 −0.135°，两腕约 0°。

### B. 视觉接触规划（统一入口）

新版入口为 `tools/vision/contact_moveit.py`，完整参数和模型限制见 [接触规划说明](contact_moveit.md)。
保留上面的固定姿态示范入口，用于重现记录姿态；它不替代实时视觉接触规划。

```bash
source /opt/ros/jazzy/setup.bash
source /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws/install/setup.bash
# 只规划：目标为球面，最后 3 cm 为直线接近段
python3 /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/tools/vision/contact_moveit.py
# 重新获取目标、规划整条路径并执行一次
python3 /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/tools/vision/contact_moveit.py --execute
```

需要重新启动过新版模型的 MoveIt、实物驱动、相机识别、相机 TF 和坐标转换。
不会自动替换为更远目标。逆解成功但末段路径不足 100% 时，不执行部分轨迹。
旧版局部靠近、重复单步和微动实验入口已经移除；历史实现仍可通过 Git 查阅。

## 5. 手动示范采集

固定底座、球和相机安装，用 RViz 发指令调整，停稳后标记。不要手掰通电关节。

```bash
source /opt/ros/jazzy/setup.bash
source /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws/install/setup.bash
python3 /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/tools/vision/record_approach.py
```

记录终端输入后按回车：`status`查看年龄；`s`起点；`m`中间点；`e`终点；`q`正常保存退出。
程序不发送运动命令。保存 joint_states、控制器状态、TF、规划显示轨迹、视觉点、状态、JPEG画面和日志。
输出 `tools/vision/approach_captures/日期时间/`：bag、markers.jsonl、标记图、配置快照、session.json。
标记不保证消息硬件同步；缺失或旧数据会告警但仍保留记录，以便诊断。需人工核对数据年龄。

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag info /absolute/path/to/session/bag
```

不要在实物控制运行时直接回放整包话题。raw bag 和图像保留本地，不纳入代码提交。

## 6. 手眼与末端几何

```bash
bash /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/tools/vision/calibrate.sh --square-mm 18
```

棋盘8×6内部角点，格子18mm。棋盘和底座固定，机械臂变换多个位置及倾角；停稳后在标定窗口按 S 保存。仅保存照片不足以求手眼，需要配对关节/TF。不要与识别程序同时占相机。

当前 camera_handeye.json 来源15组样本，留一误差RMS约13.4mm，未独立验证。
相机安装于 wrist_pitch_link，不随004腕旋转；镜头到指尖水平约110mm、高约30mm仅为近似测量。
fingertip_approx.json 采用测量姿态假设，方向继承腕部，不能当真实抓取TCP。
底座基准到肩轴模型为70mm。固定球多姿态观察应验证底座球坐标稳定性，但不同视角的可见表面点也会变化，不能直接把全部表面点变化归为外参误差。
仓库提供采集器与临时外参结果，不提供完整、独立验证的手眼求解流水线。

## 7. 当前参数及对应代码

| 功能 | 文件 | 当前值/行为 |
|---|---|---|
| 串口驱动 | arm_ws/src/arm_hardware/src/arm_system.cpp | 115200；查询40ms；缺回复最多重试2次；读写错误不重试 |
| 反馈与保护 | 同上及arm_description/urdf/arm.urdf.xacro | 过期1200ms；跟随阈值80raw；单次指令180ms；非认证物理限位 |
| 循环 | arm_moveit_config/launch/real.launch.py | 50Hz，每轮查询一个舵机，6个轮询约8.3Hz/舵机 |
| 控制器容差 | arm_description/config/controllers.yaml | arm终点5°；finger单指1mm；goal_time 3s |
| 规划限速 | arm_moveit_config/config/real_joint_limits.yaml | arm 0.6rad/s、0.1rad/s²；单指0.006m/s、0.001m/s² |
| 运动缩放 | 两个主靠近脚本 | 速度/加速度均0.1；RViz使用其自身设置 |
| 识别 | tools/vision/ball_point.py | YOLO class32；唯一检测且深度可靠才发点 |
| 图像与标记 | capture_helpers.py、record_approach.py | 压缩图像保留检测时间戳；标记保存最新消息及年龄 |
| 坐标变换 | ball_to_base.py | 按检测时间查TF；不自动使用旧变换替代 |
| TF发布 | camera_tf.launch.py、fingertip_tf.launch.py | 读取各自JSON，静态发布 |
| 夹爪命名状态 | my_robot_arm.srdf | 修复 close 名称尾部空格 |

相机点单位米、光学坐标X右Y下Z前；时间戳为主机收帧时间（推理前），不是硬件曝光同步。图像更新不等于深度有效。

## 8. 故障表与重启

| 现象 | 已观察到的原因 | 处理 |
|---|---|---|
| Joint excursion exceeds 0.5 rad | 目标解在边界，路径超限 | 已给求解留0.05rad余量；仍超限则不执行 |
| Execution result -4 | 执行失败的总错误 | 查看硬件和控制器日志，不能仅凭-4判断根因 |
| tracking mismatch | 肩81raw、夹爪82raw曾超80raw | 检查负载/阻力/指令跟随；夹爪已降速，不能保证消除 |
| Missing PRAD response ID 0/5 | 多次查询无回复 | 检查供电/插头/USB；重试耗尽仍停机 |
| No fresh ball/feedback/TF | 无有效深度或控制器停用 | 分别检查状态、时间和TF，不使用旧球坐标 |
| TF future extrapolation持续增大 | 关节状态停在旧时间 | 先恢复硬件；小毫秒级差也可能是消息时序 |
| 串口不存在 | USB尚未识别/掉线 | 检查by-id和lsusb，待设备恢复再启动 |
| 连续操作都需要reset | 硬件ERROR使整套控制器inactive | RViz Reset不能修复硬件状态；先查首次故障 |

```bash
ls -l /dev/serial/by-id/
lsusb
ps -eo pid,args | rg 'real.launch.py|ros2_control_node|ball_point.py'
source /opt/ros/jazzy/setup.bash
ros2 control list_controllers
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link fingertip_approx
```

日志在 `/home/kiloming/.ros/log/`；优先找最新 ros2_control_node 日志的首个 ERROR，再关联 move_group。
正常退出：记录器q回车，相机Q/Esc，launch/转换程序Ctrl+C。等待旧ros2_control_node退出后按第3节重启，每项只启动一个。无需每次重新编译。不自动执行任何运动脚本。

## 9. 数据结论与待办

- 第一组20261115_171842：约487s，14522条关节、6778帧图像，缺camera TF，base点0条；末端标记反馈旧约24s、球点旧约150s。
- 第二组20261115_183758：有end标记、关节年龄约2ms，但球点旧62.22s。可保存关节示范，不能由此精确求球到TCP偏置。
- 固定示范复现已多次执行成功，仍有约3°关节残差；实际空间误差未测量。
- 下一步：独立验证外参与TCP、记录新鲜预抓取球坐标、验证终点误差、处理持续通信故障，再做球心/朝向约束和抓取。

## 10. 验证命令（不下发运动）

```bash
source /opt/ros/jazzy/setup.bash
cd /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/arm_ws
colcon build --packages-select arm_description arm_hardware arm_moveit_config --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select arm_hardware --event-handlers console_direct+
colcon test-result --verbose
cd /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/tools/vision
python3 -m unittest discover -s tests
export LD_LIBRARY_PATH=/home/kiloming/OrbbecSDK_v1.10.37/SDK/lib:${LD_LIBRARY_PATH:-}
/home/kiloming/pyorbbecsdk/.venv/bin/python test_geometry.py
```

串口测试使用伪终端，不占实物串口。持续超时、断开仍应失败。测试通过不代替实物抓取精度验证。

本次整理验证：三个 ROS 包构建通过；串口伪终端测试1项、采集辅助测试2项、视觉几何测试2项均通过；Python语法检查通过。本次整理没有下发实物运动。
