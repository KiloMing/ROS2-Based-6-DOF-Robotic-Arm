# 视觉识别、标定与机械臂靠近

完整启动顺序、代码说明、采集流程、故障排查与验证记录见 [操作手册](../../docs/vision_arm_operations.md)。

固定姿态示范默认目标位于 `config/demonstrated_target.json`，与本地大体积采集文件分离。
相机和标定入口依赖本机 Orbbec SDK、Python 虚拟环境和 YOLO 权重；依赖不随仓库分发。

实时视觉运动统一使用 `contact_moveit.py`，见 [接触规划说明](../../docs/contact_moveit.md)。
`contact_path.py` 提供几何、轨迹反转和拼接验证；`tests/` 包含模型和路径测试。

单次张爪、进入、闭爪和提起实验见 [抓取流程说明](../../docs/grasp_ball.md)。

## 固定相机：眼在手外采集（CAD模型）

```bash
bash /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm/tools/vision/calibrate_eye_to_hand.sh
```

固定相机及底座，将8×6内部角点、18mm格长棋盘刚性固定在004之后的末端，采集期间不得改变棋盘相对`wrist_rotation_link`的位置（包括松夹爪）。通过现有MoveIt调整姿态，棋盘完整可见，停稳2秒后点击采集窗口并按S；Q退出。改变俯仰、左右方向和位置，收集15–20组，避免只平移或只绕同一根轴旋转。程序本身没有运动指令接口。

保存文件位于tools/vision/captures/时间目录，每组含原图、角点、内参、棋盘位姿、真实关节反馈和base_link到wrist_rotation_link的TF。mode字段区分旧眼在手上数据，不可混合。使用最新TF并要求机器人静止，时间戳是主机接收时间，不冒充硬件同步。

粗测相机位置(-0.20,+0.23,+0.35)m保存在config/eye_to_hand_initial.json；旋转未知，未将它发布为已标定TF。采集后需要按眼在手外模型求解、验证，不可直接复用旧相机安装变换。

### Fixed external camera: CAD target planning preview (2026-11-27)

`plan_fixed_camera_target.py` uses the CAD `arm` group and `cad_tip_reference`.
By default it calls FK and motion planning and publishes `/display_planned_path`.
Explicit `--execute` sends one freshly planned trajectory via MoveIt ExecuteTrajectory;
it checks fresh feedback and target stability before sending and records the outcome.
It does not command gripper closure. The current `eye_to_hand_draft.json` is an unvalidated
14-sample Park estimate (leave-one-out mean ~17 mm, max ~29 mm). It is not a
production TF or a verified grasp calibration. Older eye-in-hand approach and
recorded-pose scripts are not used by this preview.

After starting the CAD real launch, stop the calibration collector to release
the camera, then start detection in its own terminal:

```bash
export ROS_DOMAIN_ID=0
bash tools/vision/run.sh
```

With a stationary visible ball, in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source arm_ws/install/setup.bash
export ROS_DOMAIN_ID=0
python3 tools/vision/plan_fixed_camera_target.py --standoff 0.05
```

The point topic measures the ball surface, not its centre. The preview target
is 5 cm before that surface along the current CAD tip-to-point line. Only position
is constrained; gripper grasp orientation, opening-dependent TCP, table/scene
geometry and calibration accuracy still require validation before grasping.
`--xyz X Y Z` instead plans directly to a base_link point in metres without camera
input. The JSON output records success/failure, start state and trajectory.

Repeat offline analysis:

```bash
/home/kiloming/pyorbbecsdk/.venv/bin/python tools/vision/analyze_eye_to_hand.py tools/vision/captures/20261127_161358
```


### 2026-11-27 physical trial status

- Motor 000: zero raw 1600, positive radians/raw. Motor 004: zero raw 1500,
  positive radians/raw. Other motor directions unchanged.
- Motor 005: zero raw 1500; close remains 0.58 rad (~1746 raw). Open is
  -0.14844025287 rad (1437 raw). Model lower bound -0.153 rad includes rounding
  of the previous -0.15 rad command to raw 1436; this avoids START_STATE_INVALID.
- One 5 cm standoff arm trajectory and one surface approach trajectory succeeded.
  Closing the gripper then failed: target raw 1545, feedback 1464, tracking error
  81 exceeded limit 80. Hardware/controllers deactivated. No lift or confirmed grasp.
- The calibration remains draft; position-only planning does not ensure grasp
  orientation or clearance from an unmodelled tabletop. A successful controller
  result is not a verified ball-position measurement.

Explicit physical commands (do not run while calibrating with a board in the jaws):

```bash
source /opt/ros/jazzy/setup.bash
source arm_ws/install/setup.bash
export ROS_DOMAIN_ID=0
python3 tools/vision/plan_fixed_camera_target.py --standoff 0.05 --execute
# Separate gripper trials, only when deliberately requested:
python3 tools/vision/gripper_trial.py --open --execute
python3 tools/vision/gripper_trial.py --close --execute
```

The small printable board is `output/pdf/checkerboard_8x6_10mm_A4.pdf`:
8x6 inner corners, 10 mm squares. Print actual size, verify the 50 mm ruler.
Local captures, diagnostic photos and generated trajectories are not committed.
