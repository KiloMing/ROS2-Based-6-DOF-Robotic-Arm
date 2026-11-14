# 机械臂几何与零位记录

更新日期：2026-11-14。对应模型：`arm_ws/src/arm_description/urdf/arm.urdf.xacro`。

## 验证状态与来源

用户在本次对话中反馈模型已成功运行，并提供 RViz/MoveIt 界面截图，可见机械臂模型与末端交互标记。这是用户侧运行反馈，不代表已验证所有规划、轨迹执行或真实机械臂精度。
本次交付文件通过 Xacro 展开及 XML 解析，展开后为11个 link、10个 joint。未在本地执行 ROS 2 完整启动、MoveIt 规划或真实硬件测试。

CAD 数值来自前序对话转录，未独立复核原始 CAD。实测 AC 来自用户最新测量。两者与保留的旧模型占位尺寸分开记录。

## 尺寸（mm）

| 参数 | 当前值 | 含义与证据 |
|---|---:|---|
| base_height | 30.0 | CAD转录的候选底座参考高度 |
| shoulder_height | 38.4 | 暂作 base_joint 到 shoulder_joint 相对Z偏移，基准待核实 |
| upper_arm_length | 105.1 | 候选肩到肘中心距 |
| forearm_length | 87.2 | 候选肘到腕俯仰中心距 |
| wrist_pitch_length | 69.5 | 候选003到004轴线参考点距离，轴线对应待验证 |
| measured_a_to_fingertip_c | 110 | 用户实测 A→夹指前端C；仅记录，不参与几何计算 |
| wrist_rotation_length | 60 | 旧模型占位值，未测量 |
| gripper_palm_length | 35 | 旧模型占位值，未测量 |
| gripper_finger_length | 75 | 旧模型占位值，未测量 |
| gripper_finger_width | 12 | 旧模型占位值 |
| gripper_half_open_max | 30 | 简化模型单侧最大行程，非新实测 |

当前肩轴相对 base_link 原点高度为30+38.4=68.4 mm。若CAD中的38.4是绝对高度，必须重新计算相对偏移，不能直接套用。

A暂对应004腕部旋转输出轴参考点，C为夹指前端。AC=110 mm不是 wrist_rotation_length，不是真实TCP，也不能仅凭长度推导XYZ。当前旧模型A到指尖轴向总长仍为60+35+75=170 mm，与实测不符；保留它是为了避免伪造内部尺寸，末端模型尚未标定。

## 关节、零位与启动姿态

本模型有5个独立旋转关节及1个独立夹爪开合自由度；右夹指跟随左夹指，不能将项目名称中的“6-DOF”理解为6个独立机械臂旋转关节。

| ID | 关节 | 模型局部轴 | 用户确认的零位/参考 | 保留的启动raw | Mock初值 |
|---|---|---|---|---:|---:|
| 000 | base_joint | +Z | raw1600=0° | 1600 | 0 rad |
| 001 | shoulder_joint | +Y | raw1500=0° | 1900 | -0.942477796 rad |
| 002 | elbow_joint | +Y | raw1500=0° | 2100 | 1.413716694 rad |
| 003 | wrist_pitch_joint | +Y | raw1500=0° | 800 | 1.649336144 rad |
| 004 | wrist_rotation_joint | +Z | raw1500=0° | 1500 | 0 rad |
| 005 | left_finger_joint / right_finger_joint | Y平移简化 | raw1500≈35mm总开口 | 1500 | +0.0175 / -0.0175 m |

零位定义与启动姿态不同，001–003保留旧启动姿态，不强制归零。旧角度换算比例为0.00235619449 rad/raw；启动值隐含001/003负方向、000/002/004正方向，均需与硬件驱动核对。URDF本身不执行raw转换。
000按raw500至2500、零位1600及原比例换算后，限位为[-2.591813939, 2.120575041] rad；这不是新测得的机械安全限位。

## 末端、夹爪与接口

- tool0位于wrist_rotation_link的Z+60mm处，仅为旧安装参考占位。
- grasp_frame位于夹掌坐标系Z+(35+75/2)=72.5mm处，即腕旋转坐标系Z+132.5mm；尚非标定TCP。
- 夹爪使用两个prismatic关节，右指以multiplier=-1、offset=0跟随左指。真实机构为齿轮连杆，该简化不能还原真实运动轨迹。
- 35mm总开口在简化模型中对应左右各17.5mm；未建立真实夹爪raw到开口的完整映射。
- 保留原关节/link名称、包结构、ros2_control ArmMockSystem及mock_components/GenericSystem接口。没有修改硬件驱动或MoveIt配置。
- 视觉与碰撞几何为圆柱/方盒近似；底座和肩部支架随更新后的高度同步调整。宽度、半径及其他占位值未重新标定。

## 后续验证

1. 核对CAD坐标原点、肩轴高度基准，以及003/004轴线位置、方向和横向偏移。
2. 在RViz设置000–004关节角为0，比较真实零位；不要把当前非零启动姿态当作全零姿态。
3. 核对硬件驱动的raw零位、方向、比例和限位，逐关节验证。
4. 在明确夹爪开口状态下测量004参考点至安装面、夹指前端及实际接触区域中心的XYZ，解释170mm占位与110mm实测差异。
5. 标定真实TCP和夹爪开口映射后，再验证碰撞、规划与抓取精度。

## 复现静态检查

在已安装xacro和URDF检查工具的ROS环境中：

```sh
xacro arm_ws/src/arm_description/urdf/arm.urdf.xacro -o /tmp/arm.urdf
check_urdf /tmp/arm.urdf
```

本次已运行Xacro展开与Python XML解析；上述check_urdf属于后续可运行检查，未声称本次已执行。
