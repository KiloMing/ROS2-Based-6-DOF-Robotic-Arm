# Am1 原始 STEP 网格模型（2026-11-26）

依据用户提供的 `三维STEP通用格式/Am1完整体.STEP` 和 `二维CAD图纸/Am1完整体.DXF`。新模型不使用此前手工测量的70/110 mm，不使用旧方盒夹指、旧零位补偿或历史关节限位。源文件哈希、逐实体尺寸和网格信息见 manifest.json；DXF毫米单位及1:2文字记录见 dxf_inventory.json。

## 已完成

- STEP 的153个实体全部导出为153份STL，保留孔、齿、支架、紧固件和夹爪参考装配外形。曲面以0.10 mm线性/0.15 rad角度参数离散，非方盒或圆柱替代。
- 视觉网格共525762个三角形；碰撞几何按153个实体分别生成封闭凸包，共137000个三角形。孔和凹槽会被单个实体凸包填充，实体之间的间隙保留。
- 原始CAD轴向保留：肩/肘/腕俯仰沿X，底座/腕旋转沿Z；与旧模型绕Y的坐标约定不同。不能直接套用旧相机TF和舵机角度。
- 模型原点取底板底面、底座旋转轴投影。肩轴高105.401159 mm，肩肘105.069692 mm，肘腕87.161598 mm，腕到004输出面72 mm。指尖参考点来自CAD，约98.97794 mm，不再写入110 mm。
- 原始全局坐标和刚体分组均公开在导出脚本及清单中。圆柱轴位置通过舵盘实体51/52、74/75、87/88、100核对。底座轴通过11/18圆盘核对。

## 当前边界（不能称作完整实机控制模型）

STEP提供几何装配而非可直接复用的运动配合。六组刚体归属根据舵机壳体、舵盘及支架推定，必须逐关节复核；零角时所有实体重组与原STEP一致。

夹爪保留六个真实旋转坐标，通过单驱动加五个mimic接入MoveIt。RViz可直接规划和执行左右同步开合。CAD四连杆线性拟合的最大闭环偏差0.020 mm，原网格不变；尚未建立真实舵机映射。详见 gripper.md。

CAD不提供舵机实际限位/零位、材料密度或电机参数。因此五个连续关节仅作预览坐标，不表示真实舵机可以无限旋转。未捏造惯量，不含ros2_control。新模型已加入独立的arm_cad_moveit_config模拟规划配置，尚未接入旧抓取程序，也未进行实机验证。

## 启动

```bash
source /opt/ros/jazzy/setup.bash
cd /home/kiloming/ROS2-Based-6-DOF-Robotic-Arm
colcon build --base-paths arm_ws/src --build-base arm_ws/build --install-base arm_ws/install --packages-select arm_description_cad --symlink-install
source arm_ws/install/setup.bash
ros2 launch arm_description_cad preview.launch.py
```

预览使用/cad_preview命名空间及独立TF主题；滑块仅改变CAD关节姿态，不发送硬件动作。查看原始装配时全部关节设为0。

## 复现导出

使用独立Python环境安装cadquery、trimesh；本次版本cadquery2.8.0、cadquery-ocp7.9.3.1.1、trimesh5.2.2，DXF读取使用ezdxf1.4.5。

```bash
python tools/cad/export_step.py '/实际路径/Am1完整体.STEP'
python tools/cad/verify_meshes.py
check_urdf arm_ws/src/arm_description_cad/urdf/arm_cad.urdf
```

导出脚本只适用于该153实体装配，不能泛用于其它STEP。清单不是第三方网格的授权声明，原图纸权利归原作者；本次没有自动上传原始CAD或推送GitHub。
