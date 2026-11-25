# Am1 CAD URDF V2 — STEP + DXF 校核记录

本次 V2 采用 STEP 实体网格与 1:2 DXF 正投影视图交叉校核。预览功能包为 `arm_description_cad`，独立于原 arm_description、arm_hardware 与 arm_moveit_config。

**提交状态：** 此提交仅包含几何记录、关节参数表和本地导入脚本；完整模型及二进制 STL 尚未上传。需使用原始 V2 ZIP 导入后另行提交，不应将本文视为功能包已安装。

| 参数 | CAD V2 (mm) |
|---|---:|
| 安装底板长×宽×厚 | 276.941 × 146.897 × 2 |
| 旧基准距底板底面 | 37 |
| base_joint Z | 67 |
| 肩轴距底板底面 | 105.401159 |
| 肩至肘 Z | 105.069692 |
| 肘至腕俯仰 Z | 87.161598 |
| 腕俯仰至004输出面 Z | 72 |
| 004输出面至指尖参考点 Z | 98.97794 |
| 004输出面至齿纹抓持代理点 Z | 94.475949 |

[关节参数表](cad_v2_joint_parameters.csv)

## 导入完整包

从对话下载原始 `Am1_ROS2_CAD_URDF_V2_DXF.zip`，SHA-256 为 `0806d49c67a1879e11bf5905f77d1527e082f54b6e22953e151499231d78fc3f`。
在仓库根目录运行：

```bash
python3 tools/import_cad_v2.py /path/to/Am1_ROS2_CAD_URDF_V2_DXF.zip
git add arm_ws/src/arm_description_cad docs/cad_v2
git diff --cached --stat
git commit -m "feat: import Am1 CAD URDF V2 meshes and DXF analysis"
git push origin main
```

导入脚本核对 ZIP SHA-256，不覆盖已存在目录。完整包导入后，停止旧 ROS 节点，使用 ROS 2 Jazzy：

```bash
source /opt/ros/jazzy/setup.bash
cd arm_ws
colcon build --symlink-install --packages-select arm_description_cad
source install/setup.bash
ros2 launch arm_description_cad preview.launch.py
```

## 限制

- CAD 底板底面不是已经确认的真实桌面基准；历史实测桌面至肩轴为 70 mm。
- 指尖 CAD 值与实测 110 mm 存在约 11 mm 差异。原抓取脚本的 TCP 检查不得绕过。
- 夹爪采用镜像平移近似，闭环连杆尚未实现。碰撞网格为保守凸包。
- CAD 参考姿态不等于实物舵机零位；未运行 ROS2、MoveIt 或实机，不允许直接替代实机模型。
