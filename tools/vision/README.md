# 视觉识别、标定与机械臂靠近

完整启动顺序、代码说明、采集流程、故障排查与验证记录见 [操作手册](../../docs/vision_arm_operations.md)。

固定姿态示范默认目标位于 `config/demonstrated_target.json`，与本地大体积采集文件分离。
相机和标定入口依赖本机 Orbbec SDK、Python 虚拟环境和 YOLO 权重；依赖不随仓库分发。

实时视觉运动统一使用 `contact_moveit.py`，见 [接触规划说明](../../docs/contact_moveit.md)。
`contact_path.py` 提供几何、轨迹反转和拼接验证；`tests/` 包含模型和路径测试。
