# 工程与学习规则

一次只解决一个能验收的问题。先读相关官方示例，再做最小实验，最后写出自己能解释的结论；读教程不算完成。每次结束记录“测了什么、证据在哪里、下次从哪一步开始”。遇到问题先保留原始日志，列假设，再一次只改一个变量。

以下是规划目录，包尚未创建，不提供假装可运行的启动命令：

```text
arm_ws/src/
  arm_description/    # urdf、meshes、rviz
  arm_hardware/       # SystemInterface 与现有驱动适配
  arm_bringup/        # launch、config，虚拟与真机入口分开
  arm_moveit_config/  # SRDF、运动学、规划和控制器映射
  arm_task/           # 上层任务
firmware/             # 确需维护的 STM32 固件，注明来源和版本
hardware/             # 接线、实物清单、标定记录
 docs/                # 实际位于仓库根目录的文档
```

先独立工作空间验证机械臂，再考虑与底盘合并。依赖固定到 Jazzy 可用版本，记录 RK3588S 的 OS/架构与安装方式，不假设主机上能安装就代表板上能运行。

提交前检查 diff 与暂存文件；一次提交对应一个实验或文档主题。建议使用 `docs:`、`feat:`、`fix:` 前缀，正文写原因与验证结果。不要提交 build/install/log、密钥、个人照片或大体积录包。原始参考文件不改内容，解释另写。真实配置、固件或标定变化后重新运行受影响阶段的验收。共享 main 不强推。

学习资料优先查对应版本：
- [ROS 2 Jazzy 教程](https://docs.ros.org/en/jazzy/Tutorials.html)
- [ros2_control Jazzy](https://control.ros.org/jazzy/index.html)
- [Jazzy API](https://control.ros.org/jazzy/doc/api/hierarchy.html)
- [MoveIt 2 安装](https://moveit.ai/install-moveit2/binary/)

官方软件文档说明接口能力，不证明本机械臂已经实现或通过验收。
