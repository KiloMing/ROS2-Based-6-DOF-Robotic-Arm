# 2026-11-14 — YH-K32 real-hardware integration (M4 code preparation)

## Status

Initial implementation committed to existing main branch. No ROS 2 Jazzy native build, controller activation or live motion has been performed in this session.

User confirmed all IDs 000–005 have working serial communication and ID 000 zero is raw1600. Previously observed PRAD responses for 002, 003 and 004 matched historical startup values. A complete fresh six-axis feedback log remains to be saved.

## Architecture

- Existing model, SRDF, MoveIt controller mappings and demo launch remain in place.
- New arm_hardware SystemInterface uses 115200 8N1, queries #NNNPRAD!, validates exact #NNNPdddd! replies and sends #NNNPddddTdddd! only when explicitly armed.
- Six actual actuators: five revolute joints and left_finger_joint. Right finger is the mirrored mimic state.
- Physical PRAD values seed all six initial controller commands; no motion is sent on activation.
- One position query per control cycle, plus six reads on configuration and activation.
- Read failure propagates an error; command values are not substituted for actual positions.
- Command jumps, tracking deviations, stale feedback and out-of-historical-range values are rejected.
- Movement requires BOTH enable_motion=true and limits_verified=true. The included real.launch.py sets both to false.

## Safety and unverified assumptions

Historical raw limits and raw-to-radian scale are NOT mechanically certified. Motion must not be enabled until each physical limit, direction and zero is verified and a stop strategy is in place. The safety flags are administrative checks, not physical E-stop. Never release torque of supporting joints without support.

The gripper uses historical eight-point opening interpolation. The tool0/grasp_frame geometry is still approximate. Serial feedback age, update latency and controller tracking need hardware measurement before automatic tasks.

## Build and read-only acceptance on Ubuntu ROS 2 Jazzy

From the repository root:

    source /opt/ros/jazzy/setup.bash
    cd arm_ws
    colcon build --symlink-install --packages-up-to arm_moveit_config arm_hardware
    source install/setup.bash
    ros2 pkg prefix arm_hardware
    xacro src/arm_description/urdf/arm.urdf.xacro hardware_mode:=real > /tmp/arm_real.urdf
    check_urdf /tmp/arm_real.urdf

With supported physical arm, no other serial program, and confirmed port:

    ros2 launch arm_description control.launch.py hardware_mode:=real start_rviz:=false update_rate:=20 serial_port:=/dev/ttyUSB0 enable_motion:=false limits_verified:=false

In another sourced terminal:

    ros2 control list_hardware_components
    ros2 control list_controllers
    ros2 topic echo /joint_states

Never run tools/servo_query_probe.py while ros2_control owns the serial device. Confirm all six motor positions and the mirrored seventh state reflect the physical arm before issuing any trajectory.

## Hold point before MoveIt execution

- [ ] Native Jazzy build and Xacro expansion pass
- [ ] Mock Plan/Execute regression passes
- [ ] Real hardware publishes correct, changing joint states
- [ ] Raw zero, signs, measured joint limits and gripper range are verified
- [ ] Physically safe stop procedure and stable arm support demonstrated
- [ ] Single-joint low-speed trajectory test passes
- [ ] Multi-joint trajectory and MoveIt real-mode execution verified

After read-only validation, real.launch.py can be tested with ros2 launch arm_moveit_config real.launch.py (it still cannot send motion). Do not launch demo.launch.py concurrently.
