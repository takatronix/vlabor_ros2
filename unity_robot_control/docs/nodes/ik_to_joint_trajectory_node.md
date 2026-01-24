# ik_to_joint_trajectory_node

IKの `JointState` を `JointTrajectory` に変換し、ros2_controlへ送るノード。

## 入出力
- Subscribe
  - `left_ik_topic` (default: `/left_arm/ik/joint_angles`)
  - `right_ik_topic` (default: `/right_arm/ik/joint_angles`)
- Publish
  - `left_command_topic` (default: `/leader/arm_controller/joint_trajectory`)
  - `right_command_topic` (default: `/follower/arm_controller/joint_trajectory`)

## パラメータ
- `command_duration_sec` (float, default: 0.2)
- `use_joint_names_from_msg` (bool, default: true)
- `joint_names` (list[string])

## 起動例
```bash
ros2 run unity_robot_control ik_to_joint_trajectory_node
```
