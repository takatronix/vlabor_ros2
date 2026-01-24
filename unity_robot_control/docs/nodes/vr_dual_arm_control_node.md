# vr_dual_arm_control_node

VR(Quest 3)の左右手入力を、左右アームのターゲット姿勢へ変換するノード。

## 入出力
- Subscribe
  - `left_hand_topic` (default: `/quest/left_controller/pose`)
  - `right_hand_topic` (default: `/quest/right_controller/pose`)
  - `joystick_topic` (default: `/quest/joystick`)
  - `left_gesture_topic` (default: `/quest/hand_gesture/left`)
  - `right_gesture_topic` (default: `/quest/hand_gesture/right`)
- Publish
  - `left_arm_target_topic` (default: `/left_arm/target_pose`)
  - `right_arm_target_topic` (default: `/right_arm/target_pose`)
  - `left_arm_enable_topic` (default: `/left_arm/enable_flag`)
  - `right_arm_enable_topic` (default: `/right_arm/enable_flag`)
  - `gripper_left_topic` (default: `/left_arm/gripper_cmd`)
  - `gripper_right_topic` (default: `/right_arm/gripper_cmd`)

## サービス
- `reset_origin` (std_srvs/Trigger, 相対モード時のみ)

## パラメータ(主要)
- `position_scale` (float)
- `left_arm_offset_xyz`, `right_arm_offset_xyz` (list[float])
- `enable_relative_mode` (bool)
- `enable_grip_to_move` (bool)
- `enable_gripper_control` (bool)
- `enable_position_clamp` (bool)
- `position_min_xyz`, `position_max_xyz` (list[float])

## 起動例
```bash
ros2 run unity_robot_control vr_dual_arm_control_node
```

## 補足
- 詳細パラメータは `unity_robot_control/vr_dual_arm_control_node.py` を参照。
