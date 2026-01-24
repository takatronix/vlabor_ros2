# ik_solver_node

IK(逆運動学)を解いて `JointState` を出力するノード。左右アーム用に同一実装が使われます。

## 入出力
- Subscribe
  - `target_pose_topic` (default: `/target_pose`)
  - `joint_states_topic` (default: `/joint_states`)
- Publish
  - `joint_angles_topic` (default: `/ik/joint_angles`)

## パラメータ(主要)
- `urdf_file` (string)
- `base_link` (string)
- `end_effector_link` (string)
- `joint_names` (list[string])
- `use_orientation` (bool)
- `ik_max_iterations` (int)
- `ik_tolerance` (float)
- `ik_damping` (float)
- `enable_delta_limit` (bool)
- `max_joint_delta_rad` (float)

## 起動例
```bash
ros2 run unity_robot_control ik_solver_node
ros2 run unity_robot_control left_arm_ik_solver_node
ros2 run unity_robot_control right_arm_ik_solver_node
```

## 補足
- `left_arm_ik_solver_node` / `right_arm_ik_solver_node` は同じ実装で、パラメータのみ異なります。
