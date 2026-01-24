# so101_control_node

SO101実機を制御し、Piper互換I/Fで入出力するノード。

## 入出力
- Subscribe
  - `joint_ctrl_single` (sensor_msgs/JointState)
  - `enable_flag` (std_msgs/Bool)
  - `ik_enable` (std_msgs/Bool)
  - `reload_calibration` (std_msgs/String)
  - `gripper_cmd_topic` (std_msgs/Float32, default: `gripper_cmd`)
  - `save_position` (std_msgs/String)
  - `goto_position` (std_msgs/String)
- Publish
  - `joint_states_single` (sensor_msgs/JointState)
  - `joint_states` (sensor_msgs/JointState)
  - `joint_states_feedback` (sensor_msgs/JointState)
  - `joint_ctrl` (sensor_msgs/JointState)
  - `servo_detail` (std_msgs/String)
- Services
  - `enable_srv` (piper_msgs/Enable)
  - `go_home` (std_srvs/Trigger)

## パラメータ(主要)
- `serial_port` (string)
- `baudrate` (int)
- `driver_backend` (string: `mock` | `feetech`)
- `auto_enable` (bool)
- `calibration_path` (string)
- `urdf_path` (string)
- `gripper_scale` (float)

## 起動例
```bash
ros2 run unity_robot_control so101_control_node --ros-args \
  -r __ns:=/left_arm \
  -p serial_port:=/dev/ttyACM0
```
