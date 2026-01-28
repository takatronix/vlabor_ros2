# unity_robot_control

UnityテレオペレーションとSO101制御をまとめたROS2パッケージ。

## 構成
- VR入力 → ターゲット姿勢変換: `vr_dual_arm_control_node`
- IK計算: `ik_solver_node` (左右共通)
- SO101制御: `so101_control_node`
- Unity TCP Endpointラッパー: `unity_tcp_endpoint`
- WebUI: `so101_webui_node`
- ros2_control連携: `ik_to_joint_trajectory_node`

詳細は `docs/nodes/README.md` を参照。

## 起動 (YAMLプロファイル)
`config/launch_profiles.yaml` で起動構成を管理します。詳細は `docs/launch_profiles.md` を参照。

```bash
ros2 launch unity_robot_control teleop_profiles.launch.py \
  profile:=vr_dual_arm_teleop
```

**vlabor から起動する場合**（推奨）: vlabor_ros2 ルートで `./scripts/run so101_vr_dual_teleop`、または `ros2 launch vlabor_launch vlabor.launch.py profile:=so101_vr_dual_teleop`。プロファイル名は `so101_vr_dual_teleop` など（`vlabor_launch` の設定を参照）。

**本パッケージ単体で起動する場合**は `config/launch_profiles.yaml` のプロファイル名を使用する:

主なプロファイル:
- `vr_dual_arm_teleop`: VR双腕テレオペ (WebUI + Foxglove)
- `vr_dual_arm_teleop_3cam`: VR双腕テレオペ + fluent_vision_system
- `teleop_standard`: ros2_controlベースの標準テレオペ
- `remote_robot_only`: リモート現場側 (Unity TCPなし)
- `vr_server_only`: VRサーバーのみ (Unity TCP Endpoint)

### 代表的な引数
- `ros_ip`, `ros_tcp_port`, `auto_detect_ip`
- `left_serial_port`, `right_serial_port`
- `driver_backend` (`mock` | `feetech`)
- `left_calibration_path`, `right_calibration_path`
- `left_webui_port`, `right_webui_port`
- `enable_foxglove`

## 依存関係 (代表)
- `ros_tcp_endpoint` (Unity公式TCP)
- `xacro` (URDF生成)
- `aiohttp` (WebUI)

## Node Docs
`docs/nodes/README.md` に個別READMEをまとめています。
