# Unity Robot Control Package

Unityテレオペレーションとロボット制御パッケージ

## 📋 概要

このパッケージは以下を統合します：
- Unity TCP Endpoint（Unityとの通信）
- SO101ロボット制御ノード（Piper互換I/F）
- IK（逆運動学）計算ノード

## 🗂️ パッケージ構成

```
unity_robot_control/
├── unity_robot_control/
│   ├── __init__.py
│   ├── unity_tcp_endpoint.py    # Unity TCP Endpointラッパー
│   ├── so101_control_node.py    # SO101制御ノード（Piper互換I/F）
│   └── ik_solver_node.py        # IK計算ノード
├── launch/
│   └── unity_robot_control.launch.py
├── config/
├── package.xml
├── setup.py
├── CMakeLists.txt
└── README.md
```

## 🚀 使用方法

### ビルド

```bash
cd ~/ros2_ws
colcon build --packages-select unity_robot_control
source install/setup.bash
```

### 起動

```bash
# 全ノードを起動
ros2 launch unity_robot_control unity_robot_control.launch.py

# IPアドレスを指定
ros2 launch unity_robot_control unity_robot_control.launch.py ros_ip:=192.168.1.11
```

### 個別起動

```bash
# Unity TCP Endpointのみ
ros2 run unity_robot_control unity_tcp_endpoint

# SO101制御ノードのみ
ros2 run unity_robot_control so101_control_node

# IK計算ノードのみ
ros2 run unity_robot_control ik_solver_node
```

## 📡 トピック

### サブスクライブ
- `joint_ctrl_single` (sensor_msgs/JointState) - Piper互換の関節コマンド（namespace: `left_arm`/`right_arm`）
- `enable_flag` (std_msgs/Bool) - 有効/無効切り替え

### パブリッシュ
- `joint_states_single` (sensor_msgs/JointState) - 実機/モックの関節状態
- `joint_states_feedback` (sensor_msgs/JointState) - フィードバック
- `joint_ctrl` (sensor_msgs/JointState) - 最終コマンドのエコー
- `/left_arm/ik/joint_angles` (sensor_msgs/JointState) - 左アームIK計算結果（デフォルト）
- `/right_arm/ik/joint_angles` (sensor_msgs/JointState) - 右アームIK計算結果（デフォルト）

### サービス
- `enable_srv` (piper_msgs/srv/Enable) - 有効/無効切り替え

## 🖥️ WebUI

WebUIは `/left_arm` `/right_arm` の状態表示とトルク/IK切替に加え、キャリブレーション開始に対応します。

- **Calibration** セクションに `port` / `id` / `type` を入力して `Calibrate` を押すと、`lerobot-calibrate` を起動します
- 生成されたJSONのパスは WebUI の Calibration 欄に表示されます（`servo_detail` の `calibration_file`）

注意:
- `lerobot-calibrate` が PATH に無い場合は、`python -m lerobot.scripts.lerobot_calibrate` にフォールバックします
- GUIが起動するため、実行環境にディスプレイが必要です

## 🔧 SO101実機ドライバ（scservo_sdk）
- `driver_backend=feetech` でscservo_sdkを使用
- 主要パラメータ:
  - `serial_port`: `/dev/ttyACM0`
  - `baudrate`: `1000000`
  - `motor_ids`: `[1,2,3,4,5,6]`
  - `joint_names`: `['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']`
  - `ticks_per_rad`, `ticks_offset`, `min_ticks`, `max_ticks`
  - `protocol_version`: `0`
  - `calibration_path`: lerobotのキャリブJSON

インストール例:
```bash
pip install scservo_sdk
```

キャリブJSON例:
```json
{
  "shoulder_pan": {"id": 1, "homing_offset": -1206, "range_min": 1720, "range_max": 2418},
  "shoulder_lift": {"id": 2, "homing_offset": 987, "range_min": 1706, "range_max": 2392}
}
```

## 🔧 実装状況

- [x] Unity TCP Endpointラッパー
- [x] SO101制御ノード（Piper互換I/F）
- [ ] IK計算ノード（実装中）
