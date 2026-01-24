# SO101 ROS2 I/F 仕様（日本語・初心者向け）

このドキュメントは **SO101** のROS2ノードI/F仕様を、初心者向けに分かりやすく整理したものです。  
本プロジェクトの `so101_control_node` は **Piper互換I/F** で動作します。

---

## 0. まず覚える最小I/F

最短で動かすには以下だけでOKです。

- **送る**: `joint_ctrl_single`  
  → `sensor_msgs/JointState` の `position` に角度を入れて送る
- **見る**: `joint_states_single`  
  → 実機の現在角度が返ってくる
- **有効化**: `enable_flag`（`enable_srv`は任意）

---

## 1. Topics (Publish)

### `joint_states_single` (sensor_msgs/JointState)
実機の**現在角度**。  
まずこれを読むことで「今どこにいるか」がわかります。

### `joint_states_feedback` (sensor_msgs/JointState)
フィードバック用。  
基本は `joint_states_single` と同じ用途でOK。

### `joint_ctrl` (sensor_msgs/JointState)
制御目標のエコー。  
「何を送ったか」の確認に使います。

---

## 2. Topics (Subscribe)

### `joint_ctrl_single` (sensor_msgs/JointState)
**関節角度コマンド**。  
このトピックに `position` を送ると実機が動きます。

### `enable_flag` (std_msgs/Bool)
`True` で有効化、`False` で無効化。

---

## 3. Services

### `enable_srv` (piper_msgs/srv/Enable)
`enable_request: bool` → `enable_response: bool`  
ただし `piper_msgs` のtypesupportが使えない環境では **無効化** されます。

---

## 4. Joint Names (URDF準拠)
`so101_new_calib.urdf` に合わせる：
```
['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
```

---

## 5. Parameters（よく使うもの）

- `driver_backend` : `mock` | `feetech`
  - 実機は `feetech`
- `serial_port` : `/dev/ttyACM0`
- `baudrate` : `1000000`
- `motor_ids` : `[1,2,3,4,5,6]`
- `joint_names` : URDF準拠名
- `calibration_path` : lerobot形式のJSON
- `auto_enable` : `true/false`
- `protocol_version` : `0`
- `ticks_per_rad`, `ticks_offset`, `min_ticks`, `max_ticks`

---

## 6. Calibration JSON（lerobot形式）

例:
```json
{
  "shoulder_pan": {"id": 1, "homing_offset": 132, "range_min": 740, "range_max": 3337},
  "shoulder_lift": {"id": 2, "homing_offset": 496, "range_min": 1264, "range_max": 2394}
}
```

**推奨配置**:
- `~/.cache/huggingface/lerobot/calibration/robots/so101_follower/`
- または `/ros2_ws/config/calibration/so101/`

---

## 7. 動作確認（CLI例）

### 状態を見る
```bash
ros2 topic echo --once /joint_states_single
```

### 有効化
```bash
ros2 topic pub --once /enable_flag std_msgs/msg/Bool "{data: true}"
```

### 関節角度で動かす
```bash
ros2 topic pub --once /joint_ctrl_single sensor_msgs/msg/JointState "{
  name: ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'],
  position: [0.1, 0.2, -0.2, 0.3, 0.0, 0.1]
}"
```

---

## 8. 注意点（実機）

- **キャリブなしだと危険**：range外に行く可能性あり
- 最初は **小さな角度**で動作確認する
- 片腕ずつ確認してから両腕にする
