# SO101 角度変換について

## 概要

SO101ロボットアームの角度変換はシンプルです。**URDF角度のみを使用**します。

## 角度変換

### 基本式

```
rad = (tick - ticks_offset) / ticks_per_rad
tick = rad * ticks_per_rad + ticks_offset
```

### パラメータ

- **ticks_per_rad**: `4096 / (2π) ≈ 651.9` (STS3215: 4096 tick = 360°)
- **ticks_offset**: キャリブレーションで決定される原点tick値
  - `ticks_offset = 2048 - homing_offset`

## キャリブレーション

### homing_offset の意味

```
homing_offset = 2048 - tick_at_home
```

- `tick_at_home`: ロボットをURDFの原点姿勢(rad=0)に置いた時のtick値
- キャリブレーション時に、この姿勢でのtickを記録して算出

### キャリブレーション手順

1. ロボットをURDFの原点姿勢に置く
2. 各関節のtick値を読み取る
3. `homing_offset = 2048 - tick` を計算
4. キャリブレーションJSONに保存

## キャリブレーションファイル

### LeRobotキャリブレーション vs ROS2キャリブレーション

**LeRobot用** (`~/.cache/huggingface/lerobot/...`):
- LeRobotが生成・使用
- `homing_offset` = 可動範囲の中間位置基準
- ROS2では使用しない

**ROS2用** (`so101_description/config/calibration/`):
- `so101_control_node`が使用
- `homing_offset` = URDF原点姿勢基準
- launchファイルのデフォルト

### ファイル形式

```json
{
    "shoulder_pan": {
        "id": 1,
        "drive_mode": 0,
        "homing_offset": 132,
        "range_min": 740,
        "range_max": 3337
    },
    ...
}
```

## データフロー

```
[実機モーター]
     ↓ tick値
[so101_control_node]
     ↓ rad = (tick - ticks_offset) / ticks_per_rad
[/joint_states トピック]
     ↓ URDF角度 (rad)
[robot_state_publisher / IKソルバー / Foxglove]

[IKソルバー / VRコントローラー]
     ↓ URDF角度 (rad)
[/joint_ctrl_single トピック]
     ↓ tick = rad * ticks_per_rad + ticks_offset
[so101_control_node]
     ↓ tick値
[実機モーター]
```

## トラブルシューティング

### 症状: Foxgloveの表示と実機がずれている

**原因**: キャリブレーションがURDFの原点姿勢で行われていない

**解決方法**:
1. 実機をURDFの原点姿勢(全関節 rad=0)に合わせる
2. 各関節のtick値を読み取る
3. `homing_offset = 2048 - tick` を再計算
4. キャリブレーションファイルを更新
5. `reload_calibration`トピックで再読み込み

### 症状: 関節が限界を超える

**原因**: `range_min`/`range_max`が正しく設定されていない

**解決方法**:
1. 各関節の物理的な可動範囲を確認
2. キャリブレーションファイルの`range_min`/`range_max`を更新

## 参考

- STS3215サーボ: 4096 tick = 360° = 2π rad
- 中間位置: 2048 tick
