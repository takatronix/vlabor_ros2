# so101_webui_node

WebUIを提供し、左右アームの状態監視・トルク・キャリブレーション操作を行うノード。

## 入出力
- Subscribe
  - `/left_arm/joint_states_single` ほか各アームのトピック
  - `/left_arm/joint_states_feedback`
  - `/left_arm/ik/joint_angles`
  - `/left_arm/servo_detail`
  - `/left_arm/ik_enable`
  - `/right_arm/*` (同様)
- Publish
  - なし (WebSocket経由でフロントへ送信)

## パラメータ(主要)
- `web_port` (int, default: 8080)
- `arm_namespaces` (list[string], default: `[left_arm, right_arm]`)
- `positions_dir` (string)
- `calibration_dir` (string)
- `urdf_file` (string)
- `limit_warning_deg` (float)

## 起動例
```bash
ros2 run unity_robot_control so101_webui_node --ros-args \
  -p web_port:=8080 \
  -p arm_namespaces:=[left_arm]
```

## 補足
- `aiohttp` が必要です。
