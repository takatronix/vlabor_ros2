# unity_tcp_endpoint

Unity側からのTCP通信(ros_tcp_endpoint)を起動・監視するラッパーノード。

## 入出力
- Subscribe
  - `/quest/pose/headset` (geometry_msgs/PoseStamped)
  - `/quest/left_hand/pose` (geometry_msgs/PoseStamped)
  - `/quest/right_hand/pose` (geometry_msgs/PoseStamped)
- Publish
  - なし

## パラメータ
- `ros_ip` (string, default: `0.0.0.0`)
- `ros_tcp_port` (int, default: `42000`)
- `auto_detect_ip` (bool, default: `true`)

## 起動例
```bash
ros2 run unity_robot_control unity_tcp_endpoint
```

## 補足
- 内部で `ros2 run ros_tcp_endpoint default_server_endpoint` を実行します。
- `auto_detect_ip=true` の場合、外部向けIPを自動検出します。
