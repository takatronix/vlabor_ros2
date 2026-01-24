# rosbag_to_lerobot

ROS2 Bag を LeRobot 形式へ変換するツール。

## 入力
- ROS2 bag (sqlite3 / mcap)

## 出力
- LeRobot dataset (meta, data, videos)

## 起動例
```bash
ros2 run unity_robot_control rosbag_to_lerobot \
  --bag-path /path/to/bag \
  --output-dir /path/to/output
```

## 補足
- `opencv`, `cv_bridge`, `pyarrow`, `pandas` などが必要です。
