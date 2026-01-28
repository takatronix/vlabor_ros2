# LeRobot Recorder

ROS2パッケージ: LeRobotフォーマットでエピソードデータを記録

## 概要

VRテレオペや実機テレオペ中のロボット関節角度とカメラ画像をLeRobot v3形式で保存します。
WebAPIを通じて録画の開始・停止・一時停止を制御できます。

## 機能

- **マルチアーム対応**: シングルアーム、デュアルアームなど柔軟に対応
- **マルチカメラ対応**: 複数カメラの同時録画
- **マルチロボット対応**: SO101, AgileX Piper, その他のロボット
- **WebAPI**: RESTful APIで録画制御
- **LeRobot v3形式**: Parquet + MP4 + メタデータJSON

## 使い方

### Launch

```bash
# デフォルト（so101_dual_vr）
ros2 launch lerobot_recorder lerobot_recorder.launch.py

# プリセット指定
ros2 launch lerobot_recorder lerobot_recorder.launch.py preset:=so101_single_vr
ros2 launch lerobot_recorder lerobot_recorder.launch.py preset:=piper_single_vr
```

### WebAPI

```bash
# ステータス確認
curl http://localhost:8082/api/status

# 録画開始
curl -X POST http://localhost:8082/api/episode/start \
  -H "Content-Type: application/json" \
  -d '{"task": "pick_and_place", "dataset_name": "my_dataset"}'

# 一時停止/再開
curl -X POST http://localhost:8082/api/episode/pause
curl -X POST http://localhost:8082/api/episode/resume

# 録画停止
curl -X POST http://localhost:8082/api/episode/stop

# キャンセル（データ破棄）
curl -X POST http://localhost:8082/api/episode/cancel
```

## プリセット

| プリセット | アーム | アクションソース | カメラ |
|-----------|--------|-----------------|--------|
| so101_dual_vr | left_arm, right_arm | IK | 1 |
| so101_dual_leader | left_arm, right_arm | Leader | 1 |
| so101_single_vr | arm | IK | 1 |
| piper_single_vr | piper | IK | 2 |

## 出力形式

LeRobot v3形式で以下のファイルを生成:

```
~/lerobot_datasets/
└── my_dataset/
    ├── data/
    │   └── train-00000-of-00001.parquet
    ├── videos/
    │   └── observation.images.cam_front/
    │       └── episode_000000.mp4
    ├── meta/
    │   ├── info.json
    │   ├── tasks.json
    │   └── episodes.json
    └── README.md
```

## 詳細設計

[docs/episode_recorder_design.md](docs/episode_recorder_design.md) (旧名称のまま) を参照
