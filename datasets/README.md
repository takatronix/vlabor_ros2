# datasets

LeRobot 形式のデータセット（録画データ）を置くディレクトリです。

## 用途

- `lerobot_recorder` の録画保存先（`output_dir`）
- 学習用データの格納・共有

## 出力形式

LeRobot v3 形式:

```
datasets/
└── {dataset_name}/
    ├── data/
    │   └── chunk-000/
    ├── videos/
    │   └── observation.images.*/
    └── meta/
```

## ネットワーク共有

このフォルダは SMB 等で共有可能です。設定は [docs/smb_sharing.md](docs/smb_sharing.md) を参照してください。

## Recorder でこのフォルダを使う場合

`output_dir` を `vlabor_ros2/datasets` に合わせたプリセットを使うか、`config_file` で上書きしてください。

例（config_file で指定）:

```yaml
episode_recorder:
  ros__parameters:
    output_dir: "/home/ros2/ros2_ws/src/vlabor_ros2/datasets"
    default_dataset_name: "so101_dual_vr"
    # ... 他のパラメータ
```

あるいはプリセット `so101_dual_vr_shared` を使用:

```bash
ros2 launch lerobot_recorder episode_recorder.launch.py preset:=so101_dual_vr_shared
```

## 運用

- 動画・Parquet 等で大容量のため、リポジトリでは `datasets/*` を .gitignore 対象にしています
- 共有ストレージに置いて、録画マシンと学習マシンで共有する運用を想定しています
