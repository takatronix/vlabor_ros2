# Episode Recorder 設計書

## 概要

LeRobot形式のエピソードデータをROS2トピックからリアルタイムで記録するノード。
VRテレオペ、リーダーテレオペ、ポリシー実行など、様々なデータ収集シナリオに対応。

**上位のプロジェクト管理フレームワークからWebAPI経由で呼び出されることを想定。**

## アーキテクチャ

```
┌─────────────────────────────────────────────────────────────────────────┐
│              プロジェクト管理フレームワーク（外部システム）                  │
│                                                                         │
│   - データセット名の指定                                                  │
│   - タスク・タグの管理                                                    │
│   - 録画の開始/停止制御                                                   │
│   - エピソードのDB管理                                                    │
└─────────────────────────────────────────────────────────────────────────┘
                                │
                                │ WebAPI (HTTP)
                                ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      EpisodeRecorderNode                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                     WebAPI Server (aiohttp)                       │  │
│  │  POST /episode/start  POST /episode/stop  GET /status  ...       │  │
│  └──────────────────────────────────────────────────────────────────┘  │
│                                │                                        │
│                                ▼                                        │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐               │
│  │ State Sub    │   │ Action Sub   │   │ Camera Subs  │               │
│  │ (実測値)      │   │ (指令値)      │   │ (画像)       │               │
│  └──────┬───────┘   └──────┬───────┘   └──────┬───────┘               │
│         │                  │                  │                        │
│         └──────────────────┼──────────────────┘                        │
│                            ▼                                           │
│                    ┌──────────────┐                                    │
│                    │ Frame Buffer │                                    │
│                    │ (時刻同期)    │                                    │
│                    └──────┬───────┘                                    │
│                           │                                            │
│         ┌─────────────────┼─────────────────┐                          │
│         ▼                 ▼                 ▼                          │
│  ┌────────────┐   ┌────────────┐   ┌────────────┐                     │
│  │ Parquet    │   │ PNG/MP4    │   │ Metadata   │                     │
│  │ Writer     │   │ Writer     │   │ Writer     │                     │
│  └────────────┘   └────────────┘   └────────────┘                     │
│                                                                        │
└─────────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
                    ┌──────────────────────┐
                    │  ~/lerobot_datasets/ │  ← 保存先はrecorder側で管理
                    │  (LeRobot Dataset)   │
                    └──────────────────────┘
```

## データフロー（入力）

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         データ収集シナリオ                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  [VRテレオペ]           [リーダーテレオペ]        [ポリシー実行]           │
│       │                       │                       │                 │
│       ▼                       ▼                       ▼                 │
│  IKSolver出力           リーダーアーム           ポリシー出力             │
│  /ik/joint_angles       /leader/joint_states    /policy/action          │
│       │                       │                       │                 │
│       └───────────────────────┴───────────────────────┘                 │
│                               │                                         │
│                               ▼                                         │
│                      【action トピック】                                 │
│                      (設定ファイルで指定)                                │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## 保存場所の管理方針

| 項目 | 管理者 | 指定方法 | 説明 |
|------|--------|----------|------|
| `output_dir` | **recorder側** | 設定ファイル/環境変数 | 保存先ルート（外部から変更不可） |
| `dataset_name` | **外部API** | POST /episode/start | プロジェクト/セッション単位で分ける |
| `episode_index` | **recorder側** | 自動採番 | 連番で管理 |
| `tags` | **外部API** | POST /episode/start | 検索・フィルタ用キーワード |
| `metadata` | **外部API** | POST /episode/start | 任意の追加情報 |

### 保存先の決定順序

```
1. 設定ファイルの output_dir     （最優先）
2. 環境変数 LEROBOT_DATASET_DIR  （設定なしの場合）
3. デフォルト ~/lerobot_datasets  （どちらもない場合）
```

### 最終的なパス構造

```
{output_dir}/                              ← recorder側で管理
└── {dataset_name}/                        ← APIで指定
    ├── meta/
    │   ├── info.json
    │   ├── tasks.json
    │   └── episodes/
    │       └── chunk-000/
    │           └── episodes_000000.parquet
    ├── data/
    │   └── chunk-000/
    │       ├── episode_000000.parquet
    │       ├── episode_000001.parquet
    │       └── ...
    └── videos/
        └── observation.images.{camera_name}/
            ├── episode_000000.mp4
            └── ...
```

## WebAPI 仕様

### ベースURL

```
http://localhost:8082/api
```

### エンドポイント一覧

| Method | Endpoint | 説明 |
|--------|----------|------|
| `POST` | `/episode/start` | 録画開始 |
| `POST` | `/episode/stop` | 録画終了・保存 |
| `POST` | `/episode/pause` | 一時停止 |
| `POST` | `/episode/resume` | 再開 |
| `POST` | `/episode/cancel` | 中断・破棄 |
| `GET` | `/status` | 現在の状態 |
| `GET` | `/config` | 現在の設定 |
| `GET` | `/dataset/info` | データセット情報 |
| `GET` | `/episodes` | エピソード一覧 |

---

### POST /episode/start

録画を開始する。

#### リクエスト

```json
{
    // 必須
    "task": "pick_red_cube",

    // オプション（指定なければ設定ファイルのデフォルト）
    "dataset_name": "cube_picking_v2",

    // タグ・キーワード（検索・フィルタ用）
    "tags": ["training", "success", "operator_tanaka"],

    // 自由メタデータ（任意のkey-value）
    "metadata": {
        "session_id": "sess_12345",
        "difficulty": "easy",
        "robot_id": "so101_unit_03"
    }
}
```

#### レスポンス

```json
{
    "success": true,
    "dataset_name": "cube_picking_v2",
    "episode_index": 42,
    "message": "Recording started"
}
```

#### エラーレスポンス

```json
{
    "success": false,
    "error": "Already recording",
    "state": "recording"
}
```

---

### POST /episode/stop

録画を終了し、エピソードを保存する。

#### リクエスト

```json
{}
```

または追加メタデータを付与：

```json
{
    "tags_append": ["reviewed", "good_quality"],
    "metadata_append": {
        "result": "success",
        "notes": "Clean execution"
    }
}
```

#### レスポンス

```json
{
    "success": true,
    "dataset_name": "cube_picking_v2",
    "episode_index": 42,
    "frames": 450,
    "duration_sec": 15.0,
    "fps_actual": 30.0,
    "task": "pick_red_cube",
    "tags": ["training", "success", "operator_tanaka", "reviewed", "good_quality"],
    "files": {
        "parquet": "cube_picking_v2/data/chunk-000/episode_000042.parquet",
        "videos": {
            "cam_front": "cube_picking_v2/videos/observation.images.cam_front/episode_000042.mp4"
        }
    },
    "size_mb": 12.5
}
```

---

### POST /episode/pause

録画を一時停止する。

#### レスポンス

```json
{
    "success": true,
    "state": "paused",
    "frames_so_far": 123,
    "duration_so_far_sec": 4.1
}
```

---

### POST /episode/resume

一時停止から再開する。

#### レスポンス

```json
{
    "success": true,
    "state": "recording"
}
```

---

### POST /episode/cancel

録画を中断し、データを破棄する。

#### レスポンス

```json
{
    "success": true,
    "message": "Episode cancelled and data discarded",
    "frames_discarded": 123
}
```

---

### GET /status

現在の状態を取得する。

#### レスポンス（録画中）

```json
{
    "state": "recording",
    "dataset_name": "cube_picking_v2",
    "episode_index": 42,
    "task": "pick_red_cube",
    "frame_count": 123,
    "duration_sec": 4.1,
    "fps_target": 30,
    "fps_actual": 29.8,
    "topics": {
        "left_state": {
            "topic": "/left_arm/joint_states",
            "receiving": true,
            "hz": 50.2,
            "last_received_sec_ago": 0.02
        },
        "right_state": {
            "topic": "/right_arm/joint_states",
            "receiving": true,
            "hz": 50.1,
            "last_received_sec_ago": 0.01
        },
        "left_action": {
            "topic": "/left_arm/ik/joint_angles",
            "receiving": true,
            "hz": 30.0,
            "last_received_sec_ago": 0.03
        },
        "right_action": {
            "topic": "/right_arm/ik/joint_angles",
            "receiving": true,
            "hz": 30.0,
            "last_received_sec_ago": 0.02
        },
        "cam_front": {
            "topic": "/cam_front/image_raw",
            "receiving": true,
            "hz": 30.0,
            "last_received_sec_ago": 0.01
        }
    }
}
```

#### レスポンス（待機中）

```json
{
    "state": "idle",
    "dataset_name": null,
    "total_episodes": 42,
    "topics": {
        "left_state": {
            "topic": "/left_arm/joint_states",
            "receiving": true,
            "hz": 50.2
        }
        // ...
    }
}
```

---

### GET /config

現在の設定を取得する。

#### レスポンス

```json
{
    "output_dir": "/home/user/lerobot_datasets",
    "default_dataset_name": "so101_teleop",
    "fps": 30,
    "use_video": true,
    "arms": {
        "left": {
            "state_topic": "/left_arm/joint_states",
            "action_topic": "/left_arm/ik/joint_angles"
        },
        "right": {
            "state_topic": "/right_arm/joint_states",
            "action_topic": "/right_arm/ik/joint_angles"
        }
    },
    "joint_names": [
        "shoulder_pan", "shoulder_lift", "elbow_flex",
        "wrist_flex", "wrist_roll", "gripper"
    ],
    "cameras": {
        "cam_front": {
            "topic": "/cam_front/image_raw",
            "width": 640,
            "height": 480
        }
    },
    "web_port": 8082
}
```

---

### GET /dataset/info

現在のデータセット情報を取得する。

#### リクエスト（クエリパラメータ）

```
GET /dataset/info?dataset_name=cube_picking_v2
```

#### レスポンス

```json
{
    "dataset_name": "cube_picking_v2",
    "path": "/home/user/lerobot_datasets/cube_picking_v2",
    "codebase_version": "v3.0",
    "robot_type": "so101_dual_arm",
    "fps": 30,
    "total_episodes": 42,
    "total_frames": 12600,
    "total_duration_sec": 420.0,
    "features": {
        "observation.state": {
            "dtype": "float32",
            "shape": [12]
        },
        "observation.images.cam_front": {
            "dtype": "video",
            "shape": [480, 640, 3]
        },
        "action": {
            "dtype": "float32",
            "shape": [12]
        }
    }
}
```

---

### GET /episodes

エピソード一覧を取得する。

#### リクエスト（クエリパラメータ）

```
GET /episodes?dataset_name=cube_picking_v2&limit=10&offset=0&tags=training
```

#### レスポンス

```json
{
    "dataset_name": "cube_picking_v2",
    "total": 42,
    "limit": 10,
    "offset": 0,
    "episodes": [
        {
            "episode_index": 0,
            "task": "pick_red_cube",
            "frames": 300,
            "duration_sec": 10.0,
            "tags": ["training", "success"],
            "metadata": {
                "session_id": "sess_001"
            },
            "recorded_at": "2026-01-25T10:30:00Z"
        },
        {
            "episode_index": 1,
            "task": "pick_red_cube",
            "frames": 450,
            "duration_sec": 15.0,
            "tags": ["training", "success"],
            "metadata": {
                "session_id": "sess_001"
            },
            "recorded_at": "2026-01-25T10:31:00Z"
        }
        // ...
    ]
}
```

---

## 状態遷移

```
              cancel
         ┌─────────────────┐
         │                 │
         ▼                 │
     ┌──────┐   start   ┌──────────┐
     │ IDLE │──────────▶│ RECORDING│◀──┐
     └──────┘           └──────────┘   │
         ▲                 │   │       │
         │      stop       │   │ pause │ resume
         │◀────────────────┘   │       │
         │  (save episode)     ▼       │
         │               ┌────────┐    │
         │               │ PAUSED │────┘
         │               └────────┘
         │                    │
         └────────────────────┘
                cancel
```

## LeRobotデータ形式

### データスキーマ

```python
# 1フレームのデータ構造
frame = {
    # 観測（実測値）
    "observation.state": [float] * 12,         # 左6軸 + 右6軸
    "observation.images.cam_front": (H, W, 3), # RGB画像

    # 行動（指令値）
    "action": [float] * 12,                    # 左6軸 + 右6軸

    # メタデータ
    "episode_index": int,
    "frame_index": int,
    "timestamp": float,
}
```

### エピソードメタデータ

```python
# episodes parquet に保存
episode_meta = {
    "episode_index": 42,
    "length": 450,                    # フレーム数
    "task": "pick_red_cube",
    "tags": ["training", "success"],  # 検索用タグ
    "metadata": {...},                # 任意の追加情報
    "recorded_at": "2026-01-25T15:30:00Z",
}
```

### info.json

```json
{
    "codebase_version": "v3.0",
    "robot_type": "so101_dual_arm",
    "fps": 30,
    "features": {
        "observation.state": {
            "dtype": "float32",
            "shape": [12],
            "names": [
                "left_shoulder_pan", "left_shoulder_lift", "left_elbow_flex",
                "left_wrist_flex", "left_wrist_roll", "left_gripper",
                "right_shoulder_pan", "right_shoulder_lift", "right_elbow_flex",
                "right_wrist_flex", "right_wrist_roll", "right_gripper"
            ]
        },
        "observation.images.cam_front": {
            "dtype": "video",
            "shape": [480, 640, 3],
            "video_info": {
                "video.fps": 30,
                "video.codec": "libx264",
                "video.pix_fmt": "yuv420p"
            }
        },
        "action": {
            "dtype": "float32",
            "shape": [12],
            "names": [
                "left_shoulder_pan", "left_shoulder_lift", "left_elbow_flex",
                "left_wrist_flex", "left_wrist_roll", "left_gripper",
                "right_shoulder_pan", "right_shoulder_lift", "right_elbow_flex",
                "right_wrist_flex", "right_wrist_roll", "right_gripper"
            ]
        }
    },
    "total_episodes": 42,
    "total_frames": 12600
}
```

## 設定ファイル

### VRテレオペ用 (`lerobot_recorder_vr.yaml`)

```yaml
lerobot_recorder:
  ros__parameters:
    # === 保存先設定 ===
    output_dir: "~/lerobot_datasets"      # 変更する場合はここ
    default_dataset_name: "so101_vr_teleop"

    # === 記録設定 ===
    fps: 30
    use_video: true   # false=PNG画像のまま保存

    # === アーム設定 ===
    arms:
      left:
        state_topic: "/left_arm/joint_states"
        action_topic: "/left_arm/ik/joint_angles"
      right:
        state_topic: "/right_arm/joint_states"
        action_topic: "/right_arm/ik/joint_angles"

    joint_names:
      - "shoulder_pan"
      - "shoulder_lift"
      - "elbow_flex"
      - "wrist_flex"
      - "wrist_roll"
      - "gripper"

    # === カメラ設定 ===
    cameras:
      cam_front:
        topic: "/cam_front/image_raw"
        width: 640
        height: 480
      # cam_wrist:
      #   topic: "/wrist_camera/image_raw"
      #   width: 320
      #   height: 240

    # === WebAPI ===
    web_port: 8082
```

### リーダーテレオペ用 (`lerobot_recorder_leader.yaml`)

```yaml
lerobot_recorder:
  ros__parameters:
    output_dir: "~/lerobot_datasets"
    default_dataset_name: "so101_leader_teleop"
    fps: 30
    use_video: true

    arms:
      left:
        state_topic: "/left_arm/follower/joint_states"
        action_topic: "/left_arm/leader/joint_states"
      right:
        state_topic: "/right_arm/follower/joint_states"
        action_topic: "/right_arm/leader/joint_states"

    joint_names:
      - "shoulder_pan"
      - "shoulder_lift"
      - "elbow_flex"
      - "wrist_flex"
      - "wrist_roll"
      - "gripper"

    cameras:
      cam_front:
        topic: "/cam_front/image_raw"
        width: 640
        height: 480

    web_port: 8082
```

## パッケージ構成

```
vlabor_ros2/
└── unity_robot_control/
    ├── unity_robot_control/
    │   ├── lerobot_recorder/
    │   │   ├── __init__.py
    │   │   ├── lerobot_recorder_node.py  # メインノード
    │   │   ├── frame_buffer.py           # フレーム同期・バッファ
    │   │   ├── lerobot_writer.py         # Parquet/メタデータ書き込み
    │   │   └── video_encoder.py          # PNG→MP4変換
    │   └── ...
    ├── config/
    │   ├── lerobot_recorder_vr.yaml
    │   └── lerobot_recorder_leader.yaml
    ├── launch/
    │   └── lerobot_recorder.launch.py
    └── setup.py  # エントリポイント追加
```

## 依存関係

### Python packages

```
aiohttp          # WebAPI
cv_bridge        # ROS Image → OpenCV
opencv-python    # 画像処理
pyarrow          # Parquet書き込み
pandas           # DataFrame操作
numpy            # 数値計算
```

### ROS2 packages

```
sensor_msgs      # JointState, Image
std_msgs         # Bool, String
```

### 外部コマンド

```
ffmpeg           # 動画エンコード（MP4生成）
```

## 実装上の懸念点と対策

### 1. トピック同期問題

**問題**: 複数トピック（左右state、左右action、カメラ）のタイムスタンプがずれる

**対策**:
- 固定レート（30fps）でタイマーサンプリング
- 各トピックの最新値を保持
- 欠損時は前フレームの値で補完

### 2. 画像処理の負荷

**問題**: 30fpsで画像を処理すると重い

**対策**:
- 画像は即座にPNGとしてディスクに保存
- 別スレッドでバックグラウンド書き込み
- エピソード終了時にffmpegでMP4エンコード

### 3. メモリ使用量

**問題**: 長いエピソードでメモリ溢れ

**対策**:
- 画像: 即座にディスクへ（メモリに保持しない）
- 数値データ: 軽量なのでメモリに蓄積OK
- 目安: 1000フレーム × 12関節 × 2 × 4bytes ≈ 100KB

### 4. 動画エンコード

**問題**: リアルタイムMP4書き込みは複雑

**対策**:
- 録画中: PNG画像として保存
- エピソード終了時: ffmpegでMP4に変換

```python
def encode_video(image_dir: Path, output_path: Path, fps: int):
    cmd = [
        "ffmpeg", "-y",
        "-framerate", str(fps),
        "-i", str(image_dir / "frame_%06d.png"),
        "-c:v", "libx264",
        "-pix_fmt", "yuv420p",
        str(output_path)
    ]
    subprocess.run(cmd, check=True)
```

### 5. エラーハンドリング

**問題**: 録画中にトピックが途切れる

**対策**:
- `/status` でトピック受信状況をモニタリング
- 一定時間データなしで警告ログ
- 上位システムは定期的にstatusをポーリングして異常検知

## 使用例

### 起動

```bash
# VRテレオペ用
ros2 launch unity_robot_control lerobot_recorder.launch.py \
    config:=lerobot_recorder_vr.yaml

# リーダーテレオペ用
ros2 launch unity_robot_control lerobot_recorder.launch.py \
    config:=lerobot_recorder_leader.yaml
```

### 上位システムからの呼び出し例（Python）

```python
import requests
from datetime import datetime

class EpisodeRecorderClient:
    def __init__(self, base_url="http://localhost:8082/api"):
        self.base_url = base_url

    def start(self, task: str, dataset_name: str = None,
              tags: list = None, metadata: dict = None):
        """録画開始"""
        payload = {"task": task}
        if dataset_name:
            payload["dataset_name"] = dataset_name
        if tags:
            payload["tags"] = tags
        if metadata:
            payload["metadata"] = metadata

        response = requests.post(f"{self.base_url}/episode/start", json=payload)
        return response.json()

    def stop(self, tags_append: list = None, metadata_append: dict = None):
        """録画終了"""
        payload = {}
        if tags_append:
            payload["tags_append"] = tags_append
        if metadata_append:
            payload["metadata_append"] = metadata_append

        response = requests.post(f"{self.base_url}/episode/stop", json=payload)
        return response.json()

    def pause(self):
        """一時停止"""
        return requests.post(f"{self.base_url}/episode/pause").json()

    def resume(self):
        """再開"""
        return requests.post(f"{self.base_url}/episode/resume").json()

    def cancel(self):
        """キャンセル"""
        return requests.post(f"{self.base_url}/episode/cancel").json()

    def status(self):
        """状態取得"""
        return requests.get(f"{self.base_url}/status").json()


# 使用例
client = EpisodeRecorderClient()

# 録画開始
result = client.start(
    task="pick_red_cube",
    dataset_name="project_abc_dataset",
    tags=["training", "operator_tanaka"],
    metadata={
        "session_id": "sess_12345",
        "robot_id": "so101_unit_03"
    }
)
print(f"Started episode {result['episode_index']}")

# 状態確認
status = client.status()
print(f"Recording: {status['frame_count']} frames")

# 録画終了
result = client.stop(
    tags_append=["success"],
    metadata_append={"result": "completed"}
)
print(f"Saved: {result['files']['parquet']}")

# 結果をDBに保存など
db.save_episode({
    "dataset_name": result["dataset_name"],
    "episode_index": result["episode_index"],
    "parquet_path": result["files"]["parquet"],
    "video_paths": result["files"]["videos"],
    "frames": result["frames"],
    "duration_sec": result["duration_sec"],
})
```

### curlでの操作例

```bash
# 録画開始
curl -X POST http://localhost:8082/api/episode/start \
    -H "Content-Type: application/json" \
    -d '{
        "task": "pick_red_cube",
        "dataset_name": "my_dataset",
        "tags": ["training"]
    }'

# ステータス確認
curl http://localhost:8082/api/status

# 一時停止
curl -X POST http://localhost:8082/api/episode/pause

# 再開
curl -X POST http://localhost:8082/api/episode/resume

# 録画停止
curl -X POST http://localhost:8082/api/episode/stop

# キャンセル
curl -X POST http://localhost:8082/api/episode/cancel
```

## 今後の拡張案

1. **WebUI**: ブラウザから録画操作・プレビュー
2. **自動分割**: タイムアウトやトリガーでエピソード自動分割
3. **HuggingFace Hub連携**: 録画後に自動アップロード
4. **リプレイ機能**: 録画したエピソードをロボットで再生
5. **品質チェック**: 録画データの自動検証

---

## まとめ

| 項目 | 内容 |
|------|------|
| 目的 | LeRobot形式でROS2トピックをリアルタイム記録 |
| 対応シナリオ | VRテレオペ、リーダーテレオペ、ポリシー実行 |
| 制御方式 | WebAPI（上位フレームワークから呼び出し） |
| 保存場所 | recorder側で管理（`output_dir`設定） |
| 識別情報 | `dataset_name`, `tags`, `metadata`をAPIで指定 |
| 出力形式 | LeRobot v3（Parquet + MP4） |
