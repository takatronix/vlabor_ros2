# LeRobot設定ガイド - VRテレオペ学習システム

## 概要

このドキュメントでは、VRテレオペレーションで収集したデータをLeRobotで学習するために必要な設定と修正について説明します。

---

## LeRobotが参照する情報

### 1. データセット構造

LeRobotは以下の構造のデータセットを参照します：

```
dataset/
├── meta/
│   ├── info.json           # データセットのメタ情報
│   ├── stats.json          # 統計情報（正規化に使用）
│   ├── episodes.json       # エピソード一覧
│   └── tasks.json          # タスク定義（オプション）
├── data/
│   └── chunk-000/
│       ├── episode_000000.parquet    # エピソードデータ
│       ├── episode_000001.parquet
│       └── ...
└── videos/                           # 動画形式の場合
    └── observation.images.cam_front/
        ├── episode_000000.mp4
        └── ...
```

### 2. info.json の重要フィールド

```json
{
  "codebase_version": "v3.0",
  "robot_type": "so101_follower",
  "fps": 50,
  "features": {
    "observation.state": {
      "dtype": "float32",
      "shape": [12],
      "names": [
        "left_shoulder_pan.pos", "left_shoulder_lift.pos", "left_elbow_flex.pos",
        "left_wrist_flex.pos", "left_wrist_roll.pos", "left_gripper.pos",
        "right_shoulder_pan.pos", "right_shoulder_lift.pos", "right_elbow_flex.pos",
        "right_wrist_flex.pos", "right_wrist_roll.pos", "right_gripper.pos"
      ]
    },
    "action": {
      "dtype": "float32",
      "shape": [12],
      "names": ["...同上..."]
    },
    "observation.images.cam_front": {
      "dtype": "video",
      "shape": [480, 640, 3],
      "video_info": {
        "video.fps": 30,
        "video.codec": "libx264"
      }
    }
  },
  "total_episodes": 100,
  "total_frames": 50000
}
```

### 3. Parquetファイルの必須カラム

| カラム名 | 型 | 説明 |
|---------|-----|------|
| `timestamp` | int64 | タイムスタンプ（ナノ秒） |
| `episode_index` | int64 | エピソード番号 |
| `frame_index` | int64 | フレーム番号 |
| `observation.state` | list[float] | 関節角度（現在状態） |
| `action` | list[float] | 目標関節角度（アクション） |
| `observation.images.cam_front` | dict | 画像参照（video形式の場合） |

### 4. 統計情報（stats.json）

学習時の正規化に使用されます：

```json
{
  "observation.state": {
    "mean": [0.0, 0.0, ...],
    "std": [1.0, 1.0, ...],
    "min": [-3.14, -3.14, ...],
    "max": [3.14, 3.14, ...]
  },
  "action": {
    "mean": [...],
    "std": [...],
    "min": [...],
    "max": [...]
  }
}
```

---

## LeRobot側で必要な修正

### 方式1: 既存ロボット設定を使用（推奨）

VRテレオペの場合、**データセット形式をSO101互換にする**ことで、LeRobotの修正は不要です。

```python
# ROS2 Bag → LeRobot変換時に、SO101互換の命名規則を使用
JOINT_NAMES = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos"
]

# デュアルアームの場合は left_/right_ プレフィックス
DUAL_ARM_JOINT_NAMES = [
    "left_shoulder_pan.pos", "left_shoulder_lift.pos", ...
    "right_shoulder_pan.pos", "right_shoulder_lift.pos", ...
]
```

### 方式2: VRテレオペ専用ロボット設定を追加

新しいロボットタイプとして登録する場合：

#### 1. 設定ファイル作成

```
lerobot/src/lerobot/robots/so101_dual_vr/
├── __init__.py
├── config_so101_dual_vr.py
└── so101_dual_vr.py
```

#### config_so101_dual_vr.py

```python
from dataclasses import dataclass, field
from lerobot.cameras import CameraConfig
from ..config import RobotConfig


@RobotConfig.register_subclass("so101_dual_vr")
@dataclass
class SO101DualVRConfig(RobotConfig):
    """SO101デュアルアーム VRテレオペ設定"""

    # 左アーム設定
    left_arm_port: str = ""
    left_arm_id: str = "left"

    # 右アーム設定
    right_arm_port: str = ""
    right_arm_id: str = "right"

    disable_torque_on_disconnect: bool = True
    max_relative_target: float | dict[str, float] | None = None

    # カメラ設定
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # 角度単位
    use_degrees: bool = False
```

#### so101_dual_vr.py

```python
from functools import cached_property
from typing import Any
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from ..robot import Robot
from .config_so101_dual_vr import SO101DualVRConfig


class SO101DualVR(Robot):
    """SO101デュアルアーム VRテレオペ対応ロボット"""

    config_class = SO101DualVRConfig
    name = "so101_dual_vr"

    JOINT_NAMES = [
        "shoulder_pan", "shoulder_lift", "elbow_flex",
        "wrist_flex", "wrist_roll", "gripper"
    ]

    def __init__(self, config: SO101DualVRConfig):
        super().__init__(config)
        self.config = config

        norm_mode = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100

        # 左アームバス
        self.left_bus = FeetechMotorsBus(
            port=config.left_arm_port,
            motors={name: Motor(i+1, "sts3215", norm_mode)
                    for i, name in enumerate(self.JOINT_NAMES)}
        )

        # 右アームバス
        self.right_bus = FeetechMotorsBus(
            port=config.right_arm_port,
            motors={name: Motor(i+1, "sts3215", norm_mode)
                    for i, name in enumerate(self.JOINT_NAMES)}
        )

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features = {}
        # 左アーム
        for joint in self.JOINT_NAMES:
            features[f"left_{joint}.pos"] = float
        # 右アーム
        for joint in self.JOINT_NAMES:
            features[f"right_{joint}.pos"] = float
        # カメラ
        for cam, cfg in self.config.cameras.items():
            features[cam] = (cfg.height, cfg.width, 3)
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        features = {}
        for joint in self.JOINT_NAMES:
            features[f"left_{joint}.pos"] = float
            features[f"right_{joint}.pos"] = float
        return features

    def get_observation(self) -> dict[str, Any]:
        obs = {}

        # 左アーム読み取り
        left_pos = self.left_bus.sync_read("Present_Position")
        for joint, val in left_pos.items():
            obs[f"left_{joint}.pos"] = val

        # 右アーム読み取り
        right_pos = self.right_bus.sync_read("Present_Position")
        for joint, val in right_pos.items():
            obs[f"right_{joint}.pos"] = val

        return obs

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        left_cmd = {}
        right_cmd = {}

        for key, val in action.items():
            if key.startswith("left_"):
                joint = key.replace("left_", "").replace(".pos", "")
                left_cmd[joint] = val
            elif key.startswith("right_"):
                joint = key.replace("right_", "").replace(".pos", "")
                right_cmd[joint] = val

        self.left_bus.sync_write("Goal_Position", left_cmd)
        self.right_bus.sync_write("Goal_Position", right_cmd)

        return action
```

#### 2. __init__.py への登録

```python
# lerobot/src/lerobot/robots/__init__.py に追加
from .so101_dual_vr.so101_dual_vr import SO101DualVR
```

---

## VRテレオペレーター（Teleoperator）の追加

LeRobotにはVRテレオペレーターがないため、新規作成が必要です。

### 構成

```
lerobot/src/lerobot/teleoperators/vr_quest/
├── __init__.py
├── config_vr_quest.py
└── vr_quest_teleoperator.py
```

### config_vr_quest.py

```python
from dataclasses import dataclass
from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("vr_quest")
@dataclass
class VRQuestConfig(TeleoperatorConfig):
    """VR Quest テレオペレーター設定"""

    # ROS2トピック設定
    left_hand_topic: str = "/quest/left_hand/pose"
    right_hand_topic: str = "/quest/right_hand/pose"
    gesture_topic: str = "/quest/hand_gesture"

    # 座標系変換
    apply_coordinate_transform: bool = True

    # スケーリング
    position_scale: float = 1.0
```

### vr_quest_teleoperator.py

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ..teleoperator import Teleoperator
from .config_vr_quest import VRQuestConfig


class VRQuestTeleoperator(Teleoperator):
    """VR Quest 3 テレオペレーター"""

    config_class = VRQuestConfig
    name = "vr_quest"

    def __init__(self, config: VRQuestConfig):
        super().__init__(config)
        self.config = config

        # ROS2ノード初期化
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('vr_quest_teleoperator')

        # サブスクライバー
        self.left_pose = None
        self.right_pose = None

        self.left_sub = self.node.create_subscription(
            PoseStamped, config.left_hand_topic,
            self._left_callback, 10)
        self.right_sub = self.node.create_subscription(
            PoseStamped, config.right_hand_topic,
            self._right_callback, 10)

    def _left_callback(self, msg):
        self.left_pose = self._pose_to_dict(msg, "left")

    def _right_callback(self, msg):
        self.right_pose = self._pose_to_dict(msg, "right")

    def _pose_to_dict(self, msg, prefix):
        p = msg.pose.position
        o = msg.pose.orientation
        return {
            f"{prefix}_x": p.x * self.config.position_scale,
            f"{prefix}_y": p.y * self.config.position_scale,
            f"{prefix}_z": p.z * self.config.position_scale,
            f"{prefix}_qx": o.x,
            f"{prefix}_qy": o.y,
            f"{prefix}_qz": o.z,
            f"{prefix}_qw": o.w,
        }

    @property
    def action_features(self) -> dict[str, type]:
        # IK計算後の関節角度を返す
        # この実装ではIKは外部ノードで計算される
        return {
            "left_shoulder_pan.pos": float,
            "left_shoulder_lift.pos": float,
            "left_elbow_flex.pos": float,
            "left_wrist_flex.pos": float,
            "left_wrist_roll.pos": float,
            "left_gripper.pos": float,
            "right_shoulder_pan.pos": float,
            "right_shoulder_lift.pos": float,
            "right_elbow_flex.pos": float,
            "right_wrist_flex.pos": float,
            "right_wrist_roll.pos": float,
            "right_gripper.pos": float,
        }

    def get_action(self) -> dict[str, float]:
        rclpy.spin_once(self.node, timeout_sec=0.01)
        # Note: 実際にはIKソルバーからの関節角度を使用
        # ここではROS2トピックから直接取得する想定
        return {}

    def connect(self):
        pass

    def disconnect(self):
        self.node.destroy_node()
```

---

## 学習設定

### SmolVLA学習設定例

`config/training/smolvla_so101_vr.yaml`:

```yaml
# データセット設定
dataset:
  repo_id: "local/so101_vr_teleop"  # ローカルデータセット
  root: "/path/to/dataset"

  # 特徴量の指定
  image_keys: ["observation.images.cam_front"]
  state_keys: ["observation.state"]
  action_keys: ["action"]

# ポリシー設定
policy:
  name: "smolvla"
  pretrained_backbone: "HuggingFaceM4/siglip-so400m-14-364-flash-attn2-navit"

  # チャンクサイズ（50ステップ先まで予測）
  chunk_size: 50

  # 入力次元
  state_dim: 12  # デュアルアーム: 6 x 2
  action_dim: 12

# 学習設定
training:
  output_dir: "outputs/so101_vr_smolvla"
  num_epochs: 100
  batch_size: 32
  learning_rate: 1e-4

  # データ拡張
  image_transforms:
    - type: "color_jitter"
      brightness: 0.2
      contrast: 0.2
    - type: "random_crop"
      crop_size: [320, 320]
```

### 学習コマンド

```bash
# LeRobot学習スクリプト
python lerobot/scripts/train.py \
    --config config/training/smolvla_so101_vr.yaml \
    --dataset.root /path/to/vr_teleop_dataset \
    --training.output_dir outputs/so101_vr_smolvla

# または、ローカルデータセットを使用
python lerobot/scripts/train.py \
    policy=smolvla \
    dataset_repo_id=local \
    dataset.root=/path/to/vr_teleop_dataset \
    training.batch_size=32 \
    training.num_epochs=100
```

---

## データ収集から学習までの流れ

### 1. データ収集（ROS2側）

```bash
# VRテレオペ + データ収集
ros2 launch unity_robot_control vr_dual_arm_teleop.launch.py

# 別ターミナルでROS2 Bag収集
ros2 bag record -o demo_001 \
    /left_arm/joint_states \
    /right_arm/joint_states \
    /left_arm/ik/joint_angles \
    /right_arm/ik/joint_angles \
    /cam_front/image_raw
```

### 2. データ変換

```bash
# ROS2 Bag → LeRobot形式
ros2 run unity_robot_control rosbag_to_lerobot \
    --bag-path demo_001 \
    --output-dir /path/to/lerobot_dataset \
    --task "pick_and_place"
```

### 3. 統計計算

```bash
# LeRobotの統計計算スクリプトを使用
python -c "
from lerobot.datasets.compute_stats import compute_stats
compute_stats('/path/to/lerobot_dataset')
"
```

### 4. 学習

```bash
python lerobot/scripts/train.py \
    policy=smolvla \
    dataset.root=/path/to/lerobot_dataset
```

### 5. 推論（ROS2側）

```bash
# 学習済みモデルで推論
ros2 run so101_ros2_bridge policy_runner_ros2_node --ros-args \
    -p policy_path:=/path/to/outputs/so101_vr_smolvla/checkpoints/best
```

---

## まとめ

| 項目 | LeRobot側の変更 | 説明 |
|------|----------------|------|
| データセット形式 | 不要 | 変換スクリプトで対応 |
| ロボット設定 | オプション | デュアルアーム用に新規作成可能 |
| テレオペレーター | 必要 | VR Quest用を新規作成 |
| 学習設定 | 必要 | YAMLファイル作成 |
| 推論 | 不要 | 既存のPolicy Runnerを使用 |

**推奨アプローチ**:
1. まずは既存のSO101 Follower設定を使用
2. データセット変換スクリプトで互換形式に変換
3. 必要に応じてデュアルアーム用のロボット設定を追加
