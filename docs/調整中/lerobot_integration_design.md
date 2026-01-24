# VRテレオペ × LeRobot 学習システム設計書

## 概要

VR（Quest 3）テレオペレーションでロボットを操作し、そのデモンストレーションデータをLeRobotで学習させるシステムの設計書。

## 目標

1. **VRテレオペ** でSO101デュアルアームロボットを操作
2. **デモデータ収集** （関節角度、カメラ画像、VRポーズ）
3. **LeRobot形式に変換** してデータセット作成
4. **模倣学習** （SmolVLA等）でポリシーを学習
5. **学習済みポリシー** でロボットを自律制御
6. **Mujocoシミュレーション** で事前検証・追加学習

---

## システム全体アーキテクチャ

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           学習フェーズ                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐  │
│  │  VR Quest 3 │───>│  ROS2 Bag   │───>│  LeRobot    │───>│  SmolVLA    │  │
│  │  テレオペ    │    │  データ収集  │    │  Dataset    │    │  学習       │  │
│  └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘  │
│         │                                                        │          │
│         v                                                        v          │
│  ┌─────────────┐                                         ┌─────────────┐   │
│  │  SO101      │                                         │  学習済み    │   │
│  │  実機       │                                         │  モデル      │   │
│  └─────────────┘                                         └─────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                           推論フェーズ                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐  │
│  │  カメラ画像  │───>│  SmolVLA    │───>│  ROS2       │───>│  SO101      │  │
│  │  関節状態    │    │  推論       │    │  Control    │    │  実機       │  │
│  └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                           シミュレーションフェーズ                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                     │
│  │  Mujoco     │<──>│  ROS2       │<──>│  VR/Policy  │                     │
│  │  シミュレータ │    │  Bridge     │    │  Control    │                     │
│  └─────────────┘    └─────────────┘    └─────────────┘                     │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Unity側のトピック構成

### unity_ros_teleoperation からのデータ

| トピック | 型 | 説明 |
|---------|-----|------|
| `/quest/left_hand/pose` | `PoseStamped` | 左手手首の位置・姿勢 |
| `/quest/right_hand/pose` | `PoseStamped` | 右手手首の位置・姿勢 |
| `/quest/left_hand/joints` | `PointCloud` | 左手21関節の3D座標 |
| `/quest/right_hand/joints` | `PointCloud` | 右手21関節の3D座標 |
| `/quest/hand_gesture` | `HandGesture` | ジェスチャー（ピンチ等） |

### 座標系
- **FLU (Front-Left-Up)** ROS標準座標系に変換済み
- **基準フレーム**: `quest_origin`（VRプレイスペース原点）

---

## データ収集パイプライン

### Phase 1: ROS2 Bagデータ収集

```
VR Quest 3
    │
    ├─ /quest/left_hand/pose
    ├─ /quest/right_hand/pose
    ├─ /quest/hand_gesture
    │
    ▼
ROS2 Topic
    │
    ├─ /left_arm/joint_states      (IK計算後)
    ├─ /right_arm/joint_states     (IK計算後)
    ├─ /left_arm/cam_front/image_raw
    ├─ /right_arm/cam_front/image_raw
    │
    ▼
ros2 bag record (sdr_node)
    │
    ▼
.db3 ファイル
```

### 収集するデータ

| データ | 用途 | 頻度 |
|--------|------|------|
| 関節角度（左右アーム） | 状態・アクション | 50Hz |
| カメラ画像（前面） | 視覚入力 | 30Hz |
| VR手ポーズ | 参考データ | 50Hz |
| グリッパー状態 | アクション | 50Hz |
| タスク開始/終了マーカー | エピソード分割 | イベント |

---

## 既存のROS2-LeRobot連携プロジェクト

**重要**: すでにROS2とLeRobotを連携するプロジェクトが存在します。車輪の再発明を避けるため、これらを活用することを推奨します。

### 1. rosetta - ROS 2 ⇄ LeRobot Bridge（推奨）

**GitHub**: https://github.com/sea-bass/rosetta

| 機能 | 説明 |
|------|------|
| **EpisodeRecorderServer** | ROS2アクションでrosbag2にエピソードを記録。タスク情報をメタデータに保存 |
| **bag_to_lerobot.py** | rosbag2をLeRobot v3形式（Parquet + MP4）に変換 |
| **PolicyBridge** | 学習済みLeRobotモデルをROS2ロボットで実行 |
| **Contract YAML** | 観測/動作トピックを統一的に定義 |

```yaml
# rosetta contract.yaml の例
observations:
  joint_states: /left_arm/joint_states
  image: /cam_front/image_raw
actions:
  joint_commands: /left_arm/joint_commands
```

### 2. lerobot-ros - Lightweight Interface

**GitHub**: https://github.com/ycheng517/lerobot-ros

ros2_controlやMoveIt互換ロボットをLeRobotに接続する軽量ラッパー。

```bash
# テレオペレーション
lerobot-teleoperate --robot.type=so101_ros --teleop.type=keyboard_joint

# データ収集（標準LeRobotワークフロー）
lerobot-record --robot.type=so101_ros ...
```

### 3. dora-lerobot

**GitHub**: https://github.com/dora-rs/dora-lerobot

Doraフレームワークを使用したLeRobotパイプライン。ROS2ノードとの連携も可能。

### 推奨アプローチ

```
┌─────────────────────────────────────────────────────────────────┐
│                    データ収集フロー                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  選択肢A: rosettaを使用（推奨）                                   │
│  ────────────────────────────────                               │
│  VRテレオペ → ROS2トピック → rosetta EpisodeRecorder → rosbag2   │
│                                    → bag_to_lerobot.py          │
│                                    → LeRobot Dataset            │
│                                                                 │
│  選択肢B: 独自変換スクリプト                                      │
│  ────────────────────────────                                   │
│  VRテレオペ → ROS2トピック → ros2 bag record → rosbag2           │
│                           → rosbag_to_lerobot.py（本プロジェクト）│
│                           → LeRobot Dataset                     │
│                                                                 │
│  選択肢C: lerobot-rosを使用                                      │
│  ────────────────────────────                                   │
│  VRテレオペ → lerobot-ros → LeRobot標準ワークフロー               │
│              （要カスタマイズ）                                    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## LeRobotデータセット形式への変換

### 変換スクリプト

```python
# scripts/convert_rosbag_to_lerobot.py

def convert_rosbag_to_lerobot(bag_path: str, output_dir: str, task: str):
    """
    ROS2 BagファイルをLeRobot形式に変換

    LeRobot形式:
    - observations/
      - images/cam_front/  (PNG画像)
      - state/             (関節角度)
    - actions/             (目標関節角度)
    - episode_data.json    (エピソード情報)
    """
    pass
```

### LeRobotデータセット構造

```
dataset/
├── meta/
│   ├── info.json          # データセット情報
│   ├── stats.json         # 統計情報
│   └── episodes.json      # エピソード一覧
├── data/
│   ├── chunk-000/
│   │   ├── episode_000000.parquet
│   │   ├── episode_000001.parquet
│   │   └── ...
│   └── ...
└── videos/
    └── cam_front/
        ├── episode_000000.mp4
        └── ...
```

### データフィールド

```python
# 観測（入力）
observation.images.cam_front  # [H, W, 3] RGB画像
observation.state             # [6] 関節角度 (左アーム)
                              # または [12] (両アーム)

# アクション（出力）
action                        # [6] 目標関節角度
                              # または [12] (両アーム)
```

---

## 学習パイプライン

### SmolVLA学習設定

```yaml
# config/training/smolvla_so101.yaml
policy:
  name: smolvla
  pretrained_backbone: HuggingFaceM4/siglip-so400m-14-364-flash-attn2-navit

dataset:
  name: so101_vr_teleop
  image_keys: ["observation.images.cam_front"]
  state_keys: ["observation.state"]
  action_keys: ["action"]

training:
  batch_size: 32
  num_epochs: 100
  learning_rate: 1e-4
  action_chunk_size: 50  # 50ステップ先まで予測
```

### 学習コマンド

```bash
# LeRobot学習
python lerobot/scripts/train.py \
  --policy smolvla \
  --dataset so101_vr_teleop \
  --output_dir outputs/so101_smolvla
```

---

## Mujoco シミュレーション連携

### 目的

1. **事前検証**: 実機前にポリシーをテスト
2. **データ拡張**: シミュレーションでデータ量を増やす
3. **安全性**: 危険な動作を事前に検出

### アーキテクチャ

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Mujoco         │────>│  mujoco_ros2    │────>│  ROS2 Topics    │
│  Simulation     │<────│  bridge         │<────│                 │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                                                       │
                        ┌──────────────────────────────┤
                        │                              │
                        v                              v
               ┌─────────────────┐           ┌─────────────────┐
               │  VR Control     │           │  Policy Runner  │
               │  Node           │           │  Node           │
               └─────────────────┘           └─────────────────┘
```

### 必要なコンポーネント

1. **SO101 MJCFモデル**: Mujoco用ロボットモデル
2. **mujoco_ros2**: Mujoco ↔ ROS2 ブリッジ
3. **仮想カメラ**: シミュレーション内カメラ画像

### トピックマッピング

| シミュレーション | 実機 |
|-----------------|------|
| `/mujoco/left_arm/joint_states` | `/left_arm/joint_states` |
| `/mujoco/right_arm/joint_states` | `/right_arm/joint_states` |
| `/mujoco/cam_front/image_raw` | `/left_arm/cam_front/image_raw` |

---

## 実装ロードマップ

### Phase 1: データ収集基盤（現在）

- [x] VRテレオペノード (`vr_dual_arm_control_node`)
- [x] IKソルバー (`left_arm_ik_solver_node`, `right_arm_ik_solver_node`)
- [x] SO101ブリッジ (`leader_ros2_node`, `follower_ros2_node`)
- [ ] データ収集ノード (`sdr_node` 設定)
- [ ] エピソード管理（開始/終了トリガー）

### Phase 2: データ変換

- [ ] ROS2 Bag → LeRobot変換スクリプト
- [ ] 画像圧縮・リサイズ処理
- [ ] データ検証ツール

### Phase 3: 学習

- [ ] SmolVLA学習設定
- [ ] 学習パイプライン構築
- [ ] モデル評価

### Phase 4: 推論・実行

- [x] Policy Runner Node (`policy_runner_ros2_node`)
- [ ] 実機テスト
- [ ] 安全機構

### Phase 5: シミュレーション

- [ ] SO101 MJCFモデル作成
- [ ] Mujoco ROS2ブリッジ
- [ ] シミュレーション環境構築

---

## 必要なツール・依存関係

### Python

```
lerobot>=0.1.0
mujoco>=3.0.0
mujoco-py  # オプション
opencv-python
pillow
h5py
pyarrow
```

### ROS2

```
ros-humble-ros2bag
ros-humble-rosbag2-storage-mcap
ros-humble-cv-bridge
ros-humble-image-transport
```

### Unity

```
unity_ros_teleoperation (main-ros2 branch)
ROS-TCP-Connector
```

---

## タスク設計のガイドライン

### 適したタスク

- **ピック＆プレース**: 物体を掴んで移動
- **挿入タスク**: ペグ挿入など
- **組み立て**: 部品の組み合わせ

### データ収集のコツ

1. **一貫性**: 同じタスクを繰り返し実行
2. **多様性**: 物体位置・角度を変える
3. **成功のみ**: 失敗エピソードは除外
4. **ラベル付け**: タスク説明を明確に

### 推奨エピソード数

| モデル | 最小 | 推奨 |
|--------|------|------|
| SmolVLA | 50 | 200+ |
| ACT | 20 | 100+ |
| Diffusion Policy | 100 | 500+ |

---

## 次のアクション

1. **Unity側のトピック名を統一**
   - `/quest/left_hand/pose` → `/quest/left_hand/pose`
   - または vr_ros2側で対応

2. **データ収集テスト**
   - VRテレオペでros2 bag record
   - 収集データの確認

3. **変換スクリプト作成**
   - ROS2 Bag → LeRobot形式

4. **Mujoco環境構築**
   - SO101 MJCFモデル作成/入手
   - mujoco_ros2セットアップ
