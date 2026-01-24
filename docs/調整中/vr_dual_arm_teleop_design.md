# VR両手テレオペレーション設計書

## 概要

SO101デュアルアームロボットをVR（Quest 3）で両手テレオペレーションするシステムの設計書。

## 用語の定義

このシステムでは「Leader/Follower」という用語が**2つの異なる意味**で使われます：

### 1. LeRobot従来方式での意味
- **Leader**: 人間が手で動かす教示用アーム（入力デバイス）
- **Follower**: Leaderの動きを追従するアーム（出力デバイス）

### 2. VRテレオペでの意味
- **Leader/Follower**: 単なる**アームの識別子**（左アーム/右アーム）
- VRでは**両アームが独立して**VRコントローラーに追従
- Leader→Followerの追従関係は**存在しない**

**重要**: VRモードでは、VR（Quest 3）がリーダー（教示者）であり、両アームはVRの左右の手に直接追従します。

## システム構成

### アーキテクチャ

```
┌─────────────────────────────────────────────────────────────┐
│                     Unity (Quest 3)                         │
│                    [VRがリーダー役]                          │
└─────────────────────────────────────────────────────────────┘
                           ↓ TCP/IP
┌─────────────────────────────────────────────────────────────┐
│              ROS2 Unity TCP Endpoint                        │
└─────────────────────────────────────────────────────────────┘
                           ↓ Topics
           /quest/left_hand/pose    /quest/right_hand/pose
                    ↓                        ↓
┌─────────────────────────────────────────────────────────────┐
│              VR Dual Arm Control Node                       │
│         (VR入力を各アームにマッピング)                        │
└─────────────────────────────────────────────────────────────┘
          ↓                                    ↓
   /left_arm/target_pose              /right_arm/target_pose
          ↓                                    ↓
┌──────────────────────┐          ┌──────────────────────┐
│   Left Arm IK Solver │          │  Right Arm IK Solver │
└──────────────────────┘          └──────────────────────┘
          ↓                                    ↓
┌──────────────────────┐          ┌──────────────────────┐
│  SO101 ROS2 Bridge   │          │  SO101 ROS2 Bridge   │
│    (Left Arm)        │          │    (Right Arm)       │
└──────────────────────┘          └──────────────────────┘
          ↓                                    ↓
┌──────────────────────┐          ┌──────────────────────┐
│   ROS2 Control       │          │   ROS2 Control       │
│   (Left Arm)         │          │   (Right Arm)        │
└──────────────────────┘          └──────────────────────┘
          ↓                                    ↓
┌─────────────────────────────────────────────────────────────┐
│              SO101 Robot (Dual Arm)                         │
│         左アーム                    右アーム                 │
└─────────────────────────────────────────────────────────────┘
```

### 動作モード比較

| モード | 入力デバイス | 左アーム | 右アーム |
|--------|------------|---------|---------|
| VRテレオペ | Quest 3 | VR左手に追従 | VR右手に追従 |
| 従来テレオペ | 物理Leaderアーム | 教示用（手動） | 追従用 |
| AIポリシー | カメラ画像 | AI制御 | AI制御 |

## トピック構成

### Unity → ROS2

| トピック | 型 | 説明 |
|---------|-----|------|
| `/quest/left_hand/pose` | `geometry_msgs/PoseStamped` | VR左手のポーズ |
| `/quest/right_hand/pose` | `geometry_msgs/PoseStamped` | VR右手のポーズ |
| `/quest/left_hand/joints` | `sensor_msgs/PointCloud` | 左手21関節の3D座標 |
| `/quest/right_hand/joints` | `sensor_msgs/PointCloud` | 右手21関節の3D座標 |
| `/quest/hand_gesture` | `vr_haptic_msgs/HandGesture` | 手のジェスチャー（グリッパー制御） |

### ROS2内部

| トピック | 型 | 説明 |
|---------|-----|------|
| `/left_arm/target_pose` | `geometry_msgs/PoseStamped` | 左アーム目標ポーズ |
| `/right_arm/target_pose` | `geometry_msgs/PoseStamped` | 右アーム目標ポーズ |
| `/left_arm/ik/joint_angles` | `sensor_msgs/JointState` | 左アームIK計算結果 |
| `/right_arm/ik/joint_angles` | `sensor_msgs/JointState` | 右アームIK計算結果 |
| `/left_arm/joint_states` | `sensor_msgs/JointState` | 左アーム状態 |
| `/right_arm/joint_states` | `sensor_msgs/JointState` | 右アーム状態 |

**注意**: VRモード用のトピック名は `/left_arm/`, `/right_arm/` に統一されています。`so101_ros2_bridge` のノードはまだ `leader/follower` の名前を使用していますが、VRモードでは左右アームとして機能します。

## ノード構成

### 1. Unity TCP Endpoint Node
- **パッケージ**: `unity_robot_control`
- **ノード**: `unity_tcp_endpoint`
- **機能**: UnityとのTCP通信

### 2. VR Dual Arm Control Node
- **パッケージ**: `unity_robot_control`
- **ノード**: `vr_dual_arm_control_node`
- **機能**:
  - VR（Quest 3）からの左右手ポーズを受信
  - 左手→左アーム、右手→右アームにマッピング
  - 各アームのターゲットポーズをパブリッシュ
  - **VRがリーダー**として両アームを独立制御

### 3. IK Solver Nodes (2つ)
- **パッケージ**: `unity_robot_control`
- **ノード**:
  - `left_arm_ik_solver_node`
  - `right_arm_ik_solver_node`
- **機能**: ターゲットポーズからジョイント角度を計算（KDL使用）

### 4. SO101 ROS2 Bridge Nodes (2つ)
- **パッケージ**: `so101_ros2_bridge`
- **ノード**:
  - `left_arm_ros2_node` (現在は `leader_ros2_node`)
  - `right_arm_ros2_node` (現在は `follower_ros2_node`)
- **機能**: LeRobot API ↔ ROS2 Control ブリッジ

### 5. ROS2 Control
- **パッケージ**: `so101_controller`, `so101_hardware_interface`
- **機能**: ジョイント制御

## 設定ファイル

### `config/vr_dual_arm_config.yaml`

```yaml
vr_dual_arm_control:
  ros__parameters:
    # VR左手→左アーム、VR右手→右アームにマッピング
    left_hand_topic: "/quest/left_hand/pose"
    right_hand_topic: "/quest/right_hand/pose"
    left_arm_target_topic: "/left_arm/target_pose"
    right_arm_target_topic: "/right_arm/target_pose"
    enable_gripper_control: true

left_arm_ik_solver_node:
  ros__parameters:
    base_link: "base_link"
    end_effector_link: "gripper_link"
    target_pose_topic: "/left_arm/target_pose"
    joint_angles_topic: "/left_arm/ik/joint_angles"

right_arm_ik_solver_node:
  ros__parameters:
    base_link: "base_link"
    end_effector_link: "gripper_link"
    target_pose_topic: "/right_arm/target_pose"
    joint_angles_topic: "/right_arm/ik/joint_angles"
```

## デバイスID（USBポート）設定

左右アームのUSBポートとロボットIDは以下のYAMLファイルで設定します：

### 左アーム設定
**ファイル**: `so101_ros2_bridge/config/so101_leader_params.yaml`

```yaml
# 注意: ファイル名は "leader" だが、VRモードでは「左アーム」として機能
so101_leader_ros2_bridge:
  ros__parameters:
    port: "/dev/ttyACM0"  # 左アームのUSBポート
    id: "Gili"            # 左アームのロボットID
    use_degrees: true
    disable_torque_on_disconnect: true
    publish_rate: 50.0
```

### 右アーム設定
**ファイル**: `so101_ros2_bridge/config/so101_follower_params.yaml`

```yaml
# 注意: ファイル名は "follower" だが、VRモードでは「右アーム」として機能
so101_follower_ros2_bridge:
  ros__parameters:
    port: "/dev/ttyACM1"  # 右アームのUSBポート
    id: "Tzili"           # 右アームのロボットID
    use_degrees: true
    max_relative_target: 10
    disable_torque_on_disconnect: true
    publish_rate: 50.0
```

### USBポートの確認方法

```bash
lerobot-find-port
# または
ls -l /dev/ttyACM*
```

## Lerobotとの接続方法

### 接続アーキテクチャ

```
ROS2ノード (so101_ros2_bridge)
    ↓ Python import
Lerobotライブラリ
    ↓ USBシリアル通信
SO101ロボット (左アーム/右アーム)
```

### 必要な環境設定

```bash
# Conda環境のsite-packagesパスを環境変数に設定
export LECONDA_SITE_PACKAGES=/path/to/conda/env/lib/python3.x/site-packages
```

## 起動方法

```bash
# VR両手テレオペレーション
ros2 launch unity_robot_control vr_dual_arm_teleop.launch.py

# 個別起動
ros2 run unity_robot_control unity_tcp_endpoint
ros2 run unity_robot_control vr_dual_arm_control_node
ros2 run so101_ros2_bridge leader_ros2_node   # 左アーム
ros2 run so101_ros2_bridge follower_ros2_node # 右アーム
```

## 問題点と対策

### 1. 用語の混乱
- **問題**: "Leader/Follower" という名前がVRモードでは誤解を招く
- **対策**: ドキュメントで明確に説明。将来的に "left_arm/right_arm" にリネーム検討

### 2. Unity側のトピック名
- **問題**: Unity側から適切なトピック名で送信されているか確認が必要
- **対策**: Unity側の設定確認・変更

### 3. 座標系の変換
- **問題**: Unity座標系とROS2座標系の違い
- **対策**: TF変換または座標変換ノードを追加

### 4. 安全制御
- **問題**: 衝突回避や可動範囲チェック
- **対策**: ジョイント制限チェック、可動範囲検証

## 今後の拡張

- [x] VRモード用トピック名を `/left_arm/`, `/right_arm/` に統一
- [ ] so101_ros2_bridgeのトピック名も統一（remapで対応中）
- [ ] MoveIt2統合
- [ ] 力覚フィードバック（ハプティック）
- [ ] 衝突回避
- [ ] 自動キャリブレーション
