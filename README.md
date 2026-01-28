# VR ROS2

VRテレオペレーションとロボット制御用のROS2ノード群


## 目的
VR(Quest 3)でSO101やAgileX Piper等のロボットアームをテレオペレーションし、将来的にLeRobotで学習データ作成・推論に接続する。

## 現在のスコープ
- まずは **VRテレオペレーションの安定動作** に注力。
- 学習データ作成・LeRobot連携は次のフェーズで設計・実装する。


## 概要

SO101デュアルアームロボットをVR（Quest 3）で両手テレオペレーションするシステム。

**重要**: VRモードでは、**VR（Quest 3）がリーダー（教示者）** として機能し、両アームは独立してVRの左右の手に追従します。

リーダーアーム: Quest3のコントローラーまたはQuest3が認識している指

## 用語と命名
- **公式インタフェースは `/left_arm/*` と `/right_arm/*`** を使用する。
- `leader/follower` は **物理マスターアーム** を使う場合の互換名として残す想定。
  - VRモードでは「Leader/Follower」は単なる識別子であり、追従関係は存在しない。

## Unity側のソース
https://github.com/takatronix/unity_ros_teleoperation


## 対応ロボットアーム
- SO101
- AgileX Piper（予定）


## システム構成

```
VR (Quest 3) [リーダー役]   unity_ros_teleoperation
    ↓ TCP/IP（通信)
ros_tcp_endpoint / unity_tcp_endpoint (TCP受信サーバー)
    ↓

vr_dual_arm_control_node
  VR座標 -> ロボット座標変換
  アーム配置(左右30cm間隔など、設計は `docs/vr_teleop_design.md` 参照)に合わせたオフセット・スケール補正

    ↓
ik_solver_node (左右)
  対応ロボットアームのURDFを参照
    ├─ 左アーム (left_arm_ik_solver_node)
    └─ 右アーム (right_arm_ik_solver_node)
    ↓
so101_control_node (left/right, Piper互換I/F)
    ↓
ロボットアームコントロール
piper_ctrl_single_node (left/right, 参考)

    (LeRobot連携・学習データ作成は次フェーズ)

## 主要ドキュメント
- **設計・運用**: [docs/README.md](docs/README.md) に目次あり。
- VRテレオペ設計: `docs/vr_teleop_design.md`
- VRテレオペ TODO: `docs/vr_teleop_todo.md`
- SO101ドライバ調査: `docs/so101_driver_survey.md`
- Piper I/F仕様: `docs/AgileX_Piper_ROS2インタフェース仕様.md`
- SO101 I/F仕様: `docs/so101_interface_spec.md`
- SO101 vs Piper 比較: `docs/robot_interface_comparison.md`
- 役割・命名ルール: `docs/roles.md`
- Profiles/Roleの概要: `docs/profiles.md`

## 起動（VRテレオペ直結）
短い起動コマンド:
```bash
./scripts/run so101_vr_dual_teleop
```
VR/単腕/双腕テレオペのプロファイルはRViz（`so101_description/rviz/vr_teleop.rviz`）が自動起動します。

詳細指定する場合:
```bash
ros2 launch vlabor_launch vlabor.launch.py \
  profile:=so101_vr_dual_teleop \
  left_serial_port:=/dev/ttyACM0 right_serial_port:=/dev/ttyACM1 \
  driver_backend:=mock
```

実機の場合:
```bash
ros2 launch vlabor_launch vlabor.launch.py \
  profile:=so101_vr_dual_teleop \
  left_serial_port:=/dev/ttyACM0 right_serial_port:=/dev/ttyACM1 \
  driver_backend:=feetech
```

キャリブJSONを指定する場合:
```bash
ros2 launch vlabor_launch vlabor.launch.py \
  profile:=so101_vr_dual_teleop \
  left_calibration_path:=/home/takatronix/.cache/huggingface/lerobot/calibration/robots/so101_follower/5AB9069153.json \
  right_calibration_path:=/home/takatronix/.cache/huggingface/lerobot/calibration/robots/so101_follower/5AB9069153.json \
  driver_backend:=feetech
```
