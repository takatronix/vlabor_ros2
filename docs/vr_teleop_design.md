# VRテレオペ設計（v1）

## 目的とスコープ
- VR(Quest 3)でSO101の左右アームを**安定してテレオペ**する。
- 学習データ作成・LeRobot連携は**次フェーズ**として扱う（本設計では最小限の想定のみ）。

## 用語
- **left/right**: 公式な制御対象（/left_arm/*, /right_arm/*）。
- **leader/follower**: 物理マスターアームを使う場合の互換命名（VRでは追従関係は存在しない）。

## 前提
- SO101を左右に**30cm間隔**で配置する（現場の実機間隔）。
- VRはQuest 3。Unity側で `/quest/left_controller/pose`, `/quest/right_controller/pose` をPublish。

## システム構成（VRテレオペ）

```
Quest 3 (Unity)
  └─ /quest/left_controller/pose, /quest/right_controller/pose
        ↓
unity_tcp_endpoint / ros_tcp_endpoint
        ↓
vr_dual_arm_control_node
  - VR座標 → ロボット座標変換
  - 左右アームのオフセット適用
        ↓
left_arm_ik_solver_node / right_arm_ik_solver_node
        ↓
so101_control_node (left/right, Piper互換I/F)
        ↓
SO101 左右アーム
```

## 公式インタフェース（v1）

### VR入力
- `/quest/left_controller/pose` (geometry_msgs/PoseStamped)
- `/quest/right_controller/pose` (geometry_msgs/PoseStamped)
- `/quest/hand_gesture` (vr_haptic_msgs/HandGesture, optional)

### VR→ロボット
- `/left_arm/target_pose` (PoseStamped)
- `/right_arm/target_pose` (PoseStamped)

### IK
- `/left_arm/ik/joint_angles` (sensor_msgs/JointState)
- `/right_arm/ik/joint_angles` (sensor_msgs/JointState)

補足:
- IK入力は `tf_base_link` 座標系に変換して解く（TFがある場合）

### ロボット制御
- `/left_arm/joint_ctrl_single` (sensor_msgs/JointState)
- `/right_arm/joint_ctrl_single` (sensor_msgs/JointState)
- `/left_arm/joint_states_single` (sensor_msgs/JointState)
- `/right_arm/joint_states_single` (sensor_msgs/JointState)
- `/left_arm/joint_states_feedback` (sensor_msgs/JointState)
- `/right_arm/joint_states_feedback` (sensor_msgs/JointState)

### 互換I/F（Piper）
- `enable_srv` (piper_msgs/srv/Enable)
- `enable_flag` (std_msgs/Bool)

## 座標系・配置
- ROS標準 FLU（x:前, y:左, z:上）。
- `world` を基準とし、左右のアーム基準は以下を推奨。
  - `left_base` : (0.0, +0.15, 0.0)
  - `right_base`: (0.0, -0.15, 0.0)
- 30cm間隔は**パラメータ化**し、調整可能にする。

## VR→ロボット変換方針
- `vr_dual_arm_control_node` が変換責任を持つ。
  - 軸変換（Unity → ROS）
  - スケーリング
  - オフセット（左右アーム間隔）
  - 安全制限（姿勢・位置のクリップ）
- 変換の詳細は `unity_robot_control/config/vr_dual_arm_config.yaml` に集約。
  - `position_scale`, `left_arm_offset_xyz`, `right_arm_offset_xyz`, `enable_pose_offset`
  - `enable_axis_remap`, `axis_map`, `enable_position_clamp`, `position_min_xyz`, `position_max_xyz`
- Unity側は `To<FLU>()` でROS座標系に変換済みのため、`enable_axis_remap` は原則オフで運用する。

## 先端合わせ・姿勢合わせ（v1.1）
- IKの末端は `gripper_frame_link` を使用し、グリッパー先端の位置/姿勢に合わせる。
- 姿勢追従は `use_orientation: true` を基本とするが、SO101は5-DOFのため全姿勢の完全一致は不可。
  - 必要に応じて姿勢の優先軸（yaw優先など）や固定オフセットを導入する。

## 現状実装との差分（要整理）
- `vr_dual_arm_control_node` に**位置スケール・左右オフセット・軸変換・位置制限**のパラメータを実装済み。
- Unity入力トピック名は `/quest/left_controller/pose`, `/quest/right_controller/pose` が正。
  - 接続監視は `unity_tcp_endpoint` 側で手ポーズを見ているため、必要に応じて監視トピックを合わせる。
- IK末端リンクが `gripper_link` のため先端が一致しない（`gripper_frame_link` に変更が必要）。
- `use_orientation` が false のため姿勢が反映されない（姿勢追従を行う場合は true に変更）。
- `so101_control_node` をPiper互換I/Fで実装し、**直結構成**では `so101_ros2_bridge` を使用しない。
  - ros2_control + bridge 構成も別途存在（`vr_dual_arm_teleop.launch.py`）。
  - `joint_ctrl_single` を購読し、`joint_states_*` をPublish。
  - `enable_srv` に対応。
  - `driver_backend=mock` で開発確認可能。
  - `driver_backend=feetech` で scservo_sdk を使った実機制御に対応。

## デバッグ/ログ方針（追加予定）
- **vr_dual_arm_control_node**: 生Pose/変換後Poseの差分、target_poseの送信レート、グリップ状態。
- **ik_solver_node**: IK成功/失敗、resultコード、解法時間（ms）、IK初期値。

## 起動・主要ファイル
- VRテレオペ起動（直結）: `unity_robot_control/launch/vr_dual_arm_teleop_direct.launch.py`
- VR制御ノード: `unity_robot_control/unity_robot_control/vr_dual_arm_control_node.py`
- IKノード: `unity_robot_control/unity_robot_control/ik_solver_node.py`（左右で同一実装）
- IK→コントローラ: `unity_robot_control/unity_robot_control/ik_to_joint_trajectory_node.py`（ros2_control運用時のみ）
- Unity TCP: `unity_robot_control/unity_robot_control/unity_tcp_endpoint.py`
- SO101制御: `unity_robot_control/unity_robot_control/so101_control_node.py`

## ノード接続とトピック対応（実装準拠）

### ノードI/O一覧
- **unity_tcp_endpoint**
  - pub: `/quest/left_controller/pose`, `/quest/right_controller/pose`, `/quest/pose/headset`
- **vr_dual_arm_control_node**
  - sub: `/quest/left_controller/pose`, `/quest/right_controller/pose`
  - pub: `/left_arm/target_pose`, `/right_arm/target_pose`
- **left_arm_ik_solver_node**
  - sub: `/left_arm/target_pose`
  - sub: `/left_arm/joint_states_single`（IK初期値）
  - pub: `/left_arm/ik/joint_angles`
- **right_arm_ik_solver_node**
  - sub: `/right_arm/target_pose`
  - sub: `/right_arm/joint_states_single`（IK初期値）
  - pub: `/right_arm/ik/joint_angles`
- **so101_control_node**（namespace: `left_arm`/`right_arm`）
  - sub: `joint_ctrl_single`（※起動時に `ik/joint_angles` へ remap）
  - pub: `joint_states_single`, `joint_states_feedback`, `joint_ctrl`

### トピックの流れ（直結構成）
```
/quest/left_controller/pose  → /left_arm/target_pose  → /left_arm/ik/joint_angles  → /left_arm/joint_ctrl_single
/quest/right_controller/pose → /right_arm/target_pose → /right_arm/ik/joint_angles → /right_arm/joint_ctrl_single
```

補足:
- `joint_ctrl_single` は `so101_control_node` 内の購読名。launch で `ik/joint_angles` を remap して接続する。

## 関連資料
- SO101ドライバ調査: `docs/so101_driver_survey.md`

## 次フェーズ（除外）
- LeRobotデータ作成・MP4/画像保存方針の確定
- late-fusion（RGB+Depth）用ノード追加

## 物理マスターアーム（将来想定）
- VR以外に物理リーダーアームを使う場合、`leader/follower` の互換命名を使用。
- 将来的に**4アーム構成**（リーダー2 + フォロワー2）も考慮し、入力ソースを切り替える設計が望ましい。
  - 例: `teleop_source` パラメータで `vr` / `leader` を切替。

## 非スコープ（次フェーズ）
- LeRobotのデータセット作成（MP4/画像保存方針の確定）
- 物理リーダーアームの詳細運用
- 学習/推論ノードの運用・最適化
