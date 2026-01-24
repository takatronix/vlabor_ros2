# VRテレオペ テスト項目（順番）

## 前提
- ROS_DOMAIN_ID=1
- `~/ros2_ws` を source 済み（`source /opt/ros/humble/setup.bash` と `source install/setup.bash`）
- Unity側の送信先 IP/port が ROS TCP Endpoint と一致

## 0. 事前確認
- [ ] `ros2 topic list` で `/quest/*` が見える（Unity起動後）
- [ ] `/left_arm/joint_states_single` `/right_arm/joint_states_single` が出ている（mock/feetech いずれでも）

## 1. Unity → ROS TCP 受信
- [ ] `/quest/left_controller/pose` が流れる
- [ ] `/quest/right_controller/pose` が流れる
- [ ] 周波数が安定（例: 30Hz 前後）

## 2. VR制御ノード（vr_dual_arm_control）
- [ ] `/left_arm/target_pose` が流れる
- [ ] `/right_arm/target_pose` が流れる
- [ ] 位置オフセットが左右で反映される（30cm間隔 → y が ±0.15 で分離）

## 3. IKノード（left/right_arm_ik_solver）
- [ ] `/left_arm/ik/joint_angles` が流れる
- [ ] `/right_arm/ik/joint_angles` が流れる
- [ ] 初期姿勢が実機の関節角に追従している（`joint_states_single` が効いている）
- [ ] IK初期化ログで `end_effector_link=gripper_frame_link` が確認できる
- [ ] 姿勢追従が有効な場合、コントローラー回転で wrist_roll が変化する

## 4. SO101制御ノード（so101_control_node）
- [ ] `left_arm` 名前空間の `joint_ctrl_single` に入力が入る
- [ ] `right_arm` 名前空間の `joint_ctrl_single` に入力が入る
- [ ] `joint_states_single` / `joint_states_feedback` が更新され続ける

## 5. 実機確認（feetech）
- [ ] シリアルポートが認識されている（`/dev/ttyACM*`）
- [ ] `driver_backend:=feetech` で起動できる
- [ ] 初期姿勢から大きく飛ばずに追従する

## 6. 安全確認
- [ ] 位置制限（clamp）を有効化してもIKが破綻しない
- [ ] 異常時に `enable_flag` で停止できる

## 実行メモ
- 起動: `scripts/start.sh`
- 監視: `scripts/status.sh`
