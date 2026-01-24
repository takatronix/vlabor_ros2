# VRテレオペ TODO

## 目的
VRテレオペ（Quest 3 → SO101左右アーム）の安定動作を最優先で整える。

## TODO一覧
- [x] Unity入力トピックの**正式名称を確定**し、全ノード/設定/READMEを統一する。
  - [x] テレオペ入力は `/quest/left_controller/pose`, `/quest/right_controller/pose` を使用。
  - [ ] 監視トピック（`unity_tcp_endpoint`）を hand/controller どちらに合わせるか整理。
- [x] `vr_dual_arm_control_node` に**座標変換・安全制限**を実装する。
  - [x] スケールと左右オフセットは実装済み。
- [x] IK出力 → ロボット制御入力の**ブリッジ経路を確定**する。
- [x] `/leader|/follower` と `/left_arm|/right_arm` の**マッピング方針を整理**する。
- [x] SO101制御ノードを**Piper互換I/F**で実装する（中間ノード）。
- [x] SO101の**Feetech実機ドライバ**を実装（scservo_sdk, serial通信）。
- [x] SO101 ROS2ノードの**既存実装を調査比較**する。
- [x] lerobotキャリブJSONの**読み込み対応**を追加する。
- [ ] Feetech実機での**通信・可動確認**。
- [ ] 実機VRテスト（Quest 3）で**起動手順と動作確認**を行う。
- [ ] IK末端リンクを `gripper_frame_link` に変更し、**グリッパ先端合わせ**を行う。
- [ ] `use_orientation` を有効化する場合の**姿勢マッピング/制約**を設計する。
- [ ] **ギクシャク対策**（IK不収束・clamp・送信周期）をログで切り分ける。

## テスト項目
- `docs/vr_teleop_test_plan.md` を順番にチェックする

## 担当案（AI/人）
- 設計整理: GPT5-codex
- 実装/微修正: Cursor（Auto）
- 翻訳・スクリプト補助: Claude(Sonnet)
