# 2026-01-17 vr-teleop-tip-alignment
## ゴール
- VRテレオペでグリッパ先端の位置/姿勢を合わせ、ギクシャクの原因を切り分ける

## TODO / 依存関係
- [x] ドキュメント更新（設計/テスト/ TODO）
- [x] end_effector_link を gripper_frame_link へ変更
- [x] use_orientation 対応（設定側を有効化）
- [x] ログ追加（vr_dual_arm_control_node / ik_solver_node）
- [x] 30cm間隔の左右オフセット調整

## 実施内容
- VRテレオペ設計・テスト・TODOのドキュメントを更新
- `vr_dual_arm_config.yaml` を30cm間隔/先端合わせ/姿勢追従/周期調整に更新
- VR制御ノードとIKノードにデバッグログ（姿勢/レート/IK統計）を追加

## AI活用ログ
- モデル: GPT5-codex
- 指示 / 返答概要: VRテレオペの問題点整理と設計更新、ログ追加方針の明文化

## 次のアクション
- テスト計画に沿ってIK成功率/遅延/ギクシャクの原因を計測

## メトリクス / 所要時間
- 計画: 01:00 / 実績: 00:20
