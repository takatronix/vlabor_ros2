# vlabor_ros2 ドキュメント一覧

`docs/` 配下のドキュメントを用途別に整理した目次です。

## 設計・仕様（現行）

| ファイル | 内容 |
|----------|------|
| [vr_teleop_design.md](vr_teleop_design.md) | VRテレオペ設計（座標系・トピック・変換方針） |
| [profiles.md](profiles.md) | 起動プロファイルと Recorder/Role の概要 |
| [roles.md](roles.md) | 物理アーム名と役割(leader/follower)のルール |
| [so101_interface_spec.md](so101_interface_spec.md) | SO101 ROS2インタフェース仕様 |
| [AgileX_Piper_ROS2インタフェース仕様.md](AgileX_Piper_ROS2インタフェース仕様.md) | Piper ROS2インタフェース仕様 |
| [robot_interface_comparison.md](robot_interface_comparison.md) | SO101 vs Piper 比較 |

## 運用・テスト

| ファイル | 内容 |
|----------|------|
| [vr_teleop_todo.md](vr_teleop_todo.md) | VRテレオペ TODO |
| [vr_teleop_test_plan.md](vr_teleop_test_plan.md) | VRテレオペテスト計画 |
| [vr_teleop_ui_design.md](vr_teleop_ui_design.md) | VRテレオペ UI 設計 |

## 調査・外部仕様

| ファイル | 内容 |
|----------|------|
| [so101_driver_survey.md](so101_driver_survey.md) | SO101ドライバ調査 |
| [EtherCAT ROS2 対応状況調査レポート.md](EtherCAT%20ROS2%20対応状況調査レポート.md) | EtherCAT ROS2 対応調査 |

## 環境構築

| ファイル | 内容 |
|----------|------|
| [VLAbor Mac環境(UTM)セットアップ手順書](utm_setup/VLAbor%20Mac環境(UTM)セットアップ手順書.md) | Mac(UTM)でのVLAbor環境構築 |

## 調整中（WIP）

`docs/調整中/` 以下は設計・仕様の草案や将来構想であり、現行の起動構成・コードと必ずしも一致していません。

- [system_overview.md](調整中/system_overview.md)
- [system_architecture.md](調整中/system_architecture.md)
- [technical_details.md](調整中/technical_details.md)
- [vr_dual_arm_teleop_design.md](調整中/vr_dual_arm_teleop_design.md)
- [lerobot_configuration_guide.md](調整中/lerobot_configuration_guide.md)
- [lerobot_integration_design.md](調整中/lerobot_integration_design.md)
- [policy_runner_node.md](調整中/policy_runner_node.md)
- [mujoco_integration_design.md](調整中/mujoco_integration_design.md)
- [multi_robot_architecture.md](調整中/multi_robot_architecture.md)

---

起動方法・プロファイル一覧はリポジトリルートの [README.md](../README.md) および [profiles.md](profiles.md) を参照してください。
