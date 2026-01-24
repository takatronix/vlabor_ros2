# SO101 vs Piper ROS2 I/F 比較

このドキュメントは **SO101** と **AgileX Piper** のROS2 I/Fを比較し、運用視点での違いを整理したものです。

---

## 1. 目的と前提
- 上位制御は **Piper互換I/F** に寄せる。
- SO101は `so101_control_node` によって Piper互換I/Fを提供する。

---

## 2. 主要I/F比較（概要）

| 項目 | Piper | SO101 |
|---|---|---|
| 送信 | `joint_ctrl_single` / `pos_cmd` | `joint_ctrl_single` |
| 受信 | `joint_states_single` / `joint_states_feedback` | `joint_states_single` / `joint_states_feedback` |
| 有効化 | `enable_srv` / `enable_flag` | `enable_flag`（`enable_srv`は環境依存） |
| 物理制御 | CAN | USB Serial (Feetech) |
| キャリブ | 内部管理 | lerobot JSONで外部管理 |

---

## 3. トピック比較（詳細）

### 3.1 Publish
| Topic | Piper | SO101 |
|---|---|---|
| `joint_states_single` | ○ | ○ |
| `joint_states_feedback` | ○ | ○ |
| `joint_ctrl` | ○ | ○ |
| `arm_status` | ○ | × |
| `end_pose` | ○ | × |
| `end_pose_stamped` | ○ | × |

### 3.2 Subscribe
| Topic | Piper | SO101 |
|---|---|---|
| `joint_ctrl_single` | ○ | ○ |
| `pos_cmd` | ○ | × |
| `enable_flag` | ○ | ○ |

### 3.3 Services
| Service | Piper | SO101 |
|---|---|---|
| `enable_srv` | ○ | △（typesupport依存） |

---

## 4. Joint Name / Joint数

| 項目 | Piper | SO101 |
|---|---|---|
| 関節数 | 7（gripper含む） | 6（gripper含む） |
| ジョイント名 | `joint1..joint6, gripper` | `shoulder_pan..gripper` |

---

## 5. キャリブレーション

### Piper
- 基本はノード内部で完結

### SO101
- lerobot形式JSONを外部ファイルで管理  
  例: `~/.cache/huggingface/lerobot/calibration/robots/so101_follower/`
- `homing_offset`, `range_min`, `range_max` を利用  
- ticks↔rad変換に反映

---

## 6. 初心者向けまとめ

- **Piperは標準I/Fが豊富**（Pose制御やStatusあり）
- **SO101は最小I/F**（JointState中心）  
  → その代わり **Piper互換化**で上位が統一できる

---

## 7. 次の拡張案

- SO101に `arm_status` 相当の簡易ステータスを追加
- `pos_cmd` 互換（エンドエフェクタ指令）を追加
- 右腕・左腕でjoint名を統一し、上位の変換を削減
