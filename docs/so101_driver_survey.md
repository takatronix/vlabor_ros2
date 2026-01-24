# SO101 ROS2ノード調査メモ

SO101のROS2ノードサンプルを比較し、取り込みたい設計要素を整理する。

## 参照リポジトリ
- msf4-0/so101_ros2: https://github.com/msf4-0/so101_ros2
- SeeedJP/lerobot_ros2_so101: https://github.com/SeeedJP/lerobot_ros2_so101
- nimiCurtis/so101_ros2: https://github.com/nimiCurtis/so101_ros2 （空に近い）
- ganikv/SO101_ROS2: https://github.com/ganikv/SO101_ROS2 （空）

## 比較表（要点）
| 項目 | msf4-0 | SeeedJP |
|---|---|---|
| 主要I/F | `/joint_states` publish/subscribe | `/joint_states` publish、`joint_commands` subscribe |
| 依存 | `feetech-servo-sdk` | `lerobot[feetech]`（内部で scservo_sdk） |
| 角度変換 | deg ↔ rad | ticks(4096) ↔ rad |
| キャリブ | `so101_control.py` で実行 | lerobot calibration JSON を利用 |
| 付加機能 | record/replay（JSON） | calibrationファイルの具体例 |

## 取り込み候補（本PJ向け）
- **SeeedJPのキャリブレーション運用**  
  lerobotのJSONキャリブを使い、motor_id/offset/rangeを定義して変換に使う。
- **msf4-0のrecord/replay**  
  episode recorderを後で追加する際の参考。
- **I/FはPiper互換に統一**  
  上位制御は `JointState`、`enable_srv` を使いロボット差を吸収。

## scservo_sdk の注意
- PyPIに `scservo_sdk` は無い場合がある。
- 代替インストール:
  - `feetech-servo-sdk`（pip）
  - `FTServo_Python` を git から導入

## 設計メモ（SO101制御）
- **通信**: USB-Serial（Feetech, protocol 0想定）
- **変換**: ticks(0-4095) ↔ rad  
  - 例: `rad = (ticks - 2048) * (2π / 4096)`
- **校正**: lerobotキャリブJSONを読み込み  
  - `homing_offset`, `range_min/max`, `drive_mode` を使う
  - `ticks_offset = 2048 + homing_offset` を基本にする

## 次アクション
- SO101の実機キャリブJSONを揃えて `motor_ids`, `ticks_offset`, `range` を確定。
- `driver_backend=feetech` のパラメータ設定を左/右で固定化。
