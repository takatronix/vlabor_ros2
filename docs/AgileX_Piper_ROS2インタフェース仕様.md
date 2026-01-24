# AgileX Piper ROS2インタフェース仕様

このドキュメントは **AgileX Piper** のROS2ノードI/F仕様を詳しくまとめたものです。  
「どのトピックに何を投げれば動くか」「どの状態を見ればいいか」を最短で理解できる構成にしています。  
公式の詳細は以下を参照（AgileX Robotics 公式リポジトリ）:

- https://github.com/agilexrobotics/piper_ros

---

## 0. まず覚える最小I/F

最短で動かすにはこの3つだけ覚えればOKです。

- **送る**: `joint_ctrl_single`  
  → `sensor_msgs/JointState` で関節角度を送る
- **見る**: `joint_states_single`  
  → 実機の現在角度が返ってくる
- **有効化**: `enable_srv` または `enable_flag`  
  → モータON/OFF

---

## 1. トピック（送信 / Publish）

### `joint_states_single` (sensor_msgs/JointState)
実機の**現在角度**。  
この値を読むと「今の姿勢」を把握できます。

### `joint_states_feedback` (sensor_msgs/JointState)
フィードバック用の状態。  
基本的には `joint_states_single` と同じ用途でOK。

### `joint_ctrl` (sensor_msgs/JointState)
**現在の制御目標**（コマンドのエコー）。  
何を送っているか確認したいときに見る。

### `arm_status` (piper_msgs/PiperStatusMsg)
エラーや状態のビット情報。  
動かないときはまずここを確認。

### `end_pose` / `end_pose_stamped` (geometry_msgs/Pose / PoseStamped)
エンドエフェクタの現在姿勢（位置と向き）。

---

## 2. トピック（受信 / Subscribe）

### `joint_ctrl_single` (sensor_msgs/JointState)
**関節角度コマンド**。  
ここに `position` を送ると動く。

### `pos_cmd` (piper_msgs/PosCmd)
**エンドエフェクタ目標**で動かす場合に使う。  
XYZ+RPY とグリッパ、モード指定が可能。

### `enable_flag` (std_msgs/Bool)
`True` で有効化、`False` で無効化。

---

## 3. サービス

### `enable_srv` (piper_msgs/srv/Enable)
`enable_request: bool` → `enable_response: bool`  
モータON/OFFのサービス。

---

## 4. 状態の意味（arm_statusの読み方）

`PiperStatusMsg` は「動作モード・エラー・制限・通信状態」をまとめて出します。  
公式READMEの説明を日本語で整理しています。

### ctrl_mode（制御モード）
```
0x00 Standby mode（待機）
0x01 CAN command control mode（CAN指令制御）
0x02 Teaching mode（教示モード）
0x03 Ethernet control mode（Ethernet制御）
0x04 WiFi control mode（WiFi制御）
0x05 Remote controller control mode（リモコン制御）
0x06 Linked teaching mode（連携教示）
0x07 Offline track mode（オフライン軌道）
```

### arm_status（アーム状態）
```
0x00 Normal（正常）
0x01 Emergency stop（非常停止）
0x02 No solution（IK解なし）
0x03 Singularity（特異点）
0x04 Target angle exceeds limit（角度上限超過）
0x05 Joint communication error（関節通信異常）
0x06 Joint brake not released（ブレーキ未解除）
0x07 Robotic arm collision detected（衝突検知）
0x08 Overspeed during drag teaching（ドラッグ教示中の過速度）
0x09 Joint status abnormal（関節状態異常）
0x0A Other abnormality（その他異常）
0x0B Teaching record（教示記録中）
0x0C Teaching execution（教示再生中）
0x0D Teaching paused（教示一時停止）
0x0E Main control NTC overheating（制御部温度異常）
0x0F Discharge resistor NTC overheating（放電抵抗温度異常）
```

### mode_feedback（動作モード）
```
0x00 MOVE P（点移動）
0x01 MOVE J（関節移動）
0x02 MOVE L（直線移動）
0x03 MOVE C（円弧移動）
```

### teach_status（教示ステータス）
```
0x00 Close（未実行）
0x01 Start teaching recording（教示記録開始）
0x02 End teaching recording（教示記録終了）
0x03 Execute teaching trajectory（教示再生）
0x04 Pause（一時停止）
0x05 Resume（再開）
0x06 Terminate execution（終了）
0x07 Move to trajectory starting point（開始点へ移動）
```

### motion_status（到達状態）
```
0x00 Reached the specified point（目標到達）
0x01 Not reached the specified point（未到達）
```

### trajectory_num
```
0~255（オフライン軌道モードの識別子）
```

### err_code
```
エラーコード（詳細なコード表は公式SDK/マニュアル参照）
```

### joint_1_angle_limit ~ joint_6_angle_limit / communication_status_joint_1 ~ joint_6
公式READMEのコメントに**入れ替わりがある可能性**があります。  
実機での挙動を確認した上で、どちらが「角度超過」「通信異常」かを確定してください。

- `joint_X_angle_limit`  
  - README(EN)では「通信異常」と記載
- `communication_status_joint_X`  
  - README(EN)では「角度超過」と記載

**動かないときのチェック順**
1) `communication_status_joint_*` がTrueか  
2) `joint_*_angle_limit` がTrueになっていないか  
3) `err_code` が0か  
4) `enable_flag`/`enable_srv` で有効化できているか

---

## 5. アクセス方法（CLIコマンド例）

### 状態を見る
```bash
ros2 topic echo --once /joint_states_single
ros2 topic echo --once /arm_status
```

### 有効化
```bash
ros2 service call /enable_srv piper_msgs/srv/Enable "enable_request: true"
```
または
```bash
ros2 topic pub --once /enable_flag std_msgs/msg/Bool "{data: true}"
```

### 関節角度で動かす
```bash
ros2 topic pub --once /joint_ctrl_single sensor_msgs/msg/JointState "{
  name: ['joint1','joint2','joint3','joint4','joint5','joint6','gripper'],
  position: [0.2, 0.2, -0.2, 0.3, -0.2, 0.5, 0.01]
}"
```

---

## 6. 簡単なPythonサンプルコード

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class PiperDemo(Node):
    def __init__(self):
        super().__init__('piper_demo')
        self.pub = self.create_publisher(JointState, 'joint_ctrl_single', 10)
        self.enable_pub = self.create_publisher(Bool, 'enable_flag', 10)

    def enable(self):
        msg = Bool()
        msg.data = True
        self.enable_pub.publish(msg)

    def move(self):
        msg = JointState()
        msg.name = ['joint1','joint2','joint3','joint4','joint5','joint6','gripper']
        msg.position = [0.2, 0.2, -0.2, 0.3, -0.2, 0.5, 0.01]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PiperDemo()
    node.enable()
    node.move()
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 7. 注意点

- **CAN有効化が必須**  
  PiperはCAN経由で通信します。`can_activate.sh` などのスクリプトで有効化してから起動してください。

- **自動Enable失敗時は終了**  
  auto_enable を使う場合、5秒以内に有効化されないと終了する仕様があります。

- `start_single_piper.launch.py` では `joint_ctrl_single` を `/joint_states` にリマップ可能。
- dual構成では `start_two_piper.launch.py` により左右へリマップされる。

---

## 8. メッセージ定義（参照）

### piper_msgs/PosCmd
```
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
float64 gripper
int32 mode1
int32 mode2
```

### piper_msgs/PiperStatusMsg
```
uint8 ctrl_mode
uint8 arm_status
uint8 mode_feedback
uint8 teach_status
uint8 motion_status
uint8 trajectory_num
int64 err_code
bool joint_1_angle_limit
bool joint_2_angle_limit
bool joint_3_angle_limit
bool joint_4_angle_limit
bool joint_5_angle_limit
bool joint_6_angle_limit
bool communication_status_joint_1
bool communication_status_joint_2
bool communication_status_joint_3
bool communication_status_joint_4
bool communication_status_joint_5
bool communication_status_joint_6
```
