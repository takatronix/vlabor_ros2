# マルチロボットアーキテクチャ設計書

## 概要

本システムは複数のロボットアーム（SO101、AgileX Piper、Daihenアーム等）に対応するモジュール化された設計を採用する。ロボットの切り替えは設定ファイルの変更のみで可能。

---

## 重要：IKソルバーは機種非依存

**IKソルバーはURDFベースで動作するため、ロボット固有のコードは不要です。**

```
┌─────────────────────────────────────────────────────────────────┐
│                    IKソルバーの仕組み                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   URDF (ロボットモデル)                                          │
│        │                                                        │
│        ▼                                                        │
│   ┌─────────────┐                                               │
│   │ KDL Parser  │  ← URDFを解析してキネマティクスチェーンを構築    │
│   └─────────────┘                                               │
│        │                                                        │
│        ▼                                                        │
│   ┌─────────────┐                                               │
│   │ KDL Chain   │  ← リンク・ジョイントの構造（自動生成）          │
│   └─────────────┘                                               │
│        │                                                        │
│        ▼                                                        │
│   ┌─────────────┐                                               │
│   │ IK Solver   │  ← 汎用的な逆運動学計算（数値解法）             │
│   └─────────────┘                                               │
│                                                                 │
│   どのロボットでも同じコードで動作！                              │
│   必要なのはURDFファイルのみ                                      │
└─────────────────────────────────────────────────────────────────┘
```

### 機種ごとに必要なもの

| コンポーネント | SO101 | Piper | Daihen | 説明 |
|--------------|:-----:|:-----:|:------:|------|
| IKソルバー | 共通 | 共通 | 共通 | **コード変更不要** |
| URDF | 必要 | 必要 | 必要 | モデルファイルのみ |
| ドライバー | 専用 | 専用 | 専用 | 通信方式が異なる |
| 設定ファイル | 必要 | 必要 | 必要 | YAML |

**結論：新しいロボットを追加する際、IKソルバーのコードは一切変更不要。URDFファイルを用意するだけ。**

---

## ROS2ブリッジ（ドライバー）の役割

**ROS2ブリッジ = ロボット固有の通信ドライバー**

```
┌─────────────────────────────────────────────────────────────────┐
│                 ROS2ブリッジの役割                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ROS2トピック                    物理ロボット                     │
│  (共通インターフェース)            (機種固有の通信)                │
│                                                                 │
│  /left_arm/joint_commands  ──┐                                  │
│  /right_arm/joint_commands ──┼──▶ ┌─────────────────┐           │
│                               │   │  ROS2 Bridge    │           │
│  /left_arm/joint_states   ◀──┼── │  (ドライバー)     │──▶ ロボット │
│  /right_arm/joint_states  ◀──┘   │                 │           │
│                                   └─────────────────┘           │
│                                          │                      │
│                                          ▼                      │
│                                  ┌───────────────────┐          │
│                                  │ 通信プロトコル     │          │
│                                  │ • SO101: Feetech  │          │
│                                  │ • Piper: CAN      │          │
│                                  │ • Daihen: TCP/IP  │          │
│                                  └───────────────────┘          │
└─────────────────────────────────────────────────────────────────┘
```

### SO101 ROS2ブリッジの具体的な機能

現在の `so101_ros2_bridge` パッケージは以下を行います：

| 機能 | 説明 |
|------|------|
| **関節状態の取得** | LeRobotライブラリ経由でFeetechサーボから角度を読み取り |
| **関節コマンドの送信** | ROS2トピックから受け取った目標角度をサーボに送信 |
| **単位変換** | ラジアン ↔ 度数 / 正規化値の変換 |
| **キャリブレーション** | モーターの原点合わせ・範囲設定 |
| **ウォッチドッグ** | 通信異常時の安全停止 |

### なぜロボットごとにドライバーが必要？

```
┌────────────────────────────────────────────────────────────────┐
│                  通信方式の違い                                  │
├──────────────┬──────────────┬──────────────┬──────────────────┤
│              │   SO101      │   Piper      │   Daihen        │
├──────────────┼──────────────┼──────────────┼──────────────────┤
│ 物理層        │ USB Serial   │ CAN Bus      │ Ethernet        │
│ プロトコル    │ Feetech SCS  │ AgileX CAN   │ Daihen TCP      │
│ ライブラリ    │ LeRobot      │ python-can   │ socket          │
│ ボーレート    │ 1Mbps        │ 1Mbps        │ N/A             │
│ 応答時間      │ ~5ms         │ ~2ms         │ ~10ms           │
└──────────────┴──────────────┴──────────────┴──────────────────┘

→ ハードウェア通信部分のみロボット固有のコードが必要
→ ROS2トピックのインターフェースは共通化可能
```

### 各レイヤーの依存関係まとめ

```
┌────────────────────────────────────────────────────────────────┐
│                                                                │
│   VR制御層          ← 機種非依存（変更不要）                     │
│       │                                                        │
│       ▼                                                        │
│   IKソルバー層       ← 機種非依存（URDFを切り替えるだけ）         │
│       │                                                        │
│       ▼                                                        │
│   ──────────────────────────────────────                       │
│   ROS2ブリッジ層     ← 機種依存（ドライバー実装が必要）          │
│   ──────────────────────────────────────                       │
│       │                                                        │
│       ▼                                                        │
│   物理ロボット                                                  │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

---

## 対応ロボット一覧

| ロボット | メーカー | 関節数 | 特徴 |
|---------|---------|--------|------|
| SO101 | TheRobotStudio/HuggingFace | 6 | 教育・研究用、Feetechサーボ |
| AgileX Piper | AgileX Robotics | 6 | 協働ロボット、CAN通信 |
| Daihen OTC | ダイヘン | 6 | 産業用溶接ロボット |

---

## アーキテクチャ全体図

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         VR Teleoperation Layer                               │
│  ┌─────────────────────────────────────────────────────────────────────────┐│
│  │                    vr_dual_arm_control_node                              ││
│  │                    (ロボット非依存)                                       ││
│  └─────────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                    /left_arm/target_pose, /right_arm/target_pose
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           IK Solver Layer                                    │
│  ┌─────────────────────────────────────────────────────────────────────────┐│
│  │                    generic_ik_solver_node                                ││
│  │                    (URDFベース、ロボット設定で切り替え)                    ││
│  └─────────────────────────────────────────────────────────────────────────┘│
│                                    │                                         │
│         ┌──────────────────────────┼──────────────────────────┐             │
│         ▼                          ▼                          ▼             │
│  ┌─────────────┐           ┌─────────────┐           ┌─────────────┐       │
│  │ SO101 URDF  │           │ Piper URDF  │           │ Daihen URDF │       │
│  └─────────────┘           └─────────────┘           └─────────────┘       │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                    /left_arm/joint_commands, /right_arm/joint_commands
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Robot Driver Layer                                   │
│         ┌──────────────────────────┼──────────────────────────┐             │
│         ▼                          ▼                          ▼             │
│  ┌─────────────┐           ┌─────────────┐           ┌─────────────┐       │
│  │ SO101 Bridge│           │ Piper Bridge│           │ Daihen      │       │
│  │ (Feetech)   │           │ (CAN)       │           │ Bridge      │       │
│  └─────────────┘           └─────────────┘           └─────────────┘       │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            Physical Robot                                    │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## モジュール構成

### 1. ロボット設定ファイル

各ロボットの設定は `config/robots/` に配置：

```
unity_robot_control/
├── config/
│   ├── robots/
│   │   ├── so101.yaml           # SO101設定
│   │   ├── piper.yaml           # AgileX Piper設定
│   │   └── daihen.yaml          # Daihenアーム設定
│   ├── vr_dual_arm_config.yaml  # VR制御設定（ロボット非依存）
│   └── active_robot.yaml        # 現在アクティブなロボットを指定
```

### 2. ファイル構成

```
unity_robot_control/
├── unity_robot_control/
│   ├── vr_dual_arm_control_node.py    # VR制御（変更不要）
│   ├── generic_ik_solver_node.py      # 汎用IKソルバー（新規）
│   ├── robot_config_loader.py         # ロボット設定ローダー（新規）
│   └── drivers/                        # ロボットドライバー（新規）
│       ├── __init__.py
│       ├── base_driver.py             # 基底クラス
│       ├── so101_driver.py
│       ├── piper_driver.py
│       └── daihen_driver.py
├── urdf/                               # URDFファイル
│   ├── so101/
│   ├── piper/
│   └── daihen/
└── launch/
    └── vr_teleop.launch.py            # 統合起動ファイル
```

---

## 設定ファイル詳細

### active_robot.yaml（ロボット選択）

```yaml
# アクティブなロボットを指定
# 起動時にこのファイルを参照してロボット設定を読み込む
active_robot: "so101"  # "so101" | "piper" | "daihen"

# デュアルアーム設定
dual_arm:
  enabled: true
  left_arm: "so101"    # 左アームのロボット種別
  right_arm: "so101"   # 右アームのロボット種別（異種混合も可能）
```

### so101.yaml

```yaml
robot:
  name: "so101"
  type: "so101_follower"
  manufacturer: "TheRobotStudio"

  # 関節設定
  joints:
    count: 6
    names:
      - "shoulder_pan"
      - "shoulder_lift"
      - "elbow_flex"
      - "wrist_flex"
      - "wrist_roll"
      - "gripper"

  # 関節制限 (ラジアン)
  joint_limits:
    shoulder_pan:  { min: -3.14, max: 3.14 }
    shoulder_lift: { min: -1.57, max: 1.57 }
    elbow_flex:    { min: -1.57, max: 1.57 }
    wrist_flex:    { min: -1.57, max: 1.57 }
    wrist_roll:    { min: -3.14, max: 3.14 }
    gripper:       { min: 0.0,   max: 0.04 }

  # URDFファイル
  urdf:
    package: "so101_description"
    file: "urdf/so101_new_calib.urdf.xacro"

  # リンク名
  links:
    base: "base_link"
    end_effector: "gripper_link"

  # 通信設定
  communication:
    type: "serial"
    port: "/dev/ttyACM0"
    baudrate: 1000000
    protocol: "feetech"

  # LeRobot設定
  lerobot:
    robot_type: "so101_follower"
    use_degrees: false
    motor_type: "sts3215"
```

### piper.yaml

```yaml
robot:
  name: "piper"
  type: "agilex_piper"
  manufacturer: "AgileX Robotics"

  # 関節設定
  joints:
    count: 6
    names:
      - "joint1"
      - "joint2"
      - "joint3"
      - "joint4"
      - "joint5"
      - "joint6"

  # 関節制限 (ラジアン)
  joint_limits:
    joint1: { min: -2.618, max: 2.618 }
    joint2: { min: -2.094, max: 2.094 }
    joint3: { min: -2.618, max: 2.618 }
    joint4: { min: -2.094, max: 2.094 }
    joint5: { min: -2.618, max: 2.618 }
    joint6: { min: -3.14,  max: 3.14 }

  # URDFファイル
  urdf:
    package: "piper_description"
    file: "urdf/piper.urdf.xacro"

  # リンク名
  links:
    base: "base_link"
    end_effector: "link6"

  # 通信設定
  communication:
    type: "can"
    interface: "can0"
    bitrate: 1000000
    protocol: "agilex_can"

  # LeRobot設定（カスタム）
  lerobot:
    robot_type: "agilex_piper"
    use_degrees: false
```

### daihen.yaml

```yaml
robot:
  name: "daihen_otc"
  type: "daihen_arm"
  manufacturer: "Daihen"

  # 関節設定
  joints:
    count: 6
    names:
      - "s_axis"   # 旋回
      - "l_axis"   # 下腕
      - "u_axis"   # 上腕
      - "r_axis"   # 手首旋回
      - "b_axis"   # 手首曲げ
      - "t_axis"   # 手首ひねり

  # 関節制限 (ラジアン) - 産業用は広い
  joint_limits:
    s_axis: { min: -2.967, max: 2.967 }
    l_axis: { min: -1.047, max: 2.443 }
    u_axis: { min: -2.356, max: 2.443 }
    r_axis: { min: -3.49,  max: 3.49 }
    b_axis: { min: -2.094, max: 2.094 }
    t_axis: { min: -6.28,  max: 6.28 }

  # URDFファイル
  urdf:
    package: "daihen_description"
    file: "urdf/daihen_otc.urdf.xacro"

  # リンク名
  links:
    base: "base_link"
    end_effector: "tool0"

  # 通信設定
  communication:
    type: "ethernet"
    ip: "192.168.1.100"
    port: 10000
    protocol: "daihen_tcp"

  # LeRobot設定（カスタム）
  lerobot:
    robot_type: "daihen_otc"
    use_degrees: true  # 産業用は度数法が多い
```

---

## 変更が必要な箇所一覧

### ロボット追加時のチェックリスト

| 項目 | ファイル | 変更内容 |
|------|---------|---------|
| 1. ロボット設定 | `config/robots/{robot}.yaml` | 新規作成 |
| 2. URDF | `urdf/{robot}/` | URDFファイル配置 |
| 3. ドライバー | `drivers/{robot}_driver.py` | 通信ドライバー作成 |
| 4. LeRobot設定 | `lerobot/robots/{robot}/` | オプション：LeRobot対応 |

### 各コンポーネントの役割

```
┌──────────────────────────────────────────────────────────────────┐
│                    変更不要なコンポーネント                        │
├──────────────────────────────────────────────────────────────────┤
│ • vr_dual_arm_control_node.py  - VR入力処理                       │
│ • generic_ik_solver_node.py    - IK計算（URDFベース）             │
│ • rosbag_to_lerobot.py         - データ変換                       │
│ • vr_dual_arm_config.yaml      - VR制御設定                       │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│                    ロボット追加時に変更/追加                       │
├──────────────────────────────────────────────────────────────────┤
│ • config/robots/{robot}.yaml   - ロボット固有設定                  │
│ • urdf/{robot}/                - URDFファイル                      │
│ • drivers/{robot}_driver.py    - 通信ドライバー                    │
│ • active_robot.yaml            - アクティブロボット指定             │
└──────────────────────────────────────────────────────────────────┘
```

---

## IKソルバーのモジュール化

### 汎用IKソルバー設計

```python
# generic_ik_solver_node.py

class GenericIKSolverNode(Node):
    """
    汎用IKソルバーノード
    URDFベースで任意のロボットに対応
    """

    def __init__(self, node_name='generic_ik_solver_node'):
        super().__init__(node_name)

        # ロボット設定をロード
        self.declare_parameter('robot_config', '')
        robot_config_path = self.get_parameter('robot_config').value

        self.robot_config = self._load_robot_config(robot_config_path)

        # URDFからパラメータを自動設定
        self.joint_names = self.robot_config['joints']['names']
        self.base_link = self.robot_config['links']['base']
        self.ee_link = self.robot_config['links']['end_effector']

        # URDFファイル取得
        urdf_file = self._get_urdf_path(self.robot_config['urdf'])

        # KDL初期化
        self._init_kdl(urdf_file)

    def _load_robot_config(self, config_path: str) -> dict:
        """ロボット設定YAMLをロード"""
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)['robot']

    def _get_urdf_path(self, urdf_config: dict) -> str:
        """URDFファイルのパスを取得"""
        pkg_dir = get_package_share_directory(urdf_config['package'])
        return os.path.join(pkg_dir, urdf_config['file'])
```

### IKソルバーの切り替え

```yaml
# launch時のパラメータでロボットを切り替え

# SO101の場合
ros2 launch unity_robot_control vr_teleop.launch.py robot:=so101

# Piperの場合
ros2 launch unity_robot_control vr_teleop.launch.py robot:=piper

# Daihenの場合
ros2 launch unity_robot_control vr_teleop.launch.py robot:=daihen
```

---

## ロボットドライバーの抽象化

### 基底クラス

```python
# drivers/base_driver.py

from abc import ABC, abstractmethod
from typing import Dict, List, Any


class BaseRobotDriver(ABC):
    """ロボットドライバー基底クラス"""

    def __init__(self, config: dict):
        self.config = config
        self.joint_names = config['joints']['names']
        self.is_connected = False

    @abstractmethod
    def connect(self) -> bool:
        """ロボットに接続"""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """ロボットから切断"""
        pass

    @abstractmethod
    def get_joint_positions(self) -> Dict[str, float]:
        """現在の関節角度を取得"""
        pass

    @abstractmethod
    def set_joint_positions(self, positions: Dict[str, float]) -> bool:
        """目標関節角度を設定"""
        pass

    @abstractmethod
    def get_joint_velocities(self) -> Dict[str, float]:
        """現在の関節速度を取得"""
        pass

    def enable_torque(self) -> None:
        """トルクを有効化（オプション）"""
        pass

    def disable_torque(self) -> None:
        """トルクを無効化（オプション）"""
        pass
```

### SO101ドライバー

```python
# drivers/so101_driver.py

from .base_driver import BaseRobotDriver
from lerobot.motors.feetech import FeetechMotorsBus


class SO101Driver(BaseRobotDriver):
    """SO101ロボットドライバー"""

    def __init__(self, config: dict):
        super().__init__(config)

        comm_config = config['communication']
        self.port = comm_config['port']

        # Feetechモーターバス
        self.bus = None

    def connect(self) -> bool:
        from lerobot.motors import Motor, MotorNormMode

        motors = {
            name: Motor(i + 1, "sts3215", MotorNormMode.RANGE_M100_100)
            for i, name in enumerate(self.joint_names)
        }

        self.bus = FeetechMotorsBus(port=self.port, motors=motors)
        self.bus.connect()
        self.is_connected = True
        return True

    def disconnect(self) -> None:
        if self.bus:
            self.bus.disconnect()
        self.is_connected = False

    def get_joint_positions(self) -> Dict[str, float]:
        return self.bus.sync_read("Present_Position")

    def set_joint_positions(self, positions: Dict[str, float]) -> bool:
        self.bus.sync_write("Goal_Position", positions)
        return True

    def get_joint_velocities(self) -> Dict[str, float]:
        return self.bus.sync_read("Present_Speed")
```

### AgileX Piperドライバー

```python
# drivers/piper_driver.py

from .base_driver import BaseRobotDriver


class PiperDriver(BaseRobotDriver):
    """AgileX Piperロボットドライバー"""

    def __init__(self, config: dict):
        super().__init__(config)

        comm_config = config['communication']
        self.can_interface = comm_config['interface']
        self.bitrate = comm_config['bitrate']

        self.can_bus = None

    def connect(self) -> bool:
        import can

        self.can_bus = can.interface.Bus(
            channel=self.can_interface,
            bustype='socketcan',
            bitrate=self.bitrate
        )
        self.is_connected = True
        return True

    def disconnect(self) -> None:
        if self.can_bus:
            self.can_bus.shutdown()
        self.is_connected = False

    def get_joint_positions(self) -> Dict[str, float]:
        # CAN通信で関節角度を取得
        positions = {}
        for i, name in enumerate(self.joint_names):
            # Piper固有のCANプロトコル
            msg = self._send_can_query(0x100 + i)
            positions[name] = self._parse_position(msg)
        return positions

    def set_joint_positions(self, positions: Dict[str, float]) -> bool:
        # CAN通信で目標角度を送信
        for i, name in enumerate(self.joint_names):
            data = self._encode_position(positions[name])
            msg = can.Message(arbitration_id=0x200 + i, data=data)
            self.can_bus.send(msg)
        return True

    def _send_can_query(self, can_id: int):
        # 実装詳細
        pass

    def _parse_position(self, msg) -> float:
        # 実装詳細
        pass

    def _encode_position(self, position: float) -> bytes:
        # 実装詳細
        pass
```

### Daihenドライバー

```python
# drivers/daihen_driver.py

from .base_driver import BaseRobotDriver
import socket
import struct


class DaihenDriver(BaseRobotDriver):
    """Daihenロボットドライバー"""

    def __init__(self, config: dict):
        super().__init__(config)

        comm_config = config['communication']
        self.ip = comm_config['ip']
        self.port = comm_config['port']

        self.socket = None

    def connect(self) -> bool:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.ip, self.port))
        self.is_connected = True
        return True

    def disconnect(self) -> None:
        if self.socket:
            self.socket.close()
        self.is_connected = False

    def get_joint_positions(self) -> Dict[str, float]:
        # Daihen固有のTCPプロトコル
        self._send_command("GET_JOINT_POS")
        response = self._receive_response()
        return self._parse_joint_positions(response)

    def set_joint_positions(self, positions: Dict[str, float]) -> bool:
        # Daihen固有のTCPプロトコル
        cmd = self._encode_joint_command(positions)
        self._send_command(cmd)
        return True

    def _send_command(self, cmd: str):
        self.socket.send(cmd.encode())

    def _receive_response(self) -> bytes:
        return self.socket.recv(1024)

    def _parse_joint_positions(self, response: bytes) -> Dict[str, float]:
        # 実装詳細
        pass

    def _encode_joint_command(self, positions: Dict[str, float]) -> str:
        # 実装詳細
        pass
```

---

## ドライバーファクトリー

```python
# drivers/__init__.py

from .base_driver import BaseRobotDriver
from .so101_driver import SO101Driver
from .piper_driver import PiperDriver
from .daihen_driver import DaihenDriver


class RobotDriverFactory:
    """ロボットドライバーのファクトリークラス"""

    _drivers = {
        'so101': SO101Driver,
        'so101_follower': SO101Driver,
        'piper': PiperDriver,
        'agilex_piper': PiperDriver,
        'daihen': DaihenDriver,
        'daihen_otc': DaihenDriver,
    }

    @classmethod
    def create(cls, robot_type: str, config: dict) -> BaseRobotDriver:
        """ロボットタイプに応じたドライバーを生成"""
        if robot_type not in cls._drivers:
            raise ValueError(f"Unknown robot type: {robot_type}")

        driver_class = cls._drivers[robot_type]
        return driver_class(config)

    @classmethod
    def register(cls, robot_type: str, driver_class: type):
        """新しいロボットドライバーを登録"""
        cls._drivers[robot_type] = driver_class
```

---

## Launch ファイル

```python
# launch/vr_teleop.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 引数定義
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='so101',
        description='Robot type: so101, piper, daihen'
    )

    dual_arm_arg = DeclareLaunchArgument(
        'dual_arm',
        default_value='true',
        description='Enable dual arm mode'
    )

    robot = LaunchConfiguration('robot')

    # ロボット設定ファイルパス
    robot_config = PathJoinSubstitution([
        FindPackageShare('unity_robot_control'),
        'config', 'robots',
        [robot, '.yaml']
    ])

    # VR制御ノード（ロボット非依存）
    vr_control = Node(
        package='unity_robot_control',
        executable='vr_dual_arm_control_node',
        name='vr_dual_arm_control_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('unity_robot_control'),
                'config', 'vr_dual_arm_config.yaml'
            ])
        ],
    )

    # 左アームIKソルバー
    left_ik = Node(
        package='unity_robot_control',
        executable='generic_ik_solver_node',
        name='left_arm_ik_solver_node',
        parameters=[{
            'robot_config': robot_config,
            'target_pose_topic': '/left_arm/target_pose',
            'joint_angles_topic': '/left_arm/ik/joint_angles',
        }],
    )

    # 右アームIKソルバー
    right_ik = Node(
        package='unity_robot_control',
        executable='generic_ik_solver_node',
        name='right_arm_ik_solver_node',
        parameters=[{
            'robot_config': robot_config,
            'target_pose_topic': '/right_arm/target_pose',
            'joint_angles_topic': '/right_arm/ik/joint_angles',
        }],
    )

    # ロボットドライバーノード
    robot_driver = Node(
        package='unity_robot_control',
        executable='robot_driver_node',
        name='robot_driver_node',
        parameters=[{
            'robot_config': robot_config,
        }],
    )

    return LaunchDescription([
        robot_arg,
        dual_arm_arg,
        vr_control,
        left_ik,
        right_ik,
        robot_driver,
    ])
```

---

## 使用例

### ロボットの切り替え

```bash
# SO101でVRテレオペ
ros2 launch unity_robot_control vr_teleop.launch.py robot:=so101

# AgileX PiperでVRテレオペ
ros2 launch unity_robot_control vr_teleop.launch.py robot:=piper

# DaihenアームでVRテレオペ
ros2 launch unity_robot_control vr_teleop.launch.py robot:=daihen
```

### 新しいロボットの追加手順

1. **設定ファイル作成**
   ```bash
   cp config/robots/so101.yaml config/robots/new_robot.yaml
   # 設定を編集
   ```

2. **URDFファイル配置**
   ```bash
   mkdir -p urdf/new_robot/
   cp /path/to/new_robot.urdf urdf/new_robot/
   ```

3. **ドライバー作成**
   ```python
   # drivers/new_robot_driver.py
   from .base_driver import BaseRobotDriver

   class NewRobotDriver(BaseRobotDriver):
       # 実装
       pass
   ```

4. **ドライバー登録**
   ```python
   # drivers/__init__.py
   from .new_robot_driver import NewRobotDriver

   RobotDriverFactory.register('new_robot', NewRobotDriver)
   ```

5. **起動**
   ```bash
   ros2 launch unity_robot_control vr_teleop.launch.py robot:=new_robot
   ```

---

## LeRobot連携時の注意

### データセット変換

```bash
# ロボット設定を指定して変換
ros2 run unity_robot_control rosbag_to_lerobot \
    --bag-path demo_001 \
    --output-dir /path/to/dataset \
    --robot-config /path/to/config/robots/piper.yaml
```

### 関節名のマッピング

変換スクリプトはロボット設定から関節名を読み取り、LeRobot形式に変換します：

```python
# SO101: shoulder_pan.pos, shoulder_lift.pos, ...
# Piper: joint1.pos, joint2.pos, ...
# Daihen: s_axis.pos, l_axis.pos, ...
```

---

## まとめ

| コンポーネント | SO101 | Piper | Daihen | 変更箇所 |
|--------------|-------|-------|--------|---------|
| VR制御 | 共通 | 共通 | 共通 | なし |
| IKソルバー | URDF | URDF | URDF | 設定のみ |
| ドライバー | Feetech | CAN | TCP | 新規作成 |
| 設定ファイル | so101.yaml | piper.yaml | daihen.yaml | 新規作成 |
| URDF | 必要 | 必要 | 必要 | 配置 |

**ロボット切り替えは `robot:=xxx` パラメータのみで完結**
