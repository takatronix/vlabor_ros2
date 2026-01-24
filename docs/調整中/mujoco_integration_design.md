# Mujoco シミュレーション連携設計書

## 概要

SO101デュアルアームロボットのMujocoシミュレーション連携設計。VRテレオペレーション、学習、ポリシー実行をシミュレーション環境で実施可能にする。

## 目的

1. **事前検証**: 実機前にVRテレオペやポリシーをテスト
2. **データ拡張**: シミュレーションで追加のデモデータを収集
3. **安全性**: 危険な動作を実機前に検出
4. **学習効率**: ドメインランダマイゼーションで汎化性能向上

---

## システムアーキテクチャ

### 全体構成

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Mujoco Simulation Environment                        │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                         Mujoco Physics Engine                          │ │
│  │  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐    │ │
│  │  │   SO101 Left    │    │   SO101 Right   │    │   Environment   │    │ │
│  │  │   Arm Model     │    │   Arm Model     │    │   (Table,       │    │ │
│  │  │   (MJCF)        │    │   (MJCF)        │    │   Objects)      │    │ │
│  │  └─────────────────┘    └─────────────────┘    └─────────────────┘    │ │
│  │                                                                        │ │
│  │  ┌─────────────────────────────────────────────────────────────────┐  │ │
│  │  │                     Virtual Cameras                              │  │ │
│  │  │   cam_front (RGB)    cam_wrist_left    cam_wrist_right          │  │ │
│  │  └─────────────────────────────────────────────────────────────────┘  │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ Python API
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         mujoco_ros2_bridge Node                              │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │  Joint State    │    │  Camera         │    │  Control        │         │
│  │  Publisher      │    │  Publisher      │    │  Subscriber     │         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ ROS2 Topics
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              ROS2 System                                     │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │  VR Dual Arm    │    │  LeRobot        │    │  Policy Runner  │         │
│  │  Control Node   │    │  Data Recorder  │    │  Node           │         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
│           ▲                     │                       │                   │
│           │                     │                       │                   │
│           │              ┌──────▼──────┐        ┌───────▼───────┐          │
│           │              │  ROS2 Bag   │        │  Trained      │          │
│           │              │  (Demo)     │        │  Policy       │          │
│  ┌────────┴────────┐     └─────────────┘        └───────────────┘          │
│  │  VR (Quest 3)   │                                                        │
│  │  または                                                                   │
│  │  キーボード制御  │                                                         │
│  └─────────────────┘                                                        │
└─────────────────────────────────────────────────────────────────────────────┘
```

### トピックマッピング

| シミュレーション | 実機 | 説明 |
|-----------------|------|------|
| `/mujoco/left_arm/joint_states` | `/left_arm/joint_states` | 左アーム関節状態 |
| `/mujoco/right_arm/joint_states` | `/right_arm/joint_states` | 右アーム関節状態 |
| `/mujoco/left_arm/joint_commands` | `/left_arm/joint_commands` | 左アーム関節指令 |
| `/mujoco/right_arm/joint_commands` | `/right_arm/joint_commands` | 右アーム関節指令 |
| `/mujoco/cam_front/image_raw` | `/cam_front/image_raw` | 前面カメラ画像 |

remapを使用して、シミュレーションと実機を同じノードで制御可能。

---

## SO101 MJCFモデル

### ファイル構成

```
so101_description/
├── urdf/
│   └── so101_new_calib.urdf.xacro   # 既存URDF
├── mjcf/
│   ├── so101_arm.xml                 # 単一アームMJCF
│   ├── so101_dual_arm.xml            # デュアルアームMJCF
│   └── assets/
│       ├── meshes/                   # STL/OBJメッシュ
│       └── textures/                 # テクスチャ
└── config/
    └── mujoco_params.yaml            # Mujoco物理パラメータ
```

### so101_arm.xml (単一アーム)

```xml
<mujoco model="so101_arm">
  <compiler angle="radian" meshdir="assets/meshes"/>

  <default>
    <joint damping="0.5" armature="0.01"/>
    <motor ctrllimited="true" ctrlrange="-3.14 3.14"/>
    <geom type="mesh" rgba="0.8 0.8 0.8 1"/>
  </default>

  <asset>
    <mesh name="base_link" file="base_link.stl"/>
    <mesh name="shoulder" file="shoulder.stl"/>
    <mesh name="upper_arm" file="upper_arm.stl"/>
    <mesh name="forearm" file="forearm.stl"/>
    <mesh name="wrist" file="wrist.stl"/>
    <mesh name="gripper" file="gripper.stl"/>
  </asset>

  <worldbody>
    <body name="base_link" pos="0 0 0">
      <geom mesh="base_link"/>

      <body name="shoulder_link" pos="0 0 0.05">
        <joint name="shoulder_pan" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
        <geom mesh="shoulder"/>

        <body name="upper_arm_link" pos="0 0.03 0.04">
          <joint name="shoulder_lift" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
          <geom mesh="upper_arm"/>

          <body name="forearm_link" pos="0 0 0.12">
            <joint name="elbow_flex" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
            <geom mesh="forearm"/>

            <body name="wrist_link" pos="0 0 0.1">
              <joint name="wrist_flex" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
              <geom mesh="wrist"/>

              <body name="wrist_roll_link" pos="0 0 0.03">
                <joint name="wrist_roll" type="hinge" axis="0 0 1" range="-3.14 3.14"/>

                <body name="gripper_link" pos="0 0 0.02">
                  <joint name="gripper" type="slide" axis="1 0 0" range="0 0.04"/>
                  <geom mesh="gripper"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="shoulder_pan_motor" joint="shoulder_pan"/>
    <motor name="shoulder_lift_motor" joint="shoulder_lift"/>
    <motor name="elbow_flex_motor" joint="elbow_flex"/>
    <motor name="wrist_flex_motor" joint="wrist_flex"/>
    <motor name="wrist_roll_motor" joint="wrist_roll"/>
    <motor name="gripper_motor" joint="gripper"/>
  </actuator>

  <sensor>
    <jointpos name="shoulder_pan_pos" joint="shoulder_pan"/>
    <jointpos name="shoulder_lift_pos" joint="shoulder_lift"/>
    <jointpos name="elbow_flex_pos" joint="elbow_flex"/>
    <jointpos name="wrist_flex_pos" joint="wrist_flex"/>
    <jointpos name="wrist_roll_pos" joint="wrist_roll"/>
    <jointpos name="gripper_pos" joint="gripper"/>
  </sensor>
</mujoco>
```

### so101_dual_arm.xml (デュアルアーム環境)

```xml
<mujoco model="so101_dual_arm_env">
  <include file="so101_arm.xml"/>

  <compiler angle="radian"/>

  <worldbody>
    <!-- 左アーム（base_linkをinclude） -->
    <body name="left_arm_base" pos="-0.2 0 0.8">
      <include file="so101_arm.xml"/>
    </body>

    <!-- 右アーム（ミラー配置） -->
    <body name="right_arm_base" pos="0.2 0 0.8" euler="0 0 3.14">
      <include file="so101_arm.xml"/>
    </body>

    <!-- テーブル -->
    <body name="table" pos="0 0.5 0.4">
      <geom type="box" size="0.6 0.4 0.02" rgba="0.6 0.4 0.2 1"/>
    </body>

    <!-- 操作対象物 -->
    <body name="cube" pos="0 0.5 0.45">
      <joint type="free"/>
      <geom type="box" size="0.03 0.03 0.03" rgba="1 0 0 1" mass="0.1"/>
    </body>

    <!-- カメラ -->
    <camera name="cam_front" pos="0 1.5 1.2" xyaxes="1 0 0 0 -0.5 0.866"/>
    <camera name="cam_top" pos="0 0.5 2.0" xyaxes="1 0 0 0 1 0"/>
  </worldbody>

  <actuator>
    <!-- 左アームアクチュエータ -->
    <motor name="left_shoulder_pan" joint="left_arm/shoulder_pan"/>
    <motor name="left_shoulder_lift" joint="left_arm/shoulder_lift"/>
    <motor name="left_elbow_flex" joint="left_arm/elbow_flex"/>
    <motor name="left_wrist_flex" joint="left_arm/wrist_flex"/>
    <motor name="left_wrist_roll" joint="left_arm/wrist_roll"/>
    <motor name="left_gripper" joint="left_arm/gripper"/>

    <!-- 右アームアクチュエータ -->
    <motor name="right_shoulder_pan" joint="right_arm/shoulder_pan"/>
    <motor name="right_shoulder_lift" joint="right_arm/shoulder_lift"/>
    <motor name="right_elbow_flex" joint="right_arm/elbow_flex"/>
    <motor name="right_wrist_flex" joint="right_arm/wrist_flex"/>
    <motor name="right_wrist_roll" joint="right_arm/wrist_roll"/>
    <motor name="right_gripper" joint="right_arm/gripper"/>
  </actuator>
</mujoco>
```

---

## mujoco_ros2_bridge ノード

### 概要

Mujoco Python APIとROS2をブリッジするノード。

### 機能

1. **物理シミュレーション実行**
2. **関節状態パブリッシュ** (sensor_msgs/JointState)
3. **関節コマンドサブスクライブ** (sensor_msgs/JointState)
4. **カメラ画像パブリッシュ** (sensor_msgs/Image)
5. **TFパブリッシュ** (フレーム変換)

### mujoco_ros2_bridge.py

```python
#!/usr/bin/env python3
"""
Mujoco ROS2 Bridge Node
MujocoシミュレーションとROS2をブリッジするノード
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import mujoco
import numpy as np
from cv_bridge import CvBridge


class MujocoRos2Bridge(Node):
    """Mujoco - ROS2 ブリッジノード"""

    JOINT_NAMES = [
        'shoulder_pan', 'shoulder_lift', 'elbow_flex',
        'wrist_flex', 'wrist_roll', 'gripper'
    ]

    def __init__(self):
        super().__init__('mujoco_ros2_bridge')

        # パラメータ
        self.declare_parameter('model_path', '')
        self.declare_parameter('render', True)
        self.declare_parameter('sim_rate', 1000)  # Hz
        self.declare_parameter('publish_rate', 50)  # Hz
        self.declare_parameter('camera_rate', 30)  # Hz

        model_path = self.get_parameter('model_path').value
        self.render_enabled = self.get_parameter('render').value

        # Mujocoモデル読み込み
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # レンダラー（オプション）
        if self.render_enabled:
            self.renderer = mujoco.Renderer(self.model, 640, 480)

        # CV Bridge
        self.cv_bridge = CvBridge()

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # パブリッシャー
        self.left_joint_pub = self.create_publisher(
            JointState, '/mujoco/left_arm/joint_states', 10)
        self.right_joint_pub = self.create_publisher(
            JointState, '/mujoco/right_arm/joint_states', 10)
        self.camera_pub = self.create_publisher(
            Image, '/mujoco/cam_front/image_raw', 10)

        # サブスクライバー
        self.left_cmd_sub = self.create_subscription(
            JointState, '/mujoco/left_arm/joint_commands',
            self.left_command_callback, 10)
        self.right_cmd_sub = self.create_subscription(
            JointState, '/mujoco/right_arm/joint_commands',
            self.right_command_callback, 10)

        # 制御コマンドバッファ
        self.left_cmd = np.zeros(6)
        self.right_cmd = np.zeros(6)

        # タイマー
        sim_period = 1.0 / self.get_parameter('sim_rate').value
        pub_period = 1.0 / self.get_parameter('publish_rate').value
        cam_period = 1.0 / self.get_parameter('camera_rate').value

        self.sim_timer = self.create_timer(sim_period, self.simulation_step)
        self.pub_timer = self.create_timer(pub_period, self.publish_state)
        self.cam_timer = self.create_timer(cam_period, self.publish_camera)

        self.get_logger().info('Mujoco ROS2 Bridge initialized')

    def simulation_step(self):
        """シミュレーションステップ実行"""
        # 制御入力を設定
        self.data.ctrl[:6] = self.left_cmd
        self.data.ctrl[6:12] = self.right_cmd

        # シミュレーションステップ
        mujoco.mj_step(self.model, self.data)

    def publish_state(self):
        """関節状態をパブリッシュ"""
        now = self.get_clock().now().to_msg()

        # 左アーム
        left_msg = JointState()
        left_msg.header.stamp = now
        left_msg.header.frame_id = 'left_arm_base'
        left_msg.name = [f'left_{j}' for j in self.JOINT_NAMES]
        left_msg.position = self.data.qpos[:6].tolist()
        left_msg.velocity = self.data.qvel[:6].tolist()
        self.left_joint_pub.publish(left_msg)

        # 右アーム
        right_msg = JointState()
        right_msg.header.stamp = now
        right_msg.header.frame_id = 'right_arm_base'
        right_msg.name = [f'right_{j}' for j in self.JOINT_NAMES]
        right_msg.position = self.data.qpos[6:12].tolist()
        right_msg.velocity = self.data.qvel[6:12].tolist()
        self.right_joint_pub.publish(right_msg)

    def publish_camera(self):
        """カメラ画像をパブリッシュ"""
        if not self.render_enabled:
            return

        # レンダリング
        self.renderer.update_scene(self.data, camera='cam_front')
        image = self.renderer.render()

        # ROS2メッセージに変換
        msg = self.cv_bridge.cv2_to_imgmsg(image, encoding='rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'cam_front'
        self.camera_pub.publish(msg)

    def left_command_callback(self, msg: JointState):
        """左アームコマンド受信"""
        if len(msg.position) >= 6:
            self.left_cmd = np.array(msg.position[:6])

    def right_command_callback(self, msg: JointState):
        """右アームコマンド受信"""
        if len(msg.position) >= 6:
            self.right_cmd = np.array(msg.position[:6])


def main(args=None):
    rclpy.init(args=args)
    node = MujocoRos2Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 使用シナリオ

### 1. VRテレオペ事前検証

```bash
# Mujocoシミュレーション起動
ros2 run mujoco_ros2 mujoco_ros2_bridge --ros-args \
    -p model_path:=/path/to/so101_dual_arm.xml

# VRテレオペノード起動（トピックをリマップ）
ros2 run unity_robot_control vr_dual_arm_control_node --ros-args \
    -r /left_arm/joint_commands:=/mujoco/left_arm/joint_commands \
    -r /right_arm/joint_commands:=/mujoco/right_arm/joint_commands

# Unity VR接続
ros2 run ros_tcp_endpoint default_server_endpoint
```

### 2. シミュレーションデータ収集

```bash
# Mujocoシミュレーション + データ収集
ros2 launch mujoco_ros2 mujoco_teleop_record.launch.py \
    model_path:=/path/to/so101_dual_arm.xml \
    bag_output:=/path/to/sim_demo_001
```

### 3. ポリシーテスト

```bash
# シミュレーションでポリシー実行
ros2 run so101_ros2_bridge policy_runner_ros2_node --ros-args \
    -p policy_path:=/path/to/trained_policy \
    -r /left_arm/joint_commands:=/mujoco/left_arm/joint_commands \
    -r /right_arm/joint_commands:=/mujoco/right_arm/joint_commands \
    -r /cam_front/image_raw:=/mujoco/cam_front/image_raw
```

---

## ドメインランダマイゼーション

学習時の汎化性能向上のため、以下のランダマイゼーションを実装:

### 物理パラメータ

```python
# 摩擦係数
model.geom_friction[:] *= np.random.uniform(0.8, 1.2)

# 質量
model.body_mass[:] *= np.random.uniform(0.9, 1.1)

# 関節ダンピング
model.dof_damping[:] *= np.random.uniform(0.8, 1.2)
```

### 視覚パラメータ

```python
# 照明
model.light_diffuse[:] = np.random.uniform(0.5, 1.0, 3)

# カメラ位置ノイズ
cam_pos += np.random.normal(0, 0.01, 3)

# テクスチャ変更
# ...
```

### 環境パラメータ

```python
# 物体初期位置
obj_pos = base_pos + np.random.uniform(-0.1, 0.1, 3)

# 物体サイズ
obj_size *= np.random.uniform(0.9, 1.1)
```

---

## 実装ロードマップ

### Phase 1: 基本シミュレーション

- [ ] SO101 MJCFモデル作成（URDFから変換）
- [ ] mujoco_ros2_bridge基本実装
- [ ] 関節状態パブリッシュ/サブスクライブ
- [ ] ビューアー表示

### Phase 2: VR連携

- [ ] VRテレオペとの接続テスト
- [ ] リアルタイム性能最適化
- [ ] 遅延計測・調整

### Phase 3: 学習連携

- [ ] カメラ画像パブリッシュ
- [ ] LeRobotデータ収集連携
- [ ] ドメインランダマイゼーション実装

### Phase 4: Sim-to-Real

- [ ] 物理パラメータ同定
- [ ] ビジュアルドメイン適応
- [ ] 実機転移テスト

---

## 必要なパッケージ

### Python

```
mujoco>=3.0.0
mujoco-python-viewer  # オプション（GUIビューアー）
opencv-python
numpy
```

### ROS2

```
ros-humble-cv-bridge
ros-humble-tf2-ros
ros-humble-image-transport
```

---

## 参考リンク

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MuJoCo Python Bindings](https://mujoco.readthedocs.io/en/stable/python.html)
- [URDF to MJCF Converter](https://github.com/openai/mujoco-py/blob/master/mujoco_py/utils.py)
- [LeRobot Simulation Environments](https://github.com/huggingface/lerobot)
