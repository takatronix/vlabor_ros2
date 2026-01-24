# VR テレオペレーション × LeRobot 学習システム構成図

## 概要

複数のロボットアーム（SO101、Piper、Daihen等）をVR（Quest 3）でテレオペレーションし、収集したデータをLeRobotで学習させるシステムの構成図。

**特徴:**
- 🤖 マルチロボット対応（設定ファイルで切り替え）
- 🥽 VRテレオペレーション（Quest 3）
- 🧠 模倣学習（SmolVLA / ACT）
- 🎮 シミュレーション連携（MuJoCo / Isaac Sim）

---

## 0. ノード関係図（シンプル版）

```mermaid
flowchart TB
    subgraph VR["VR入力"]
        Quest3["Quest 3"]
    end

    subgraph ROS2["ROS2ノード"]
        VRCtrl["vr_dual_arm_control_node"]
        IKL["left_arm_ik_solver"]
        IKR["right_arm_ik_solver"]
        Recorder["episode_recorder_node"]
        Runner["policy_runner_node"]
    end

    subgraph Bridge["ロボットブリッジ"]
        Leader["leader_ros2_node"]
        Follower["follower_ros2_node"]
    end

    subgraph Robot["ロボット"]
        LeftArm["左アーム"]
        RightArm["右アーム"]
        Camera["カメラ"]
    end

    subgraph LeRobot["LeRobot"]
        Converter["rosbag_to_lerobot.py"]
        Dataset[("Dataset")]
        Train["train.py"]
        Model[("学習済みモデル")]
    end

    subgraph Sim["シミュレータ"]
        MuJoCo["MuJoCo"]
        Isaac["Isaac Sim"]
    end

    %% VRテレオペフロー
    Quest3 --> VRCtrl
    VRCtrl --> IKL
    VRCtrl --> IKR
    IKL --> Leader
    IKR --> Follower
    Leader --> LeftArm
    Follower --> RightArm

    %% データ収集フロー
    LeftArm -.-> Recorder
    RightArm -.-> Recorder
    Camera -.-> Recorder
    Recorder --> Converter
    Converter --> Dataset
    Dataset --> Train
    Train --> Model

    %% 推論フロー
    Model --> Runner
    Camera -.-> Runner
    LeftArm -.-> Runner
    RightArm -.-> Runner
    Runner --> Leader
    Runner --> Follower

    %% シミュレータ連携
    Leader <-.-> MuJoCo
    Follower <-.-> MuJoCo
    Leader <-.-> Isaac
    Follower <-.-> Isaac
```

### ノード一覧

| ノード | 役割 |
|--------|------|
| **vr_dual_arm_control_node** | VR入力を左右アームの目標ポーズに変換 |
| **left_arm_ik_solver** | 左アーム逆運動学（KDL） |
| **right_arm_ik_solver** | 右アーム逆運動学（KDL） |
| **leader_ros2_node** | 左アーム制御ドライバー |
| **follower_ros2_node** | 右アーム制御ドライバー |
| **episode_recorder_node** | エピソード記録（ROS2 Bag） |
| **policy_runner_node** | 学習済みモデル推論 |

### LeRobot連携

| コンポーネント | 役割 |
|---------------|------|
| **rosbag_to_lerobot.py** | ROS2 Bag → LeRobot Dataset変換 |
| **train.py** | LeRobot学習スクリプト |
| **学習済みモデル** | SmolVLA / ACT チェックポイント |

---

## 1. システム全体構成図

```mermaid
flowchart TB
    subgraph Input["🎮 Input Devices<br/>入力デバイス"]
        VR["🥽 Quest 3<br/>VRヘッドセット<br/>・左手ポーズ<br/>・右手ポーズ<br/>・ジェスチャー"]
        LeaderArm["🦾 リーダーアーム<br/>従来方式<br/>(オプション)"]
    end

    subgraph Central["🖥️ Central Server"]
        subgraph API["API Interface Layer"]
            ROS2TCP["ROS2 Unity TCP<br/>Endpoint"]
        end

        subgraph Backend["Backend Layer"]
            subgraph ROS2Nodes["ROS2 ノード群"]
                VRControl["vr_dual_arm<br/>_control_node"]
                IKLeft["left_arm<br/>_ik_solver"]
                IKRight["right_arm<br/>_ik_solver"]
                EpisodeRec["episode<br/>_recorder_node"]
                PolicyRunner["policy<br/>_runner_node"]
            end

            subgraph LeRobotCore["LeRobot コア"]
                Bridge["so101_ros2_bridge<br/>・leader_ros2_node<br/>・follower_ros2_node"]
                Converter["rosbag_to_lerobot<br/>変換スクリプト"]
                Policy["ポリシー<br/>・SmolVLA<br/>・ACT"]
            end
        end

        subgraph Storage["Storage"]
            DB[("DB<br/>・エピソード管理<br/>・学習履歴")]
            Files[("File Storage<br/>・ROS2 Bags<br/>・LeRobot Dataset<br/>・Checkpoints")]
        end
    end

    subgraph Output["🤖 Output Devices<br/>出力デバイス"]
        LeftArm["🦾 左アーム<br/>SO101 (Leader)"]
        RightArm["🦾 右アーム<br/>SO101 (Follower)"]
        Camera["📷 カメラ<br/>・cam_front (USB)<br/>・cam_side (RealSense)"]
    end

    subgraph Sim["🎮 Simulation<br/>シミュレータ"]
        MuJoCo["MuJoCo<br/>・物理演算<br/>・仮想カメラ"]
        Isaac["Isaac Sim<br/>・ドメインランダム化<br/>・大規模並列"]
    end

    subgraph Cloud["☁️ GPU Clouds<br/>学習サーバー"]
        GPU["GPU Server<br/>・A100 / H100<br/>・学習実行"]
    end

    subgraph Store["💾 Storage Server<br/>モデル保存"]
        HF["HuggingFace Hub"]
        S3["AWS S3 /<br/>Cloudflare R2"]
    end

    %% Input Connections
    VR -->|"TCP/IP<br/>WebSocket"| ROS2TCP
    LeaderArm -->|"USB Serial"| Bridge

    %% API to Backend
    ROS2TCP --> VRControl

    %% VR Control Flow
    VRControl --> IKLeft
    VRControl --> IKRight
    IKLeft --> Bridge
    IKRight --> Bridge

    %% Robot Control
    Bridge --> LeftArm
    Bridge --> RightArm

    %% Data Collection
    Camera --> EpisodeRec
    LeftArm -.->|"joint_states"| EpisodeRec
    RightArm -.->|"joint_states"| EpisodeRec

    %% Storage Flow
    EpisodeRec --> Files
    Files --> Converter
    Converter --> DB

    %% Training Flow
    DB -->|"Dataset<br/>Upload"| GPU
    GPU -->|"Trained<br/>Model"| Store
    Store -->|"Model<br/>Download"| PolicyRunner

    %% Inference Flow
    PolicyRunner --> Bridge
    Camera -.->|"image"| PolicyRunner
    LeftArm -.->|"state"| PolicyRunner
    RightArm -.->|"state"| PolicyRunner

    %% Simulation
    Bridge <-->|"ROS2 Topics"| MuJoCo
    Bridge <-->|"ROS2 Topics"| Isaac
```

---

## 2. データフロー図

```mermaid
flowchart LR
    subgraph Collect["1️⃣ データ収集"]
        direction TB
        VR1["🥽 VRテレオペ"]
        Robot1["🦾 双腕ロボット"]
        Cam1["📷 カメラ"]
        Rec["episode_recorder_node"]
        Bag[("ROS2 Bags<br/>episode_000000/<br/>episode_000001/")]

        VR1 --> Rec
        Robot1 --> Rec
        Cam1 --> Rec
        Rec --> Bag
    end

    subgraph Convert["2️⃣ データ変換"]
        direction TB
        Conv["rosbag_to_lerobot.py"]
        Dataset[("LeRobot Dataset<br/>・Parquet<br/>・MP4")]

        Bag --> Conv
        Conv --> Dataset
    end

    subgraph Train["3️⃣ 学習"]
        direction TB
        GPU["☁️ GPU Server"]
        Model[("学習済みモデル<br/>SmolVLA")]

        Dataset --> GPU
        GPU --> Model
    end

    subgraph Infer["4️⃣ 推論"]
        direction TB
        Runner["policy_runner_node"]
        Robot2["🦾 双腕ロボット"]
        Cam2["📷 カメラ"]

        Model --> Runner
        Cam2 --> Runner
        Runner --> Robot2
    end
```

---

## 3. VRテレオペレーション詳細

```mermaid
flowchart TB
    subgraph Unity["Unity (Quest 3)"]
        LeftHand["👋 左手"]
        RightHand["👋 右手"]
        Gesture["✊ ジェスチャー"]
    end

    subgraph Topics1["ROS2 Topics (VR入力)"]
        T1["/quest/left_hand/pose"]
        T2["/quest/right_hand/pose"]
        T3["/quest/hand_gesture"]
    end

    subgraph Control["VR制御ノード"]
        VRC["vr_dual_arm_control_node<br/>・VR入力をアームにマッピング<br/>・座標変換"]
    end

    subgraph Topics2["ROS2 Topics (アーム目標)"]
        T4["/left_arm/target_pose"]
        T5["/right_arm/target_pose"]
    end

    subgraph IK["IKソルバー"]
        IKL["left_arm_ik_solver<br/>(KDL)"]
        IKR["right_arm_ik_solver<br/>(KDL)"]
    end

    subgraph Topics3["ROS2 Topics (関節角度)"]
        T6["/left_arm/ik/joint_angles"]
        T7["/right_arm/ik/joint_angles"]
    end

    subgraph Bridge["SO101 Bridge"]
        BL["leader_ros2_node<br/>(左アーム)"]
        BR["follower_ros2_node<br/>(右アーム)"]
    end

    subgraph Robot["SO101 デュアルアーム"]
        LA["🦾 左アーム"]
        RA["🦾 右アーム"]
    end

    LeftHand --> T1
    RightHand --> T2
    Gesture --> T3

    T1 --> VRC
    T2 --> VRC
    T3 --> VRC

    VRC --> T4
    VRC --> T5

    T4 --> IKL
    T5 --> IKR

    IKL --> T6
    IKR --> T7

    T6 --> BL
    T7 --> BR

    BL --> LA
    BR --> RA
```

---

## 4. 学習パイプライン詳細

```mermaid
flowchart TB
    subgraph Collection["データ収集"]
        Teleop["VRテレオペ"]
        Record["ros2 bag record"]
        Bags[("ROS2 Bags<br/>.db3")]

        Teleop --> Record
        Record --> Bags
    end

    subgraph Conversion["データ変換"]
        Script["rosbag_to_lerobot.py"]

        subgraph LeRobotDS["LeRobot Dataset"]
            Meta["meta/<br/>・info.json<br/>・stats.json<br/>・episodes.json"]
            Data["data/<br/>・episode_*.parquet"]
            Videos["videos/<br/>・episode_*.mp4"]
        end

        Bags --> Script
        Script --> Meta
        Script --> Data
        Script --> Videos
    end

    subgraph Training["学習"]
        TrainScript["lerobot/scripts/train.py"]

        subgraph Config["設定"]
            PolicyCfg["policy: smolvla"]
            DataCfg["dataset: local"]
            TrainCfg["batch_size: 32<br/>epochs: 100"]
        end

        Checkpoint[("checkpoints/<br/>・last/<br/>・best/")]

        Meta --> TrainScript
        Data --> TrainScript
        Videos --> TrainScript
        Config --> TrainScript
        TrainScript --> Checkpoint
    end

    subgraph Inference["推論"]
        Runner["policy_runner_ros2_node"]

        Checkpoint --> Runner
    end
```

---

## 5. シミュレーション連携

```mermaid
flowchart LR
    subgraph Real["実機環境"]
        RealRobot["🦾 SO101実機"]
        RealCam["📷 実カメラ"]
    end

    subgraph ROS["ROS2"]
        Topics["ROS2 Topics<br/>/left_arm/*<br/>/right_arm/*<br/>/cam_*/image_raw"]
    end

    subgraph Sim["シミュレーション環境"]
        subgraph MJ["MuJoCo"]
            MJRobot["仮想SO101"]
            MJCam["仮想カメラ"]
            MJPhys["物理演算"]
        end

        subgraph IS["Isaac Sim"]
            ISRobot["仮想SO101"]
            ISCam["仮想カメラ"]
            ISDR["ドメイン<br/>ランダム化"]
        end
    end

    subgraph Controller["制御"]
        VRCtrl["VRテレオペ"]
        PolicyCtrl["Policy推論"]
    end

    RealRobot <--> Topics
    RealCam --> Topics

    Topics <-->|"remap"| MJRobot
    MJCam --> Topics

    Topics <-->|"remap"| ISRobot
    ISCam --> Topics

    VRCtrl --> Topics
    PolicyCtrl <--> Topics
```

---

## 6. エピソード記録シーケンス

```mermaid
sequenceDiagram
    participant User as 👤 ユーザー
    participant VR as 🥽 Quest 3
    participant Rec as EpisodeRecorder
    participant ROS as ROS2 Topics
    participant Robot as 🦾 ロボット
    participant Cam as 📷 カメラ
    participant Bag as ROS2 Bag

    User->>Rec: /episode/start (サービス呼び出し)
    activate Rec
    Rec->>Bag: ros2 bag record 開始

    loop テレオペレーション中
        VR->>ROS: 手のポーズ
        ROS->>Robot: 関節コマンド
        Robot->>ROS: 関節状態
        Cam->>ROS: 画像
        ROS->>Bag: 記録
    end

    User->>Rec: /episode/stop (サービス呼び出し)
    Rec->>Bag: ros2 bag record 停止
    Rec->>Rec: metadata.json 保存
    deactivate Rec

    Note over Bag: episode_000000/<br/>├── metadata.db<br/>└── *.db3
```

---

## 7. 推論シーケンス

```mermaid
sequenceDiagram
    participant Cam as 📷 カメラ
    participant State as 🦾 関節状態
    participant Sync as TimeSynchronizer
    participant Policy as SmolVLA
    participant Buffer as ActionBuffer
    participant Pub as Publisher
    participant Robot as 🦾 ロボット

    loop 30Hz (カメラ)
        Cam->>Sync: image_raw
    end

    loop 50Hz (関節)
        State->>Sync: joint_states
    end

    Sync->>Sync: 時刻同期

    loop 1Hz (推論)
        Sync->>Policy: 同期済み観測
        Policy->>Policy: GPU推論
        Policy->>Buffer: action_chunk[50]
    end

    loop 20Hz (発行)
        Buffer->>Pub: next_action
        Pub->>Robot: joint_commands
    end
```

---

## 8. コンポーネント一覧

| カテゴリ | コンポーネント | 説明 |
|---------|---------------|------|
| **入力** | Quest 3 | VRヘッドセット、手のポーズ・ジェスチャー |
| **入力** | リーダーアーム | 従来方式のテレオペ入力（オプション） |
| **ROS2ノード** | vr_dual_arm_control_node | VR入力→左右アームマッピング |
| **ROS2ノード** | left_arm_ik_solver_node | 左アームIK計算（KDL） |
| **ROS2ノード** | right_arm_ik_solver_node | 右アームIK計算（KDL） |
| **ROS2ノード** | episode_recorder_node | エピソード記録管理 |
| **ROS2ノード** | policy_runner_node | 学習済みモデル推論 |
| **ブリッジ** | leader_ros2_node | 左アーム制御（LeRobot API） |
| **ブリッジ** | follower_ros2_node | 右アーム制御（LeRobot API） |
| **変換** | rosbag_to_lerobot.py | ROS2 Bag→LeRobot Dataset |
| **出力** | SO101 左アーム | Feetech STS3215サーボ |
| **出力** | SO101 右アーム | Feetech STS3215サーボ |
| **出力** | カメラ | USB cam / RealSense |
| **シミュレータ** | MuJoCo | 物理シミュレーション |
| **シミュレータ** | Isaac Sim | NVIDIA GPU シミュレーション |
| **学習** | SmolVLA | Vision-Language-Action モデル |
| **学習** | ACT | Action Chunking Transformer |

---

## 9. トピック一覧

### VR入力トピック

| トピック | 型 | 説明 |
|---------|-----|------|
| `/quest/left_hand/pose` | PoseStamped | VR左手ポーズ |
| `/quest/right_hand/pose` | PoseStamped | VR右手ポーズ |
| `/quest/left_hand/joints` | PointCloud | 左手21関節座標 |
| `/quest/right_hand/joints` | PointCloud | 右手21関節座標 |
| `/quest/hand_gesture` | HandGesture | ジェスチャー |

### 内部トピック

| トピック | 型 | 説明 |
|---------|-----|------|
| `/left_arm/target_pose` | PoseStamped | 左アーム目標ポーズ |
| `/right_arm/target_pose` | PoseStamped | 右アーム目標ポーズ |
| `/left_arm/ik/joint_angles` | JointState | 左アームIK結果 |
| `/right_arm/ik/joint_angles` | JointState | 右アームIK結果 |
| `/left_arm/joint_states` | JointState | 左アーム現在状態 |
| `/right_arm/joint_states` | JointState | 右アーム現在状態 |
| `/left_arm/joint_commands` | JointState | 左アームコマンド |
| `/right_arm/joint_commands` | JointState | 右アームコマンド |

### カメラトピック

| トピック | 型 | 説明 |
|---------|-----|------|
| `/follower/cam_front/image_raw` | Image | 前面カメラ画像 |
| `/static_camera/cam_side/color/image_raw` | Image | サイドカメラ画像 |

---

## 10. 使用方法

### データ収集

```bash
# 1. VRテレオペ + ロボット起動
ros2 launch unity_robot_control vr_dual_arm_teleop.launch.py

# 2. エピソード記録ノード起動
ros2 run unity_robot_control episode_recorder_node

# 3. 記録開始/停止
ros2 service call /episode/start std_srvs/srv/Trigger
# ... テレオペ操作 ...
ros2 service call /episode/stop std_srvs/srv/Trigger
```

### データ変換

```bash
# ROS2 Bag → LeRobot Dataset
python3 rosbag_to_lerobot.py \
    --input-dir ~/ros2_bags \
    --output-dir ~/lerobot_dataset \
    --fps 30
```

### 学習

```bash
# LeRobotで学習
python lerobot/scripts/train.py \
    policy=smolvla \
    dataset.root=~/lerobot_dataset \
    training.batch_size=32 \
    training.num_epochs=100
```

### 推論

```bash
# 学習済みモデルで推論
ros2 run so101_ros2_bridge policy_runner_ros2_node --ros-args \
    -p checkpoint_path:=~/outputs/checkpoints/last \
    -p device:=cuda:0
```

---

## 11. マルチロボット対応アーキテクチャ

複数のロボットアームに対応するため、抽象化レイヤーを設けています。

### 対応ロボット

| ロボット | メーカー | 関節数 | 通信方式 |
|---------|---------|--------|---------|
| SO101 | TheRobotStudio | 6 | USB Serial (Feetech) |
| Piper | AgileX | 6 | CAN |
| Daihen OTC | ダイヘン | 6 | TCP/IP |
| Koch | - | 6 | USB Serial (Dynamixel) |

### 抽象化レイヤー構成

```mermaid
flowchart TB
    subgraph App["アプリケーション層（共通）"]
        VR["VRテレオペ"]
        Policy["Policy推論"]
        Recorder["データ収集"]
    end

    subgraph Abstract["抽象化レイヤー（共通インターフェース）"]
        Topics["標準トピック<br/>/left_arm/joint_states<br/>/left_arm/joint_commands"]
        Config["ロボット設定<br/>robot_config.yaml"]
    end

    subgraph Drivers["ドライバー層（機種別）"]
        SO101D["SO101 Driver<br/>(Feetech)"]
        PiperD["Piper Driver<br/>(CAN)"]
        DaihenD["Daihen Driver<br/>(TCP)"]
    end

    subgraph Robots["実機"]
        R1["🦾 SO101"]
        R2["🦾 Piper"]
        R3["🦾 Daihen"]
    end

    App --> Topics
    Topics --> Config
    Config --> SO101D
    Config --> PiperD
    Config --> DaihenD

    SO101D --> R1
    PiperD --> R2
    DaihenD --> R3
```

### 関節名の標準化

全ロボットで共通の関節インデックスを使用し、設定ファイルで実機名にマッピングします。

```mermaid
flowchart LR
    subgraph Standard["標準関節名"]
        J0["joint_0"]
        J1["joint_1"]
        J2["joint_2"]
        J3["joint_3"]
        J4["joint_4"]
        J5["joint_5"]
    end

    subgraph SO101["SO101"]
        S0["shoulder_pan"]
        S1["shoulder_lift"]
        S2["elbow_flex"]
        S3["wrist_flex"]
        S4["wrist_roll"]
        S5["gripper"]
    end

    subgraph Piper["Piper"]
        P0["joint1"]
        P1["joint2"]
        P2["joint3"]
        P3["joint4"]
        P4["joint5"]
        P5["joint6"]
    end

    J0 --> S0
    J1 --> S1
    J2 --> S2
    J3 --> S3
    J4 --> S4
    J5 --> S5

    J0 --> P0
    J1 --> P1
    J2 --> P2
    J3 --> P3
    J4 --> P4
    J5 --> P5
```

### ロボット設定ファイル例

```yaml
# config/robots/so101.yaml
robot:
  name: so101
  type: so101_follower

  joints:
    count: 6
    mapping:
      joint_0: shoulder_pan
      joint_1: shoulder_lift
      joint_2: elbow_flex
      joint_3: wrist_flex
      joint_4: wrist_roll
      joint_5: gripper

  limits:
    joint_0: { min: -3.14, max: 3.14 }
    joint_1: { min: -1.57, max: 1.57 }
    joint_2: { min: -1.57, max: 1.57 }
    joint_3: { min: -1.57, max: 1.57 }
    joint_4: { min: -3.14, max: 3.14 }
    joint_5: { min: 0.0, max: 0.04 }

  communication:
    type: serial
    port: /dev/ttyACM0
    protocol: feetech

  urdf:
    package: so101_description
    file: urdf/so101.urdf.xacro
```

### ロボット切り替え方法

```bash
# SO101で起動
ros2 launch unity_robot_control vr_teleop.launch.py robot:=so101

# Piperで起動
ros2 launch unity_robot_control vr_teleop.launch.py robot:=piper

# Daihenで起動
ros2 launch unity_robot_control vr_teleop.launch.py robot:=daihen
```

---

## 12. 推論時のROS2連携詳細

学習済みモデルの推論はROS2ノードとして動作し、全ての入出力はROS2トピック経由です。

### 推論ノードの内部構造

```mermaid
flowchart TB
    subgraph Input["ROS2 Subscribers"]
        CamSub["📷 /cam_front/image_raw"]
        StateSub["🦾 /left_arm/joint_states<br/>/right_arm/joint_states"]
    end

    subgraph PolicyRunner["policy_runner_ros2_node"]
        Sync["TimeSynchronizer<br/>時刻同期"]
        Preprocess["前処理<br/>・画像リサイズ<br/>・正規化"]
        Infer["SmolVLA推論<br/>(GPU)"]
        Buffer["ActionBuffer<br/>50ステップ分"]
        Postprocess["後処理<br/>・関節名マッピング"]
    end

    subgraph Output["ROS2 Publishers"]
        CmdPub["🎯 /left_arm/joint_commands<br/>/right_arm/joint_commands"]
    end

    subgraph Bridge["ロボットドライバー"]
        Driver["generic_robot_driver<br/>または<br/>so101_ros2_bridge"]
    end

    subgraph Robot["実機"]
        Arm["🦾 ロボットアーム"]
    end

    CamSub --> Sync
    StateSub --> Sync
    Sync --> Preprocess
    Preprocess --> Infer
    Infer --> Buffer
    Buffer --> Postprocess
    Postprocess --> CmdPub
    CmdPub --> Driver
    Driver --> Arm
```

### 推論パラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `inference_rate` | 1.0 Hz | 推論実行頻度（重い処理） |
| `publish_rate` | 20.0 Hz | コマンド発行頻度 |
| `inference_delay` | 0.4 s | 遅延補償 |
| `chunk_size` | 50 | 予測ステップ数 |
| `device` | cuda:0 | 推論デバイス |

### io.yaml 設定

```yaml
# config/policies/io.yaml
observations:
  observation.images.camera1:
    topic: "/cam_front/image_raw"
    msg_type: "sensor_msgs/msg/Image"

  observation.state:
    topic: "/left_arm/joint_states"
    msg_type: "sensor_msgs/msg/JointState"

action:
  topic: "/left_arm/joint_commands"
  msg_type: "sensor_msgs/msg/JointState"
```

---

## 13. データセットの標準化

異なるロボットで収集したデータを統合して学習できるよう、データセット形式を標準化しています。

### LeRobotデータセット形式

```mermaid
flowchart LR
    subgraph Collect["データ収集"]
        R1["SO101で収集"]
        R2["Piperで収集"]
    end

    subgraph Standardize["標準化"]
        D1["Dataset A<br/>joint_0..5"]
        D2["Dataset B<br/>joint_0..5"]
    end

    subgraph Train["学習"]
        Merge["統合Dataset"]
        Model["学習済みモデル"]
    end

    subgraph Deploy["推論"]
        M1["SO101で推論"]
        M2["Piperで推論"]
    end

    R1 -->|"標準関節名<br/>に変換"| D1
    R2 -->|"標準関節名<br/>に変換"| D2
    D1 --> Merge
    D2 --> Merge
    Merge --> Model
    Model -->|"実機関節名<br/>にマッピング"| M1
    Model -->|"実機関節名<br/>にマッピング"| M2
```

### 標準データ形式

```python
# 全ロボット共通
observation.state = [
    joint_0,  # ベース回転
    joint_1,  # 肩
    joint_2,  # 肘
    joint_3,  # 手首1
    joint_4,  # 手首2
    joint_5,  # グリッパー
]

# デュアルアームの場合
observation.state = [
    left_joint_0, left_joint_1, ..., left_joint_5,
    right_joint_0, right_joint_1, ..., right_joint_5,
]  # 計12要素
```

---

## 14. ライセンス

MIT License
