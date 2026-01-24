#!/usr/bin/env python3
"""
ROS2 Bag → LeRobot Dataset変換スクリプト

VRテレオペレーションで収集したROS2 BagファイルをLeRobot形式のデータセットに変換する。

使用方法:
    ros2 run unity_robot_control rosbag_to_lerobot --bag-path /path/to/bag --output-dir /path/to/output

LeRobot データセット形式:
    dataset/
    ├── meta/
    │   ├── info.json
    │   ├── stats.json
    │   └── episodes.json
    ├── data/
    │   └── chunk-000/
    │       └── episode_000000.parquet
    └── videos/
        └── observation.images.cam_front/
            └── episode_000000.mp4
"""
import argparse
import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from datetime import datetime

import numpy as np

# ROS2関連のインポート
try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState, Image
    from geometry_msgs.msg import PoseStamped
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print("Warning: ROS2 packages not found. Running in mock mode.")

# データ処理
try:
    import pandas as pd
    import pyarrow as pa
    import pyarrow.parquet as pq
    HAS_PARQUET = True
except ImportError:
    HAS_PARQUET = False

# 画像処理
try:
    import cv2
    from cv_bridge import CvBridge
    HAS_CV = True
except ImportError:
    HAS_CV = False


class RosbagToLerobotConverter:
    """ROS2 BagをLeRobotデータセット形式に変換するクラス"""

    # SO101のジョイント名（6関節）
    JOINT_NAMES = [
        'shoulder_pan',
        'shoulder_lift',
        'elbow_flex',
        'wrist_flex',
        'wrist_roll',
        'gripper'
    ]

    # トピック設定
    DEFAULT_TOPICS = {
        'left_arm_joints': '/left_arm/joint_states',
        'right_arm_joints': '/right_arm/joint_states',
        'left_arm_ik': '/left_arm/ik/joint_angles',
        'right_arm_ik': '/right_arm/ik/joint_angles',
        'left_hand_pose': '/quest/left_hand/pose',
        'right_hand_pose': '/quest/right_hand/pose',
        'cam_front': '/cam_front/image_raw',
    }

    def __init__(
        self,
        bag_path: str,
        output_dir: str,
        task_name: str = "vr_teleop",
        fps: int = 50,
        image_size: Tuple[int, int] = (480, 640),
        dual_arm: bool = True,
    ):
        """
        Args:
            bag_path: ROS2 Bagファイルのパス
            output_dir: 出力ディレクトリ
            task_name: タスク名
            fps: データのフレームレート
            image_size: 画像サイズ (H, W)
            dual_arm: デュアルアームモード
        """
        self.bag_path = Path(bag_path)
        self.output_dir = Path(output_dir)
        self.task_name = task_name
        self.fps = fps
        self.image_size = image_size
        self.dual_arm = dual_arm

        # データストレージ
        self.episodes: List[Dict] = []
        self.current_episode: Dict = {
            'timestamps': [],
            'left_arm_positions': [],
            'right_arm_positions': [],
            'left_hand_poses': [],
            'right_hand_poses': [],
            'images': [],
        }

        # CV Bridge
        if HAS_CV:
            self.cv_bridge = CvBridge()

    def convert(self) -> bool:
        """変換を実行"""
        print(f"Converting ROS2 Bag: {self.bag_path}")
        print(f"Output directory: {self.output_dir}")

        # 出力ディレクトリ作成
        self._create_output_dirs()

        # Bagファイル読み込み
        if not self._read_bag():
            return False

        # エピソードに分割
        self._split_episodes()

        # Parquetファイル生成
        self._write_parquet_files()

        # メタデータ生成
        self._write_metadata()

        print(f"Conversion complete. {len(self.episodes)} episodes created.")
        return True

    def _create_output_dirs(self):
        """出力ディレクトリ構造を作成"""
        dirs = [
            self.output_dir / "meta",
            self.output_dir / "data" / "chunk-000",
            self.output_dir / "videos" / "observation.images.cam_front",
        ]
        for d in dirs:
            d.mkdir(parents=True, exist_ok=True)

    def _read_bag(self) -> bool:
        """ROS2 Bagファイルを読み込む"""
        if not HAS_ROS2:
            print("Error: ROS2 packages not available")
            return False

        if not self.bag_path.exists():
            print(f"Error: Bag file not found: {self.bag_path}")
            return False

        try:
            # Bagリーダー設定
            storage_options = StorageOptions(
                uri=str(self.bag_path),
                storage_id='sqlite3'  # or 'mcap'
            )
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )

            reader = SequentialReader()
            reader.open(storage_options, converter_options)

            # トピック情報取得
            topic_types = reader.get_all_topics_and_types()
            type_map = {t.name: t.type for t in topic_types}

            print(f"Available topics: {list(type_map.keys())}")

            # メッセージ読み込み
            while reader.has_next():
                topic, data, timestamp = reader.read_next()
                self._process_message(topic, data, timestamp, type_map.get(topic))

            return True

        except Exception as e:
            print(f"Error reading bag: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _process_message(self, topic: str, data: bytes, timestamp: int, msg_type: str):
        """メッセージを処理"""
        if msg_type is None:
            return

        try:
            if 'JointState' in msg_type:
                msg = deserialize_message(data, JointState)
                self._process_joint_state(topic, msg, timestamp)
            elif 'PoseStamped' in msg_type:
                msg = deserialize_message(data, PoseStamped)
                self._process_pose(topic, msg, timestamp)
            elif 'Image' in msg_type and HAS_CV:
                msg = deserialize_message(data, Image)
                self._process_image(topic, msg, timestamp)

        except Exception as e:
            print(f"Warning: Failed to process message on {topic}: {e}")

    def _process_joint_state(self, topic: str, msg: 'JointState', timestamp: int):
        """JointStateメッセージを処理"""
        positions = list(msg.position)

        # 6関節に正規化
        while len(positions) < 6:
            positions.append(0.0)
        positions = positions[:6]

        if 'left' in topic:
            self.current_episode['left_arm_positions'].append(positions)
        elif 'right' in topic:
            self.current_episode['right_arm_positions'].append(positions)

        self.current_episode['timestamps'].append(timestamp)

    def _process_pose(self, topic: str, msg: 'PoseStamped', timestamp: int):
        """PoseStampedメッセージを処理"""
        pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]

        if 'left' in topic:
            self.current_episode['left_hand_poses'].append(pose)
        elif 'right' in topic:
            self.current_episode['right_hand_poses'].append(pose)

    def _process_image(self, topic: str, msg: 'Image', timestamp: int):
        """Imageメッセージを処理"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            # リサイズ
            cv_image = cv2.resize(cv_image, (self.image_size[1], self.image_size[0]))
            self.current_episode['images'].append(cv_image)
        except Exception as e:
            print(f"Warning: Image conversion failed: {e}")

    def _split_episodes(self):
        """データをエピソードに分割（現在は単一エピソード）"""
        # TODO: タスク開始/終了マーカーに基づいてエピソード分割
        if self.current_episode['timestamps']:
            self.episodes.append(self.current_episode)

    def _write_parquet_files(self):
        """Parquetファイルを書き込む"""
        if not HAS_PARQUET:
            print("Warning: pyarrow not available, skipping parquet generation")
            return

        for ep_idx, episode in enumerate(self.episodes):
            # データフレーム作成
            n_frames = len(episode['timestamps'])
            if n_frames == 0:
                continue

            # 観測データ（状態）
            left_positions = np.array(episode['left_arm_positions'][:n_frames])
            right_positions = np.array(episode['right_arm_positions'][:n_frames])

            # デュアルアームの場合は結合
            if self.dual_arm and len(right_positions) > 0:
                state = np.concatenate([left_positions, right_positions], axis=1)
            else:
                state = left_positions

            # アクション（次フレームの状態を目標値として使用）
            action = np.roll(state, -1, axis=0)
            action[-1] = state[-1]  # 最終フレームは同じ値

            # データフレーム作成
            data = {
                'timestamp': episode['timestamps'][:n_frames],
                'observation.state': state.tolist(),
                'action': action.tolist(),
                'episode_index': [ep_idx] * n_frames,
                'frame_index': list(range(n_frames)),
            }

            df = pd.DataFrame(data)

            # Parquet書き込み
            output_path = self.output_dir / "data" / "chunk-000" / f"episode_{ep_idx:06d}.parquet"
            table = pa.Table.from_pandas(df)
            pq.write_table(table, output_path)

            print(f"Written episode {ep_idx}: {n_frames} frames -> {output_path}")

    def _write_metadata(self):
        """メタデータファイルを書き込む"""
        # info.json
        info = {
            "codebase_version": "v2.0",
            "robot_type": "so101_dual_arm",
            "fps": self.fps,
            "features": {
                "observation.state": {
                    "dtype": "float32",
                    "shape": [12] if self.dual_arm else [6],
                    "names": self.JOINT_NAMES * 2 if self.dual_arm else self.JOINT_NAMES
                },
                "action": {
                    "dtype": "float32",
                    "shape": [12] if self.dual_arm else [6],
                    "names": self.JOINT_NAMES * 2 if self.dual_arm else self.JOINT_NAMES
                }
            },
            "total_episodes": len(self.episodes),
            "total_frames": sum(len(ep['timestamps']) for ep in self.episodes),
            "task": self.task_name,
            "created_at": datetime.now().isoformat(),
        }

        with open(self.output_dir / "meta" / "info.json", 'w') as f:
            json.dump(info, f, indent=2)

        # episodes.json
        episodes_meta = []
        for ep_idx, episode in enumerate(self.episodes):
            episodes_meta.append({
                "episode_index": ep_idx,
                "length": len(episode['timestamps']),
                "task": self.task_name,
            })

        with open(self.output_dir / "meta" / "episodes.json", 'w') as f:
            json.dump(episodes_meta, f, indent=2)

        # stats.json (統計情報)
        # TODO: 実際の統計を計算
        stats = {
            "observation.state": {
                "mean": [0.0] * (12 if self.dual_arm else 6),
                "std": [1.0] * (12 if self.dual_arm else 6),
                "min": [-3.14] * (12 if self.dual_arm else 6),
                "max": [3.14] * (12 if self.dual_arm else 6),
            },
            "action": {
                "mean": [0.0] * (12 if self.dual_arm else 6),
                "std": [1.0] * (12 if self.dual_arm else 6),
                "min": [-3.14] * (12 if self.dual_arm else 6),
                "max": [3.14] * (12 if self.dual_arm else 6),
            }
        }

        with open(self.output_dir / "meta" / "stats.json", 'w') as f:
            json.dump(stats, f, indent=2)

        print(f"Metadata written to {self.output_dir / 'meta'}")


def main():
    """メインエントリポイント"""
    parser = argparse.ArgumentParser(
        description='Convert ROS2 Bag to LeRobot Dataset format'
    )
    parser.add_argument(
        '--bag-path', '-b',
        type=str,
        required=True,
        help='Path to ROS2 bag file'
    )
    parser.add_argument(
        '--output-dir', '-o',
        type=str,
        required=True,
        help='Output directory for LeRobot dataset'
    )
    parser.add_argument(
        '--task', '-t',
        type=str,
        default='vr_teleop',
        help='Task name (default: vr_teleop)'
    )
    parser.add_argument(
        '--fps',
        type=int,
        default=50,
        help='Frame rate (default: 50)'
    )
    parser.add_argument(
        '--single-arm',
        action='store_true',
        help='Single arm mode (default: dual arm)'
    )

    args = parser.parse_args()

    converter = RosbagToLerobotConverter(
        bag_path=args.bag_path,
        output_dir=args.output_dir,
        task_name=args.task,
        fps=args.fps,
        dual_arm=not args.single_arm,
    )

    success = converter.convert()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
