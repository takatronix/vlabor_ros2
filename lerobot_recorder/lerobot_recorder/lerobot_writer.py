"""
LeRobot Writer - Write episode data in LeRobot v3 format
"""
import json
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

CODEBASE_VERSION = "v3.0"


class LeRobotWriter:
    """Writer for LeRobot v3 format datasets."""

    def __init__(
        self,
        output_dir: Path,
        dataset_name: str,
        robot_type: str,
        fps: int,
        features: dict[str, dict],
        use_video: bool = True,
    ):
        """
        Initialize LeRobot writer.

        Args:
            output_dir: Root output directory
            dataset_name: Name of the dataset
            robot_type: Robot type string
            fps: Frame rate
            features: Feature definitions
            use_video: Whether to use video encoding
        """
        self.output_dir = Path(output_dir).expanduser()
        self.dataset_name = dataset_name
        self.robot_type = robot_type
        self.fps = fps
        self.features = features
        self.use_video = use_video

        self.dataset_dir = self.output_dir / dataset_name
        self.meta_dir = self.dataset_dir / "meta"
        self.data_dir = self.dataset_dir / "data" / "chunk-000"
        self.videos_dir = self.dataset_dir / "videos"

        self._episode_count = 0
        self._total_frames = 0
        self._tasks: dict[str, int] = {}  # task -> task_index

        # Create directories
        self._create_directories()

        # Load existing metadata if resuming
        self._load_existing_metadata()

    def _create_directories(self):
        """Create necessary directories."""
        self.meta_dir.mkdir(parents=True, exist_ok=True)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        (self.meta_dir / "episodes" / "chunk-000").mkdir(parents=True, exist_ok=True)

        # Create video directories for each image feature
        for key in self.features:
            if key.startswith("observation.images."):
                video_dir = self.videos_dir / key
                video_dir.mkdir(parents=True, exist_ok=True)

    def _load_existing_metadata(self):
        """Load existing metadata if resuming a dataset."""
        info_path = self.meta_dir / "info.json"
        if info_path.exists():
            try:
                with open(info_path, 'r') as f:
                    info = json.load(f)
                self._episode_count = info.get('total_episodes', 0)
                self._total_frames = info.get('total_frames', 0)
            except Exception:
                pass

        tasks_path = self.meta_dir / "tasks.json"
        if tasks_path.exists():
            try:
                with open(tasks_path, 'r') as f:
                    tasks_list = json.load(f)
                for item in tasks_list:
                    self._tasks[item['task']] = item['task_index']
            except Exception:
                pass

    def get_next_episode_index(self) -> int:
        """Get the next episode index."""
        return self._episode_count

    def _get_or_create_task_index(self, task: str) -> int:
        """Get or create task index for a task string."""
        if task not in self._tasks:
            self._tasks[task] = len(self._tasks)
        return self._tasks[task]

    def write_episode(
        self,
        frames: list[dict],
        task: str,
        tags: list[str] = None,
        metadata: dict = None,
    ) -> dict:
        """
        Write an episode to the dataset.

        Args:
            frames: List of frame dictionaries
            task: Task description
            tags: Optional tags for the episode
            metadata: Optional metadata

        Returns:
            Dictionary with episode info
        """
        if not frames:
            return {"success": False, "error": "No frames to write"}

        episode_index = self._episode_count
        task_index = self._get_or_create_task_index(task)
        n_frames = len(frames)

        # Build DataFrame
        data = {
            "episode_index": [episode_index] * n_frames,
            "frame_index": list(range(n_frames)),
            "task_index": [task_index] * n_frames,
            "timestamp": [f.get("timestamp", i / self.fps) for i, f in enumerate(frames)],
        }

        # Add observation.state
        if "observation.state" in self.features:
            data["observation.state"] = [f.get("observation.state", []) for f in frames]

        # Add action
        if "action" in self.features:
            data["action"] = [f.get("action", []) for f in frames]

        # Write parquet
        df = pd.DataFrame(data)
        parquet_path = self.data_dir / f"episode_{episode_index:06d}.parquet"

        # Convert lists to proper arrays for parquet
        table = pa.Table.from_pandas(df)
        pq.write_table(table, parquet_path, compression='snappy')

        # Write episode metadata
        episode_meta = {
            "episode_index": episode_index,
            "task_index": task_index,
            "length": n_frames,
            "task": task,
            "tags": tags or [],
            "metadata": metadata or {},
            "recorded_at": datetime.now().isoformat(),
        }

        episodes_parquet_path = self.meta_dir / "episodes" / "chunk-000" / f"episodes_{episode_index:06d}.parquet"
        episodes_df = pd.DataFrame([episode_meta])
        pq.write_table(pa.Table.from_pandas(episodes_df), episodes_parquet_path)

        # Update counters
        self._episode_count += 1
        self._total_frames += n_frames

        # Write updated metadata
        self._write_info()
        self._write_tasks()

        # Calculate relative paths
        rel_parquet = str(parquet_path.relative_to(self.output_dir))

        return {
            "success": True,
            "episode_index": episode_index,
            "frames": n_frames,
            "parquet": rel_parquet,
        }

    def _write_info(self):
        """Write info.json."""
        info = {
            "codebase_version": CODEBASE_VERSION,
            "robot_type": self.robot_type,
            "fps": self.fps,
            "features": self.features,
            "total_episodes": self._episode_count,
            "total_frames": self._total_frames,
        }

        with open(self.meta_dir / "info.json", 'w') as f:
            json.dump(info, f, indent=2)

    def _write_tasks(self):
        """Write tasks.json."""
        tasks_list = [
            {"task_index": idx, "task": task}
            for task, idx in sorted(self._tasks.items(), key=lambda x: x[1])
        ]

        with open(self.meta_dir / "tasks.json", 'w') as f:
            json.dump(tasks_list, f, indent=2)

    def get_video_dir(self, camera_name: str, episode_index: int) -> Path:
        """Get the directory for storing video frames (PNGs)."""
        feature_key = f"observation.images.{camera_name}"
        return self.videos_dir / feature_key / f"episode_{episode_index:06d}_frames"

    def get_video_path(self, camera_name: str, episode_index: int) -> Path:
        """Get the output video path."""
        feature_key = f"observation.images.{camera_name}"
        return self.videos_dir / feature_key / f"episode_{episode_index:06d}.mp4"

    def get_dataset_info(self) -> dict:
        """Get current dataset information."""
        return {
            "dataset_name": self.dataset_name,
            "path": str(self.dataset_dir),
            "codebase_version": CODEBASE_VERSION,
            "robot_type": self.robot_type,
            "fps": self.fps,
            "total_episodes": self._episode_count,
            "total_frames": self._total_frames,
            "features": self.features,
        }


def build_features(
    arm_configs: list[dict],
    joint_names: list[str],
    camera_configs: list[dict],
) -> dict:
    """
    Build feature definitions from configuration.

    Args:
        arm_configs: List of arm configurations
        joint_names: List of joint names per arm
        camera_configs: List of camera configurations

    Returns:
        Feature definitions dictionary
    """
    features = {}

    # Build state/action feature names
    state_names = []
    for arm in arm_configs:
        ns = arm['namespace']
        for joint in joint_names:
            state_names.append(f"{ns}_{joint}")

    total_joints = len(state_names)

    # observation.state
    features["observation.state"] = {
        "dtype": "float32",
        "shape": [total_joints],
        "names": state_names,
    }

    # action (same structure as state)
    features["action"] = {
        "dtype": "float32",
        "shape": [total_joints],
        "names": state_names,
    }

    # Cameras
    for cam in camera_configs:
        key = f"observation.images.{cam['name']}"
        features[key] = {
            "dtype": "video",
            "shape": [cam['height'], cam['width'], 3],
            "video_info": {
                "video.fps": 30,
                "video.codec": "libx264",
                "video.pix_fmt": "yuv420p",
            }
        }

    return features
