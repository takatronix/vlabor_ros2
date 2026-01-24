"""
Frame Buffer - Synchronize and buffer data from multiple topics
"""
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np


@dataclass
class TopicStatus:
    """Status of a subscribed topic."""
    topic: str
    receiving: bool = False
    last_received: float = 0.0
    hz: float = 0.0
    _timestamps: list = field(default_factory=list)

    def update(self, timestamp: float):
        """Update status with new message."""
        self.receiving = True
        self.last_received = timestamp
        self._timestamps.append(timestamp)
        # Keep last 10 timestamps for rate calculation
        if len(self._timestamps) > 10:
            self._timestamps.pop(0)
        if len(self._timestamps) >= 2:
            dt = self._timestamps[-1] - self._timestamps[0]
            if dt > 0:
                self.hz = (len(self._timestamps) - 1) / dt

    def to_dict(self, now: float) -> dict:
        return {
            "topic": self.topic,
            "receiving": self.receiving and (now - self.last_received < 1.0),
            "hz": round(self.hz, 1),
            "last_received_sec_ago": round(now - self.last_received, 2) if self.last_received > 0 else None,
        }


class FrameBuffer:
    """
    Buffer and synchronize data from multiple ROS2 topics.

    Uses fixed-rate sampling: at each tick, capture the latest values
    from all topics to form a synchronized frame.
    """

    def __init__(self, fps: int = 30, image_writer_threads: int = 4):
        """
        Initialize frame buffer.

        Args:
            fps: Target frame rate
            image_writer_threads: Number of threads for async image writing
        """
        self.fps = fps
        self.frame_interval = 1.0 / fps

        # Latest values from each topic
        self._lock = threading.Lock()
        self._latest_state: dict[str, list[float]] = {}  # namespace -> positions
        self._latest_action: dict[str, list[float]] = {}  # namespace -> positions
        self._latest_images: dict[str, np.ndarray] = {}   # camera_name -> image

        # Topic status tracking
        self._topic_status: dict[str, TopicStatus] = {}

        # Frame storage
        self._frames: list[dict] = []
        self._frame_index = 0

        # Image writer
        self._image_executor = ThreadPoolExecutor(max_workers=image_writer_threads)
        self._image_output_dir: Optional[Path] = None

        # Recording state
        self._recording = False
        self._start_time: Optional[float] = None

    def start_recording(self, image_output_base: Optional[Path] = None):
        """Start recording frames."""
        with self._lock:
            self._frames = []
            self._frame_index = 0
            self._recording = True
            self._start_time = time.time()
            self._image_output_dir = image_output_base

    def stop_recording(self) -> list[dict]:
        """Stop recording and return collected frames."""
        with self._lock:
            self._recording = False
            frames = self._frames.copy()
            self._frames = []
            return frames

    def pause_recording(self):
        """Pause recording."""
        with self._lock:
            self._recording = False

    def resume_recording(self):
        """Resume recording."""
        with self._lock:
            self._recording = True

    def is_recording(self) -> bool:
        """Check if recording is active."""
        return self._recording

    def get_frame_count(self) -> int:
        """Get current frame count."""
        with self._lock:
            return len(self._frames)

    def get_duration(self) -> float:
        """Get recording duration in seconds."""
        if self._start_time is None:
            return 0.0
        return time.time() - self._start_time

    # === Data update methods (called from ROS2 callbacks) ===

    def update_state(self, namespace: str, positions: list[float], topic: str):
        """Update state data for an arm."""
        now = time.time()
        with self._lock:
            self._latest_state[namespace] = list(positions)
            if topic not in self._topic_status:
                self._topic_status[topic] = TopicStatus(topic)
            self._topic_status[topic].update(now)

    def update_action(self, namespace: str, positions: list[float], topic: str):
        """Update action data for an arm."""
        now = time.time()
        with self._lock:
            self._latest_action[namespace] = list(positions)
            if topic not in self._topic_status:
                self._topic_status[topic] = TopicStatus(topic)
            self._topic_status[topic].update(now)

    def update_image(self, camera_name: str, image: np.ndarray, topic: str):
        """Update image data for a camera."""
        now = time.time()
        with self._lock:
            self._latest_images[camera_name] = image.copy()
            if topic not in self._topic_status:
                self._topic_status[topic] = TopicStatus(topic)
            self._topic_status[topic].update(now)

    # === Frame sampling ===

    def sample_frame(self, arm_namespaces: list[str], camera_names: list[str]) -> Optional[dict]:
        """
        Sample current data to create a frame.

        Called by timer at fixed rate (e.g., 30fps).
        """
        if not self._recording:
            return None

        now = time.time()
        timestamp = now - self._start_time if self._start_time else 0.0

        with self._lock:
            # Build observation.state (concatenate all arms)
            state = []
            for ns in arm_namespaces:
                positions = self._latest_state.get(ns, [])
                state.extend(positions)

            # Build action (concatenate all arms)
            action = []
            for ns in arm_namespaces:
                positions = self._latest_action.get(ns, [])
                action.extend(positions)

            # Skip frame if no data
            if not state and not action:
                return None

            frame = {
                "frame_index": self._frame_index,
                "timestamp": timestamp,
                "observation.state": state,
                "action": action,
            }

            # Save images asynchronously
            for cam_name in camera_names:
                image = self._latest_images.get(cam_name)
                if image is not None and self._image_output_dir:
                    self._save_image_async(cam_name, image, self._frame_index)

            self._frames.append(frame)
            self._frame_index += 1

        return frame

    def _save_image_async(self, camera_name: str, image: np.ndarray, frame_index: int):
        """Save image asynchronously."""
        if self._image_output_dir is None:
            return

        output_dir = self._image_output_dir / camera_name
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / f"frame_{frame_index:06d}.png"

        # Submit to thread pool
        self._image_executor.submit(self._write_image, image.copy(), output_path)

    @staticmethod
    def _write_image(image: np.ndarray, path: Path):
        """Write image to disk."""
        try:
            # Convert RGB to BGR for OpenCV
            if len(image.shape) == 3 and image.shape[2] == 3:
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(path), image)
        except Exception as e:
            print(f"Failed to write image: {e}")

    # === Status ===

    def get_topic_status(self) -> dict[str, dict]:
        """Get status of all topics."""
        now = time.time()
        with self._lock:
            return {
                key: status.to_dict(now)
                for key, status in self._topic_status.items()
            }

    def clear(self):
        """Clear all buffers."""
        with self._lock:
            self._frames = []
            self._frame_index = 0
            self._latest_state.clear()
            self._latest_action.clear()
            self._latest_images.clear()

    def shutdown(self):
        """Shutdown the frame buffer."""
        self._image_executor.shutdown(wait=True)
