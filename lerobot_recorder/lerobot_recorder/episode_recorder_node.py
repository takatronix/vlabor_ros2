#!/usr/bin/env python3
"""
Episode Recorder Node - Record ROS2 topics to LeRobot format

Main node that:
- Subscribes to joint states and camera topics
- Provides WebAPI for recording control
- Saves episodes in LeRobot v3 format
"""
import asyncio
import json
import os
import threading
import time
from pathlib import Path
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState, Image

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False
    CvBridge = None

try:
    from aiohttp import web
    HAS_AIOHTTP = True
except ImportError:
    HAS_AIOHTTP = False
    web = None

try:
    from ament_index_python.packages import get_package_share_directory
    HAS_AMENT = True
except ImportError:
    HAS_AMENT = False

import yaml

from .frame_buffer import FrameBuffer
from .lerobot_writer import LeRobotWriter, build_features
from .video_encoder import VideoEncodingManager, encode_video


class EpisodeRecorderNode(Node):
    """
    Episode Recorder ROS2 Node with WebAPI.

    Records joint states and camera images to LeRobot format.
    Controlled via HTTP REST API.
    """

    def __init__(self):
        super().__init__('episode_recorder_node')
        self.get_logger().info('Episode Recorder Node starting...')

        # Declare parameters
        self._declare_parameters()

        # Load parameters
        self._load_parameters()

        # Load robot configuration
        self._load_robot_config()

        # Build features
        self._features = build_features(
            self._arm_configs,
            self._joint_names,
            self._camera_configs,
        )

        # Initialize components
        self._cv_bridge = CvBridge() if HAS_CV_BRIDGE else None
        self._frame_buffer = FrameBuffer(fps=self._fps)
        self._writer: Optional[LeRobotWriter] = None
        self._video_manager = VideoEncodingManager(fps=self._fps)

        # Recording state
        self._state = "idle"  # idle, recording, paused
        self._current_task: Optional[str] = None
        self._current_tags: list[str] = []
        self._current_metadata: dict = {}
        self._current_episode_index: int = 0

        # Namespaces for iteration
        self._arm_namespaces = [arm['namespace'] for arm in self._arm_configs]
        self._camera_names = [cam['name'] for cam in self._camera_configs]

        # Setup ROS2 subscribers
        self._setup_subscribers()

        # Setup timer for frame sampling
        self._sample_timer = self.create_timer(
            1.0 / self._fps,
            self._sample_frame_callback
        )

        # WebAPI
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._web_thread: Optional[threading.Thread] = None
        self._start_web_server()

        self.get_logger().info(f'Episode Recorder ready')
        self.get_logger().info(f'  Output: {self._output_dir}')
        self.get_logger().info(f'  Arms: {self._arm_namespaces}')
        self.get_logger().info(f'  Cameras: {self._camera_names}')
        self.get_logger().info(f'  WebAPI: http://0.0.0.0:{self._web_port}/api')

    def _declare_parameters(self):
        """Declare ROS2 parameters."""
        self.declare_parameter('output_dir', '~/lerobot_datasets')
        self.declare_parameter('default_dataset_name', 'teleop_dataset')
        self.declare_parameter('fps', 30)
        self.declare_parameter('use_video', True)
        self.declare_parameter('robot_config', 'robots/so101.yaml')
        self.declare_parameter('web_port', 8082)

        # Declare flat-format parameters for arms and cameras
        # These will be declared dynamically based on arm_count/camera_count
        self.declare_parameter('arm_count', 2)
        self.declare_parameter('camera_count', 1)

        # Declare default arm parameters (up to 4 arms)
        for i in range(4):
            self.declare_parameter(f'arm_{i}_namespace', '')
            self.declare_parameter(f'arm_{i}_action_source', 'ik')

        # Declare default camera parameters (up to 4 cameras)
        for i in range(4):
            self.declare_parameter(f'camera_{i}_name', '')
            self.declare_parameter(f'camera_{i}_topic', '')
            self.declare_parameter(f'camera_{i}_width', 640)
            self.declare_parameter(f'camera_{i}_height', 480)

    def _load_parameters(self):
        """Load parameters."""
        self._output_dir = Path(self.get_parameter('output_dir').value).expanduser()
        self._default_dataset_name = self.get_parameter('default_dataset_name').value
        self._fps = self.get_parameter('fps').value
        self._use_video = self.get_parameter('use_video').value
        self._robot_config_file = self.get_parameter('robot_config').value
        self._web_port = self.get_parameter('web_port').value

        # Parse flat-format arm parameters
        arm_count = self.get_parameter('arm_count').value
        self._arm_configs = []
        for i in range(arm_count):
            ns = self.get_parameter(f'arm_{i}_namespace').value
            action_source = self.get_parameter(f'arm_{i}_action_source').value
            if ns:  # Only add if namespace is set
                self._arm_configs.append({
                    'namespace': ns,
                    'action_source': action_source
                })

        # Default arm config if none specified
        if not self._arm_configs:
            self._arm_configs = [
                {'namespace': 'left_arm', 'action_source': 'ik'},
                {'namespace': 'right_arm', 'action_source': 'ik'},
            ]

        # Parse flat-format camera parameters
        camera_count = self.get_parameter('camera_count').value
        self._camera_configs = []
        for i in range(camera_count):
            name = self.get_parameter(f'camera_{i}_name').value
            topic = self.get_parameter(f'camera_{i}_topic').value
            width = self.get_parameter(f'camera_{i}_width').value
            height = self.get_parameter(f'camera_{i}_height').value
            if name and topic:  # Only add if name and topic are set
                self._camera_configs.append({
                    'name': name,
                    'topic': topic,
                    'width': width,
                    'height': height
                })

        # Default camera config if none specified
        if not self._camera_configs:
            self._camera_configs = [
                {'name': 'cam_front', 'topic': '/cam_front/image_raw', 'width': 640, 'height': 480}
            ]

    def _load_robot_config(self):
        """Load robot configuration from YAML file."""
        config_path = None

        # Try to find config file
        if HAS_AMENT:
            try:
                pkg_dir = get_package_share_directory('lerobot_recorder')
                config_path = Path(pkg_dir) / 'config' / self._robot_config_file
            except:
                pass

        if config_path is None or not config_path.exists():
            # Try relative to package source
            config_path = Path(__file__).parent.parent / 'config' / self._robot_config_file

        self._robot_type = "unknown"
        self._joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self._topic_templates = {
            'state': '/{ns}/joint_states',
            'action_ik': '/{ns}/ik/joint_angles',
            'action_leader': '/{ns}/leader/joint_states',
        }

        if config_path and config_path.exists():
            try:
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                robot = config.get('robot', {})
                self._robot_type = robot.get('name', 'unknown')
                joints = robot.get('joints', {})
                self._joint_names = joints.get('names', self._joint_names)
                self._topic_templates = robot.get('topics', self._topic_templates)
                self.get_logger().info(f'Loaded robot config: {self._robot_type}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load robot config: {e}')
        else:
            self.get_logger().warn(f'Robot config not found: {self._robot_config_file}')

    def _setup_subscribers(self):
        """Setup ROS2 subscribers for all topics."""
        for arm in self._arm_configs:
            ns = arm['namespace']
            action_source = arm.get('action_source', 'ik')

            # State topic
            state_topic = self._topic_templates['state'].replace('{ns}', ns)
            self.create_subscription(
                JointState,
                state_topic,
                lambda msg, namespace=ns, topic=state_topic: self._on_state(namespace, msg, topic),
                10
            )
            self.get_logger().info(f'  Subscribe state: {state_topic}')

            # Action topic
            if action_source == 'ik':
                action_topic = self._topic_templates['action_ik'].replace('{ns}', ns)
            elif action_source == 'leader':
                action_topic = self._topic_templates['action_leader'].replace('{ns}', ns)
            else:
                action_topic = arm.get('action_topic', self._topic_templates['action_ik'].replace('{ns}', ns))

            self.create_subscription(
                JointState,
                action_topic,
                lambda msg, namespace=ns, topic=action_topic: self._on_action(namespace, msg, topic),
                10
            )
            self.get_logger().info(f'  Subscribe action: {action_topic}')

        # Camera subscribers
        for cam in self._camera_configs:
            topic = cam['topic']
            name = cam['name']
            self.create_subscription(
                Image,
                topic,
                lambda msg, cam_name=name, cam_topic=topic: self._on_image(cam_name, msg, cam_topic),
                10
            )
            self.get_logger().info(f'  Subscribe camera: {topic}')

    def _on_state(self, namespace: str, msg: JointState, topic: str):
        """Handle state message."""
        positions = list(msg.position[:len(self._joint_names)])
        self._frame_buffer.update_state(namespace, positions, topic)

    def _on_action(self, namespace: str, msg: JointState, topic: str):
        """Handle action message."""
        positions = list(msg.position[:len(self._joint_names)])
        self._frame_buffer.update_action(namespace, positions, topic)

    def _on_image(self, camera_name: str, msg: Image, topic: str):
        """Handle image message."""
        if not HAS_CV_BRIDGE or self._cv_bridge is None:
            return
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self._frame_buffer.update_image(camera_name, cv_image, topic)
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')

    def _sample_frame_callback(self):
        """Timer callback for frame sampling."""
        if self._state != "recording":
            return
        self._frame_buffer.sample_frame(self._arm_namespaces, self._camera_names)

    # === Recording Control ===

    def start_recording(
        self,
        task: str,
        dataset_name: Optional[str] = None,
        tags: list[str] = None,
        metadata: dict = None,
    ) -> dict:
        """Start recording an episode."""
        if self._state == "recording":
            return {"success": False, "error": "Already recording", "state": self._state}

        dataset_name = dataset_name or self._default_dataset_name

        # Initialize writer if needed or dataset changed
        if self._writer is None or self._writer.dataset_name != dataset_name:
            self._writer = LeRobotWriter(
                output_dir=self._output_dir,
                dataset_name=dataset_name,
                robot_type=self._robot_type,
                fps=self._fps,
                features=self._features,
                use_video=self._use_video,
            )

        self._current_episode_index = self._writer.get_next_episode_index()
        self._current_task = task
        self._current_tags = tags or []
        self._current_metadata = metadata or {}

        # Setup image output directory
        image_output_base = None
        if self._use_video and self._camera_names:
            image_output_base = self._writer.dataset_dir / "videos" / "_tmp"

        self._frame_buffer.start_recording(image_output_base)
        self._state = "recording"

        self.get_logger().info(f'Recording started: episode {self._current_episode_index}, task="{task}"')

        return {
            "success": True,
            "dataset_name": dataset_name,
            "episode_index": self._current_episode_index,
            "message": "Recording started",
        }

    def stop_recording(
        self,
        tags_append: list[str] = None,
        metadata_append: dict = None,
    ) -> dict:
        """Stop recording and save episode."""
        if self._state not in ("recording", "paused"):
            return {"success": False, "error": "Not recording", "state": self._state}

        frames = self._frame_buffer.stop_recording()
        self._state = "idle"

        if not frames:
            return {"success": False, "error": "No frames recorded"}

        # Merge tags and metadata
        tags = self._current_tags + (tags_append or [])
        metadata = {**self._current_metadata, **(metadata_append or {})}

        # Write episode
        result = self._writer.write_episode(
            frames=frames,
            task=self._current_task,
            tags=tags,
            metadata=metadata,
        )

        # Encode videos
        video_files = {}
        if self._use_video:
            for cam_name in self._camera_names:
                image_dir = self._writer.dataset_dir / "videos" / "_tmp" / cam_name
                video_path = self._writer.get_video_path(cam_name, self._current_episode_index)
                if image_dir.exists():
                    if encode_video(image_dir, video_path, fps=self._fps):
                        video_files[cam_name] = str(video_path.relative_to(self._output_dir))
                        # Cleanup temp images
                        import shutil
                        shutil.rmtree(image_dir)

        duration = len(frames) / self._fps

        self.get_logger().info(f'Recording stopped: {len(frames)} frames, {duration:.1f}s')

        return {
            "success": True,
            "dataset_name": self._writer.dataset_name,
            "episode_index": self._current_episode_index,
            "frames": len(frames),
            "duration_sec": round(duration, 2),
            "fps_actual": round(len(frames) / duration, 1) if duration > 0 else 0,
            "task": self._current_task,
            "tags": tags,
            "files": {
                "parquet": result.get("parquet"),
                "videos": video_files,
            },
        }

    def pause_recording(self) -> dict:
        """Pause recording."""
        if self._state != "recording":
            return {"success": False, "error": "Not recording", "state": self._state}

        self._frame_buffer.pause_recording()
        self._state = "paused"

        return {
            "success": True,
            "state": "paused",
            "frames_so_far": self._frame_buffer.get_frame_count(),
            "duration_so_far_sec": round(self._frame_buffer.get_duration(), 2),
        }

    def resume_recording(self) -> dict:
        """Resume recording."""
        if self._state != "paused":
            return {"success": False, "error": "Not paused", "state": self._state}

        self._frame_buffer.resume_recording()
        self._state = "recording"

        return {"success": True, "state": "recording"}

    def cancel_recording(self) -> dict:
        """Cancel recording and discard data."""
        if self._state not in ("recording", "paused"):
            return {"success": False, "error": "Not recording", "state": self._state}

        frames_discarded = self._frame_buffer.get_frame_count()
        self._frame_buffer.stop_recording()
        self._frame_buffer.clear()
        self._state = "idle"

        # Cleanup temp images
        if self._writer and self._use_video:
            tmp_dir = self._writer.dataset_dir / "videos" / "_tmp"
            if tmp_dir.exists():
                import shutil
                shutil.rmtree(tmp_dir)

        self.get_logger().info(f'Recording cancelled: {frames_discarded} frames discarded')

        return {
            "success": True,
            "message": "Episode cancelled and data discarded",
            "frames_discarded": frames_discarded,
        }

    def get_status(self) -> dict:
        """Get current status."""
        return {
            "state": self._state,
            "dataset_name": self._writer.dataset_name if self._writer else None,
            "episode_index": self._current_episode_index if self._state != "idle" else None,
            "task": self._current_task if self._state != "idle" else None,
            "frame_count": self._frame_buffer.get_frame_count(),
            "duration_sec": round(self._frame_buffer.get_duration(), 2),
            "fps_target": self._fps,
            "topics": self._frame_buffer.get_topic_status(),
        }

    def get_config(self) -> dict:
        """Get current configuration."""
        return {
            "output_dir": str(self._output_dir),
            "default_dataset_name": self._default_dataset_name,
            "fps": self._fps,
            "use_video": self._use_video,
            "robot_type": self._robot_type,
            "arms": self._arm_configs,
            "joint_names": self._joint_names,
            "cameras": self._camera_configs,
            "web_port": self._web_port,
        }

    # === WebAPI ===

    def _start_web_server(self):
        """Start the WebAPI server in a background thread."""
        if not HAS_AIOHTTP:
            self.get_logger().error('aiohttp not installed. WebAPI disabled.')
            return

        def run_server():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)

            app = web.Application()
            app.router.add_post('/api/episode/start', self._handle_start)
            app.router.add_post('/api/episode/stop', self._handle_stop)
            app.router.add_post('/api/episode/pause', self._handle_pause)
            app.router.add_post('/api/episode/resume', self._handle_resume)
            app.router.add_post('/api/episode/cancel', self._handle_cancel)
            app.router.add_get('/api/status', self._handle_status)
            app.router.add_get('/api/config', self._handle_config)
            app.router.add_get('/api/dataset/info', self._handle_dataset_info)

            runner = web.AppRunner(app)
            self._loop.run_until_complete(runner.setup())
            site = web.TCPSite(runner, '0.0.0.0', self._web_port)
            self._loop.run_until_complete(site.start())
            self._loop.run_forever()

        self._web_thread = threading.Thread(target=run_server, daemon=True)
        self._web_thread.start()

    async def _handle_start(self, request: web.Request) -> web.Response:
        try:
            data = await request.json()
        except:
            data = {}

        task = data.get('task')
        if not task:
            return web.json_response({"success": False, "error": "task is required"}, status=400)

        result = self.start_recording(
            task=task,
            dataset_name=data.get('dataset_name'),
            tags=data.get('tags'),
            metadata=data.get('metadata'),
        )
        return web.json_response(result)

    async def _handle_stop(self, request: web.Request) -> web.Response:
        try:
            data = await request.json()
        except:
            data = {}

        result = self.stop_recording(
            tags_append=data.get('tags_append'),
            metadata_append=data.get('metadata_append'),
        )
        return web.json_response(result)

    async def _handle_pause(self, request: web.Request) -> web.Response:
        result = self.pause_recording()
        return web.json_response(result)

    async def _handle_resume(self, request: web.Request) -> web.Response:
        result = self.resume_recording()
        return web.json_response(result)

    async def _handle_cancel(self, request: web.Request) -> web.Response:
        result = self.cancel_recording()
        return web.json_response(result)

    async def _handle_status(self, request: web.Request) -> web.Response:
        return web.json_response(self.get_status())

    async def _handle_config(self, request: web.Request) -> web.Response:
        return web.json_response(self.get_config())

    async def _handle_dataset_info(self, request: web.Request) -> web.Response:
        if self._writer:
            return web.json_response(self._writer.get_dataset_info())
        return web.json_response({"error": "No dataset initialized"}, status=404)


def main(args=None):
    rclpy.init(args=args)
    node = EpisodeRecorderNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._frame_buffer.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
