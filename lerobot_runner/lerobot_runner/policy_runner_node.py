#!/usr/bin/env python3
"""
Policy Runner Node - Run LeRobot policies with WebAPI control.

Main node that:
- Loads and runs LeRobot policies (SmolVLA, ACT)
- Subscribes to joint states and camera topics
- Publishes joint commands
- Provides WebAPI for external control
"""
import json
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String

try:
    from ament_index_python.packages import get_package_share_directory
    HAS_AMENT = True
except ImportError:
    HAS_AMENT = False

from .policy import POLICY_REGISTRY, get_policy_class, BasePolicy, PolicyConfig
from .utils import (
    ros_image_to_numpy,
    ros_jointstate_to_array,
    array_to_ros_jointstate,
    ActionBuffer,
    LowPassFilter,
)
from .webapi import WebAPIServer


class PolicyRunnerNode(Node):
    """
    Policy Runner ROS2 Node.

    States:
    - idle: No model loaded
    - loading: Model is being loaded
    - ready: Model loaded, inference stopped
    - running: Inference active
    - error: Error state
    """

    def __init__(self):
        super().__init__('policy_runner_node')
        self.get_logger().info('Policy Runner Node starting...')

        # State
        self._state = "idle"
        self._error_message = ""

        # Declare and load parameters
        self._declare_parameters()
        self._load_parameters()
        self._load_robot_config()

        # Components
        self._policy: Optional[BasePolicy] = None
        self._action_buffer = ActionBuffer()
        self._filters: Dict[str, LowPassFilter] = {}

        # Observation buffers (latest values)
        self._latest_images: Dict[str, np.ndarray] = {}
        self._latest_states: Dict[str, np.ndarray] = {}

        # Statistics
        self._stats = {
            'inference_count': 0,
            'inference_time_ms_sum': 0.0,
            'publish_count': 0,
            'start_time': None,
        }

        # Setup subscribers
        self._setup_subscribers()

        # Setup publishers
        self._cmd_publishers: Dict[str, rclpy.publisher.Publisher] = {}
        for ns in self._arm_namespaces:
            topic = f'/{ns}/joint_commands'
            self._cmd_publishers[ns] = self.create_publisher(JointState, topic, 10)
            self.get_logger().info(f'  Publisher: {topic}')

        # Status publisher
        self._status_pub = self.create_publisher(String, '/policy_runner/status', 10)

        # Timers (created when inference starts)
        self._inference_timer = None
        self._publish_timer = None

        # WebAPI
        self._web_server = WebAPIServer(self, self._web_port)
        self._web_server.start()

        self.get_logger().info(f'Policy Runner ready')
        self.get_logger().info(f'  State: {self._state}')
        self.get_logger().info(f'  WebAPI: http://0.0.0.0:{self._web_port}/api')

        # Auto-load if configured
        if self._auto_load and self._default_checkpoint:
            self.get_logger().info(f'Auto-loading model: {self._default_checkpoint}')
            self.load_model(self._default_checkpoint, self._default_policy_type)

    def _declare_parameters(self):
        """Declare ROS2 parameters."""
        # Models
        self.declare_parameter('models_dir', '/home/ros2/ros2_ws/src/vlabor_ros2/models')
        self.declare_parameter('default_checkpoint', '')
        self.declare_parameter('default_policy_type', 'smolvla')
        self.declare_parameter('auto_load', False)

        # Device
        self.declare_parameter('device', 'cuda:0')

        # Timing
        self.declare_parameter('inference_rate', 1.0)
        self.declare_parameter('publish_rate', 25.0)
        self.declare_parameter('sync_slop', 0.1)

        # Filtering
        self.declare_parameter('lpf_enable', True)
        self.declare_parameter('lpf_alpha', 0.1)

        # Robot config
        self.declare_parameter('robot_config', 'robots/so101.yaml')

        # Arms
        self.declare_parameter('arm_count', 2)
        for i in range(4):
            self.declare_parameter(f'arm_{i}_namespace', '')

        # Cameras
        self.declare_parameter('camera_count', 1)
        for i in range(4):
            self.declare_parameter(f'camera_{i}_name', '')
            self.declare_parameter(f'camera_{i}_topic', '')

        # WebAPI
        self.declare_parameter('web_port', 8083)

    def _load_parameters(self):
        """Load parameters."""
        self._models_dir = Path(self.get_parameter('models_dir').value)
        self._default_checkpoint = self.get_parameter('default_checkpoint').value
        self._default_policy_type = self.get_parameter('default_policy_type').value
        self._auto_load = self.get_parameter('auto_load').value

        self._device = self.get_parameter('device').value
        self._inference_rate = self.get_parameter('inference_rate').value
        self._publish_rate = self.get_parameter('publish_rate').value

        self._lpf_enable = self.get_parameter('lpf_enable').value
        self._lpf_alpha = self.get_parameter('lpf_alpha').value

        self._robot_config_file = self.get_parameter('robot_config').value
        self._web_port = self.get_parameter('web_port').value

        # Parse arms
        arm_count = self.get_parameter('arm_count').value
        self._arm_namespaces = []
        for i in range(arm_count):
            ns = self.get_parameter(f'arm_{i}_namespace').value
            if ns:
                self._arm_namespaces.append(ns)
        if not self._arm_namespaces:
            self._arm_namespaces = ['left_arm', 'right_arm']

        # Parse cameras
        camera_count = self.get_parameter('camera_count').value
        self._camera_configs = []
        for i in range(camera_count):
            name = self.get_parameter(f'camera_{i}_name').value
            topic = self.get_parameter(f'camera_{i}_topic').value
            if name and topic:
                self._camera_configs.append({'name': name, 'topic': topic})
        if not self._camera_configs:
            self._camera_configs = [{'name': 'cam_front', 'topic': '/cam_front/image_raw'}]

    def _load_robot_config(self):
        """Load robot configuration."""
        config_path = None

        if HAS_AMENT:
            try:
                pkg_dir = get_package_share_directory('lerobot_runner')
                config_path = Path(pkg_dir) / 'config' / self._robot_config_file
            except:
                pass

        if config_path is None or not config_path.exists():
            config_path = Path(__file__).parent.parent / 'config' / self._robot_config_file

        self._robot_type = "unknown"
        self._joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        if config_path and config_path.exists():
            try:
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                robot = config.get('robot', {})
                self._robot_type = robot.get('name', 'unknown')
                joints = robot.get('joints', {})
                self._joint_names = joints.get('names', self._joint_names)
                self.get_logger().info(f'Loaded robot config: {self._robot_type}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load robot config: {e}')

        # Initialize filters for each arm
        for ns in self._arm_namespaces:
            self._filters[ns] = LowPassFilter(
                alpha=self._lpf_alpha,
                num_joints=len(self._joint_names)
            )

    def _setup_subscribers(self):
        """Setup ROS2 subscribers."""
        # Joint state subscribers
        for ns in self._arm_namespaces:
            topic = f'/{ns}/joint_states'
            self.create_subscription(
                JointState,
                topic,
                lambda msg, namespace=ns: self._on_joint_state(namespace, msg),
                10
            )
            self.get_logger().info(f'  Subscribe: {topic}')

        # Camera subscribers
        for cam in self._camera_configs:
            self.create_subscription(
                Image,
                cam['topic'],
                lambda msg, cam_name=cam['name']: self._on_image(cam_name, msg),
                10
            )
            self.get_logger().info(f'  Subscribe: {cam["topic"]}')

    def _on_joint_state(self, namespace: str, msg: JointState):
        """Handle joint state message."""
        state = ros_jointstate_to_array(msg, self._joint_names)
        self._latest_states[namespace] = state

    def _on_image(self, camera_name: str, msg: Image):
        """Handle image message."""
        img = ros_image_to_numpy(msg)
        if img is not None:
            self._latest_images[camera_name] = img

    # === Model Management ===

    def load_model(
        self,
        checkpoint_path: str,
        policy_type: Optional[str] = None,
        device: Optional[str] = None
    ) -> Dict[str, Any]:
        """Load a model."""
        if self._state == "running":
            return {"success": False, "error": "Cannot load while running. Stop first."}

        self._state = "loading"
        start_time = time.time()

        # Detect policy type from checkpoint if not specified
        if policy_type is None:
            policy_type = self._detect_policy_type(checkpoint_path)
            if policy_type is None:
                policy_type = self._default_policy_type

        device = device or self._device

        try:
            # Get policy class
            policy_class = get_policy_class(policy_type)

            # Create policy config
            config = PolicyConfig(
                checkpoint_path=checkpoint_path,
                device=device,
                joint_names=self._joint_names,
                arm_namespaces=self._arm_namespaces,
                lpf_enable=self._lpf_enable,
                lpf_alpha=self._lpf_alpha,
            )

            # Create and load policy
            self._policy = policy_class(config, logger=self.get_logger())
            success = self._policy.load(checkpoint_path, device)

            if success:
                self._state = "ready"
                load_time_ms = (time.time() - start_time) * 1000
                self.get_logger().info(f'Model loaded: {policy_type} in {load_time_ms:.0f}ms')
                return {
                    "success": True,
                    "message": "Model loaded successfully",
                    "model_info": {
                        "policy_type": policy_type,
                        "checkpoint_path": checkpoint_path,
                        "device": device,
                        "load_time_ms": round(load_time_ms),
                    }
                }
            else:
                self._state = "error"
                self._error_message = "Failed to load model"
                return {"success": False, "error": self._error_message}

        except Exception as e:
            self._state = "error"
            self._error_message = str(e)
            self.get_logger().error(f'Load failed: {e}')
            return {"success": False, "error": str(e)}

    def unload_model(self) -> Dict[str, Any]:
        """Unload current model."""
        if self._state == "running":
            self.stop_inference()

        if self._policy is not None:
            self._policy.unload()
            self._policy = None

        self._state = "idle"
        return {"success": True, "message": "Model unloaded"}

    def _detect_policy_type(self, checkpoint_path: str) -> Optional[str]:
        """Try to detect policy type from checkpoint."""
        config_path = Path(checkpoint_path) / "config.json"
        if config_path.exists():
            try:
                with open(config_path) as f:
                    config = json.load(f)
                # Check for policy type indicators
                if 'smolvla' in str(config).lower():
                    return 'smolvla'
                elif 'act' in str(config).lower():
                    return 'act'
            except:
                pass
        return None

    # === Inference Control ===

    def start_inference(
        self,
        task: Optional[str] = None,
        inference_rate: Optional[float] = None,
        publish_rate: Optional[float] = None
    ) -> Dict[str, Any]:
        """Start inference."""
        if self._state != "ready":
            return {"success": False, "error": f"Cannot start in state: {self._state}"}

        if self._policy is None:
            return {"success": False, "error": "No model loaded"}

        # Update task if provided
        if task:
            self._policy.set_task(task)

        # Update rates if provided
        if inference_rate:
            self._inference_rate = inference_rate
        if publish_rate:
            self._publish_rate = publish_rate

        # Reset statistics
        self._stats = {
            'inference_count': 0,
            'inference_time_ms_sum': 0.0,
            'publish_count': 0,
            'start_time': datetime.now().isoformat(),
        }

        # Reset filters
        for lpf in self._filters.values():
            lpf.reset()

        # Reset policy
        self._policy.reset()
        self._action_buffer.clear()

        # Create timers
        self._inference_timer = self.create_timer(
            1.0 / self._inference_rate,
            self._inference_callback
        )
        self._publish_timer = self.create_timer(
            1.0 / self._publish_rate,
            self._publish_callback
        )

        self._state = "running"
        self.get_logger().info(f'Inference started: rate={self._inference_rate}Hz, publish={self._publish_rate}Hz')

        return {
            "success": True,
            "message": "Inference started",
            "state": "running",
            "task": self._policy.config.task,
        }

    def stop_inference(self) -> Dict[str, Any]:
        """Stop inference."""
        if self._state != "running":
            return {"success": False, "error": f"Not running (state: {self._state})"}

        # Cancel timers
        if self._inference_timer:
            self._inference_timer.cancel()
            self._inference_timer = None
        if self._publish_timer:
            self._publish_timer.cancel()
            self._publish_timer = None

        self._action_buffer.clear()
        self._state = "ready"

        # Calculate stats
        duration = 0
        if self._stats['start_time']:
            start = datetime.fromisoformat(self._stats['start_time'])
            duration = (datetime.now() - start).total_seconds()

        avg_inference_ms = 0
        if self._stats['inference_count'] > 0:
            avg_inference_ms = self._stats['inference_time_ms_sum'] / self._stats['inference_count']

        self.get_logger().info(f'Inference stopped: {self._stats["inference_count"]} inferences')

        return {
            "success": True,
            "message": "Inference stopped",
            "stats": {
                "total_inferences": self._stats['inference_count'],
                "total_publishes": self._stats['publish_count'],
                "avg_inference_ms": round(avg_inference_ms, 1),
                "total_duration_sec": round(duration, 1),
            }
        }

    def set_task(self, task: str) -> Dict[str, Any]:
        """Update task description."""
        if self._policy is None:
            return {"success": False, "error": "No model loaded"}

        previous_task = self._policy.config.task
        self._policy.set_task(task)

        return {
            "success": True,
            "previous_task": previous_task,
            "new_task": task,
        }

    def _inference_callback(self):
        """Timer callback for inference."""
        if self._state != "running" or self._policy is None:
            return

        start_time = time.time()

        # Build observation
        observation = {
            'images': self._latest_images.copy(),
            'state': self._get_combined_state(),
            'task': self._policy.config.task,
        }

        # Run inference
        action = self._policy.predict(observation)

        if action is not None:
            # Split action for each arm
            joints_per_arm = len(self._joint_names)
            for i, ns in enumerate(self._arm_namespaces):
                start_idx = i * joints_per_arm
                end_idx = start_idx + joints_per_arm
                if end_idx <= len(action):
                    arm_action = action[start_idx:end_idx]
                    self._action_buffer.set([arm_action])

            self._stats['inference_count'] += 1
            self._stats['inference_time_ms_sum'] += (time.time() - start_time) * 1000

    def _publish_callback(self):
        """Timer callback for publishing commands."""
        if self._state != "running":
            return

        action = self._action_buffer.get()
        if action is None:
            return

        # Publish to each arm
        for i, ns in enumerate(self._arm_namespaces):
            # Apply filter if enabled
            if self._lpf_enable and ns in self._filters:
                action = self._filters[ns].filter(action)

            # Create and publish message
            msg = array_to_ros_jointstate(
                action,
                self._joint_names,
                self.get_clock().now().to_msg()
            )
            self._cmd_publishers[ns].publish(msg)

        self._stats['publish_count'] += 1

        # Publish status periodically
        if self._stats['publish_count'] % 25 == 0:
            self._publish_status()

    def _get_combined_state(self) -> np.ndarray:
        """Get combined state from all arms."""
        states = []
        for ns in self._arm_namespaces:
            if ns in self._latest_states:
                states.append(self._latest_states[ns])
            else:
                states.append(np.zeros(len(self._joint_names)))
        return np.concatenate(states)

    def _publish_status(self):
        """Publish status to ROS topic."""
        status = self.get_status()
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    # === Status and Configuration ===

    def get_status(self) -> Dict[str, Any]:
        """Get current status."""
        avg_inference_ms = 0
        if self._stats['inference_count'] > 0:
            avg_inference_ms = self._stats['inference_time_ms_sum'] / self._stats['inference_count']

        return {
            "state": self._state,
            "error": self._error_message if self._state == "error" else None,
            "policy_type": self._policy.policy_type if self._policy else None,
            "model_loaded": self._policy is not None and self._policy.is_loaded,
            "checkpoint_path": self._policy.config.checkpoint_path if self._policy else None,
            "task": self._policy.config.task if self._policy else None,
            "device": self._device,
            "inference_rate": self._inference_rate,
            "publish_rate": self._publish_rate,
            "stats": {
                "inference_count": self._stats['inference_count'],
                "avg_inference_ms": round(avg_inference_ms, 1),
                "publish_count": self._stats['publish_count'],
                "start_time": self._stats['start_time'],
            }
        }

    def get_config(self) -> Dict[str, Any]:
        """Get current configuration."""
        return {
            "models_dir": str(self._models_dir),
            "device": self._device,
            "inference_rate": self._inference_rate,
            "publish_rate": self._publish_rate,
            "lpf_enable": self._lpf_enable,
            "lpf_alpha": self._lpf_alpha,
            "robot_type": self._robot_type,
            "joint_names": self._joint_names,
            "arm_namespaces": self._arm_namespaces,
            "cameras": self._camera_configs,
            "web_port": self._web_port,
        }

    def list_models(self) -> Dict[str, Any]:
        """List available models in models directory."""
        models = []

        if self._models_dir.exists():
            for item in self._models_dir.iterdir():
                if item.is_dir():
                    # Check if it's a valid checkpoint
                    config_file = item / "config.json"
                    if config_file.exists():
                        try:
                            with open(config_file) as f:
                                config = json.load(f)
                            policy_type = self._detect_policy_type(str(item)) or "unknown"
                            models.append({
                                "name": item.name,
                                "path": str(item),
                                "policy_type": policy_type,
                                "created": datetime.fromtimestamp(
                                    item.stat().st_mtime
                                ).isoformat(),
                            })
                        except:
                            pass

        return {
            "models_dir": str(self._models_dir),
            "models": models,
        }


def main(args=None):
    rclpy.init(args=args)
    node = PolicyRunnerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._web_server.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
