#!/usr/bin/env python3
"""
VLAbor Dashboard Node
プロファイル対応の統合ダッシュボード (サーボ情報、カメラ、レコーダー制御、ノード監視、ログ)

aiohttp + WebSocket + ROS2
"""
from __future__ import annotations

import asyncio
import base64
import json
import math
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import Log

try:
    from ament_index_python.packages import get_package_share_directory
    HAS_AMENT = True
except ImportError:
    HAS_AMENT = False

try:
    from aiohttp import web, ClientSession
    HAS_AIOHTTP = True
except ImportError:
    HAS_AIOHTTP = False
    web = None
    ClientSession = None

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False
    cv2 = None
    np = None


# ---------------------------------------------------------------------------
# Data Classes
# ---------------------------------------------------------------------------

class ArmData:
    """各アームのデータを保持"""

    def __init__(self, name: str):
        self.name = name
        self.joint_states: Optional[Dict] = None
        self.joint_feedback: Optional[Dict] = None
        self.ik_target_pose: Optional[Dict] = None
        self.ik_joint_angles: Optional[Dict] = None
        self.servo_detail: Optional[Dict] = None
        self.enabled: bool = False
        self.ik_enabled: bool = True
        self.last_joint_states_time: Optional[float] = None
        self.last_joint_feedback_time: Optional[float] = None
        self.last_target_pose_time: Optional[float] = None
        self.last_ik_time: Optional[float] = None
        self.joint_states_rate_hz: Optional[float] = None
        self.joint_feedback_rate_hz: Optional[float] = None
        self.target_pose_rate_hz: Optional[float] = None
        self.ik_rate_hz: Optional[float] = None


class CameraData:
    """カメラフレームデータ"""

    def __init__(self, name: str, topic: str):
        self.name = name
        self.topic = topic
        self.last_frame_b64: Optional[str] = None
        self.width: int = 0
        self.height: int = 0
        self.last_frame_time: float = 0.0
        self.frame_count: int = 0


class NodeStatusData:
    """ノードステータスデータ"""

    def __init__(self, name: str, namespace: str):
        self.name = name
        self.namespace = namespace
        self.running: bool = False
        self.last_seen: Optional[float] = None


class LogEntry:
    """ログエントリ"""

    def __init__(self, stamp_sec: float, level: int, name: str,
                 msg: str, file: str, function: str, line: int):
        self.stamp_sec = stamp_sec
        self.level = level
        self.name = name
        self.msg = msg
        self.file = file
        self.function = function
        self.line = line

    def to_dict(self) -> Dict[str, Any]:
        return {
            'stamp': self.stamp_sec,
            'level': self.level,
            'name': self.name,
            'msg': self.msg,
            'file': self.file,
            'function': self.function,
            'line': self.line,
        }


# ---------------------------------------------------------------------------
# プロファイル別の期待ノード一覧
# (ノード名, 名前空間) のリスト
# ---------------------------------------------------------------------------

EXPECTED_NODES_BY_PROFILE: Dict[str, List[Dict[str, str]]] = {
    'so101_vr_dual_teleop': [
        {'name': 'ros_tcp_endpoint', 'ns': '/'},
        {'name': 'vr_dual_arm_control_node', 'ns': '/'},
        {'name': 'left_arm_ik_solver_node', 'ns': '/'},
        {'name': 'right_arm_ik_solver_node', 'ns': '/'},
        {'name': 'so101_control_node', 'ns': '/left_arm'},
        {'name': 'so101_control_node', 'ns': '/right_arm'},
        {'name': 'robot_state_publisher', 'ns': '/left_arm'},
        {'name': 'robot_state_publisher', 'ns': '/right_arm'},
        {'name': 'so101_webui_left', 'ns': '/'},
        {'name': 'so101_webui_right', 'ns': '/'},
        {'name': 'vlabor_dashboard', 'ns': '/'},
        {'name': 'overhead_camera', 'ns': '/'},
        {'name': 'episode_recorder', 'ns': '/'},
    ],
    'so101_dual_teleop': [
        {'name': 'joint_state_mirror_dual', 'ns': '/'},
        {'name': 'vr_dual_arm_control_node', 'ns': '/'},
        {'name': 'left_arm_ik_solver_node', 'ns': '/'},
        {'name': 'right_arm_ik_solver_node', 'ns': '/'},
        {'name': 'so101_control_node', 'ns': '/left_arm'},
        {'name': 'so101_control_node', 'ns': '/right_arm'},
        {'name': 'robot_state_publisher', 'ns': '/left_arm'},
        {'name': 'robot_state_publisher', 'ns': '/right_arm'},
        {'name': 'so101_webui_left', 'ns': '/'},
        {'name': 'so101_webui_right', 'ns': '/'},
        {'name': 'vlabor_dashboard', 'ns': '/'},
        {'name': 'overhead_camera', 'ns': '/'},
        {'name': 'episode_recorder', 'ns': '/'},
    ],
    'so101_single_teleop': [
        {'name': 'joint_state_mirror_single', 'ns': '/'},
        {'name': 'so101_control_node', 'ns': '/left_arm'},
        {'name': 'so101_control_node', 'ns': '/right_arm'},
        {'name': 'robot_state_publisher', 'ns': '/left_arm'},
        {'name': 'robot_state_publisher', 'ns': '/right_arm'},
        {'name': 'so101_webui_left', 'ns': '/'},
        {'name': 'vlabor_dashboard', 'ns': '/'},
        {'name': 'overhead_camera', 'ns': '/'},
        {'name': 'episode_recorder', 'ns': '/'},
    ],
    'vr_server_only': [
        {'name': 'ros_tcp_endpoint', 'ns': '/'},
    ],
    'overhead_camera': [
        {'name': 'overhead_camera', 'ns': '/'},
        {'name': 'vlabor_dashboard', 'ns': '/'},
    ],
}


# ---------------------------------------------------------------------------
# Dashboard Node
# ---------------------------------------------------------------------------

class VlaborDashboardNode(Node):
    """統合ダッシュボード ROS2ノード"""

    def __init__(self):
        super().__init__('vlabor_dashboard_node')

        # パラメータ宣言
        self.declare_parameter('web_port', 8888)
        self.declare_parameter('profile', '')
        self.declare_parameter('arm_namespaces', ['left_arm', 'right_arm'])
        self.declare_parameter('camera_topics', ['/overhead_camera/image_raw'])
        self.declare_parameter('camera_names', ['overhead_camera'])
        self.declare_parameter('recorder_api_url', 'http://localhost:8082/api')
        self.declare_parameter('webui_ports', [0])
        self.declare_parameter('image_quality', 50)
        self.declare_parameter('image_max_fps', 5)

        # パラメータ取得
        self.web_port = self.get_parameter('web_port').value
        self.profile = self.get_parameter('profile').value or ''
        self.arm_namespaces = list(self.get_parameter('arm_namespaces').value)
        camera_topics = list(self.get_parameter('camera_topics').value)
        camera_names = list(self.get_parameter('camera_names').value)
        self.recorder_api_url = self.get_parameter('recorder_api_url').value
        self.image_quality = self.get_parameter('image_quality').value
        self.image_max_fps = self.get_parameter('image_max_fps').value

        # アームごとの WebUI ポートマッピング
        webui_ports_raw = list(self.get_parameter('webui_ports').value)
        self.webui_port_map: Dict[str, int] = {}
        for i, ns in enumerate(self.arm_namespaces):
            if i < len(webui_ports_raw):
                port = int(webui_ports_raw[i])
                if port > 0:
                    self.webui_port_map[ns] = port

        self.rate_alpha = 0.3
        self.min_frame_interval = 1.0 / max(self.image_max_fps, 1)

        # アームデータ
        self.arms: Dict[str, ArmData] = {}
        for ns in self.arm_namespaces:
            self.arms[ns] = ArmData(ns)
            self._subscribe_arm(ns)

        # カメラデータ
        self.cameras: Dict[str, CameraData] = {}
        for i, topic in enumerate(camera_topics):
            name = camera_names[i] if i < len(camera_names) else f'camera_{i}'
            self.cameras[name] = CameraData(name, topic)
            self._subscribe_camera(name, topic)

        # WebSocket クライアント
        self.ws_clients: List = []
        self.ws_lock = threading.Lock()

        # Web サーバー
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.web_thread: Optional[threading.Thread] = None

        # レートリミット
        self._last_broadcast: Dict[str, float] = {}
        self._last_recorder_poll: float = 0.0
        self._recorder_status: Optional[Dict] = None

        # --- ノード監視 ---
        # 期待ノードリストの構築
        self.expected_nodes = EXPECTED_NODES_BY_PROFILE.get(self.profile, [])

        # ノードステータスデータ
        self.node_statuses: Dict[str, NodeStatusData] = {}
        for node_info in self.expected_nodes:
            key = f"{node_info['ns']}/{node_info['name']}".replace('//', '/')
            self.node_statuses[key] = NodeStatusData(
                node_info['name'], node_info['ns'])

        # ログバッファ
        self.log_buffer: deque = deque(maxlen=500)
        self.log_pending: List[Dict] = []
        self.log_lock = threading.Lock()

        # /rosout 購読
        self.create_subscription(Log, '/rosout', self._on_rosout, 50)

        # ノードディスカバリタイマー (2秒間隔)
        self.create_timer(2.0, self._poll_node_status)

        self.get_logger().info(f'VLAbor Dashboard initialized')
        self.get_logger().info(f'  Profile: {self.profile}')
        self.get_logger().info(f'  Port: {self.web_port}')
        self.get_logger().info(f'  Arms: {self.arm_namespaces}')
        self.get_logger().info(f'  Cameras: {list(self.cameras.keys())}')
        self.get_logger().info(f'  Recorder API: {self.recorder_api_url}')
        self.get_logger().info(f'  Expected nodes: {len(self.expected_nodes)}')

    # ------------------------------------------------------------------
    # ROS2 Subscriptions
    # ------------------------------------------------------------------

    def _subscribe_arm(self, ns: str):
        """アームのトピックを購読"""
        self.create_subscription(
            JointState, f'/{ns}/joint_states_single',
            lambda msg, arm=ns: self._on_joint_states(arm, msg), 10)
        self.create_subscription(
            JointState, f'/{ns}/joint_states_feedback',
            lambda msg, arm=ns: self._on_joint_feedback(arm, msg), 10)
        self.create_subscription(
            PoseStamped, f'/{ns}/target_pose',
            lambda msg, arm=ns: self._on_target_pose(arm, msg), 10)
        self.create_subscription(
            JointState, f'/{ns}/ik/joint_angles',
            lambda msg, arm=ns: self._on_ik_joint_angles(arm, msg), 10)
        self.create_subscription(
            String, f'/{ns}/servo_detail',
            lambda msg, arm=ns: self._on_servo_detail(arm, msg), 10)

        # パブリッシャー (トルク / IK 制御)
        pub = self.create_publisher(Bool, f'/{ns}/enable_flag', 1)
        setattr(self, f'_enable_pub_{ns}', pub)
        ik_pub = self.create_publisher(Bool, f'/{ns}/ik_enable', 1)
        setattr(self, f'_ik_enable_pub_{ns}', ik_pub)

    def _subscribe_camera(self, name: str, topic: str):
        """カメラトピックを購読"""
        if not HAS_CV2:
            self.get_logger().warning(
                f'cv2 not available; camera streaming disabled for {name}')
            return

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.create_subscription(
            Image, topic,
            lambda msg, cam=name: self._on_camera_image(cam, msg),
            qos)

    # ------------------------------------------------------------------
    # Arm Callbacks
    # ------------------------------------------------------------------

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _update_rate(self, last_time: Optional[float], now: float,
                     prev_rate: Optional[float]) -> Optional[float]:
        if last_time is None:
            return prev_rate
        dt = now - last_time
        if dt <= 0:
            return prev_rate
        inst = 1.0 / dt
        if prev_rate is None:
            return inst
        return prev_rate * (1.0 - self.rate_alpha) + inst * self.rate_alpha

    def _stream_status(self, last_time: Optional[float],
                       rate: Optional[float]) -> Optional[Dict[str, float]]:
        if last_time is None:
            return None
        age_sec = max(self._now_sec() - last_time, 0.0)
        return {'age_sec': round(age_sec, 2), 'rate_hz': round(rate, 1) if rate else None}

    def _joint_state_to_dict(self, msg: JointState) -> Dict:
        joints = {}
        names = list(msg.name) if msg.name else [f'joint_{i}' for i in range(len(msg.position))]
        for i, name in enumerate(names):
            pos_rad = msg.position[i] if i < len(msg.position) else 0.0
            joints[name] = {
                'position_rad': round(pos_rad, 4),
                'position_deg': round(math.degrees(pos_rad), 2),
                'velocity': round(msg.velocity[i], 4) if msg.velocity and i < len(msg.velocity) else 0.0,
                'effort': round(msg.effort[i], 4) if msg.effort and i < len(msg.effort) else 0.0,
            }
        return {'joints': joints, 'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9}

    def _on_joint_states(self, arm: str, msg: JointState):
        if arm not in self.arms:
            return
        a = self.arms[arm]
        now = self._now_sec()
        a.joint_states_rate_hz = self._update_rate(a.last_joint_states_time, now, a.joint_states_rate_hz)
        a.last_joint_states_time = now
        a.joint_states = self._joint_state_to_dict(msg)
        self._broadcast_arm_update(arm)

    def _on_joint_feedback(self, arm: str, msg: JointState):
        if arm not in self.arms:
            return
        a = self.arms[arm]
        now = self._now_sec()
        a.joint_feedback_rate_hz = self._update_rate(a.last_joint_feedback_time, now, a.joint_feedback_rate_hz)
        a.last_joint_feedback_time = now
        a.joint_feedback = self._joint_state_to_dict(msg)
        self._broadcast_arm_update(arm)

    def _on_target_pose(self, arm: str, msg: PoseStamped):
        if arm not in self.arms:
            return
        a = self.arms[arm]
        now = self._now_sec()
        a.target_pose_rate_hz = self._update_rate(a.last_target_pose_time, now, a.target_pose_rate_hz)
        a.last_target_pose_time = now
        a.ik_target_pose = {
            'frame_id': msg.header.frame_id,
            'position': {
                'x': round(msg.pose.position.x, 4),
                'y': round(msg.pose.position.y, 4),
                'z': round(msg.pose.position.z, 4),
            },
            'orientation': {
                'x': round(msg.pose.orientation.x, 4),
                'y': round(msg.pose.orientation.y, 4),
                'z': round(msg.pose.orientation.z, 4),
                'w': round(msg.pose.orientation.w, 4),
            }
        }
        self._broadcast_arm_update(arm)

    def _on_ik_joint_angles(self, arm: str, msg: JointState):
        if arm not in self.arms:
            return
        a = self.arms[arm]
        now = self._now_sec()
        a.ik_rate_hz = self._update_rate(a.last_ik_time, now, a.ik_rate_hz)
        a.last_ik_time = now
        a.ik_joint_angles = self._joint_state_to_dict(msg)
        self._broadcast_arm_update(arm)

    def _on_servo_detail(self, arm: str, msg: String):
        if arm not in self.arms:
            return
        try:
            detail = json.loads(msg.data)
            self.arms[arm].servo_detail = detail
            self.arms[arm].enabled = detail.get('enabled', False)
            self.arms[arm].ik_enabled = detail.get('ik_enabled', True)
        except json.JSONDecodeError:
            pass

    # ------------------------------------------------------------------
    # Camera Callback
    # ------------------------------------------------------------------

    def _on_camera_image(self, name: str, msg: Image):
        cam = self.cameras.get(name)
        if cam is None:
            return

        now = time.time()
        if now - cam.last_frame_time < self.min_frame_interval:
            return

        try:
            # ROS Image → numpy → JPEG → base64
            if msg.encoding in ('rgb8', 'bgr8'):
                dtype = np.uint8
                channels = 3
            elif msg.encoding in ('mono8',):
                dtype = np.uint8
                channels = 1
            elif msg.encoding in ('16UC1',):
                dtype = np.uint16
                channels = 1
            else:
                dtype = np.uint8
                channels = 3

            img = np.frombuffer(msg.data, dtype=dtype).reshape(
                msg.height, msg.width, channels) if channels > 1 else \
                np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)

            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'mono8':
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            elif msg.encoding == '16UC1':
                # depth: normalize to 8bit for visualization
                img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                img = cv2.applyColorMap(img, cv2.COLORMAP_JET)

            _, buf = cv2.imencode('.jpg', img,
                                  [cv2.IMWRITE_JPEG_QUALITY, self.image_quality])
            b64 = base64.b64encode(buf).decode('ascii')

            cam.last_frame_b64 = b64
            cam.width = msg.width
            cam.height = msg.height
            cam.last_frame_time = now
            cam.frame_count += 1

            self._broadcast_camera_frame(name)

        except Exception as e:
            self.get_logger().warning(f'Camera {name} frame error: {e}')

    # ------------------------------------------------------------------
    # Node Monitoring
    # ------------------------------------------------------------------

    def _on_rosout(self, msg: Log):
        """ログメッセージ受信コールバック"""
        stamp_sec = msg.stamp.sec + msg.stamp.nanosec / 1e9
        entry = LogEntry(
            stamp_sec=stamp_sec,
            level=msg.level,
            name=msg.name,
            msg=msg.msg,
            file=msg.file,
            function=msg.function,
            line=msg.line,
        )
        self.log_buffer.append(entry)
        with self.log_lock:
            self.log_pending.append(entry.to_dict())

    def _poll_node_status(self):
        """ノードの存在をポーリングで確認"""
        try:
            discovered = self.get_node_names_and_namespaces()
        except Exception:
            return

        # 検出されたノードをセットに変換
        running_set = set()
        for name, ns in discovered:
            full_name = f"{ns}/{name}".replace('//', '/')
            running_set.add(full_name)

        now = self._now_sec()
        for key, status in self.node_statuses.items():
            status.running = key in running_set
            if status.running:
                status.last_seen = now

        self._broadcast_node_status()

    def _broadcast_node_status(self):
        """ノードステータスをブロードキャスト"""
        if self.loop is None or not self.ws_clients:
            return
        now = time.time()
        last = self._last_broadcast.get('_node_status', 0)
        if now - last < 1.0:
            return
        self._last_broadcast['_node_status'] = now

        try:
            data = self._get_node_status_data()
            payload = json.dumps({'type': 'node_status', 'data': data})
            asyncio.run_coroutine_threadsafe(
                self._broadcast_ws(payload), self.loop)
        except Exception as e:
            self.get_logger().warning(f'Broadcast node status failed: {e}')

    def _get_node_status_data(self) -> List[Dict]:
        """ノードステータスを辞書リストで返す"""
        result = []
        for key, status in self.node_statuses.items():
            result.append({
                'key': key,
                'name': status.name,
                'namespace': status.namespace,
                'running': status.running,
                'last_seen': status.last_seen,
            })
        return result

    # ------------------------------------------------------------------
    # Control Publishers
    # ------------------------------------------------------------------

    def set_arm_enabled(self, arm: str, enabled: bool):
        pub = getattr(self, f'_enable_pub_{arm}', None)
        if pub:
            msg = Bool()
            msg.data = enabled
            pub.publish(msg)
            self.get_logger().info(f'Dashboard: {arm} torque -> {enabled}')

    def set_ik_enabled(self, arm: str, enabled: bool):
        pub = getattr(self, f'_ik_enable_pub_{arm}', None)
        if pub:
            msg = Bool()
            msg.data = enabled
            pub.publish(msg)
            self.get_logger().info(f'Dashboard: {arm} IK -> {enabled}')

    # ------------------------------------------------------------------
    # WebSocket Broadcast
    # ------------------------------------------------------------------

    def _broadcast_arm_update(self, arm: str):
        if self.loop is None or not self.ws_clients:
            return
        now = time.time()
        last = self._last_broadcast.get(arm, 0)
        if now - last < 0.2:  # 5Hz
            return
        self._last_broadcast[arm] = now

        try:
            data = self._get_arm_status(arm)
            payload = json.dumps({'type': 'arm_update', 'arm': arm, 'data': data})
            asyncio.run_coroutine_threadsafe(self._broadcast_ws(payload), self.loop)
        except Exception as e:
            self.get_logger().warning(f'Broadcast arm failed: {e}')

    def _broadcast_camera_frame(self, name: str):
        if self.loop is None or not self.ws_clients:
            return
        cam = self.cameras.get(name)
        if cam is None or cam.last_frame_b64 is None:
            return
        try:
            payload = json.dumps({
                'type': 'camera_frame',
                'camera': name,
                'data': cam.last_frame_b64,
                'width': cam.width,
                'height': cam.height,
            })
            asyncio.run_coroutine_threadsafe(self._broadcast_ws(payload), self.loop)
        except Exception as e:
            self.get_logger().warning(f'Broadcast camera failed: {e}')

    async def _broadcast_ws(self, message: str):
        with self.ws_lock:
            clients = list(self.ws_clients)
        if not clients:
            return
        dead = []
        for ws in clients:
            try:
                await asyncio.wait_for(ws.send_str(message), timeout=1.0)
            except Exception:
                dead.append(ws)
        if dead:
            with self.ws_lock:
                for ws in dead:
                    if ws in self.ws_clients:
                        self.ws_clients.remove(ws)

    def _get_arm_status(self, arm: str) -> Dict:
        if arm not in self.arms:
            return {}
        a = self.arms[arm]
        return {
            'joint_states': a.joint_states,
            'joint_feedback': a.joint_feedback,
            'ik_target_pose': a.ik_target_pose,
            'ik_joint_angles': a.ik_joint_angles,
            'servo_detail': a.servo_detail,
            'enabled': a.enabled,
            'ik_enabled': a.ik_enabled,
            'stream_status': {
                'joint_states': self._stream_status(a.last_joint_states_time, a.joint_states_rate_hz),
                'joint_feedback': self._stream_status(a.last_joint_feedback_time, a.joint_feedback_rate_hz),
                'target_pose': self._stream_status(a.last_target_pose_time, a.target_pose_rate_hz),
                'ik_joint_angles': self._stream_status(a.last_ik_time, a.ik_rate_hz),
            },
        }

    # ------------------------------------------------------------------
    # LeRobot Recorder Proxy
    # ------------------------------------------------------------------

    async def _recorder_request(self, method: str, path: str,
                                body: Optional[Dict] = None) -> Dict:
        """LeRobot Recorder HTTP API にプロキシリクエスト"""
        url = f'{self.recorder_api_url}{path}'
        try:
            async with ClientSession() as session:
                if method == 'GET':
                    async with session.get(url, timeout=3) as resp:
                        return await resp.json()
                else:
                    async with session.post(url, json=body or {}, timeout=5) as resp:
                        return await resp.json()
        except Exception as e:
            return {'error': str(e), 'available': False}

    async def _poll_recorder_status(self):
        """レコーダーの状態をポーリングしてブロードキャスト"""
        result = await self._recorder_request('GET', '/status')
        self._recorder_status = result
        payload = json.dumps({'type': 'recorder_status', 'data': result})
        await self._broadcast_ws(payload)

    # ------------------------------------------------------------------
    # Web Server
    # ------------------------------------------------------------------

    def start_web_server(self):
        if not HAS_AIOHTTP:
            self.get_logger().error('aiohttp not installed. pip install aiohttp')
            return

        def run_server():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)

            app = web.Application()
            app.router.add_get('/ws', self._ws_handler)
            app.router.add_get('/', self._index_handler)
            web_dir = self._get_web_dir()
            if Path(web_dir).exists():
                app.router.add_static('/static/', web_dir)

            runner = web.AppRunner(app)
            self.loop.run_until_complete(runner.setup())
            site = web.TCPSite(runner, '0.0.0.0', self.web_port)
            self.loop.run_until_complete(site.start())
            self.get_logger().info(f'Dashboard at http://0.0.0.0:{self.web_port}/')

            # レコーダーポーリングタスク
            self.loop.create_task(self._recorder_poll_loop())
            # ログフラッシュタスク
            self.loop.create_task(self._log_flush_loop())

            self.loop.run_forever()

        self.web_thread = threading.Thread(target=run_server, daemon=True)
        self.web_thread.start()

    async def _recorder_poll_loop(self):
        """1秒ごとにレコーダー状態をポーリング"""
        while True:
            await asyncio.sleep(1.0)
            if self.ws_clients:
                try:
                    await self._poll_recorder_status()
                except Exception:
                    pass

    async def _log_flush_loop(self):
        """未送信のログを定期的にフラッシュ"""
        while True:
            await asyncio.sleep(0.2)
            if not self.ws_clients:
                continue
            with self.log_lock:
                if not self.log_pending:
                    continue
                batch = self.log_pending[:]
                self.log_pending.clear()
            try:
                payload = json.dumps({'type': 'log_batch', 'logs': batch})
                await self._broadcast_ws(payload)
            except Exception:
                pass

    def _get_web_dir(self) -> str:
        if HAS_AMENT:
            try:
                pkg_dir = get_package_share_directory('vlabor_dashboard')
                web_dir = Path(pkg_dir) / 'web'
                if web_dir.exists():
                    return str(web_dir)
            except Exception:
                pass
        return str(Path(__file__).parent.parent / 'web')

    async def _index_handler(self, request):
        index_path = Path(self._get_web_dir()) / 'index.html'
        if index_path.exists():
            return web.FileResponse(index_path)
        return web.Response(text='Dashboard index.html not found', status=404)

    async def _ws_handler(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        with self.ws_lock:
            self.ws_clients.append(ws)
            self.get_logger().info(f'Dashboard WS connected. Total: {len(self.ws_clients)}')

        # 初期状態送信
        init_data = {
            'type': 'init',
            'profile': self.profile,
            'arms': self.arm_namespaces,
            'cameras': list(self.cameras.keys()),
            'webui_ports': self.webui_port_map,
            'data': {arm: self._get_arm_status(arm) for arm in self.arm_namespaces},
            'recorder_status': self._recorder_status,
            'node_status': self._get_node_status_data(),
            'log_history': [e.to_dict() for e in self.log_buffer],
        }
        await ws.send_str(json.dumps(init_data))

        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    await self._handle_ws_message(ws, msg.data)
                elif msg.type == web.WSMsgType.ERROR:
                    break
        finally:
            with self.ws_lock:
                if ws in self.ws_clients:
                    self.ws_clients.remove(ws)

        return ws

    async def _handle_ws_message(self, ws, data: str):
        try:
            msg = json.loads(data)
            msg_type = msg.get('type')

            if msg_type == 'set_torque':
                arm = msg.get('arm')
                enabled = msg.get('enabled', False)
                self.set_arm_enabled(arm, enabled)
                await ws.send_str(json.dumps({
                    'type': 'torque_response', 'arm': arm,
                    'enabled': enabled, 'success': True
                }))

            elif msg_type == 'set_ik':
                arm = msg.get('arm')
                enabled = msg.get('enabled', True)
                self.set_ik_enabled(arm, enabled)
                await ws.send_str(json.dumps({
                    'type': 'ik_response', 'arm': arm,
                    'enabled': enabled, 'success': True
                }))

            elif msg_type == 'get_status':
                await ws.send_str(json.dumps({
                    'type': 'all_status',
                    'data': {arm: self._get_arm_status(arm) for arm in self.arm_namespaces}
                }))

            elif msg_type == 'recorder_start':
                result = await self._recorder_request('POST', '/episode/start', {
                    'task': msg.get('task', ''),
                    'dataset_name': msg.get('dataset_name', ''),
                })
                await ws.send_str(json.dumps({'type': 'recorder_start_response', **result}))

            elif msg_type == 'recorder_stop':
                result = await self._recorder_request('POST', '/episode/stop')
                await ws.send_str(json.dumps({'type': 'recorder_stop_response', **result}))

            elif msg_type == 'recorder_pause':
                result = await self._recorder_request('POST', '/episode/pause')
                await ws.send_str(json.dumps({'type': 'recorder_pause_response', **result}))

            elif msg_type == 'recorder_resume':
                result = await self._recorder_request('POST', '/episode/resume')
                await ws.send_str(json.dumps({'type': 'recorder_resume_response', **result}))

            elif msg_type == 'recorder_cancel':
                result = await self._recorder_request('POST', '/episode/cancel')
                await ws.send_str(json.dumps({'type': 'recorder_cancel_response', **result}))

            elif msg_type == 'recorder_status':
                result = await self._recorder_request('GET', '/status')
                await ws.send_str(json.dumps({'type': 'recorder_status', 'data': result}))

            elif msg_type == 'recorder_dataset_info':
                result = await self._recorder_request('GET', '/dataset/info')
                await ws.send_str(json.dumps({'type': 'recorder_dataset_info', 'data': result}))

            elif msg_type == 'get_node_status':
                data = self._get_node_status_data()
                await ws.send_str(json.dumps(
                    {'type': 'node_status', 'data': data}))

            elif msg_type == 'get_log_history':
                logs = [e.to_dict() for e in self.log_buffer]
                await ws.send_str(json.dumps(
                    {'type': 'log_history', 'logs': logs}))

        except json.JSONDecodeError:
            await ws.send_str(json.dumps({'type': 'error', 'message': 'Invalid JSON'}))
        except Exception as e:
            await ws.send_str(json.dumps({'type': 'error', 'message': str(e)}))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(args=None):
    if not HAS_AIOHTTP:
        print('ERROR: aiohttp not installed. Run: pip install aiohttp')
        return

    rclpy.init(args=args)
    node = VlaborDashboardNode()
    node.start_web_server()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
