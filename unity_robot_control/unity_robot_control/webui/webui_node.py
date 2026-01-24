#!/usr/bin/env python3
"""
SO101 WebUI Node
WebSocketサーバー + ROS2トピック購読でリアルタイムモニタリング

機能:
- 左右アームのjoint_states表示
- IK target_pose / ik/joint_angles 表示
- トルク On/Off 制御
- ポジション保存/読込/削除
- キャリブレーション表示
"""
import asyncio
import json
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Any

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

try:
    from ament_index_python.packages import get_package_share_directory
    HAS_AMENT = True
except ImportError:
    HAS_AMENT = False

try:
    from aiohttp import web
    HAS_AIOHTTP = True
except ImportError:
    HAS_AIOHTTP = False
    web = None

try:
    import tf2_ros
    from rclpy.time import Time
    HAS_TF2 = True
except ImportError:
    HAS_TF2 = False
    tf2_ros = None
    Time = None


class ArmData:
    """各アームのデータを保持"""

    def __init__(self, name: str):
        self.name = name
        self.joint_states: Optional[Dict] = None
        self.joint_feedback: Optional[Dict] = None
        self.ik_target_pose: Optional[Dict] = None
        self.ik_joint_angles: Optional[Dict] = None
        self.servo_detail: Optional[Dict] = None  # 詳細サーボ情報（tick値、エラー）
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
        self.tf_status: Optional[Dict] = None


class SO101WebUINode(Node):
    """SO101 WebUI ROS2ノード"""

    def __init__(self):
        super().__init__('so101_webui_node')

        self.declare_parameter('web_port', 8080)
        self.declare_parameter('arm_namespaces', ['left_arm', 'right_arm'])
        self.declare_parameter('positions_dir', '')
        self.declare_parameter('urdf_file', '')
        self.declare_parameter('limit_warning_deg', 5.0)
        self.declare_parameter('calibration_dir', '')

        self.web_port = self.get_parameter('web_port').value
        self.arm_namespaces = list(self.get_parameter('arm_namespaces').value)

        positions_dir = self.get_parameter('positions_dir').value
        if positions_dir:
            self.positions_dir = Path(positions_dir)
        else:
            # デフォルト: パッケージのconfig/positions
            self.positions_dir = Path(__file__).parent.parent.parent / 'config' / 'positions'
        self.positions_dir.mkdir(parents=True, exist_ok=True)

        # キャリブレーションディレクトリ (LeRobot互換)
        calibration_dir = self.get_parameter('calibration_dir').value
        if calibration_dir:
            self.calibration_dir = Path(calibration_dir)
        else:
            self.calibration_dir = Path.home() / '.cache' / 'lerobot' / 'calibration' / 'robots' / 'so101_follower'
        self.calibration_dir.mkdir(parents=True, exist_ok=True)

        self.limit_warning_deg = float(self.get_parameter('limit_warning_deg').value)
        self.rate_alpha = 0.3
        self.joint_limits = self._load_joint_limits(self.get_parameter('urdf_file').value)

        # アームデータ
        self.arms: Dict[str, ArmData] = {}
        for ns in self.arm_namespaces:
            self.arms[ns] = ArmData(ns)
            self._subscribe_arm(ns)

        # キャリブレーション状態 (WebUI内蔵キャリブレーション)
        self._calib_lock = threading.Lock()
        self._calib_status: Dict[str, Dict[str, Any]] = {}
        self._calib_recording: Dict[str, bool] = {}  # Range記録中フラグ
        self._calib_mins: Dict[str, Dict[str, int]] = {}  # 記録中のmin値
        self._calib_maxs: Dict[str, Dict[str, int]] = {}  # 記録中のmax値
        self._calib_homing_offsets: Dict[str, Dict[str, int]] = {}  # ホーミングオフセット

        # WebSocketクライアント
        self.ws_clients: List[web.WebSocketResponse] = []
        self.ws_lock = threading.Lock()

        # Webサーバー
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.web_thread: Optional[threading.Thread] = None

        # TF
        self.tf_buffer = None
        self.tf_listener = None
        if HAS_TF2:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'SO101 WebUI Node initialized')
        self.get_logger().info(f'Arms: {self.arm_namespaces}')
        self.get_logger().info(f'Positions dir: {self.positions_dir}')
        self.get_logger().info(f'Calibration dir: {self.calibration_dir}')
        if not self.joint_limits:
            self.get_logger().info('Joint limit warnings disabled (no limits loaded)')

    def _subscribe_arm(self, ns: str):
        """アームのトピックを購読"""
        # joint_states_single
        self.create_subscription(
            JointState,
            f'/{ns}/joint_states_single',
            lambda msg, arm=ns: self._on_joint_states(arm, msg),
            10
        )
        # joint_states_feedback
        self.create_subscription(
            JointState,
            f'/{ns}/joint_states_feedback',
            lambda msg, arm=ns: self._on_joint_feedback(arm, msg),
            10
        )
        # target_pose (IK入力)
        self.create_subscription(
            PoseStamped,
            f'/{ns}/target_pose',
            lambda msg, arm=ns: self._on_target_pose(arm, msg),
            10
        )
        # ik/joint_angles (IK出力)
        self.create_subscription(
            JointState,
            f'/{ns}/ik/joint_angles',
            lambda msg, arm=ns: self._on_ik_joint_angles(arm, msg),
            10
        )
        # servo_detail (詳細サーボ情報)
        self.create_subscription(
            String,
            f'/{ns}/servo_detail',
            lambda msg, arm=ns: self._on_servo_detail(arm, msg),
            10
        )

        # enable_flag publisher (トルク制御)
        pub = self.create_publisher(Bool, f'/{ns}/enable_flag', 1)
        setattr(self, f'_enable_pub_{ns}', pub)
        # ik_enable publisher (IK有効/無効)
        ik_pub = self.create_publisher(Bool, f'/{ns}/ik_enable', 1)
        setattr(self, f'_ik_enable_pub_{ns}', ik_pub)
        # reload_calibration publisher (キャリブレーションリロード)
        reload_pub = self.create_publisher(String, f'/{ns}/reload_calibration', 1)
        setattr(self, f'_reload_calib_pub_{ns}', reload_pub)

    def _resolve_urdf_file(self, urdf_file: str) -> Optional[Path]:
        if urdf_file:
            path = Path(urdf_file)
            if path.exists():
                return path

        if HAS_AMENT:
            try:
                share_dir = Path(get_package_share_directory('so101_description'))
                candidate = share_dir / 'urdf' / 'so101_new_calib.urdf'
                if candidate.exists():
                    return candidate
            except Exception:
                pass

        try:
            repo_root = Path(__file__).resolve().parents[3]
            candidate = repo_root / 'so101_description' / 'urdf' / 'so101_new_calib.urdf'
            if candidate.exists():
                return candidate
        except Exception:
            pass

        return None

    def _load_joint_limits(self, urdf_file: str) -> Dict[str, Dict[str, float]]:
        path = self._resolve_urdf_file(urdf_file)
        if path is None:
            return {}

        try:
            import math
            tree = ET.parse(path)
            root = tree.getroot()
            limits: Dict[str, Dict[str, float]] = {}
            for joint in root.findall('joint'):
                name = joint.get('name')
                limit = joint.find('limit')
                if not name or limit is None:
                    continue
                lower = limit.get('lower')
                upper = limit.get('upper')
                if lower is None or upper is None:
                    continue
                try:
                    lower_f = float(lower)
                    upper_f = float(upper)
                except ValueError:
                    continue
                limits[name] = {
                    'lower': lower_f,
                    'upper': upper_f,
                    'lower_deg': math.degrees(lower_f),
                    'upper_deg': math.degrees(upper_f),
                }
            if limits:
                self.get_logger().info(f'Loaded joint limits from {path}')
            return limits
        except Exception as exc:
            self.get_logger().warning(f'Failed to load joint limits: {exc}')
            return {}

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _update_rate(self, last_time: Optional[float], now: float, prev_rate: Optional[float]) -> Optional[float]:
        if last_time is None:
            return prev_rate
        dt = now - last_time
        if dt <= 0:
            return prev_rate
        inst = 1.0 / dt
        if prev_rate is None:
            return inst
        return prev_rate * (1.0 - self.rate_alpha) + inst * self.rate_alpha

    def _stream_status(self, last_time: Optional[float], rate: Optional[float]) -> Optional[Dict[str, float]]:
        if last_time is None:
            return None
        age_sec = max(self._now_sec() - last_time, 0.0)
        return {'age_sec': age_sec, 'rate_hz': rate}

    def _touch_stream(self, arm_data: ArmData, stream: str) -> float:
        now = self._now_sec()
        if stream == 'joint_states':
            arm_data.joint_states_rate_hz = self._update_rate(
                arm_data.last_joint_states_time, now, arm_data.joint_states_rate_hz
            )
            arm_data.last_joint_states_time = now
        elif stream == 'joint_feedback':
            arm_data.joint_feedback_rate_hz = self._update_rate(
                arm_data.last_joint_feedback_time, now, arm_data.joint_feedback_rate_hz
            )
            arm_data.last_joint_feedback_time = now
        elif stream == 'target_pose':
            arm_data.target_pose_rate_hz = self._update_rate(
                arm_data.last_target_pose_time, now, arm_data.target_pose_rate_hz
            )
            arm_data.last_target_pose_time = now
        elif stream == 'ik_joint_angles':
            arm_data.ik_rate_hz = self._update_rate(
                arm_data.last_ik_time, now, arm_data.ik_rate_hz
            )
            arm_data.last_ik_time = now
        return now

    def _on_joint_states(self, arm: str, msg: JointState):
        if arm not in self.arms:
            return
        self._touch_stream(self.arms[arm], 'joint_states')
        self.arms[arm].joint_states = self._joint_state_to_dict(msg)
        self._broadcast_update(arm)

    def _on_joint_feedback(self, arm: str, msg: JointState):
        if arm not in self.arms:
            return
        self._touch_stream(self.arms[arm], 'joint_feedback')
        self.arms[arm].joint_feedback = self._joint_state_to_dict(msg)
        self._broadcast_update(arm)

    def _on_target_pose(self, arm: str, msg: PoseStamped):
        if arm not in self.arms:
            return
        now = self._touch_stream(self.arms[arm], 'target_pose')
        self.arms[arm].ik_target_pose = {
            'frame_id': msg.header.frame_id,
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            'received_at': now,
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
            },
            'orientation': {
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w,
            }
        }
        frame_id = msg.header.frame_id
        if self.tf_buffer is not None and frame_id:
            target_frame = f'{arm}/base_link'
            tf_status = {
                'source_frame': frame_id,
                'target_frame': target_frame,
                'ok': False,
                'error': '',
                'checked_at': now,
            }
            try:
                tf_status['ok'] = bool(self.tf_buffer.can_transform(target_frame, frame_id, Time()))
                if not tf_status['ok']:
                    tf_status['error'] = 'transform unavailable'
            except Exception as exc:
                tf_status['ok'] = False
                tf_status['error'] = str(exc)
            self.arms[arm].tf_status = tf_status
        self._broadcast_update(arm)

    def _on_ik_joint_angles(self, arm: str, msg: JointState):
        if arm not in self.arms:
            return
        self._touch_stream(self.arms[arm], 'ik_joint_angles')
        self.arms[arm].ik_joint_angles = self._joint_state_to_dict(msg)
        self._broadcast_update(arm)

    def _on_servo_detail(self, arm: str, msg: String):
        if arm not in self.arms:
            return
        try:
            detail = json.loads(msg.data)
            self.arms[arm].servo_detail = detail
            self.arms[arm].enabled = detail.get('enabled', False)
            self.arms[arm].ik_enabled = detail.get('ik_enabled', True)
            # Range記録中ならmin/max更新
            self.calib_update_range(arm)
        except json.JSONDecodeError:
            pass

    def _joint_state_to_dict(self, msg: JointState) -> Dict:
        import math
        joints = {}
        names = list(msg.name) if msg.name else [f'joint_{i}' for i in range(len(msg.position))]
        for i, name in enumerate(names):
            pos_rad = msg.position[i] if i < len(msg.position) else 0.0
            joints[name] = {
                'position_rad': pos_rad,
                'position_deg': math.degrees(pos_rad),
                'velocity': msg.velocity[i] if msg.velocity and i < len(msg.velocity) else 0.0,
                'effort': msg.effort[i] if msg.effort and i < len(msg.effort) else 0.0,
            }
            limit = self.joint_limits.get(name) if self.joint_limits else None
            if limit:
                joints[name]['limit'] = limit
                violation = pos_rad < limit['lower'] or pos_rad > limit['upper']
                margin = math.radians(self.limit_warning_deg)
                warning = violation or (pos_rad - limit['lower'] < margin) or (limit['upper'] - pos_rad < margin)
                joints[name]['limit_violation'] = violation
                joints[name]['limit_warning'] = warning
        return {'joints': joints, 'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9}

    def _broadcast_update(self, arm: str):
        """WebSocketクライアントに更新を送信（レートリミット付き）"""
        if self.loop is None or not self.ws_clients:
            return

        # レートリミット: 最大5Hz
        now = time.time()
        if not hasattr(self, '_last_broadcast'):
            self._last_broadcast = {}
        last = self._last_broadcast.get(arm, 0)
        if now - last < 0.2:  # 200ms = 5Hz
            return
        self._last_broadcast[arm] = now

        try:
            data = self._get_arm_status(arm)
            msg = json.dumps({'type': 'arm_update', 'arm': arm, 'data': data})
            asyncio.run_coroutine_threadsafe(self._broadcast_ws(msg), self.loop)
        except Exception as e:
            self.get_logger().warning(f'Broadcast failed: {e}')

    async def _broadcast_ws(self, message: str):
        # クライアントリストのスナップショットを取得（ロック時間を最小化）
        with self.ws_lock:
            clients = list(self.ws_clients)

        if not clients:
            return

        dead_clients = []
        for ws in clients:
            try:
                await asyncio.wait_for(ws.send_str(message), timeout=1.0)
            except Exception:
                dead_clients.append(ws)

        if dead_clients:
            with self.ws_lock:
                for ws in dead_clients:
                    if ws in self.ws_clients:
                        self.ws_clients.remove(ws)

    def _get_arm_status(self, arm: str) -> Dict:
        if arm not in self.arms:
            return {}
        a = self.arms[arm]
        calib_status = self._calib_status.get(arm, {})
        return {
            'joint_states': a.joint_states,
            'joint_feedback': a.joint_feedback,
            'ik_target_pose': a.ik_target_pose,
            'ik_joint_angles': a.ik_joint_angles,
            'servo_detail': a.servo_detail,
            'enabled': a.enabled,
            'ik_enabled': a.ik_enabled,
            'calibration_status': calib_status,
            'stream_status': {
                'joint_states': self._stream_status(a.last_joint_states_time, a.joint_states_rate_hz),
                'joint_feedback': self._stream_status(a.last_joint_feedback_time, a.joint_feedback_rate_hz),
                'target_pose': self._stream_status(a.last_target_pose_time, a.target_pose_rate_hz),
                'ik_joint_angles': self._stream_status(a.last_ik_time, a.ik_rate_hz),
            },
            'tf_status': a.tf_status,
        }

    def set_arm_enabled(self, arm: str, enabled: bool):
        """アームのトルクを設定"""
        pub = getattr(self, f'_enable_pub_{arm}', None)
        if pub:
            msg = Bool()
            msg.data = enabled
            pub.publish(msg)
            self.get_logger().info(f'WebUI: {arm} torque -> {enabled}')

    def set_ik_enabled(self, arm: str, enabled: bool):
        """IKの有効/無効を設定"""
        pub = getattr(self, f'_ik_enable_pub_{arm}', None)
        if pub:
            msg = Bool()
            msg.data = enabled
            pub.publish(msg)
            self.arms[arm].ik_enabled = enabled
            self.get_logger().info(f'{arm} IK (Remote): {"ON" if enabled else "OFF"}')

    def save_position(self, arm: str, name: str) -> bool:
        """現在位置を保存"""
        if arm not in self.arms or not self.arms[arm].joint_states:
            return False
        positions = {
            k: v['position_rad']
            for k, v in self.arms[arm].joint_states.get('joints', {}).items()
        }
        file_path = self.positions_dir / f'{arm}_{name}.json'
        try:
            with open(file_path, 'w') as f:
                json.dump({'name': name, 'arm': arm, 'positions': positions}, f, indent=2)
            self.get_logger().info(f'Position saved: {file_path}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save position: {e}')
            return False

    def load_position(self, arm: str, name: str) -> Optional[Dict]:
        """保存位置を読込"""
        file_path = self.positions_dir / f'{arm}_{name}.json'
        if not file_path.exists():
            return None
        try:
            with open(file_path, 'r') as f:
                return json.load(f)
        except Exception:
            return None

    def delete_position(self, arm: str, name: str) -> bool:
        """保存位置を削除"""
        file_path = self.positions_dir / f'{arm}_{name}.json'
        if file_path.exists():
            file_path.unlink()
            return True
        return False

    def list_positions(self, arm: str = None) -> List[Dict]:
        """保存位置一覧"""
        positions = []
        for f in self.positions_dir.glob('*.json'):
            try:
                with open(f, 'r') as fp:
                    data = json.load(fp)
                    if arm is None or data.get('arm') == arm:
                        positions.append({
                            'name': data.get('name', f.stem),
                            'arm': data.get('arm', ''),
                            'file': f.name,
                        })
            except Exception:
                pass
        return positions

    # ========== LeRobot互換キャリブレーション機能 ==========

    def _send_ws_message(self, ws: web.WebSocketResponse, payload: Dict[str, Any]) -> None:
        if self.loop is None:
            return
        try:
            asyncio.run_coroutine_threadsafe(ws.send_str(json.dumps(payload)), self.loop)
        except Exception as exc:
            self.get_logger().warning(f'WS send failed: {exc}')

    def _broadcast_ws_message(self, payload: Dict[str, Any]) -> None:
        if self.loop is None:
            return
        asyncio.run_coroutine_threadsafe(self._broadcast_ws(json.dumps(payload)), self.loop)

    def _set_calib_status(self, arm: str, status: Dict[str, Any]) -> None:
        self._calib_status[arm] = status
        self._broadcast_ws_message({
            'type': 'calibration_status',
            'arm': arm,
            'status': status,
        })

    def _get_current_ticks(self, arm: str) -> Optional[Dict[str, int]]:
        """現在のtick値を取得 (servo_detailから)"""
        if arm not in self.arms:
            return None
        detail = self.arms[arm].servo_detail
        if not detail:
            return None
        ticks = detail.get('ticks', [])
        joint_names = detail.get('joint_names', [])
        if len(ticks) != len(joint_names):
            return None
        return {name: tick for name, tick in zip(joint_names, ticks)}

    def calib_set_home(self, arm: str, robot_id: str) -> Dict[str, Any]:
        """
        現在位置をホームポジション(中央=2047)として設定
        homing_offset = 2047 - current_tick で計算
        """
        ticks = self._get_current_ticks(arm)
        if not ticks:
            return {'success': False, 'error': 'No tick data available'}

        # STS3215の中央値 (12bit = 4096, 中央 = 2048)
        HALF_TURN = 2048
        homing_offsets = {}
        for name, tick in ticks.items():
            homing_offsets[name] = HALF_TURN - tick

        with self._calib_lock:
            self._calib_homing_offsets[arm] = homing_offsets

        self.get_logger().info(f'[{arm}] Home set: {homing_offsets}')
        self._set_calib_status(arm, {
            'state': 'home_set',
            'message': f'Home position set (offsets: {list(homing_offsets.values())})',
            'homing_offsets': homing_offsets,
        })
        return {'success': True, 'homing_offsets': homing_offsets}

    def calib_start_recording(self, arm: str) -> Dict[str, Any]:
        """Range記録を開始"""
        ticks = self._get_current_ticks(arm)
        if not ticks:
            return {'success': False, 'error': 'No tick data available'}

        with self._calib_lock:
            self._calib_recording[arm] = True
            self._calib_mins[arm] = ticks.copy()
            self._calib_maxs[arm] = ticks.copy()

        self.get_logger().info(f'[{arm}] Range recording started')
        self._set_calib_status(arm, {
            'state': 'recording',
            'message': 'Move joints through full range, then stop recording',
        })
        return {'success': True}

    def calib_update_range(self, arm: str) -> None:
        """Range記録中に呼ばれ、min/maxを更新し、リアルタイムで配信"""
        if not self._calib_recording.get(arm):
            return
        ticks = self._get_current_ticks(arm)
        if not ticks:
            return

        changed = False
        with self._calib_lock:
            for name, tick in ticks.items():
                if name in self._calib_mins[arm]:
                    old_min = self._calib_mins[arm][name]
                    new_min = min(old_min, tick)
                    if new_min != old_min:
                        self._calib_mins[arm][name] = new_min
                        changed = True
                if name in self._calib_maxs[arm]:
                    old_max = self._calib_maxs[arm][name]
                    new_max = max(old_max, tick)
                    if new_max != old_max:
                        self._calib_maxs[arm][name] = new_max
                        changed = True

        # 変更があった場合、または定期的にリアルタイム配信 (min/max更新をUIに反映)
        now = time.time()
        last_broadcast = getattr(self, '_calib_last_broadcast', {}).get(arm, 0)
        if changed or (now - last_broadcast > 0.5):  # 500ms間隔でも配信
            if not hasattr(self, '_calib_last_broadcast'):
                self._calib_last_broadcast = {}
            self._calib_last_broadcast[arm] = now
            with self._calib_lock:
                mins = self._calib_mins.get(arm, {}).copy()
                maxs = self._calib_maxs.get(arm, {}).copy()
            self._broadcast_ws_message({
                'type': 'calib_recording_update',
                'arm': arm,
                'mins': mins,
                'maxs': maxs,
                'ticks': ticks,
            })

    def calib_stop_recording(self, arm: str) -> Dict[str, Any]:
        """Range記録を停止"""
        with self._calib_lock:
            self._calib_recording[arm] = False
            mins = self._calib_mins.get(arm, {})
            maxs = self._calib_maxs.get(arm, {})

        self.get_logger().info(f'[{arm}] Range recording stopped: mins={mins}, maxs={maxs}')
        self._set_calib_status(arm, {
            'state': 'recorded',
            'message': f'Range recorded: mins={list(mins.values())}, maxs={list(maxs.values())}',
            'mins': mins,
            'maxs': maxs,
        })
        return {'success': True, 'mins': mins, 'maxs': maxs}

    def calib_save(self, arm: str, robot_id: str) -> Dict[str, Any]:
        """LeRobot互換形式でキャリブレーションを保存"""
        with self._calib_lock:
            homing_offsets = self._calib_homing_offsets.get(arm, {})
            mins = self._calib_mins.get(arm, {})
            maxs = self._calib_maxs.get(arm, {})

        if not homing_offsets:
            return {'success': False, 'error': 'Home not set. Click "Set Home" first.'}
        if not mins or not maxs:
            return {'success': False, 'error': 'Range not recorded. Click "Record Range" first.'}

        # LeRobot MotorCalibration形式
        # joint_names の順序 (SO101標準)
        joint_order = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']

        calibration = {}
        for i, name in enumerate(joint_order):
            if name not in homing_offsets:
                # servo_detailの名前と一致しない場合、インデックスで取得を試みる
                detail = self.arms[arm].servo_detail
                if detail and 'joint_names' in detail:
                    detail_names = detail['joint_names']
                    if i < len(detail_names):
                        actual_name = detail_names[i]
                        homing = homing_offsets.get(actual_name, 0)
                        range_min = mins.get(actual_name, 0)
                        range_max = maxs.get(actual_name, 4095)
                    else:
                        homing, range_min, range_max = 0, 0, 4095
                else:
                    homing, range_min, range_max = 0, 0, 4095
            else:
                homing = homing_offsets[name]
                range_min = mins.get(name, 0)
                range_max = maxs.get(name, 4095)

            calibration[name] = {
                'id': i + 1,
                'drive_mode': 0,
                'homing_offset': homing,
                'range_min': range_min,
                'range_max': range_max,
            }

        # ファイル保存
        calib_file = self.calibration_dir / f'{robot_id}.json'
        try:
            with open(calib_file, 'w') as f:
                json.dump(calibration, f, indent=4)
            self.get_logger().info(f'[{arm}] Calibration saved to {calib_file}')

            # コントロールノードにキャリブレーションリロードを通知
            reload_pub = getattr(self, f'_reload_calib_pub_{arm}', None)
            if reload_pub:
                msg = String()
                msg.data = str(calib_file)
                reload_pub.publish(msg)
                self.get_logger().info(f'[{arm}] Sent reload_calibration: {calib_file}')

            self._set_calib_status(arm, {
                'state': 'saved',
                'message': f'Saved & reloaded: {calib_file}',
                'file': str(calib_file),
            })
            return {'success': True, 'file': str(calib_file), 'calibration': calibration}
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')
            return {'success': False, 'error': str(e)}

    def calib_load(self, arm: str, robot_id: str) -> Dict[str, Any]:
        """LeRobot互換キャリブレーションファイルを読み込み"""
        calib_file = self.calibration_dir / f'{robot_id}.json'
        if not calib_file.exists():
            return {'success': False, 'error': f'File not found: {calib_file}'}

        try:
            with open(calib_file, 'r') as f:
                calibration = json.load(f)
            self.get_logger().info(f'[{arm}] Calibration loaded from {calib_file}')

            # 内部状態を復元
            with self._calib_lock:
                self._calib_homing_offsets[arm] = {
                    name: data['homing_offset'] for name, data in calibration.items()
                }
                self._calib_mins[arm] = {
                    name: data['range_min'] for name, data in calibration.items()
                }
                self._calib_maxs[arm] = {
                    name: data['range_max'] for name, data in calibration.items()
                }

            self._set_calib_status(arm, {
                'state': 'loaded',
                'message': f'Loaded from {calib_file}',
                'file': str(calib_file),
            })
            return {'success': True, 'calibration': calibration, 'file': str(calib_file)}
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration: {e}')
            return {'success': False, 'error': str(e)}

    def start_web_server(self):
        """Webサーバーを開始"""
        if not HAS_AIOHTTP:
            self.get_logger().error('aiohttp not installed. pip install aiohttp')
            return

        def run_server():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            app = web.Application()
            app.router.add_get('/ws', self._ws_handler)
            app.router.add_get('/', self._index_handler)
            app.router.add_static('/static/', self._get_web_dir())

            runner = web.AppRunner(app)
            self.loop.run_until_complete(runner.setup())
            site = web.TCPSite(runner, '0.0.0.0', self.web_port)
            self.loop.run_until_complete(site.start())
            self.get_logger().info(f'WebUI started at http://0.0.0.0:{self.web_port}/')
            self.loop.run_forever()

        self.web_thread = threading.Thread(target=run_server, daemon=True)
        self.web_thread.start()

    def _get_web_dir(self) -> str:
        # Try installed location first
        if HAS_AMENT:
            try:
                pkg_dir = get_package_share_directory('unity_robot_control')
                web_dir = Path(pkg_dir) / 'web'
                if web_dir.exists():
                    return str(web_dir)
            except Exception:
                pass
        # Fallback to source location
        return str(Path(__file__).parent.parent.parent / 'web')

    async def _index_handler(self, request):
        index_path = Path(self._get_web_dir()) / 'index.html'
        if index_path.exists():
            return web.FileResponse(index_path)
        return web.Response(text='WebUI index.html not found', status=404)

    async def _ws_handler(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        with self.ws_lock:
            self.ws_clients.append(ws)
            self.get_logger().info(f'WebSocket client connected. Total: {len(self.ws_clients)}')

        # 初期状態送信
        init_data = {
            'type': 'init',
            'arms': self.arm_namespaces,
            'data': {arm: self._get_arm_status(arm) for arm in self.arm_namespaces},
            'positions': self.list_positions(),
        }
        self.get_logger().info(f'Sending init data: arms={self.arm_namespaces}')
        for arm in self.arm_namespaces:
            status = self._get_arm_status(arm)
            has_data = status.get('joint_states') is not None
            self.get_logger().info(f'  {arm}: has_data={has_data}')
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
                    'type': 'torque_response',
                    'arm': arm,
                    'enabled': enabled,
                    'success': True
                }))

            elif msg_type == 'set_ik':
                arm = msg.get('arm')
                enabled = msg.get('enabled', True)
                self.set_ik_enabled(arm, enabled)
                await ws.send_str(json.dumps({
                    'type': 'ik_response',
                    'arm': arm,
                    'enabled': enabled,
                    'success': True
                }))

            elif msg_type == 'save_position':
                arm = msg.get('arm')
                name = msg.get('name')
                success = self.save_position(arm, name)
                await ws.send_str(json.dumps({
                    'type': 'save_response',
                    'success': success,
                    'positions': self.list_positions()
                }))

            elif msg_type == 'delete_position':
                arm = msg.get('arm')
                name = msg.get('name')
                success = self.delete_position(arm, name)
                await ws.send_str(json.dumps({
                    'type': 'delete_response',
                    'success': success,
                    'positions': self.list_positions()
                }))

            elif msg_type == 'load_position':
                arm = msg.get('arm')
                name = msg.get('name')
                data = self.load_position(arm, name)
                await ws.send_str(json.dumps({
                    'type': 'load_response',
                    'arm': arm,
                    'name': name,
                    'data': data,
                    'success': data is not None
                }))

            elif msg_type == 'list_positions':
                await ws.send_str(json.dumps({
                    'type': 'positions_list',
                    'positions': self.list_positions()
                }))

            elif msg_type == 'get_status':
                arm = msg.get('arm')
                if arm:
                    await ws.send_str(json.dumps({
                        'type': 'arm_status',
                        'arm': arm,
                        'data': self._get_arm_status(arm)
                    }))
                else:
                    await ws.send_str(json.dumps({
                        'type': 'all_status',
                        'data': {arm: self._get_arm_status(arm) for arm in self.arm_namespaces}
                    }))
            # ========== LeRobot互換キャリブレーションコマンド ==========
            elif msg_type == 'calib_set_home':
                arm = msg.get('arm')
                robot_id = msg.get('robot_id', arm)
                result = self.calib_set_home(arm, robot_id)
                await ws.send_str(json.dumps({
                    'type': 'calib_set_home_response',
                    'arm': arm,
                    **result
                }))

            elif msg_type == 'calib_start_recording':
                arm = msg.get('arm')
                result = self.calib_start_recording(arm)
                await ws.send_str(json.dumps({
                    'type': 'calib_start_recording_response',
                    'arm': arm,
                    **result
                }))

            elif msg_type == 'calib_stop_recording':
                arm = msg.get('arm')
                result = self.calib_stop_recording(arm)
                await ws.send_str(json.dumps({
                    'type': 'calib_stop_recording_response',
                    'arm': arm,
                    **result
                }))

            elif msg_type == 'calib_save':
                arm = msg.get('arm')
                robot_id = msg.get('robot_id', arm)
                result = self.calib_save(arm, robot_id)
                await ws.send_str(json.dumps({
                    'type': 'calib_save_response',
                    'arm': arm,
                    **result
                }))

            elif msg_type == 'calib_load':
                arm = msg.get('arm')
                robot_id = msg.get('robot_id', arm)
                result = self.calib_load(arm, robot_id)
                await ws.send_str(json.dumps({
                    'type': 'calib_load_response',
                    'arm': arm,
                    **result
                }))

        except json.JSONDecodeError:
            await ws.send_str(json.dumps({'type': 'error', 'message': 'Invalid JSON'}))
        except Exception as e:
            await ws.send_str(json.dumps({'type': 'error', 'message': str(e)}))


def main(args=None):
    if not HAS_AIOHTTP:
        print('ERROR: aiohttp not installed. Run: pip install aiohttp')
        return

    rclpy.init(args=args)
    node = SO101WebUINode()
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
