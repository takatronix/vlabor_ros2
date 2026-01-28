#!/usr/bin/env python3
"""
SO101 Robot Control Node
Piper互換I/FでSO101を制御する中間ノード
"""
from typing import Dict, List, Optional

import json
import math
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

try:
    from piper_msgs.srv import Enable
except Exception:  # piper_msgs not available
    Enable = None


class BaseSo101Driver:
    """SO101ドライバ共通I/F（実機/モック共通）"""

    def connect(self) -> None:
        raise NotImplementedError

    def set_enabled(self, enabled: bool) -> bool:
        raise NotImplementedError

    def write_joint_targets(self, joint_positions: List[float]) -> None:
        raise NotImplementedError

    def read_joint_states(self) -> List[float]:
        raise NotImplementedError


class MockSo101Driver(BaseSo101Driver):
    """開発用モックドライバ（実機なしで動作確認用）"""

    def __init__(self, joint_count: int) -> None:
        self._enabled = False
        self._positions = [0.0] * joint_count

    def connect(self) -> None:
        return

    def set_enabled(self, enabled: bool) -> bool:
        self._enabled = enabled
        return True

    def write_joint_targets(self, joint_positions: List[float]) -> None:
        if not self._enabled:
            return
        self._positions = list(joint_positions)

    def read_joint_states(self) -> List[float]:
        return list(self._positions)


class FeetechScservoDriver(BaseSo101Driver):
    """scservo_sdkを使ったFeetechドライバ（STS/SMS系, protocol 0想定）"""

    # STS3215 レジスタアドレス定義
    ADDR_TORQUE_ENABLE = 40
    ADDR_GOAL_POSITION = 42
    ADDR_LOCK = 55  # ロック解除（0でEEPROM書き込み可、エラーリセット）
    ADDR_PRESENT_POSITION = 56
    ADDR_PRESENT_SPEED = 58
    ADDR_PRESENT_LOAD = 60
    ADDR_PRESENT_VOLTAGE = 62
    ADDR_PRESENT_TEMPERATURE = 63
    ADDR_PRESENT_CURRENT = 69

    def __init__(
        self,
        port: str,
        baudrate: int,
        motor_ids: List[int],
        ticks_per_rad: List[float],
        ticks_offset: List[int],
        min_ticks: List[int],
        max_ticks: List[int],
        protocol_version: int,
        node_logger,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.motor_ids = motor_ids
        self.ticks_per_rad = ticks_per_rad
        self.ticks_offset = ticks_offset
        self.min_ticks = min_ticks
        self.max_ticks = max_ticks
        self.protocol_version = protocol_version
        self.logger = node_logger

        import scservo_sdk as scs

        self.scs = scs
        self.port_handler = scs.PortHandler(self.port)
        self.packet_handler = scs.PacketHandler(self.protocol_version)
        self._use_sync_write = hasattr(scs, 'GroupSyncWrite')
        self.addr_torque_enable = self.ADDR_TORQUE_ENABLE
        self.addr_goal_position = self.ADDR_GOAL_POSITION
        self.addr_present_position = self.ADDR_PRESENT_POSITION
        self._last_read_error_log_time = 0.0

    def connect(self) -> None:
        if not self.port_handler.openPort():
            raise RuntimeError(f'Failed to open serial port: {self.port}')
        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f'Failed to set baudrate: {self.baudrate}')
        self.logger.info(f'Feetech port opened: {self.port} @ {self.baudrate}')

    def set_enabled(self, enabled: bool) -> bool:
        value = 1 if enabled else 0
        ok = True
        self.logger.info(f'set_enabled({enabled}): writing torque={value} to {len(self.motor_ids)} motors')
        for motor_id in self.motor_ids:
            comm_result, error = self._write1(self.addr_torque_enable, motor_id, value)
            self.logger.info(f'  motor_id={motor_id}: comm_result={comm_result}, error={error}')
            if comm_result != self.scs.COMM_SUCCESS or error != 0:
                ok = False
        self.logger.info(f'set_enabled result: {ok}')
        return ok

    def clear_errors(self) -> bool:
        """サーボエラーをリセット（トルクOFF→ロック解除→トルクON）"""
        ok = True
        self.logger.info('Clearing servo errors...')
        for motor_id in self.motor_ids:
            # 1. トルクOFF
            self._write1(self.addr_torque_enable, motor_id, 0)
            # 2. ロック解除（エラークリア）
            comm_result, error = self._write1(self.ADDR_LOCK, motor_id, 0)
            if comm_result != self.scs.COMM_SUCCESS or error != 0:
                ok = False
            # 3. ロック
            self._write1(self.ADDR_LOCK, motor_id, 1)
        self.logger.info(f'clear_errors result: {ok}')
        return ok

    def write_joint_targets(self, joint_positions: List[float]) -> None:
        """全モータに目標位置を書き込み（SyncWrite優先、フォールバックあり）"""
        ticks_list = []
        for idx in range(len(self.motor_ids)):
            rad = joint_positions[idx]
            ticks = int(round(rad * self.ticks_per_rad[idx] + self.ticks_offset[idx]))
            ticks = max(self.min_ticks[idx], min(self.max_ticks[idx], ticks))
            ticks_list.append(ticks)

        if self._use_sync_write:
            try:
                sync_write = self.scs.GroupSyncWrite(
                    self.port_handler, self.packet_handler,
                    self.addr_goal_position, 2
                )
                for idx, motor_id in enumerate(self.motor_ids):
                    ticks = ticks_list[idx]
                    param = [ticks & 0xFF, (ticks >> 8) & 0xFF]
                    sync_write.addParam(motor_id, param)
                sync_write.txPacket()
                sync_write.clearParam()
                return
            except Exception:
                self._use_sync_write = False
                self.logger.warn('SyncWrite失敗、個別書き込みにフォールバック')

        for idx, motor_id in enumerate(self.motor_ids):
            self._write2(self.addr_goal_position, motor_id, ticks_list[idx])

    def read_joint_states(self) -> List[float]:
        positions = []
        error_count = 0
        first_error = None
        for idx, motor_id in enumerate(self.motor_ids):
            ticks, comm_result, error = self._read2(self.addr_present_position, motor_id)
            if comm_result != self.scs.COMM_SUCCESS or error != 0:
                error_count += 1
                if first_error is None:
                    first_error = (motor_id, comm_result, error)
                positions.append(0.0)
                continue
            rad = (ticks - self.ticks_offset[idx]) / self.ticks_per_rad[idx]
            positions.append(rad)
        if error_count > 0:
            now = time.time()
            if now - self._last_read_error_log_time > 1.0:
                motor_id, comm_result, error = first_error
                self.logger.warn(
                    'Feetech read failed: %d/%d motors, first motor_id=%d comm_result=%d error=%d'
                    % (error_count, len(self.motor_ids), motor_id, comm_result, error)
                )
                self._last_read_error_log_time = now
        return positions

    def read_joint_states_detailed(self) -> Dict:
        """詳細なジョイント情報を返す（tick値、エラー情報、温度、電圧、負荷、電流含む）"""
        result = {
            'positions': [],
            'ticks': [],
            'motor_ids': list(self.motor_ids),
            'errors': [],
            'comm_results': [],
            'temperatures': [],  # 温度 (°C)
            'voltages': [],      # 電圧 (V)
            'loads': [],         # 負荷 (%)
            'currents': [],      # 電流 (mA)
            'speeds': [],        # 速度
        }
        for idx, motor_id in enumerate(self.motor_ids):
            ticks, comm_result, error = self._read2(self.addr_present_position, motor_id)
            result['ticks'].append(ticks)
            result['comm_results'].append(comm_result)
            result['errors'].append(error)
            if comm_result != self.scs.COMM_SUCCESS or error != 0:
                result['positions'].append(0.0)
            else:
                rad = (ticks - self.ticks_offset[idx]) / self.ticks_per_rad[idx]
                result['positions'].append(rad)

            # 温度 (1バイト)
            temp, _, _ = self._read1(self.ADDR_PRESENT_TEMPERATURE, motor_id)
            result['temperatures'].append(temp)

            # 電圧 (1バイト, 0.1V単位)
            volt_raw, _, _ = self._read1(self.ADDR_PRESENT_VOLTAGE, motor_id)
            result['voltages'].append(volt_raw * 0.1)

            # 負荷 (2バイト, 符号付き)
            load_raw, _, _ = self._read2(self.ADDR_PRESENT_LOAD, motor_id)
            # 負荷は符号付き（bit15: 方向, 0-1023: 値）
            if load_raw >= 0x8000:
                load = -((load_raw & 0x7FFF) / 10.0)
            else:
                load = (load_raw & 0x3FF) / 10.0
            result['loads'].append(load)

            # 電流 (2バイト, 符号付き, 6.5mA単位)
            curr_raw, _, _ = self._read2(self.ADDR_PRESENT_CURRENT, motor_id)
            if curr_raw >= 0x8000:
                current = -((curr_raw & 0x7FFF) * 6.5)
            else:
                current = curr_raw * 6.5
            result['currents'].append(current)

            # 速度 (2バイト)
            speed_raw, _, _ = self._read2(self.ADDR_PRESENT_SPEED, motor_id)
            if speed_raw >= 0x8000:
                speed = -(speed_raw & 0x7FFF)
            else:
                speed = speed_raw
            result['speeds'].append(speed)

        return result

    def _read1(self, address: int, motor_id: int) -> tuple[int, int, int]:
        """1バイト読み込み"""
        result = self.packet_handler.read1ByteTxRx(self.port_handler, motor_id, address)
        if len(result) == 2:
            value, comm_result = result
            error = 0
        else:
            value, comm_result, error = result
        return value, comm_result, error

    def _write1(self, address: int, motor_id: int, value: int) -> tuple[int, int]:
        result = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, address, value)
        if len(result) == 2:
            comm_result, error = result
        else:
            _, comm_result, error = result
        return comm_result, error

    def _write2(self, address: int, motor_id: int, value: int) -> tuple[int, int]:
        result = self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, address, value)
        if len(result) == 2:
            comm_result, error = result
        else:
            _, comm_result, error = result
        return comm_result, error

    def _read2(self, address: int, motor_id: int) -> tuple[int, int, int]:
        result = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, address)
        if len(result) == 2:
            value, comm_result = result
            error = 0
        else:
            value, comm_result, error = result
        return value, comm_result, error


class SO101ControlNode(Node):
    """SO101ロボット制御ノード（Piper互換I/F）"""

    def __init__(self) -> None:
        super().__init__('so101_control_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter(
            'joint_names',
            ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
        )
        self.declare_parameter('publish_rate_hz', 50)
        self.declare_parameter('driver_backend', 'mock')
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('motor_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('ticks_per_rad', [])
        self.declare_parameter('ticks_offset', [])
        self.declare_parameter('min_ticks', [])
        self.declare_parameter('max_ticks', [])
        self.declare_parameter('protocol_version', 0)
        self.declare_parameter('calibration_path', '')
        self.declare_parameter('enable_gripper_cmd', True)
        self.declare_parameter('gripper_cmd_topic', 'gripper_cmd')
        self.declare_parameter('gripper_min', -0.174533)
        self.declare_parameter('gripper_max', 1.74533)
        self.declare_parameter('gripper_scale', 1.0)
        self.declare_parameter('gripper_invert', False)
        self.declare_parameter('urdf_path', '')
        # ホームポジション（rad）- デフォルトは全関節0
        self.declare_parameter('home_position', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # ホーム移動時間（秒）
        self.declare_parameter('home_move_duration', 0.3)
        # 詳細フィードバック（温度・電圧等）の配信レート
        self.declare_parameter('detail_publish_rate_hz', 2)

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().integer_value
        self.driver_backend = self.get_parameter('driver_backend').get_parameter_value().string_value
        self.auto_enable = self.get_parameter('auto_enable').get_parameter_value().bool_value
        self.motor_ids = self.get_parameter('motor_ids').get_parameter_value().integer_array_value
        self.ticks_per_rad = list(self.get_parameter('ticks_per_rad').get_parameter_value().double_array_value)
        self.ticks_offset = list(self.get_parameter('ticks_offset').get_parameter_value().integer_array_value)
        self.min_ticks = list(self.get_parameter('min_ticks').get_parameter_value().integer_array_value)
        self.max_ticks = list(self.get_parameter('max_ticks').get_parameter_value().integer_array_value)
        self.protocol_version = self.get_parameter('protocol_version').get_parameter_value().integer_value
        self.calibration_path = self.get_parameter('calibration_path').get_parameter_value().string_value
        self.enable_gripper_cmd = self.get_parameter('enable_gripper_cmd').get_parameter_value().bool_value
        self.gripper_cmd_topic = self.get_parameter('gripper_cmd_topic').get_parameter_value().string_value
        self.gripper_min = float(self.get_parameter('gripper_min').get_parameter_value().double_value)
        self.gripper_max = float(self.get_parameter('gripper_max').get_parameter_value().double_value)
        self.gripper_scale = float(self.get_parameter('gripper_scale').get_parameter_value().double_value)
        self.gripper_invert = self.get_parameter('gripper_invert').get_parameter_value().bool_value
        self.urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        self.home_position = list(self.get_parameter('home_position').get_parameter_value().double_array_value)
        self.home_move_duration = float(self.get_parameter('home_move_duration').get_parameter_value().double_value)
        self.detail_publish_rate_hz = int(self.get_parameter('detail_publish_rate_hz').get_parameter_value().integer_value)

        self._last_command: Optional[List[float]] = None
        # 記録したポジション（originはデフォルトで全関節0）
        joint_count = len(self.joint_names)
        self._saved_positions: Dict[str, List[float]] = {
            'origin': [0.0] * joint_count
        }
        self._last_feedback: Optional[List[float]] = None
        self._joint_index: Dict[str, int] = {name: idx for idx, name in enumerate(self.joint_names)}
        self._enabled: bool = False  # ティーチングモード用：Falseの時はコマンドをスキップ

        self.driver = self._create_driver()
        self.driver.connect()

        self.joint_states_pub = self.create_publisher(JointState, 'joint_states_single', 1)
        # robot_state_publisher expects joint_states in the same namespace
        self.joint_states_pub_compat = self.create_publisher(JointState, 'joint_states', 1)
        self.joint_states_feedback_pub = self.create_publisher(JointState, 'joint_states_feedback', 1)
        self.joint_ctrl_pub = self.create_publisher(JointState, 'joint_ctrl', 1)
        # 詳細サーボ情報（tick値、エラー情報）をJSON文字列で配信
        self.servo_detail_pub = self.create_publisher(String, 'servo_detail', 1)

        self.create_subscription(JointState, 'joint_ctrl_single', self.joint_command_callback, 1)
        self.create_subscription(Bool, 'enable_flag', self.enable_flag_callback, 1)
        self.create_subscription(Bool, 'ik_enable', self.ik_enable_callback, 1)
        self.create_subscription(String, 'reload_calibration', self.reload_calibration_callback, 1)
        self._ik_enabled: bool = True  # IK有効フラグ（デフォルトは有効）
        if self.enable_gripper_cmd:
            self.create_subscription(Float32, self.gripper_cmd_topic, self.gripper_cmd_callback, 1)
        if Enable is None:
            self.get_logger().warn('enable_srv disabled (piper_msgs not found).')
        else:
            try:
                self.create_service(Enable, 'enable_srv', self.handle_enable_service)
            except Exception as exc:
                self.get_logger().warn(
                    f"enable_srv disabled (piper_msgs typesupport unavailable): {exc}"
                )

        # ホーム復帰サービス
        self.create_service(Trigger, 'go_home', self.handle_go_home_service)
        # 現在位置を記録するトピック（名前を送信）
        self.create_subscription(String, 'save_position', self.save_position_callback, 1)
        # 記録した位置に移動するトピック（名前を送信）
        self.create_subscription(String, 'goto_position', self.goto_position_callback, 1)
        self.get_logger().info('Home/position services enabled: go_home, save_position, goto_position')

        # 位置フィードバック（軽量、高速）
        self.create_timer(1.0 / max(self.publish_rate_hz, 1), self.publish_feedback)
        # 詳細フィードバック（温度・電圧等、低速）
        if self.detail_publish_rate_hz > 0:
            self.create_timer(1.0 / self.detail_publish_rate_hz, self.publish_detail_feedback)

        if self.auto_enable:
            self._enabled = True
            self.driver.set_enabled(True)

        self.get_logger().info('SO101 Control Node ready (Piper compatible I/F).')

    def _create_driver(self) -> BaseSo101Driver:
        if self.driver_backend == 'mock':
            self.get_logger().warn('SO101 driver backend is mock. No hardware control.')
            return MockSo101Driver(len(self.joint_names))

        if self.driver_backend == 'feetech':
            if len(self.motor_ids) != len(self.joint_names):
                self.get_logger().warn('motor_ids length does not match joint_names. Will try calibration if provided.')

            if self.calibration_path:
                self._load_calibration(self.calibration_path)

            default_ticks_per_rad = 4096.0 / (2.0 * math.pi)
            if not self.ticks_per_rad:
                self.ticks_per_rad = [default_ticks_per_rad] * len(self.joint_names)
            if not self.ticks_offset:
                self.ticks_offset = [2048] * len(self.joint_names)
            if not self.min_ticks:
                self.min_ticks = [0] * len(self.joint_names)
            if not self.max_ticks:
                self.max_ticks = [4095] * len(self.joint_names)

            return FeetechScservoDriver(
                port=self.serial_port,
                baudrate=self.baudrate,
                motor_ids=list(self.motor_ids),
                ticks_per_rad=self.ticks_per_rad,
                ticks_offset=self.ticks_offset,
                min_ticks=self.min_ticks,
                max_ticks=self.max_ticks,
                protocol_version=self.protocol_version,
                node_logger=self.get_logger(),
            )

        self.get_logger().error(
            f"SO101 driver backend '{self.driver_backend}' is not implemented. Falling back to mock."
        )
        return MockSo101Driver(len(self.joint_names))

    def _load_calibration(self, calibration_path: str) -> None:
        if not os.path.exists(calibration_path):
            self.get_logger().error(f'Calibration file not found: {calibration_path}')
            return

        try:
            with open(calibration_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().error(f'Failed to load calibration file: {exc}')
            return

        motor_ids: List[int] = []
        ticks_offset: List[int] = []
        min_ticks: List[int] = []
        max_ticks: List[int] = []

        for name in self.joint_names:
            entry = data.get(name)
            if not entry:
                self.get_logger().warn(f'Calibration entry missing for joint: {name}')
                motor_ids.append(1)
                ticks_offset.append(2048)
                min_ticks.append(0)
                max_ticks.append(4095)
                continue

            motor_ids.append(int(entry.get('id', 1)))
            homing_offset = int(entry.get('homing_offset', 0))
            # homing_offset = 2048 - tick_at_home
            # rad=0 のときの tick = 2048 - homing_offset
            ticks_offset.append(2048 - homing_offset)
            min_ticks.append(int(entry.get('range_min', 0)))
            max_ticks.append(int(entry.get('range_max', 4095)))

        self.motor_ids = motor_ids
        self.ticks_offset = ticks_offset
        self.min_ticks = min_ticks
        self.max_ticks = max_ticks

    def enable_flag_callback(self, msg: Bool) -> None:
        new_enabled = bool(msg.data)
        # 状態が変わった時だけドライバーを呼び出す（重複コマンド防止）
        if new_enabled == self._enabled:
            self.get_logger().debug(f'enable_flag received: {msg.data} (no change)')
            return
        self.get_logger().info(f'enable_flag received: {msg.data}')
        self._enabled = new_enabled
        result = self.driver.set_enabled(self._enabled)
        self.get_logger().info(f'driver.set_enabled result: {result}, teaching_mode: {not self._enabled}')

    def ik_enable_callback(self, msg: Bool) -> None:
        self._ik_enabled = bool(msg.data)
        self.get_logger().info(f'ik_enable received: {msg.data}')

    def reload_calibration_callback(self, msg: String) -> None:
        """キャリブレーション設定をリロード（オプションで新しいパスを指定可能）"""
        new_path = msg.data.strip() if msg.data else ''
        if new_path:
            self.calibration_path = new_path

        if not self.calibration_path:
            self.get_logger().warn('reload_calibration: No calibration_path configured')
            return

        self.get_logger().info(f'Reloading calibration from: {self.calibration_path}')
        self._load_calibration(self.calibration_path)

        # ドライバのキャリブレーション値を更新
        if hasattr(self.driver, 'ticks_offset'):
            self.driver.ticks_offset = self.ticks_offset
            self.driver.min_ticks = self.min_ticks
            self.driver.max_ticks = self.max_ticks
            self.get_logger().info(f'Calibration reloaded: ticks_offset={self.ticks_offset}')

    def handle_enable_service(self, request, response):
        response.enable_response = self.driver.set_enabled(bool(request.enable_request))
        return response

    def joint_command_callback(self, msg: JointState) -> None:
        # IK無効の場合はコマンドを無視
        if not self._ik_enabled:
            return
        positions = self._map_joint_positions(msg)
        if positions is None:
            return
        self._last_command = positions
        # デバッグログ（IKコマンド受信）
        self.get_logger().info(
            f'IK CMD received: joints={[f"{p:.2f}" for p in positions]}, enabled={self._enabled}',
            throttle_duration_sec=0.5,
        )
        # ティーチングモード: _enabled=Falseの時はサーボへの書き込みをスキップ
        if self._enabled:
            self.driver.write_joint_targets(positions)
        self._publish_joint_ctrl(positions)

    def _map_joint_positions(self, msg: JointState) -> Optional[List[float]]:
        if not msg.position:
            return None

        if not msg.name:
            if len(msg.position) < len(self.joint_names):
                self.get_logger().warn('JointState position length is shorter than joint_names.')
                return None
            return list(msg.position[: len(self.joint_names)])

        if self._last_command is not None:
            positions = list(self._last_command)
        elif self._last_feedback is not None:
            positions = list(self._last_feedback)
        else:
            positions = [0.0] * len(self.joint_names)
        for name, pos in zip(msg.name, msg.position):
            if name not in self._joint_index:
                continue
            positions[self._joint_index[name]] = pos
        return positions

    def publish_feedback(self) -> None:
        """位置フィードバック（軽量・高速）— 位置レジスタのみ読み取り"""
        positions = self.driver.read_joint_states()

        if not positions:
            return
        self._last_feedback = list(positions)
        now = self.get_clock().now().to_msg()

        joint_states = JointState()
        joint_states.header.stamp = now
        joint_states.name = list(self.joint_names)
        joint_states.position = list(positions)

        joint_feedback = JointState()
        joint_feedback.header.stamp = now
        joint_feedback.name = list(self.joint_names)
        joint_feedback.position = list(positions)

        self.joint_states_pub.publish(joint_states)
        self.joint_states_pub_compat.publish(joint_states)
        self.joint_states_feedback_pub.publish(joint_feedback)

    def publish_detail_feedback(self) -> None:
        """詳細フィードバック（温度・電圧等、低速）— シリアルバス負荷を分散"""
        if not hasattr(self.driver, 'read_joint_states_detailed'):
            return
        detail = self.driver.read_joint_states_detailed()
        detail_msg = String()
        detail_msg.data = json.dumps({
            'joint_names': list(self.joint_names),
            'motor_ids': detail['motor_ids'],
            'positions': detail['positions'],
            'ticks': detail['ticks'],
            'errors': detail['errors'],
            'comm_results': detail['comm_results'],
            'temperatures': detail['temperatures'],
            'voltages': detail['voltages'],
            'loads': detail['loads'],
            'currents': detail['currents'],
            'speeds': detail['speeds'],
            'enabled': self._enabled,
            'ik_enabled': self._ik_enabled,
            'calibration_file': self.calibration_path,
            'urdf_file': self.urdf_path,
            'calibration': {
                'ticks_offset': list(self.ticks_offset),
                'min_ticks': list(self.min_ticks),
                'max_ticks': list(self.max_ticks),
            },
        })
        self.servo_detail_pub.publish(detail_msg)

    def gripper_cmd_callback(self, msg: Float32) -> None:
        if not self.enable_gripper_cmd:
            return
        # ティーチングモード: _enabled=Falseの時はグリッパーコマンドもスキップ
        if not self._enabled:
            return
        value = max(0.0, min(1.0, float(msg.data)))
        if self.gripper_invert:
            value = 1.0 - value
        base_range = (self.gripper_max - self.gripper_min)
        target = self.gripper_min + base_range * value
        if self.gripper_scale != 1.0:
            target = self.gripper_min + (target - self.gripper_min) * self.gripper_scale
            max_target = self.gripper_min + base_range * self.gripper_scale
            if target < self.gripper_min:
                target = self.gripper_min
            elif target > max_target:
                target = max_target

        if self._last_command is not None:
            positions = list(self._last_command)
        elif self._last_feedback is not None:
            positions = list(self._last_feedback)
        else:
            positions = [0.0] * len(self.joint_names)

        idx = self._joint_index.get('gripper')
        if idx is None:
            return
        positions[idx] = target
        self._last_command = list(positions)
        self.driver.write_joint_targets(positions)
        self._publish_joint_ctrl(positions)

    def _publish_joint_ctrl(self, positions: List[float]) -> None:
        joint_ctrl = JointState()
        joint_ctrl.header.stamp = self.get_clock().now().to_msg()
        joint_ctrl.name = list(self.joint_names)
        joint_ctrl.position = list(positions)
        self.joint_ctrl_pub.publish(joint_ctrl)

    def handle_go_home_service(self, request, response):
        """ホームポジションに移動するサービス（エラーリセット・サーボON含む）"""
        try:
            # 1. エラーリセット（実機の場合）
            if hasattr(self.driver, 'clear_errors'):
                self.driver.clear_errors()
                self.get_logger().info('Servo errors cleared')

            # 2. サーボON
            self._enabled = True
            self.driver.set_enabled(True)

            # 3. ホーム移動
            self._go_to_position(self.home_position, 'home')
            response.success = True
            response.message = f'Reset & moving to home: {self.home_position}'
        except Exception as e:
            response.success = False
            response.message = f'Failed to go home: {e}'
        return response

    def save_position_callback(self, msg: String) -> None:
        """現在位置を名前付きで保存"""
        name = msg.data.strip() or 'default'
        if self._last_feedback is None:
            self.get_logger().warn('Cannot save position: no feedback received yet')
            return
        self._saved_positions[name] = list(self._last_feedback)
        self.get_logger().info(f'Position saved: {name} = {self._saved_positions[name]}')

    def goto_position_callback(self, msg: String) -> None:
        """記録した位置に移動"""
        name = msg.data.strip() or 'default'
        if name == 'home':
            self._go_to_position(self.home_position, 'home')
            return
        if name not in self._saved_positions:
            self.get_logger().warn(f'Position not found: {name}')
            return
        self._go_to_position(self._saved_positions[name], name)

    def _go_to_position(self, target_positions: List[float], name: str, duration_sec: float = None) -> None:
        """指定位置に移動（補間付き）

        Args:
            target_positions: 目標関節角度
            name: ポジション名（ログ用）
            duration_sec: 移動にかける時間（秒）、Noneならパラメータ値を使用
        """
        if duration_sec is None:
            duration_sec = self.home_move_duration

        # 既存の補間タイマーがあればキャンセル
        if hasattr(self, '_interpolation_timer') and self._interpolation_timer is not None:
            self._interpolation_timer.cancel()
            self.destroy_timer(self._interpolation_timer)
            self._interpolation_timer = None

        # ホーム位置の長さを調整
        if len(target_positions) < len(self.joint_names):
            target_positions = list(target_positions) + [0.0] * (len(self.joint_names) - len(target_positions))
        elif len(target_positions) > len(self.joint_names):
            target_positions = target_positions[:len(self.joint_names)]

        # トルクON
        if not self._enabled:
            self._enabled = True
            self.driver.set_enabled(True)
            self.get_logger().info('Torque enabled for position move')

        # 現在位置を取得
        if self._last_feedback is not None:
            start_positions = list(self._last_feedback)
        elif self._last_command is not None:
            start_positions = list(self._last_command)
        else:
            # 現在位置不明なら直接移動
            self._last_command = list(target_positions)
            self.driver.write_joint_targets(target_positions)
            self._publish_joint_ctrl(target_positions)
            self.get_logger().info(f'Moving to position (direct): {name}')
            return

        # 補間移動をタイマーで実行
        self._interpolation_start = start_positions
        self._interpolation_target = list(target_positions)
        self._interpolation_name = name
        self._interpolation_duration = duration_sec
        self._interpolation_start_time = self.get_clock().now().nanoseconds / 1e9
        self._interpolation_timer = self.create_timer(0.02, self._interpolation_step)  # 50Hz
        self.get_logger().info(f'Moving to position: {name} (duration={duration_sec}s)')

    def _interpolation_step(self):
        """補間移動の1ステップ"""
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self._interpolation_start_time
        t = min(1.0, elapsed / self._interpolation_duration)

        # 線形補間
        positions = [
            start + (target - start) * t
            for start, target in zip(self._interpolation_start, self._interpolation_target)
        ]

        self._last_command = positions
        self.driver.write_joint_targets(positions)
        self._publish_joint_ctrl(positions)

        # 完了チェック
        if t >= 1.0:
            self._interpolation_timer.cancel()
            self.destroy_timer(self._interpolation_timer)
            self._interpolation_timer = None
            self.get_logger().info(f'Position reached: {self._interpolation_name}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SO101ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
