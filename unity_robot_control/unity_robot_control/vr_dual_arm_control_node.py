#!/usr/bin/env python3
"""
VR Dual Arm Control Node
VR両手テレオペレーション制御ノード

VR（Quest 3）からの左右手ポーズを受信し、左右アームに直接マッピング。
VRがリーダー（教示者）として機能し、両アームは独立してVRに追従する。

設定ファイルでアーム種類（SO101, PIPER, DAIHENなど）を切り替え可能。
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger

# vr_haptic_msgsはオプション（ビルドされていない場合はグリッパー制御を無効化）
HAS_HAPTIC_MSGS = False
HandGesture = None
try:
    from vr_haptic_msgs.msg import HandGesture
    HandGesture.__class__.__import_type_support__()
    HAS_HAPTIC_MSGS = True
except (ImportError, Exception):
    HAS_HAPTIC_MSGS = False
    HandGesture = None


class OneEuroFilter:
    """One Euro Filter — VR入力のジッター除去に最適な適応フィルタ
    静止時は強くスムーズ化、素早い動きでは遅延を最小化する。
    参考: https://cristal.univ-lille.fr/~casiez/1euro/
    """

    def __init__(self, min_cutoff=0.8, beta=5.0, d_cutoff=0.9):
        self.min_cutoff = min_cutoff  # 最低カットオフ周波数(Hz) — 小さいほどスムーズ
        self.beta = beta              # 速度係数 — 大きいほど速い動きへの追従が良い
        self.d_cutoff = d_cutoff      # 微分のカットオフ周波数(Hz)
        self._x_prev = None
        self._dx_prev = 0.0
        self._t_prev = None

    def reset(self):
        self._x_prev = None
        self._dx_prev = 0.0
        self._t_prev = None

    def __call__(self, x, t=None):
        if t is None:
            t = time.monotonic()
        if self._t_prev is None or self._x_prev is None:
            self._x_prev = x
            self._t_prev = t
            return x

        dt = t - self._t_prev
        if dt <= 0.0:
            return self._x_prev
        self._t_prev = t

        # 微分の推定
        dx = (x - self._x_prev) / dt
        alpha_d = self._alpha(self.d_cutoff, dt)
        self._dx_prev = alpha_d * dx + (1.0 - alpha_d) * self._dx_prev

        # 適応カットオフ周波数（速い動き→高カットオフ→少ないスムーズ化）
        cutoff = self.min_cutoff + self.beta * abs(self._dx_prev)
        alpha = self._alpha(cutoff, dt)

        self._x_prev = alpha * x + (1.0 - alpha) * self._x_prev
        return self._x_prev

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)


class ArmState:
    """片腕の状態管理"""
    def __init__(self, name: str):
        self.name = name
        self.origin = None
        self.last_pose_time = None
        self.last_publish_time = None
        self.publish_rate = None
        self.grip_pressed = False
        self.last_trigger = None
        # One-Euro Filter（init_filtersで初期化）
        self.filter_x = None
        self.filter_y = None
        self.filter_z = None

    def init_filters(self, min_cutoff=0.8, beta=5.0, d_cutoff=0.9):
        self.filter_x = OneEuroFilter(min_cutoff, beta, d_cutoff)
        self.filter_y = OneEuroFilter(min_cutoff, beta, d_cutoff)
        self.filter_z = OneEuroFilter(min_cutoff, beta, d_cutoff)

    def reset_filters(self):
        if self.filter_x is not None:
            self.filter_x.reset()
            self.filter_y.reset()
            self.filter_z.reset()


class VRDualArmControlNode(Node):
    """
    VR両手制御ノード

    VR（Quest 3）がリーダーとして機能し、左右のアームを独立して制御する。
    設定ファイルでトピック名やアーム種類を柔軟に変更可能。
    """

    def __init__(self):
        super().__init__('vr_dual_arm_control_node')
        self.get_logger().info('VR Dual Arm Control Node starting...')

        # パラメータ宣言
        self._declare_parameters()

        # パラメータ取得
        self._load_parameters()

        # 状態管理
        self._left = ArmState('left')
        self._right = ArmState('right')
        if self.enable_smoothing:
            self._left.init_filters(self.smoothing_min_cutoff, self.smoothing_beta, self.smoothing_d_cutoff)
            self._right.init_filters(self.smoothing_min_cutoff, self.smoothing_beta, self.smoothing_d_cutoff)
        self._last_pose_log_time = 0.0
        self._last_clamp_log_time = 0.0
        # ボタン状態（左右独立）
        self._last_reset_left_state = 0
        self._last_reset_right_state = 0
        self._last_home_left_state = 0
        self._last_home_right_state = 0
        # 緊急リセット用
        self._emergency_hold_start = None

        # ログ出力
        self._log_configuration()

        # サービス
        if self.enable_relative_mode:
            self.create_service(Trigger, 'reset_origin', self.reset_origin_callback)
            self.get_logger().info('Origin reset service: /vr_dual_arm_control_node/reset_origin')

        # Subscriber/Publisher作成
        self._setup_subscriptions()
        self._setup_publishers()

        # grip_to_move無効時は起動時に両アームを有効化
        if not self.enable_grip_to_move:
            self._enable_arms_on_startup()

        self.get_logger().info('VR Dual Arm Control Node ready')

    def _enable_arms_on_startup(self):
        """起動時に両アームを有効化（grip_to_move無効時）"""
        # 少し遅延してPublisher接続を待つ（1回だけ実行）
        self._startup_timer = self.create_timer(0.5, self._send_initial_enable)

    def _declare_parameters(self):
        """パラメータ宣言"""
        # === VR入力トピック ===
        self.declare_parameter('left_hand_topic', '/quest/left_controller/pose')
        self.declare_parameter('right_hand_topic', '/quest/right_controller/pose')
        self.declare_parameter('joystick_topic', '/quest/joystick')
        self.declare_parameter('left_gesture_topic', '/quest/hand_gesture/left')
        self.declare_parameter('right_gesture_topic', '/quest/hand_gesture/right')

        # === アーム出力トピック ===
        self.declare_parameter('left_arm_target_topic', '/left_arm/target_pose')
        self.declare_parameter('right_arm_target_topic', '/right_arm/target_pose')
        self.declare_parameter('left_arm_enable_topic', '/left_arm/enable_flag')
        self.declare_parameter('right_arm_enable_topic', '/right_arm/enable_flag')
        self.declare_parameter('gripper_left_topic', '/left_arm/gripper_cmd')
        self.declare_parameter('gripper_right_topic', '/right_arm/gripper_cmd')

        # === レガシー（後方互換性） ===
        self.declare_parameter('enable_single_arm_mode', False)
        self.declare_parameter('legacy_target_pose_topic', '/target_pose')

        # === 位置スケールとオフセット ===
        self.declare_parameter('position_scale', 1.0)
        self.declare_parameter('left_arm_offset_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('right_arm_offset_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('enable_pose_offset', True)

        # === 座標変換と安全制限 ===
        self.declare_parameter('enable_axis_remap', False)
        self.declare_parameter('axis_map', ['x', 'y', 'z'])
        self.declare_parameter('enable_position_clamp', False)
        self.declare_parameter('position_min_xyz', [-0.5, -0.5, 0.0])
        self.declare_parameter('position_max_xyz', [0.5, 0.5, 0.6])

        # === 相対モード ===
        self.declare_parameter('enable_relative_mode', True)
        self.declare_parameter('enable_auto_origin_reset', True)
        self.declare_parameter('auto_reset_timeout_sec', 2.0)

        # === グリッパー制御 ===
        self.declare_parameter('enable_gripper_control', True)
        self.declare_parameter('enable_gripper_with_trigger', True)
        self.declare_parameter('gripper_left_axis', 4)
        self.declare_parameter('gripper_right_axis', 5)
        self.declare_parameter('gripper_deadband', 0.02)

        # === ボタン制御 ===
        self.declare_parameter('enable_grip_to_move', True)
        self.declare_parameter('grip_left_button_index', 10)   # 左グリップ
        self.declare_parameter('grip_right_button_index', 11)  # 右グリップ
        # 原点リセット（左右独立）
        self.declare_parameter('enable_reset_with_joystick', True)
        self.declare_parameter('reset_left_button_index', 2)   # Xボタン(左)
        self.declare_parameter('reset_right_button_index', 0)  # Aボタン(右)
        # ホーム復帰（左右独立）
        self.declare_parameter('enable_home_button', True)
        self.declare_parameter('home_left_button_index', 3)    # Yボタン(左)
        self.declare_parameter('home_right_button_index', 1)   # Bボタン(右)
        # 緊急全リセット（両スティック長押し）
        self.declare_parameter('enable_emergency_reset', True)
        self.declare_parameter('left_stick_button_index', 8)
        self.declare_parameter('right_stick_button_index', 9)
        self.declare_parameter('emergency_reset_hold_sec', 1.0)  # 長押し時間

        # === ホーム復帰サービス ===
        self.declare_parameter('left_arm_go_home_service', '/left_arm/go_home')
        self.declare_parameter('right_arm_go_home_service', '/right_arm/go_home')

        # === スムーズフィルタ (One-Euro Filter) ===
        self.declare_parameter('enable_smoothing', True)
        self.declare_parameter('smoothing_min_cutoff', 0.8)  # 最低カットオフ(Hz): 小さい=よりスムーズ
        self.declare_parameter('smoothing_beta', 5.0)         # 速度係数: 大きい=速い動きの追従が良い
        self.declare_parameter('smoothing_d_cutoff', 0.9)     # 微分カットオフ(Hz)

        # === デバッグ ===
        self.declare_parameter('enable_debug_pose_logs', True)

    def _load_parameters(self):
        """パラメータ取得"""
        # VR入力トピック
        self.left_hand_topic = self.get_parameter('left_hand_topic').value
        self.right_hand_topic = self.get_parameter('right_hand_topic').value
        self.joystick_topic = self.get_parameter('joystick_topic').value
        self.left_gesture_topic = self.get_parameter('left_gesture_topic').value
        self.right_gesture_topic = self.get_parameter('right_gesture_topic').value

        # アーム出力トピック
        self.left_arm_target_topic = self.get_parameter('left_arm_target_topic').value
        self.right_arm_target_topic = self.get_parameter('right_arm_target_topic').value
        self.left_arm_enable_topic = self.get_parameter('left_arm_enable_topic').value
        self.right_arm_enable_topic = self.get_parameter('right_arm_enable_topic').value
        self.gripper_left_topic = self.get_parameter('gripper_left_topic').value
        self.gripper_right_topic = self.get_parameter('gripper_right_topic').value

        # レガシー
        self.enable_single_arm_mode = self.get_parameter('enable_single_arm_mode').value
        self.legacy_target_pose_topic = self.get_parameter('legacy_target_pose_topic').value

        # 位置・オフセット
        self.position_scale = float(self.get_parameter('position_scale').value)
        self.left_arm_offset = self.get_parameter('left_arm_offset_xyz').value
        self.right_arm_offset = self.get_parameter('right_arm_offset_xyz').value
        self.enable_pose_offset = self.get_parameter('enable_pose_offset').value

        # 座標変換・安全制限
        self.enable_axis_remap = self.get_parameter('enable_axis_remap').value
        self.axis_map = self.get_parameter('axis_map').value
        self.enable_position_clamp = self.get_parameter('enable_position_clamp').value
        self.position_min = self.get_parameter('position_min_xyz').value
        self.position_max = self.get_parameter('position_max_xyz').value

        # 相対モード
        self.enable_relative_mode = self.get_parameter('enable_relative_mode').value
        self.enable_auto_origin_reset = self.get_parameter('enable_auto_origin_reset').value
        self.auto_reset_timeout_sec = float(self.get_parameter('auto_reset_timeout_sec').value)

        # グリッパー
        self.enable_gripper = self.get_parameter('enable_gripper_control').value
        self.enable_gripper_with_trigger = self.get_parameter('enable_gripper_with_trigger').value
        self.gripper_left_axis = int(self.get_parameter('gripper_left_axis').value)
        self.gripper_right_axis = int(self.get_parameter('gripper_right_axis').value)
        self.gripper_deadband = float(self.get_parameter('gripper_deadband').value)

        # ボタン
        self.enable_grip_to_move = self.get_parameter('enable_grip_to_move').value
        self.grip_left_button_index = int(self.get_parameter('grip_left_button_index').value)
        self.grip_right_button_index = int(self.get_parameter('grip_right_button_index').value)
        self.enable_reset_with_joystick = self.get_parameter('enable_reset_with_joystick').value
        self.reset_left_button_index = int(self.get_parameter('reset_left_button_index').value)
        self.reset_right_button_index = int(self.get_parameter('reset_right_button_index').value)
        self.enable_home_button = self.get_parameter('enable_home_button').value
        self.home_left_button_index = int(self.get_parameter('home_left_button_index').value)
        self.home_right_button_index = int(self.get_parameter('home_right_button_index').value)
        # 緊急リセット
        self.enable_emergency_reset = self.get_parameter('enable_emergency_reset').value
        self.left_stick_button_index = int(self.get_parameter('left_stick_button_index').value)
        self.right_stick_button_index = int(self.get_parameter('right_stick_button_index').value)
        self.emergency_reset_hold_sec = float(self.get_parameter('emergency_reset_hold_sec').value)

        # ホーム復帰サービス
        self.left_arm_go_home_service = self.get_parameter('left_arm_go_home_service').value
        self.right_arm_go_home_service = self.get_parameter('right_arm_go_home_service').value

        # スムーズフィルタ (One-Euro Filter)
        self.enable_smoothing = self.get_parameter('enable_smoothing').value
        self.smoothing_min_cutoff = float(self.get_parameter('smoothing_min_cutoff').value)
        self.smoothing_beta = float(self.get_parameter('smoothing_beta').value)
        self.smoothing_d_cutoff = float(self.get_parameter('smoothing_d_cutoff').value)

        # デバッグ
        self.enable_debug_pose_logs = self.get_parameter('enable_debug_pose_logs').value

    def _log_configuration(self):
        """設定内容をログ出力"""
        self.get_logger().info(f'VR Input: left={self.left_hand_topic}, right={self.right_hand_topic}')
        self.get_logger().info(f'Arm Output: left={self.left_arm_target_topic}, right={self.right_arm_target_topic}')
        self.get_logger().info(
            f'Pose offset: enable={self.enable_pose_offset}, scale={self.position_scale}, '
            f'left_offset={self.left_arm_offset}, right_offset={self.right_arm_offset}'
        )
        self.get_logger().info(
            f'Axis remap: enable={self.enable_axis_remap}, map={self.axis_map}; '
            f'Position clamp: enable={self.enable_position_clamp}'
        )
        if self.enable_smoothing:
            self.get_logger().info(
                f'One-Euro Filter enabled (min_cutoff={self.smoothing_min_cutoff}, '
                f'beta={self.smoothing_beta}, d_cutoff={self.smoothing_d_cutoff})'
            )
        if self.enable_relative_mode:
            self.get_logger().info(f'Relative mode enabled (auto_reset={self.enable_auto_origin_reset})')
        if self.enable_grip_to_move:
            self.get_logger().info(
                f'Grip-to-move enabled (L:{self.grip_left_button_index}, R:{self.grip_right_button_index})'
            )

    def _setup_subscriptions(self):
        """Subscriber作成"""
        # QoS: 最新のメッセージのみ使用（キュー溜まり防止）
        realtime_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # VR手ポーズ
        self.left_hand_sub = self.create_subscription(
            PoseStamped, self.left_hand_topic, self.left_hand_callback, realtime_qos
        )
        self.right_hand_sub = self.create_subscription(
            PoseStamped, self.right_hand_topic, self.right_hand_callback, realtime_qos
        )

        # Joystick（グリップ、リセット、ホーム、グリッパー）
        need_joystick = (self.enable_grip_to_move or self.enable_reset_with_joystick or
                         self.enable_home_button or self.enable_gripper_with_trigger)
        if need_joystick:
            self.joystick_sub = self.create_subscription(
                Joy, self.joystick_topic, self.joystick_callback, 10
            )
            self.get_logger().info(f'Joystick subscription: {self.joystick_topic}')

        # レガシー /target_pose
        if self.enable_single_arm_mode:
            self.target_pose_sub = self.create_subscription(
                PoseStamped, self.legacy_target_pose_topic, self.target_pose_callback, 10
            )
            self.get_logger().info(f'Legacy mode enabled: {self.legacy_target_pose_topic}')

        # ジェスチャー（vr_haptic_msgs必要）
        if self.enable_gripper and HAS_HAPTIC_MSGS:
            self.left_gesture_sub = self.create_subscription(
                HandGesture, self.left_gesture_topic, self.left_gesture_callback, 10
            )
            self.right_gesture_sub = self.create_subscription(
                HandGesture, self.right_gesture_topic, self.right_gesture_callback, 10
            )
            self.get_logger().info('Gesture gripper control enabled')
        elif self.enable_gripper and not HAS_HAPTIC_MSGS:
            self.get_logger().warn('vr_haptic_msgs not found - gesture gripper disabled')

    def _setup_publishers(self):
        """Publisher作成"""
        # ターゲットポーズ
        self.left_arm_target_pub = self.create_publisher(PoseStamped, self.left_arm_target_topic, 10)
        self.right_arm_target_pub = self.create_publisher(PoseStamped, self.right_arm_target_topic, 10)

        # Enable flag（トルク制御）
        self.left_enable_pub = self.create_publisher(Bool, self.left_arm_enable_topic, 1)
        self.right_enable_pub = self.create_publisher(Bool, self.right_arm_enable_topic, 1)

        # グリッパー
        if self.enable_gripper_with_trigger:
            self.gripper_left_pub = self.create_publisher(Float32, self.gripper_left_topic, 10)
            self.gripper_right_pub = self.create_publisher(Float32, self.gripper_right_topic, 10)
            self.get_logger().info(
                f'Gripper trigger: L axis {self.gripper_left_axis}, R axis {self.gripper_right_axis}'
            )

        # ホーム復帰サービスクライアント
        if self.enable_home_button:
            self.left_go_home_client = self.create_client(Trigger, self.left_arm_go_home_service)
            self.right_go_home_client = self.create_client(Trigger, self.right_arm_go_home_service)
            self.get_logger().info(
                f'Home button enabled (L:Y={self.home_left_button_index}, R:B={self.home_right_button_index}): '
                f'L={self.left_arm_go_home_service}, R={self.right_arm_go_home_service}'
            )

    # =========================================================================
    # コールバック
    # =========================================================================

    def left_hand_callback(self, msg: PoseStamped):
        """VR左手ポーズ受信 → 左アームに送信"""
        if self.enable_grip_to_move and not self._left.grip_pressed:
            return
        self._process_hand_pose(msg, self._left, self.left_arm_offset, self.left_arm_target_pub)

    def right_hand_callback(self, msg: PoseStamped):
        """VR右手ポーズ受信 → 右アームに送信"""
        if self.enable_grip_to_move and not self._right.grip_pressed:
            return
        self._process_hand_pose(msg, self._right, self.right_arm_offset, self.right_arm_target_pub)

    def _process_hand_pose(self, msg: PoseStamped, arm: ArmState, offset, publisher):
        """手ポーズを処理してアームに送信"""
        target_msg = PoseStamped()
        target_msg.header = msg.header
        target_msg.header.frame_id = ''  # IKソルバーでTF変換をスキップ

        # 値をコピー（参照ではなく）
        target_msg.pose.position.x = msg.pose.position.x
        target_msg.pose.position.y = msg.pose.position.y
        target_msg.pose.position.z = msg.pose.position.z
        target_msg.pose.orientation.x = msg.pose.orientation.x
        target_msg.pose.orientation.y = msg.pose.orientation.y
        target_msg.pose.orientation.z = msg.pose.orientation.z
        target_msg.pose.orientation.w = msg.pose.orientation.w

        # 相対モード処理
        if self.enable_relative_mode:
            self._maybe_auto_reset_origin(arm, msg.pose.position)
            target_msg.pose.position.x -= arm.origin.x
            target_msg.pose.position.y -= arm.origin.y
            target_msg.pose.position.z -= arm.origin.z

        # フィルタ・オフセット・スケール・クランプ適用
        self._apply_pose_offset(target_msg, offset, arm)

        publisher.publish(target_msg)
        self._update_publish_rate(arm)
        self._maybe_log_pose(arm.name, msg, target_msg, arm.publish_rate)

    def target_pose_callback(self, msg: PoseStamped):
        """既存の/target_poseトピック（後方互換性）→ 左アームに送信"""
        self.left_arm_target_pub.publish(msg)

    def left_gesture_callback(self, msg):
        """VR左手ジェスチャー → 左アームグリッパー"""
        self.get_logger().debug(f'Left gesture: {msg.name}')

    def right_gesture_callback(self, msg):
        """VR右手ジェスチャー → 右アームグリッパー"""
        self.get_logger().debug(f'Right gesture: {msg.name}')

    def joystick_callback(self, msg: Joy):
        """コントローラボタン処理"""
        # デバッグ: 初回でボタン/軸の数を表示
        if not hasattr(self, '_joystick_info_logged'):
            self._joystick_info_logged = True
            self.get_logger().info(f'Joystick: {len(msg.buttons)} buttons, {len(msg.axes)} axes')

        if not msg.buttons:
            return

        # グリップボタン処理
        if self.enable_grip_to_move:
            self._handle_grip_buttons(msg)

        # リセットボタン処理（左右独立）
        if self.enable_reset_with_joystick:
            self._handle_reset_buttons(msg)

        # ホームボタン処理（左右独立）
        if self.enable_home_button:
            self._handle_home_buttons(msg)

        # 緊急全リセット（両スティック長押し）
        if self.enable_emergency_reset:
            self._handle_emergency_reset(msg)

        # グリッパー（トリガー）処理
        if self.enable_gripper_with_trigger and msg.axes:
            self._handle_gripper_trigger(msg)

    def _handle_grip_buttons(self, msg: Joy):
        """グリップボタン状態更新"""
        left_prev = self._left.grip_pressed
        right_prev = self._right.grip_pressed

        if self.grip_left_button_index < len(msg.buttons):
            self._left.grip_pressed = bool(msg.buttons[self.grip_left_button_index])
        if self.grip_right_button_index < len(msg.buttons):
            self._right.grip_pressed = bool(msg.buttons[self.grip_right_button_index])

        # 押下/解放時の処理
        if self._left.grip_pressed and not left_prev:
            self.get_logger().info('Left grip pressed - arm control enabled')
            self._left.origin = None
            self._left.reset_filters()
            self.left_enable_pub.publish(Bool(data=True))
        elif not self._left.grip_pressed and left_prev:
            self.get_logger().info('Left grip released - arm control disabled')

        if self._right.grip_pressed and not right_prev:
            self.get_logger().info('Right grip pressed - arm control enabled')
            self._right.origin = None
            self._right.reset_filters()
            self.right_enable_pub.publish(Bool(data=True))
        elif not self._right.grip_pressed and right_prev:
            self.get_logger().info('Right grip released - arm control disabled')

    def _handle_reset_buttons(self, msg: Joy):
        """リセットボタン処理（左右独立）"""
        # 左アーム（Xボタン）
        if self.reset_left_button_index < len(msg.buttons):
            current = int(msg.buttons[self.reset_left_button_index])
            if current == 1 and self._last_reset_left_state == 0:
                self._left.origin = None
                self._left.last_pose_time = None
                self.get_logger().info('Left arm origin reset (X button)')
            self._last_reset_left_state = current

        # 右アーム（Aボタン）
        if self.reset_right_button_index < len(msg.buttons):
            current = int(msg.buttons[self.reset_right_button_index])
            if current == 1 and self._last_reset_right_state == 0:
                self._right.origin = None
                self._right.last_pose_time = None
                self.get_logger().info('Right arm origin reset (A button)')
            self._last_reset_right_state = current

    def _handle_home_buttons(self, msg: Joy):
        """ホームボタン処理（左右独立）"""
        # 左アーム（Yボタン）
        if self.home_left_button_index < len(msg.buttons):
            current = int(msg.buttons[self.home_left_button_index])
            if current == 1 and self._last_home_left_state == 0:
                self._call_go_home_service('left')
                self.get_logger().info('Left arm home (Y button)')
            self._last_home_left_state = current

        # 右アーム（Bボタン）
        if self.home_right_button_index < len(msg.buttons):
            current = int(msg.buttons[self.home_right_button_index])
            if current == 1 and self._last_home_right_state == 0:
                self._call_go_home_service('right')
                self.get_logger().info('Right arm home (B button)')
            self._last_home_right_state = current

    def _handle_emergency_reset(self, msg: Joy):
        """緊急全リセット（両スティック長押し）"""
        left_pressed = (self.left_stick_button_index < len(msg.buttons) and
                        int(msg.buttons[self.left_stick_button_index]) == 1)
        right_pressed = (self.right_stick_button_index < len(msg.buttons) and
                         int(msg.buttons[self.right_stick_button_index]) == 1)

        if left_pressed and right_pressed:
            now = self.get_clock().now().nanoseconds / 1e9
            if self._emergency_hold_start is None:
                self._emergency_hold_start = now
            elif now - self._emergency_hold_start >= self.emergency_reset_hold_sec:
                self._execute_emergency_reset()
                self._emergency_hold_start = None  # リセット後は再度長押し必要
        else:
            self._emergency_hold_start = None

    def _handle_gripper_trigger(self, msg: Joy):
        """グリッパートリガー処理"""
        if self.gripper_left_axis < len(msg.axes):
            value = self._normalize_trigger_value(msg.axes[self.gripper_left_axis])
            if self._left.last_trigger is None or abs(value - self._left.last_trigger) > self.gripper_deadband:
                self.gripper_left_pub.publish(Float32(data=value))
                self._left.last_trigger = value

        if self.gripper_right_axis < len(msg.axes):
            value = self._normalize_trigger_value(msg.axes[self.gripper_right_axis])
            if self._right.last_trigger is None or abs(value - self._right.last_trigger) > self.gripper_deadband:
                self.gripper_right_pub.publish(Float32(data=value))
                self._right.last_trigger = value

    # =========================================================================
    # ユーティリティ
    # =========================================================================

    def _apply_pose_offset(self, target_msg: PoseStamped, offset_xyz, arm: ArmState):
        """座標変換・スムーズフィルタ・スケール・オフセット・安全制限を適用"""
        p = target_msg.pose.position

        if self.enable_axis_remap:
            p.x, p.y, p.z = self._remap_axes(p.x, p.y, p.z)

        # One-Euro Filter（速い動きは追従、静止時はジッター除去）
        if self.enable_smoothing and arm.filter_x is not None:
            t = time.monotonic()
            p.x = arm.filter_x(p.x, t)
            p.y = arm.filter_y(p.y, t)
            p.z = arm.filter_z(p.z, t)

        if self.enable_pose_offset:
            p.x = p.x * self.position_scale + float(offset_xyz[0])
            p.y = p.y * self.position_scale + float(offset_xyz[1])
            p.z = p.z * self.position_scale + float(offset_xyz[2])

        if self.enable_position_clamp:
            before = (p.x, p.y, p.z)
            p.x = min(max(p.x, float(self.position_min[0])), float(self.position_max[0]))
            p.y = min(max(p.y, float(self.position_min[1])), float(self.position_max[1]))
            p.z = min(max(p.z, float(self.position_min[2])), float(self.position_max[2]))
            if before != (p.x, p.y, p.z) and self.enable_debug_pose_logs:
                now_sec = self.get_clock().now().nanoseconds / 1e9
                if now_sec - self._last_clamp_log_time > 1.0:
                    self._last_clamp_log_time = now_sec
                    self.get_logger().info(f'Position clamped: {before} -> ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')

    def _remap_axes(self, x, y, z):
        """指定マップに従って軸を並べ替える"""
        def resolve(axis_name):
            sign = -1.0 if axis_name.startswith('-') else 1.0
            axis = axis_name.lstrip('-')
            if axis == 'x':
                return sign * x
            if axis == 'y':
                return sign * y
            if axis == 'z':
                return sign * z
            return 0.0

        if len(self.axis_map) != 3:
            return x, y, z
        return resolve(self.axis_map[0]), resolve(self.axis_map[1]), resolve(self.axis_map[2])

    def _normalize_trigger_value(self, value: float) -> float:
        """Quest3のトリガー値を0-1に正規化"""
        if value < 0.0:
            value = (value + 1.0) * 0.5
        return max(0.0, min(1.0, float(value)))

    def _maybe_auto_reset_origin(self, arm: ArmState, position):
        """必要なら原点を自動リセット"""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        should_reset = arm.origin is None

        if self.enable_auto_origin_reset and arm.last_pose_time is not None:
            if now_sec - arm.last_pose_time > self.auto_reset_timeout_sec:
                should_reset = True

        if should_reset:
            arm.origin = position
            self.get_logger().info(f'Auto origin reset ({arm.name})')

        arm.last_pose_time = now_sec

    def _update_publish_rate(self, arm: ArmState):
        """パブリッシュレート計算"""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if arm.last_publish_time is not None:
            dt = now_sec - arm.last_publish_time
            if dt > 1e-6:
                arm.publish_rate = 1.0 / dt
        arm.last_publish_time = now_sec

    def _maybe_log_pose(self, label: str, src_msg: PoseStamped, target_msg: PoseStamped, rate):
        """デバッグ用ポーズログ（1Hz）"""
        if not self.enable_debug_pose_logs:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self._last_pose_log_time < 1.0:
            return
        self._last_pose_log_time = now_sec

        sp = src_msg.pose.position
        tp = target_msg.pose.position
        rate_str = f'{rate:.1f}Hz' if rate else 'n/a'
        self.get_logger().info(
            f'[{label}] src=({sp.x:.3f}, {sp.y:.3f}, {sp.z:.3f}) '
            f'-> tgt=({tp.x:.3f}, {tp.y:.3f}, {tp.z:.3f}) rate={rate_str}'
        )

    def _send_initial_enable(self):
        """起動時のenable_flag送信（1回だけ実行）"""
        self.left_enable_pub.publish(Bool(data=True))
        self.right_enable_pub.publish(Bool(data=True))
        self.get_logger().info('Arms enabled on startup (grip_to_move disabled)')
        # タイマーを破棄（1回だけ実行）
        if hasattr(self, '_startup_timer') and self._startup_timer is not None:
            self._startup_timer.cancel()
            self.destroy_timer(self._startup_timer)
            self._startup_timer = None

    def _reset_relative_origin(self):
        """相対モードの原点をリセット"""
        self._left.origin = None
        self._right.origin = None
        self._left.last_pose_time = None
        self._right.last_pose_time = None
        # One-Euro Filterもリセット
        self._left.reset_filters()
        self._right.reset_filters()

    def _call_go_home_service(self, arm: str):
        """指定アームのgo_homeサービスを呼び出す"""
        if arm == 'left' and hasattr(self, 'left_go_home_client'):
            future = self.left_go_home_client.call_async(Trigger.Request())
            future.add_done_callback(lambda f: self._log_home_result('left', f))
            # 左アームのVR原点もリセット
            self._left.origin = None
            self._left.last_pose_time = None
        elif arm == 'right' and hasattr(self, 'right_go_home_client'):
            future = self.right_go_home_client.call_async(Trigger.Request())
            future.add_done_callback(lambda f: self._log_home_result('right', f))
            # 右アームのVR原点もリセット
            self._right.origin = None
            self._right.last_pose_time = None

    def _execute_emergency_reset(self):
        """緊急全リセット実行"""
        self.get_logger().warn('=== EMERGENCY RESET ===')
        # 1. 両アームのgo_homeを呼び出し（エラーリセット + サーボON + ホーム移動）
        if hasattr(self, 'left_go_home_client'):
            future = self.left_go_home_client.call_async(Trigger.Request())
            future.add_done_callback(lambda f: self._log_home_result('left', f))
        if hasattr(self, 'right_go_home_client'):
            future = self.right_go_home_client.call_async(Trigger.Request())
            future.add_done_callback(lambda f: self._log_home_result('right', f))
        # 2. VR原点リセット
        self._reset_relative_origin()
        self.get_logger().warn('Emergency reset complete - both arms returning to home')

    def _log_home_result(self, arm_name: str, future):
        """go_homeサービス結果をログ出力"""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'{arm_name} arm: {result.message}')
            else:
                self.get_logger().warn(f'{arm_name} arm go_home failed: {result.message}')
        except Exception as e:
            self.get_logger().error(f'{arm_name} arm go_home error: {e}')

    def reset_origin_callback(self, _request, response):
        """相対モードの原点をリセットするサービス"""
        self._reset_relative_origin()
        response.success = True
        response.message = 'Relative origin reset'
        self.get_logger().info('Relative origin reset (service)')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VRDualArmControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
