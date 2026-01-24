#!/usr/bin/env python3
"""
IK Solver Node
SO101ロボットの逆運動学計算ノード（KDL使用）
urdfpyでURDFをパースし、PyKDLでIK計算

VRモードでは左アーム/右アームとして機能する。

改善点:
- IK失敗時でも近似解を出力（動きが止まらない）
- 関節角度変化量の制限（滑らかな動き）
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState
import PyKDL
import os
import subprocess
import tempfile
import time
import numpy as np
from ament_index_python.packages import get_package_share_directory
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped

# urdfpyでURDFパース（kdl_parser_pyの代替）
try:
    from urdfpy import URDF
    HAS_URDFPY = True
except ImportError:
    HAS_URDFPY = False


class IKSolverNode(Node):
    """逆運動学計算ノード（KDL使用）"""

    def __init__(self, node_name='ik_solver_node'):
        super().__init__(node_name)

        self.get_logger().info(f'IK Solver Node ({node_name}) starting...')

        # パラメータ宣言
        self._declare_parameters()

        # パラメータ取得
        self._load_parameters()

        # 状態変数
        self._joint_index = {name: idx for idx, name in enumerate(self.joint_names)}
        self._latest_joint_positions = None
        self._last_output_positions = None  # 前回出力した関節角度（delta limit用）
        self.ik_joint_count = None
        self._last_tf_warn = 0.0
        self._last_ik_warn = 0.0
        self._last_joint_warn = 0.0
        self._has_joint_state = False
        self._ik_stats_window_start = None
        self._ik_stats_last_log_time = 0.0
        self._ik_stats = self._create_empty_stats()

        # TFバッファ
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # KDLの初期化
        self.chain = None
        self.ik_solver = None
        self.fk_solver = None
        self.jac_solver = None
        self.joint_limits = {}

        if self.urdf_file and os.path.exists(self.urdf_file):
            self._init_kdl(self.urdf_file)
        else:
            self.get_logger().warn(f'URDF file not found: {self.urdf_file}')
            self.get_logger().warn('IK solver will not work. Please set urdf_file parameter.')

        # Subscriber/Publisher
        self._setup_subscriptions()
        self._setup_publishers()

        self.get_logger().info('IK Solver Node ready')

    def _declare_parameters(self):
        """パラメータ宣言"""
        self.declare_parameter('urdf_file', '')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tf_base_link', '')
        self.declare_parameter('end_effector_link', 'gripper_link')
        self.declare_parameter('joint_names', ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'])
        self.declare_parameter('target_pose_topic', '/target_pose')
        self.declare_parameter('joint_angles_topic', '/ik/joint_angles')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('use_orientation', True)
        self.declare_parameter('max_reach', 0.0)
        self.declare_parameter('ik_max_iterations', 50)
        self.declare_parameter('ik_tolerance', 1e-3)
        self.declare_parameter('ik_damping', 0.01)
        self.declare_parameter('enable_debug_ik_stats', True)
        self.declare_parameter('ik_stats_log_interval_sec', 1.0)

        # 新パラメータ: IK失敗時の挙動
        self.declare_parameter('use_approximate_on_fail', True)  # 失敗時でも近似解を使う
        self.declare_parameter('approximate_error_threshold', 0.1)  # この誤差以下なら近似解を使う(m)

        # 新パラメータ: 関節角度変化量制限（滑らかさ）
        self.declare_parameter('enable_delta_limit', True)
        self.declare_parameter('max_joint_delta_rad', 0.3)  # 1回の更新での最大変化量(rad) ≈ 17度

    def _load_parameters(self):
        """パラメータ取得"""
        # URDFファイル
        self.urdf_file = self.get_parameter('urdf_file').value
        if not self.urdf_file:
            try:
                pkg_dir = get_package_share_directory('so101_description')
                self.urdf_file = os.path.join(pkg_dir, 'urdf', 'so101_new_calib.urdf.xacro')
            except Exception:
                self.urdf_file = ''

        self.base_link = self.get_parameter('base_link').value
        self.ee_link = self.get_parameter('end_effector_link').value
        self.tf_base_link = self.get_parameter('tf_base_link').value or self.base_link
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.use_orientation = self.get_parameter('use_orientation').value
        self.max_reach = float(self.get_parameter('max_reach').value)
        self.ik_max_iterations = int(self.get_parameter('ik_max_iterations').value)
        self.ik_tolerance = float(self.get_parameter('ik_tolerance').value)
        self.ik_damping = float(self.get_parameter('ik_damping').value)
        self.enable_debug_ik_stats = bool(self.get_parameter('enable_debug_ik_stats').value)
        self.ik_stats_log_interval_sec = float(self.get_parameter('ik_stats_log_interval_sec').value)

        # トピック
        self.target_pose_topic = self.get_parameter('target_pose_topic').value
        self.joint_angles_topic = self.get_parameter('joint_angles_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value

        # IK失敗時の挙動
        self.use_approximate_on_fail = self.get_parameter('use_approximate_on_fail').value
        self.approximate_error_threshold = float(self.get_parameter('approximate_error_threshold').value)

        # 関節角度変化量制限
        self.enable_delta_limit = self.get_parameter('enable_delta_limit').value
        self.max_joint_delta_rad = float(self.get_parameter('max_joint_delta_rad').value)

    def _setup_subscriptions(self):
        """Subscriber作成"""
        # QoS: 最新のメッセージのみ使用
        realtime_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.target_pose_sub = self.create_subscription(
            PoseStamped, self.target_pose_topic, self.target_pose_callback, realtime_qos
        )
        self.joint_states_sub = self.create_subscription(
            JointState, self.joint_states_topic, self.joint_states_callback, 10
        )

        self.get_logger().info(f'Subscribing to: {self.target_pose_topic}')
        self.get_logger().info(f'Joint states (IK init): {self.joint_states_topic}')

    def _setup_publishers(self):
        """Publisher作成"""
        self.joint_angles_pub = self.create_publisher(JointState, self.joint_angles_topic, 10)
        self.get_logger().info(f'Publishing to: {self.joint_angles_topic}')

    def _create_empty_stats(self):
        """統計情報の初期化"""
        return {
            'count': 0,
            'success': 0,
            'approximate': 0,  # 近似解を使った回数
            'fail': 0,
            'skip_no_state': 0,
            'skip_no_solver': 0,
            'skip_tf': 0,
            'time_ms_sum': 0.0,
            'delta_limited': 0,  # delta limitが適用された回数
        }

    # =========================================================================
    # KDL初期化
    # =========================================================================

    def _init_kdl(self, urdf_file):
        """KDLの初期化（urdfpyでパース）"""
        try:
            # urdfpy が numpy の deprecated alias を使うため互換性を付与
            import numpy as np
            if not hasattr(np, 'float'):
                np.float = float
            if not hasattr(np, 'int'):
                np.int = int

            # xacroの場合は処理
            if urdf_file.endswith('.xacro'):
                urdf_content = self._process_xacro(urdf_file)
                if not urdf_content:
                    self.get_logger().error(f'Failed to process xacro: {urdf_file}')
                    return
            else:
                with open(urdf_file, 'r') as f:
                    urdf_content = f.read()

            # package:// URI を実パスに置換
            urdf_content = self._resolve_package_uris(urdf_content)

            if not HAS_URDFPY:
                self.get_logger().error('urdfpy not installed. pip install urdfpy')
                return

            # 一時ファイルに書き出してパース
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp:
                tmp.write(urdf_content)
                tmp_path = tmp.name

            try:
                robot = URDF.load(tmp_path)
            finally:
                os.unlink(tmp_path)

            # URDFからKDLチェーンを構築
            self.chain, self.joint_limits = self._build_kdl_chain(robot)
            if self.chain is None:
                self.get_logger().error('Failed to build KDL chain')
                return

            self.ik_joint_count = self.chain.getNrOfJoints()
            if len(self.joint_names) != self.ik_joint_count:
                self.get_logger().warn(
                    f'joint_names length ({len(self.joint_names)}) != IK joints ({self.ik_joint_count}). '
                    'Trimming to IK joint count.'
                )
                self.joint_names = list(self.joint_names[:self.ik_joint_count])
                self._joint_index = {name: idx for idx, name in enumerate(self.joint_names)}

            # IKソルバーを初期化
            self.ik_solver = PyKDL.ChainIkSolverPos_LMA(self.chain)
            self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
            self.jac_solver = PyKDL.ChainJntToJacSolver(self.chain)

            # 関節制限をログ出力
            if self.joint_limits:
                for name, (lower, upper) in self.joint_limits.items():
                    self.get_logger().info(f'Joint limit: {name} = [{lower:.3f}, {upper:.3f}]')

            self.get_logger().info(f'KDL initialized: {self.base_link} -> {self.ee_link}')
            self.get_logger().info(f'Chain has {self.chain.getNrOfJoints()} joints')
            self.get_logger().info(f'Approximate on fail: {self.use_approximate_on_fail}')
            self.get_logger().info(f'Delta limit: {self.enable_delta_limit} (max {self.max_joint_delta_rad:.2f} rad)')

        except Exception as e:
            self.get_logger().error(f'KDL initialization failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _process_xacro(self, xacro_file):
        """xacroファイルをURDFに変換"""
        try:
            result = subprocess.run(['xacro', xacro_file], capture_output=True, text=True)
            if result.returncode == 0:
                return result.stdout
            else:
                self.get_logger().error(f'xacro error: {result.stderr}')
                return None
        except Exception as e:
            self.get_logger().error(f'xacro failed: {e}')
            return None

    def _resolve_package_uris(self, urdf_content: str) -> str:
        """URDF内の package://<pkg>/ を実パスに置換"""
        import re

        def replace_match(match):
            pkg = match.group(1)
            try:
                pkg_dir = get_package_share_directory(pkg)
            except Exception:
                self.get_logger().warn(f'Package not found for URI: {pkg}')
                return match.group(0)
            return f'{pkg_dir}/'

        return re.sub(r'package://([^/]+)/', replace_match, urdf_content)

    def _build_kdl_chain(self, robot):
        """URDFからKDLチェーンを構築"""
        chain = PyKDL.Chain()
        joint_limits = {}

        link_path = self._find_link_path(robot, self.base_link, self.ee_link)
        if not link_path:
            self.get_logger().error(f'No path from {self.base_link} to {self.ee_link}')
            return None, {}

        self.get_logger().info(f'Link path: {" -> ".join(link_path)}')

        for i in range(len(link_path) - 1):
            parent_link = link_path[i]
            child_link = link_path[i + 1]
            joint = self._find_joint(robot, parent_link, child_link)
            if joint:
                segment = self._joint_to_kdl_segment(joint)
                if segment:
                    chain.addSegment(segment)
                if joint.joint_type in ('revolute', 'continuous') and joint.limit is not None:
                    joint_limits[joint.name] = (joint.limit.lower, joint.limit.upper)

        return chain, joint_limits

    def _find_link_path(self, robot, start_link, end_link):
        """開始リンクから終了リンクまでのパスを見つける"""
        parent_map = {}
        for joint in robot.joints:
            parent_map[joint.child] = (joint.parent, joint)

        path = [end_link]
        current = end_link
        while current != start_link:
            if current not in parent_map:
                return None
            parent, _ = parent_map[current]
            path.append(parent)
            current = parent

        return list(reversed(path))

    def _find_joint(self, robot, parent_link, child_link):
        """2つのリンク間のジョイントを見つける"""
        for joint in robot.joints:
            if joint.parent == parent_link and joint.child == child_link:
                return joint
        return None

    def _joint_to_kdl_segment(self, joint):
        """URDFジョイントをKDLセグメントに変換"""
        origin = joint.origin
        if origin is not None:
            pos = PyKDL.Vector(origin[0, 3], origin[1, 3], origin[2, 3])
            rot = PyKDL.Rotation(
                origin[0, 0], origin[0, 1], origin[0, 2],
                origin[1, 0], origin[1, 1], origin[1, 2],
                origin[2, 0], origin[2, 1], origin[2, 2]
            )
            frame = PyKDL.Frame(rot, pos)
        else:
            frame = PyKDL.Frame()

        if joint.joint_type in ('revolute', 'continuous'):
            axis = joint.axis if joint.axis is not None else [0, 0, 1]
            if sum(v * v for v in axis) < 1e-8:
                axis = [0, 0, 1]
            kdl_joint = PyKDL.Joint(
                joint.name, PyKDL.Vector(0.0, 0.0, 0.0),
                PyKDL.Vector(*axis), PyKDL.Joint.RotAxis
            )
        elif joint.joint_type == 'prismatic':
            axis = joint.axis if joint.axis is not None else [0, 0, 1]
            if sum(v * v for v in axis) < 1e-8:
                axis = [0, 0, 1]
            kdl_joint = PyKDL.Joint(
                joint.name, PyKDL.Vector(0.0, 0.0, 0.0),
                PyKDL.Vector(*axis), PyKDL.Joint.TransAxis
            )
        else:
            kdl_joint = PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)

        return PyKDL.Segment(joint.child, kdl_joint, frame)

    # =========================================================================
    # IK計算
    # =========================================================================

    def target_pose_callback(self, msg: PoseStamped):
        """ターゲットポーズ受信時のコールバック"""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self._ik_stats['count'] += 1

        # 早期リターンチェック
        if self.ik_solver is None:
            self._ik_stats['skip_no_solver'] += 1
            self._maybe_log_ik_stats(now_sec)
            return

        if not self._has_joint_state:
            self._ik_stats['skip_no_state'] += 1
            if now_sec - self._last_joint_warn > 1.0:
                self.get_logger().warn('Waiting for joint states; skipping IK solve')
                self._last_joint_warn = now_sec
            self._maybe_log_ik_stats(now_sec)
            return

        # TF変換
        msg = self._transform_to_base_frame(msg, now_sec)
        if msg is None:
            self._ik_stats['skip_tf'] += 1
            self._maybe_log_ik_stats(now_sec)
            return

        # 位置取得とリーチ制限
        pos = msg.pose.position
        quat = msg.pose.orientation
        if self.max_reach > 0.0:
            dist = (pos.x ** 2 + pos.y ** 2 + pos.z ** 2) ** 0.5
            if dist > self.max_reach:
                scale = self.max_reach / max(dist, 1e-9)
                pos = Point(x=pos.x * scale, y=pos.y * scale, z=pos.z * scale)

        # IK計算
        joint_count = self.ik_joint_count or len(self.joint_names)
        q_init = self._get_initial_joints(joint_count)
        q_out = PyKDL.JntArray(joint_count)

        ik_start = time.perf_counter()
        if self.use_orientation:
            rotation = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
            frame = PyKDL.Frame(rotation, PyKDL.Vector(pos.x, pos.y, pos.z))
            result = self.ik_solver.CartToJnt(q_init, frame, q_out)
            position_error = None
        else:
            result, position_error = self._solve_ik_position_only(pos, q_init, q_out)
        ik_elapsed_ms = (time.perf_counter() - ik_start) * 1000

        # 結果処理
        positions = self._process_ik_result(
            result, q_out, q_init, joint_count, pos, position_error, ik_elapsed_ms, now_sec
        )

        if positions is not None:
            # 関節角度をパブリッシュ
            joint_state = JointState()
            joint_state.header = msg.header
            joint_state.name = self.joint_names
            joint_state.position = positions
            self.joint_angles_pub.publish(joint_state)

            # 次回のdelta limit用に保存
            self._last_output_positions = positions

        self._maybe_log_ik_stats(now_sec)

    def _transform_to_base_frame(self, msg: PoseStamped, now_sec: float):
        """目標フレームをbase_linkに変換"""
        if msg.header.frame_id in ('odom', 'map'):
            msg.header.frame_id = 'world'

        if msg.header.frame_id and msg.header.frame_id != self.tf_base_link:
            try:
                stamp = msg.header.stamp
                if stamp.sec == 0 and stamp.nanosec == 0:
                    stamp = self.get_clock().now().to_msg()
                tf_msg = self.tf_buffer.lookup_transform(
                    self.tf_base_link, msg.header.frame_id, Time.from_msg(stamp)
                )
                return do_transform_pose_stamped(msg, tf_msg)
            except Exception as exc:
                if now_sec - self._last_tf_warn > 1.0:
                    self.get_logger().warn(
                        f'TF transform failed: {msg.header.frame_id} -> {self.tf_base_link}: {exc}'
                    )
                    self._last_tf_warn = now_sec
                return None

        return msg

    def _get_initial_joints(self, joint_count: int) -> PyKDL.JntArray:
        """IK初期値を取得"""
        q_init = PyKDL.JntArray(joint_count)
        if self._latest_joint_positions is None:
            for i in range(joint_count):
                q_init[i] = 0.0
        else:
            for i in range(min(joint_count, len(self._latest_joint_positions))):
                q_init[i] = float(self._latest_joint_positions[i])
        return q_init

    def _solve_ik_position_only(self, pos: Point, q_init: PyKDL.JntArray, q_out: PyKDL.JntArray):
        """位置のみでIKを解く（返り値: (result, position_error)）"""
        if self.fk_solver is None or self.jac_solver is None:
            return -1, None

        q_tmp = PyKDL.JntArray(q_init)
        damping = max(self.ik_damping, 1e-6)
        target = np.array([pos.x, pos.y, pos.z], dtype=float)

        best_error = float('inf')
        best_q = None

        for iteration in range(max(self.ik_max_iterations, 1)):
            frame = PyKDL.Frame()
            self.fk_solver.JntToCart(q_tmp, frame)
            current = np.array([frame.p[0], frame.p[1], frame.p[2]], dtype=float)
            error = target - current
            error_norm = np.linalg.norm(error)

            # ベスト解を保存
            if error_norm < best_error:
                best_error = error_norm
                best_q = [q_tmp[i] for i in range(q_out.rows())]

            if error_norm < self.ik_tolerance:
                for i in range(q_out.rows()):
                    q_out[i] = q_tmp[i]
                return 0, error_norm

            # ヤコビアン計算
            jac = PyKDL.Jacobian(q_out.rows())
            self.jac_solver.JntToJac(q_tmp, jac)
            j_mat = np.zeros((3, q_out.rows()), dtype=float)
            for col in range(q_out.rows()):
                j_mat[0, col] = jac[0, col]
                j_mat[1, col] = jac[1, col]
                j_mat[2, col] = jac[2, col]

            # ダンピング最小二乗法
            jj_t = j_mat @ j_mat.T
            inv = np.linalg.inv(jj_t + (damping * damping) * np.eye(3))
            dq = j_mat.T @ (inv @ error)
            for i in range(q_out.rows()):
                q_tmp[i] += float(dq[i])

        # 収束しなかった場合はベスト解を返す
        if best_q is not None:
            for i in range(q_out.rows()):
                q_out[i] = best_q[i]

        return -101, best_error

    def _process_ik_result(self, result, q_out, q_init, joint_count, pos, position_error, ik_elapsed_ms, now_sec):
        """IK結果を処理して関節角度を返す（失敗時はNone or 近似解）"""
        positions = []

        if result >= 0:
            # 成功
            for i in range(joint_count):
                angle = float(q_out[i])
                name = self.joint_names[i] if i < len(self.joint_names) else None
                if name and name in self.joint_limits:
                    lower, upper = self.joint_limits[name]
                    angle = max(lower, min(upper, angle))
                positions.append(angle)

            self._ik_stats['success'] += 1
            self._ik_stats['time_ms_sum'] += ik_elapsed_ms
            status = 'SUCCESS'

        else:
            # 失敗 - 近似解を使うかどうか判断
            use_approximate = False

            if self.use_approximate_on_fail:
                # 誤差が閾値以下なら近似解を使用
                if position_error is not None and position_error < self.approximate_error_threshold:
                    use_approximate = True
                elif position_error is None:
                    # orientation使用時は常に近似解を試す
                    use_approximate = True

            if use_approximate:
                for i in range(joint_count):
                    angle = float(q_out[i])
                    name = self.joint_names[i] if i < len(self.joint_names) else None
                    if name and name in self.joint_limits:
                        lower, upper = self.joint_limits[name]
                        angle = max(lower, min(upper, angle))
                    positions.append(angle)

                self._ik_stats['approximate'] += 1
                self._ik_stats['time_ms_sum'] += ik_elapsed_ms
                status = f'APPROXIMATE (err={position_error:.3f}m)' if position_error else 'APPROXIMATE'
            else:
                self._ik_stats['fail'] += 1
                if now_sec - self._last_ik_warn > 0.5:
                    err_str = f'{position_error:.3f}m' if position_error else 'unknown'
                    self.get_logger().warn(
                        f'IK FAILED: error={err_str} > threshold={self.approximate_error_threshold}m, '
                        f'target=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})'
                    )
                    self._last_ik_warn = now_sec
                return None

        # Delta limit適用
        if positions and self.enable_delta_limit and self._last_output_positions is not None:
            positions, limited = self._apply_delta_limit(positions)
            if limited:
                self._ik_stats['delta_limited'] += 1
                status += ' [DELTA_LIMITED]'

        # ログ出力
        self.get_logger().debug(
            f'IK {status} ({ik_elapsed_ms:.1f}ms): joints={[f"{p:.2f}" for p in positions]}'
        )

        return positions

    def _apply_delta_limit(self, positions):
        """関節角度の変化量を制限（滑らかな動き）"""
        limited = False
        result = []

        for i, (new_pos, old_pos) in enumerate(zip(positions, self._last_output_positions)):
            delta = new_pos - old_pos

            if abs(delta) > self.max_joint_delta_rad:
                # 制限を適用
                clamped_delta = self.max_joint_delta_rad if delta > 0 else -self.max_joint_delta_rad
                result.append(old_pos + clamped_delta)
                limited = True
            else:
                result.append(new_pos)

        return result, limited

    def joint_states_callback(self, msg: JointState):
        """IK初期値用の関節角を取得"""
        if not msg.position:
            return

        if not msg.name:
            if len(msg.position) < len(self.joint_names):
                return
            self._latest_joint_positions = list(msg.position[:len(self.joint_names)])
            self._has_joint_state = True
            # 初回は出力位置も初期化
            if self._last_output_positions is None:
                self._last_output_positions = self._latest_joint_positions.copy()
            return

        positions = [0.0] * len(self.joint_names)
        for name, pos in zip(msg.name, msg.position):
            if name in self._joint_index:
                positions[self._joint_index[name]] = pos

        self._latest_joint_positions = positions
        self._has_joint_state = True

        # 初回は出力位置も初期化
        if self._last_output_positions is None:
            self._last_output_positions = positions.copy()

    # =========================================================================
    # 統計
    # =========================================================================

    def _reset_ik_stats_window(self, now_sec: float):
        self._ik_stats_window_start = now_sec
        self._ik_stats_last_log_time = now_sec
        self._ik_stats = self._create_empty_stats()

    def _maybe_log_ik_stats(self, now_sec: float):
        if not self.enable_debug_ik_stats:
            return
        if self._ik_stats_window_start is None:
            self._reset_ik_stats_window(now_sec)
            return
        if now_sec - self._ik_stats_last_log_time < self.ik_stats_log_interval_sec:
            return

        elapsed = max(now_sec - self._ik_stats_window_start, 1e-6)
        rate = self._ik_stats['count'] / elapsed
        success = self._ik_stats['success']
        approx = self._ik_stats['approximate']
        fail = self._ik_stats['fail']
        delta_limited = self._ik_stats['delta_limited']
        avg_ms = self._ik_stats['time_ms_sum'] / max(success + approx, 1)

        self.get_logger().info(
            f'IK stats: rate={rate:.1f}Hz '
            f'success={success} approx={approx} fail={fail} '
            f'delta_limit={delta_limited} avg_ms={avg_ms:.1f}'
        )
        self._reset_ik_stats_window(now_sec)


def main(args=None):
    """メイン関数（デフォルト）"""
    rclpy.init(args=args)
    node = IKSolverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_left_arm(args=None):
    """左アーム用IKソルバー"""
    import sys
    left_arm_args = (args or sys.argv) + [
        '--ros-args',
        '-p', 'target_pose_topic:=/left_arm/target_pose',
        '-p', 'joint_angles_topic:=/left_arm/ik/joint_angles'
    ]
    rclpy.init(args=left_arm_args)
    node = IKSolverNode('left_arm_ik_solver_node')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_right_arm(args=None):
    """右アーム用IKソルバー"""
    import sys
    right_arm_args = (args or sys.argv) + [
        '--ros-args',
        '-p', 'target_pose_topic:=/right_arm/target_pose',
        '-p', 'joint_angles_topic:=/right_arm/ik/joint_angles'
    ]
    rclpy.init(args=right_arm_args)
    node = IKSolverNode('right_arm_ik_solver_node')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
