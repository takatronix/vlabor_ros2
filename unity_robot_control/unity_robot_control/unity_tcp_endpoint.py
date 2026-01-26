#!/usr/bin/env python3
"""
Unity TCP Endpoint Wrapper Node
ROS2 Unity TCP Endpointのラッパーノード
受信データのログ出力機能付き
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import subprocess
import os
import socket


class UnityTcpEndpointNode(Node):
    """Unity TCP Endpointラッパーノード"""

    def __init__(self):
        super().__init__('unity_tcp_endpoint')

        # パラメータ宣言
        self.declare_parameter('ros_ip', '0.0.0.0')
        self.declare_parameter('ros_tcp_port', 42000)
        self.declare_parameter('auto_detect_ip', True)

        # IPアドレス取得
        if self.get_parameter('auto_detect_ip').value:
            ros_ip = self._get_local_ip()
            self.get_logger().info(f'📡 Auto-detected IP: {ros_ip}')
        else:
            ros_ip = self.get_parameter('ros_ip').value

        ros_tcp_port = self.get_parameter('ros_tcp_port').value

        self.get_logger().info(f'🚀 Starting Unity TCP Endpoint...')
        self.get_logger().info(f'📡 Server IP: {ros_ip}')
        self.get_logger().info(f'🔌 TCP Port: {ros_tcp_port}')

        # ros_tcp_endpointを起動
        cmd = [
            'ros2', 'run', 'ros_tcp_endpoint', 'default_server_endpoint',
            '--ros-args',
            '-p', f'ROS_IP:={ros_ip}',
            '-p', f'ROS_TCP_PORT:={ros_tcp_port}'
        ]

        self.process = subprocess.Popen(cmd)
        self.get_logger().info('✅ Unity TCP Endpoint started')

        # 接続監視用
        self.ros_ip = ros_ip
        self.ros_tcp_port = ros_tcp_port
        self.last_connection_count = 0
        self.connection_timer = self.create_timer(2.0, self.check_connections)

        # Quest3からの受信データを監視するサブスクライバー
        self.headset_sub = self.create_subscription(
            PoseStamped, '/quest/pose/headset', self.headset_callback, 10)
        self.left_hand_sub = self.create_subscription(
            PoseStamped, '/quest/left_hand/pose', self.left_hand_callback, 10)
        self.right_hand_sub = self.create_subscription(
            PoseStamped, '/quest/right_hand/pose', self.right_hand_callback, 10)

        # 受信カウンター
        self.headset_count = 0
        self.left_count = 0
        self.right_count = 0

    def check_connections(self):
        """TCP接続を監視"""
        try:
            result = subprocess.run(
                ['ss', '-tnp'],
                capture_output=True,
                text=True
            )
            # ポート42000への接続を確認
            connections = []
            for line in result.stdout.split('\n'):
                if f':{self.ros_tcp_port}' in line and 'ESTAB' in line:
                    # 接続元IPを抽出
                    parts = line.split()
                    if len(parts) >= 5:
                        peer = parts[4]  # 例: 192.168.1.21:52914
                        connections.append(peer)

            conn_count = len(connections)
            if conn_count != self.last_connection_count:
                if conn_count > self.last_connection_count:
                    for conn in connections:
                        self.get_logger().info(f'🔗 Client connected: {conn}')
                elif conn_count == 0:
                    self.get_logger().info('❌ All clients disconnected')
                self.last_connection_count = conn_count

            # 接続中でデータなしの場合は警告
            if conn_count > 0 and self.headset_count == 0 and self.left_count == 0 and self.right_count == 0:
                self.get_logger().warn(f'⚠️ {conn_count} client(s) connected but no pose data received yet')

        except Exception as e:
            pass  # ssコマンドエラーは無視

    def headset_callback(self, msg: PoseStamped):
        self.headset_count += 1
        p = msg.pose.position
        if self.headset_count % 30 == 1:  # 30回に1回ログ
            self.get_logger().info(f'🎧 Headset: pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) [#{self.headset_count}]')

    def left_hand_callback(self, msg: PoseStamped):
        self.left_count += 1
        p = msg.pose.position
        if self.left_count % 30 == 1:
            self.get_logger().info(f'👈 Left hand: pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) [#{self.left_count}]')

    def right_hand_callback(self, msg: PoseStamped):
        self.right_count += 1
        p = msg.pose.position
        if self.right_count % 30 == 1:
            self.get_logger().info(f'👉 Right hand: pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) [#{self.right_count}]')
    
    def _get_local_ip(self):
        """ローカルIPアドレスを自動取得"""
        try:
            # 外部接続可能なIPを取得
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            # フォールバック: hostname -I
            import subprocess
            result = subprocess.run(
                ['hostname', '-I'],
                capture_output=True,
                text=True
            )
            if result.returncode == 0:
                return result.stdout.strip().split()[0]
            return '127.0.0.1'
    
    def destroy_node(self):
        """ノード終了時にプロセスを終了"""
        if hasattr(self, 'process') and self.process:
            self.get_logger().info('🛑 Stopping Unity TCP Endpoint...')
            self.process.terminate()
            self.process.wait()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnityTcpEndpointNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
