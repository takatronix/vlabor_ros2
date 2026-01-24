#!/usr/bin/env python3
"""
JointState Mirror Node
指定したJointStateトピックを別トピックにコピーする。
"""
from typing import Any, Dict, List

import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMirrorNode(Node):
    """JointStateを別トピックへ転送するノード"""

    def __init__(self):
        super().__init__('joint_state_mirror_node')

        self.declare_parameter('config_file', '')
        self.declare_parameter('mappings', [])
        self.declare_parameter('update_stamp', True)

        self.update_stamp = bool(self.get_parameter('update_stamp').value)
        mappings = self._load_mappings()

        self._publishers = []
        for mapping in mappings:
            src = mapping.get('src')
            dst = mapping.get('dst')
            if not src or not dst:
                self.get_logger().warn(f'Invalid mapping (src/dst required): {mapping}')
                continue

            pub = self.create_publisher(JointState, dst, 10)
            self._publishers.append(pub)
            self.create_subscription(
                JointState,
                src,
                self._make_callback(pub),
                10,
            )
            self.get_logger().info(f'Mirror: {src} -> {dst}')

        if not self._publishers:
            self.get_logger().warn('No valid mappings found; node is idle.')

    def _load_mappings(self) -> List[Dict[str, Any]]:
        config_file = self.get_parameter('config_file').value
        if isinstance(config_file, str) and config_file.strip():
            try:
                with open(config_file, 'r') as f:
                    data = yaml.safe_load(f) or {}
                mappings = data.get('copy_map', [])
                if isinstance(mappings, list):
                    return mappings
            except Exception as exc:
                self.get_logger().error(f'Failed to load config_file: {exc}')
                return []

        raw_mappings = self.get_parameter('mappings').value
        if isinstance(raw_mappings, list):
            return [m for m in raw_mappings if isinstance(m, dict)]

        return []

    def _make_callback(self, publisher):
        def _callback(msg: JointState):
            out = JointState()
            out.header = msg.header
            if self.update_stamp:
                out.header.stamp = self.get_clock().now().to_msg()
            out.name = list(msg.name)
            out.position = list(msg.position)
            out.velocity = list(msg.velocity)
            out.effort = list(msg.effort)
            publisher.publish(out)
        return _callback


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMirrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
