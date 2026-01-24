#!/usr/bin/env python3
"""
IK to JointTrajectory Node
IK結果(JointState)をJointTrajectoryに変換してコントローラへ送る。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class IKToJointTrajectoryNode(Node):
    """IK出力をJointTrajectoryに変換するノード"""

    def __init__(self):
        super().__init__('ik_to_joint_trajectory_node')

        # パラメータ
        self.declare_parameter('left_ik_topic', '/left_arm/ik/joint_angles')
        self.declare_parameter('right_ik_topic', '/right_arm/ik/joint_angles')
        self.declare_parameter('left_command_topic', '/leader/arm_controller/joint_trajectory')
        self.declare_parameter('right_command_topic', '/follower/arm_controller/joint_trajectory')
        self.declare_parameter('command_duration_sec', 0.2)
        self.declare_parameter('use_joint_names_from_msg', True)
        self.declare_parameter(
            'joint_names',
            ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'],
        )

        self.left_ik_topic = self.get_parameter('left_ik_topic').value
        self.right_ik_topic = self.get_parameter('right_ik_topic').value
        self.left_command_topic = self.get_parameter('left_command_topic').value
        self.right_command_topic = self.get_parameter('right_command_topic').value
        self.command_duration_sec = float(self.get_parameter('command_duration_sec').value)
        self.use_joint_names_from_msg = bool(self.get_parameter('use_joint_names_from_msg').value)
        self.default_joint_names = self.get_parameter('joint_names').value

        self.left_pub = self.create_publisher(JointTrajectory, self.left_command_topic, 10)
        self.right_pub = self.create_publisher(JointTrajectory, self.right_command_topic, 10)

        self.create_subscription(JointState, self.left_ik_topic, self._left_cb, 10)
        self.create_subscription(JointState, self.right_ik_topic, self._right_cb, 10)

        self.get_logger().info(
            f'IK->Trajectory: left {self.left_ik_topic} -> {self.left_command_topic}, '
            f'right {self.right_ik_topic} -> {self.right_command_topic}'
        )

    def _left_cb(self, msg: JointState):
        self._publish_trajectory(msg, self.left_pub)

    def _right_cb(self, msg: JointState):
        self._publish_trajectory(msg, self.right_pub)

    def _publish_trajectory(self, msg: JointState, pub):
        if not msg.position:
            return

        if self.use_joint_names_from_msg and msg.name:
            joint_names = list(msg.name)
            positions = list(msg.position)
            if len(joint_names) != len(positions):
                self.get_logger().warn(
                    f'Joint count mismatch: names={len(joint_names)}, positions={len(positions)}'
                )
                return
        else:
            joint_names = list(self.default_joint_names)
            positions = list(msg.position)
            if len(positions) < len(joint_names):
                self.get_logger().warn(
                    f'Joint count mismatch: names={len(joint_names)}, positions={len(positions)}'
                )
                return
            if len(positions) > len(joint_names):
                positions = positions[:len(joint_names)]

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(joint_names)

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(self.command_duration_sec),
                                         nanosec=int((self.command_duration_sec % 1.0) * 1e9))
        traj.points = [point]

        pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = IKToJointTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
