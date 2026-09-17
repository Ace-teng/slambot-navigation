#!/usr/bin/env python3
"""Seed AMCL with the known Gazebo spawn pose."""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('landerpi_initial_pose')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_confirmed, 10)
        self.sent = 0
        self.timer = self.create_timer(1.0, self.publish_pose)

    def publish_pose(self):
        if self.get_clock().now().nanoseconds == 0:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = -5.6
        msg.pose.pose.position.y = -3.5
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.04
        msg.pose.covariance[7] = 0.04
        msg.pose.covariance[35] = 0.02
        self.publisher.publish(msg)
        self.sent += 1
        if self.sent == 1:
            self.get_logger().info('Published known initial pose: x=-5.6, y=-3.5, yaw=0.')
        if self.sent >= 10:
            self.timer.cancel()

    def pose_confirmed(self, _msg):
        if self.sent > 0 and not self.timer.is_canceled():
            self.timer.cancel()
            self.get_logger().info('AMCL confirmed the initial pose; stopped republishing it.')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
