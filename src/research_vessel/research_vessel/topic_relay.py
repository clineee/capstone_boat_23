#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

custom_qos_profile = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

class OdometryToPose(Node):
    def __init__(self):
        super().__init__('odometry_to_pose')
        self.subscription = self.create_subscription(
            Odometry,
            'odometry/global',
            self.odometry_callback,
            10)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 
                                               'odom_to_pose', 
                                               qos_profile=custom_qos_profile)

    def odometry_callback(self, msg):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.pose.pose = msg.pose.pose
        pose_msg.pose.covariance = msg.pose.covariance
        self.publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
