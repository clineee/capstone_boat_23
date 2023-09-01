#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import laser_geometry.laser_geometry as lg

class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/wamv/sensors/lidars/lidar_wamv_sensor/scan',
            self.scan_callback,
            10)

        self.lp = lg.LaserProjection()
        self.publisher = self.create_publisher(sensor_msgs.PointCloud2, 'filtered_cloud', 10)

    def scan_callback(self, msg: LaserScan):
        pcs2_msg = self.lp.projectLaser(msg)
        self.publisher.publish(pcs2_msg)

def main(args=None):
    rclpy.init(args=args)
    scan_filter_node = ScanFilterNode()
    rclpy.spin(scan_filter_node)
    scan_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()