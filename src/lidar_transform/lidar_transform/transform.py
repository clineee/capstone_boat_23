# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

import numpy as np
import sys
#from sensor_msgs import PointCloud2, PointField
#from sensor_msgs_py import point_cloud2
#from std_msgs.msg import Header
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import laser_geometry.laser_geometry as lg


class LidarTransform(Node):

    def __init__(self):
        super().__init__('lidar_transform')

        self.lp = lg.LaserProjection()

        # set up lidar faking publisher to topic "filtered_cloud"
        self.publisher = self.create_publisher(sensor_msgs.PointCloud2, 'filtered_cloud', 10)

        # set up listener which listens for clusters from topic "cluster_0"
        self.subscription = self.create_subscription(sensor_msgs.LaserScan, "/scan", self.convert_callback, 10)
        #self.subs = [self.create_subscription(sensor_msgs.PointCloud2, f"cluster_{i}", self.cluster_callback, 10) for i in range(6)]

    def convert_callback(self, msg):
        # convert msg from LaserScan to PC2
        pc2_msg = self.lp.projectLaser(msg)

        # and now publish it
        self.publisher.publish(pc2_msg)


def main(args=None):
    rclpy.init(args=args)

    lidar_transform= LidarTransform()

    rclpy.spin(lidar_transform)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_transform.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
