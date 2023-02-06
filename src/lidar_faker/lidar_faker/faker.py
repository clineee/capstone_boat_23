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


class LidarFaker(Node):

    def __init__(self):
        super().__init__('lidar_faker')

        # set up lidar faking publisher to topic "filtered_cloud"
        self.publisher_ = self.create_publisher(sensor_msgs.PointCloud2, 'filtered_cloud', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # set up listener which listens for clusters from topic "cluster_0"
        self.subscription = self.create_subscription(sensor_msgs.PointCloud2, "cluster_0", self.cluster_callback, 10)
        self.subs = [self.create_subscription(sensor_msgs.PointCloud2, f"cluster_{i}", self.cluster_callback, 10) for i in range(6)]


    def timer_callback(self):
        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        fields = [sensor_msgs.PointField(name="theta", offset=0, datatype=sensor_msgs.PointField.FLOAT32, count=1),
                  sensor_msgs.PointField(name="z", offset=4, datatype=sensor_msgs.PointField.FLOAT32, count=1)]
        points = np.array([1.0,2.0,3.0,4.0])
        itemsize = points.itemsize
        msg = sensor_msgs.PointCloud2(header=std_msgs.Header(frame_id="frame"),
                          height=1,
                          width=points.shape[0],
                          is_dense=False,
                          is_bigendian=sys.byteorder != 'little',
                          fields=fields,
                          point_step=(itemsize * 2), # each point consists of 2 float32s
                          row_step=(itemsize * 2 * points.shape[0]),
                          data = points.tobytes())

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def cluster_callback(self, msg):

        self.get_logger().info(f"Received: {msg.height}x{msg.width} | {np.frombuffer(msg.data, dtype=np.float64)}")


def main(args=None):
    rclpy.init(args=args)

    lidar_faker = LidarFaker()

    rclpy.spin(lidar_faker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_faker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
