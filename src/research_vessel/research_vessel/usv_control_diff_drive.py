#!/usr/bin/env python3
'''
Using instances of the pypid Pid class to control yaw and velocity
'''
# Python
# Original work by Brian Bingham <briansbingham@gmail.com>
# ported to ROS2 and for use with VRX by Samuel Jadzak

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

import sys
from math import pi

# ROS
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from research_vessel.msg import PidDiagnose

# From this package
from pypid import Pid


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Setup Yaw Pid
        self.ypid = Pid(0.0, 0.0, 0.0)
        self.ypid.set_setpoint(0.0)
        self.ypid.set_derivfeedback(True)  # D term in feedback look
        fc = 20;  # cutoff freq in hz
        wc = fc*(2.0*pi)  # cutoff freq. in rad/s
        self.ypid.set_derivfilter(1,wc)
        self.ypid.set_maxIout(1.0)
        
        # Setup Velocity Pid
        self.vpid = Pid(0.0, 0.0, 0.0)
        self.vpid.set_setpoint(0.0)
        self.vpid.set_maxIout(1.0)
        self.vpid.set_derivfeedback(True)  # D term in feedback look
        fc = 20;  # cutoff freq in hz
        wc = fc*(2.0*pi)  # cutoff freq. in rad/s
        self.vpid.set_derivfilter(1,wc)
        
        # Initialize some bits as none - for now
        self.lasttime = None
        
        # For diagnosing/tuning PID
        self.ydebugmsg = PidDiagnose()
        self.vdebugmsg = PidDiagnose()

        self.declare_parameter('yawKp', 0.0)
        self.declare_parameter('yawKd', 0.0)
        self.declare_parameter('yawKi', 0.0)
        self.declare_parameter('velKp', 0.0)
        self.declare_parameter('velKd', 0.0)
        self.declare_parameter('velKi', 0.0)
        
        # Set initial gains from parameters
        yawKp = self.get_parameter('yawKp').value or 180.0
        yawKd = self.get_parameter('yawKd').value or 100.0
        yawKi = self.get_parameter('yawKi').value or 100.0

        velKp = self.get_parameter('velKp').value or 180.0
        velKd = self.get_parameter('velKd').value or 100.0
        velKi = self.get_parameter('velKi').value or 100.0
        
        self.ypid.Kp = yawKp
        self.ypid.Kd = yawKd
        self.ypid.Ki = yawKi
        
        self.vpid.Kp = velKp
        self.vpid.Kd = velKd
        self.vpid.Ki = velKi
        
        # Setup outbound messages
        self.left_cmd = Float64()
        self.right_cmd = Float64()
        
        # Setup publisher and subscribers 
        self.left_publisher = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 1)
        self.right_publisher = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 1)
        self.ypubdebug = self.create_publisher(PidDiagnose, "yaw_pid_debug", 10)
        self.vpubdebug = self.create_publisher(PidDiagnose, "vel_pid_debug", 10)
        
        self.create_subscription(Odometry, 'odometry/global', self.odom_callback, 10)
        self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        
    def twist_callback(self,msg):
        self.ypid.set_setpoint(msg.angular.z)
        self.vpid.set_setpoint(msg.linear.x)

    def odom_callback(self,msg):
         # Yaw Control
        dyaw = msg.twist.twist.angular.z # measured rate (process variable)
        now = self.get_clock().now()
        if not self.lasttime:
            self.lasttime = now.to_msg().sec + now.to_msg().nanosec * 1e-9 
            return 
        dt = (now.to_msg().sec + now.to_msg().nanosec * 1e-9) -self.lasttime 
        if dt < 1e-6:
            return 
        yout =self.ypid.execute(dt,dyaw) 
        torque=yout[0] 
        
        dx=msg.twist.twist.linear.x 
        vout=self.vpid.execute(dt,dx) 
        thrust=vout[0] 
        
        left_thrust=-1*torque+thrust 
        right_thrust=torque+thrust 
        
        left_msg=Float64() 
        left_msg.data=left_thrust 
        
        right_msg=Float64() 
        right_msg.data=right_thrust 
         
        if not (self.left_publisher is None):
            left_msg.data=left_thrust 
            right_msg.data=right_thrust 
            self.left_publisher.publish(left_msg) 
            self.right_publisher.publish(right_msg)
            
        if not (self.ypubdebug is None):
            self.ydebugmsg.pid = yout[0]
            self.ydebugmsg.p = yout[1]
            self.ydebugmsg.i = yout[2]
            self.ydebugmsg.d = yout[3]
            self.ydebugmsg.error = yout[4]
            self.ydebugmsg.setpoint = yout[5]
            self.ydebugmsg.derivative= yout[6]
            self.ydebugmsg.integral = yout[7]
            self.ypubdebug.publish(self.ydebugmsg)
        if not (self.vpubdebug is None):
            self.vdebugmsg.pid = vout[0]
            self.vdebugmsg.p = vout[1]
            self.vdebugmsg.i = vout[2]
            self.vdebugmsg.d = vout[3]
            self.vdebugmsg.error = vout[4]
            self.vdebugmsg.setpoint = vout[5]
            self.vdebugmsg.derivative= vout[6]
            self.vdebugmsg.integral = vout[7]
            self.vpubdebug.publish(self.vdebugmsg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()