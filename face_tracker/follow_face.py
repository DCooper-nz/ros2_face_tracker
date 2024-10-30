# Copyright 2023 Josh Newans
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
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped
import time

class FollowFace(Node):

    def __init__(self):
        super().__init__('follow_face')
        self.subscription = self.create_subscription(
            Point,
            '/detected_face',
            self.listener_callback,
            10)
        #self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.frame_id = "link_0"
        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 1)
        self.declare_parameter("forward_chase_speed", 0.1)
        self.declare_parameter("search_angular_speed", 0.0)
        self.declare_parameter("max_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.1)
        self.declare_parameter("offset_thresh", 0.1)
        self.declare_parameter("dist_thresh", 0.05)
        self.declare_parameter("dist_offset", 0.75)


        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value
        self.offset_thresh = self.get_parameter('offset_thresh').get_parameter_value().double_value
        self.dist_thresh = self.get_parameter('dist_thresh').get_parameter_value().double_value
        self.dist_offset = self.get_parameter('dist_offset').get_parameter_value().double_value

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target_x_val = 0.0
        self.target_y_val = 0.0
        self.target_z_val = 0.8
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000

    def timer_callback(self):
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_0"
        if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
            if (abs(self.target_x_val) > self.offset_thresh):
                msg.twist.linear.x = -self.target_x_val          
            if (abs(self.target_y_val) > self.offset_thresh):
               msg.twist.linear.z = self.target_y_val

            if (abs(self.target_dist - self.dist_offset) > self.dist_thresh):
                msg.twist.linear.y = (self.target_dist - self.dist_offset) 


            #self.get_logger().info('Sent: {} {}'.format(msg.twist.linear.x, msg.twist.linear.z))
        else:
            self.get_logger().info('Target lost')
            msg.twist.linear.z = 0.0
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0

        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        f = self.filter_value
        self.target_x_val = (self.target_x_val * f + msg.x * (1-f))
        self.target_y_val = self.target_y_val * f + msg.y * (1-f)
        self.target_dist = self.target_dist * f + (msg.z / 1000) * (1-f)
        self.lastrcvtime = time.time()
        #self.get_logger().info('Received: {} {}'.format(msg.x, msg.y))
        self.get_logger().info('Received: {}'.format(self.target_dist))


def main(args=None):
    rclpy.init(args=args)
    follow_face = FollowFace()
    rclpy.spin(follow_face)
    follow_face.destroy_node()
    rclpy.shutdown()
