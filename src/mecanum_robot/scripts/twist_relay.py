#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')
        self.sub = self.create_subscription(Twist, 'cmd_vel_raw', self.callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def callback(self, msg):
        out = Twist()
        out.linear.x = -msg.linear.y
        out.linear.y = msg.linear.x
        out.angular.z = msg.angular.z
        self.pub.publish(out)

rclpy.init()
rclpy.spin(TwistRelay())
