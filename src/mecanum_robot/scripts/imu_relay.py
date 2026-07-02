#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuRelay(Node):
    def __init__(self):
        super().__init__('imu_relay')
        self.sub = self.create_subscription(Imu, 'imu/data_raw', self.callback, 10)
        self.pub = self.create_publisher(Imu, 'imu/data', 10)

    def callback(self, msg):
        msg.orientation_covariance[0] = 0.001
        msg.orientation_covariance[4] = 0.001
        msg.orientation_covariance[8] = 0.001

        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.001

        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        self.pub.publish(msg)

rclpy.init()
node = ImuRelay()
node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
rclpy.spin(node)