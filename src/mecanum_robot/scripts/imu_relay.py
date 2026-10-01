#!/usr/bin/env python3
import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ImuRelay(Node):
    def __init__(self):
        super().__init__('imu_relay')
        # Gazebo's IMU orientation is exact; real heading drifts. These add that drift.
        self.yaw_bias = math.radians(self.declare_parameter('yaw_bias', 0.5).value) / 60.0  # deg/min of steady creep
        self.yaw_scale_error = self.declare_parameter('yaw_scale_error', 0.002).value      # 0.2% of each rotation (~0.7 deg per spin)
        self.yaw_noise = math.radians(self.declare_parameter('yaw_noise', 0.02).value)     # deg per sqrt(s) random walk
        random.seed(self.declare_parameter('seed', 7).value)

        self.yaw_offset = 0.0
        self.last_yaw = None
        self.last_t = None

        self.sub = self.create_subscription(Imu, 'imu/data_raw', self.callback, 10)
        self.pub = self.create_publisher(Imu, 'imu/data', 10)

    def callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        true_yaw = yaw_from_quat(msg.orientation)

        # Grow the heading error: steady bias + scale error on actual rotation + random walk
        rate_error = 0.0
        if self.last_t is not None and t > self.last_t:
            dt = t - self.last_t
            true_rate = wrap(true_yaw - self.last_yaw) / dt
            rate_error = self.yaw_bias + self.yaw_scale_error * true_rate
            self.yaw_offset += rate_error * dt + random.gauss(0.0, self.yaw_noise * math.sqrt(dt))
        self.last_yaw, self.last_t = true_yaw, t

        # Rotate the orientation about world Z by the offset (q_offset * q)
        q = msg.orientation
        cw, sz = math.cos(self.yaw_offset / 2.0), math.sin(self.yaw_offset / 2.0)
        q.w, q.x, q.y, q.z = (cw * q.w - sz * q.z, cw * q.x - sz * q.y,
                              cw * q.y + sz * q.x, cw * q.z + sz * q.w)

        # Gyro reads the same systematic error, so it agrees with the drifting orientation
        msg.angular_velocity.z += rate_error

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

        self.get_logger().info('IMU yaw drift: %+.1f deg' % math.degrees(wrap(self.yaw_offset)),
                               throttle_duration_sec=5.0)

rclpy.init()
node = ImuRelay()
node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
rclpy.spin(node)
