#!/usr/bin/env python3
"""Turns Gazebo's perfect /odom into drifting dead-reckoning on /odom_noisy.

Each message, the true motion since the last one is taken in the robot frame,
corrupted with a fixed scale error plus random noise, and integrated into a
separate pose. Like real wheel odometry, the error accumulates and never
corrects itself. Ground truth stays available on /odom for comparison.
"""
import math
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomNoise(Node):
    def __init__(self):
        super().__init__('odom_noise')
        # Systematic errors: the same bias all run, like miscalibrated wheels
        self.forward_scale = self.declare_parameter('forward_scale', 1.03).value  # overestimates forward travel 3%
        self.strafe_scale = self.declare_parameter('strafe_scale', 0.92).value    # mecanum strafing slips
        self.yaw_scale = self.declare_parameter('yaw_scale', 1.0).value
        # Random errors: std dev grows with sqrt(distance), i.e. a random walk
        self.linear_noise = self.declare_parameter('linear_noise', 0.02).value  # m per sqrt(m)
        self.yaw_noise = self.declare_parameter('yaw_noise', 0.01).value        # rad per sqrt(m + rad)
        random.seed(self.declare_parameter('seed', 42).value)

        self.last_true = None              # (x, y, yaw, t) of the previous true pose
        self.x = self.y = self.yaw = 0.0   # drifting pose

        self.sub = self.create_subscription(Odometry, 'odom', self.callback, 10)
        self.pub = self.create_publisher(Odometry, 'odom_noisy', 10)

    def callback(self, msg):
        p = msg.pose.pose
        tx, ty, tyaw = p.position.x, p.position.y, yaw_from_quat(p.orientation)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_true is None:
            # Start the drifting pose exactly on the true one
            self.x, self.y, self.yaw = tx, ty, tyaw
            self.last_true = (tx, ty, tyaw, t)
            return

        lx, ly, lyaw, lt = self.last_true
        self.last_true = (tx, ty, tyaw, t)
        dt = t - lt
        if dt <= 0.0:
            return

        # True motion since last message, in the robot frame (forward is +Y)
        dx, dy = tx - lx, ty - ly
        c, s = math.cos(lyaw), math.sin(lyaw)
        strafe = c * dx + s * dy
        forward = -s * dx + c * dy
        dyaw = wrap(tyaw - lyaw)

        # Corrupt it. No motion -> no added noise, so a parked robot doesn't drift.
        dist = math.hypot(strafe, forward)
        strafe = strafe * self.strafe_scale + random.gauss(0.0, self.linear_noise * math.sqrt(dist))
        forward = forward * self.forward_scale + random.gauss(0.0, self.linear_noise * math.sqrt(dist))
        dyaw = dyaw * self.yaw_scale + random.gauss(0.0, self.yaw_noise * math.sqrt(dist + abs(dyaw)))

        # Integrate into the drifting pose using its own (also drifting) heading
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += c * strafe - s * forward
        self.y += s * strafe + c * forward
        self.yaw = wrap(self.yaw + dyaw)

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        out.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        out.twist.twist.linear.x = strafe / dt
        out.twist.twist.linear.y = forward / dt
        out.twist.twist.angular.z = dyaw / dt

        out.pose.covariance[0] = 0.01
        out.pose.covariance[7] = 0.01
        out.pose.covariance[35] = 0.05
        out.twist.covariance[0] = 0.01
        out.twist.covariance[7] = 0.01
        out.twist.covariance[35] = 0.05

        self.pub.publish(out)

        self.get_logger().info(
            'Odom drift: %.3f m, %.1f deg' % (
                math.hypot(self.x - tx, self.y - ty), math.degrees(wrap(self.yaw - tyaw))),
            throttle_duration_sec=5.0)


rclpy.init()
node = OdomNoise()
rclpy.spin(node)
