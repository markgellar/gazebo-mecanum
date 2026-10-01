#!/usr/bin/env python3
"""Measures how far the robot's pose estimate is from Gazebo ground truth.

Ground truth is /odom (exact, sim only). The estimate is base_link in
`estimate_frame`, read from TF: 'odom' is the EKF's dead-reckoning, 'map' will
be the SLAM-corrected pose. Observe-only: nothing here affects the robot.
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def relative(origin, pose):
    """`pose` expressed in the frame of `origin`; both are (x, y, yaw)."""
    ox, oy, oyaw = origin
    dx, dy = pose[0] - ox, pose[1] - oy
    c, s = math.cos(oyaw), math.sin(oyaw)
    return (c * dx + s * dy, -s * dx + c * dy, wrap(pose[2] - oyaw))


class PoseError(Node):
    def __init__(self):
        super().__init__('pose_error')
        self.estimate_frame = self.declare_parameter('estimate_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        # Evaluate truth samples this old, so TF for that exact moment has arrived
        self.delay = Duration(seconds=self.declare_parameter('delay', 0.2).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pending = deque()   # (stamp, truth pose) waiting to be evaluated
        self.truth_origin = None
        self.est_origin = None
        self.last_truth = None

        self.distance = 0.0
        self.count = 0
        self.sum_err = 0.0
        self.sum_sq_err = 0.0
        self.max_err = 0.0

        self.sub = self.create_subscription(Odometry, 'odom', self.callback, 50)
        self.pos_pub = self.create_publisher(Float64, 'pose_error/position', 10)
        self.hdg_pub = self.create_publisher(Float64, 'pose_error/heading', 10)

    def callback(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if self.pending and stamp < self.pending[-1][0]:
            self.pending.clear()   # sim time jumped back (Gazebo reset)

        p = msg.pose.pose
        self.pending.append((stamp, (p.position.x, p.position.y, yaw_from_quat(p.orientation))))

        cutoff = stamp - self.delay
        while self.pending and self.pending[0][0] <= cutoff:
            self.evaluate(*self.pending.popleft())

    def evaluate(self, stamp, truth):
        try:
            tf = self.tf_buffer.lookup_transform(self.estimate_frame, self.base_frame, stamp)
        except TransformException:
            return
        t = tf.transform
        est = (t.translation.x, t.translation.y, yaw_from_quat(t.rotation))

        # Measure from where each one started, so a spawn offset doesn't count as error
        if self.truth_origin is None:
            self.truth_origin, self.est_origin, self.last_truth = truth, est, truth
            self.get_logger().info('Comparing %s in %s against ground truth'
                                   % (self.base_frame, self.estimate_frame))
            return

        self.distance += math.hypot(truth[0] - self.last_truth[0], truth[1] - self.last_truth[1])
        self.last_truth = truth

        tr = relative(self.truth_origin, truth)
        er = relative(self.est_origin, est)
        pos_err = math.hypot(er[0] - tr[0], er[1] - tr[1])
        hdg_err = math.degrees(wrap(er[2] - tr[2]))

        self.count += 1
        self.sum_err += pos_err
        self.sum_sq_err += pos_err * pos_err
        self.max_err = max(self.max_err, pos_err)

        self.pos_pub.publish(Float64(data=pos_err))
        self.hdg_pub.publish(Float64(data=hdg_err))

        pct = 100.0 * pos_err / self.distance if self.distance > 0.01 else 0.0
        self.get_logger().info(
            'Error %.3f m (max %.3f), heading %+.1f deg, traveled %.2f m, %.1f%% of distance'
            % (pos_err, self.max_err, hdg_err, self.distance, pct),
            throttle_duration_sec=5.0)

    def summary(self):
        if self.count == 0:
            self.get_logger().info('No samples compared (was TF %s -> %s ever available?)'
                                   % (self.estimate_frame, self.base_frame))
            return
        mean = self.sum_err / self.count
        rms = math.sqrt(self.sum_sq_err / self.count)
        self.get_logger().info(
            'Run summary [%s]: %d samples, %.2f m traveled, error mean %.3f / RMS %.3f / max %.3f m'
            % (self.estimate_frame, self.count, self.distance, mean, rms, self.max_err))


rclpy.init()
node = PoseError()
try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    node.summary()
    node.destroy_node()
    rclpy.try_shutdown()
