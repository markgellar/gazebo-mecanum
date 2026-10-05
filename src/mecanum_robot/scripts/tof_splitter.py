#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

class TofSplitter(Node):
    def __init__(self):
        super().__init__('tof_splitter')
        # The robot publishes ToF best effort (no per-message acks over WiFi); a reliable subscription
        # would be incompatible and receive nothing. Republished below with default (reliable) QoS.
        self.sub = self.create_subscription(Range, 'tof/range', self.callback, qos_profile_sensor_data)
        self.pubs = {
            'front_tof_link': self.create_publisher(Range, 'front_tof/range', 10),
            'right_tof_link': self.create_publisher(Range, 'right_tof/range', 10),
            'rear_right_tof_link': self.create_publisher(Range, 'rear_right_tof/range', 10),
            'rear_left_tof_link': self.create_publisher(Range, 'rear_left_tof/range', 10),
            'left_tof_link': self.create_publisher(Range, 'left_tof/range', 10),
        }

    def callback(self, msg):
        frame = msg.header.frame_id
        if frame in self.pubs:
            self.pubs[frame].publish(msg)

def main():
    rclpy.init()
    rclpy.spin(TofSplitter())

if __name__ == '__main__':
    main()
