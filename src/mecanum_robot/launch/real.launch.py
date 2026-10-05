"""Real robot: everything on the PC side of the ESP32.

    ros2 launch mecanum_robot real.launch.py            # also starts the micro-ROS agent
    ros2 launch mecanum_robot real.launch.py agent:=false

The robot itself publishes /imu/data, /tof/range and /odom over WiFi through the agent.
Not started here yet: the navigator (the firmware has no cmd_vel input yet).
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('mecanum_robot')
    robot_description = xacro.process_file(
        os.path.join(pkg_path, 'urdf', 'mecanum_robot.urdf.xacro')).toxml()
    # Spin recordings for tools/slam_replay.py and tools/wall_heading.py (no truth.jsonl on hardware)
    record_dir = os.path.expanduser(time.strftime('~/mecanum_ws/slam_records/real_%Y-%m-%d_%H-%M-%S'))

    return LaunchDescription([
        DeclareLaunchArgument('agent', default_value='true',
                              description='start the micro-ROS agent (UDP 9999)'),

        # micro-ROS agent: the bridge to the ESP32 (it retries until this is up)
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('agent')),
            cmd=['bash', '-c', 'source ~/microros_ws/install/setup.bash && '
                               'exec ros2 run micro_ros_agent micro_ros_agent udp4 --port 9999'],
            output='screen',
        ),

        # Robot geometry: base_link -> sensors / IMU, so readings can be placed in the world
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        # The wheels are continuous joints; publish them at 0 so their TF exists (RViz model)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        ),

        # /tof/range (all five sensors) -> front_tof/range, right_tof/range, ...
        Node(
            package='mecanum_robot',
            executable='tof_splitter.py',
            output='screen',
        ),

        # Wheel velocities + IMU heading -> odom -> base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_path, 'config', 'ekf_real.yaml')],
        ),

        # Mapping + wall heading correction. manhattan_axes_deg is left unset: the robot's start angle
        # to the room is unknown, so the wall axes are learned from the first clear spin.
        Node(
            package='mecanum_robot',
            executable='sparse_slam',
            output='screen',
            parameters=[{'record_dir': record_dir}],
        ),
    ])
