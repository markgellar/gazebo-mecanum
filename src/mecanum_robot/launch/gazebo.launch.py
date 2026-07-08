import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('mecanum_robot')
    urdf_file = os.path.join(pkg_path, 'urdf', 'mecanum_robot.urdf.xacro')

    robot_description = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([
        # Point Gazebo to mesh files
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.path.join(pkg_path, '..')
        ),

        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'],
            output='screen',
        ),

        # Publish URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'mecanum_robot'],
            output='screen',
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_path, 'config', 'ekf.yaml'),
            {'use_sim_time': True}],
        ),

        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-topic', 'robot_description',
                        '-entity', 'mecanum_robot',
                        '-x', '0',
                        '-y', '0',
                        '-z', '0.04',
                    ],
                    output='screen',
                ),

                # SLAM
                Node(
                    package='mecanum_robot',
                    executable='sparse_slam',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ),

                # IMU relay
                Node(
                    package='mecanum_robot',
                    executable='imu_relay.py',
                    name='imu_relay',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ),

                # Twist relay
                Node(
                    package='mecanum_robot',
                    executable='twist_relay.py',
                    name='twist_relay',
                    output='screen',
                ),

                # Navigator
                Node(
                    package='mecanum_robot',
                    executable='navigator',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                ),
            ],
        ),
    ])