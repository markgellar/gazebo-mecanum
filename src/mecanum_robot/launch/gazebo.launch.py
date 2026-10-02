import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='false = gzserver only (headless)'),
        DeclareLaunchArgument('seed', default_value='', description='noise seed for odom/IMU drift (default: fixed)'),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    gui = LaunchConfiguration('gui').perform(context).lower() != 'false'
    seed = LaunchConfiguration('seed').perform(context)
    odom_noise_params = {'use_sim_time': True}
    imu_params = {'use_sim_time': True}
    if seed:
        odom_noise_params['seed'] = int(seed)
        imu_params['seed'] = int(seed) + 1000

    pkg_path = get_package_share_directory('mecanum_robot')
    urdf_file = os.path.join(pkg_path, 'urdf', 'mecanum_robot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'obstacles.world')
    # One folder per run for spin recordings + truth (analyze with tools/slam_replay.py)
    record_dir = os.path.expanduser(time.strftime('~/mecanum_ws/slam_records/%Y-%m-%d_%H-%M-%S'))

    robot_description = xacro.process_file(urdf_file).toxml()

    return [
        # Point Gazebo to mesh files
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.path.join(pkg_path, '..')
        ),

        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo' if gui else 'gzserver', '--verbose', world_file,
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
                    parameters=[{'use_sim_time': True, 'record_dir': record_dir}],
                ),

                # IMU relay
                Node(
                    package='mecanum_robot',
                    executable='imu_relay.py',
                    name='imu_relay',
                    output='screen',
                    parameters=[imu_params],
                ),

                # Odometry noise: realistic drift between Gazebo's perfect /odom and the EKF
                Node(
                    package='mecanum_robot',
                    executable='odom_noise.py',
                    name='odom_noise',
                    output='screen',
                    parameters=[odom_noise_params],
                ),

                # Pose error vs ground truth (observe only). 'map' = SLAM-corrected pose, 'odom' = EKF alone.
                Node(
                    package='mecanum_robot',
                    executable='pose_error.py',
                    name='pose_error',
                    output='screen',
                    parameters=[{'use_sim_time': True, 'estimate_frame': 'map', 'record_dir': record_dir}],
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
    ]
