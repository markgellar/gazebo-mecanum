import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('mecanum_robot')
    urdf_file = os.path.join(pkg_path, 'urdf', 'mecanum_robot.urdf.xacro')

    # Process xacro into raw URDF XML
    robot_description = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([
        # Publish the URDF to /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # GUI sliders to move continuous joints (your wheels)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_path, 'config', 'display.rviz')],
        ),
    ])
