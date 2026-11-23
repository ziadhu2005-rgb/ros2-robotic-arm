import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'arm.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'arm.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Publishes /robot_description and the TF tree from /joint_states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        # Slider window to move the joints by hand
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        # RViz2, preconfigured with fixed frame base_link and a RobotModel display
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
