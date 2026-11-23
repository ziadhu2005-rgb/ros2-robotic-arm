import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory('arm_description')
    gz_share = get_package_share_directory('arm_gazebo')

    with open(os.path.join(desc_share, 'urdf', 'arm.urdf'), 'r') as f:
        robot_description = f.read()

    world = os.path.join(gz_share, 'worlds', 'arm_world.sdf')
    bridge_config = os.path.join(gz_share, 'config', 'bridge.yaml')
    rviz_config = os.path.join(desc_share, 'rviz', 'arm.rviz')

    rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')
    hardware = LaunchConfiguration('hardware')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz2'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Open the slider GUI'),
        DeclareLaunchArgument('hardware', default_value='false',
                              description='Also send commands to the real servos'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Serial port of the microcontroller'),

        # Gazebo with our world, started running (-r)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r {world}'}.items(),
        ),

        # Publishes /robot_description and TF from /joint_states (coming from Gazebo)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}],
            output='screen',
        ),

        # Spawns the URDF from /robot_description into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'arm', '-z', '0.0'],
            output='screen',
        ),

        # Bridges clock, joint states and joint commands between ROS 2 and Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),

        # Turns /arm_controller/joint_trajectory into per-joint Gazebo commands
        Node(
            package='arm_control',
            executable='trajectory_follower',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # Sliders that publish to /arm_controller/joint_trajectory
        Node(
            package='arm_control',
            executable='slider_gui',
            condition=IfCondition(gui),
        ),

        # Sends the same trajectories to the microcontroller over USB serial
        Node(
            package='arm_hardware',
            executable='serial_bridge',
            parameters=[{'port': port}],
            output='screen',
            condition=IfCondition(hardware),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz),
        ),
    ])
