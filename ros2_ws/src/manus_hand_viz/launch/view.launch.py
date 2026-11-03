from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('demo', default_value='false', description='Use synthetic data instead of real gloves'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        Node(package='manus_hand_viz', executable='hand_viz', output='screen'),
        Node(package='manus_hand_viz', executable='mock_gloves', condition=IfCondition(LaunchConfiguration('demo')), output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('manus_hand_viz'), 'config', 'hands.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
