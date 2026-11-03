"""Start the official remote publisher and the hand-only RViz viewer."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='manus_ros2', executable='manus_data_publisher', output='screen',
             parameters=[{'glove_topic_template': 'manus/{side}/glove_{glove_id}'}]),
        Node(package='manus_hand_viz', executable='hand_viz', output='screen'),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('manus_hand_viz'), 'config', 'hands.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
