from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qt_robot_control',
            executable='qt_robot_control',
            name='qt_robot_control',
            output='screen',
        ),
    ])


