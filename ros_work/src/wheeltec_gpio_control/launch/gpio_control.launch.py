from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成launch描述"""
    gpio_control_node = Node(
        package='wheeltec_gpio_control',
        executable='gpio_control_node',
        name='gpio_control_node',
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        gpio_control_node,
    ])


