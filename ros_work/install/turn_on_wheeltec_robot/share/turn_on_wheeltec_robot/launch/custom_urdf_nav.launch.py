import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 参数：URDF路径、地图与Nav2参数
    urdf_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='/home/wheeltec/test/test_09/urdf/test_09.urdf',
        description='Absolute path to custom URDF file')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file = LaunchConfiguration('urdf_file')

    # 载入URDF文本到robot_description
    # 注意：直接读取文件内容传给robot_state_publisher
    def load_urdf(context):
        path = context.perform_substitution(urdf_file)
        with open(path, 'r') as f:
            return f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': load_urdf}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # 引入底盘串口、EKF、激光雷达
    turn_on_dir = get_package_share_directory('turn_on_wheeltec_robot')
    base_serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turn_on_dir, 'launch', 'base_serial.launch.py')),
    )
    wheeltec_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turn_on_dir, 'launch', 'wheeltec_ekf.launch.py')),
    )
    wheeltec_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turn_on_dir, 'launch', 'wheeltec_lidar.launch.py')),
    )

    # Nav2 bringup
    nav2_dir = get_package_share_directory('wheeltec_nav2')
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # 可按需修改地图与参数文件，默认复用wheeltec_nav2内部默认
            # 也可在命令行通过 map:=xxx.yaml params_file:=yyy.yaml 覆盖
        }.items()
    )

    return LaunchDescription([
        urdf_arg,
        joint_state_publisher,
        robot_state_publisher,
        base_serial_launch,
        wheeltec_lidar_launch,
        wheeltec_ekf_launch,
        bringup_launch,
    ])






