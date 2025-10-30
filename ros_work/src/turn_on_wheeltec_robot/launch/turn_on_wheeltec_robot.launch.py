import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
        
    ekf_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'ekf.yaml')
    ekf_carto_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'ekf_carto.yaml')

    imu_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'imu.yaml')

    
    carto_slam = LaunchConfiguration('carto_slam', default='false')
    use_custom_urdf = LaunchConfiguration('use_custom_urdf', default='true')
    custom_urdf_path = LaunchConfiguration('custom_urdf_path', default='/home/HwHiAiUser/test/test_09/urdf/test_09.urdf')
    
    carto_slam_dec = DeclareLaunchArgument('carto_slam',default_value='false')
    use_custom_urdf_dec = DeclareLaunchArgument('use_custom_urdf', default_value='true')
    custom_urdf_path_dec = DeclareLaunchArgument('custom_urdf_path', default_value='/home/HwHiAiUser/test/test_09/urdf/test_09.urdf')
            
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
    )
     
    robot_ekf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_ekf.launch.py')),
            launch_arguments={'carto_slam':carto_slam}.items(),            
    )
    #Modify the z-axis transformation parameters of the base_to_link node according to the vehicle model:
    #(r3s_mec,0.02027), (r3s_4wd,0.01463), 
    #(mini_akm,0.0216), (mini_diff,0.01563),(mini_tank,0.03715), (mini_mec,0.07258), (mini_omni,0.06542), (mini_4wd,0.0682),
    #(brushless_senior_diff,0.07348), (S100_diff,0.0277), 
    #(senior_akm,0.03826), (senior_mec_bs,0.02391), (senior_4wd_bs,0.04994), (senior_omni,0.09016),
    #(senior_mec_dl,0.05882), (senior_4wd_dl,0.05823),(senior_diff,0.0374),  
    #(top_akm_bs,0.049), (top_mec_bs,0.05274), (top_4wd_bs,0.08786), (top_omni,0.0192), (top_akm_dl,0.11744), 
    #(top_mec_dl,0.03802), (top_4wd_dl,0.07454), (top_mec_EightDrive,0.03553),
    #(flagship_mec_bs,0.11274), (four_wheel_diff_bs,0.14153),(four_wheel_diff_dl,0.06887), 
    #(flagship_4wd_bs,0.14917), (flagship_mec_dl,0.03816), (flagship_4wd_dl_robot,0.07384)  
    #e.g. 'minibot_type:mini_akm'
    #arguments=['0', '0', '0.0216','0', '0','0','base_footprint','base_link'] 
    base_to_link = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_link',
            arguments=['0', '0', '0','0', '0','0','base_footprint','base_link'],
    )
    base_to_gyro = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_gyro',
            arguments=['0', '0', '0','0', '0','0','base_footprint','gyro_link'],
    )
    
    imu_filter_node =  launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_config]
    )
    
                           
    joint_state_publisher_node = launch_ros.actions.Node(
            package='joint_state_publisher', 
            executable='joint_state_publisher', 
            name='joint_state_publisher',
    )
    
    #select a robot model,the default model is mini_mec 
    #minibot.launch.py contains commonly used robot models
    #launch_arguments choices:r3s_mec/r3s_4wd/mini_mec/mini_akm/mini_tank/mini_4wd/mini_diff/mini_omni/brushless_senior_diff
    #!!!At the same time, you need to modify ld.add_action(minibot_type) and #ld.add_action(flagship_type)
    minibot_type = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description_minibot.launch.py')),
            launch_arguments={'mini_4wd': 'true'}.items(),
            condition=UnlessCondition(use_custom_urdf)
    )

    # When using custom URDF, read the file content
    def load_urdf(file_path):
        """Load URDF file content"""
        try:
            with open(file_path, 'r') as f:
                return f.read()
        except Exception as e:
            print(f"Error loading URDF: {e}")
            return ""
    
    # Get the actual file path value
    urdf_content = load_urdf('/home/HwHiAiUser/test/test_09/urdf/test_09.urdf')
    
    # When using custom URDF, publish it instead of built-in models
    custom_robot_description = launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': urdf_content,
            }],
            condition=IfCondition(use_custom_urdf)
    )


    #robot_mode_description.launch.py contains flagship products, usually larger chassis robots
    #launch_arguments choices:

    #senior_akm/top_akm_bs/top_akm_dl
    #senior_mec_bs/senior_mec_dl/top_mec_bs/top_mec_dl/ mec_EightDrive_robot/flagship_mec_bs_robot/flagship_mec_dl_robot??
    #senior_omni/top_omni
    #senior_4wd_bs_robot/senior_4wd_dl_robot/flagship_4wd_bs_robot/flagship_4wd_dl_robot/top_4wd_bs_robot/top_4wd_dl_robot
    #senior_diff_robot/four_wheel_diff_bs/four_wheel_diff_dl/flagship_four_wheel_diff_bs_robot/flagship_four_wheel_diff_dl_robot
    #!!!At the same time, you need to modify ld.add_action(flagship_type) and #ld.add_action(minibot_type)
    flagship_type = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
            launch_arguments={'senior_akm': 'true'}.items(),
    )
    
    ld = LaunchDescription()

    ld.add_action(use_custom_urdf_dec)
    ld.add_action(custom_urdf_path_dec)
    ld.add_action(minibot_type)
    ld.add_action(custom_robot_description)
    #ld.add_action(flagship_type)
    ld.add_action(carto_slam_dec)
    ld.add_action(wheeltec_robot)
    ld.add_action(base_to_link)
    ld.add_action(base_to_gyro)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(imu_filter_node)    
    ld.add_action(robot_ekf)

    return ld

