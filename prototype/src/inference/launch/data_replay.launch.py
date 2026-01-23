from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get inference package share directory for default config
    try:
        inference_share_dir = get_package_share_directory('inference')
        default_config_path = os.path.join(
            inference_share_dir,
            'config',
            'replay_config.yaml'
        )
    except:
        # Fallback if inference package not found, use relative path
        default_config_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'config',
            'replay_config.yaml'
        )
    
    # Declare launch arguments
    hdf5_file_arg = DeclareLaunchArgument(
        'hdf5_file',
        description='Path to HDF5 file to replay'
    )
    
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Path to replay config YAML file'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='Whether to loop the replay'
    )
    
    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='yam',
        description='Arm type: yam or arx'
    )
    
    # Create node
    replay_node = Node(
        package='inference',
        executable='data_replay_node',
        name='data_replay_node',
        output='screen',
        parameters=[{
            'hdf5_file': LaunchConfiguration('hdf5_file'),
            'config_path': LaunchConfiguration('config_path'),
            'loop': LaunchConfiguration('loop'),
            'arm_type': LaunchConfiguration('arm_type'),
        }]
    )
    
    return LaunchDescription([
        hdf5_file_arg,
        config_path_arg,
        loop_arg,
        arm_type_arg,
        replay_node,
    ])
