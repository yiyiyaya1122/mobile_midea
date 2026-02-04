from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Default: main config, streaming HDF5
    config_path = PathJoinSubstitution([
        FindPackageShare('data_collector'),
        'config',
        'collect_config.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=config_path,
            description='Path to collect config YAML (datasets, topics, etc.)',
        ),
        DeclareLaunchArgument(
            'use_streaming',
            default_value='true',
            description='If true, stream write to HDF5; if false, batch write on stop.',
        ),
        Node(
            package='data_collector',
            executable='data_collector_node',
            name='data_collector_node',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'use_streaming': LaunchConfiguration('use_streaming'),
            }],
        ),
    ])