#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def check_device_exists(device_path):
    """Check if camera device exists"""
    # Check if the symlink exists
    if not os.path.exists(device_path):
        return False
    # Check if it's a valid symlink pointing to a real device
    if os.path.islink(device_path):
        real_path = os.readlink(device_path)
        if not os.path.isabs(real_path):
            # Relative symlink, resolve it
            real_path = os.path.join(os.path.dirname(device_path), real_path)
        return os.path.exists(real_path)
    return os.path.exists(device_path)

def create_camera_node(camera_config):
    return Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name=f"{camera_config['camera_name']}_node",
        parameters=[
            {
                'video_device': camera_config['device_path'],
                'camera_id': camera_config['camera_id'],
                'image_size': camera_config['image_size'],
                'time_per_frame': camera_config['time_per_frame']
            }
        ],
        remappings=[
            ('image_raw', camera_config['topic_name'])
        ]
    )

def generate_launch_description():
    # Common parameters for both cameras
    common_params = {
        'image_size': [640, 480],
        'time_per_frame': [1, 50],
    }
    
    # Configuration for both cameras
    v4l2_configs = [
        {  # Left camera
            'camera_name': 'cam_left_wrist',
            'device_path': '/dev/uvc_camera/camera_left',
            'camera_id': 'left_cam',
            'topic_name': '/cam_left_wrist/color/image_rect_raw',
            **common_params
        },
        {  # Right camera
            'camera_name': 'cam_right_wrist',
            'device_path': '/dev/uvc_camera/camera_right',
            'camera_id': 'right_cam',
            'topic_name': '/cam_right_wrist/color/image_rect_raw',
            **common_params
        },
        {  # High camera
            'camera_name': 'cam_fish_top',
            'device_path': '/dev/uvc_camera/camera_top',
            'camera_id': 'top_cam',
            'topic_name': '/cam_fish_top/color/image_rect_raw',
            **common_params
        }
    ]

    # Create nodes only for existing cameras
    nodes = []
    for config in v4l2_configs:
        device_path = config['device_path']
        if check_device_exists(device_path):
            nodes.append(create_camera_node(config))
            print(f"✅ Found camera: {config['camera_name']} at {device_path}")
        else:
            print(f"⚠️  Camera device not found: {config['camera_name']} at {device_path}")
            print(f"   Please check USB connection and udev rules")
    
    if not nodes:
        print("❌ No camera devices found. Please check:")
        print("   1. USB connections")
        print("   2. udev rules: sudo udevadm control --reload-rules && sudo udevadm trigger")
        print("   3. Device permissions: ls -la /dev/uvc_camera/")
        return LaunchDescription([])

    return LaunchDescription(nodes)

if __name__ == '__main__':
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
