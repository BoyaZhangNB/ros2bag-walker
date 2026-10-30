"""Camera -> ROS 2 topics -> rosbag2 + TCP stream to the server.

Example:
    ros2 launch walker_camera walker.launch.py \
        camera_url:=rtsp://admin:pass@192.168.1.64:554/stream1 \
        server_host:=192.168.1.100
"""
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('walker_camera'), 'config', 'camera.yaml')
    bag_name = datetime.now().strftime('walker_%Y%m%d_%H%M%S')

    args = [
        DeclareLaunchArgument('camera_url', default_value='rtsp://192.168.1.64:554/stream1'),
        DeclareLaunchArgument('codec', default_value='h264'),
        DeclareLaunchArgument('server_host', default_value='192.168.33.203'),
        DeclareLaunchArgument('server_port', default_value='5600'),
        DeclareLaunchArgument('stream', default_value='true',
                              description='Send frames to the remote server'),
        DeclareLaunchArgument('record', default_value='true',
                              description='Record a rosbag'),
        DeclareLaunchArgument('bag_dir', default_value=os.path.expanduser('~/bags')),
        DeclareLaunchArgument('storage', default_value='sqlite3',
                              description='sqlite3 or mcap (needs ros-humble-rosbag2-storage-mcap)'),
        DeclareLaunchArgument('max_bag_duration', default_value='300',
                              description='Split the bag every N seconds so a power cut loses at most one chunk'),
        DeclareLaunchArgument('topics', default_value='/camera/image_raw/compressed',
                              description='Space-separated topics to record'),
    ]

    camera = Node(
        package='walker_camera',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[config, {
            'camera_url': LaunchConfiguration('camera_url'),
            'codec': LaunchConfiguration('codec'),
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    sender = Node(
        package='walker_camera',
        executable='stream_sender',
        name='stream_sender',
        output='screen',
        parameters=[config, {
            'server_host': LaunchConfiguration('server_host'),
            'server_port': LaunchConfiguration('server_port'),
        }],
        condition=IfCondition(LaunchConfiguration('stream')),
        respawn=True,
        respawn_delay=2.0,
    )

    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '-s', LaunchConfiguration('storage'),
             '-d', LaunchConfiguration('max_bag_duration'),
             '-o', PathJoinSubstitution([LaunchConfiguration('bag_dir'), bag_name]),
             LaunchConfiguration('topics')],
        shell=True,  # so the space-separated topics list expands into separate args
        output='screen',
        condition=IfCondition(LaunchConfiguration('record')),
    )

    return LaunchDescription(args + [camera, sender, record])
