from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_follower',
            executable='drone_detection_follower',
            name='drone_detection_follower',
            output='screen'
        ),
    ])
