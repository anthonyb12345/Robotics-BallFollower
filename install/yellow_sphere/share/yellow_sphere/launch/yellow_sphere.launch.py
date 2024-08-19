from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='/usr/bin/gzserver',  # Full path to gzserver
            name='gazebo_server',
            output='screen',
            parameters=[{'world': '/home/anthony/Desktop/Assignment12/my_custom_world.world'}]
        ),
        Node(
            package='gazebo_ros',
            executable='/usr/bin/gzclient',  # Full path to gzclient
            name='gazebo_client',
            output='screen'
        ),
    ])

