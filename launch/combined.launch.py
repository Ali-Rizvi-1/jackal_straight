# launch/combined.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jackal_straight',
            executable='straight_stop',
            name='straight_stop', namespace='jackal_0329',
            output='screen',
            parameters=[{
                'distance_threshold': 0.9,
                'forward_speed':      0.1,
                'stop_duration':     5.0
            }]
        ),
        Node(
            package='jackal_straight',
            executable='heading_maintainer',
            name='heading_maintainer', namespace='jackal_0329',
            output='screen',
            parameters=[{'P':4.0,'I':0.0,'D':0.1}]
        ),
    ])
