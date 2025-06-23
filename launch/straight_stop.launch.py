from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jackal_straight',
            executable='straight_stop',   # must match your entry_point name
            name='straight_stop',
            namespace='jackal_0329',
            output='screen',
            parameters=[{
                'distance_threshold': 0.09,
                'forward_speed':      0.1,
                'stop_duration':     10.0
            }]
        )
    ])
