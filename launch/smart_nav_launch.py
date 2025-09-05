from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smart_nav_bot',
            executable='location_manager',
            name='location_manager',
            output='screen'
        ),
        Node(
            package='smart_nav_bot',
            executable='navigation_manager',
            name='navigation_manager',
            output='screen'
        ),
        Node(
            package='smart_nav_bot',
            executable='voice_controller.py',
            name='voice_controller',
            output='screen'
        )
    ])