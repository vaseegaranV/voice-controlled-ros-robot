from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Set TURTLEBOT3_MODEL=burger
        SetEnvironmentVariable(
            'TURTLEBOT3_MODEL',
            'burger'
        ),

        SetEnvironmentVariable(
            'QT_QPA_PLATFORM',
            'xcb'
        ),

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
        
        # Launch Gazebo world
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'],
            output='screen'
        ),
        
        # Launch Nav2 after 10 seconds delay
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                        'use_sim_time:=True',
                        'map:=/home/vaseegaran/ros2_ws/src/smart_nav_bot/maps/my_house_map.yaml'
                    ],
                    output='screen'
                )
            ]
        )
    ])