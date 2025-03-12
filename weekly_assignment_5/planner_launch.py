import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Turtlesim başlatılıyor
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # Turtle için yol planlama düğümü (controller)
        Node(
            package='turtlesim_path_planner',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen'
        ),

        # Rastgele hedef belirleyen düğüm
        Node(
            package='turtlesim_path_planner',
            executable='goal_publisher',
            name='goal_publisher',
            output='screen'
        )
    ])
