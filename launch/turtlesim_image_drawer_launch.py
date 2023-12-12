from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Run turtlesim_node before image_drawer
        ExecuteProcess(
            cmd=['ros2', 'run', 'turtlesim', 'turtlesim_node'],
            output='screen',
        ),

        # Start image_drawer node
        Node(
            package='ros2_course',
            executable='image_drawer',
            name='image_drawer',
            output='screen',
        ),
    ])

