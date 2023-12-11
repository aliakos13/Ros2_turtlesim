from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Run turtlesim_node before koch_snow
        ExecuteProcess(
            cmd=['ros2', 'run', 'turtlesim', 'turtlesim_node'],
            output='screen',
        ),

        # Start koch_snow node
        Node(
            package='ros2_course',
            executable='koch_snow',  # Replace with your actual Python script
            name='koch_snow',
            output='screen',
        ),
    ])

