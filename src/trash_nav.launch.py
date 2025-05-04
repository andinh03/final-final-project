from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch file for the trash navigation system
    """
    return LaunchDescription([
        # Launch the trash navigation node
        Node(
            package='bwi_trash_navigation',  # Replace with your actual package name
            executable='trash_navigation_node',
            name='trash_navigation_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                # Add any parameters here
            ]
        ),
        
        # You could include RViz with a specific configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/path/to/your/config.rviz'],  # Replace with actual path
            output='screen'
        )
    ])