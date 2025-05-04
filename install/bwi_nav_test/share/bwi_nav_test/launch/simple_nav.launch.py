from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Launch file for simple navigation testing"""
    
    # Launch Nav2 if needed (you may already have this running on the BWI robot)
    # Uncomment and modify as needed for your specific setup
    """
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file_dir = os.path.join(nav2_bringup_dir, 'launch')
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_file_dir, 'bringup_launch.py')
        )
    )
    """
    
    return LaunchDescription([
        # Launch the simple navigation node
        Node(
            package='bwi_nav_test',
            executable='trash_navigator',
            name='trash_navigator',
            output='screen',
            emulate_tty=True
        ),
        
        # Launch RViz with default configuration
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen'
        # )
    ])