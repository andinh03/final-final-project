from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Our trash navigator node
    trash_navigator_node = Node(
        package='bwi_nav_test',
        executable='trash_navigator',
        name='trash_navigation_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        trash_navigator_node,
        rviz_node
    ])
