import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Main package
    pkg = get_package_share_directory('my_components')

    # Manual composition node configuration
    declare_obstacle = DeclareLaunchArgument(
        'obstacle', default_value='0.3', description='Obstacle distance threshold')
    declare_degrees = DeclareLaunchArgument(
        'degrees', default_value='-90', description='Rotation after reaching obstacle')
    #declare_approach = DeclareLaunchArgument(
    #    'final_approach', default_value='false', description='Rotation after reaching obstacle')

    manual_node = Node(
            package='my_components',
            executable='manual_composition',
            output='screen',
            name='manual_composition',
            parameters=[{
                'obstacle': LaunchConfiguration('obstacle'),
                'degrees': LaunchConfiguration('degrees')}])

    # RVIZ configuration
    rviz_config_dir = os.path.join(pkg, 'rviz', 'config.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    return LaunchDescription([
        declare_obstacle,
        declare_degrees,
        #declare_approach,
        rviz_node,
        manual_node
    ])