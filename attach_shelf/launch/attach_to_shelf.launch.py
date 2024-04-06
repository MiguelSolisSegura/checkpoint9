import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    # Main package
    pkg = get_package_share_directory('attach_shelf')

    # Main node configuration
    declare_obstacle = DeclareLaunchArgument(
        'obstacle', default_value='0.3', description='Obstacle distance threshold')
    declare_degrees = DeclareLaunchArgument(
        'degrees', default_value='-90', description='Rotation after reaching obstacle')
    declare_approach = DeclareLaunchArgument(
        'final_approach', default_value='false', description='Rotation after reaching obstacle')

    main_node = Node(
            package='attach_shelf',
            executable='pre_approach_v2',
            output='screen',
            name='pre_approach_v2',
            parameters=[{
                'obstacle': LaunchConfiguration('obstacle'),
                'degrees': LaunchConfiguration('degrees'),
                'final_approach': LaunchConfiguration('final_approach')}])

    # Service configuration
    service_server = Node(
            package='attach_shelf',
            executable='approach_service_server',
            output='screen',
            name='approach_service_server')

    # RVIZ configuration
    rviz_config_dir = os.path.join(pkg, 'rviz', 'config.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    return LaunchDescription([
        declare_obstacle,
        declare_degrees,
        declare_approach,
        rviz_node,
        service_server,
        main_node
    ])