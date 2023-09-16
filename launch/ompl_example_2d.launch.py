import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    map_path = os.path.join(get_package_share_directory('hybrid_planning'), 'maps', 'map_with_many_homotopy_classes.yaml')

    map_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hybrid_planning'), 'launch', 'map_launch.py')
        ),
        launch_arguments={'map': map_path}.items()
    )


    rviz_config_dir = os.path.join(
        get_package_share_directory('hybrid_planning'), 'rviz', 'rviz_cfg.rviz')

    rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen')
    
    ompl_example_cmd = Node(
            package='hybrid_planning',
            executable='ompl_example_2d_node',
            name='ompl_example_2d_node',
            output='screen')

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(ompl_example_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(map_server_cmd)

    return ld
