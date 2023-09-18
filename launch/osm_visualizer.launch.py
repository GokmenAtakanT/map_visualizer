'''
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="map_visualizer",
            executable="osm_visualizer",
            name="osm_visualizer",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"map_path": "/home/atakan/vuran_ws/src/vuran_simulation/mapping/map_visualizer/osm/Town10.osm"},
                {'enable_inc_path_points', 'True'},
                {"interval", "2.0"}
            ]
        )
    ])'''

import launch
import launch.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from launch_ros.actions import Node

def generate_launch_description():

    # Declare a variable Node for each node
    DeclareLaunchArgument(
        'arg_name',
        default_value='default_value',  # Set default value if needed
        description='Description of the argument'
    ),

    visualizer_node = (Node
        (
        package="map_visualizer",
        executable="osm_visualizer_node",
        name='osm_visualizer_node',
        output='screen',  # Choose 'log' or 'screen' depending on your preference
        parameters=[ {"map_path": "/home/atakan/vuran_ws/src/vuran_simulation/mapping/map_visualizer/osm/Town10.osm"},
                     {'enable_inc_path_points', 'True'},
                     {"interval", "2.0"}
                   ]  # Add parameters if needed
        )
    )
    occupancy_node = Node(
        package="map_visualizer",
        executable="occupancyGrid_node",
        name='occupancyGrid_node',
        output='screen',  # Choose 'log' or 'screen' depending on your preference
        parameters=[{'param_name': 'param_value'}]  # Add parameters if needed
    )
    # Add the nodes and the process to the LaunchDescription list
    ld = [visualizer_node,occupancy_node]
    return LaunchDescription(ld)


if __name__ == '__main__':
    generate_launch_description()