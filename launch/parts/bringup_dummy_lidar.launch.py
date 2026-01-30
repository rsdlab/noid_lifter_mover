from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import  DeclareLaunchArgument, Shutdown
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def bringup_dummy_lidar(dummy_map_file, description):
    lidar1 = Node(
        package="dummy_scan",
        executable="dummy_scan",
        output="screen",
        remappings=[('/scan', '/scan_raw')]
        )

    scan_map = Node(
                package='nav2_map_server',
                executable='map_server',
                name='scan_map_server',
                output='screen',
                parameters=[{'yaml_filename': dummy_map_file}],
                remappings=[('/map', '/scan_map')])
                
    lifecycle = Node(
                 package='nav2_lifecycle_manager',
                 executable='lifecycle_manager',
                 name='lifecycle_manager_scan',
                 output='log',
                 parameters=[{'use_sim_time': False},
                             {'autostart': True},
                             {'node_names': ['scan_map_server']}])

    description.add_action(lidar1)
    description.add_action(scan_map)
    description.add_action(lifecycle)
    
    description.add_action(Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name='front_laser_filter',
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("noid_lifter_mover"),
                    "config","laser", "laser_filter.yaml",
                ])],
            remappings=[('/scan', '/scan_raw'),('/scan_filtered', '/scan')],
        )
    )
    

def generate_launch_description():
    dummy_map_file = LaunchConfiguration("dummy_map")
    dummy_map_file_arg = DeclareLaunchArgument("dummy_map")

    ld = LaunchDescription()
    ld.add_action(dummy_map_file_arg)
    
    bringup_dummy_lidar(dummy_map_file, ld)

    return ld
