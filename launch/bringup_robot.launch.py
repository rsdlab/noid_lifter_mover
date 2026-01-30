import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import RegisterEventHandler, Shutdown

def bringup_rviz(robot_pkg, display_rviz2, context):
    rviz_config_file = PathJoinSubstitution([robot_pkg, "rviz", "rviz.rviz"]).perform(context)
    kinematics_yaml_path = PathJoinSubstitution([robot_pkg, "moveit", "config", "kinematics.yaml"]).perform(context)
    with open(kinematics_yaml_path, "r") as f:
        kinematics_content = yaml.safe_load(f)
    
    kinematics_dict = {
        "robot_description_kinematics": kinematics_content
    }

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
        parameters=[kinematics_dict],
        condition=IfCondition(display_rviz2)
    )
    return [rviz_node]

def call_launch(name, description, robot_pkg, extra_args=None):
    launch_arguments = {'robot_pkg_path': PathJoinSubstitution([robot_pkg])}

    if extra_args:
        launch_arguments.update(extra_args)

    launch_file_path = PathJoinSubstitution([
        robot_pkg,
        'launch',
        'parts',
        name
    ])

    action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path),
        launch_arguments=[(key, value) for key, value in launch_arguments.items()]
    )
    return action

def generate_launch_description():
    pkg_name = 'noid_lifter_mover'
    
    robot_pkg = FindPackageShare(pkg_name)
    robot_pkg_path = get_package_share_directory(pkg_name)
    ld = LaunchDescription()
    
    simulation = LaunchConfiguration('simulation')
    slam_mode = LaunchConfiguration('slam')
    display_rviz2 = LaunchConfiguration('display_rviz2')
    change_slam_mode = LaunchConfiguration('slam_mode')

    simulation_arg = DeclareLaunchArgument('simulation', default_value='false')
    slam_mode_arg = DeclareLaunchArgument('slam', default_value='false')
    change_slam_mode_arg = DeclareLaunchArgument('slam_mode', default_value='async')
    display_rviz2_arg = DeclareLaunchArgument('display_rviz2', default_value='true')
    
    ld.add_action(simulation_arg)
    ld.add_action(slam_mode_arg)
    ld.add_action(change_slam_mode_arg)    
    ld.add_action(display_rviz2_arg)    

    read_map_yaml_file = PathJoinSubstitution([robot_pkg_path, 'config', 'navigation', 'map', 'scan_map.yaml'])
    bringup_nav_monitor_node = Node(
        package=pkg_name,
        executable='bringup_navigation_monitor_node',
        name='bringup_navigation_monitor_node',
        output='screen',
        parameters=[
            {'robot_pkg_path': robot_pkg_path},
            {'simulation': simulation},
            {'slam': slam_mode},
            {'map': read_map_yaml_file},
        ],
    )
    ld.add_action(bringup_nav_monitor_node)
    
    bringup_tools_node = Node(
        package=pkg_name,
        executable='amcl_state_monitor_node',
        name='amcl_state_monitor_node',
        output='screen',
        condition=UnlessCondition(slam_mode)
    )
    ld.add_action(bringup_tools_node)

    controllers_state_monitor_node = Node(
        package=pkg_name,
        executable='controllers_state_monitor_node',
        name='controllers_state_monitor_node',
        output='screen',
        parameters=[
            {'robot_pkg_path': robot_pkg_path},
            {'dummy_map': read_map_yaml_file},
        ],
        condition=IfCondition(simulation)
    )
    ld.add_action(controllers_state_monitor_node)
    
    bringup_moveit_monitor_node = Node(
        package=pkg_name,
        executable='bringup_moveit_monitor_node',
        name='bringup_moveit_monitor_node',
        output='screen',
        parameters=[
            {'pkg_name': pkg_name},
            {'robot_pkg_path': robot_pkg_path},
        ]
    )
    ld.add_action(bringup_moveit_monitor_node)

    bringup_robot_model_node = call_launch("bringup_robot_model.launch.py", ld, robot_pkg, extra_args={'simulation': simulation,'pkg_name': pkg_name})
    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=bringup_moveit_monitor_node, 
            on_start=[bringup_robot_model_node],
        )
    ))

    bringup_rviz2_monitor_node = Node(
        package=pkg_name,
        executable="bringup_rviz2_monitor_node",
        name="bringup_rviz2_monitor_node",
        output='screen'
    )
    ld.add_action(bringup_rviz2_monitor_node)

    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=bringup_rviz2_monitor_node, 
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[OpaqueFunction(
                        function=lambda context: bringup_rviz(robot_pkg, display_rviz2, context)
                    )]
                )
            ],
        )
    ))

    return ld
