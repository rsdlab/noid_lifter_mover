import os
import shutil
import yaml
import time
from distutils.util import strtobool
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, OpaqueFunction, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from rclpy.node import Node as RclpyNode


def replace_usb_settings(fpath_in, fpath_out):
    with open(fpath_in) as in_file:
        config = yaml.safe_load(in_file)
    for usb_setting in config["usb_settings"]:
        usb_setting["port"] = "./tmp" + usb_setting["port"]
    with open(fpath_out, "w") as out_file:
        yaml.dump(config, out_file, default_flow_style=False)


def load_driver_settings(context, *args, **kwargs):
    simulation = LaunchConfiguration("simulation")
    driver_settings_file_raw = kwargs["driver_settings_raw"].perform(context)
    driver_settings_file = kwargs["driver_settings"].perform(context)
    simu = bool(strtobool(simulation.perform(context)))
    if simu:
        replace_usb_settings(driver_settings_file_raw, driver_settings_file)
    else:
        shutil.copy(driver_settings_file_raw, driver_settings_file)


def interpret_robot_model(driver_settings_file, robot_pkg):
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([robot_pkg, "model", "noid_lifter_mover.urdf.xacro"]),
        " ",
        "driver_settings:=",
        driver_settings_file,
    ])
    robot_description = {"robot_description": robot_description_content}
    return robot_description


def bringup_stub(driver_settings_file, description, condition):
    ms_stub_node = Node(
        package="ms_stub",
        name='ms_stub',
        executable="ms_stub",
        arguments=[driver_settings_file],
        output="screen",
        condition=condition,
    )
    description.add_action(ms_stub_node)


def call_launch(name, description, robot_pkg, delay_time=0, extra_args=None):
    launch_arguments = {
        'robot_pkg_path': PathJoinSubstitution([robot_pkg])
    }
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
    if delay_time != 0:
        delayed_controller_node = TimerAction(
            period=delay_time,
            actions=[action]
        )
        description.add_action(delayed_controller_node)
    else:
        description.add_action(action)


def launch_setup(context, *args, **kwargs):
    pkg_name = kwargs["pkg_name"]
    robot_pkg = FindPackageShare(pkg_name).perform(context)

    controller_defs = [
        {
            "name": "rarm_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"),
            "remappings": []
        },
        {
            "name": "larm_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"),
            "remappings": []
        },
        {
            "name": "head_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"),
            "remappings": []
        },
        {
            "name": "waist_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"),
            "remappings": []
        },
        {
            "name": "lifter_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"),
            "remappings": []
        },
        {
            "name": "rhand_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"),
            "remappings": []
        },
        {
            "name": "lhand_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"),
            "remappings": []
        },
        {
            "name": "joint_state_broadcaster",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_state.yaml"),
            "remappings": []
        },
        {
            "name": "mechanum_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_mechanum.yaml"),
            "remappings": [("~/cmd_vel", "/cmd_vel_nav")]
        },
        {
            "name": "diagnostic_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_mechanum.yaml"),
            "remappings": []
        },
        {
            "name": "aero_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_mechanum.yaml"),
            "remappings": []
        },
        {
            "name": "status_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_mechanum.yaml"),
            "remappings": []
        },
        {
            "name": "robotstatus_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_mechanum.yaml"),
            "remappings": []
        },
        {
            "name": "motion_player",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_mechanum.yaml"),
            "remappings": []
        },
        {
            "name": "config_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_mechanum.yaml"),
            "remappings": []
        },
    ]

    priority_controllers = ["mechanum_controller", "joint_state_broadcaster", "lifter_controller", "lhand_controller",  "rhand_controller", "waist_controller", "head_controller", "larm_controller", "rarm_controller"]
    priority_defs = [ctrl for ctrl in controller_defs if ctrl["name"] in priority_controllers]
    non_priority_defs = [ctrl for ctrl in controller_defs if ctrl["name"] not in priority_controllers]

    actions = []

    delay_sec = 0.0
    for ctrl in priority_defs:
        spawner = Node(
            package="controller_manager",
            executable="spawner",
            name=f"{ctrl['name']}_spawner",
            output="screen",
            arguments=[
                ctrl["name"],
                "--controller-manager", "/controller_manager",
                "-p", ctrl["param_file"]
            ],
            remappings=ctrl["remappings"]
        )
        actions.append(TimerAction(period=delay_sec, actions=[spawner]))
        delay_sec += 0.2

    non_priority_spawners = []
    for ctrl in non_priority_defs:
        spawner = Node(
            package="controller_manager",
            executable="spawner",
            name=f"{ctrl['name']}_spawner",
            output="screen",
            arguments=[
                ctrl["name"],
                "--controller-manager", "/controller_manager",
                "-p", ctrl["param_file"]
            ],
            remappings=ctrl["remappings"]
        )
        non_priority_spawners.append(spawner)

    actions.append(TimerAction(period=delay_sec, actions=non_priority_spawners))

    return actions


def generate_launch_description():
    simulation = LaunchConfiguration('simulation')
    pkg_name = LaunchConfiguration('pkg_name')

    simulation_arg = DeclareLaunchArgument('simulation')
    pkg_name_arg = DeclareLaunchArgument('pkg_name')

    robot_pkg = FindPackageShare(pkg_name)

    driver_settings_raw = PathJoinSubstitution([robot_pkg, 'config', 'driver_settings.yaml'])
    driver_settings = PathJoinSubstitution([robot_pkg, 'config', 'driver_settings_tmp.yaml'])
    controller_settings = PathJoinSubstitution([robot_pkg, 'config', 'controller_settings.yaml'])
    teleop_settings = PathJoinSubstitution([robot_pkg, 'config', 'teleop', 'teleop_settings.yaml'])
    lidar_settings = PathJoinSubstitution([robot_pkg, 'config', 'laser', 'laser_filter.yaml'])

    ld = LaunchDescription()
    ld.add_action(simulation_arg)
    ld.add_action(pkg_name_arg)

    ld.add_action(Node(
        package=pkg_name,
        executable="tf_static_monitor_node",
        name="tf_static_monitor_node",
        output="screen",
    ))
    ld.add_action(Node(
        package=pkg_name,
        executable="tf_static_ready_monitor_node",
        name="tf_static_ready_monitor_node",
        output="screen",
        parameters=[{'cm_param_path': controller_settings}, {'teleop_param_path': teleop_settings}, {'lidar_param_path': lidar_settings}, {'simulation': simulation}],
    ))

    ld.add_action(OpaqueFunction(function=load_driver_settings, kwargs={
        "driver_settings_raw": driver_settings_raw,
        "driver_settings": driver_settings
    }))

    robot_description = interpret_robot_model(driver_settings, robot_pkg)

    ld.add_action(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    ))

    bringup_stub(driver_settings, ld, condition=IfCondition(simulation))

    ld.add_action(OpaqueFunction(function=launch_setup, kwargs={"pkg_name": pkg_name}))

    return ld
