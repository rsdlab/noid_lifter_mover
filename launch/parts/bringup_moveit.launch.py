from launch import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder 
from moveit_configs_utils.moveit_configs_builder import load_yaml
from launch.actions import Shutdown

def bringup_moveit(context, *args, **kwargs):
    import os
    from ament_index_python.packages import get_package_share_directory
    pkg_name = kwargs["pkg_name"].perform(context)
    # 下限0の関節が静止時の数値ノイズ(~1e-14)で範囲外判定されSTART_STATE_INVALIDに
    # なるのを防ぐ(シミュレーションで必須。実機はデフォルトfalseのまま)
    fix_start_state = kwargs["fix_start_state"].perform(context).lower() == 'true'
    xacro_settings = kwargs["xacro_settings"].perform(context)
    # ファイル名のみの場合はパッケージのmodelディレクトリから解決(実機デフォルト)。
    # シミュレーションは生成済みURDFの絶対パスを渡してくる
    if not os.path.isabs(xacro_settings):
        xacro_settings = os.path.join(
            get_package_share_directory(pkg_name), 'model', xacro_settings)
    srdf_settings = kwargs["srdf_settings"].perform(context)
    kinematics_settings = kwargs["kinematics_settings"].perform(context)
    moveit_controller_settings = kwargs["moveit_controller_settings"].perform(context)
    pilz_cartesian_limits = kwargs["pilz_cartesian_limits"].perform(context)
    joint_limits_settings = kwargs["joint_limits_settings"].perform(context)
    
    actions = []    
    moveit_config = (
        MoveItConfigsBuilder(robot_name=pkg_name, package_name=pkg_name)
        .robot_description(file_path=xacro_settings)
        .robot_description_semantic(file_path=srdf_settings)
        .robot_description_kinematics(file_path=kinematics_settings)
        .trajectory_execution(file_path=moveit_controller_settings)
        .planning_scene_monitor(True,True,True,True,True,True)
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits)
        .planning_pipelines( default_planning_pipeline= "ompl",pipelines=['ompl'])
        .joint_limits(file_path=joint_limits_settings)
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        sigterm_timeout="20",
        sigkill_timeout="20",
        parameters=[
            moveit_config.to_dict(),
            {'ompl.fix_start_state': fix_start_state},
        ],
    )
    actions.append(move_group_node)    
    return actions

def generate_launch_description():
    pkg_name = LaunchConfiguration('pkg_name')
    pkg_name_arg = DeclareLaunchArgument('pkg_name')
    fix_start_state = LaunchConfiguration('fix_start_state')
    fix_start_state_arg = DeclareLaunchArgument('fix_start_state', default_value='false')
    robot_pkg = FindPackageShare(pkg_name)
    # シミュレーションではsim用URDFの絶対パスを渡すこと(model_xacro引数)。move_groupは
    # /robot_descriptionを発行するため、gz_ros2_controlと異なるURDFだと
    # ハードウェアプラグインのロード競合が起きる
    model_xacro = LaunchConfiguration('model_xacro')
    model_xacro_arg = DeclareLaunchArgument('model_xacro', default_value='noid_lifter_mover.urdf.xacro')
    xacro_settings = model_xacro
    srdf_file = LaunchConfiguration('srdf_file')
    srdf_file_arg = DeclareLaunchArgument('srdf_file', default_value='SEED-Noid-Lifter-Mover-typeG.srdf')
    srdf_settings = PathJoinSubstitution([robot_pkg, 'moveit', 'config', srdf_file])
    kinematics_settings = PathJoinSubstitution([robot_pkg, 'moveit', 'config', 'kinematics.yaml'])
    moveit_controller_settings = PathJoinSubstitution([robot_pkg, 'moveit', 'config', 'moveit_controllers.yaml'])
    joint_limits_settings = PathJoinSubstitution([robot_pkg, 'moveit', 'config', 'joint_limits.yaml'])
    pilz_cartesian_limits = PathJoinSubstitution([robot_pkg, 'moveit', 'config', 'pilz_cartesian_limits.yaml'])

    ld = LaunchDescription()
    ld.add_action(pkg_name_arg)
    ld.add_action(fix_start_state_arg)
    ld.add_action(model_xacro_arg)
    ld.add_action(srdf_file_arg)
    ld.add_action(
    TimerAction(
        period=1.0,
        actions=[
            OpaqueFunction(
                function=bringup_moveit,
                kwargs={
                    "pkg_name": pkg_name,
                    "fix_start_state": fix_start_state,
                    "xacro_settings": xacro_settings,
                    "srdf_settings": srdf_settings,
                    "kinematics_settings": kinematics_settings,
                    "moveit_controller_settings": moveit_controller_settings,
                    "pilz_cartesian_limits": pilz_cartesian_limits,
                    "joint_limits_settings": joint_limits_settings,
                }
            )
        ]
    ))

    return ld
