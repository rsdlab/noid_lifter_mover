import os
import re
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import math

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter


def controller_spawner(name):
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=[name, '--controller-manager', '/controller_manager'],
        output='screen',
    )


def setup(context, *args, **kwargs):
    pkg_name = 'noid_lifter_mover'
    pkg_share = get_package_share_directory(pkg_name)

    world = LaunchConfiguration('world').perform(context)
    use_nav = LaunchConfiguration('nav')
    use_slam = LaunchConfiguration('slam').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    spawn_x = LaunchConfiguration('x').perform(context)
    spawn_y = LaunchConfiguration('y').perform(context)
    spawn_yaw = LaunchConfiguration('yaw').perform(context)

    # URDF生成
    xacro_file = os.path.join(pkg_share, 'model', 'noid_lifter_mover_sim.urdf.xacro')
    robot_description = subprocess.run(
        ['xacro', xacro_file], check=True, capture_output=True, text=True
    ).stdout

    # リミットがちょうど0の関節は、静止時の数値ノイズ(~1e-14)で範囲外になり
    # MoveItがSTART_STATE_INVALIDで計画拒否するため、微小マージンを付ける
    robot_description = re.sub(r'\blower="(0|0\.0+)"', 'lower="-0.02"', robot_description)
    robot_description = re.sub(r'\bupper="(0|0\.0+)"', 'upper="0.02"', robot_description)
    # 実機の関節トルク上限(10Nm)では腕を伸ばした姿勢の重力に負けて垂れるため、
    # シミュレーションでは引き上げる(位置制御の追従性確保)
    robot_description = re.sub(r'\beffort="10"', 'effort="50"', robot_description)
    # waist_pは両腕前伸ばし時の重力負荷でgzの下限まで沈み込み、MoveItの
    # 境界チェックに常時引っかかるため、シムでは下限をさらに緩める
    robot_description = re.sub(
        r'(<joint name="waist_p_joint".*?lower=")[^"]*(")',
        r'\g<1>-0.15\g<2>', robot_description, flags=re.S)

    # move_groupにも同一URDFを使わせる(実機URDFだとgz_ros2_controlと
    # /robot_description が競合し、リミットも不整合になる)
    sim_urdf_path = os.path.join(tempfile.mkdtemp(prefix='noid_moveit_'), 'noid_sim.urdf')
    with open(sim_urdf_path, 'w') as f:
        f.write(robot_description)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {"-s --headless-rendering " if headless else ""}{world}'
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'noid_lifter_mover',
            '-x', spawn_x, '-y', spawn_y, '-z', '0.05',
            '-Y', spawn_yaw,
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # VelocityControlプラグインへのベース速度指令(cmd_vel_stamperが配信)
            '/model/noid_lifter_mover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
    )

    # スポーン完了後にコントローラーを順次起動
    jsb_spawner = controller_spawner('joint_state_broadcaster')
    mechanum_spawner = controller_spawner('mechanum_controller')
    other_spawners = [
        controller_spawner(n) for n in [
            'rarm_controller', 'larm_controller', 'head_controller',
            'waist_controller', 'lifter_controller',
            'rhand_controller', 'lhand_controller',
        ]
    ]

    spawner_chain = [
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[jsb_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[mechanum_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=mechanum_spawner, on_exit=other_spawners)),
    ]

    # mecanum_drive_controller は odom TF を ~/tf_odometry に出すため /tf へ中継する
    odom_tf_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_tf_relay',
        arguments=['/mechanum_controller/tf_odometry', '/tf'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ベース駆動の必須経路(ホイール+VelocityControl両方への配信とウォッチドッグ)のため常時起動
    cmd_vel_stamper = Node(
        package=pkg_name,
        executable='cmd_vel_stamper.py',
        name='cmd_vel_stamper',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'parts', 'bringup_navigation.launch.py')
        ),
        condition=IfCondition(use_nav),
        launch_arguments={
            'robot_pkg_path': pkg_share,
            'slam': 'True' if use_slam else 'False',
            'use_sim_time': 'True',
            'map': LaunchConfiguration('map'),
            # map->odom は SLAM時は slam_toolbox、それ以外は AMCL が出す
            'publish_static_map_odom_tf': 'False',
            'bond_timeout': '0.0',  # シミュレーションのCPU負荷でbondが切れるのを防ぐ
            # AMCLにactivate直後から自己位置(localization.yamlのinitial_pose)を持たせる。
            # これがないとmap->odom TFが出ずglobal_costmapのactivateがタイムアウトする
            'set_initial_pose': 'True',
            # composed bringupはロード要求が無言で停止することがあるため通常プロセスで起動する
            'use_composition': 'False',
        }.items(),
    )

    # MoveIt (move_group): 実機用launchを流用。コントローラー名/関節名はシムのJTCと一致済み
    moveit = GroupAction(
        [
            SetParameter('use_sim_time', True),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'parts', 'bringup_moveit.launch.py')
                ),
                launch_arguments={
                    'pkg_name': pkg_name,
                    'fix_start_state': 'true',
                    'model_xacro': sim_urdf_path,
                    # フック指(trx_s_sim)の自己干渉を無効化したシム用SRDF
                    'srdf_file': 'SEED-Noid-Lifter-Mover-typeG_sim.srdf',
                }.items(),
            ),
        ],
        condition=IfCondition(LaunchConfiguration('moveit')),
    )

    # スポーン位置をAMCL初期姿勢として自動設定する
    # (AMCLがmap->odomを出すまでlocal_costmapがactivateできないため必須)
    qz, qw = math.sin(float(spawn_yaw) / 2), math.cos(float(spawn_yaw) / 2)
    initial_pose_msg = (
        '{header: {frame_id: map}, pose: {pose: {'
        f'position: {{x: {spawn_x}, y: {spawn_y}}}, '
        f'orientation: {{z: {qz}, w: {qw}}}}}, '
        'covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, '
        '0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}'
    )
    # SLAM(マッピング)時は slam_toolbox が map->odom を出すため AMCL 初期姿勢は不要
    publish_initial_pose = TimerAction(
        period=15.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--times', '10', '-r', '0.2',
                 '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
                 initial_pose_msg],
            output='log',
            condition=IfCondition(use_nav),
        )],
    ) if not use_slam else None

    # MotionPlanningパネルがロボットモデルを読むには、RVizノード自身に
    # robot_description / robot_description_semantic / robot_description_kinematics を
    # 直接渡す必要がある。これらが無いとmove_groupのパラメータサービス経由の取得に
    # フォールバックし、長時間稼働後などに"Unable to parse URDF"で失敗する
    import yaml
    with open(os.path.join(pkg_share, 'moveit', 'config', 'kinematics.yaml')) as f:
        kinematics = yaml.safe_load(f)
    # move_groupと同一のシム用SRDF(フック指の自己干渉を無効化したもの)
    with open(os.path.join(pkg_share, 'moveit', 'config',
                           'SEED-Noid-Lifter-Mover-typeG_sim.srdf')) as f:
        robot_description_semantic = f.read()

    # rviz.rviz はnav表示に加えMoveItのMotionPlanningパネルを含む統合設定
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'rviz.rviz')],
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
            'robot_description_semantic': robot_description_semantic,
            'robot_description_kinematics': kinematics,
        }],
        condition=IfCondition(use_rviz),
    )

    return [
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        *spawner_chain,
        odom_tf_relay,
        cmd_vel_stamper,
        navigation,
        moveit,
        *([publish_initial_pose] if publish_initial_pose is not None else []),
        rviz,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('noid_lifter_mover')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'world', 'seed_noid.sdf')),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution(
                [pkg_share, 'config', 'navigation', 'map', 'rsdlab.yaml'])),
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='Trueでslam_toolbox(マッピング)を起動。既存マップ+AMCLの代わりに地図を新規作成する'),
        DeclareLaunchArgument('moveit', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])
