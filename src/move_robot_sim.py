#!/usr/bin/env python3
# coding: UTF-8
"""
シミュレーション用ロボット動作スクリプト
RViz2 + MoveIt2 上で上半身・リフターを動かします。

使い方:
  python3 move_robot_sim.py              # 初期姿勢のみ
  python3 move_robot_sim.py sample       # サンプル動作シーケンス
"""

import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

MOVE_ACTION = 'move_action'
GROUP_UPPER_BODY = 'upper_body'
GROUP_LIFTER = 'lifter'
PLANNING_TIMEOUT = 10.0
# 実行完了待ちの上限[壁時計秒]。シミュレーションはRTF(実時間係数)が低いと
# 実行に壁時計で数倍かかるため余裕を持たせる
EXECUTION_TIMEOUT = 120.0

UPPER_BODY_JOINTS = [
    'waist_y_joint',    'waist_p_joint',    'waist_r_joint',
    'l_shoulder_p_joint', 'l_shoulder_r_joint', 'l_shoulder_y_joint',
    'l_elbow_joint',    'l_wrist_y_joint',  'l_wrist_p_joint',  'l_wrist_r_joint',
    'neck_y_joint',     'neck_p_joint',     'neck_r_joint',
    'r_shoulder_p_joint', 'r_shoulder_r_joint', 'r_shoulder_y_joint',
    'r_elbow_joint',    'r_wrist_y_joint',  'r_wrist_p_joint',  'r_wrist_r_joint',
]
LIFTER_JOINTS = ['ankle_joint', 'knee_joint']


class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        # /clockが出ていればシミュレーションと判断してシム時刻を使う
        # (実機では/clockが無いので壁時計のまま → 同一ファイルで両対応)
        import time as _t
        _t.sleep(1.0)  # discovery待ち
        if self.count_publishers('/clock') > 0:
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
            self.get_logger().info('シミュレーション検出: use_sim_time=true')

        self._client = ActionClient(self, MoveGroup, MOVE_ACTION)
        self.get_logger().info('MoveGroup アクションサーバーへの接続を待機中...')
        self._client.wait_for_server()
        self.get_logger().info('接続完了')

        self._cmd_vel_pub = self.create_publisher(Twist, '/mechanum_controller/cmd_vel_teleop', 10)

        # バックグラウンド spin（コールバック処理のため）
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    # ------------------------------------------------------------------
    # 公開インターフェース
    # ------------------------------------------------------------------

    def move_upper_body(self,
                        waist_y=0.0, waist_p=0.0, waist_r=0.0,
                        l_sh_p=0.0,  l_sh_r=0.0,  l_sh_y=0.0,
                        l_elbow=0.0, l_wr_y=0.0,  l_wr_p=0.0, l_wr_r=0.0,
                        neck_y=0.0,  neck_p=0.0,  neck_r=0.0,
                        r_sh_p=0.0,  r_sh_r=0.0,  r_sh_y=0.0,
                        r_elbow=0.0, r_wr_y=0.0,  r_wr_p=0.0, r_wr_r=0.0,
                        vel=0.5) -> bool:
        """上半身を指定角度（度）に動かす。"""
        values_deg = [
            waist_y, waist_p, waist_r,
            l_sh_p,  l_sh_r,  l_sh_y,  l_elbow, l_wr_y, l_wr_p, l_wr_r,
            neck_y,  neck_p,  neck_r,
            r_sh_p,  r_sh_r,  r_sh_y,  r_elbow, r_wr_y, r_wr_p, r_wr_r,
        ]
        goals = {name: math.radians(v) for name, v in zip(UPPER_BODY_JOINTS, values_deg)}
        return self._send_goal(goals, GROUP_UPPER_BODY, vel)

    def move_lifter(self, ankle_deg=0.0, knee_deg=0.0, vel=0.3, accel=0.2) -> bool:
        """リフターを指定角度（度）に動かす。"""
        goals = {
            'ankle_joint': math.radians(ankle_deg),
            'knee_joint':  math.radians(knee_deg),
        }
        return self._send_goal(goals, GROUP_LIFTER, vel, accel)

    def initial_pose(self) -> bool:
        """初期姿勢: 両肘 -170°、他はゼロ。"""
        self.get_logger().info('=== 初期姿勢に移動 ===')
        ok = self.move_upper_body(l_elbow=-170.0, r_elbow=-170.0, vel=0.5)
        if ok:
            ok = self.move_lifter(ankle_deg=0.0, knee_deg=0.0, vel=0.3)
        return ok

    def ReadyPose(self) -> bool:
        """準備姿勢: 両肘 -70°、両肩を少し前に。"""
        self.get_logger().info('=== 準備姿勢に移動 ===')
        return self.move_upper_body(
            l_sh_p=-52.0, l_sh_r=12.0,  l_sh_y=-4.0,  l_elbow=-70.0, l_wr_y=0.0, l_wr_p=0.0, l_wr_r=0.0,
            r_sh_p=-52.0, r_sh_r=-12.0, r_sh_y=4.0,   r_elbow=-70.0, r_wr_y=0.0, r_wr_p=0.0, r_wr_r=0.0,
            vel=0.5,
        )

    def ApproachPose(self) -> bool:
        """アプローチ準備姿勢: 両肘 -60°、両肩をさらに前に。"""
        self.get_logger().info('=== アプローチ準備姿勢に移動 ===')
        return self.move_upper_body(
            l_sh_p=-43.0, l_sh_r=24.0,  l_sh_y=-17.0, l_elbow=-60.0, l_wr_y=30.0,  l_wr_p=2.0,  l_wr_r=-5.0,
            r_sh_p=-46.0, r_sh_r=-12.0, r_sh_y=2.0,  r_elbow=-51.0, r_wr_y=-12.0, r_wr_p=-3.0, r_wr_r=16.0,
            vel=0.5,
        )

    def PickPose(self) -> bool:
        """把持姿勢: 両肘 -60°、お盆に手を掛ける。"""
        self.get_logger().info('=== 把持姿勢に移動 ===')
        return self.move_upper_body(
            l_sh_p=-46.0, l_sh_r=17.0,  l_sh_y=-28.0, l_elbow=-61.0, l_wr_y=26.0,  l_wr_p=4.0,  l_wr_r=-0.0,
            r_sh_p=-44.0, r_sh_r=-6.0, r_sh_y=5.0,  r_elbow=-57.0, r_wr_y=-5.0, r_wr_p=-2.0, r_wr_r=16.0,
            vel=0.5,
        )

    def UpPose(self) -> bool:
        """上げ姿勢: 両肘 -62°、お盆を持ち上げる。"""
        self.get_logger().info('=== 上げ姿勢に移動 ===')
        return self.move_upper_body(
            l_sh_p=-46.0, l_sh_r=17.0,  l_sh_y=-28.0, l_elbow=-66.0, l_wr_y=26.0,  l_wr_p=4.0,  l_wr_r=-0.0,
            r_sh_p=-44.0, r_sh_r=-6.0, r_sh_y=5.0,  r_elbow=-60.0, r_wr_y=-5.0, r_wr_p=-2.0, r_wr_r=16.0,
            vel=0.5,
        )

    def move_base(self, linear_x: float, duration: float) -> None:
        """台車を linear_x [m/s] で duration 秒間移動させる。"""
        direction = '後退' if linear_x < 0 else '前進'
        self.get_logger().info(f'=== 台車{direction} {duration}秒 ===')
        twist = Twist()
        twist.linear.x = linear_x
        # ROSクロック基準(シム時はRTFに依らず距離が合う)
        clock_sec = lambda: self.get_clock().now().nanoseconds * 1e-9
        end_time = clock_sec() + duration
        while clock_sec() < end_time:
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self._cmd_vel_pub.publish(Twist())  # 停止

    def CrouchPose(self, accel: float = 0.2) -> bool:
        """しゃがみ姿勢: リフターを下げる (ankle=55°, knee=-55°)。"""
        self.get_logger().info('=== しゃがみ姿勢に移動 ===')
        return self.move_lifter(ankle_deg=45.0, knee_deg=-45.0, vel=1.0, accel=accel)

    def StandPose(self, accel: float = 0.2) -> bool:
        """立ち姿勢: リフターを初期位置に戻す (ankle=0°, knee=0°)。"""
        self.get_logger().info('=== 立ち姿勢に移動 ===')
        return self.move_lifter(ankle_deg=0.0, knee_deg=0.0, vel=1.0, accel=accel)

    # ------------------------------------------------------------------
    # 内部実装
    # ------------------------------------------------------------------

    def _send_goal(self, joint_goals: dict, group: str, vel: float, accel: float = None) -> bool:
        constraints = Constraints()
        for name, position in joint_goals.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = group
        req.goal_constraints.append(constraints)
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = accel if accel is not None else vel
        req.allowed_planning_time = PLANNING_TIMEOUT

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options.plan_only = False

        self.get_logger().info(f'[{group}] 計画・実行中...')

        done = threading.Event()
        result_holder = [None]

        def on_result(future):
            result = future.result()
            result_holder[0] = result.result.error_code.val == 1
            done.set()

        def on_goal_response(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'[{group}] ゴールが拒否されました')
                result_holder[0] = False
                done.set()
                return
            goal_handle.get_result_async().add_done_callback(on_result)

        self._client.send_goal_async(goal_msg).add_done_callback(on_goal_response)
        done.wait(timeout=EXECUTION_TIMEOUT)

        if result_holder[0] is None:
            self.get_logger().error(f'[{group}] タイムアウト')
            return False
        if result_holder[0]:
            self.get_logger().info(f'[{group}] 成功')
        else:
            self.get_logger().error(f'[{group}] 失敗')
        return result_holder[0]


# ----------------------------------------------------------------------
# 動作シーケンス例
# ----------------------------------------------------------------------

def run_sample(mover: RobotMover):
    mover.initial_pose()
    time.sleep(0.5)

    mover.ReadyPose()
    time.sleep(0.5)
    mover.ApproachPose()
    time.sleep(0.5)
    mover.PickPose()
    time.sleep(0.5)
    mover.UpPose()

    time.sleep(0.5)
    mover.move_base(-0.05, 10.0)  # 2秒後退
    mover.CrouchPose()
    time.sleep(2.0)
    mover.StandPose()
    mover.move_base( 0.05, 10.0)  # 2秒前進
    time.sleep(0.5)

    time.sleep(0.5)
    mover.PickPose()
    time.sleep(0.5)
    mover.ApproachPose()
    time.sleep(0.5)
    mover.ReadyPose()
    time.sleep(0.5)
    mover.initial_pose()


# ----------------------------------------------------------------------
# エントリーポイント
# ----------------------------------------------------------------------

def main():
    rclpy.init()
    mover = RobotMover()

    try:
        mode = sys.argv[1] if len(sys.argv) > 1 else 'init'

        if mode == 'sample':
            run_sample(mover)
        else:
            mover.initial_pose()

        mover.get_logger().info('完了')
    except KeyboardInterrupt:
        mover.get_logger().info('中断されました')
    finally:
        mover.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
