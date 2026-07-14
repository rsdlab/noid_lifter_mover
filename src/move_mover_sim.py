#!/usr/bin/env python3
# coding: UTF-8
"""
シミュレーション用台車移動スクリプト
WaypointManagementNode.move_to_goal() を使って台車を指定座標に移動します。

使い方:
  python3 move_mover_sim.py                        # サンプルシーケンス
  python3 move_mover_sim.py 1.0 0.5 1.57          # 指定座標へ移動 (x y theta)
"""

import os
import sys
import time
import threading
import math

sys.path.append(os.path.dirname(__file__))

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf_transformations

from waypoint_management.waypoint_management_node import WaypointManagementNode
from move_robot_sim import RobotMover


class MoverController:
    def __init__(self, node: Node):
        self._node = node
        self._wm = WaypointManagementNode(node)
        self._initialpose_pub = node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self._current_pose = None
        self._amcl_sub = node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_callback,
            10
        )

    def _amcl_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [ori.x, ori.y, ori.z, ori.w]
        )
        self._current_pose = (pos.x, pos.y, yaw)

    def get_current_pose(self, timeout: float = 5.0):
        """AMCLから現在の (x, y, theta[rad]) を返す。取得できなければ None。"""
        start = time.time()
        while self._current_pose is None and (time.time() - start) < timeout:
            time.sleep(0.1)
        if self._current_pose is None:
            self._node.get_logger().warn('AMCLの自己位置を取得できませんでした。')
        return self._current_pose

    def move_to_goal(self, message_id: str, x: float, y: float, theta: float) -> bool:
        """台車をマップ座標 (x, y, theta[rad]) に移動する。"""
        self._node.get_logger().info(
            f'=== 台車移動: x={x:.3f}, y={y:.3f}, theta={theta:.3f} ==='
        )
        return self._wm.move_to_goal(message_id, x, y, theta)

    def back_off(self, duration: float = 1.0):
        """後退する。（/cmd_vel_nav 直接制御）"""
        self._node.get_logger().info(f'=== 後退 ({duration:.1f}秒) ===')
        self._wm.back_off_if_needed(duration)

    def move_forward(self, duration: float = 1.0):
        """前進する。（/cmd_vel_nav 直接制御）"""
        self._node.get_logger().info(f'=== 前進 ({duration:.1f}秒) ===')
        self._wm.move_forward_if_needed(duration)

    def adjust_pose(self, duration: float = 1.0):
        """前進微調整する。（/cmd_vel_nav 直接制御）"""
        self._node.get_logger().info(f'=== 微調整 ({duration:.1f}秒) ===')
        self._wm.adjust_pose_if_needed(duration)

    def turn_in_place(self, angle_deg: float, speed: float = 0.3):
        """
        その場旋回する。（/cmd_vel_nav 直接制御）

        Args:
            angle_deg: 旋回角度 [度]
                       正の値 → 左旋回（反時計回り）
                       負の値 → 右旋回（時計回り）
            speed:     旋回速度 [rad/s]（デフォルト: 0.3）
        """
        self._wm.rotate_in_place(angle_deg, speed)

    def reset_amcl_pose(self, x: float, y: float, theta: float):
        """
        AMCLの自己位置を強制リセットする。
        旋回後など自己位置がずれた際に呼び出す。
        """
        self._node.get_logger().info(
            f'=== AMCLリセット: x={x:.3f}, y={y:.3f}, theta={theta:.3f} ==='
        )
        time.sleep(0.5)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        msg.pose.covariance[0]  = 0.1
        msg.pose.covariance[7]  = 0.1
        msg.pose.covariance[35] = 0.1

        self._initialpose_pub.publish(msg)
        time.sleep(2.0)
        self._node.get_logger().info('AMCLリセット完了。')


# ----------------------------------------------------------------------
# 座標定数（Rviz2の /clicked_point で取得して設定）
# ----------------------------------------------------------------------
POS_1_X = -2.00   # 位置1（台の前）のX座標 ← 実際の値に変更
POS_1_Y =  0.06   # 位置1（台の前）のY座標 ← 実際の値に変更
POS_2_X = -2.67   # 位置1（台の前）のX座標 ← 実際の値に変更
POS_2_Y =  0.06   # 位置1（台の前）のY座標 ← 実際の値に変更
POS_3_X =  -0.86   # 位置3（初期位置）のX座標 ← 実際の値に変更
POS_3_Y =  0.07   # 位置3（初期位置）のY座標 ← 実際の値に変更

# ----------------------------------------------------------------------
# 動作シーケンス
# ----------------------------------------------------------------------

def run_sample(mover: MoverController, robot: RobotMover):

    # =============================================
    # 配膳：位置1 → 位置2 → 位置3
    # =============================================

    # ① 位置1（台の前・左向き←）から後退 → 位置2へ
    mover._node.get_logger().info('=== 配膳：位置1 → 位置2（後退） ===')
    mover.back_off(duration=9.5)
    robot.CrouchPose()
    time.sleep(1)

    # ③ 位置2 → 位置3へ横移動（左向き← → 下向き↓ に変更）
    mover._node.get_logger().info('=== 配膳：位置2 → 位置3（横移動） ===')

    mover.turn_in_place(angle_deg=180)

    mover.move_to_goal(
        message_id="move_to_3",
        x=POS_2_X,
        y=POS_2_Y,
        theta=1.57
    )
    time.sleep(1)

    # ④ 前進微調整
    mover.adjust_pose(duration=16.0)
    time.sleep(1)

    mover._node.get_logger().info('=== 配膳完了 ===')

    # =============================================
    # 配膳後：位置3 → 位置2 → 位置1
    # =============================================

    # ⑤ 位置3（初期位置・下向き↓）から後退 → 位置2へ
    mover._node.get_logger().info('=== 配膳後：位置3 → 位置2（後退） ===')
    mover.back_off(duration=8.0)
    time.sleep(1)

    # ⑦ 位置2 → 位置1へ横移動（下向き↓ → 左向き← に変更）
    mover._node.get_logger().info('=== 配膳後：位置2 → 位置1（横移動） ===')
    mover.turn_in_place(-90)
    time.sleep(1)
    mover.move_to_goal(
        message_id="move_to_1",
        x=POS_3_X,
        y=POS_3_Y,
        theta=0.00     # （初期位置の向き）
    )
    time.sleep(1)

    # ⑧ 前進微調整（台に近づく）
    mover.adjust_pose(duration=17.0)
    time.sleep(1)

    mover._node.get_logger().info('=== 全動作完了 ===')


# ----------------------------------------------------------------------
# エントリーポイント
# ----------------------------------------------------------------------

USAGE = """\
使い方:
  python3 move_mover_sim.py [オプション] [x y theta]

説明:
  シミュレーション上の台車を指定座標に移動します。
  引数なしの場合はサンプルシーケンスを実行します。

引数:
  x       目標 X 座標 [m]
  y       目標 Y 座標 [m]
  theta   目標向き [rad]

オプション:
  -h, --help    このヘルプを表示して終了します

例:
  python3 move_mover_sim.py                  # サンプルシーケンスを実行
  python3 move_mover_sim.py 1.0 0.5 1.57    # (x=1.0, y=0.5, theta=1.57) へ移動
"""


def main():
    if '-h' in sys.argv or '--help' in sys.argv:
        print(USAGE, end='')
        sys.exit(0)

    rclpy.init()
    node = rclpy.create_node('mover_controller')
    # /clockが出ていればシミュレーションと判断してシム時刻を使う
    # (実機では/clockが無いので壁時計のまま → 同一ファイルで両対応)
    time.sleep(1.0)  # discovery待ち
    if node.count_publishers('/clock') > 0:
        node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        node.get_logger().info('シミュレーション検出: use_sim_time=true')

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    mover = MoverController(node)
    robot = RobotMover()

    try:
        if len(sys.argv) == 4:
            x, y, theta = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
            mover.move_to_goal(message_id='move_cmd', x=x, y=y, theta=theta)
        else:
            run_sample(mover, robot)

        node.get_logger().info('完了')
    except KeyboardInterrupt:
        node.get_logger().info('中断されました')
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        rclpy.shutdown()


if __name__ == '__main__':
    main()