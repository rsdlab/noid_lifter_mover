#!/usr/bin/env python3
# coding: UTF-8
"""
シミュレーション用リフター操作スクリプト
リフターのしゃがみ・立ち姿勢を切り替えます。

使い方:
  python3 move_lifter_sim.py                 # しゃがむ (crouch)
  python3 move_lifter_sim.py crouch          # しゃがむ
  python3 move_lifter_sim.py stand           # 立つ
  python3 move_lifter_sim.py crouch 0.1      # しゃがむ (accel=0.1)
  python3 move_lifter_sim.py stand  0.3      # 立つ    (accel=0.3)
"""

import os
import sys
import time

sys.path.append(os.path.dirname(__file__))

import rclpy
from move_robot_sim import RobotMover


def main():
    rclpy.init()
    robot = RobotMover()

    try:
        mode  = sys.argv[1] if len(sys.argv) > 1 else 'crouch'
        accel = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2

        if mode == 'stand':
            robot.StandPose(accel=accel)
        else:
            robot.CrouchPose(accel=accel)

        robot.get_logger().info('完了')
    except KeyboardInterrupt:
        robot.get_logger().info('中断されました')
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
