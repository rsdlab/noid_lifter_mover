#!/usr/bin/env python3
"""
Mover（メカナムホイール台車）動作検証スクリプト
SEED-Noid-Lifter-Mover の下半身移動動作を検証します
"""

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoverTestNode(Node):
    def __init__(self):
        super().__init__('mover_test_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/mechanum_controller/cmd_vel_teleop', 10)
        # サブスクライバーが接続されるまで待つ
        self.get_logger().info("サブスクライバーの接続を待っています...")
        deadline = time.time() + 10.0
        while self.cmd_vel_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() > deadline:
                self.get_logger().warn("サブスクライバーが見つかりません。トピック名を確認してください。")
                break
        self.get_logger().info(
            f"サブスクライバー数: {self.cmd_vel_pub.get_subscription_count()}"
        )
        self.get_logger().info("Mover 動作検証を開始します...")

    def send_cmd_vel(self, linear_x: float = 0.0, linear_y: float = 0.0,
                     angular_z: float = 0.0, duration: float = 2.0,
                     description: str = ""):
        """指定した速度指令を duration 秒間送信し、その後停止する"""
        self.get_logger().info(f"=== {description} ===")
        self.get_logger().info(
            f"  linear_x={linear_x:.2f}  linear_y={linear_y:.2f}  angular_z={angular_z:.2f}  duration={duration}s"
        )

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.angular.z = angular_z

        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        # 停止
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("  -> 停止")

    def test_basic(self):
        """基本動作テスト（前後・左右・旋回）"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("基本動作テスト")
        self.get_logger().info("=" * 60)

        self.send_cmd_vel(linear_x=0.3,  duration=2.0, description="前進")
        time.sleep(1)
        self.send_cmd_vel(linear_x=-0.3, duration=2.0, description="後退")
        time.sleep(1)
        self.send_cmd_vel(linear_y=0.3,  duration=2.0, description="左サイドステップ")
        time.sleep(1)
        self.send_cmd_vel(linear_y=-0.3, duration=2.0, description="右サイドステップ")
        time.sleep(1)
        self.send_cmd_vel(angular_z=0.5,  duration=2.0, description="左旋回")
        time.sleep(1)
        self.send_cmd_vel(angular_z=-0.5, duration=2.0, description="右旋回")
        time.sleep(1)

        self.get_logger().info("\n基本動作テストが完了しました")

    def test_diagonal(self):
        """斜め移動テスト（メカナムホイール特有）"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("斜め移動テスト")
        self.get_logger().info("=" * 60)

        self.send_cmd_vel(linear_x=0.3, linear_y=0.3,  duration=2.0, description="右前斜め移動")
        time.sleep(1)
        self.send_cmd_vel(linear_x=0.3, linear_y=-0.3, duration=2.0, description="左前斜め移動")
        time.sleep(1)
        self.send_cmd_vel(linear_x=-0.3, linear_y=0.3,  duration=2.0, description="右後斜め移動")
        time.sleep(1)
        self.send_cmd_vel(linear_x=-0.3, linear_y=-0.3, duration=2.0, description="左後斜め移動")
        time.sleep(1)

        self.get_logger().info("\n斜め移動テストが完了しました")

    def test_combined(self):
        """複合動作テスト（並進 + 旋回）"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("複合動作テスト（並進 + 旋回）")
        self.get_logger().info("=" * 60)

        self.send_cmd_vel(linear_x=0.2, angular_z=0.3,  duration=2.0, description="前進しながら左旋回")
        time.sleep(1)
        self.send_cmd_vel(linear_x=0.2, angular_z=-0.3, duration=2.0, description="前進しながら右旋回")
        time.sleep(1)
        self.send_cmd_vel(linear_y=0.2, angular_z=0.3,  duration=2.0, description="左移動しながら左旋回")
        time.sleep(1)

        self.get_logger().info("\n複合動作テストが完了しました")

    def test_all(self):
        """全テスト実行"""
        self.test_basic()
        time.sleep(2)
        self.test_diagonal()
        time.sleep(2)
        self.test_combined()


def main(args=None):
    rclpy.init(args=args)
    node = MoverTestNode()

    test_mode = sys.argv[1] if len(sys.argv) > 1 else 'all'

    try:
        if test_mode == 'basic':
            node.test_basic()
        elif test_mode == 'diagonal':
            node.test_diagonal()
        elif test_mode == 'combined':
            node.test_combined()
        elif test_mode == 'all':
            node.test_all()
        else:
            node.get_logger().error(f"不明なテストモード: {test_mode}")
            node.get_logger().info("利用可能なモード: all, basic, diagonal, combined")
    except KeyboardInterrupt:
        node.get_logger().info("テストが中断されました")
    finally:
        node.cmd_vel_pub.publish(Twist())  # 必ず停止
        rclpy.shutdown()


if __name__ == '__main__':
    main()
