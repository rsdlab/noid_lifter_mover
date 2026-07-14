#!/usr/bin/env python3
# Nav2のTwistをベース駆動の2経路へ配る:
#  1) TwistStamped -> mecanum_drive_controller (ホイール回転 = オドメトリ源)
#  2) Twist -> gz VelocityControlプラグイン (実際のベース移動)
# VelocityControlには指令タイムアウトが無く、入力が途絶えると動き続けて
# オドメトリとずれるため、timeout超過でゼロ指令を送るウォッチドッグを持つ。
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelStamper(Node):
    def __init__(self):
        super().__init__('cmd_vel_stamper')
        self.declare_parameter('input_topic', '/mechanum_controller/cmd_vel_nav')
        # waypoint_management等が直接駆動に使うteleopトピックも受ける
        self.declare_parameter('teleop_topic', '/mechanum_controller/cmd_vel_teleop')
        self.declare_parameter('reference_topic', '/mechanum_controller/reference')
        self.declare_parameter('gz_cmd_vel_topic', '/model/noid_lifter_mover/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('timeout', 0.5)

        self.frame_id = self.get_parameter('frame_id').value
        self.timeout = self.get_parameter('timeout').value
        self.ref_pub = self.create_publisher(
            TwistStamped, self.get_parameter('reference_topic').value, 10)
        self.gz_pub = self.create_publisher(
            Twist, self.get_parameter('gz_cmd_vel_topic').value, 10)
        self.sub = self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self.callback, 10)
        # waypoint_management はBEST_EFFORTで発行するため購読側も合わせる
        # (BEST_EFFORT購読はRELIABLE発行とも互換なので手動pubも届く)
        teleop_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.teleop_sub = self.create_subscription(
            Twist, self.get_parameter('teleop_topic').value, self.callback, teleop_qos)

        self.last_msg_time = None
        self.stopped = True
        self.create_timer(0.1, self.watchdog)

    def callback(self, msg: Twist):
        self.last_msg_time = self.get_clock().now()
        self.stopped = False
        out = TwistStamped()
        out.header.stamp = self.last_msg_time.to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.ref_pub.publish(out)
        self.gz_pub.publish(msg)

    def watchdog(self):
        if self.stopped or self.last_msg_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds * 1e-9
        if elapsed > self.timeout:
            self.gz_pub.publish(Twist())
            self.stopped = True


def main():
    rclpy.init()
    node = CmdVelStamper()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
