#!/usr/bin/env python3
"""
上半身のMoveIt動作検証スクリプト
SEED-Noid-Lifter-Mover の上半身（上腕、腰、頭）、リフター、下半身を動かします
ROS 2 アクションを直接使用（MoveIt依存なし）
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import math
import time


class UpperBodyTestNode(Node):
    def __init__(self):
        super().__init__('upper_body_test_node')
        
        # アクションクライアントの作成（各グループ用）
        self.action_clients = {
            'rarm': ActionClient(self, FollowJointTrajectory, 'rarm_controller/follow_joint_trajectory'),
            'larm': ActionClient(self, FollowJointTrajectory, 'larm_controller/follow_joint_trajectory'),
            'waist': ActionClient(self, FollowJointTrajectory, 'waist_controller/follow_joint_trajectory'),
            'head': ActionClient(self, FollowJointTrajectory, 'head_controller/follow_joint_trajectory'),
            'lifter': ActionClient(self, FollowJointTrajectory, 'lifter_controller/follow_joint_trajectory'),
        }
        
        # cmd_vel Publisher（下半身の移動制御用）
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # コントローラー情報
        self.controller_joints = {
            'rarm': ['r_shoulder_p_joint', 'r_shoulder_r_joint', 'r_shoulder_y_joint',
                     'r_elbow_joint', 'r_wrist_y_joint', 'r_wrist_p_joint', 'r_wrist_r_joint'],
            'larm': ['l_shoulder_p_joint', 'l_shoulder_r_joint', 'l_shoulder_y_joint',
                     'l_elbow_joint', 'l_wrist_y_joint', 'l_wrist_p_joint', 'l_wrist_r_joint'],
            'waist': ['waist_y_joint', 'waist_p_joint', 'waist_r_joint'],
            'head': ['neck_p_joint', 'neck_r_joint', 'neck_y_joint'],
            'lifter': ['knee_joint', 'ankle_joint'],
        }
        
        # 定義済み状態
        self.named_states = {
            'reset-pose': {
                'rarm': [-0.2443, -0.02, -0.3, -2.3562, 0.0, 0.0, 0.0],
                'larm': [-0.2443, 0.02, 0.3, -2.3562, 0.0, 0.0, 0.0],
                'waist': [0.0, 0.0, 0.0],
                'head': [0.0, 0.0, 0.0],
                'lifter': [0.0, 0.0],
            },
            'reset-pose-rarm': [-0.2443, -0.02, -0.3, -2.3562, 0.0, 0.0, 0.0],
            'reset-pose-larm': [-0.2443, 0.02, 0.3, -2.3562, 0.0, 0.0, 0.0],
            'reset-pose-head': [0.0, 0.0, 0.0],
            'reset-pose-lifter': [0.0, 0.0],
            'lifter-lowered': [-0.5, 0.5],
            'lifter-raised': [-1.2, 1.0],
        }
        
        self.get_logger().info("上半身のMoveIt動作検証を開始します...")

    def execute_trajectory(self, controller_name: str, target_positions: list, 
                          execution_time: float = 3.0, description: str = ""):
        """軌跡を実行する
        
        Args:
            controller_name: コントローラー名（'rarm', 'larm', 'waist', 'head', 'lifter'）
            target_positions: 目標関節角度のリスト（ラジアン）
            execution_time: 軌跡実行時間（秒）
            description: 動作の説明
        """
        self.get_logger().info(f"=== {description} ===")
        self.get_logger().info(f"コントローラー: {controller_name}")
        self.get_logger().info(f"目標角度: {target_positions}")
        
        try:
            # アクションクライアントの確認
            if controller_name not in self.action_clients:
                self.get_logger().error(f"✗ 不明なコントローラー: {controller_name}")
                return False
            
            client = self.action_clients[controller_name]
            joints = self.controller_joints[controller_name]
            
            # サーバーの接続確認
            if not client.wait_for_server(timeout_sec=5):
                self.get_logger().error(f"✗ コントローラー {controller_name} に接続できません")
                return False
            
            # 軌跡メッセージの作成
            trajectory = JointTrajectory()
            trajectory.joint_names = joints
            
            # 開始点（現在位置 - ここでは0を仮定）
            point0 = JointTrajectoryPoint()
            point0.positions = [0.0] * len(joints)
            point0.time_from_start = Duration(seconds=0).to_msg()
            trajectory.points.append(point0)
            
            # 終了点（目標位置）
            if len(target_positions) != len(joints):
                self.get_logger().error(f"✗ 関節数が一致しません: 期待値 {len(joints)}, 入力 {len(target_positions)}")
                return False
            
            point1 = JointTrajectoryPoint()
            point1.positions = target_positions
            point1.time_from_start = Duration(seconds=execution_time).to_msg()
            trajectory.points.append(point1)
            
            # アクションゴールの作成
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
            
            # アクション実行
            self.get_logger().info("軌跡を実行しています...")
            future = client.send_goal_async(goal)
            
            # 結果を待機
            rclpy.spin_until_future_complete(self, future, timeout_sec=execution_time + 5)
            
            if future.done():
                result = future.result()
                if result is not None:
                    self.get_logger().info("✓ 軌跡の実行が完了しました")
                    time.sleep(1)
                    return True
                else:
                    self.get_logger().error("✗ アクション実行に失敗しました")
                    return False
            else:
                self.get_logger().error("✗ アクション実行のタイムアウト")
                return False
                
        except Exception as e:
            self.get_logger().error(f"エラー: {str(e)}")
            return False

    def go_to_named_state(self, controller_name: str, state_name: str, 
                         execution_time: float = 3.0):
        """事前に定義された状態に移動する
        
        Args:
            controller_name: コントローラー名
            state_name: 定義された状態の名前
            execution_time: 実行時間
        """
        self.get_logger().info(f"=== 定義済み状態に移動: {state_name} ===")
        
        if state_name not in self.named_states:
            self.get_logger().error(f"✗ 不明な状態: {state_name}")
            return False
        
        state = self.named_states[state_name]
        
        # 複数グループの状態に対応
        if isinstance(state, dict):
            if controller_name not in state:
                self.get_logger().error(f"✗ 状態 {state_name} に {controller_name} が含まれていません")
                return False
            target_positions = state[controller_name]
        else:
            target_positions = state
        
        return self.execute_trajectory(controller_name, target_positions, 
                                      execution_time, f"状態: {state_name}")

    def test_all_upper_body(self):
        """上半身全体の動作検証"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("上半身全体の動作検証を開始します")
        self.get_logger().info("="*60)
        
        # 1. リセットポーズに移動（各グループ）
        self.get_logger().info("\n--- 全体をリセットポーズに移動 ---")
        for controller in ['rarm', 'larm', 'waist', 'head']:
            self.go_to_named_state(controller, 'reset-pose')
            time.sleep(1)
        
        time.sleep(2)
        
        # 2. 右腕の動作テスト
        self.get_logger().info("\n--- 右腕のテスト ---")
        self.go_to_named_state('rarm', 'reset-pose-rarm')
        
        time.sleep(1)
        
        # 右腕を動かす（肩を持ち上げる）
        rarm_pose2 = [0.0, 0.5, 0.0, -1.5, 0.0, 0.0, 0.0]
        self.execute_trajectory('rarm', rarm_pose2, 3.0, "右腕を上げる動作")
        
        time.sleep(1)
        
        # 右腕をリセットポーズに戻す
        self.go_to_named_state('rarm', 'reset-pose-rarm')
        
        time.sleep(2)
        
        # 3. 左腕の動作テスト
        self.get_logger().info("\n--- 左腕のテスト ---")
        self.go_to_named_state('larm', 'reset-pose-larm')
        
        time.sleep(1)
        
        # 左腕を動かす（肩を持ち上げる）
        larm_pose2 = [0.0, -0.5, 0.0, -1.5, 0.0, 0.0, 0.0]
        self.execute_trajectory('larm', larm_pose2, 3.0, "左腕を上げる動作")
        
        time.sleep(1)
        
        # 左腕をリセットポーズに戻す
        self.go_to_named_state('larm', 'reset-pose-larm')
        
        time.sleep(2)
        
        # 4. 頭の動作テスト
        self.get_logger().info("\n--- 頭のテスト ---")
        self.go_to_named_state('head', 'reset-pose-head')
        
        time.sleep(1)
        
        # 頭を左に回転
        head_left = [0.0, 0.0, 0.5]
        self.execute_trajectory('head', head_left, 2.0, "頭を左に回転")
        
        time.sleep(1)
        
        # 頭を右に回転
        head_right = [0.0, 0.0, -0.5]
        self.execute_trajectory('head', head_right, 2.0, "頭を右に回転")
        
        time.sleep(1)
        
        # 頭を前に傾ける
        head_forward = [0.3, 0.0, 0.0]
        self.execute_trajectory('head', head_forward, 2.0, "頭を前に傾ける")
        
        time.sleep(1)
        
        # 頭をリセット
        self.go_to_named_state('head', 'reset-pose-head')
        
        time.sleep(2)
        
        # 5. 腰の動作テスト
        self.get_logger().info("\n--- 腰のテスト ---")
        
        # 腰を回転
        waist_turn = [0.3, 0.0, 0.0]
        self.execute_trajectory('waist', waist_turn, 2.0, "腰を左に回転")
        
        time.sleep(1)
        
        # 腰を反対に回転
        waist_turn_right = [-0.3, 0.0, 0.0]
        self.execute_trajectory('waist', waist_turn_right, 2.0, "腰を右に回転")
        
        time.sleep(1)
        
        # 腰をリセット
        waist_reset = [0.0, 0.0, 0.0]
        self.execute_trajectory('waist', waist_reset, 2.0, "腰をリセット")
        
        time.sleep(2)
        
        # 6. 上半身全体のリセット
        self.get_logger().info("\n--- 上半身全体をリセット ---")
        for controller in ['rarm', 'larm', 'waist', 'head']:
            self.go_to_named_state(controller, 'reset-pose')
            time.sleep(0.5)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("上半身全体の動作検証が完了しました")
        self.get_logger().info("="*60)

    def test_lifter(self):
        """リフターの動作検証"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("リフターの動作検証を開始します")
        self.get_logger().info("="*60)
        
        # 1. リセットポーズに移動
        self.get_logger().info("\n--- リフターをリセットポーズに移動 ---")
        self.go_to_named_state('lifter', 'reset-pose-lifter')
        
        time.sleep(2)
        
        # 2. リフターを下げる（座位状態）
        self.get_logger().info("\n--- リフターを下げる（座位状態） ---")
        self.go_to_named_state('lifter', 'lifter-lowered')
        
        time.sleep(2)
        
        # 3. リフターを上げる（立位状態）
        self.get_logger().info("\n--- リフターを上げる（立位状態） ---")
        self.go_to_named_state('lifter', 'lifter-raised')
        
        time.sleep(2)
        
        # 4. 足首を動かすテスト
        self.get_logger().info("\n--- 足首を動かすテスト ---")
        lifter_ankle_move = [-0.8, 0.3]
        self.execute_trajectory('lifter', lifter_ankle_move, 2.0, "足首を動かす")
        
        time.sleep(1)
        
        # 5. リセット
        self.get_logger().info("\n--- リフターをリセット ---")
        self.go_to_named_state('lifter', 'reset-pose-lifter')
        
        time.sleep(1)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("リフターの動作検証が完了しました")
        self.get_logger().info("="*60)

    def publish_cmd_vel(self, linear_x: float = 0.0, linear_y: float = 0.0, 
                        angular_z: float = 0.0, duration: float = 1.0):
        """下半身に速度指令を送信
        
        Args:
            linear_x: X方向の速度（m/s）
            linear_y: Y方向の速度（m/s）
            angular_z: Z軸周りの角速度（rad/s）
            duration: 指令の持続時間（秒）
        """
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.angular.z = angular_z
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(cmd)
            time.sleep(0.1)
        
        # 停止指令を送信
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)

    def test_lower_body(self):
        """下半身（メカナムホイール）の動作検証"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("下半身（メカナムホイール）の動作検証を開始します")
        self.get_logger().info("="*60)
        
        # 1. 前進動作
        self.get_logger().info("\n--- 前進動作（2秒間） ---")
        self.publish_cmd_vel(linear_x=0.3, duration=2.0)
        time.sleep(1)
        
        # 2. 後退動作
        self.get_logger().info("\n--- 後退動作（2秒間） ---")
        self.publish_cmd_vel(linear_x=-0.3, duration=2.0)
        time.sleep(1)
        
        # 3. 左サイドステップ
        self.get_logger().info("\n--- 左サイドステップ（2秒間） ---")
        self.publish_cmd_vel(linear_y=0.3, duration=2.0)
        time.sleep(1)
        
        # 4. 右サイドステップ
        self.get_logger().info("\n--- 右サイドステップ（2秒間） ---")
        self.publish_cmd_vel(linear_y=-0.3, duration=2.0)
        time.sleep(1)
        
        # 5. 左回転
        self.get_logger().info("\n--- 左回転（2秒間） ---")
        self.publish_cmd_vel(angular_z=0.5, duration=2.0)
        time.sleep(1)
        
        # 6. 右回転
        self.get_logger().info("\n--- 右回転（2秒間） ---")
        self.publish_cmd_vel(angular_z=-0.5, duration=2.0)
        time.sleep(1)
        
        # 7. 複合動作（前進 + 右回転）
        self.get_logger().info("\n--- 複合動作：前進 + 右回転（2秒間） ---")
        self.publish_cmd_vel(linear_x=0.2, angular_z=-0.3, duration=2.0)
        time.sleep(1)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("下半身の動作検証が完了しました")
        self.get_logger().info("="*60)

    def test_combined_motion(self):
        """上半身全体の複合動作テスト"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("上半身の複合動作テストを開始します")
        self.get_logger().info("="*60)
        
        # リセットポーズからスタート
        for controller in ['rarm', 'larm', 'waist', 'head']:
            self.go_to_named_state(controller, 'reset-pose')
            time.sleep(0.5)
        
        time.sleep(2)
        
        # 両腕を上げて、腰も回転させる
        self.get_logger().info("\n--- 複合動作: 両腕を上げて腰を回転 ---")
        
        # 右腕を上げる
        rarm_pose = [0.5, 0.3, 0.0, -1.2, 0.0, 0.0, 0.0]
        self.execute_trajectory('rarm', rarm_pose, 3.0, "右腕を上げる")
        
        # 左腕を上げる
        larm_pose = [0.5, -0.3, 0.0, -1.2, 0.0, 0.0, 0.0]
        self.execute_trajectory('larm', larm_pose, 3.0, "左腕を上げる")
        
        # 腰を回転
        waist_pose = [0.3, 0.0, 0.0]
        self.execute_trajectory('waist', waist_pose, 2.0, "腰を回転")
        
        time.sleep(2)
        
        # リセット
        self.get_logger().info("\n--- リセット ---")
        for controller in ['rarm', 'larm', 'waist', 'head']:
            self.go_to_named_state(controller, 'reset-pose')
            time.sleep(0.5)
        
        self.get_logger().info("\n複合動作テストが完了しました")


def main(args=None):
    rclpy.init(args=args)
    
    node = UpperBodyTestNode()
    
    try:
        # テストの選択
        if len(sys.argv) > 1:
            test_mode = sys.argv[1]
        else:
            test_mode = 'all'
        
        if test_mode == 'lifter':
            node.test_lifter()
        elif test_mode == 'lower_body':
            node.test_lower_body()
        elif test_mode == 'combined':
            node.test_combined_motion()
        elif test_mode == 'all':
            node.test_all_upper_body()
            time.sleep(3)
            node.test_lifter()
            time.sleep(3)
            node.test_lower_body()
        else:
            node.get_logger().error(f"不明なテストモード: {test_mode}")
            node.get_logger().info("利用可能なテストモード: all, lifter, lower_body, combined")
        
        node.get_logger().info("\nテストが完了しました")
        
    except KeyboardInterrupt:
        node.get_logger().info("テストが中断されました")
    except Exception as e:
        node.get_logger().error(f"エラーが発生しました: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
