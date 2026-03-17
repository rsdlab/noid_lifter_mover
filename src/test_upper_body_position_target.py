#!/usr/bin/env python3
"""
上半身の位置目標検証スクリプト
SEED-Noid-Lifter-Mover の上半身（右腕・左腕）をエンドエフェクター位置目標で動かします
MoveGroup ActionClient を使用して逆運動学を計算・実行し、位置精度を検証します
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer
import math
import time
import threading


class UpperBodyPositionTargetNode(Node):
    def __init__(self):
        super().__init__('upper_body_position_target_node')
        
        # 現在のジョイント状態を保持
        self.current_positions = [0.0] * 19
        self.current_positions_lock = False
        
        # MoveGroup ActionClientの初期化
        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info("Waiting for MoveGroup action server (/move_action)...")
        self.client.wait_for_server()
        self.get_logger().info("MoveGroup action server connected.")
        
        # 結果待ちイベント
        self.done_event = threading.Event()
        self.motion_result = None
        
        # Joint状態購読
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.update_position,
            10
        )
        
        # TFリスナー
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # エンドエフェクターリンク名
        self.ee_link_names = {
            'rarm': 'r_eef_grasp_link',
            'larm': 'l_eef_grasp_link',
        }
        
        # 定義済み位置目標（ベースフレームから見たエンドエフェクター位置）
        # 座標系: x(前), y(左), z(上)
        self.named_poses = {
            'rarm': {
                'reset': [0.3, -0.3, 0.8],  # x, y, z (メートル)
                'raised_front': [0.5, -0.3, 1.0],  # 前方に上げた位置
                'raised_side': [0.3, -0.5, 1.0],   # 横に上げた位置
                'lowered': [0.4, -0.3, 0.5],       # 下げた位置
            },
            'larm': {
                'reset': [0.3, 0.3, 0.8],   # x, y, z
                'raised_front': [0.5, 0.3, 1.0],   # 前方に上げた位置
                'raised_side': [0.3, 0.5, 1.0],    # 横に上げた位置
                'lowered': [0.4, 0.3, 0.5],        # 下げた位置
            },
        }
        
        # デフォルト姿勢（手のひらが下に向いた姿勢）
        # [x, y, z, w]
        self.default_orientation = [0.0, 0.7071, 0.0, 0.7071]
        
        self.get_logger().info("初期化が完了しました")

    def update_position(self, msg):
        """JointState購読コールバック"""
        if not self.current_positions_lock:
            self.current_positions_lock = True
            self.current_positions = list(msg.position)
            self.current_positions_lock = False

    def create_arm_pose_goal(self, arm_name: str, position: list, orientation: list) -> MoveGroup.Goal:
        """腕のポーズ目標を作成（位置と姿勢を指定）
        
        Args:
            arm_name: 'rarm' または 'larm'
            position: [x, y, z]
            orientation: [x, y, z, w]
            
        Returns:
            MoveGroup.Goal
        """
        group_name = 'rarm' if arm_name == 'rarm' else 'larm'
        ee_link = self.ee_link_names[arm_name]
        
        goal = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = group_name
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5
        
        constraints = Constraints()
        
        # PositionConstraint: 球状の許容範囲を指定（位置精度）
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "body_link"  # SRDFのグループ定義に合わせる
        pos_constraint.header.stamp = self.get_clock().now().to_msg()
        pos_constraint.link_name = ee_link
        
        # 許容範囲: 半径10cm（0.1m）の球
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.1]
        
        pose_target = PoseStamped()
        pose_target.header.frame_id = "body_link"  # SRDFのグループ定義に合わせる
        pose_target.header.stamp = self.get_clock().now().to_msg()
        pose_target.pose.position.x = float(position[0])
        pose_target.pose.position.y = float(position[1])
        pose_target.pose.position.z = float(position[2])
        
        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(sphere)
        bounding_volume.primitive_poses.append(pose_target.pose)
        
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0
        
        # OrientationConstraint: 姿勢制約
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "body_link"  # SRDFのグループ定義に合わせる
        ori_constraint.header.stamp = self.get_clock().now().to_msg()
        ori_constraint.link_name = ee_link
        ori_constraint.orientation.x = float(orientation[0])
        ori_constraint.orientation.y = float(orientation[1])
        ori_constraint.orientation.z = float(orientation[2])
        ori_constraint.orientation.w = float(orientation[3])
        ori_constraint.absolute_x_axis_tolerance = 0.05
        ori_constraint.absolute_y_axis_tolerance = 0.05
        ori_constraint.absolute_z_axis_tolerance = 0.05
        ori_constraint.weight = 1.0
        
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        
        request.goal_constraints.append(constraints)
        goal.request = request
        
        return goal
    
    def get_current_end_effector_position(self, arm_name: str) -> list:
        """TFからエンドエフェクター位置を取得
        
        Args:
            arm_name: 'rarm' または 'larm'
            
        Returns:
            [x, y, z] のリスト
        """
        try:
            ee_link = self.ee_link_names[arm_name]
            # body_linkを基準とする
            transform = self.tf_buffer.lookup_transform('body_link', ee_link, rclpy.time.Time())
            pos = transform.transform.translation
            return [pos.x, pos.y, pos.z]
        except Exception as e:
            self.get_logger().warn(f"TF取得エラー（body_link基準）: {str(e)}")
            # フォールバック: base_linkを試す
            try:
                ee_link = self.ee_link_names[arm_name]
                transform = self.tf_buffer.lookup_transform('base_link', ee_link, rclpy.time.Time())
                pos = transform.transform.translation
                return [pos.x, pos.y, pos.z]
            except Exception as e2:
                self.get_logger().error(f"TF取得失敗: {str(e2)}")
                return None
    
    def calculate_distance(self, pos1: list, pos2: list) -> float:
        """2つの位置のユークリッド距離を計算"""
        if pos1 is None or pos2 is None:
            return float('inf')
        return math.sqrt(
            (pos1[0] - pos2[0])**2 +
            (pos1[1] - pos2[1])**2 +
            (pos1[2] - pos2[2])**2
        )
    
    def pose_sent_callback(self, future):
        """ゴール送信後のコールバック"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Motion goal rejected")
            self.motion_result = False
            self.done_event.set()
            return
        
        self.get_logger().info("Motion goal accepted, waiting for result...")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.pose_result_callback)
    
    def pose_result_callback(self, future):
        """ゴール実行後のコールバック"""
        result = future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("✓ Motion executed successfully")
            self.motion_result = True
        else:
            self.get_logger().error(f"✗ Motion failed with error code: {result.error_code.val}")
            self.motion_result = False
        
        self.done_event.set()

    def move_to_position_target(self, arm_name: str, target_position: list,
                               target_orientation=None, description: str = "") -> bool:
        """指定した位置を目標として移動
        
        Args:
            arm_name: 腕名 ('rarm' または 'larm')
            target_position: 目標位置 [x, y, z] (メートル)
            target_orientation: 目標姿勢 [x, y, z, w] (Quaternion)
            description: 動作の説明
            
        Returns:
            成功したかどうか
        """
        self.get_logger().info(f"\n=== {description} ===")
        self.get_logger().info(f"腕: {arm_name}")
        self.get_logger().info(f"目標位置: x={target_position[0]:.3f}, y={target_position[1]:.3f}, z={target_position[2]:.3f}")
        
        try:
            if target_orientation is None:
                target_orientation = self.default_orientation
            
            # 現在位置を記録
            current_pos = self.get_current_end_effector_position(arm_name)
            if current_pos:
                self.get_logger().info(f"現在位置: x={current_pos[0]:.3f}, y={current_pos[1]:.3f}, z={current_pos[2]:.3f}")
            
            # ゴール作成
            goal = self.create_arm_pose_goal(arm_name, target_position, target_orientation)
            
            # ゴール送信
            self.get_logger().info("計画・実行を開始しています...")
            self.motion_result = None
            self.done_event.clear()
            
            goal_future = self.client.send_goal_async(goal)
            goal_future.add_done_callback(self.pose_sent_callback)
            
            # 結果を待機
            finished = self.done_event.wait(timeout=15.0)
            
            if not finished:
                self.get_logger().error("✗ タイムアウト")
                return False
            
            if not self.motion_result:
                self.get_logger().error("✗ 計画/実行に失敗しました")
                return False
            
            time.sleep(1.0)
            
            # 到達位置を取得
            actual_pos = self.get_current_end_effector_position(arm_name)
            if actual_pos:
                self.get_logger().info(f"到達位置: x={actual_pos[0]:.3f}, y={actual_pos[1]:.3f}, z={actual_pos[2]:.3f}")
                
                # 誤差を計算
                error = self.calculate_distance(target_position, actual_pos)
                self.get_logger().info(f"位置誤差: {error:.4f} m ({error*100:.2f} cm)")
                
                # 誤差のしきい値（5cm以内なら成功と判定）
                if error < 0.05:
                    self.get_logger().info("✓ 目標位置への到達に成功しました")
                    return True
                else:
                    self.get_logger().warn(f"⚠ 位置誤差が大きいです（目安: 5cm以下）")
                    return False
            else:
                self.get_logger().warn("⚠ 到達位置を取得できません")
                return self.motion_result
            
        except Exception as e:
            self.get_logger().error(f"エラー: {str(e)}")
            import traceback
            traceback.print_exc()
            return False

    def test_right_arm_positions(self):
        """右腕の位置目標テスト"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("右腕の位置目標テストを開始します")
        self.get_logger().info("="*60)
        
        success_count = 0
        total_count = 0
        
        # リセットポーズ
        total_count += 1
        if self.move_to_position_target('rarm', self.named_poses['rarm']['reset'],
                                       description="右腕をリセット位置に移動"):
            success_count += 1
        time.sleep(1.5)
        
        # 前方に上げた位置
        total_count += 1
        if self.move_to_position_target('rarm', self.named_poses['rarm']['raised_front'],
                                       description="右腕を前方に上げる"):
            success_count += 1
        time.sleep(1.5)
        
        # リセットに戻す
        total_count += 1
        if self.move_to_position_target('rarm', self.named_poses['rarm']['reset'],
                                       description="右腕をリセット位置に戻す"):
            success_count += 1
        time.sleep(1.5)
        
        # 横に上げた位置
        total_count += 1
        if self.move_to_position_target('rarm', self.named_poses['rarm']['raised_side'],
                                       description="右腕を横に上げる"):
            success_count += 1
        time.sleep(1.5)
        
        # リセットに戻す
        total_count += 1
        if self.move_to_position_target('rarm', self.named_poses['rarm']['reset'],
                                       description="右腕をリセット位置に戻す"):
            success_count += 1
        time.sleep(1.5)
        
        # 下げた位置
        total_count += 1
        if self.move_to_position_target('rarm', self.named_poses['rarm']['lowered'],
                                       description="右腕を下げる"):
            success_count += 1
        time.sleep(1.5)
        
        # リセットに戻す
        total_count += 1
        if self.move_to_position_target('rarm', self.named_poses['rarm']['reset'],
                                       description="右腕をリセット位置に戻す"):
            success_count += 1
        
        self.get_logger().info(f"\n右腕テスト結果: {success_count}/{total_count} 成功")

    def test_left_arm_positions(self):
        """左腕の位置目標テスト"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("左腕の位置目標テストを開始します")
        self.get_logger().info("="*60)
        
        success_count = 0
        total_count = 0
        
        # リセットポーズ
        total_count += 1
        if self.move_to_position_target('larm', self.named_poses['larm']['reset'],
                                       description="左腕をリセット位置に移動"):
            success_count += 1
        time.sleep(1.5)
        
        # 前方に上げた位置
        total_count += 1
        if self.move_to_position_target('larm', self.named_poses['larm']['raised_front'],
                                       description="左腕を前方に上げる"):
            success_count += 1
        time.sleep(1.5)
        
        # リセットに戻す
        total_count += 1
        if self.move_to_position_target('larm', self.named_poses['larm']['reset'],
                                       description="左腕をリセット位置に戻す"):
            success_count += 1
        time.sleep(1.5)
        
        # 横に上げた位置
        total_count += 1
        if self.move_to_position_target('larm', self.named_poses['larm']['raised_side'],
                                       description="左腕を横に上げる"):
            success_count += 1
        time.sleep(1.5)
        
        # リセットに戻す
        total_count += 1
        if self.move_to_position_target('larm', self.named_poses['larm']['reset'],
                                       description="左腕をリセット位置に戻す"):
            success_count += 1
        time.sleep(1.5)
        
        # 下げた位置
        total_count += 1
        if self.move_to_position_target('larm', self.named_poses['larm']['lowered'],
                                       description="左腕を下げる"):
            success_count += 1
        time.sleep(1.5)
        
        # リセットに戻す
        total_count += 1
        if self.move_to_position_target('larm', self.named_poses['larm']['reset'],
                                       description="左腕をリセット位置に戻す"):
            success_count += 1
        
        self.get_logger().info(f"\n左腕テスト結果: {success_count}/{total_count} 成功")

    def test_both_arms_combined(self):
        """両腕の複合動作テスト"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("両腕の複合動作テストを開始します")
        self.get_logger().info("="*60)
        
        # リセットポーズに移動
        self.get_logger().info("\n--- 初期位置に移動 ---")
        self.move_to_position_target('rarm', self.named_poses['rarm']['reset'],
                                    description="右腕をリセット位置に")
        self.move_to_position_target('larm', self.named_poses['larm']['reset'],
                                    description="左腕をリセット位置に")
        time.sleep(2)
        
        # 両腕を同時に上げる
        self.get_logger().info("\n--- 両腕を前方に上げる ---")
        self.move_to_position_target('rarm', self.named_poses['rarm']['raised_front'],
                                    description="右腕を前方に上げる")
        self.move_to_position_target('larm', self.named_poses['larm']['raised_front'],
                                    description="左腕を前方に上げる")
        time.sleep(2)
        
        # 両腕をいったんリセット
        self.get_logger().info("\n--- 両腕をリセット位置に戻す ---")
        self.move_to_position_target('rarm', self.named_poses['rarm']['reset'],
                                    description="右腕をリセット位置に")
        self.move_to_position_target('larm', self.named_poses['larm']['reset'],
                                    description="左腕をリセット位置に")
        time.sleep(2)
        
        # 両腕を横に上げる
        self.get_logger().info("\n--- 両腕を横に上げる ---")
        self.move_to_position_target('rarm', self.named_poses['rarm']['raised_side'],
                                    description="右腕を横に上げる")
        self.move_to_position_target('larm', self.named_poses['larm']['raised_side'],
                                    description="左腕を横に上げる")
        time.sleep(2)
        
        # 両腕をリセット
        self.get_logger().info("\n--- 最終的にリセット位置に戻す ---")
        self.move_to_position_target('rarm', self.named_poses['rarm']['reset'],
                                    description="右腕をリセット位置に")
        self.move_to_position_target('larm', self.named_poses['larm']['reset'],
                                    description="左腕をリセット位置に")
        
        self.get_logger().info("\n複合動作テストが完了しました")

    def test_custom_positions(self, rarm_pos=None, larm_pos=None):
        """カスタム位置でテスト
        
        Args:
            rarm_pos: 右腕の目標位置 [x, y, z]
            larm_pos: 左腕の目標位置 [x, y, z]
        """
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("カスタム位置テストを実行します")
        self.get_logger().info("="*60)
        
        if rarm_pos is not None:
            self.move_to_position_target('rarm', rarm_pos, description="右腕をカスタム位置に移動")
            time.sleep(1.5)
        
        if larm_pos is not None:
            self.move_to_position_target('larm', larm_pos, description="左腕をカスタム位置に移動")
            time.sleep(1.5)


def main(args=None):
    rclpy.init(args=args)
    
    node = UpperBodyPositionTargetNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # テストモードの選択
        if len(sys.argv) > 1:
            test_mode = sys.argv[1]
        else:
            test_mode = 'all'
        
        # スピナーをスレッドで実行
        import threading
        spinner_thread = threading.Thread(target=executor.spin, daemon=True)
        spinner_thread.start()
        
        time.sleep(1)  # MoveGroupアクションサーバーの接続を待つ
        
        if test_mode == 'rarm':
            node.test_right_arm_positions()
        elif test_mode == 'larm':
            node.test_left_arm_positions()
        elif test_mode == 'combined':
            node.test_both_arms_combined()
        elif test_mode.startswith('custom:'):
            # カスタム位置の指定: custom:0.3,-0.3,0.8:0.4,0.3,0.9
            parts = test_mode.split(':')
            if len(parts) >= 2:
                rarm_pos = [float(x) for x in parts[1].split(',')]
                larm_pos = [float(x) for x in parts[2].split(',')] if len(parts) > 2 else None
                node.test_custom_positions(rarm_pos, larm_pos)
            else:
                node.get_logger().error("カスタムモードの指定が不正です: custom:x,y,z または custom:x,y,z:x,y,z")
        else:
            # デフォルト: 全テスト実行
            node.test_right_arm_positions()
            time.sleep(2)
            node.test_left_arm_positions()
            time.sleep(2)
            node.test_both_arms_combined()
        
        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("全テストが完了しました")
        node.get_logger().info("="*60)
        
    except KeyboardInterrupt:
        node.get_logger().info("テストが中断されました")
    except Exception as e:
        node.get_logger().error(f"エラーが発生しました: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
