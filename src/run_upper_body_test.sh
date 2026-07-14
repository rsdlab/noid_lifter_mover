#!/bin/bash
# 上半身 MoveIt 動作検証スクリプト - クイックスタート

echo "==========================================="
echo "SEED-Noid-Lifter-Mover 上半身MoveIt動作検証"
echo "==========================================="
echo ""

# ワークスペースの後処理を確認
if [ ! -d "$HOME/seed_ROS2_ws/src" ]; then
    echo "エラー: ~/seed_ROS2_ws ワークスペースが見つかりません"
    echo "セットアップについては README.md を参照してください"
    exit 1
fi

cd ~/seed_ROS2_ws

# ROS 2 環境のセットアップ
echo "ROS 2 環境をセットアップしています..."
source install/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash

# MoveItが起動しているか確認
echo ""
echo "MoveItの状態を確認中..."
if ! ros2 node list 2>/dev/null | grep -q "moveit"; then
    echo " ⚠️  警告: MoveItノードが起動していません"
    echo ""
    echo "別のターミナルで以下を実行してください："
    echo "  source ~/seed_ROS2_ws/install/setup.bash"
    echo "  ros2 launch noid_lifter_mover moveit.launch.py"
    echo ""
    read -p "MoveItの起動を確認してから Enter キーを押してください..."
fi

echo ""
echo "テストモードを選択してください："
echo "  1) 全テスト実行（上半身 + リフター + 下半身）"
echo "  2) 上半身 複合動作テスト"
echo "  3) Mover 全テスト（前後・左右・斜め・旋回）"
echo "  4) Mover 基本動作テスト（前後・左右・旋回）"
echo "  5) Mover 斜め移動テスト"
echo "  6) Mover 複合動作テスト（並進 + 旋回）"
echo "  7) キャンセル"
echo ""
read -p "選択 [1-7]: " choice

case $choice in
    1)
        echo ""
        echo "全テストを実行します..."
        python3 src/seed_robot_ros2_pkg/robots/noid_lifter_mover/src/test_upper_body_moveit.py all
        ;;
    2)
        echo ""
        echo "上半身 複合動作テストを実行します..."
        python3 src/seed_robot_ros2_pkg/robots/noid_lifter_mover/src/test_upper_body_moveit.py combined
        ;;
    3)
        echo ""
        echo "Mover 全テストを実行します..."
        python3 src/seed_robot_ros2_pkg/robots/noid_lifter_mover/src/test_mover_movement.py all
        ;;
    4)
        echo ""
        echo "Mover 基本動作テストを実行します..."
        python3 src/seed_robot_ros2_pkg/robots/noid_lifter_mover/src/test_mover_movement.py basic
        ;;
    5)
        echo ""
        echo "Mover 斜め移動テストを実行します..."
        python3 src/seed_robot_ros2_pkg/robots/noid_lifter_mover/src/test_mover_movement.py diagonal
        ;;
    6)
        echo ""
        echo "Mover 複合動作テストを実行します..."
        python3 src/seed_robot_ros2_pkg/robots/noid_lifter_mover/src/test_mover_movement.py combined
        ;;
    7)
        echo "キャンセルしました"
        exit 0
        ;;
    *)
        echo "無効な選択です"
        exit 1
        ;;
esac

echo ""
echo "==========================================="
echo "テストが完了しました"
echo "==========================================="
