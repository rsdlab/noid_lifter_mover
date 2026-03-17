#!/bin/bash
# 上半身 MoveIt 動作検証スクリプト - クイックスタート

echo "==========================================="
echo "SEED-Noid-Lifter-Mover 上半身MoveIt動作検証"
echo "==========================================="
echo ""

# ワークスペースの後処理を確認
if [ ! -d "$HOME/ros2_ws/src" ]; then
    echo "エラー: ~/ros2_ws ワークスペースが見つかりません"
    echo "セットアップについては README.md を参照してください"
    exit 1
fi

cd ~/ros2_ws

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
    echo "  source ~/ros2_ws/install/setup.bash"
    echo "  ros2 launch noid_lifter_mover moveit.launch.py"
    echo ""
    read -p "MoveItの起動を確認してから Enter キーを押してください..."
fi

echo ""
echo "テストモードを選択してください："
echo "  1) 全テスト実行（推奨）"
echo "  2) 複合動作テスト"
echo "  3) キャンセル"
echo ""
read -p "選択 [1-3]: " choice

case $choice in
    1)
        echo ""
        echo "全テストを実行します..."
        python3 src/seed_robot_ros2_pkg/robots/noid_lifter_mover/src/test_upper_body_moveit.py all
        ;;
    2)
        echo ""
        echo "複合動作テストを実行します..."
        python3 src/seed_robot_ros2_pkg/robots/noid_lifter_mover/src/test_upper_body_moveit.py combined
        ;;
    3)
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
