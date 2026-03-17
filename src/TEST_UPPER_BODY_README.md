# 上半身 MoveIt 動作検証スクリプト

上半身（右腕、左腕、腰、頭）をMoveItを使って動作検証するPythonスクリプトです。

## 概要

このスクリプトは SEED-Noid-Lifter-Mover ロボットの上半身（`upper_body` グループ）各部位の動作をテストします。

対象部位：
- **右腕 (rarm)**: 肩・肘・手首の関節
- **左腕 (larm)**: 肩・肘・手首の関節  
- **腰 (waist)**: 腰関節
- **頭 (head)**: 首関節

## 必要な環境

- ROS 2 Jazzy
- MoveIt 2
- `noid_lifter_mover` パッケージ
- MoveIt 設定（SRDF, URDF, config files）

## ファイル位置

```
src/seed_robot_ros2_pkg/robots/noid_lifter_mover/src/test_upper_body_moveit.py
```

## 実行方法

### 1. 前準備

ROS2ワークスペースをセットアップします：

```bash
cd ~/ros2_ws
source install/setup.bash
```

### 2. スクリプト実行

#### 全テストを実行（推奨）

```bash
python3 src/seed_robot_ros2_pkg/robots/noid_lifter_mover/src/test_upper_body_moveit.py
```

または

```bash
python3 src/seed_robot_ros2_pkg/robots/noid_lifter_mover/src/test_upper_body_moveit.py all
```

このモードでは以下を順に実行します：
1. リセットポーズへの移動
2. 右腕の動作テスト（複数ポーズ）
3. 左腕の動作テスト（複数ポーズ）
4. 頭の動作テスト（回転・傾き）
5. 腰の動作テスト（回転）
6. 上半身全体のリセット

#### 複合動作テストを実行

```bash
python3 src/seed_robot_ros2_pkg/robots/noid_lifter_mover/src/test_upper_body_moveit.py combined
```

このモードでは両腕を同時に動かす複合動作をテストします。

### 3. 必要なサービス・ノード

スクリプト実行時に以下が起動していることを確認してください：

**別のターミナルで以下を実行：**

```bash
# MoveIt をセットアップして実行
ros2 launch noid_lifter_mover moveit.launch.py
```

または

```bash
# RVizで動作を可視化する場合
ros2 launch noid_lifter_mover moveit.launch.py rviz:=true
```

## スクリプトの構成

### 主要なメソッド

| メソッド | 説明 |
|---------|------|
| `plan_and_execute()` | 指定した関節角度への軌跡をプラン・実行 |
| `go_to_named_state()` | SRDFで事前定義された状態へ移動 |
| `test_all_upper_body()` | 全体的な動作テスト を実行 |
| `test_combined_motion()` | 複合動作（両腕同時）のテスト |

### 定義済み状態（Named States）

SRDF ファイルで以下の状態が定義されています：

- `reset-pose`: 上半身リセットポーズ
- `initial`: 初期ポーズ
- `reset-pose-rarm`: 右腕リセットポーズ
- `reset-pose-larm`: 左腕リセットポーズ
- `reset-pose-head`: 頭リセットポーズ

## ログ出力例

```
[INFO] ... 上半身のMoveIt動作検証を開始します...
=== リセットポーズに移動: reset-pose ===
[INFO] グループ: upper_body
[INFO] 軌跡をプランしています...
[INFO] ✓ 軌跡のプランに成功しました
[INFO] 軌跡を実行しています...
[INFO] ✓ 軌跡の実行が完了しました
...
```

## トラブルシューティング

### エラー: "Could not find URDF parameter"

**原因**: MoveIt が起動していない、または ROS パラメータが設定されていない

**解決法**:
```bash
# 別ターミナルで MoveIt を起動
ros2 launch noid_lifter_mover moveit.launch.py
```

### エラー: "Planning failed" または軌跡がプランできない

**原因**: 
- 目標ポーズが到達不可能な位置
- コリジョンの発生
- 関節の可動域制限
- コントローラーが応答しない

**解決法**:
1. RViz でロボットの現在状態を確認
2. 目標ポーズの妥当性を確認
3. `joint_limits.yaml` で関節制限を確認
4. コントローラーの状態を確認：
   ```bash
   ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
   ```

### エラー: "Controller not available"

**原因**: コントローラーが起動していない

**解決法**:
```bash
# コントローラーを起動
ros2 launch noid_lifter_mover controllers.launch.py
```

## パラメータ調整

### 動作速度の調整

`moveit_controllers.yaml` の以下パラメータを編集：

```yaml
trajectory_execution:
  allowed_execution_duration_scaling: 1.2  # 1.0より大きいと遅くなる
```

### 関節角度の追加・変更

スクリプト内の以下部分を編集：

```python
rarm_pose2 = {
    'r_shoulder_p_joint': 0.0,  # 各値を変更
    'r_shoulder_r_joint': 0.5,
    # ...
}
```

## RViz での可視化

MoveIt と RViz を起動することで、計画された軌跡が3D表示されます：

```bash
# RViz 付きで起動
ros2 launch noid_lifter_mover moveit.launch.py rviz:=true
```

RViz でのワークフロー：
1. "Planning" パネルで `upper_body` グループを選択
2. 各グループの目標ポーズをセット
3. "Plan" をクリック（軌跡が表示される）
4. "Execute" をクリック（ロボットが動作）

## カスタマイズ

### 新しいテストシーケンスの追加

```python
def test_custom_sequence(self):
    """カスタムテストシーケンス"""
    self.get_logger().info("カスタムテストを開始")
    
    # ポーズの定義
    custom_pose = {
        'joint_name': 1.57,  # ラジアン単位
        # ...
    }
    
    # 実行
    self.plan_and_execute('group_name', custom_pose, "説明")
```

## 参考資料

- [MoveIt 2 公式ドキュメント](https://moveit.picknik.ai/)
- [ROS2 コントローラーフレームワーク](https://control.ros.org/)
- [noid_lifter_mover パッケージ](../README.md)

## 注意事項

- ⚠️ 実際のロボット環境で実行する場合は、必ず安全エリアを確保してください
- ⚠️ スクリプト実行中は、ロボットの周辺に人がいないことを確認してください
- ⚠️ 緊急停止ボタンのアクセスを確認してから実行してください
