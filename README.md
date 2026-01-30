# ROS2 noid_lifter_moverセットアップ

## 1.noid_lifter_moverインストール
1. [seed_robot_ros2_pkg](https://github.com/thkrrc1/seed_robot_ros2_pkg)のREADMEに従ってプロジェクトをクローンする。

2. 上記README内、**3.seed_robot_ros2_pkgインストール**の**5.ロボットプロジェクトのクローン**にて、*noid_lifter_mover*を入力してクローンする。
    ```
    $ python3 clone_robots.py
    クローンしたいロボット名を入力してください：　noid_lifter_mover
    ```
    
以降は用途に応じて下記の作業に従ってください。  
※起動に失敗する場合は一度Ctrl+Cによる終了後、プロセスが残っていないかを確認してください。

## 2.slamによる地図作成
1. slamの実行  
    ```
    $ ros2 launch noid_lifter_mover bringup_robot.launch.py simulation:=false slam:=true
    ```

    一定時間経過するとLiDARの情報と地図が表示されるので、そこからロボットを動作させて地図作成を開始してください。
   

    ※下記コマンドでのslamは**実行不可**となります。ご注意ください。
    ```
    $ ros2 launch noid_lifter_mover bringup_robot.launch.py simulation:=true slam:=true
    ```

2. 地図データの保存（地図ファイルは**launchファイルを起動した場所**に保存されます。）
```
 $ ros2 run noid_lifter_mover save_map_client_node --ros-args -p map_topic:=map_nav -p map_url:=（.map/.yamlのファイル名）
```

## 3.amclを利用した自律移動
a. シミュレーション環境で自律移動を実行する場合
```
 $ ros2 launch noid_lifter_mover bringup_robot.launch.py simulation:=true slam:=false
```

b. 実機を使用して自律移動を実行する場合
```
 $ ros2 launch noid_lifter_mover bringup_robot.launch.py simulation:=false slam:=false
```

一定時間経過するとLiDARの情報と地図が表示されるので、そこからロボットを動作させてください。

※読み込ませる地図ファイルを変更する場合は、launchファイルで地図名を指定している箇所がありますので該当箇所を編集してください。
デフォルトでは下記のように設定されています。
  - bringup_robot.launch.py　→　scan_map.yaml

## 4.ハンド(TRX-S)のGrasp / Release　(実機接続時のみ使用可)

* Right Hand    
  grasp :
  ```
  $ ros2 service call /aero_controller/run_script aero_controller_msgs/srv/RunScript "{jname_interf: [], jno_interf: [{msid: 1, send_no: 11, script_no: 2, arg: 0, dio_run: 0}], timeout_sec: 5.0}"
  ``` 
    release :
  ```
  $ ros2 service call /aero_controller/run_script aero_controller_msgs/srv/RunScript "{jname_interf: [], jno_interf: [{msid: 1, send_no: 11, script_no: 3, arg: 0, dio_run: 0}], timeout_sec: 5.0}"
  ```

* Left Hand   
   grasp :
  ```
  $ ros2 service call /aero_controller/run_script aero_controller_msgs/srv/RunScript "{jname_interf: [], jno_interf: [{msid: 1, send_no: 26, script_no: 2, arg: 0, dio_run: 0}], timeout_sec: 5.0}"
  ```
   release :
  ```
  $ ros2 service call /aero_controller/run_script aero_controller_msgs/srv/RunScript "{jname_interf: [], jno_interf: [{msid: 1, send_no: 26, script_no: 3, arg: 0, dio_run: 0}], timeout_sec: 5.0}"
  ```

## その他 : 細かいインストール(必要に応じて)

- rqt-joint-trajectory-controller（noid,lifterの関節角度を変化させる）
```
$ sudo apt install ros-jazzy-rqt-joint-trajectory-controller
```

