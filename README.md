# xarm_control_isaac
## 目的
xarmでnerfをするためのテストベンチをisaac-sim上で作成すること

## 実行環境
```
　ubuntu 18.04
　ros-melodic
　python 2.7/3.6
```

## pythonモジュール
```
    astropy
    json
    geometry_msgs
    message_filters
    moveit
    moveit_commander
    numpy
    numpy-quaternion
    opencv-python
    opencv-contrib-python
    pybullet
    ros_numpy
    roslib
    rospy
    sensor_msgs
    tf2_ros
    time
    transforms3d
    tf
```

## ビルド
```
　$ source /opt/ros/melodic/setup.bash
　$ mkdir -p omniverse_ros/src
　$ cd omniverse_ros/src
　$ git clone git@github.com:MY-CODE-1981/xarm_control_isaac.git
　$ wstool init . ./xarm_control_isaac/xarm_control/.rosinstall
　$ cd ..
　$ catkin_make -j4 --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE='/usr/bin/python3.6'
　$ source devel/setup.bash
```

## 使い方
```
ロボットモデルをrvizで表示
　$ roslaunch xarm_control test_robot_model.launch

moveitでアーム先端（link7）のターゲットとなるtfリストを作成
　$ rosrun xarm_control generate_target_frame.py

シミュレータを準備して起動する
　omniverseをダウンロード
　omniverse-launcher-linux.AppImage
　ダウンロード先に移動して権限変更
　sudo chmod a+x omniverse-launcher-linux.AppImage
　アプリを起動
　./omniverse-launcher-linux.AppImage
　各種アプリをインストール
　　isaac-sim 2022.2.1
　　cache 2022.2.0
　　nucleus-service 2022.4.2
　（assetを使いたいときはnucleusで常にomniverseアカウントにログインが必要）

isaac-simを起動して以下のファイルを開く
　xarm_control_isaac/xarm_control/model/isaac/xarm7_camera/xarm_control_isaac.usda

isaac-simのインターフェースからPlay(SPACE)ボタンを押す

ターミナルを新しく開き、
　$ source /opt/ros/melodic/setup.bash
  $ cd 
　$ source devel/setup.bash
　$ roslaunch xarm_control xarm_isaac_execution_camera.launch

ターミナルを新しく開き、
　$ source /opt/ros/melodic/setup.bash
　$ source devel/setup.bash
　$ rosrun xarm_control pose_planner.py

ターミナルを新しく開き、
　$ source /opt/ros/melodic/setup.bash
　$ source devel/setup.bash
　$ rosrun xarm_control save_image.py

instant-ngpをインストールしてから以下を実行し、transform.jsonを作成
　$ cd [parrent folder path of omniverse_ros]/omniverse_ros/src/xarm_control_isaac/xarm_control/data/image
　$ python3 [parrent folder path of instant-ngp]/instant-ngp/scripts/colmap2nerf.py --colmap_matcher exhaustive --run_colmap --aabb_scale 128 --images /[parrent folder path of omniverse_ros]/omniverse_ros/src/xarm_control_isaac/xarm_control/data/image
　（質問文ではYを選択）

instant-ngpによるtransform.jsonと自作したsample.jsonを比較
　$ rosrun xarm_control compare_transform.py

instant-ngpのあるフォルダに移動して
　$ ./instant-ngp --mode nerf --scene /home/initial/workspace/omniverse_ros2/src/xarm_control_isaac/xarm_control/data/
　$ ./instant-ngp --mode nerf --scene /home/initial/workspace/omniverse_ros2/src/xarm_control_isaac/xarm_control/data/image2colmap
```

## ライセンス
BSD ライセンス