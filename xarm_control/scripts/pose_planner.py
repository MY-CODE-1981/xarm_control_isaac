#!/usr/bin/env python
# coding: UTF-8
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
import moveit_commander
import numpy as np
import roslib.packages
import rospy
from rotations import rotations as rt
import sys
import tf
import tf2_ros
import time
from xarm_control.srv import image_save


class PosePlanner(object):
    def __init__(self, filepath):
        data = np.load(filepath)
        self.pos_list = data['pos_list']
        self.ori_list = data['ori_list']
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.count = 0
        self.init_pose()

    def pose_to_transform(self, pose):
        transform = Transform()
        transform.translation.x = pose[0]
        transform.translation.y = pose[1]
        transform.translation.z = pose[2]
        transform.rotation.x = pose[3]
        transform.rotation.y = pose[4]
        transform.rotation.z = pose[5]
        transform.rotation.w = pose[6]
        return transform

    def init_pose(self):
        # tfを発行
        actual_camera_trans = TransformStamped()
        actual_camera_trans.header.stamp = rospy.Time.now()
        actual_camera_trans.header.frame_id = 'link_base'
        actual_camera_trans.child_frame_id = 'tf_goal'
        actual_camera_trans.transform = self.pose_to_transform([0.131, 0.000, 0.376, 1.000, 0.000, 0.014, 0.000])
        self.broadcaster.sendTransform(actual_camera_trans)

        try:
            moveit_commander.roscpp_initialize(sys.argv)

            # ノードの生成
            # MoveGroupCommanderの準備
            move_group = moveit_commander.MoveGroupCommander("xarm7_urdf")

            # 関節の角度でゴール状態を指定
            joint_goal = [0.0000, -1.4232, 0.0000, 0.2430, 0.0000, 1.6315, 0.0000]
            move_group.set_joint_value_target(joint_goal)
            # モーションプランの計画と実行
            move_group.go(wait=True)
            # 後処理
            move_group.stop()

        except BaseException:
            pass

    def image_save_control_client(self, save_request):
        rospy.wait_for_service('image_save_control')
        try:
            image_save_control = rospy.ServiceProxy('image_save_control', image_save)
            resp1 = image_save_control(save_request)
            return resp1.save_answer
        except BaseException:
            pass

    def calc(self):
        save_answer = self.image_save_control_client(False)
        i = self.count
        (trans_x, trans_y, trans_z) = self.pos_list[i]
        (ori_r, ori_p, ori_y) = rt.rotmat2rpy(self.ori_list[i])
        (quat_x, quat_y, quat_z, quat_w) = rt.rotmat2quat(self.ori_list[i])
        # tfを発行
        actual_camera_trans = TransformStamped()
        actual_camera_trans.header.stamp = rospy.Time.now()
        actual_camera_trans.header.frame_id = 'link_base'
        actual_camera_trans.child_frame_id = 'tf_goal'
        actual_camera_trans.transform = self.pose_to_transform([trans_x, trans_y, trans_z, quat_x, quat_y, quat_z, quat_w])
        self.broadcaster.sendTransform(actual_camera_trans)
        ###
        print(trans_x, trans_y, trans_z)
        print(ori_r, ori_p, ori_y)

        try:
            # MoveitCommanderの初期化
            moveit_commander.roscpp_initialize(sys.argv)

            # ノードの生成
            # MoveGroupCommanderの準備
            move_group = moveit_commander.MoveGroupCommander("xarm7_urdf")

            # エンドポイントの姿勢でゴール状態を指定
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position = Vector3(trans_x, trans_y, trans_z)
            q = tf.transformations.quaternion_from_euler(ori_r, ori_p, ori_y)
            pose_goal.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            move_group.set_pose_target(pose_goal)

            # モーションプランの計画と実行
            move_group.go(wait=True)

            # 後処理
            move_group.stop()
            move_group.clear_pose_targets()

        except BaseException:
            pass

        self.count += 1

        save_answer = self.image_save_control_client(True)

        time.sleep(2)

        if self.count == len(self.pos_list) / 2:
            self.init_pose()

        if save_answer is True:
            save_answer = self.image_save_control_client(False)
        else:
            print("miss to save")

    def spin_once(self):
        self.calc()


if __name__ == '__main__':
    rospy.init_node('calibration_transform_obj')
    r = rospy.Rate(10.0)
    pkg_name = 'xarm_control'
    path_root = roslib.packages.get_pkg_dir(pkg_name)
    path = path_root + '/data/data.npz'
    mock = PosePlanner(path)
    while not rospy.is_shutdown():
        try:
            mock.spin_once()
            r.sleep()
        except rospy.exceptions.ROSInterruptException:
            break
