#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import quaternion
import numpy as np
import sys
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
import tf


class rotations:
    def __init__(self):
        pass

    @classmethod
    def quat(cls, x):
        q = quaternion.quaternion(x[3], x[0], x[1], x[2])
        return q

    @classmethod
    def rpy2quat(cls, x):
        vx = np.array([x[0], 0., 0.])
        vy = np.array([0., x[1], 0.])
        vz = np.array([0., 0., x[2]])
        qx = quaternion.from_rotation_vector(vx)
        qy = quaternion.from_rotation_vector(vy)
        qz = quaternion.from_rotation_vector(vz)
        q = qz * qy * qx
        q = [q.x, q.y, q.z, q.w]
        return q

    @classmethod
    def quat2rpy(cls, x):
        q = cls.quat(x)
        rpy = np.array(tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w]))
        return rpy

    @classmethod
    def quat2rotmat(cls, x):
        mat = quaternion.as_rotation_matrix(cls.quat(x))
        return mat

    @classmethod
    def rotmat2quat(cls, x):
        q = quaternion.from_rotation_matrix(x)
        return [q.x, q.y, q.z, q.w]

    @classmethod
    def rpy2rotmat(cls, x):
        q = cls.rpy2quat(x)
        mat = cls.quat2rotmat(q)
        return mat

    @classmethod
    def rotmat2rpy(cls, x):
        q = quaternion.from_rotation_matrix(x)
        q = [q.x, q.y, q.z, q.w]
        rpy = cls.quat2rpy(q)
        return rpy

    # @classmethod
    # def conv_pose_rpy2quat(cls, x):
    #     return np.hstack((x[0:3], rot.quat2rpy(x[3:7])))

    @classmethod
    def rotate_matrix(cls, rot_mat_a, rot_mat_b):
        rot_mat_a_inv = np.linalg.inv(rot_mat_a)
        rot_mat_a2 = np.dot(rot_mat_a_inv, rot_mat_a)  # フレームの回転をキャンセルする
        rot_mat_a3 = np.dot(rot_mat_b, rot_mat_a2)  # フレームを回転する
        obj_ori_new = np.dot(rot_mat_a, rot_mat_a3)  # キャンセルした分だけ元に戻すための回転をくわえる
        return obj_ori_new

    @classmethod
    def additional_rotation(cls, x, y):
        obj_ori_new = cls.rotate_matrix(x, y)  # 元のrot_mat、オフセットのrot_mat
        return cls.rotmat2quat(obj_ori_new)

    @classmethod
    def point_transform(cls, point, add_trans):
        return np.dot(cls.rpy2rotmat(add_trans[3:6]), point.T).T + add_trans[0:3]

    @classmethod
    def point_transform2(cls, point, add_trans):
        return np.dot(cls.quat2rotmat(add_trans[3:7]), point.T).T + add_trans[0:3]

    @classmethod
    def point_transform_rpy(cls, point, add_trans):
        return np.dot(cls.rpy2rotmat(add_trans[3]), point.T).T + add_trans[0]

    @classmethod
    def point_transform_rpy2(cls, point, add_trans):
        return np.dot(cls.rpy2rotmat(add_trans[3:6]), point.T).T + add_trans[0:3]

    @classmethod
    def point_transform_quat_inv_point_only(cls, point, add_trans):
        # print("add_trans[0:3]: ", add_trans[0:3])
        return np.dot(np.linalg.inv(cls.quat2rotmat(add_trans[3:7])), np.array(point - add_trans[0:3]).T).T  # pointが角度情報をもたない場合はこっち

    @classmethod
    def point_transform_quat_point_only(cls, point, add_trans):
        return np.dot(cls.quat2rotmat(add_trans[3:7]), point.T).T + add_trans[0:3]

    @classmethod
    def point_transform_quat(cls, point, add_trans):
        trans = np.dot(cls.quat2rotmat(add_trans[3:7]), point[0:3].T).T + add_trans[0:3]
        rotmat = np.dot(cls.quat2rotmat(add_trans[3:7]), cls.quat2rotmat(point[3:7]))
        quat = cls.rotmat2quat(rotmat)
        return np.hstack((trans, quat))

    @classmethod
    def point_transform_pose(cls, point, add_trans, alpha_rot):
        point = cls.point_transform_quat_inv(point, add_trans)
        # 移動量を設定
        # 角度をrpyからquaternionへ
        alpha_rot_quat = cls.rpy2quat(alpha_rot[3:6])
        alpha_rot2 = np.hstack((alpha_rot[0:3], alpha_rot_quat))
        # 移動量の回転変換と平行移動変換を実行
        point_quat = cls.rotmat2quat(cls.rotate_matrix(cls.quat2rotmat(point[3:7]), cls.quat2rotmat(alpha_rot2[3:7])))
        point_trans = point[0:3] + alpha_rot2[0:3]
        point = np.hstack((point_trans, point_quat))
        # 物体の座標をcameraからmapへ
        point = cls.point_transform_quat(point, add_trans)
        return point

    @classmethod
    def point_transform_quat_inv(cls, point, add_trans):
        trans = np.dot(np.linalg.inv(cls.quat2rotmat(add_trans[3:7])), np.array(point[0:3] - add_trans[0:3]).T).T
        rotmat = np.dot(np.linalg.inv(cls.quat2rotmat(add_trans[3:7])), cls.quat2rotmat(point[3:7]))
        quat = cls.rotmat2quat(rotmat)
        return np.hstack((trans, quat))

    @classmethod
    def point_transform_rpy_quat_inv(cls, point, add_trans):
        trans = np.dot(np.linalg.inv(cls.quat2rotmat(add_trans[3:7])), np.array(point[0:3] - add_trans[0:3]).T).T
        rotmat = np.dot(np.linalg.inv(cls.quat2rotmat(add_trans[3:7])), cls.rpy2rotmat(point[3:6]))
        quat = cls.rotmat2quat(rotmat)
        return np.hstack((trans, quat))

    @classmethod
    def point_transform_rpy_inv(cls, point, add_trans):
        return np.dot(np.linalg.inv(cls.rpy2rotmat(add_trans[3])), np.array(point[0:3] - add_trans[0])).T  # pointが角度情報をもつ場合はこっち

    @classmethod
    def point_transform_rpy_inv_point_only(cls, point, add_trans):
        return np.dot(np.linalg.inv(cls.rpy2rotmat(add_trans[3])), np.array(point - add_trans[0]).T).T  # pointが角度情報をもたない場合はこっち


if __name__ == '__main__':
    rt = rotations()

    a = np.array([0, 0, 0])
    r = np.deg2rad(np.array([-130, 20, 30]))
    print(np.rad2deg(rt.quat2rpy(rt.rpy2quat(r))))

    ans = np.rad2deg(rt.quat2rpy(np.array([-0.5, 0.5, -0.5, 0.5])))
    print(ans)
