#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import json
import message_filters
import numpy as np
import ros_numpy
import roslib.packages
import rospy
from rotations import rotations as rt
from sensor_msgs.msg import Image
import tf
from xarm_control.srv import image_save, image_saveResponse


class ImageGet:
    def __init__(self, filepath_image_folder, filepath_saved_json):
        self.count = 0
        self.count_filename = 0
        self.count_no_ini = 0
        self.flag_save_on = False
        self.path = filepath_image_folder
        image_sub = message_filters.Subscriber('rgb', Image)
        ts = message_filters.ApproximateTimeSynchronizer([image_sub], 2, 1)
        ts.registerCallback(self.callback)
        self.save_answer = rospy.Service('image_save_control', image_save, self.handle_image_save_control)
        self.tf_buffer = tf.TransformListener()
        self.set_json()
        self.filepath_saved_json = filepath_saved_json
        self.offset = np.array([0.25, 0, 0.25])

    def set_json(self):
        self.saved_data = {
            "camera_angle_x": 0.0,
            "camera_angle_y": 0.0,
            "fl_x": 607.7645263671875,
            "fl_y": 607.7645263671875,
            "k1": 0.0,
            "k2": 0.0,
            "k3": 0.0,
            "k4": 0.0,
            "p1": 0.0,
            "p2": 0.0,
            "is_fisheye": False,
            "cx": 320.0,
            "cy": 240.0,
            "w": 640.0,
            "h": 480.0,
            "aabb_scale": 128,
            "frames": []
        }

    def set_json_frames(self, file_path, transform_matrix):
        sub_saved_data = {
            "file_path": file_path,
            "sharpness": 300.0,
            "transform_matrix": transform_matrix
        }
        return sub_saved_data

    def handle_image_save_control(self, req):
        self.save_answer = req.save_request
        return self.save_answer

    def rot_offset(self, rot_mat, offset):
        add_rot = rt.rpy2rotmat(np.deg2rad(offset))
        return np.dot(rot_mat, add_rot)

    def callback(self, data):
        data = ros_numpy.numpify(data)
        data = cv2.cvtColor(data, cv2.COLOR_BGR2RGB)

        if self.count_no_ini > 0:
            self.flag_save_on = True

        if self.save_answer is True:
            filename = str(self.count_filename).zfill(4)
            filename = self.path + str(filename) + '.jpg'
            print('count_filename: ' + filename)

            cv2.imwrite(filename, data, [cv2.IMWRITE_JPEG_QUALITY, 100])
            self.count_filename += 1
            self.count = 0
            image_saveResponse(True)
            self.save_answer = False

            # tf
            tf_camera = self.tf_buffer.lookupTransform('ActionGraph', 'camera_color_optical_frame', rospy.Time(0))
            rot_offset_ini_left = [180.0, 0.0, 0.0]

            transform_matrix = np.zeros([4, 4])
            transform_matrix[3, 3] = 1
            transform_matrix[0:3, 0:3] = self.rot_offset(rt.quat2rotmat(tf_camera[1]), rot_offset_ini_left)
            transform_matrix[0:3, 3] = (tf_camera[0] - self.offset)
            self.saved_data["frames"].append(self.set_json_frames(filename, transform_matrix.tolist()))

            for i, f in enumerate(self.saved_data["frames"]):
                point = np.array(f["transform_matrix"])[0:3, 3]
                dist = np.sqrt(np.sum(np.power(point, 2)))
                tmp = np.array(f["transform_matrix"])
                tmp[0:3, 3] = tmp[0:3, 3] * 4 / dist
                print("camera distance from origin", dist)
                f["transform_matrix"] = tmp.tolist()
                self.saved_data["frames"][i]["transform_matrix"] = f["transform_matrix"]

            f = open(self.filepath_saved_json, 'w')
            json.dump(self.saved_data, f, ensure_ascii=False, indent=4, sort_keys=True, separators=(',', ': '))

        else:
            image_saveResponse(False)

        self.count += 1
        self.count_no_ini += 1


if __name__ == '__main__':
    rospy.init_node('rosinit')
    # データを保存
    pkg_name = 'xarm_control'
    path_root = roslib.packages.get_pkg_dir(pkg_name)
    path0 = path_root + '/data/image/'
    path1 = path_root + '/data/sample.json'
    imGet = ImageGet(path0, path1)
    r = rospy.Rate(30.0)

    while not rospy.is_shutdown():
        try:
            r.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass  # break
