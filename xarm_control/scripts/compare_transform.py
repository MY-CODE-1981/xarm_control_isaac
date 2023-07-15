#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import numpy as np
import pybullet as pb
import roslib.packages
import time


class NGP_JSON:
    def __init__(self, path_json):
        d = json.load(open(path_json, "r"))
        self.camera_angle_x = self.type_float(d['camera_angle_x'])
        self.camera_angle_y = self.type_float(d['camera_angle_y'])
        self.fl_x = self.type_float(d['fl_x'])
        self.fl_y = self.type_float(d['fl_y'])
        self.k1 = self.type_float(d['k1'])
        self.k2 = self.type_float(d['k2'])
        self.k3 = self.type_float(d['k3'])
        self.k4 = self.type_float(d['k4'])
        self.p1 = self.type_float(d['p1'])
        self.p2 = self.type_float(d['p2'])
        self.ish_fisheye = self.type_bool(d['is_fisheye'])
        self.cx = self.type_float(d['cx'])
        self.cy = self.type_float(d['cy'])
        self.w = self.type_int(d['w'])
        self.h = self.type_int(d['h'])
        self.aabb_scale = self.type_float(d['aabb_scale'])
        self.transform_matrix = [np.array(i['transform_matrix']) for i in d['frames']]
        self.file_path = [i['file_path'] for i in d['frames']]
        self.sharpness = [self.type_float(i['sharpness']) for i in d['frames']]

    def type_bool(self, inp):
        return bool(inp)

    def type_int(self, inp):
        return int(inp)

    def type_float(self, inp):
        return float(inp)

    def type_array(self, inp):
        return np.array(inp)


class COMPARE_TRANSFORM:
    def __init__(self):
        use_gui = True
        dt = 1. / 240.

        if use_gui:
            pb.connect(pb.GUI)
        else:
            pb.connect(pb.DIRECT)
        pb.setPhysicsEngineParameter(numSubSteps=1)

        pb.setTimeStep(dt)

        # gui settings
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
        pb.configureDebugVisualizer(pb.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        pb.configureDebugVisualizer(pb.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        pb.configureDebugVisualizer(pb.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

        # set camera view
        pb.resetDebugVisualizerCamera(
            cameraDistance=0.2,
            cameraYaw=180,
            cameraPitch=22,
            cameraTargetPosition=[0.05, 0.05, -0.05])


class VISUALIZATION:
    def __init__(self):
        pass

    def draw_axes(self, ids=[-1, -1, -1], scale=.02, line_width=1, pos=[0, 0, 0], ori=np.eye(3), vector=np.eye(3)):
        vector = np.dot(ori, vector)

        ids[0] = pb.addUserDebugLine(pos, [pos[0] + scale * vector[0][0], pos[1] + scale * vector[1][0], pos[2] + scale * vector[2][0]],
                                     lineColorRGB=[1, 0, 0], lineWidth=line_width, replaceItemUniqueId=int(ids[0]))
        ids[1] = pb.addUserDebugLine(pos, [pos[0] + scale * vector[0][1], pos[1] + scale * vector[1][1], pos[2] + scale * vector[2][1]],
                                     lineColorRGB=[0, 1, 0], lineWidth=line_width, replaceItemUniqueId=int(ids[1]))
        ids[2] = pb.addUserDebugLine(pos, [pos[0] + scale * vector[0][2], pos[1] + scale * vector[1][2], pos[2] + scale * vector[2][2]],
                                     lineColorRGB=[0, 0, 1], lineWidth=line_width, replaceItemUniqueId=int(ids[2]))
        return ids

    def draw_line(self, ids=-1, scale=.02, line_width=1, line=[], color=[0, 0, 0], normal=False):
        if normal is True:
            new_ids = 0
            new_ids = pb.addUserDebugLine(
                line[0],
                line[0] + line[1] * scale,
                lineColorRGB=color,
                lineWidth=line_width,
                replaceItemUniqueId=int(ids))
        else:
            new_ids = 0
            new_ids = pb.addUserDebugLine(
                line[0],
                line[1],
                lineColorRGB=color,
                lineWidth=line_width,
                replaceItemUniqueId=int(ids))
        return new_ids

    def draw_lines(self, ids=[-1, -1, -1], scale=.02, line_width=1, lines=[], color=[0, 0, 0], normal=False):
        for i in range(0, len(lines), 1):
            ids[i] = self.draw_line(ids=ids[i], scale=scale, line_width=1, line=lines[i], color=color, normal=normal)

    def draw_normal(self, id_normal=[-1, -1, -1], scale=.02, line_width=1, pos=[0, 0, 0], normal=[0., 0., 1.]):
        new_id_normal = pb.addUserDebugLine(
            pos,
            [pos[0] + scale * normal[0],
             pos[1] + scale * normal[1],
             pos[2] + scale * normal[2]],
            lineColorRGB=[0, 0, 1],
            lineWidth=line_width,
            replaceItemUniqueId=int(id_normal[0]))
        return new_id_normal

    def draw_frame2(self, pos, ori, ids=[], sim=[],  scale=.02, line_width=1):
        for i in range(0, len(pos), 1):
            ids[i] = self.draw_axes(ids=ids[i], pos=pos[i], ori=ori[i], scale=scale, line_width=line_width)

    def draw_frame(self, name=[], ids=[], sim=[]):
        for i in range(0, len(name), 1):
            pos_ee, ori_ee, id = sim.get_link_pose(name[i])
            ids[i] = self.draw_axes(ids=ids[i], pos=pos_ee, ori=sim.quat2rotmat(ori_ee))

    def draw_grasp_area(self, id_lines=[], id_x_axis=[], id_y_axis=[], id_z_axis=[], grasp_area=[]):
        self.draw_lines(ids=id_lines, scale=1, line_width=2, lines=grasp_area.lines, color=[1, 0, 0], normal=False)
        self.draw_frame2([grasp_area.normals[0][0]], [grasp_area.rot_mat], ids=id_x_axis)

    def step_simulation(self, grasp_time_limit):
        finish_time = time.time() + grasp_time_limit
        while time.time() < finish_time:
            pb.stepSimulation()


if __name__ == '__main__':
    comp_transform = COMPARE_TRANSFORM()
    vis = VISUALIZATION()

    pkg_name = 'xarm_control'
    path_root = roslib.packages.get_pkg_dir(pkg_name)

    path = path_root + '/data/sample.json'
    data_sample_json = NGP_JSON(path_json=path)

    path = path_root + '/data/image2colmap/transforms.json'
    data_transform_json = NGP_JSON(path_json=path)

    # sample.jsonのターゲットフレームを描画
    pos_list = []
    ori_list = []
    for i in range(0, len(data_sample_json.transform_matrix), 1):
        pos_list.append(data_sample_json.transform_matrix[i][0:3, 3])
        ori_list.append(data_sample_json.transform_matrix[i][0:3, 0:3])

    # transform.jsonのターゲットフレームを描画
    for i in range(0, len(data_transform_json.transform_matrix), 1):
        pos_list.append(data_transform_json.transform_matrix[i][0:3, 3])
        ori_list.append(data_transform_json.transform_matrix[i][0:3, 0:3])
    id_orbit = np.ones((len(pos_list), 3)) * -1
    vis.draw_frame2(pos_list, ori_list, ids=id_orbit, scale=0.3, line_width=2)

    # 原点描画
    ids_obj_target_axis = np.array([-1, -1, -1])  # ライン描画用変数
    vis.draw_axes(ids=ids_obj_target_axis, pos=np.array([0, 0, 0]),
                  ori=np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), scale=0.3,  line_width=2)  # 手首座標系を描画

    while(1):
        vis.step_simulation(1)
