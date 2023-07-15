#!/usr/bin/env python
# -*- coding: utf-8 -*-
import astropy.coordinates
import numpy as np
import pybullet as pb
import roslib.packages
from rotations import rotations as rt
import time
from transforms3d import euler


class TARGET_FRAMES:
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

    def get_given_point(self, dist, theta_rad, phi_rad):
        carts = astropy.coordinates.spherical_to_cartesian(dist, theta_rad, phi_rad)
        flip_carts = np.array(carts) * -1
        quat = euler.euler2quat(phi_rad + np.pi, np.pi / 2 - theta_rad, np.pi, axes='sxyz')
        # quat = rt.rpy2quat([phi_rad + np.pi, np.pi / 2 - theta_rad, np.pi])
        return np.hstack([flip_carts, quat])

    def sphere_set(self, phi_init=np.deg2rad(-45),
                   phi_span=np.deg2rad(275),
                   theta_init=np.deg2rad(60),
                   theta_span=np.deg2rad(30),
                   num_grasps_per_cycle=50,
                   num_cycles_to_grasp=1,
                   init_grasp_distance=.25,
                   offset=np.array([0.25, 0, 0.05])):
        output = []

        phi = phi_init
        increment_phi = phi_span/num_grasps_per_cycle

        theta = theta_init
        increment_theta = theta_span / num_cycles_to_grasp

        for theta_i in range(0, (num_cycles_to_grasp + 1)):
            for phi_i in range(0, (num_grasps_per_cycle)):
                point = self.get_given_point(dist=init_grasp_distance,
                                             theta_rad=(-theta) + increment_theta * theta_i,
                                             phi_rad=(-phi) + increment_phi * phi_i)
                output.append(point)
        output = np.array(output)
        output[:, 0:3] = output[:, 0:3] + offset
        return output

    def step_simulation(self, grasp_time_limit):
        finish_time = time.time() + grasp_time_limit
        while time.time() < finish_time:
            pb.stepSimulation()

    def change_order(self, data):
        ind_half = len(data)/2
        data_0 = data[:ind_half, :]
        data_1 = data[ind_half:, :]

        ind_half = len(data_0)/2
        data_0_0 = data_0[:ind_half, :]
        data_0_1 = data_0[ind_half:, :]

        ind_half = len(data_1)/2
        data_1_0 = data_1[:ind_half, :]
        data_1_1 = data_1[ind_half:, :]

        data_1st_half = np.vstack([data_0_0, np.flipud(data_1_0)])
        data_2nd_half = np.vstack([np.flipud(data_0_1), data_1_1])

        return np.vstack([data_1st_half, data_2nd_half])


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

    def draw_frame2(self, pos, ori, ids=[], sim=[]):
        for i in range(0, len(pos), 1):
            ids[i] = self.draw_axes(ids=ids[i], pos=pos[i], ori=ori[i])

    def draw_frame(self, name=[], ids=[], sim=[]):
        for i in range(0, len(name), 1):
            pos_ee, ori_ee, id = sim.get_link_pose(name[i])
            ids[i] = self.draw_axes(ids=ids[i], pos=pos_ee, ori=sim.quat2rotmat(ori_ee))

    def draw_grasp_area(self, id_lines=[], id_x_axis=[], id_y_axis=[], id_z_axis=[], grasp_area=[]):
        self.draw_lines(ids=id_lines, scale=1, line_width=2, lines=grasp_area.lines, color=[1, 0, 0], normal=False)
        self.draw_frame2([grasp_area.normals[0][0]], [grasp_area.rot_mat], ids=id_x_axis)


if __name__ == '__main__':
    vis = VISUALIZATION()

    # アーム先端のエンドエフェクタのターゲットフレームの設定と描画
    tg_frames = TARGET_FRAMES()

    ans = np.array(tg_frames.sphere_set(phi_init=np.deg2rad(-45), phi_span=np.deg2rad(275), theta_init=np.deg2rad(60), theta_span=np.deg2rad(30)))
    print(ans)
    pos_list = ans[:, 0:3]
    ori_list = []
    for i in range(0, len(pos_list), 1):
        ori_list.append(rt.quat2rotmat(ans[i, 3:7]))
    ori_list = np.array(ori_list)
    pos_list = tg_frames.change_order(pos_list)
    ori_list = tg_frames.change_order(np.array(ori_list))

    id_orbit = np.ones((len(pos_list), 3)) * -1
    vis.draw_frame2(pos_list, ori_list, ids=id_orbit)

    # 原点描画
    ids_obj_target_axis = np.array([-1, -1, -1])
    vis.draw_axes(ids=ids_obj_target_axis, pos=np.array([0, 0, 0]), ori=np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))

    # データを保存
    pkg_name = 'xarm_control'
    path_root = roslib.packages.get_pkg_dir(pkg_name)
    path = path_root + '/data/data.npz'
    np.savez(path,
             pos_list=pos_list,
             ori_list=ori_list)

    # pybullet上で点を繰り返し表示
    while(1):
        tg_frames.step_simulation(1)
