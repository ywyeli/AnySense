# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


import os
import json
from math import pi, sin, cos, atan, sqrt
import numpy as np
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
from tqdm import trange
from scipy.spatial.transform import Rotation as Ro
import argparse
from argparse import RawTextHelpFormatter
import toml
import psutil
import time
import math
from format_utils import *






# write json file
def nuread(pathh, path_position, path_timestamp, root_path, scene_numbers, args):
    files = os.listdir(pathh)
    scene_json = []
    sensor_json = []
    calibrated_sensor_json = []
    log_json = []
    sample_json = []
    sample_annotation_json = []
    sample_data_json = []
    instance_json = []
    ego_pose_json = []
    attribute_json = []
    category_json = []
    
    visibility_json = []
    map_json = []

    instances = {}

    instance_token = 0

    params = toml.load(f"../carla_nus/hyperparams/{args.hyperparams}")

    lids = params['lidar']
    LIDAR_HEIGHT_POS = lids['GLOBAL_HEIGHT_POS']

    sensor_list = ['LIDAR_TOP', 'CAM_BACK', 'CAM_BACK_LEFT', 'CAM_BACK_RIGHT',
                   'CAM_FRONT', 'CAM_FRONT_LEFT', 'CAM_FRONT_RIGHT']

    # label_index
    label_index = 0

    # scene_numbers
    # scene_numbers = 30

    # calibrated_sensor_token
    calibrated_sensor_token = 0

    # scene_name
    scene_name = 1001

    # scene_token
    scene_token = 70000000000000000000000000000001

    # sample_token
    sample_token = 10000000000000000000000000000001

    # sample_annotation_token
    sample_annotation_token = 20000000000000000000000000000001

    # ego_pose_token
    ego_pose_token = 0

    # sample_data_token
    sample_data_token_lidar_top = 30000000000000000000000000000001

    sample_data_token_cam_front = 30100000000000000000000000000001

    sample_data_token_cam_back = 30200000000000000000000000000001

    sample_data_token_cam_front_left = 30300000000000000000000000000001

    sample_data_token_cam_front_right = 30400000000000000000000000000001

    sample_data_token_cam_back_left = 30500000000000000000000000000001

    sample_data_token_cam_back_right = 30600000000000000000000000000001

    #######################################################################
    category_token = None

    attribute_tokens = None

    check = []

    n = 0
    it = 0

    for sum_no in trange(scene_numbers):
        # print(sum_no)
        check_no = 1500_0000_0000_0000 + 40 * (it + 1) + n
        check_2 = pathh + '/' + str(check_no) + '.txt'

        while os.path.exists(check_2) == 0:
            n += 10_0000
            it = 0
            check_no = 1500000000000000 + 40 * (it + 1) + n
            check_2 = pathh + '/' + str(check_no) + '.txt'

            if n > 90_0000_0000:
                break

        if n > 90_0000_0000:
            break

        if os.path.exists(check_2):
            for ic in range(40):
                check.append(1500000000000000 + 40*it + (ic+1) + n)
            it += 1
        else:
            print('Error!')

    # open txt file
    for ie in trange(len(check)):
        file_no = check[ie]

        label_index = str(file_no)

        # open position
        label_2 = pathh + '/' + str(file_no) + '.txt'

        if os.path.exists(label_2):
            datas = openreadtxt(label_2)

            position_2 = path_position + '/' + str(file_no) + '.txt'
            position_data = openreadtxt(position_2)
            go_x, go_y, go_z, ego_roll, ego_pitch, ego_yaw = position_data[0]

            ego_yaw = - float(ego_yaw)
            ego_x = float(go_x)
            ego_y = - float(go_y)
            ego_z = float(go_z)

            timestamp_2 = path_timestamp + '/' + str(file_no) + '.txt'
            timestamp_data = openreadtxt(timestamp_2)
            timestamp = timestamp_data[0][0]

            if sample_token % 40 == 1:
                sample_json.append(
                    {'token': str(sample_token),
                     'timestamp': int(timestamp),
                     'prev': '',
                     'next': str(sample_token + 1),
                     'scene_token': str(scene_token)}
                )
            elif sample_token % 40 == 0:
                sample_json.append(
                    {'token': str(sample_token),
                     'timestamp': int(timestamp),
                     'prev': str(sample_token - 1),
                     'next': '',
                     'scene_token': str(scene_token)}
                )
            else:
                sample_json.append(
                    {'token': str(sample_token),
                     'timestamp': int(timestamp),
                     'prev': str(sample_token - 1),
                     'next': str(sample_token + 1),
                     'scene_token': str(scene_token)}
                )

            for s in sensor_list:
                if s == 'LIDAR_TOP':
                    fileformat = 'pcd'
                    forma = '.pcd.bin'
                    height = 0
                    width = 0
                    calibrated_sensor_token = '90000000000000000000000000000000'
                    ego_pose_token = sample_data_token_lidar_top

                else:
                    fileformat = 'jpg'
                    forma = '.jpg'
                    height = 900
                    width = 1600
                    if s == 'CAM_FRONT':
                        calibrated_sensor_token = '90000000000000000000000000000001'
                        ego_pose_token = sample_data_token_cam_front
                    if s == 'CAM_BACK':
                        calibrated_sensor_token = '90000000000000000000000000000002'
                        ego_pose_token = sample_data_token_cam_back
                    if s == 'CAM_FRONT_LEFT':
                        calibrated_sensor_token = '90000000000000000000000000000003'
                        ego_pose_token = sample_data_token_cam_front_left
                    if s == 'CAM_FRONT_RIGHT':
                        calibrated_sensor_token = '90000000000000000000000000000004'
                        ego_pose_token = sample_data_token_cam_front_right
                    if s == 'CAM_BACK_LEFT':
                        calibrated_sensor_token = '90000000000000000000000000000005'
                        ego_pose_token = sample_data_token_cam_back_left
                    if s == 'CAM_BACK_RIGHT':
                        calibrated_sensor_token = '90000000000000000000000000000006'
                        ego_pose_token = sample_data_token_cam_back_right

                ego_pose_json.append(
                    {'token': str(ego_pose_token),
                     'timestamp': int(timestamp),
                     'rotation': [cos(ego_yaw / 2), 0, 0, sin(ego_yaw / 2)],
                     'translation': [ego_x, ego_y, ego_z]}
                )

                filename = 'samples/' + s + '/n008-2018-08-01-00-00-00-0400__' + s + '__' + label_index + forma

                if sample_token % 40 == 1:
                    sample_data_json.append(
                        {'token': str(ego_pose_token),
                         'sample_token': str(sample_token),
                         'ego_pose_token': str(ego_pose_token),
                         'calibrated_sensor_token': calibrated_sensor_token,
                         'timestamp': int(timestamp),
                         'fileformat': fileformat,
                         'is_key_frame': True,
                         'height': height,
                         'width': width,
                         'filename': filename,
                         'prev': '',
                         'next': str(ego_pose_token + 1)}
                    )
                elif sample_token % 40 == 0:
                    sample_data_json.append(
                        {'token': str(ego_pose_token),
                         'sample_token': str(sample_token),
                         'ego_pose_token': str(ego_pose_token),
                         'calibrated_sensor_token': calibrated_sensor_token,
                         'timestamp': int(timestamp),
                         'fileformat': fileformat,
                         'is_key_frame': True,
                         'height': height,
                         'width': width,
                         'filename': filename,
                         'prev': str(ego_pose_token - 1),
                         'next': ''}
                    )
                else:
                    sample_data_json.append(
                        {'token': str(ego_pose_token),
                         'sample_token': str(sample_token),
                         'ego_pose_token': str(ego_pose_token),
                         'calibrated_sensor_token': calibrated_sensor_token,
                         'timestamp': int(timestamp),
                         'fileformat': fileformat,
                         'is_key_frame': True,
                         'height': height,
                         'width': width,
                         'filename': filename,
                         'prev': str(ego_pose_token - 1),
                         'next': str(ego_pose_token + 1)}
                    )

            # next sample_data_token
            sample_data_token_lidar_top = sample_data_token_lidar_top + 1
            sample_data_token_cam_front = sample_data_token_cam_front + 1
            sample_data_token_cam_back = sample_data_token_cam_back + 1
            sample_data_token_cam_front_left = sample_data_token_cam_front_left + 1
            sample_data_token_cam_front_right = sample_data_token_cam_front_right + 1
            sample_data_token_cam_back_left = sample_data_token_cam_back_left + 1
            sample_data_token_cam_back_right = sample_data_token_cam_back_right + 1

            # overlap
            c_i = 0
            H, W, L, X, Y, Z, D, theta, pool, result = [], [], [], [], [], [], [], [], [], []

            for data in datas:
                H.append(float(data[8]))
                W.append(float(data[9]))
                L.append(float(data[10]))
                X.append(float(data[11]))
                Y.append(float(data[12]))
                Z.append(float(data[13]))
                # D.append(sqrt(X[c_i] ** 2 + Z[c_i] ** 2))
                theta.append(float(data[14]))

                z1, z2 = [], []

                R = [[cos(theta[c_i]), sin(theta[c_i])], [-sin(theta[c_i]), cos(theta[c_i])]]
                T = [X[c_i], Z[c_i]]


                pp = [L[c_i] / 2, W[c_i] / 2]
                x1, y1 = np.matmul(R, pp) + T
                z1.append(Y[c_i] / sqrt(x1 ** 2 + y1 ** 2))
                z2.append((Y[c_i] - H[c_i]) / sqrt(x1 ** 2 + y1 ** 2))
                if x1 >= 0 and y1 >= 0:
                    psi0 = atan(x1 / y1)
                if x1 < 0 and y1 >= 0:
                    psi0 = atan(x1 / y1)
                if x1 < 0 and y1 < 0:
                    psi0 = atan(x1 / y1) - pi
                if x1 >= 0 and y1 < 0:
                    psi0 = atan(x1 / y1) + pi


                pp = [-L[c_i] / 2, W[c_i] / 2]
                x1, y1 = np.matmul(R, pp) + T
                z1.append(Y[c_i] / sqrt(x1 ** 2 + y1 ** 2))
                z2.append((Y[c_i] - H[c_i]) / sqrt(x1 ** 2 + y1 ** 2))
                if x1 >= 0 and y1 >= 0:
                    psi1 = atan(x1 / y1)
                if x1 < 0 and y1 >= 0:
                    psi1 = atan(x1 / y1)
                if x1 < 0 and y1 < 0:
                    psi1 = atan(x1 / y1) - pi
                if x1 >= 0 and y1 < 0:
                    psi1 = atan(x1 / y1) + pi


                pp = [-L[c_i] / 2, -W[c_i] / 2]
                x1, y1 = np.matmul(R, pp) + T
                z1.append(Y[c_i] / sqrt(x1 ** 2 + y1 ** 2))
                z2.append((Y[c_i] - H[c_i]) / sqrt(x1 ** 2 + y1 ** 2))
                if x1 >= 0 and y1 >= 0:
                    psi2 = atan(x1 / y1)
                if x1 < 0 and y1 >= 0:
                    psi2 = atan(x1 / y1)
                if x1 < 0 and y1 < 0:
                    psi2 = atan(x1 / y1) - pi
                if x1 >= 0 and y1 < 0:
                    psi2 = atan(x1 / y1) + pi


                pp = [L[c_i] / 2, -W[c_i] / 2]
                x1, y1 = np.matmul(R, pp) + T
                z1.append(Y[c_i] / sqrt(x1 ** 2 + y1 ** 2))
                z2.append((Y[c_i] - H[c_i]) / sqrt(x1 ** 2 + y1 ** 2))
                if x1 >= 0 and y1 >= 0:
                    psi3 = atan(x1 / y1)
                if x1 < 0 and y1 >= 0:
                    psi3 = atan(x1 / y1)
                if x1 < 0 and y1 < 0:
                    psi3 = atan(x1 / y1) - pi
                if x1 >= 0 and y1 < 0:
                    psi3 = atan(x1 / y1) + pi

                if max(psi0, psi1, psi2, psi3) - min(psi0, psi1, psi2, psi3) > pi:
                    if psi0 < 0:
                        psi0 += 2 * pi
                    if psi1 < 0:
                        psi1 += 2 * pi
                    if psi2 < 0:
                        psi2 += 2 * pi
                    if psi3 < 0:
                        psi3 += 2 * pi
                    pool.append(
                        [[psi0, z1[0]], [psi0, z2[0]], [psi1, z1[1]], [psi1, z2[1]], [psi2, z1[2]], [psi2, z2[2]],
                         [psi3, z1[3]], [psi3, z2[3]]])
                    D.append(sqrt(X[c_i] ** 2 + Z[c_i] ** 2))

                elif min(psi0, psi1, psi2, psi3) < 0:
                    pool.append(
                        [[psi0, z1[0]], [psi0, z2[0]], [psi1, z1[1]], [psi1, z2[1]], [psi2, z1[2]], [psi2, z2[2]],
                         [psi3, z1[3]], [psi3, z2[3]]])
                    D.append(sqrt(X[c_i] ** 2 + Z[c_i] ** 2))
                    psi0 += 2 * pi
                    psi1 += 2 * pi
                    psi2 += 2 * pi
                    psi3 += 2 * pi
                    pool.append(
                        [[psi0, z1[0]], [psi0, z2[0]], [psi1, z1[1]], [psi1, z2[1]], [psi2, z1[2]], [psi2, z2[2]],
                         [psi3, z1[3]], [psi3, z2[3]]])
                    D.append(sqrt(X[c_i] ** 2 + Z[c_i] ** 2))

                else:
                    pool.append(
                        [[psi0, z1[0]], [psi0, z2[0]], [psi1, z1[1]], [psi1, z2[1]], [psi2, z1[2]], [psi2, z2[2]],
                         [psi3, z1[3]], [psi3, z2[3]]])
                    D.append(sqrt(X[c_i] ** 2 + Z[c_i] ** 2))

                c_i += 1

            #########################################################################################################

            for data in datas:
                instance_token_h = 'it' + str(file_no)[2:8] + '{0:08d}'.format(scene_name) + '{0:08d}'.format(int(data[-2]))
                speed_h = float(data[-1])

                if data[0] == 'car':
                    category_token = 'categorylidardetcategory00000car'
                    if speed_h >= 1:
                        attribute_tokens = 'vehiclemovingvehiclemovingvehicl'
                    else:
                        attribute_tokens = 'vehicle_stopped_vehicle_stopped_'
                    instance_token = instance_token_h + '00000car'
                    # print(data)
                elif data[0] == 'bus':
                    category_token = 'categorylidardetcategory00000bus'
                    if speed_h >= 1:
                        attribute_tokens = 'vehiclemovingvehiclemovingvehicl'
                    else:
                        attribute_tokens = 'vehicle_stopped_vehicle_stopped_'
                    instance_token = instance_token_h + '00000bus'
                    # print(data)
                elif data[0] == 'truck':
                    category_token = 'categorylidardetcategory000truck'
                    if speed_h >= 1:
                        attribute_tokens = 'vehiclemovingvehiclemovingvehicl'
                    else:
                        attribute_tokens = 'vehicle_stopped_vehicle_stopped_'
                    instance_token = instance_token_h + '000truck'
                    # print(data)

                elif data[0] == 'bicycle':
                    category_token = 'categorylidardetcategory0bicycle'
                    if speed_h >= 1:
                        attribute_tokens = 'cyclewithridercyclewithridercycl'
                    else:
                        attribute_tokens = 'bicycle_stopped_bicycle_stopped_'
                    instance_token = instance_token_h + '0bicycle'
                    # print(data)
                elif data[0] == 'motorcycle':
                    category_token = 'categorylidardetcategory000motor'
                    if speed_h >= 1:
                        attribute_tokens = 'cyclewithridercyclewithridercycl'
                    else:
                        attribute_tokens = 'bicycle_stopped_bicycle_stopped_'
                    instance_token = instance_token_h + '000motor'
                    # print(data)

                elif data[0] == 'pedestrian':
                    category_token = 'categorylidardetcategory000adult'
                    if speed_h >= 0.2:
                        attribute_tokens = 'pedestrianmovingpedestrianmoving'
                    else:
                        attribute_tokens = 'standingstandingstandingstanding'
                    instance_token = instance_token_h + '000adult'
                    # print(data)

                else:
                    with open('error.txt', 'w') as ftt:
                        ftt.write('Error!')
                
                # print(instance_token)

                x0 = float(data[11])
                y0 = float(data[12])
                z0 = float(data[13])
                h0 = float(data[8])
                w0 = float(data[9])
                l0 = float(data[10])
                d0 = sqrt(x0 ** 2 + z0 ** 2)
                theta0 = float(data[14])

                ###
                s = []
                p_pool = []
                for D_i in range(len(D)):
                    if d0 > D[D_i]:
                        s.append(D_i)

                for s_i in range(len(s)):
                    p_pool.append(pool[s[s_i]])
                    # print(p_pool)

                z1, z2 = [], []

                R = [[cos(theta0), sin(theta0)], [-sin(theta0), cos(theta0)]]
                T = [x0, z0]

                pp = [l0 / 2, w0 / 2]
                x1, y1 = np.matmul(R, pp) + T
                z1.append(y0 / sqrt(x1 ** 2 + y1 ** 2))
                z2.append((y0 - h0) / sqrt(x1 ** 2 + y1 ** 2))
                if x1 >= 0 and y1 >= 0:
                    psi0 = atan(x1 / y1)
                if x1 < 0 and y1 >= 0:
                    psi0 = atan(x1 / y1)
                if x1 < 0 and y1 < 0:
                    psi0 = atan(x1 / y1) - pi
                if x1 >= 0 and y1 < 0:
                    psi0 = atan(x1 / y1) + pi

                pp = [-l0 / 2, w0 / 2]
                x1, y1 = np.matmul(R, pp) + T
                z1.append(y0 / sqrt(x1 ** 2 + y1 ** 2))
                z2.append((y0 - h0) / sqrt(x1 ** 2 + y1 ** 2))
                if x1 >= 0 and y1 >= 0:
                    psi1 = atan(x1 / y1)
                if x1 < 0 and y1 >= 0:
                    psi1 = atan(x1 / y1)
                if x1 < 0 and y1 < 0:
                    psi1 = atan(x1 / y1) - pi
                if x1 >= 0 and y1 < 0:
                    psi1 = atan(x1 / y1) + pi

                pp = [-l0 / 2, -w0 / 2]
                x1, y1 = np.matmul(R, pp) + T
                z1.append(y0 / sqrt(x1 ** 2 + y1 ** 2))
                z2.append((y0 - h0) / sqrt(x1 ** 2 + y1 ** 2))
                if x1 >= 0 and y1 >= 0:
                    psi2 = atan(x1 / y1)
                if x1 < 0 and y1 >= 0:
                    psi2 = atan(x1 / y1)
                if x1 < 0 and y1 < 0:
                    psi2 = atan(x1 / y1) - pi
                if x1 >= 0 and y1 < 0:
                    psi2 = atan(x1 / y1) + pi

                pp = [l0 / 2, -w0 / 2]
                x1, y1 = np.matmul(R, pp) + T
                z1.append(y0 / sqrt(x1 ** 2 + y1 ** 2))
                z2.append((y0 - h0) / sqrt(x1 ** 2 + y1 ** 2))
                if x1 >= 0 and y1 >= 0:
                    psi3 = atan(x1 / y1)
                if x1 < 0 and y1 >= 0:
                    psi3 = atan(x1 / y1)
                if x1 < 0 and y1 < 0:
                    psi3 = atan(x1 / y1) - pi
                if x1 >= 0 and y1 < 0:
                    psi3 = atan(x1 / y1) + pi

                if max(psi0, psi1, psi2, psi3) - min(psi0, psi1, psi2, psi3) > pi:
                    if psi0 < 0:
                        psi0 += 2 * pi
                    if psi1 < 0:
                        psi1 += 2 * pi
                    if psi2 < 0:
                        psi2 += 2 * pi
                    if psi3 < 0:
                        psi3 += 2 * pi
                    p_point = [[psi0, z1[0]], [psi0, z2[0]], [psi1, z1[1]], [psi1, z2[1]], [psi2, z1[2]], [psi2, z2[2]],
                         [psi3, z1[3]], [psi3, z2[3]]]

                elif max(psi0, psi1, psi2, psi3) < 0:
                    psi0 += 2 * pi
                    psi1 += 2 * pi
                    psi2 += 2 * pi
                    psi3 += 2 * pi
                    p_point = [[psi0, z1[0]], [psi0, z2[0]], [psi1, z1[1]], [psi1, z2[1]], [psi2, z1[2]], [psi2, z2[2]],
                               [psi3, z1[3]], [psi3, z2[3]]]

                else:
                    p_point = [[psi0, z1[0]], [psi0, z2[0]], [psi1, z1[1]], [psi1, z2[1]], [psi2, z1[2]], [psi2, z2[2]],
                               [psi3, z1[3]], [psi3, z2[3]]]

                points_sets = np.array(p_pool)

                polygons = [points_to_convex_polygon(points) for points in points_sets]

                target_polygon = points_to_convex_polygon(np.array(p_point))

                original_area = target_polygon.area

                overlapping_polygons = [p for p in polygons if p != target_polygon and p.intersects(target_polygon)]

                overlapping_area = sum(target_polygon.intersection(p).area for p in overlapping_polygons)

                non_overlapping_area = original_area - overlapping_area

                non_overlapping_ratio = non_overlapping_area / original_area

                # print(f"original_area: {original_area}")
                # print(f"overlapping_area: {overlapping_area}")
                # print(f"non_overlapping_ratio: {non_overlapping_ratio}")

                if non_overlapping_ratio >= 0.9:
                    visibility_token = 4
                elif non_overlapping_ratio >= 0.6:
                    visibility_token = 3
                elif non_overlapping_ratio >= 0.3:
                    visibility_token = 2
                else:
                    visibility_token = 1

                rot_1 = [cos(ego_yaw / 2), 0.0, 0.0, sin(ego_yaw / 2)]
                rot_2 = [cos((- float(data[14]) - pi / 2) / 2), 0.0, 0.0, sin((- float(data[14]) - pi / 2) / 2)]

                tra_1 = [ego_x, ego_y, ego_z]

                tra_2 = [float(data[13]), - float(data[11]), LIDAR_HEIGHT_POS - float(data[12]) + float(data[8]) / 2],

                xyzw_1 = [rot_1[1], rot_1[2], rot_1[3], rot_1[0]]

                xyzw_2 = [rot_2[1], rot_2[2], rot_2[3], rot_2[0]]



                # A相对B的位置和旋转四元数，使用具体数值替换这些占位符
                a_relative_position = np.array(tra_2)  # 用具体的数值替换x, y, z
                a_relative_rotation = xyzw_2  # 用具体的数值替换a, b, c, d

                # B相对世界坐标系的位置和旋转四元数，使用具体数值替换这些占位符
                b_world_position = np.array(tra_1)  # 用具体的数值替换x2, y2, z2
                b_world_rotation = xyzw_1  # 用具体的数值替换a2, b2, c2, d2

                # 创建四元数旋转对象
                a_relative_rotation_obj = Ro.from_quat(a_relative_rotation)
                b_world_rotation_obj = Ro.from_quat(b_world_rotation)

                # 使用B的旋转四元数旋转A的位置向量到世界坐标系中
                a_rotated_position = b_world_rotation_obj.apply(a_relative_position)

                # 将旋转后的位置向量加上B的位置向量，得到A在世界坐标系中的位置
                a_world_position = a_rotated_position.flatten() + b_world_position

                # 将A相对于B的旋转四元数与B相对于世界的旋转四元数进行四元数乘法，得到A相对于世界的旋转四元数
                a_world_rotation_obj = b_world_rotation_obj * a_relative_rotation_obj

                # 获取四元数的[x, y, z, w]格式
                a_world_rotation_quat = a_world_rotation_obj.as_quat()

                translation_1 = a_world_position.tolist()
                rotation_xyzw = a_world_rotation_quat.tolist()

                rotation1 = [rotation_xyzw[3], rotation_xyzw[0], rotation_xyzw[1], rotation_xyzw[2]]

                # print(a_rotated_position)
                # print(a_relative_position)

                if data[0] == 'pedestrian':
                    size1 = [1.15 * float(data[9]), 1.15 * float(data[10]), float(data[8])]
                elif data[0] == 'bicycle' or data[0] == 'motorcycle':
                    size1 = [float(data[9]), float(data[10]), float(data[8]) + 0.64]
                    translation_1[2] += 0.32
                else:
                    size1 = [float(data[9]), float(data[10]), float(data[8])]

                if instance_token not in instances:
                    instances[instance_token] = {}
                    instances[instance_token]['category_token'] = category_token
                    instances[instance_token]['annotation'] = []

                    instances[instance_token]['sample_token'] = []
                    instances[instance_token]['visibility_token'] = []
                    instances[instance_token]['attribute_tokens'] = []
                    instances[instance_token]['translation'] = []
                    instances[instance_token]['size'] = []
                    instances[instance_token]['rotation'] = []

                instances[instance_token]['annotation'].append(sample_annotation_token)
                instances[instance_token]['sample_token'].append(sample_token)
                instances[instance_token]['visibility_token'].append(visibility_token)
                instances[instance_token]['attribute_tokens'].append(attribute_tokens)
                instances[instance_token]['translation'].append(translation_1)
                instances[instance_token]['size'].append(size1)
                instances[instance_token]['rotation'].append(rotation1)

                sample_annotation_token = sample_annotation_token + 1

            if sample_token % 40 == 0:
                name = 'carla-' + str(scene_name)
                scene_json.append(
                    {'token': str(scene_token),
                     'log_token': 'logtokenlogtokenlogtokenlogtoken',
                     'nbr_samples': 40,
                     'first_sample_token': str(sample_token - 39),
                     'last_sample_token': str(sample_token),
                     'name': name,
                     'description': 'carla'}
                )
                scene_token = scene_token + 1
                scene_name = scene_name + 1

            # next sample
            sample_token = sample_token + 1

    # name = 'carla-' + str(scene_name)
    # scene_json.append(
    #     {'token': str(scene_token),
    #      'log_token': 'logtokenlogtokenlogtokenlogtoken',
    #      'nbr_samples': (sample_token - 1) % 40,
    #      'first_sample_token': str(sample_token - 1 - ((sample_token - 1) % 40) + 1),
    #      'last_sample_token': str(sample_token - 1),
    #      'name': name,
    #      'description': 'carla'}
    # )

    for instance_token in instances:
        instance_json.append(
            {'token': instance_token,
             'category_token': instances[instance_token]['category_token'],
             'nbr_annotations': len(instances[instance_token]['annotation']),
             'first_annotation_token': str(instances[instance_token]['annotation'][0]),
             'last_annotation_token': str(instances[instance_token]['annotation'][-1])}
        )

        ins = instances[instance_token]

        for to in range(len(instances[instance_token]['annotation'])):
            if len(instances[instance_token]['annotation']) == 1:
                prev_1 = ''
                next_1 = ''
            else:
                if to == 0:
                    prev_1 = ''
                else:
                    prev_1 = str(instances[instance_token]['annotation'][to - 1])

                if to == len(instances[instance_token]['annotation']) - 1:
                    next_1 = ''
                else:
                    next_1 = str(instances[instance_token]['annotation'][to + 1])

            sample_annotation_json.append({
                'token': str(ins['annotation'][to]),
                'sample_token': str(ins['sample_token'][to]),
                'instance_token': instance_token,
                'visibility_token': str(ins['visibility_token'][to]),
                'attribute_tokens': [
                    ins['attribute_tokens'][to]
                ],
                'translation': ins['translation'][to],
                'size': ins['size'][to],
                'rotation': ins['rotation'][to],
                'prev': prev_1,
                'next': next_1,
                'num_lidar_pts': 10,
                'num_radar_pts': 0
            })

    scene_jsondata = json.dumps(scene_json, indent=4, separators=(',', ':'))
    f = open(root_path + 'v1.0-trainval/scene.json', 'w')
    f.write(scene_jsondata)
    f.close()

    sample_jsondata = json.dumps(sample_json, indent=4, separators=(',', ':'))
    f = open(root_path + 'v1.0-trainval/sample.json', 'w')
    f.write(sample_jsondata)
    f.close()

    sample_annotation_jsondata = json.dumps(sample_annotation_json, indent=4, separators=(',', ':'))
    f = open(root_path + 'v1.0-trainval/sample_annotation.json', 'w')
    f.write(sample_annotation_jsondata)
    f.close()

    instance_jsondata = json.dumps(instance_json, indent=4, separators=(',', ':'))
    f = open(root_path + 'v1.0-trainval/instance.json', 'w')
    f.write(instance_jsondata)
    f.close()

    sample_data_jsondata = json.dumps(sample_data_json, indent=4, separators=(',', ':'))
    f = open(root_path + 'v1.0-trainval/sample_data.json', 'w')
    f.write(sample_data_jsondata)
    f.close()

    ego_pose_jsondata = json.dumps(ego_pose_json, indent=4, separators=(',', ':'))
    f = open(root_path + 'v1.0-trainval/ego_pose.json', 'w')
    f.write(ego_pose_jsondata)
    f.close()

    create_sensor_json(root_path, sensor_json, sensor_list)
    create_calibrated_sensor_json(root_path, calibrated_sensor_json, sensor_list, args)
    create_category_json(root_path, category_json)
    
    create_visibility_json(root_path, visibility_json)
    create_attribute_json(root_path, attribute_json)
    create_log_json(root_path, log_json)
    create_map_json(root_path, map_json)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    set_cpu_affinity('Efficient')

    description = (
        "July3D."
    )

    parser = argparse.ArgumentParser(
        description=description, formatter_class=RawTextHelpFormatter
    )

    parser.add_argument(
        "--hyperparams",
        required=True,
        type=str,
        help="Path to hyperparameter configuration file",
    )

    arguments = parser.parse_args()

    pat = "../carla_nus/dataset/nus/training/refine_2"
    pat_position = "../carla_nus/dataset/nus/training/position"
    pat_timestamp = "../carla_nus/dataset/nus/training/timestamp"
    root_path = "../carla_nus/dataset/nus/LIDAR_p0_samples/"
    os.makedirs(root_path + "v1.0-trainval", exist_ok=True)

    nuread(pat, pat_position, pat_timestamp, root_path, 500, arguments)

    print('Successfully create v1.0-trainval file!')


# See PyCharm help at https://www.jetbrains.com/help/pycharm/























