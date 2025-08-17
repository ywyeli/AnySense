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




# Function to set CPU affinity
def set_cpu_affinity(mode):
    cpu_cores = []
    total_cpus = 16
    used_cpus = None
    start = int(time.time()) % total_cpus

    if mode == 'Efficient':
        used_cpus = 2
    elif mode == 'Performance':
        used_cpus = 4
    else:
        print(f'Error {mode}!')

    for _ in range(used_cpus):
        cpu_cores.append(start)
        start += 2
        if start >= total_cpus:
            start -= total_cpus

    p = psutil.Process(os.getpid())
    p.cpu_affinity(cpu_cores)

    print(f"Process is set to use CPU cores: {cpu_cores}")


def get_FOV_from(s, args):
    params = toml.load(f"../carla_nus/hyperparams/{args.hyperparams}")
    param = params['camera']

    fov_1 = param[f'FOV_{s}']

    fov_horizontal = float(fov_1)

    image_width = 1600  # Image width in pixels
    image_height = 900  # Image height in pixels

    # Convert horizontal FOV from degrees to radians
    fov_horizontal_rad = math.radians(fov_horizontal)

    # Calculate focal lengths in terms of pixels
    f_x = image_width / (2 * math.tan(fov_horizontal_rad / 2))

    # Assuming square pixels and that vertical FOV is determined by aspect ratio
    aspect_ratio = image_width / image_height
    fov_vertical_rad = 2 * math.atan(math.tan(fov_horizontal_rad / 2) / aspect_ratio)
    f_y = image_height / (2 * math.tan(fov_vertical_rad / 2))

    # Principal point (usually at the center of the image)
    c_x = image_width / 2
    c_y = image_height / 2

    # Construct the intrinsic matrix
    intrinsic_matrix = [
        [f_x, 0, c_x],
        [0, f_y, c_y],
        [0, 0, 1]
    ]

    return intrinsic_matrix


def get_T_from(s, args):
    params = toml.load(f"../carla_nus/hyperparams/{args.hyperparams}")
    param = params['camera']

    list1 = param[f'T_{s}']

    list2 = [1, -1, 1]

    array1 = np.array(list1)
    array2 = np.array(list2)

    right = array1 * array2

    return list(right)


def get_R_from(s, args):
    params = toml.load(f"../carla_nus/hyperparams/{args.hyperparams}")
    param = params['camera']

    left = param[f'R_{s}']

    yw = - left[2]

    quaternion_yaw = get_q(yw)
    quaternion = [0.5, -0.5, 0.5, -0.5]

    new_quaternion = quaternion_multiply(quaternion_yaw, quaternion)

    right = list(new_quaternion)

    return right


def get_q(angle_degrees):
    angle_radians = np.radians(angle_degrees)
    cos_theta = np.cos(angle_radians)
    sin_theta = np.sin(angle_radians)

    rotation_matrix = np.array([
        [cos_theta, -sin_theta, 0],
        [sin_theta, cos_theta, 0],
        [0, 0, 1]
    ])

    quaternion_yaw = np.array([
        np.cos(angle_radians / 2),
        0,
        0,
        np.sin(angle_radians / 2)
    ])

    return quaternion_yaw


def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


def points_to_convex_polygon(points):
    hull = ConvexHull(points)
    hull_points = points[hull.vertices]
    return Polygon(hull_points)


# open label folder
def openreadtxt(file_name):
    data = []
    file = open(file_name, 'r')
    file_data = file.readlines()
    for row in file_data:
        tmp_list = row.split(' ')
        data.append(tmp_list)
    return data


def create_sensor_json(sensor_json, sensor_list):
    create_sensor_token = None
    for s in sensor_list:
        if s == 'LIDAR_TOP':
            modality = 'lidar'
            create_sensor_token = 'lidartoplidartoplidartoplidartop'
        else:
            modality = 'camera'
            if s == 'CAM_FRONT':
                create_sensor_token = 'camfrontcamfrontcamfrontcamfront'
            if s == 'CAM_BACK':
                create_sensor_token = 'cambackcambackcambackcambackcamb'
            if s == 'CAM_FRONT_LEFT':
                create_sensor_token = 'camfrontleftcamfrontleftcamfront'
            if s == 'CAM_FRONT_RIGHT':
                create_sensor_token = 'camfrontrightcamfrontrightcamfro'
            if s == 'CAM_BACK_LEFT':
                create_sensor_token = 'cambackleftcambackleftcambacklef'
            if s == 'CAM_BACK_RIGHT':
                create_sensor_token = 'cambackrightcambackrightcambackr'
        sensor_json.append(
            {'token': create_sensor_token,
             'channel': s,
             'modality': modality}
        )
    sensor_jsondata = json.dumps(sensor_json, indent=4, separators=(',', ':'))
    f = open(root_path + 'v1.0-trainval/sensor.json', 'w')
    f.write(sensor_jsondata)
    f.close()


def create_visibility_json(visibility_json):
    visibility_json.append(
        {'description': 'visibility of whole object is between 80 and 100%',
         'token': '4',
         'level': 'v80-100'}
    )
    visibility_json.append(
                {'description': 'visibility of whole object is between 60 and 80%',
             'token': '3',
             'level': 'v60-80'}
    )
    visibility_json.append(
                {'description': 'visibility of whole object is between 40 and 60%',
             'token': '2',
             'level': 'v40-60'}
    )
    visibility_json.append(
                {'description': 'visibility of whole object is between 0 and 40%',
             'token': '1',
             'level': 'v0-40'}
    )
    visibility_jsondata = json.dumps(visibility_json, indent=4, separators=(',', ':'))
    f = open(root_path + 'v1.0-trainval/visibility.json', 'w')
    f.write(visibility_jsondata)
    f.close()


def create_category_json(category_json):
    create_category_token = 'categorylidardetcategory00000car'
    category_json.append(
        {'token': create_category_token,
         'name': 'vehicle.car',
         'description': 'vehicle_car'}
    )
    create_category_token = 'categorylidardetcategory00000bus'
    category_json.append(
        {'token': create_category_token,
         'name': 'vehicle.bus.rigid',
         'description': 'vehicle_bus'}
    )
    create_category_token = 'categorylidardetcategory000truck'
    category_json.append(
        {'token': create_category_token,
         'name': 'vehicle.truck',
         'description': 'vehicle_truck'}
    )
    create_category_token = 'categorylidardetcategory0bicycle'
    category_json.append(
        {'token': create_category_token,
         'name': 'vehicle.bicycle',
         'description': 'vehicle_bicycle'}
    )
    create_category_token = 'categorylidardetcategory000motor'
    category_json.append(
        {'token': create_category_token,
         'name': 'vehicle.motorcycle',
         'description': 'vehicle_motorcycle'}
    )
    create_category_token = 'categorylidardetcategory000adult'
    category_json.append(
        {'token': create_category_token,
         'name': 'human.pedestrian.adult',
         'description': 'human_pedestrian_adult'}
    )
    category_jsondata = json.dumps(category_json, indent=4, separators=(',', ':'))
    f = open(root_path + 'v1.0-trainval/category.json', 'w')
    f.write(category_jsondata)
    f.close()


def create_attribute_json(attribute_json):
    create_attribute_token = 'vehiclemovingvehiclemovingvehicl'
    attribute_json.append(
        {'token': create_attribute_token,
         'name': 'vehicle.moving',
         'description': 'Vehicle is moving.'}
    )
    create_attribute_token = 'vehicle_stopped_vehicle_stopped_'
    attribute_json.append(
        {'token': create_attribute_token,
         'name': 'vehicle.stopped',
         'description': 'Vehicle, with a driver/rider in/on it, is currently stationary but has an intent to move.'}
    )

    create_attribute_token = 'cyclewithridercyclewithridercycl'
    attribute_json.append(
        {'token': create_attribute_token,
         'name': 'cycle.with_rider',
         'description': 'There is a rider on the bicycle or motorcycle.'}
    )

    create_attribute_token = 'bicycle_stopped_bicycle_stopped_'
    attribute_json.append(
        {'token': create_attribute_token,
         'name': 'cycle.without_rider',
         'description': 'Bicycle is stopped.'}
    )

    create_attribute_token = 'pedestrianmovingpedestrianmoving'
    attribute_json.append(
        {'token': create_attribute_token,
         'name': 'pedestrian.moving',
         'description': 'The human is moving.'}
    )

    create_attribute_token = 'standingstandingstandingstanding'
    attribute_json.append(
        {'token': create_attribute_token,
         'name': 'pedestrian.standing',
         'description': 'The human is standing.'}
    )

    attribute_jsondata = json.dumps(attribute_json, indent=4, separators=(',', ':'))
    f = open(root_path + 'v1.0-trainval/attribute.json', 'w')
    f.write(attribute_jsondata)
    f.close()


def create_log_json(log_json):
    log_json.append(
        {'token': 'logtokenlogtokenlogtokenlogtoken',
         'logfile': 'n008-2018-08-01-00-00-00-0400',
         'vehicle': 'n008',
         'date_captured': '2018-08-01',
         'location': 'singapore-onenorth'}
    )
    log_jsondata = json.dumps(log_json, indent=4, separators=(',', ':'))
    f = open(root_path + 'v1.0-trainval/log.json', 'w')
    f.write(log_jsondata)
    f.close()


def create_map_json(map_json):
    map_json.append({
        "category": "semantic_prior",
        "token": "53992ee3023e5494b90c316c183be829",
        "filename": "maps/53992ee3023e5494b90c316c183be829.png",
        "log_tokens": [
        "logtokenlogtokenlogtokenlogtoken"
        ]
    })
    map_jsondata = json.dumps(map_json, indent=4, separators=(',', ':'))
    f = open(root_path + 'v1.0-trainval/map.json', 'w')
    f.write(map_jsondata)
    f.close()


def create_calibrated_sensor_json(calibrated_sensor_json, sensor_list, args):

    params = toml.load(f"../carla_nus/hyperparams/{args.hyperparams}")

    lids = params['lidar']
    LIDAR_HEIGHT_POS = lids['GLOBAL_HEIGHT_POS']

    create_sensor_token = None
    create_calibrated_sensor_token = None
    for s in sensor_list:
        if s == 'LIDAR_TOP':
            create_sensor_token = 'lidartoplidartoplidartoplidartop'
            create_calibrated_sensor_token = '90000000000000000000000000000000'
            translation = [0, 0, LIDAR_HEIGHT_POS]
            rotation = [
                0.707, 0, 0, -0.707
            ]
            camera_intrinsic = []

        if s == 'CAM_FRONT':
            create_sensor_token = 'camfrontcamfrontcamfrontcamfront'
            create_calibrated_sensor_token = '90000000000000000000000000000001'
            translation = get_T_from(s, args)
            rotation = get_R_from(s, args)
            camera_intrinsic = get_FOV_from(s, args)

        if s == 'CAM_BACK':
            create_sensor_token = 'cambackcambackcambackcambackcamb'
            create_calibrated_sensor_token = '90000000000000000000000000000002'
            translation = get_T_from(s, args)
            rotation = get_R_from(s, args)
            camera_intrinsic = get_FOV_from(s, args)

        if s == 'CAM_FRONT_LEFT':
            create_sensor_token = 'camfrontleftcamfrontleftcamfront'
            create_calibrated_sensor_token = '90000000000000000000000000000003'
            translation = get_T_from(s, args)
            rotation = get_R_from(s, args)
            camera_intrinsic = get_FOV_from(s, args)

        if s == 'CAM_FRONT_RIGHT':
            create_sensor_token = 'camfrontrightcamfrontrightcamfro'
            create_calibrated_sensor_token = '90000000000000000000000000000004'
            translation = get_T_from(s, args)
            rotation = get_R_from(s, args)
            camera_intrinsic = get_FOV_from(s, args)

        if s == 'CAM_BACK_LEFT':
            create_sensor_token = 'cambackleftcambackleftcambacklef'
            create_calibrated_sensor_token = '90000000000000000000000000000005'
            translation = get_T_from(s, args)
            rotation = get_R_from(s, args)
            camera_intrinsic = get_FOV_from(s, args)

        if s == 'CAM_BACK_RIGHT':
            create_sensor_token = 'cambackrightcambackrightcambackr'
            create_calibrated_sensor_token = '90000000000000000000000000000006'
            translation = get_T_from(s, args)
            rotation = get_R_from(s, args)
            camera_intrinsic = get_FOV_from(s, args)

        calibrated_sensor_json.append(
            {'token': create_calibrated_sensor_token,
             'sensor_token': create_sensor_token,
             'translation': translation,
             'rotation': rotation,
             'camera_intrinsic': camera_intrinsic}
        )
    calibrated_sensor_jsondata = json.dumps(calibrated_sensor_json, indent=4, separators=(',', ':'))
    f = open(root_path + 'v1.0-trainval/calibrated_sensor.json', 'w')
    f.write(calibrated_sensor_jsondata)
    f.close()





