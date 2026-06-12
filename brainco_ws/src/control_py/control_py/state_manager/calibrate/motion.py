import time
import struct
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml
import copy
import json
import requests


from loguru import logger
from control_py.utils.loguru_settings import setup_loguru
setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG')

config_file = 'src/control_py/control_py/state_manager/calibrate/calibrate_config.yaml'
with open(config_file, 'r') as f:
    calibrate_cfg = yaml.safe_load(f) or {}

CAM_CALIBR_DIR = Path(calibrate_cfg["paths"]["cam_calibr_dir"])

with open(CAM_CALIBR_DIR / "settings.yaml", 'r') as f:
    settings_config = yaml.load(f, Loader=yaml.FullLoader)

eye_hand_rotation_matrix_left = settings_config['eye_hand_rotation_matrix_left']
eye_hand_translation_vector_left = settings_config['eye_hand_translation_vector_left']
eye_hand_rotation_matrix_right = settings_config['eye_hand_rotation_matrix_right']
eye_hand_translation_vector_right = settings_config['eye_hand_translation_vector_right']


# ==========================================================
# 眼在手外，相机坐标系物体到机械臂基坐标系转换函数
# ==========================================================
def convert_eye2hand(eye_coordinate, hand_side):
    x, y, z = eye_coordinate[0], eye_coordinate[1], eye_coordinate[2]

    if hand_side == "left":
        # 头部摄像头坐标系到左机械臂基座坐标系的旋转矩阵和平移向量
        rotation_matrix = np.array(eye_hand_rotation_matrix_left)
        translation_vector = np.array(eye_hand_translation_vector_left)
    elif hand_side == "right":
        # 头部摄像头坐标系到右机械臂基座坐标系的旋转矩阵和平移向量
        rotation_matrix = np.array(eye_hand_rotation_matrix_right)
        translation_vector = np.array(eye_hand_translation_vector_right)
    else:
        logger.error("Wrong Hand Side!")

    # 物体在环境摄像头坐标系下的坐标
    object_cam_coords = np.array([x, y, z, 1])

    # 构建齐次变换矩阵（旋转和平移）
    cam_to_base_matrix = np.eye(4)
    cam_to_base_matrix[:3, :3] = rotation_matrix
    cam_to_base_matrix[:3, 3] = translation_vector

    # 物体在基座坐标系下的坐标
    object_base_coords = np.dot(cam_to_base_matrix, object_cam_coords)

    # 提取位姿信息
    obj_base_position = object_base_coords[:3]
    obj_base_rotation_matrix = rotation_matrix
    obj_base_euler = R.from_matrix(obj_base_rotation_matrix).as_euler('xyz')

    obj_base_pose = np.hstack((obj_base_position, obj_base_euler))

    return obj_base_position.tolist()


def calculate_obj_coordinate_hand(obj_coordinate_camera_avg, hand_side):
    obj_coordinate_hand_avg = copy.deepcopy(obj_coordinate_camera_avg)
    for obj_name, obj_coordinate_camera in obj_coordinate_camera_avg.items():
        obj_coordinate_hand = convert_eye2hand(obj_coordinate_camera_avg[obj_name], hand_side)
        obj_coordinate_hand_avg[obj_name] = obj_coordinate_hand

    return obj_coordinate_hand_avg


def build_pose(pos, offset):
    """
    pos:    [x, y, z] 或 [x, y, z, rx, ry, rz]
    offset: [dx, dy, dz, rx, ry, rz]

    return:
        - pos为3位: [x+dx, y+dy, z+dz, rx, ry, rz]
        - pos为6位: [x+dx, y+dy, z+dz, r+dr, p+dp, y+dy]
    """
    if len(pos) == 3:
        pos_new = [p + o for p, o in zip(pos, offset[:3])]
        return pos_new + offset[3:]
    
    elif len(pos) == 6:
        return [p + o for p, o in zip(pos, offset)]
    
    else:
        raise ValueError("pos must be length 3 or 6")
