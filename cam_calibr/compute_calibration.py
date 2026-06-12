# coding=utf-8
"""
眼在手外手眼标定（离线版）
输入：
    - imgs/left 或 right 下的棋盘格图片
    - arm_pose/left.json 或 right.json 机械臂位姿
输出：
    - settings.yaml
"""

import os
import cv2
import json
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
import re
from pathlib import Path

np.set_printoptions(precision=8, suppress=True)


# ================== 核心函数 ==================
def func(images_path, json_path):

    # ===== 棋盘格参数 =====
    XX = 8
    YY = 5
    L = 0.0195

    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

    # ===== 世界坐标 =====
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)
    objp = L * objp

    obj_points = []
    img_points = []

    # ===== 自动读取图片 =====
    def extract_num(name):
        nums = re.findall(r'\d+', name)
        return int(nums[0]) if nums else -1

    image_files = sorted(
        [f for f in os.listdir(images_path) if f.endswith(".jpg")],
        key=extract_num
    )

    if len(image_files) == 0:
        raise ValueError(f"文件夹没有图片: {images_path}")

    for fname in image_files:
        image = os.path.join(images_path, fname)

        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        size = gray.shape[::-1]

        ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)

        if ret:
            obj_points.append(objp)

            corners2 = cv2.cornerSubPix(
                gray, corners, (5, 5), (-1, -1), criteria
            )

            img_points.append(corners2 if corners2 is not None else corners)

    N = len(img_points)

    if N == 0:
        raise ValueError("没有检测到有效棋盘格角点")

    print(f"有效图像数量: {N}")

    # ===== 相机标定 =====
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, size, None, None
    )

    print("内参矩阵:\n", mtx)
    print("畸变系数:\n", dist)
    print("-----------------------------------------------------")

    # ===== 读取 JSON =====
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"未找到 JSON: {json_path}")

    with open(json_path, 'r') as f:
        data = json.load(f)

    # ===== 读取 JSON =====
    R_tool = []
    t_tool = []

    pose_keys = sorted(data.keys(), key=lambda x: int(x.replace("pose", "")))

    for key in pose_keys:
        if len(R_tool) >= N:
            break

        mat = np.array(data[key]["mat"])

        if mat.shape != (4, 4):
            raise ValueError(f"{key} 不是 4x4 矩阵")

        # ================== ⭐ 核心修改开始 ⭐ ==================

        # 原始：base → tool
        R_base2tool = mat[:3, :3]
        t_base2tool = mat[:3, 3]

        # 👉 转换成：tool → base（眼在手外必须）
        R_tool2base = R_base2tool.T
        t_tool2base = -R_base2tool.T @ t_base2tool

        R_tool.append(R_tool2base)
        t_tool.append(t_tool2base)


    if len(R_tool) != N:
        raise ValueError("位姿数量与图像数量不一致")

    # ===== 手眼标定 =====
    R_cam_to_base, t_cam_to_base = cv2.calibrateHandEye(
        R_tool,
        t_tool,
        rvecs,
        tvecs,
        cv2.CALIB_HAND_EYE_TSAI
    )

    print("相机到零位旋转:\n", R_cam_to_base)
    print("相机到零位平移:\n", t_cam_to_base)

    return R_cam_to_base, t_cam_to_base


# ================== compute ==================
def compute(images_path, json_path):
    R_mat, t_vec = func(images_path, json_path)

    rotation = R.from_matrix(R_mat)
    qx, qy, qz, qw = rotation.as_quat()

    t = t_vec.reshape(-1)
    x, y, z = t

    print("\n====== 相机位姿（相对于零位） ======")
    print(f"qw: {qw}")
    print(f"qx: {qx}")
    print(f"qy: {qy}")
    print(f"qz: {qz}")
    print(f"x: {x}")
    print(f"y: {y}")
    print(f"z: {z}")

    return R_mat, t_vec


# ================== 主流程 ==================
def run_calibration(root_path, cam_paths, pose_paths):

    results = {}

    for side in cam_paths.keys():
        print(f"\n========== 处理 {side} ==========")

        img_path = cam_paths[side]
        json_path = pose_paths[side]

        # 👉 调用手眼标定
        R_cam2base, t_cam2base = compute(img_path, json_path)

        results[f"eye_hand_rotation_matrix_{side}"] = R_cam2base.tolist()
        results[f"eye_hand_translation_vector_{side}"] = t_cam2base.reshape(3).tolist()

    # ===== 保存 YAML =====
    save_path = os.path.join(root_path, "settings.yaml")

    with open(save_path, "w") as f:
        yaml.dump(results, f, default_flow_style=False)

    print(f"\n标定完成，保存到: {save_path}")


# ================== 入口 ==================
if __name__ == "__main__":
        # ===== 根目录 =====
    root_path = Path(__file__).resolve().parent

    cam_paths = {
        'left': os.path.join(root_path, 'imgs/left'),
        'right': os.path.join(root_path, 'imgs/right')
    }

    pose_paths = {
        'left': os.path.join(root_path, 'arm_pose/left.json'),
        'right': os.path.join(root_path, 'arm_pose/right.json')
    }
    run_calibration(root_path, cam_paths, pose_paths)
