import numpy as np
import pinocchio as pin
from typing import Dict, Optional, Tuple, List, Union
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
import json, os

from loguru import logger
from control_py.utils.loguru_settings import setup_loguru
setup_loguru(log_folder_path="log", show_on_terminal=True, terminal_level='DEBUG')

rot_mat_48 = np.array([
    [[ 1, 0, 0], [ 0, 1, 0], [ 0, 0, 1]], # 0
    [[ 1, 0, 0], [ 0, 1, 0], [ 0, 0,-1]], # 1
    [[ 1, 0, 0], [ 0,-1, 0], [ 0, 0, 1]], # 2
    [[ 1, 0, 0], [ 0,-1, 0], [ 0, 0,-1]], # 3
    [[-1, 0, 0], [ 0, 1, 0], [ 0, 0, 1]], # 4
    [[-1, 0, 0], [ 0, 1, 0], [ 0, 0,-1]], # 5
    [[-1, 0, 0], [ 0,-1, 0], [ 0, 0, 1]], # 6
    [[-1, 0, 0], [ 0,-1, 0], [ 0, 0,-1]], # 7

    [[ 1, 0, 0], [ 0, 0, 1], [ 0, 1, 0]], # 8
    [[ 1, 0, 0], [ 0, 0, 1], [ 0,-1, 0]], # 9
    [[ 1, 0, 0], [ 0, 0,-1], [ 0, 1, 0]], # 10
    [[ 1, 0, 0], [ 0, 0,-1], [ 0,-1, 0]], # 11
    [[-1, 0, 0], [ 0, 0, 1], [ 0, 1, 0]], # 12
    [[-1, 0, 0], [ 0, 0, 1], [ 0,-1, 0]], # 13
    [[-1, 0, 0], [ 0, 0,-1], [ 0, 1, 0]], # 14
    [[-1, 0, 0], [ 0, 0,-1], [ 0,-1, 0]], # 15

    [[ 0, 1, 0], [ 1, 0, 0], [ 0, 0, 1]], # 16
    [[ 0, 1, 0], [ 1, 0, 0], [ 0, 0,-1]], # 17
    [[ 0, 1, 0], [ -1,0, 0], [ 0, 0, 1]], # 18
    [[ 0, 1, 0], [ -1,0, 0], [ 0, 0,-1]], # 19
    [[ 0,-1, 0], [ 1, 0, 0], [ 0, 0, 1]], # 20
    [[ 0,-1, 0], [ 1, 0, 0], [ 0, 0,-1]], # 21
    [[ 0,-1, 0], [ -1,0, 0], [ 0, 0, 1]], # 22
    [[ 0,-1, 0], [ -1,0, 0], [ 0, 0,-1]], # 23

    [[ 0, 1, 0], [ 0, 0, 1], [ 1, 0, 0]], # 24
    [[ 0, 1, 0], [ 0, 0, 1], [-1, 0, 0]], # 25
    [[ 0, 1, 0], [ 0, 0,-1], [ 1, 0, 0]], # 26
    [[ 0, 1, 0], [ 0, 0,-1], [-1, 0, 0]], # 27
    [[ 0,-1, 0], [ 0, 0, 1], [ 1, 0, 0]], # 28
    [[ 0,-1, 0], [ 0, 0, 1], [-1, 0, 0]], # 29
    [[ 0,-1, 0], [ 0, 0,-1], [ 1, 0, 0]], # 30
    [[ 0,-1, 0], [ 0, 0,-1], [-1, 0, 0]], # 31

    [[ 0, 0, 1], [ 1, 0, 0], [ 0, 1, 0]], # 32
    [[ 0, 0, 1], [ 1, 0, 0], [ 0,-1, 0]], # 33
    [[ 0, 0, 1], [ -1,0, 0], [ 0, 1, 0]], # 34
    [[ 0, 0, 1], [ -1,0, 0], [ 0,-1, 0]], # 35
    [[ 0, 0,-1], [ 1, 0, 0], [ 0, 1, 0]], # 36
    [[ 0, 0,-1], [ 1, 0, 0], [ 0,-1, 0]], # 37
    [[ 0, 0,-1], [ -1,0, 0], [ 0, 1, 0]], # 38
    [[ 0, 0,-1], [ -1,0, 0], [ 0,-1, 0]], # 39

    [[ 0, 0, 1], [ 0, 1, 0], [ 1, 0, 0]], # 40
    [[ 0, 0, 1], [ 0, 1, 0], [-1, 0, 0]], # 41
    [[ 0, 0, 1], [ 0,-1, 0], [ 1, 0, 0]], # 42
    [[ 0, 0, 1], [ 0,-1, 0], [-1, 0, 0]], # 43
    [[ 0, 0,-1], [ 0, 1, 0], [ 1, 0, 0]], # 44
    [[ 0, 0,-1], [ 0, 1, 0], [-1, 0, 0]], # 45
    [[ 0, 0,-1], [ 0,-1, 0], [ 1, 0, 0]], # 46
    [[ 0, 0,-1], [ 0,-1, 0], [-1, 0, 0]]]) # 47


def quat_to_xyzw(quat, order):
    if order == "wxyz":
        return np.array([quat[1], quat[2], quat[3], quat[0]])
    elif order == "xyzw":
        return np.array(quat)
    else:
        raise ValueError("quat_order must be 'wxyz' or 'xyzw'")


def quat_from_xyzw(quat, order):
    if order == "wxyz":
        return np.array([quat[3], quat[0], quat[1], quat[2]])
    elif order == "xyzw":
        return np.array(quat)
    else:
        raise ValueError("quat_order must be 'wxyz' or 'xyzw'")


class PoseData:
    def __init__(self):
        self.pos = None
        self.quat = None   # [w, x, y, z]
        self.euler = None  # [roll, pitch, yaw]
        self.mat = None    # 4x4 SE3 matrix

    def set_pose_quat(self, pos, quat, quat_order='wxyz', euler_order='xyz'):
        self.pos = np.array(pos).copy()
        self.quat = np.array(quat).copy()

        quat_xyzw = quat_to_xyzw(self.quat, quat_order)
        rot = R.from_quat(quat_xyzw)

        self.euler = rot.as_euler(euler_order)

        self.mat = np.eye(4)
        self.mat[:3, :3] = rot.as_matrix()
        self.mat[:3, 3] = self.pos

    def set_pose_euler(self, pos, euler, quat_order='wxyz', euler_order='xyz'):
        self.pos = np.array(pos).copy()
        self.euler = np.array(euler).copy()

        rot = R.from_euler(euler_order, self.euler)

        quat_xyzw = rot.as_quat()
        self.quat = quat_from_xyzw(quat_xyzw, quat_order)

        self.mat = np.eye(4)
        self.mat[:3, :3] = rot.as_matrix()
        self.mat[:3, 3] = self.pos

    def set_pose_mat(self, mat, quat_order='wxyz', euler_order='xyz'):
        self.mat = np.array(mat).reshape(4,4)

        self.pos = self.mat[:3, 3].copy()

        rot = R.from_matrix(self.mat[:3, :3])

        quat_xyzw = rot.as_quat()
        self.quat = quat_from_xyzw(quat_xyzw, quat_order)

        self.euler = rot.as_euler(euler_order)

    def __str__(self):
        return (
            f"PoseData("
            f"pos={np.round(self.pos, 3)}, "
            f"quat={np.round(self.quat, 3)}(wxyz), "
            f"euler={np.round(self.euler, 3)}(order:xyz), "
            f"mat={np.round(self.mat, 3).tolist()}"
            f")"
        )
    
    def __repr__(self):
        return (
            f"PoseData("
            f"pos={np.round(self.pos, 3)}, "
            f"quat={np.round(self.quat, 3)}(wxyz), "
            f"euler={np.round(self.euler, 3)}(order:xyz), "
            f"mat={np.round(self.mat, 3).tolist()}"
            f")"
        )
    
    def __iter__( self ):
        return (
            f"PoseData("
            f"pos={np.round(self.pos, 3)}, "
            f"quat={np.round(self.quat, 3)}(wxyz), "
            f"euler={np.round(self.euler, 3)}(order:xyz), "
            f"mat={np.round(self.mat, 3).tolist()}"
            f")"
        )
    
    def save_json(self, save_id, json_path):

        data = {
            "pos": self.pos.tolist() if self.pos is not None else None,
            "quat": self.quat.tolist() if self.quat is not None else None,
            "euler": self.euler.tolist() if self.euler is not None else None,
            "mat": self.mat.tolist() if self.mat is not None else None,
        }

        # 创建目录
        dir_path = os.path.dirname(json_path)
        if dir_path != "":
            os.makedirs(dir_path, exist_ok=True)

        # 读取已有数据
        if os.path.exists(json_path):
            with open(json_path, "r") as f:
                all_data = json.load(f)
        else:
            all_data = {}

        all_data[save_id] = data

        # 写入
        with open(json_path, "w") as f:
            json.dump(all_data, f, indent=4)


    def load_json(self, read_id, json_path="poses.json"):
        with open(json_path, "r") as f:
            all_data = json.load(f)

        data = all_data[read_id]

        self.pos = np.array(data["pos"]) if data["pos"] else None
        self.quat = np.array(data["quat"]) if data["quat"] else None
        self.euler = np.array(data["euler"]) if data["euler"] else None
        self.mat = np.array(data["mat"]) if data["mat"] else None
    

class ArmPoseData:
    def __init__(self):
        self.left = PoseData()
        self.right = PoseData()


def get_rot_mat(index: int) -> List:
    """
    根据索引获取对应的旋转矩阵
    """
    return rot_mat_48[index]


def mat_to_pos_quat(tf_mat: np.ndarray, quat_order='wxyz') -> np.ndarray:
    pos = tf_mat[:3, 3]
    rot = tf_mat[:3, :3]
    quat = mat_to_quat(rot, quat_order=quat_order)
    return np.concatenate((pos, quat), axis=0)


def normalize_quat(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return quat / norm


def quat_to_mat(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize_quat(quat)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


# def mat_to_quat(rot: np.ndarray) -> np.ndarray:
#     quat = pin.Quaternion(rot)
#     coeffs = quat.coeffs()  # [x, y, z, w]
#     return normalize_quat(np.array([coeffs[3], coeffs[0], coeffs[1], coeffs[2]], dtype=np.float64))

def mat_to_quat(rot: np.ndarray, quat_order='wxyz') -> np.ndarray:
    quat = pin.Quaternion(rot)
    coeffs = quat.coeffs()  # [x, y, z, w]
    if quat_order == 'wxyz':
        return normalize_quat(np.array([coeffs[3], coeffs[0], coeffs[1], coeffs[2]], dtype=np.float64))
    elif quat_order == 'xyzw':
        return normalize_quat(coeffs)

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return normalize_quat(
        np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ], dtype=np.float64)
    )


def extract_mat(pose_mat) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """
    从给定的齐次位姿矩阵中提取位置和旋转矩阵，并进行正交化处理。
    """
    try:
        rot_tracker = np.asarray(pose_mat[:3, :3], dtype=np.float64)
        pos_tracker = np.asarray(pose_mat[:3, 3], dtype=np.float64)

        # Orthonormalize to remove accumulated drift/noise
        u, _, vh = np.linalg.svd(rot_tracker)
        rot_tracker = u @ vh

        return pos_tracker, rot_tracker
    
    except Exception as exc:
        print(f"[TrackerTeleoperate] Failed to read tracker pose: {exc}")
        return None, None
    




def mat_to_pose_msg(T: np.ndarray) -> Pose:
    pose_msg = Pose()
    pose_msg.position.x = T[0, 3]
    pose_msg.position.y = T[1, 3]
    pose_msg.position.z = T[2, 3]

    quat = mat_to_quat(T)  # 返回 [x, y, z, w]
    pose_msg.orientation.x = quat[0]
    pose_msg.orientation.y = quat[1]
    pose_msg.orientation.z = quat[2]
    pose_msg.orientation.w = quat[3]

    return pose_msg


def flatten_matrix(mat: np.ndarray) -> List[float]:
    """
    将二维矩阵压缩为一维列表
    """
    if not isinstance(mat, np.ndarray):
        raise TypeError("Input must be a numpy.ndarray")
    if mat.ndim != 2:
        raise ValueError("Input must be a 2D matrix")
    
    return mat.flatten().tolist()


def mat_to_jointstate_msg(mat: np.ndarray, ros2_timestamp) -> JointState:
    mat_flat = flatten_matrix(mat)
    msg = JointState()
    msg.header.stamp = ros2_timestamp
    msg.name = ["mat"]
    msg.position = mat_flat

    return msg


def jointstate_msg_to_pose_mat(msg: JointState) -> np.ndarray:
    """
    将JointState消息中的一维列表恢复成原来的3x4矩阵
    """
    mat_flat = np.array(msg.position, dtype=np.float64)
    if mat_flat.size != 16:
        raise ValueError(f"Expected 16 elements to reshape into 4x4, got {mat_flat.size}")
    mat = mat_flat.reshape(4, 4)
    return mat



def switch_quat_order(quat: Union[List[float], np.ndarray], to_order: str = "wxyz") -> np.ndarray:
    """
    切换四元数的顺序
    """
    if len(quat) != 4:
        raise ValueError("Quaternion must have 4 elements")

    quat = np.array(quat, dtype=np.float64)
    
    if to_order == "wxyz":
        # xyzw -> wxyz
        return np.array([quat[3], quat[0], quat[1], quat[2]], dtype=np.float64)
    elif to_order == "xyzw":
        # wxyz -> xyzw
        return np.array([quat[1], quat[2], quat[3], quat[0]], dtype=np.float64)
    else:
        raise ValueError(f"Unsupported conversion to {to_order}")


def transform_matrix(matrix, trans="", rot=""):
    """
    对 4x4 矩阵进行平移和旋转变换。
    
    参数:
    - matrix: 4x4 numpy数组
    - trans: 平移轴字符串，如 "x", "xy", "xyz"
             如果包含 'x'，x轴平移取负数
    - rot: 旋转轴字符串，如 "x", "yz", "xyz"
    
    返回:
    - 变换后的 4x4 矩阵
    """
    if matrix.shape != (4, 4):
        raise ValueError("输入矩阵必须是 4x4")
    
    # 平移矩阵
    t = np.eye(4)
    for axis in trans:
        if axis == 'x':
            t[0, 3] = -1  # x轴平移取负
        elif axis == 'y':
            t[1, 3] = 1
        elif axis == 'z':
            t[2, 3] = 1
    
    # 旋转矩阵（绕世界坐标轴旋转 90 度示例）
    r = np.eye(4)
    angle = np.pi  # 180度旋转
    
    for axis in rot:
        if axis == 'x':
            rx = np.array([[1, 0, 0, 0],
                           [0, np.cos(angle), -np.sin(angle), 0],
                           [0, np.sin(angle), np.cos(angle), 0],
                           [0, 0, 0, 1]])
            r = r @ rx
        elif axis == 'y':
            ry = np.array([[np.cos(angle), 0, np.sin(angle), 0],
                           [0, 1, 0, 0],
                           [-np.sin(angle), 0, np.cos(angle), 0],
                           [0, 0, 0, 1]])
            r = r @ ry
        elif axis == 'z':
            rz = np.array([[np.cos(angle), -np.sin(angle), 0, 0],
                           [np.sin(angle), np.cos(angle), 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
            r = r @ rz
    
    # 先旋转后平移
    transformed_matrix = t @ r @ matrix
    return transformed_matrix


# def process_pose(
#         pos_current, 
#         rot_current,
#         calib_pos: Optional[np.ndarray],
#         calib_rot: Optional[np.ndarray],
#         # coord_vr_to_robot: Optional[np.ndarray],
#         robot_init_pos: np.ndarray,
#         robot_init_rot: np.ndarray,
#         R_axes: np.ndarray,
#         position_scale: float = 1.0,
#         coord: str = 'vr'
#     ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
#     """
#     基于 calib_pos 和 coord_vr_to_robot 计算当前位姿
#     R_axes: 平移轴和旋转轴变换矩阵R
#     """

#     if coord == 'tracker':

#         # 基于tracker坐标系
#         # 平移
#         pos_delta = pos_current - calib_pos
#         # pos_delta_robot = coord_vr_to_robot @ pos_delta
#         pos_delta_robot = robot_init_rot @ calib_rot.T @ pos_delta
#         pos_delta_robot_axes = R_axes[2] @ pos_delta_robot  # 坐标轴映射
#         pos_abs = robot_init_pos + position_scale * pos_delta_robot_axes

#         # 旋转
#         rot_current_axes = robot_init_rot @ calib_rot.T @ rot_current
#         rot_abs = R_axes[3] @ rot_current_axes @ R_axes[3].T  # 坐标轴映射

#     elif coord == 'world':
#         # 2. 基于世界坐标系平移
#         pos_delta = pos_current - calib_pos
#         pos_delta_axes = R_axes[0] @ pos_delta  # 坐标轴映射
#         pos_delta_robot = robot_init_rot @ pos_delta_axes
#         pos_abs = robot_init_pos + position_scale * pos_delta_robot

#         # 2. 基于世界坐标系旋转
#         rot_delta = rot_current @ calib_rot.T          # 相对标定旋转增量
#         rot_delta_robot = R_axes[1] @ rot_delta @ R_axes[1].T  # 坐标轴映射
#         rot_abs = robot_init_rot @ rot_delta_robot


#     quat_abs = mat_to_quat(rot_abs)
#     return pos_abs, quat_abs


def process_pose(
        pos_current, 
        rot_current,
        calib_pos: Optional[np.ndarray],
        calib_rot: Optional[np.ndarray],
        robot_init_pos: np.ndarray,
        robot_init_rot: np.ndarray,
        R_axes: np.ndarray,
        position_scale: float = 1.0,
        coord: str = 'vr'
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """
    基于 calib_pos 和 coord_vr_to_robot 计算当前位姿
    R_axes: 平移轴和旋转轴变换矩阵R
    """

    
    R_t, R_r = R_axes[0], R_axes[1]
    
    # 坐标轴映射
    pos_current_r = R_t @ pos_current  
    rot_current_r = R_r @ rot_current @ R_r.T  
    calib_pos_r = R_t @ calib_pos  
    calib_rot_r = R_r @ calib_rot @ R_r.T  
    
    if coord == 'tracker':
        # 基于tracker坐标系

        # 平移
        pos_delta = pos_current_r - calib_pos_r
        pos_delta_robot = robot_init_rot @ calib_rot_r.T @ pos_delta
        pos_abs = robot_init_pos + position_scale * pos_delta_robot

        # 旋转
        rot_abs = robot_init_rot @ calib_rot_r.T @ rot_current_r

    elif coord == 'world':

        # 2. 基于世界坐标系平移
        pos_delta = pos_current_r - calib_pos_r
        pos_delta_robot = robot_init_rot @ pos_delta
        pos_abs = robot_init_pos + position_scale * pos_delta_robot

        # 2. 基于世界坐标系旋转
        rot_delta = rot_current_r @ calib_rot_r.T  # 相对标定旋转增量
        rot_abs = robot_init_rot @ rot_delta

    quat_abs = mat_to_quat(rot_abs, quat_order='wxyz')

    return pos_abs, quat_abs


def flip_trans_axes(mat, axes=''):
    mat_flip = mat.copy()
    for i, axis in enumerate(['x', 'y', 'z']):
        if axis in axes:
            mat_flip[i] *= -1
    return mat_flip



def scale_to_joint_range(data, hand_joint_range):
    """
    将 0-1000 的列表数据映射到 hand_joint_range 范围
    data: list of int/float, 长度应与 hand_joint_range 一致
    hand_joint_range: [[min, max], ...] 每个关节的范围
    返回: list of float
    """
    scaled = []
    for value, (min_val, max_val) in zip(data, hand_joint_range):
        # 线性映射
        joint_value = min_val + (max_val - min_val) * (value / 1000)
        scaled.append(joint_value)
    return scaled

def scale_to_0_1000(joint_values, hand_joint_range):
    """
    将 hand_joint_range 范围内的关节值映射回 0-1000
    joint_values: list of float
    hand_joint_range: [[min, max], ...] 每个关节的范围
    返回: list of int
    """
    scaled = []
    for joint_value, (min_val, max_val) in zip(joint_values, hand_joint_range):
        # 线性映射并四舍五入到整数
        value_0_1000 = int(round((joint_value - min_val) / (max_val - min_val) * 1000))
        # 防止超界
        value_0_1000 = max(0, min(1000, value_0_1000))
        scaled.append(value_0_1000)
    return scaled


def pose_quat_to_se3(pose, quat_order="wxyz"):
    """
    [x, y, z, qx, qy, qz, qw] or [x, y, z, qw, qx, qy, qz]
    list or ndarray -> pin.SE3
    """
    pose = np.asarray(pose)

    x, y, z = pose[:3]

    if quat_order == "wxyz":
        qw, qx, qy, qz = pose[3:]
    elif quat_order == "xyzw":
        qx, qy, qz, qw = pose[3:]
    else:
        raise ValueError("quat_order must be 'wxyz' or 'xyzw'")

    quat = pin.Quaternion(qx, qy, qz, qw)
    quat.normalize()

    return pin.SE3(quat, np.array([x, y, z]))

def pose_euler_to_se3(pose, euler_order="xyz"):
    """
    [x, y, z, rx, ry, rz]
    list or ndarray -> pin.SE3

    euler_order:
        "xyz" -> roll pitch yaw
        "zyx" -> yaw pitch roll
    """
    pose = np.asarray(pose)

    x, y, z = pose[:3]
    rx, ry, rz = pose[3:]

    if euler_order == "xyz":
        R = pin.rpy.rpyToMatrix(rx, ry, rz)
    elif euler_order == "zyx":
        R = pin.rpy.rpyToMatrix(rz, ry, rx)
    else:
        raise ValueError("euler_order must be 'xyz' or 'zyx'")

    return pin.SE3(R, np.array([x, y, z]))


def pose_to_se3(pose, quat_order="wxyz", euler_order="xyz"):

    if isinstance(pose, pin.SE3):
        return pose
    
    if isinstance(pose, (list, np.ndarray)):
        pose = np.asarray(pose)
        if pose.size == 7:
            return pose_quat_to_se3(pose, quat_order=quat_order)
        elif pose.size == 6:
            return pose_euler_to_se3(pose, euler_order=euler_order)
        else:
            raise ValueError("Pose must be pin.SE3 or length 6/7 list/array")

    return pose