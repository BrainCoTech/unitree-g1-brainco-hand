import numpy as np
from math import atan2, asin, degrees
from scipy.spatial.transform import Rotation as R
import pinocchio as pin   

# 24种旋转矩阵
rotation_matrices = [
    [[1, 0, 0], [0, 1, 0], [0, 0, 1]],      #0
    [[1, 0, 0], [0, 0, -1], [0, 1, 0]],     #1
    [[1, 0, 0], [0, -1, 0], [0, 0, -1]],    #2
    [[1, 0, 0], [0, 0, 1], [0, -1, 0]],     #3
    [[0, 1, 0], [-1, 0, 0], [0, 0, 1]],     #4
    [[0, 0, 1], [-1, 0, 0], [0, 1, 0]],     #5
    [[0, -1, 0], [-1, 0, 0], [0, 0, -1]],   #6
    [[0, 0, -1], [-1, 0, 0], [0, -1, 0]],   #7
    [[0, -1, 0], [1, 0, 0], [0, 0, 1]],     #8
    [[0, 0, 1], [1, 0, 0], [0, 1, 0]],      #9
    [[0, 1, 0], [1, 0, 0], [0, 0, -1]],     #10
    [[0, 0, -1], [1, 0, 0], [0, -1, 0]],    #11
    [[0, 0, -1], [0, 1, 0], [1, 0, 0]],     #12
    [[0, 1, 0], [0, 0, 1], [1, 0, 0]],      #13
    [[0, 0, 1], [0, -1, 0], [1, 0, 0]],     #14
    [[0, -1, 0], [0, 0, -1], [1, 0, 0]],    #15
    [[0, 0, -1], [0, -1, 0], [-1, 0, 0]],   #16
    [[0, -1, 0], [0, 0, 1], [-1, 0, 0]],    #17
    [[0, 0, 1], [0, 1, 0], [-1, 0, 0]],     #18
    [[0, 1, 0], [0, 0, -1], [-1, 0, 0]],    #19
    [[-1, 0, 0], [0, -1, 0], [0, 0, 1]],    #20
    [[-1, 0, 0], [0, 0, -1], [0, -1, 0]],   #21
    [[-1, 0, 0], [0, 1, 0], [0, 0, -1]],    #22
    [[-1, 0, 0], [0, 0, 1], [0, 1, 0]],     #23
    ]


def rotate_axis(vectors: np.ndarray, matrix_index=0) -> np.ndarray:
    """
    使用预定义的旋转矩阵对向量进行旋转
    - vectors: np.ndarray, shape (N, 3)
    - matrix_index: 旋转矩阵索引 (0-23)
    - 返回: 旋转后的向量 np.ndarray, shape (N, 3)
    """

    R = np.array(rotation_matrices[matrix_index])
    return vectors @ R.T


def rotate_pose_matrix(pose: np.ndarray, matrix_index=0) -> np.ndarray:
    """
    使用预定义的旋转矩阵对 4x4 位姿矩阵进行旋转
    - pose: np.ndarray, shape (4, 4)，输入的位姿矩阵
    - matrix_index: int, 旋转矩阵索引 (0 - 23)
    - 返回: np.ndarray, shape (4, 4)，旋转后的位姿矩阵
    """

    if pose.shape != (4, 4):
        raise ValueError("pose 必须是 4x4 矩阵")

    # 提取旋转和平移
    R_in = pose[:3, :3]
    t_in = pose[:3, 3]

    # 把旋转矩阵的三个基向量送进 rotate_axis
    basis_vectors = np.eye(3)  # [[1,0,0],[0,1,0],[0,0,1]]
    R_extra = rotate_axis(basis_vectors, matrix_index)

    # 新旋转 = R_extra * R_in
    R_new = R_extra @ R_in

    # 组装新的位姿矩阵
    pose_new = np.eye(4)
    pose_new[:3, :3] = R_new
    pose_new[:3, 3] = t_in  # 平移保持不变

    return pose_new


def relative_poses_flat(matrices):
    """
    计算每个4x4位姿矩阵相对于第0个矩阵的位姿
    - matrices: np.ndarray, shape (25, 16)  # 每行是4x4展平
    - 返回: np.ndarray, shape (25, 16)  # 每行是相对位姿展平
    """
    n = matrices.shape[0]
    rel_matrices = np.zeros_like(matrices)

    # 将第0个矩阵reshape成4x4
    ref = matrices[0].reshape(4, 4).T
    ref_inv = np.linalg.inv(ref)

    for i in range(n):
        mat_i = matrices[i].reshape(4, 4).T
        rel_mat = ref_inv @ mat_i
        rel_matrices[i] = rel_mat.T.flatten()

    return rel_matrices


def bytes_to_matrix(hand_bytes):
    """
    将 bytes 或 list 形式的手部数据转换为 25x16 的 numpy 矩阵
    - hand_bytes: bytes 或 25*16 list
    - 返回: np.array(25,16) 或 None
    """
    if isinstance(hand_bytes, bytes) and len(hand_bytes) > 1:
        arr = np.frombuffer(hand_bytes, dtype=np.float32)
        if arr.size != 25*16:
            print(f"Warning: unexpected hand data size: {arr.size}")
        return arr.reshape(25,16)
    elif isinstance(hand_bytes, list) and len(hand_bytes) == 25*16:
        return np.array(hand_bytes, dtype=np.float32).reshape(25,16)
    else:
        return None

    
def mat_to_xyzabc(mat: np.ndarray, rad=False):
    """
    将 4x4 位姿矩阵转为位姿 [x, y, z, a, b, c]
    - mat: (4,4) np.ndarray
        [[a0, a1, a2, a3],
        [a4, a5, a6, a7],
        [a8, a9, a10, a11],
        [a12, a13, a14, a15]]
    - rad: 如果为 True 则返回弧度, 否则返回角度
    - 返回: [x, y, z, a, b, c]
    """
    T = mat[:3, 3]
    R = mat[:3, :3]
    if abs(R[2,0]) != 1:
        b = -asin(R[2,0])
        a = atan2(R[2,1]/np.cos(b), R[2,2]/np.cos(b))
        c = atan2(R[1,0]/np.cos(b), R[0,0]/np.cos(b))
    else:  # Gimbal lock
        c = 0
        if R[2,0] == -1:
            b = np.pi/2
            a = c + atan2(R[0,1], R[0,2])
        else:
            b = -np.pi/2
            a = -c + atan2(-R[0,1], -R[0,2])
    xyz = [round(float(x),2) for x in T]
    if rad:
        abc = [round(float(x),2) for x in [a,b,c]]  # 弧度
    else:
        abc = [round(degrees(x),2) for x in [a,b,c]]  # 角度
    return xyz + abc


def joint_to_xyz_abc(joint_matrix, transpose=False, rad=False, rotate_index=0):
    """
    将单个关节的展平 4x4 矩阵转为 [x, y, z, a, b, c], 并使用预定义旋转矩阵调整坐标系
    - joint_matrix: 长度为16的list 或 (16,) array
    - transpose: 是否先转置矩阵
    - rad: 如果为 True 则返回弧度, 否则返回角度
    - rotate_index: 旋转矩阵索引 (0-23), 用于调整坐标系, 默认为0不旋转
    - 返回: [x, y, z, a, b, c]
    """
    mat = np.array(joint_matrix, dtype=np.float32).reshape(4,4)
    if transpose:
        mat = mat.T
    mat_rotote = rotate_pose_matrix(mat, matrix_index=rotate_index)
    return mat_to_xyzabc(mat_rotote, rad=rad)


def mat_to_xyz_quat(mat, xyz_axes=None, rot_axes=None):
    """
    将 4x4 齐次位姿矩阵转换为 [x, y, z, qx, qy, qz, qw]
    可调换 XYZ 轴顺序和旋转方向
    - mat: np.ndarray 或 list, shape=(4,4), 齐次位姿矩阵
        [[R, p],
        [0, 1]]
    - xyz_axes: np.ndarray, shape=(3,3), 用于调换位置轴顺序
    - rot_axes: np.ndarray, shape=(3,3), 用于调换旋转矩阵

    返回: pose: np.ndarray, shape=(7,) [x, y, z, qx, qy, qz, qw]
    """
    mat = np.array(mat, dtype=float)
    assert mat.shape == (4,4), "输入必须是4x4矩阵"

    # 提取平移和旋转
    T = mat[:3, 3]
    R_mat = mat[:3, :3]

    # 调换位置轴顺序
    if xyz_axes is not None:
        xyz_axes = np.array(xyz_axes, dtype=float)
        assert xyz_axes.shape == (3,3), "xyz_axes 必须是3x3矩阵"
        T = xyz_axes @ T

    # 调换旋转轴
    if rot_axes is not None:
        rot_axes = np.array(rot_axes, dtype=float)
        assert rot_axes.shape == (3,3), "rot_axes 必须是3x3矩阵"
        R_mat = rot_axes @ R_mat

    # 转四元数
    quat = R.from_matrix(R_mat).as_quat()  # [x, y, z, w]

    return np.concatenate([T, quat])


def joint_to_xyz_quat(joint_matrix, transpose=False, xyz_index=0, rot_index=0):
    """
    将单个关节的展平 4x4 矩阵转为 [x, y, z, qx, qy, qz, qw], 并可调换位置轴顺序和旋转方向
    - joint_matrix: 长度为16的list 或 (16,) array
    - transpose: 是否先转置矩阵
    - rad: 如果为 True 则返回弧度, 否则返回角度
    - xyz_index: 位置轴顺序索引 (0-23), 用于调换位置轴顺序
    - rot_index: 旋转轴顺序索引 (0-23), 用于调换旋转轴顺序
    - 返回: [x, y, z, a, b, c]
    """
    
    mat = np.array(joint_matrix, dtype=np.float32).reshape(4,4)
    if transpose:
        mat = mat.T
    mat_quat = mat_to_xyz_quat(mat, 
                               xyz_axes=rotation_matrices[xyz_index], 
                               rot_axes=rotation_matrices[rot_index])

    return mat_quat


def compute_relative_pose_quat(init_pose, target_pose):
    """
    计算目标位姿相对初始位姿的 [dx, dy, dx, dqx, dqy, dqz, dqw]
    - init_pose: [x, y, z, qx, qy, qz, qw]
    - target_pose: [x, y, z, qx, qy, qz, qw]
    - 返回: np.array [dx, dy, dz, dqx, dqy, dqz, dqw]
    """
    init_pose = np.array(init_pose, dtype=float)
    target_pose = np.array(target_pose, dtype=float)

    # 平移差
    d_pos = target_pose[:3] - init_pose[:3]

    # 四元数差（目标相对初始）
    q_init = R.from_quat(init_pose[3:])
    q_target = R.from_quat(target_pose[3:])
    q_rel = q_target * q_init.inv()

    # 转为四元数输出
    d_quat = q_rel.as_quat()

    return np.concatenate([d_pos, d_quat])


def pose_euler_to_quat(curr_pose, euler_order='zyx', degrees=True):
    """
    将长度6的姿态列表 [x, y, z, Rx, Ry, Rz] 转换为 [x, y, z, qx, qy, qz, qw]
    - curr_pose: list 或 np.ndarray, [x, y, z, Rx, Ry, Rz]
    - euler_order: 欧拉角顺序，默认 'zyx'
    - degrees: 欧拉角单位是否为度，默认 True
    - 返回: list: [x, y, z, qx, qy, qz, qw]
    """
    x, y, z, Rx, Ry, Rz = curr_pose

    Ry = Ry + np.pi

    # 转四元数
    r = R.from_euler(euler_order, [Rx, Ry, Rz], degrees=degrees)
    qx, qy, qz, qw = r.as_quat()

    return [x, y, z, float(qx), float(qy), float(qz), float(qw)]


def pose_quat_to_euler(curr_pose, euler_order='zyx', degrees=True):
    """
    将长度7的姿态列表 [x, y, z, qx, qy, qz, qw]
    转换为 [x, y, z, Rx, Ry, Rz]

    - curr_pose: list 或 np.ndarray, [x, y, z, qx, qy, qz, qw]
    - euler_order: 欧拉角顺序，默认 'zyx'
    - degrees: 欧拉角单位是否为度，默认 True
    - 返回: list: [x, y, z, Rx, Ry, Rz]
    """
    x, y, z, qx, qy, qz, qw = curr_pose

    # 四元数转欧拉角
    r = R.from_quat([qx, qy, qz, qw])
    Rx, Ry, Rz = r.as_euler(euler_order, degrees=degrees)

    # 与 pose_euler_to_quat 中的处理保持对称
    Ry = Ry - np.pi

    return [x, y, z, float(Rx), float(Ry), float(Rz)]



# def add_pose_quat_flip_axis(initial_pose, relative_pose, euler_order="xyz", degrees=False, flip_axis=""):
#     """
#     在初始位姿基础上，叠加相对位姿，返回新的位姿,
#     并可对相对旋转的指定轴翻转角度。
#     - initial_pose: [x, y, z, qx, qy, qz, qw]
#     - relative_pose: [dx, dy, dz, dqx, dqy, dqz, dqw]
#     - euler_order: 欧拉角顺序
#     - degrees: 欧拉角单位是否为度
#     - flip_axis: 需要翻转的轴，例如 'x' 或 'yz' 表示翻转 Ry 和 Rz
#     - 返回: [x, y, z, qx, qy, qz, qw]
#     """
#     initial_pose = np.array(initial_pose, dtype=float)
#     relative_pose = np.array(relative_pose, dtype=float)

#     # 位置直接相加
#     new_pos = initial_pose[:3] + relative_pose[:3]

#     # 四元数
#     q_init = R.from_quat(initial_pose[3:])
#     q_rel  = R.from_quat(relative_pose[3:])

#     # 转欧拉角
#     eulers = q_rel.as_euler(euler_order, degrees=degrees)

#     # 翻转指定轴
#     axis_map = {'x':0, 'y':1, 'z':2}
#     for ax in flip_axis.lower():
#         if ax in axis_map:
#             eulers[axis_map[ax]] *= -1

#     # 转回四元数
#     q_rel_flipped = R.from_euler(euler_order, eulers, degrees=degrees)

#     # 四元数叠加
#     q_new = q_rel_flipped * q_init
#     new_quat = q_new.as_quat()

#     return np.concatenate([new_pos, new_quat])


def add_pose_quat_flip_axis(initial_pose, relative_pose, euler_order="xyz", degrees=False, flip_xyz="", flip_axis=""):
    """
    在初始位姿基础上，叠加相对位姿，返回新的位姿,
    并可对相对旋转的指定轴翻转角度。
    - initial_pose: [x, y, z, qx, qy, qz, qw]
    - relative_pose: [dx, dy, dz, dqx, dqy, dqz, dqw]
    - euler_order: 欧拉角顺序
    - degrees: 欧拉角单位是否为度
    - flip_axis: 需要翻转的轴，例如 'x' 或 'yz' 表示翻转 Ry 和 Rz
    - 返回: [x, y, z, qx, qy, qz, qw]
    """
    initial_pose = np.array(initial_pose, dtype=float)
    relative_pose = np.array(relative_pose, dtype=float)

    # 位置直接相加
    delta_pos = relative_pose[:3].copy()

    axis_map = {'x': 0, 'y': 1, 'z': 2}
    for ax in flip_xyz.lower():
        if ax in axis_map:
            delta_pos[axis_map[ax]] *= -1

    new_pos = initial_pose[:3] + delta_pos
    # new_pos = initial_pose[:3] + relative_pose[:3]

    # 四元数
    q_init = R.from_quat(initial_pose[3:])
    q_rel  = R.from_quat(relative_pose[3:])

    # 转欧拉角
    eulers = q_rel.as_euler(euler_order, degrees=degrees)

    # 翻转指定轴
    for ax in flip_axis.lower():
        if ax in axis_map:
            eulers[axis_map[ax]] *= -1

    # 转回四元数
    q_rel_flipped = R.from_euler(euler_order, eulers, degrees=degrees)

    # 四元数叠加
    q_new = q_rel_flipped * q_init
    new_quat = q_new.as_quat()

    return np.concatenate([new_pos, new_quat])


def print_matrix4(matrix, label="Matrix"):
    """打印 4x4 矩阵"""
    rounded = [round(float(x), 2) for x in matrix]
    print(f"{label}:")
    for r in range(0, 16, 4):
        print(f"{rounded[r:r+4]}")


def print_all_joints(joint_matrix, hand_name="Hand", pose_matrix=False):
    """
    打印手的所有关节信息
    - joint_matrix: np.ndarray, shape (25, 16) 或 None
    - hand_name: 手的名称
    - pose_matrix: bool, 如果为 True 则打印 4x4 位姿矩阵, 否则打印 xyzabc
    """
    if joint_matrix is None:
        print(f"No {hand_name} data")
        return

    for i, joint in enumerate(joint_matrix):
        if pose_matrix:
            # 调用打印矩阵的函数
            print_matrix4(joint, label=f"{hand_name} Joint {i}")
        else:
            # 打印 xyzabc
            pose = joint_to_xyz_abc(joint_matrix[i], transpose=True)
            print(f"{hand_name} Joint {i}: x,y,z,a,b,c = {pose}")


def print_hand_state(state_dict, hand_name="Hand"):
    """
    逐项打印手的状态信息
    - state_dict: dict, 手的状态字典
    - hand_name: str, 手的名称
    """
    print(f"{hand_name} state:")
    for k, v in state_dict.items():
        print(f"  {k} = {v}")


def print_controller_state(label: str, state: dict):
    """
    逐项打印手柄状态
    - label: str, 手柄名称
    - state: dict, 手柄状态字典
    """
    if not state:
        print(f"  {label} State: None")
        return
    print(f"  {label} State:")
    for k, v in state.items():
        print(f" {k}: {v}")


def pose_quat_to_se3(pose):
    """[x, y, z, qx, qy, qz, qw] list or ndarray -> pin.SE3"""
    x, y, z, qx, qy, qz, qw = pose
    quat = pin.Quaternion(qx, qy, qz, qw)
    quat.normalize()
    return pin.SE3(quat, np.array([x, y, z]))
