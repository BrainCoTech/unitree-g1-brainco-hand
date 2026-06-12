import numpy as np

def add_lists(list1, list2):
    """
    将两个列表对应位置的元素相加
    """
    if len(list1) != len(list2):
        raise ValueError("两个列表长度必须相同")
    
    return [a + b for a, b in zip(list1, list2)]


def clip(x, limit_min, limit_max, round_d=7):
    return round(max(min(x, limit_max), limit_min), round_d)


def generate_random_pose(init_pose: list, move_range: list) -> list:
    """
    在 init_pose 基础上按照 move_range 生成随机位姿

    Args:
        init_pose (list): 初始位姿 [x, y, z, rx, ry, rz]
        move_range (list): 每一维允许的扰动范围

    Returns:
        list: 随机生成的位姿
    """
    init_pose_np = np.array(init_pose)
    move_range_np = np.array(move_range)

    random_pose = init_pose_np + np.random.uniform(-move_range_np, move_range_np)

    return random_pose.tolist()