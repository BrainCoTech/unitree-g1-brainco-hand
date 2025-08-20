import numpy as np

def mat_update(prev_mat, mat):
    if np.linalg.det(mat) == 0:
        return prev_mat, False # Return previous matrix and False flag if the new matrix is non-singular (determinant ≠ 0).
    else:
        return mat, True


def fast_mat_inv(mat):
    ret = np.eye(4)
    ret[:3, :3] = mat[:3, :3].T
    ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return ret



def pseudo_inverse(A, b):
    A_np = np.array(A)
    b_np = np.array(b)
    # 计算伪逆矩阵
    A_pinv = np.linalg.pinv(A_np)

    # 求解线性方程组
    x = A_pinv @ b_np
    return x.tolist()



