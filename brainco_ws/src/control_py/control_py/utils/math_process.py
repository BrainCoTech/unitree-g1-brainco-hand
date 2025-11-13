def add_lists(list1, list2):
    """
    将两个列表对应位置的元素相加
    """
    if len(list1) != len(list2):
        raise ValueError("两个列表长度必须相同")
    
    return [a + b for a, b in zip(list1, list2)]


def clip(x, limit_min, limit_max, round_d=7):
    return round(max(min(x, limit_max), limit_min), round_d)