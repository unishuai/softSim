# ============================================
# @File    : tactoBodyCreateClass.py
# @Date    : 2025-02-10 下午9:03
# @Author  : 帅宇昕
# ============================================
from dataclasses import dataclass
from typing import List
import numpy as np

@dataclass
class TactoBodyDataClass:
    """

    urdf_path: str  urdf路径
    base_position: List[float]  初始位置
    global_scaling: float = 0.15    初始大小
    """
    urdf_path: str
    base_position: np.ndarray
    global_scaling: float = 1
    id: int = -1


