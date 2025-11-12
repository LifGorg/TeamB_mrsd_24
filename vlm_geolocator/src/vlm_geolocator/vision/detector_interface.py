"""目标检测器接口"""
from abc import ABC, abstractmethod
from typing import List, Tuple
import numpy as np


class DetectorInterface(ABC):
    """检测器抽象接口"""
    
    @abstractmethod
    def detect(self, image: np.ndarray) -> List[Tuple[float, float]]:
        """
        检测图像中的目标
        
        Args:
            image: 图像 (numpy array)
            
        Returns:
            检测结果列表 [(cx, cy), ...]
        """
        pass

