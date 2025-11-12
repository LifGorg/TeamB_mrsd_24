"""
视觉处理模块
"""
from .video_receiver import VideoFrameReceiver
from .detector_interface import DetectorInterface
from .isaac_detector import IsaacDetectorWrapper

__all__ = [
    'VideoFrameReceiver',
    'DetectorInterface',
    'IsaacDetectorWrapper'
]

