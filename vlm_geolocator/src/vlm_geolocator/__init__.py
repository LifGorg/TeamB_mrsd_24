"""
VLM Geolocator - Vision Language Model Geolocation System

A modular system for detecting targets from drone video streams and estimating their GPS coordinates.
"""
__version__ = '2.0.0'
__author__ = 'LifGorg'

from . import core
from . import sensors
from . import gps
from . import vision
from . import ros_interface

__all__ = [
    'core',
    'sensors',
    'gps',
    'vision',
    'ros_interface'
]
