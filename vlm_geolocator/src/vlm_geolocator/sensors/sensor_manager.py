"""Sensor Data Management Module"""
import time
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass
import numpy as np


@dataclass
class SensorData:
    """Sensor Data Encapsulation"""
    value: Any
    timestamp: float
    
    def get_age(self, current_time: float = None) -> float:
        """Get data age"""
        if current_time is None:
            current_time = time.time()
        return current_time - self.timestamp
    
    def is_valid(self, timeout: float, current_time: float = None) -> bool:
        """Check if data is valid"""
        return self.get_age(current_time) <= timeout


class SensorManager:
    """Sensor Data Manager"""
    
    def __init__(self, timeout: float = 0.5, stale_warning: float = 3.0, use_gimbal: bool = True, default_gimbal_attitude: Optional[Tuple[float, float, float]] = None):
        """
        Args:
            timeout: Sensor data validity timeout (seconds)
            stale_warning: Sensor data stale warning threshold (seconds)
            use_gimbal: Whether to use gimbal sensor input; if False, assume fixed attitude
            default_gimbal_attitude: Default gimbal attitude (roll, pitch, yaw in radians) when use_gimbal is False
        """
        self.timeout = timeout
        self.stale_warning = stale_warning
        self.use_gimbal = use_gimbal
        if default_gimbal_attitude is None:
            # Default: no roll/yaw, 60 degrees down pitch
            # This default is also configurable in camera_config.yaml
            self.default_gimbal_attitude = (0.0, -np.pi / 3.0, 0.0)
        else:
            self.default_gimbal_attitude = default_gimbal_attitude
        
        # Sensor data
        self._gps: Optional[SensorData] = None
        self._heading: Optional[SensorData] = None
        self._altitude: Optional[SensorData] = None
        self._gimbal: Optional[SensorData] = None
        
        # Warning state
        self._last_warning_time = {
            'gps': 0,
            'heading': 0,
            'altitude': 0,
            'gimbal': 0
        }
    
    def update_gps(self, latitude: float, longitude: float):
        """Update GPS data"""
        self._gps = SensorData(
            value=(latitude, longitude),
            timestamp=time.time()
        )
    
    def update_heading(self, heading_deg: float):
        """Update heading data (degrees)"""
        self._heading = SensorData(
            value=np.radians(heading_deg),
            timestamp=time.time()
        )
    
    def update_altitude(self, altitude_m: float):
        """Update altitude data (meters)"""
        self._altitude = SensorData(
            value=altitude_m,
            timestamp=time.time()
        )
    
    def update_gimbal(self, roll_deg: float, pitch_deg: float, yaw_deg: float):
        """Update gimbal attitude (degrees)"""
        self._gimbal = SensorData(
            value=(
                np.radians(roll_deg),
                np.radians(pitch_deg),
                np.radians(yaw_deg)
            ),
            timestamp=time.time()
        )
    
    def get_gps(self) -> Optional[Tuple[float, float]]:
        """Get GPS data (latitude, longitude)"""
        if self._gps and self._gps.is_valid(self.timeout):
            return self._gps.value
        return None
    
    def get_heading(self) -> Optional[float]:
        """Get heading data (radians)"""
        if self._heading and self._heading.is_valid(self.timeout):
            return self._heading.value
        return None
    
    def get_altitude(self) -> Optional[float]:
        """Get altitude data (meters)"""
        if self._altitude and self._altitude.is_valid(self.timeout):
            return self._altitude.value
        return None
    
    def get_gimbal(self) -> Optional[Tuple[float, float, float]]:
        """Get gimbal attitude (roll, pitch, yaw radians)"""
        if self._gimbal and self._gimbal.is_valid(self.timeout):
            return self._gimbal.value
        return None
    
    def get_snapshot(self, capture_time: float = None) -> Dict[str, Any]:
        """
        Get sensor data snapshot
        
        Args:
            capture_time: Capture timestamp
            
        Returns:
            Sensor data snapshot dictionary
        """
        if capture_time is None:
            capture_time = time.time()
        
        snapshot = {
            'gps': None,
            'heading': None,
            'altitude': None,
            'gimbal': None,
            'capture_time': capture_time,
            'data_age': {}
        }
        
        # GPS
        if self._gps and self._gps.is_valid(self.timeout, capture_time):
            snapshot['gps'] = self._gps.value
            snapshot['data_age']['gps'] = self._gps.get_age(capture_time)
        
        # Heading
        if self._heading and self._heading.is_valid(self.timeout, capture_time):
            snapshot['heading'] = self._heading.value
            snapshot['data_age']['heading'] = self._heading.get_age(capture_time)
        
        # Altitude
        if self._altitude and self._altitude.is_valid(self.timeout, capture_time):
            snapshot['altitude'] = self._altitude.value
            snapshot['data_age']['altitude'] = self._altitude.get_age(capture_time)
        
        # Gimbal
        if self.use_gimbal:
            if self._gimbal and self._gimbal.is_valid(self.timeout, capture_time):
                snapshot['gimbal'] = self._gimbal.value
                snapshot['data_age']['gimbal'] = self._gimbal.get_age(capture_time)
        else:
            snapshot['gimbal'] = self.default_gimbal_attitude
            snapshot['data_age']['gimbal'] = 0.0
        
        return snapshot
    
    def check_health(self, uptime: float, warning_interval: float = 10.0) -> Dict[str, str]:
        """
        Check sensor health status
        
        Args:
            uptime: System uptime
            warning_interval: Warning interval time
            
        Returns:
            Warning message dictionary {sensor_name: warning_message}
        """
        current_time = time.time()
        warnings = {}
        
        # Give system 5 seconds to start up
        if uptime < 5.0:
            return warnings
        
        def check_sensor(name: str, sensor_data: Optional[SensorData]):
            """Check single sensor"""
            # Check if warning should be issued
            if current_time - self._last_warning_time[name] < warning_interval:
                return
            
            if sensor_data is None:
                warnings[name] = f"{name}: No data received"
                self._last_warning_time[name] = current_time
            else:
                age = sensor_data.get_age(current_time)
                if age > self.stale_warning:
                    warnings[name] = f"{name}: Data is stale ({age:.1f}s)"
                    self._last_warning_time[name] = current_time
        
        check_sensor('gps', self._gps)
        check_sensor('heading', self._heading)
        check_sensor('altitude', self._altitude)
        if self.use_gimbal:
            check_sensor('gimbal', self._gimbal)
        
        return warnings
    
    def format_snapshot(self, snapshot: Dict[str, Any]) -> str:
        """Format snapshot to readable string"""
        lines = ['='*60, '📡 Sensor Data Snapshot:']
        
        if snapshot['gps'] is not None:
            age = snapshot['data_age'].get('gps', 0)
            lat, lon = snapshot['gps']
            lines.append(f'  GPS: lat={lat:.6f}, lon={lon:.6f} (age: {age:.3f}s)')
        else:
            lines.append('  GPS: ✗ Missing')
        
        if snapshot['heading'] is not None:
            age = snapshot['data_age'].get('heading', 0)
            lines.append(
                f'  Heading: {np.degrees(snapshot["heading"]):.2f}° '
                f'({snapshot["heading"]:.6f} rad) (age: {age:.3f}s)'
            )
        else:
            lines.append('  Heading: ✗ Missing')
        
        if snapshot['altitude'] is not None:
            age = snapshot['data_age'].get('altitude', 0)
            lines.append(f'  Altitude: {snapshot["altitude"]:.2f} m (age: {age:.3f}s)')
        else:
            lines.append('  Altitude: ✗ Missing')
        
        if snapshot['gimbal'] is not None:
            age = snapshot['data_age'].get('gimbal', 0)
            roll, pitch, yaw = snapshot['gimbal']
            lines.append(
                f'  Gimbal: roll={np.degrees(roll):.2f}°, '
                f'pitch={np.degrees(pitch):.2f}°, '
                f'yaw={np.degrees(yaw):.2f}° (age: {age:.3f}s)'
            )
        else:
            lines.append('  Gimbal: ✗ Missing')
        
        lines.append('='*60)
        return '\n'.join(lines)
