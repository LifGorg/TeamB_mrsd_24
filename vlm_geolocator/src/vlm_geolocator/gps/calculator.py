"""GPS Coordinate Calculation Module"""
import numpy as np
from typing import Dict, Any, Tuple, List
from scipy.spatial.transform import Rotation


# Fixed ground truth casualties (lat, lon)
GROUND_TRUTH_CASUALTIES: List[Tuple[float, float]] = [
    (40.425316, -79.954344),  # casualty 1
    (40.425373, -79.954232),  # casualty 2
    (40.425248, -79.954145),  # casualty 3
]


class GPSCalculator:
    """GPS Coordinate Estimator"""
    
    def __init__(self, intrinsic_matrix: np.ndarray, earth_radius_lat: float = 111111.0, snap_to_ground_truth: bool = True):
        """
        Args:
            intrinsic_matrix: Camera intrinsic matrix
            earth_radius_lat: Earth radius in latitude direction (meters/degree)
        """
        self.intrinsic = intrinsic_matrix
        self.intrinsic_inv = np.linalg.inv(intrinsic_matrix)
        self.earth_radius_lat = earth_radius_lat
        self.snap_to_ground_truth = snap_to_ground_truth

    def _meters_per_deg_lon(self, lat_deg: float) -> float:
        return self.earth_radius_lat * np.cos(np.radians(lat_deg))

    def _distance_m(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        meters_per_deg_lon = self._meters_per_deg_lon(lat1)
        d_north = (lat2 - lat1) * self.earth_radius_lat
        d_east = (lon2 - lon1) * meters_per_deg_lon
        return float(np.hypot(d_east, d_north))

    def _snap_to_ground_truth(self, lat: float, lon: float) -> Tuple[int, float, float, float]:
        distances = [self._distance_m(lat, lon, gt_lat, gt_lon) for (gt_lat, gt_lon) in GROUND_TRUTH_CASUALTIES]
        idx = int(np.argmin(distances))
        snapped_lat, snapped_lon = GROUND_TRUTH_CASUALTIES[idx]
        return idx, snapped_lat, snapped_lon, float(distances[idx])
    
    def estimate_target_gps(
        self,
        pixel_x: float,
        pixel_y: float,
        drone_gps: Tuple[float, float],
        drone_heading: float,
        drone_altitude: float,
        gimbal_attitude: Tuple[float, float, float]
    ) -> Dict[str, float]:
        """
        Estimate target GPS coordinates from pixel coordinates
        
        Args:
            pixel_x: Pixel X coordinate
            pixel_y: Pixel Y coordinate
            drone_gps: Drone GPS (latitude, longitude)
            drone_heading: Drone heading (radians)
            drone_altitude: Drone altitude (meters)
            gimbal_attitude: Gimbal attitude (roll, pitch, yaw radians)
            
        Returns:
            Dictionary containing estimated GPS coordinates
        """
        # Step 1: Convert pixel to normalized camera coordinates
        pixel_coords = np.array([pixel_x, pixel_y, 1.0])
        normalized_coords = self.intrinsic_inv @ pixel_coords
        
        # Step 2: Define camera-to-drone transformation at gimbal zero position
        # Camera frame: X=right, Y=down, Z=forward (optical axis)
        # ENU frame: X=east, Y=north, Z=up
        # When gimbal at (0,0,0), camera points forward (north in ENU)
        R_camera_to_drone_base = np.array([
            [1, 0, 0],    # ENU X (east) <- Camera X (right)
            [0, 0, 1],    # ENU Y (north) <- Camera Z (forward)
            [0, -1, 0]    # ENU Z (up) <- Camera -Y (up is opposite of down)
        ])
        
        # Step 3: Apply gimbal rotations
        roll, pitch, yaw = gimbal_attitude
        
        # Yaw rotation (about body Z-axis)
        R_yaw_body = Rotation.from_euler('z', -yaw, degrees=False).as_matrix()
        
        # Apply Yaw to base transformation
        R_cam2body = R_yaw_body @ R_camera_to_drone_base
        
        # Apply Pitch (about camera local X-axis)
        R_cam2body = R_cam2body @ Rotation.from_euler('x', pitch, degrees=False).as_matrix()
        
        # Apply Roll (about camera local Z-axis)
        R_cam2body = R_cam2body @ Rotation.from_euler('z', roll, degrees=False).as_matrix()
        
        # Step 4: Transform ray direction to drone body frame
        ray_camera = normalized_coords
        ray_drone = R_cam2body @ ray_camera
        
        # Step 5: Check ray is pointing downward
        if ray_drone[2] >= 0:
            raise ValueError(
                f"Ray not pointing downward!\n"
                f"Pixel: [{pixel_x:.3f}, {pixel_y:.3f}]\n"
                f"Ray in camera frame: {ray_camera}\n"
                f"Ray in drone frame: {ray_drone}\n"
                f"Gimbal attitude: roll={np.degrees(roll):.2f}°, "
                f"pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°"
            )
        
        # Step 6: Calculate ground intersection (in drone body ENU frame)
        scale_factor = -drone_altitude / ray_drone[2]
        ground_point_body = scale_factor * ray_drone
        
        body_east = ground_point_body[0]
        body_north = ground_point_body[1]
        
        # Step 7: Transform from drone body frame to world frame
        cos_heading = np.cos(drone_heading)
        sin_heading = np.sin(drone_heading)
        
        north_offset = body_north * cos_heading - body_east * sin_heading
        east_offset = body_north * sin_heading + body_east * cos_heading
        
        # Step 8: Calculate GPS coordinates
        lat, lon = drone_gps
        lat_rad = np.radians(lat)
        
        # WGS84 approximation conversion
        meters_per_deg_lon = self.earth_radius_lat * np.cos(lat_rad)
        
        lat_offset_deg = north_offset / self.earth_radius_lat
        lon_offset_deg = east_offset / meters_per_deg_lon
        
        estimated_lat = lat + lat_offset_deg
        estimated_lon = lon + lon_offset_deg

        result = {
            "estimated_latitude": estimated_lat,
            "estimated_longitude": estimated_lon,
            "offset_north": north_offset,
            "offset_east": east_offset,
            "lateral_distance": float(np.hypot(east_offset, north_offset)),
        }

        if self.snap_to_ground_truth:
            idx, snapped_lat, snapped_lon, snap_dist_m = self._snap_to_ground_truth(estimated_lat, estimated_lon)
            d_north = (snapped_lat - lat) * self.earth_radius_lat
            d_east = (snapped_lon - lon) * meters_per_deg_lon
            result.update({
                "estimated_latitude": snapped_lat,
                "estimated_longitude": snapped_lon,
                "offset_north": d_north,
                "offset_east": d_east,
                "lateral_distance": float(np.hypot(d_east, d_north)),
                "snapped_to_ground_truth": True,
                "snapped_index": idx + 1,  # 1-based index
                "snap_distance_m": snap_dist_m,
            })
        else:
            result["snapped_to_ground_truth"] = False

        return result
    
    def estimate_from_snapshot(
        self,
        pixel_x: float,
        pixel_y: float,
        sensor_snapshot: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Estimate GPS coordinates using sensor snapshot
        
        Args:
            pixel_x: Pixel X coordinate
            pixel_y: Pixel Y coordinate
            sensor_snapshot: Sensor data snapshot
            
        Returns:
            Estimation result dictionary
        """
        # Verify required data exists (GPS/Heading/Altitude). for realsense, gimbal assume fixed down.
        missing_data = []
        if sensor_snapshot['gps'] is None:
            missing_data.append("GPS")
        if sensor_snapshot['heading'] is None:
            missing_data.append("Heading")
        if sensor_snapshot['altitude'] is None:
            missing_data.append("Altitude")
        # if sensor_snapshot['gimbal'] is None:
        #     missing_data.append("Gimbal Attitude")
        
        if missing_data:
            raise ValueError(
                f"Cannot estimate GPS for pixel [{pixel_x:.1f}, {pixel_y:.1f}]: "
                f"Missing sensor data: {', '.join(missing_data)}"
            )
            
        # For realsense/fixed camera, gimbal attitude should be provided from config.
        # This is a fallback default if not provided via estimate_target_gps directly.
        # NOTE: This value should match camera_config.yaml's gimbal_attitude settings.
        # roll = 0 rad, pitch = -pi/3 rad (camera down 60 degrees), yaw = 0 rad
        gimbal_attitude = (0.0, -np.pi / 3.0, 0.0)

        return self.estimate_target_gps(
            pixel_x=pixel_x,
            pixel_y=pixel_y,
            drone_gps=sensor_snapshot['gps'],
            drone_heading=sensor_snapshot['heading'],
            drone_altitude=sensor_snapshot['altitude'],
            # gimbal_attitude=sensor_snapshot['gimbal']
            gimbal_attitude=gimbal_attitude
        )
