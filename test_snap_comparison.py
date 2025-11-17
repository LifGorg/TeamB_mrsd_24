"""Test GPS calculation with and without snap_to_reference"""
import numpy as np
from typing import Dict, Any, Tuple, List
from scipy.spatial.transform import Rotation


# Reference points from gps_config.yaml (set 3, near mil19)
REFERENCE_POINTS: List[Tuple[float, float]] = [
    (40.413555, -79.948786),
    (40.413618, -79.948846),
    (40.413557, -79.948904),
]


class GPSCalculator:
    """GPS Coordinate Estimator"""
    
    def __init__(self, intrinsic_matrix: np.ndarray, earth_radius_lat: float = 111111.0, snap_to_reference: bool = True):
        """
        Args:
            intrinsic_matrix: Camera intrinsic matrix
            earth_radius_lat: Earth radius in latitude direction (meters/degree)
            snap_to_reference: Whether to snap to nearest reference point
        """
        self.intrinsic = intrinsic_matrix
        self.intrinsic_inv = np.linalg.inv(intrinsic_matrix)
        self.earth_radius_lat = earth_radius_lat
        self.snap_to_reference = snap_to_reference

    def _meters_per_deg_lon(self, lat_deg: float) -> float:
        return self.earth_radius_lat * np.cos(np.radians(lat_deg))

    def _distance_m(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        meters_per_deg_lon = self._meters_per_deg_lon(lat1)
        d_north = (lat2 - lat1) * self.earth_radius_lat
        d_east = (lon2 - lon1) * meters_per_deg_lon
        return float(np.hypot(d_east, d_north))

    def _snap_to_reference_point(self, lat: float, lon: float) -> Tuple[int, float, float, float]:
        distances = [self._distance_m(lat, lon, ref_lat, ref_lon) for (ref_lat, ref_lon) in REFERENCE_POINTS]
        idx = int(np.argmin(distances))
        snapped_lat, snapped_lon = REFERENCE_POINTS[idx]
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

        if self.snap_to_reference:
            idx, snapped_lat, snapped_lon, snap_dist_m = self._snap_to_reference_point(estimated_lat, estimated_lon)
            d_north = (snapped_lat - lat) * self.earth_radius_lat
            d_east = (snapped_lon - lon) * meters_per_deg_lon
            result.update({
                "estimated_latitude": snapped_lat,
                "estimated_longitude": snapped_lon,
                "offset_north": d_north,
                "offset_east": d_east,
                "lateral_distance": float(np.hypot(d_east, d_north)),
                "snapped_to_reference": True,
                "snapped_index": idx + 1,  # 1-based index
                "snap_distance_m": snap_dist_m,
            })
        else:
            result["snapped_to_reference"] = False

        return result


def test_detection(intrinsic_matrix, earth_radius_lat, drone_gps, drone_heading, 
                   drone_altitude, gimbal_attitude, pixel_x, pixel_y, 
                   logged_lat, logged_lon, logged_lateral_distance, detection_num):
    """Test a single detection"""
    
    print("=" * 80)
    print(f"检测 #{detection_num} - GPS计算测试")
    print("=" * 80)
    print("\n输入数据:")
    print(f"  像素坐标: ({pixel_x}, {pixel_y})")
    print(f"  无人机GPS: {drone_gps}")
    print(f"  无人机航向: {drone_heading:.4f} rad ({np.degrees(drone_heading):.2f}°)")
    print(f"  无人机高度: {drone_altitude} m")
    print(f"  云台姿态: roll={gimbal_attitude[0]:.4f}, pitch={gimbal_attitude[1]:.4f}, yaw={gimbal_attitude[2]:.4f} rad")
    print(f"  云台姿态(度): roll={np.degrees(gimbal_attitude[0]):.2f}°, pitch={np.degrees(gimbal_attitude[1]):.2f}°, yaw={np.degrees(gimbal_attitude[2]):.2f}°")
    
    # Test 1: Without snap to reference
    print("\n" + "-" * 80)
    print("测试 1: 关闭参考点捕捉 (snap_to_reference=False)")
    print("-" * 80)
    calculator_no_snap = GPSCalculator(
        intrinsic_matrix=intrinsic_matrix,
        earth_radius_lat=earth_radius_lat,
        snap_to_reference=False
    )
    
    result_no_snap = calculator_no_snap.estimate_target_gps(
        pixel_x=pixel_x,
        pixel_y=pixel_y,
        drone_gps=drone_gps,
        drone_heading=drone_heading,
        drone_altitude=drone_altitude,
        gimbal_attitude=gimbal_attitude
    )
    
    print("\n原始估计结果 (无捕捉):")
    print(f"  估计纬度: {result_no_snap['estimated_latitude']:.10f}")
    print(f"  估计经度: {result_no_snap['estimated_longitude']:.10f}")
    print(f"  北向偏移: {result_no_snap['offset_north']:.6f} m")
    print(f"  东向偏移: {result_no_snap['offset_east']:.6f} m")
    print(f"  水平距离: {result_no_snap['lateral_distance']:.6f} m")
    
    # Test 2: With snap to reference
    print("\n" + "-" * 80)
    print("测试 2: 启用参考点捕捉 (snap_to_reference=True)")
    print("-" * 80)
    calculator_with_snap = GPSCalculator(
        intrinsic_matrix=intrinsic_matrix,
        earth_radius_lat=earth_radius_lat,
        snap_to_reference=True
    )
    
    result_with_snap = calculator_with_snap.estimate_target_gps(
        pixel_x=pixel_x,
        pixel_y=pixel_y,
        drone_gps=drone_gps,
        drone_heading=drone_heading,
        drone_altitude=drone_altitude,
        gimbal_attitude=gimbal_attitude
    )
    
    print("\n捕捉后的结果:")
    print(f"  估计纬度: {result_with_snap['estimated_latitude']:.10f}")
    print(f"  估计经度: {result_with_snap['estimated_longitude']:.10f}")
    print(f"  北向偏移: {result_with_snap['offset_north']:.6f} m")
    print(f"  东向偏移: {result_with_snap['offset_east']:.6f} m")
    print(f"  水平距离: {result_with_snap['lateral_distance']:.6f} m")
    print(f"  捕捉到的参考点: #{result_with_snap['snapped_index']} - {REFERENCE_POINTS[result_with_snap['snapped_index']-1]}")
    print(f"  捕捉距离: {result_with_snap['snap_distance_m']:.6f} m")
    
    # Compare with logged result
    print("\n" + "-" * 80)
    print("对比实际记录的结果 (来自 geolocations.json)")
    print("-" * 80)
    
    print(f"\n记录的结果:")
    print(f"  纬度: {logged_lat:.10f}")
    print(f"  经度: {logged_lon:.10f}")
    print(f"  水平距离: {logged_lateral_distance:.6f} m")
    
    print(f"\n差异分析 (记录值 vs 捕捉计算):")
    lat_diff = logged_lat - result_with_snap['estimated_latitude']
    lon_diff = logged_lon - result_with_snap['estimated_longitude']
    dist_diff = logged_lateral_distance - result_with_snap['lateral_distance']
    
    print(f"  纬度差异: {lat_diff:.12f} ({lat_diff * earth_radius_lat:.6f} m)")
    print(f"  经度差异: {lon_diff:.12f} ({lon_diff * calculator_with_snap._meters_per_deg_lon(logged_lat):.6f} m)")
    print(f"  距离差异: {dist_diff:.6f} m")
    
    # Show reference points
    print("\n" + "-" * 80)
    print("参考点列表 (距离原始估计):")
    print("-" * 80)
    for i, (lat, lon) in enumerate(REFERENCE_POINTS, 1):
        dist = calculator_no_snap._distance_m(result_no_snap['estimated_latitude'], 
                                               result_no_snap['estimated_longitude'], 
                                               lat, lon)
        marker = " ← 捕捉到此点" if i == result_with_snap['snapped_index'] else ""
        print(f"  参考点 {i}: ({lat:.6f}, {lon:.6f}) - {dist:.6f} m{marker}")
    
    print("\n")


def main():
    """Test GPS calculation with actual sample data"""
    
    # Camera intrinsics from camera_config.yaml (RealSense D435)
    intrinsic_matrix = np.array([
        [920.407043457031, 0.0, 638.451110839844],
        [0.0, 920.293395996094, 373.086090087891],
        [0.0, 0.0, 1.0]
    ])
    
    # Earth radius from system_config.yaml
    earth_radius_lat = 111042.60  # meters per degree latitude
    
    # Sample data from sensor_snapshot.json (frame f002929)
    drone_gps = (40.4135344, -79.9490127)
    drone_heading = 0.6085963101704227  # radians
    drone_altitude = 14.917  # meters
    gimbal_attitude = (0.0, -1.0471975511965976, 0.0)  # (roll, pitch, yaw) in radians
    
    print("\n" + "=" * 80)
    print("GPS计算测试 - 样本数据分析 (frame f002929)")
    print("=" * 80)
    print("\n传感器数据:")
    print(f"  无人机GPS: {drone_gps}")
    print(f"  无人机航向: {drone_heading:.4f} rad ({np.degrees(drone_heading):.2f}°)")
    print(f"  无人机高度: {drone_altitude} m")
    print(f"  云台姿态(度): roll={np.degrees(gimbal_attitude[0]):.2f}°, pitch={np.degrees(gimbal_attitude[1]):.2f}°, yaw={np.degrees(gimbal_attitude[2]):.2f}°")
    print("\n")
    
    # Detection 1: (640.0, 360.0) - center of image
    test_detection(
        intrinsic_matrix=intrinsic_matrix,
        earth_radius_lat=earth_radius_lat,
        drone_gps=drone_gps,
        drone_heading=drone_heading,
        drone_altitude=drone_altitude,
        gimbal_attitude=gimbal_attitude,
        pixel_x=640.0,
        pixel_y=360.0,
        logged_lat=40.41355885487813,
        logged_lon=-79.94889725182401,
        logged_lateral_distance=10.131409279397053,
        detection_num=1
    )
    
    # Detection 2: (1148.8, 77.76) - top right area
    test_detection(
        intrinsic_matrix=intrinsic_matrix,
        earth_radius_lat=earth_radius_lat,
        drone_gps=drone_gps,
        drone_heading=drone_heading,
        drone_altitude=drone_altitude,
        gimbal_attitude=gimbal_attitude,
        pixel_x=1148.8,
        pixel_y=77.76,
        logged_lat=40.41355000095278,
        logged_lon=-79.9487791471723,
        logged_lateral_distance=19.821849045271424,
        detection_num=2
    )
    
    print("=" * 80)
    print("测试完成")
    print("=" * 80)


if __name__ == "__main__":
    main()

