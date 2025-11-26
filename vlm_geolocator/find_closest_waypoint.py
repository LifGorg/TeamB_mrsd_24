#!/usr/bin/env python3
"""
Find the closest distance from an obstacle point to a mission path.

This script reads YAML-formatted mission waypoint messages (from ROS mavros)
and calculates the minimum distance from the obstacle to the path formed by
connecting the waypoints (finds closest point on any path segment/edge).

Usage:
    python find_closest_waypoint.py <obstacle_lat> <obstacle_lon> [obstacle_alt]

Then paste the YAML waypoint messages, and press Ctrl+D when done.
"""

import math
import sys
import yaml
from typing import List, Dict, Any, Optional, Tuple


# Mean Earth radius in meters (WGS84-ish)
EARTH_RADIUS_M = 6371000.0


def haversine_distance_m(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    """
    Great-circle (surface) distance between two lat/lon points in meters.
    """
    # Convert to radians
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = (math.sin(dlat / 2.0) ** 2
         + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    return EARTH_RADIUS_M * c


def distance_3d(lat1, lon1, alt1, lat2, lon2, alt2):
    """Calculate 3D distance between two points."""
    horiz = haversine_distance_m(lat1, lon1, lat2, lon2)
    dz = alt2 - alt1
    return math.sqrt(horiz * horiz + dz * dz)


def calculate_bearing(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    """
    Calculate initial bearing from point 1 to point 2.
    Returns bearing in degrees (0-360, where 0=North, 90=East)
    """
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)
    
    dlon = lon2 - lon1
    
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    
    bearing_rad = math.atan2(x, y)
    bearing_deg = math.degrees(bearing_rad)
    
    # Normalize to 0-360
    return (bearing_deg + 360) % 360


class Waypoint:
    """Represents a single waypoint."""
    def __init__(self, lat: float, lon: float, alt: float, 
                 frame: int = 0, command: int = 0, is_current: bool = False,
                 index: int = 0, message_index: int = 0):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.frame = frame
        self.command = command
        self.is_current = is_current
        self.index = index  # Index within the message
        self.message_index = message_index  # Which message this came from

    def __repr__(self):
        return (f"Waypoint(lat={self.lat:.6f}, lon={self.lon:.6f}, alt={self.alt:.1f}, "
                f"frame={self.frame}, cmd={self.command}, current={self.is_current})")


def parse_waypoint_messages(yaml_content: str) -> List[Waypoint]:
    """
    Parse YAML content containing multiple waypoint messages.
    Returns a list of all valid waypoints found.
    """
    waypoints = []
    
    # Split by document separator '---'
    documents = yaml_content.split('---')
    
    for msg_idx, doc in enumerate(documents):
        doc = doc.strip()
        if not doc:
            continue
            
        try:
            data = yaml.safe_load(doc)
            if not data or 'waypoints' not in data:
                continue
            
            wp_list = data.get('waypoints', [])
            if not wp_list:
                continue
            
            current_seq = data.get('current_seq', -1)
            
            for wp_idx, wp in enumerate(wp_list):
                if not isinstance(wp, dict):
                    continue
                
                # Extract GPS coordinates
                lat = wp.get('x_lat')
                lon = wp.get('y_long')
                alt = wp.get('z_alt')
                
                # Skip invalid waypoints
                if lat is None or lon is None or lat == 0 or lon == 0:
                    continue
                
                # Use default altitude if missing
                if alt is None:
                    alt = 0.0
                
                waypoint = Waypoint(
                    lat=lat,
                    lon=lon,
                    alt=alt,
                    frame=wp.get('frame', 0),
                    command=wp.get('command', 0),
                    is_current=wp.get('is_current', False),
                    index=wp_idx,
                    message_index=msg_idx
                )
                waypoints.append(waypoint)
                
        except yaml.YAMLError as e:
            print(f"⚠️  Warning: Failed to parse document {msg_idx + 1}: {e}", file=sys.stderr)
            continue
    
    return waypoints


def gps_to_meters(lat: float, lon: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """
    Convert GPS coordinates to local ENU (East-North-Up) coordinates in meters.
    Returns (east, north) offsets from reference point.
    """
    # Approximate meters per degree at reference latitude
    meters_per_deg_lat = 111111.0
    meters_per_deg_lon = 111111.0 * math.cos(math.radians(ref_lat))
    
    north = (lat - ref_lat) * meters_per_deg_lat
    east = (lon - ref_lon) * meters_per_deg_lon
    
    return east, north


def meters_to_gps(east: float, north: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """
    Convert local ENU coordinates back to GPS.
    Returns (latitude, longitude).
    """
    meters_per_deg_lat = 111111.0
    meters_per_deg_lon = 111111.0 * math.cos(math.radians(ref_lat))
    
    lat = ref_lat + (north / meters_per_deg_lat)
    lon = ref_lon + (east / meters_per_deg_lon)
    
    return lat, lon


def point_to_segment_distance(px: float, py: float, 
                              ax: float, ay: float, 
                              bx: float, by: float) -> Tuple[float, float, float, float]:
    """
    Calculate the shortest distance from point P to line segment AB.
    All coordinates are in meters (local ENU frame).
    
    Returns:
        (distance, closest_x, closest_y, t)
        where (closest_x, closest_y) is the closest point on the segment,
        and t is the parameter (0 = at A, 1 = at B)
    """
    # Vector from A to B
    abx = bx - ax
    aby = by - ay
    
    # Vector from A to P
    apx = px - ax
    apy = py - ay
    
    # Length squared of AB
    ab_len_sq = abx * abx + aby * aby
    
    if ab_len_sq == 0:
        # A and B are the same point
        dist = math.sqrt((px - ax)**2 + (py - ay)**2)
        return dist, ax, ay, 0.0
    
    # Project P onto line AB, computing parameter t
    # t = 0 means closest point is A, t = 1 means B
    t = (apx * abx + apy * aby) / ab_len_sq
    
    # Clamp t to [0, 1] to stay within segment
    t = max(0.0, min(1.0, t))
    
    # Compute closest point on segment
    closest_x = ax + t * abx
    closest_y = ay + t * aby
    
    # Distance from P to closest point
    dist = math.sqrt((px - closest_x)**2 + (py - closest_y)**2)
    
    return dist, closest_x, closest_y, t


def find_closest_path_point(target_lat: float, target_lon: float, target_alt: Optional[float],
                            waypoints: List[Waypoint]) -> Dict[str, Any]:
    """
    Find the closest point on the path formed by connecting waypoints.
    
    Returns a dictionary with detailed information about the closest segment.
    """
    if not waypoints:
        raise ValueError("No valid waypoints found!")
    
    if len(waypoints) < 2:
        # Only one waypoint, return distance to that point
        wp = waypoints[0]
        horiz_dist = haversine_distance_m(target_lat, target_lon, wp.lat, wp.lon)
        
        return {
            'type': 'single_waypoint',
            'waypoint': wp,
            'horizontal_distance': horiz_dist,
            'closest_lat': wp.lat,
            'closest_lon': wp.lon,
            'closest_alt': wp.alt,
        }
    
    # Use first waypoint as reference for local coordinate conversion
    ref_lat = waypoints[0].lat
    ref_lon = waypoints[0].lon
    
    # Convert obstacle to local coordinates
    target_east, target_north = gps_to_meters(target_lat, target_lon, ref_lat, ref_lon)
    
    # Find closest segment
    min_distance = float('inf')
    closest_segment = None
    closest_point_east = None
    closest_point_north = None
    closest_t = None
    
    for i in range(len(waypoints) - 1):
        wp_a = waypoints[i]
        wp_b = waypoints[i + 1]
        
        # Convert waypoints to local coordinates
        a_east, a_north = gps_to_meters(wp_a.lat, wp_a.lon, ref_lat, ref_lon)
        b_east, b_north = gps_to_meters(wp_b.lat, wp_b.lon, ref_lat, ref_lon)
        
        # Calculate distance to this segment
        dist, closest_east, closest_north, t = point_to_segment_distance(
            target_east, target_north,
            a_east, a_north,
            b_east, b_north
        )
        
        if dist < min_distance:
            min_distance = dist
            closest_segment = (i, wp_a, wp_b)
            closest_point_east = closest_east
            closest_point_north = closest_north
            closest_t = t
    
    # Convert closest point back to GPS
    seg_idx, wp_a, wp_b = closest_segment
    closest_lat, closest_lon = meters_to_gps(closest_point_east, closest_point_north, ref_lat, ref_lon)
    
    # Interpolate altitude
    closest_alt = wp_a.alt + closest_t * (wp_b.alt - wp_a.alt)
    
    # Calculate bearing from obstacle to closest point
    bearing = calculate_bearing(target_lat, target_lon, closest_lat, closest_lon)
    
    # Calculate 3D distance if altitude provided
    if target_alt is not None:
        alt_diff = closest_alt - target_alt
        dist_3d = math.sqrt(min_distance**2 + alt_diff**2)
    else:
        dist_3d = min_distance
        alt_diff = None
    
    return {
        'type': 'path_segment',
        'segment_index': seg_idx,
        'waypoint_a': wp_a,
        'waypoint_b': wp_b,
        'segment_parameter': closest_t,
        'horizontal_distance': min_distance,
        'distance_3d': dist_3d,
        'altitude_difference': alt_diff,
        'closest_lat': closest_lat,
        'closest_lon': closest_lon,
        'closest_alt': closest_alt,
        'bearing': bearing,
    }


def main():
    # Parse command-line arguments
    if len(sys.argv) < 3:
        print("Usage: python find_closest_waypoint.py <obstacle_lat> <obstacle_lon> [obstacle_alt]")
        print()
        print("Then paste the YAML waypoint messages and press Ctrl+D (Linux/Mac) or Ctrl+Z (Windows) when done.")
        print()
        print("Example:")
        print("  python find_closest_waypoint.py 40.4254603 -79.9545695")
        print("  python find_closest_waypoint.py 40.4254603 -79.9545695 6.0")
        sys.exit(1)
    
    try:
        target_lat = float(sys.argv[1])
        target_lon = float(sys.argv[2])
        target_alt = float(sys.argv[3]) if len(sys.argv) > 3 else None
    except ValueError as e:
        print(f"❌ Invalid coordinates: {e}")
        sys.exit(1)
    
    # Read YAML content from stdin
    print("=" * 70)
    print(f"🎯 Target Obstacle: ({target_lat:.6f}°, {target_lon:.6f}°)" + 
          (f" @ {target_alt:.1f}m" if target_alt is not None else ""))
    print("=" * 70)
    print()
    print("📥 Paste YAML waypoint messages below, then press Ctrl+D when done:")
    print("-" * 70)
    
    try:
        yaml_content = sys.stdin.read()
    except KeyboardInterrupt:
        print("\n\n⚠️  Operation cancelled")
        sys.exit(0)
    
    if not yaml_content.strip():
        print("❌ No input received!")
        sys.exit(1)
    
    # Parse waypoints
    print()
    print("🔍 Parsing waypoint messages...")
    waypoints = parse_waypoint_messages(yaml_content)
    
    if not waypoints:
        print("❌ No valid waypoints found in the input!")
        sys.exit(1)
    
    print(f"✅ Found {len(waypoints)} valid waypoint(s)")
    print()
    
    # Remove duplicates (same lat/lon/alt)
    unique_waypoints = []
    seen = set()
    for wp in waypoints:
        key = (round(wp.lat, 7), round(wp.lon, 7), round(wp.alt, 2))
        if key not in seen:
            seen.add(key)
            unique_waypoints.append(wp)
    
    if len(unique_waypoints) < len(waypoints):
        print(f"ℹ️  Removed {len(waypoints) - len(unique_waypoints)} duplicate waypoint(s)")
        print(f"📍 Analyzing {len(unique_waypoints)} unique waypoint(s)")
        print()
    
    # Find closest point on path
    result = find_closest_path_point(target_lat, target_lon, target_alt, unique_waypoints)
    
    # Print results
    print("=" * 70)
    print("📊 RESULTS:")
    print("=" * 70)
    print()
    print(f"🎯 Target Obstacle:")
    print(f"     Latitude:  {target_lat:.6f}°")
    print(f"     Longitude: {target_lon:.6f}°")
    if target_alt is not None:
        print(f"     Altitude:  {target_alt:.1f} m")
    print()
    
    if result['type'] == 'single_waypoint':
        wp = result['waypoint']
        print(f"ℹ️  Only one waypoint found - showing distance to that waypoint")
        print()
        print(f"🔵 Waypoint:")
        print(f"     Latitude:  {wp.lat:.6f}°")
        print(f"     Longitude: {wp.lon:.6f}°")
        print(f"     Altitude:  {wp.alt:.1f} m")
        print()
        print(f"📏 Distance: {result['horizontal_distance']:.3f} m")
    else:
        # Path segment result
        wp_a = result['waypoint_a']
        wp_b = result['waypoint_b']
        seg_idx = result['segment_index']
        t = result['segment_parameter']
        
        print(f"🛤️  Closest Path Segment: #{seg_idx + 1} → #{seg_idx + 2}")
        print(f"     Waypoint A (#{seg_idx + 1}):")
        print(f"       Lat: {wp_a.lat:.6f}°, Lon: {wp_a.lon:.6f}°, Alt: {wp_a.alt:.1f}m")
        print(f"     Waypoint B (#{seg_idx + 2}):")
        print(f"       Lat: {wp_b.lat:.6f}°, Lon: {wp_b.lon:.6f}°, Alt: {wp_b.alt:.1f}m")
        print()
        print(f"📍 Closest Point on Path:")
        print(f"     Latitude:  {result['closest_lat']:.6f}°")
        print(f"     Longitude: {result['closest_lon']:.6f}°")
        print(f"     Altitude:  {result['closest_alt']:.1f} m")
        print(f"     Position on segment: {t:.1%} from A to B")
        
        # Describe position
        if t < 0.01:
            pos_desc = "at waypoint A"
        elif t > 0.99:
            pos_desc = "at waypoint B"
        else:
            pos_desc = f"between waypoints ({t:.1%} along segment)"
        print(f"     ({pos_desc})")
        print()
        print(f"📏 Distance Measurements:")
        print(f"     Horizontal distance: {result['horizontal_distance']:.3f} m")
        print(f"     Bearing (obstacle→path): {result['bearing']:.1f}° (0°=N, 90°=E, 180°=S, 270°=W)")
        
        if target_alt is not None and result['altitude_difference'] is not None:
            alt_diff = result['altitude_difference']
            print(f"     Altitude difference: {alt_diff:+.3f} m (path is {'above' if alt_diff > 0 else 'below'} obstacle)")
            print(f"     3D distance:         {result['distance_3d']:.3f} m")
    
    print()
    
    # Show all path segments sorted by distance
    if len(unique_waypoints) >= 2:
        print("=" * 70)
        print("📋 All Path Segments (sorted by distance to obstacle):")
        print("=" * 70)
        print()
        
        # Use first waypoint as reference
        ref_lat = unique_waypoints[0].lat
        ref_lon = unique_waypoints[0].lon
        target_east, target_north = gps_to_meters(target_lat, target_lon, ref_lat, ref_lon)
        
        # Calculate distance to each segment
        segment_info = []
        for i in range(len(unique_waypoints) - 1):
            wp_a = unique_waypoints[i]
            wp_b = unique_waypoints[i + 1]
            
            # Convert to local coordinates
            a_east, a_north = gps_to_meters(wp_a.lat, wp_a.lon, ref_lat, ref_lon)
            b_east, b_north = gps_to_meters(wp_b.lat, wp_b.lon, ref_lat, ref_lon)
            
            # Distance to segment
            dist, closest_e, closest_n, t = point_to_segment_distance(
                target_east, target_north, a_east, a_north, b_east, b_north
            )
            
            # Segment length
            seg_length = math.sqrt((b_east - a_east)**2 + (b_north - a_north)**2)
            
            segment_info.append((i, wp_a, wp_b, dist, t, seg_length))
        
        # Sort by distance
        segment_info.sort(key=lambda x: x[3])
        
        for rank, (seg_idx, wp_a, wp_b, dist, t, seg_len) in enumerate(segment_info, 1):
            # Mark the closest segment
            if result['type'] == 'path_segment' and seg_idx == result['segment_index']:
                marker = "👉"
            else:
                marker = "  "
            
            # Position description
            if t < 0.01:
                pos = "at A"
            elif t > 0.99:
                pos = "at B"
            else:
                pos = f"{t:.0%} A→B"
            
            print(f"{marker} Segment #{seg_idx + 1}→#{seg_idx + 2}: Distance = {dist:.3f} m ({pos})")
            print(f"       A: ({wp_a.lat:.6f}°, {wp_a.lon:.6f}°) @ {wp_a.alt:.1f}m")
            print(f"       B: ({wp_b.lat:.6f}°, {wp_b.lon:.6f}°) @ {wp_b.alt:.1f}m")
            print(f"       Segment length: {seg_len:.3f} m")
            print()
    else:
        print("=" * 70)
        print("ℹ️  Only one waypoint - no path segments to analyze")
    
    print("=" * 70)


if __name__ == "__main__":
    main()


