#!/usr/bin/env python3
"""
Compute distance between two GNSS points in meters.

Supports two modes:
1. Interactive mode: Run script and enter coordinates step by step
2. Command-line mode: Pass coordinates as arguments

Command-line usage:
    python gnss_distance.py lat1 lon1 lat2 lon2
    python gnss_distance.py lat1 lon1 alt1 lat2 lon2 alt2
"""

import math
import sys


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


def distance_gnss(
    lat1_deg, lon1_deg, lat2_deg, lon2_deg,
    alt1_m=None, alt2_m=None
):
    """
    Distance between two GNSS points (lat, lon, optional alt) in meters.

    If altitudes are provided (both alt1_m and alt2_m not None),
    returns the 3D distance, else returns horizontal distance.
    """
    horiz = haversine_distance_m(lat1_deg, lon1_deg, lat2_deg, lon2_deg)

    if alt1_m is not None and alt2_m is not None:
        dz = alt2_m - alt1_m
        return math.sqrt(horiz * horiz + dz * dz)

    return horiz


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


def parse_coordinate_input(prompt, coord_name):
    """
    Parse coordinate input with error handling.
    Returns float value or None if user wants to skip.
    """
    while True:
        try:
            value = input(prompt).strip()
            if value.lower() in ['', 'n', 'no', 'skip']:
                return None
            return float(value)
        except ValueError:
            print(f"❌ Invalid input! Please enter a valid {coord_name} value.")
        except KeyboardInterrupt:
            print("\n\n⚠️  Operation cancelled")
            sys.exit(0)


def interactive_mode():
    """
    Interactive mode: prompt user to input two waypoints.
    """
    print("=" * 60)
    print("  GNSS Waypoint Distance Calculator (Interactive Mode)")
    print("=" * 60)
    print()
    
    # Waypoint 1
    print("📍 Waypoint 1:")
    lat1 = parse_coordinate_input("  Latitude: ", "latitude")
    if lat1 is None:
        print("❌ Latitude is required!")
        sys.exit(1)
    
    lon1 = parse_coordinate_input("  Longitude: ", "longitude")
    if lon1 is None:
        print("❌ Longitude is required!")
        sys.exit(1)
    
    alt1 = parse_coordinate_input("  Altitude (optional, press Enter to skip): ", "altitude")
    
    print()
    
    # Waypoint 2
    print("📍 Waypoint 2:")
    lat2 = parse_coordinate_input("  Latitude: ", "latitude")
    if lat2 is None:
        print("❌ Latitude is required!")
        sys.exit(1)
    
    lon2 = parse_coordinate_input("  Longitude: ", "longitude")
    if lon2 is None:
        print("❌ Longitude is required!")
        sys.exit(1)
    
    alt2 = parse_coordinate_input("  Altitude (optional, press Enter to skip): ", "altitude")
    
    print()
    print("-" * 60)
    
    # Calculate distance
    horiz_dist = haversine_distance_m(lat1, lon1, lat2, lon2)
    
    # Calculate bearing
    bearing = calculate_bearing(lat1, lon1, lat2, lon2)
    
    # Print results
    print()
    print("📊 Results:")
    print(f"  Waypoint 1: ({lat1:.6f}°, {lon1:.6f}°)" + (f" @ {alt1:.1f}m" if alt1 is not None else ""))
    print(f"  Waypoint 2: ({lat2:.6f}°, {lon2:.6f}°)" + (f" @ {alt2:.1f}m" if alt2 is not None else ""))
    print()
    print(f"  Horizontal distance: {horiz_dist:.3f} m ({horiz_dist / 1000:.6f} km)")
    print(f"  Bearing:             {bearing:.1f}° (0°=North, 90°=East, 180°=South, 270°=West)")
    
    if alt1 is not None and alt2 is not None:
        dz = alt2 - alt1
        dist_3d = math.sqrt(horiz_dist * horiz_dist + dz * dz)
        print(f"  Altitude difference: {dz:+.3f} m")
        print(f"  3D distance:         {dist_3d:.3f} m")
    
    print()
    print("=" * 60)


def commandline_mode(args):
    """
    Command-line mode: parse arguments and calculate distance.
    """
    if len(args) not in (4, 6):
        print("❌ Invalid arguments!")
        print()
        print("Usage:")
        print("  python gnss_distance.py lat1 lon1 lat2 lon2")
        print("  python gnss_distance.py lat1 lon1 alt1 lat2 lon2 alt2")
        print()
        print("Or run without arguments for interactive mode:")
        print("  python gnss_distance.py")
        sys.exit(1)

    try:
    if len(args) == 4:
        lat1, lon1, lat2, lon2 = map(float, args)
            alt1, alt2 = None, None
    else:
        lat1, lon1, alt1, lat2, lon2, alt2 = map(float, args)
        
        # Calculate distances
        horiz_dist = haversine_distance_m(lat1, lon1, lat2, lon2)
        bearing = calculate_bearing(lat1, lon1, lat2, lon2)
        
        print(f"Horizontal distance: {horiz_dist:.3f} m")
        print(f"Bearing:             {bearing:.1f}°")
        
        if alt1 is not None and alt2 is not None:
            dz = alt2 - alt1
            dist_3d = math.sqrt(horiz_dist * horiz_dist + dz * dz)
            print(f"Altitude difference: {dz:+.3f} m")
            print(f"3D distance:         {dist_3d:.3f} m")
            
    except ValueError as e:
        print(f"❌ Input error: {e}")
        sys.exit(1)


def main():
    args = sys.argv[1:]
    
    # If no arguments, run interactive mode
    if len(args) == 0:
        interactive_mode()
    else:
        commandline_mode(args)


if __name__ == "__main__":
    main()

