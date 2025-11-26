# GNSS Distance Tools Usage Guide

This directory contains two GNSS distance calculation tools:

## 1. `gnss_distance.py` - Point-to-Point Distance

Calculate the distance between two GNSS waypoints.

### Usage

**Interactive Mode:**
```bash
cd vlm_geolocator
python3 gnss_distance.py
```

Then follow the prompts to enter two waypoints.

**Command-line Mode:**
```bash
# 2D distance (lat/lon only)
python3 gnss_distance.py 40.4254603 -79.9545695 40.4256789 -79.9543210

# 3D distance (with altitude)
python3 gnss_distance.py 40.4254603 -79.9545695 6.0 40.4256789 -79.9543210 8.0
```

### Output
- Horizontal distance (meters and km)
- Bearing angle (degrees from North)
- Altitude difference (if provided)
- 3D distance (if altitude provided)

---

## 2. `find_closest_waypoint.py` - Obstacle to Path Distance

Find the minimum distance from an obstacle point to a mission path (connected waypoint segments).

### Usage

```bash
# Basic usage (2D)
python3 find_closest_waypoint.py <obstacle_lat> <obstacle_lon>

# With altitude (3D)
python3 find_closest_waypoint.py <obstacle_lat> <obstacle_lon> <obstacle_alt>
```

Then paste the YAML waypoint messages from ROS (e.g., from `/dtc_mrsd_/mavros/mission/waypoints` topic) and press **Ctrl+D** when done.

### Example

```bash
python3 find_closest_waypoint.py 40.4254603 -79.9545695 6.0 < test_waypoints.yaml
```

Or interactively:
```bash
python3 find_closest_waypoint.py 40.4254603 -79.9545695 6.0
# Then paste YAML waypoint messages
# Press Ctrl+D when finished
```

### Input Format

The script expects YAML-formatted waypoint messages like:
```yaml
current_seq: 1
waypoints:
- frame: 3
  command: 16
  is_current: true
  x_lat: 40.4255868
  y_long: -79.9545044
  z_alt: 6.0
- frame: 3
  command: 16
  is_current: false
  x_lat: 40.42546018810975
  y_long: -79.95466930666403
  z_alt: 6.0
---
```

### Output

The script will:
1. Parse all waypoint messages and extract unique waypoints
2. Find the closest point on the path (on any segment/edge between waypoints)
3. Report:
   - Which path segment is closest
   - The exact closest point coordinates on that segment
   - Horizontal distance from obstacle to path
   - Position along the segment (e.g., "43% from A to B")
   - Bearing from obstacle to closest point
   - 3D distance (if altitude provided)
4. List all path segments sorted by distance

### Key Features

- **Point-to-path distance**: Finds perpendicular distance to line segments, not just nearest waypoint
- **Handles multiple messages**: Processes YAML with multiple `---` separated documents
- **Removes duplicates**: Automatically filters duplicate waypoints
- **Detailed output**: Shows closest segment, position along segment, and bearing

---

## Test Data

Use `test_waypoints.yaml` for testing:
```bash
python3 find_closest_waypoint.py 40.4254603 -79.9545695 6.0 < test_waypoints.yaml
```

---

## Mathematical Notes

### Point-to-Point Distance
Uses the **Haversine formula** for great-circle distance on Earth's surface.

### Point-to-Path Distance
1. Converts GPS coordinates to local ENU (East-North-Up) Cartesian coordinates
2. For each segment, calculates perpendicular distance using vector projection
3. Finds the segment with minimum distance
4. Converts closest point back to GPS coordinates

This ensures accurate distance calculation for obstacle avoidance and path planning.



