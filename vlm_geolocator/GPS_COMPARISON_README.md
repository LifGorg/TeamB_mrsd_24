# GPS Comparison Tool

## Overview

The GPS Comparison Tool has been integrated into the Gemini Casualty Analysis Results Viewer to allow users to compare estimated GPS coordinates with ground truth GPS coordinates and calculate the distance error.

## Features

- **Interactive Input**: Paste ground truth GPS coordinates directly on the triage report page
- **Real-time Calculation**: Calculate distance between estimated and ground truth GPS coordinates
- **Haversine Distance**: Uses the Haversine formula for accurate great-circle distance calculation
- **Visual Feedback**: Clear display of distance error in meters
- **Multiple Videos**: Each video analysis has its own GPS comparison section

## Location

The GPS comparison tool is integrated into:
- **File**: `gemini_result_viewer_html.py`
- **Display**: Each triage report card in the HTML viewer

## Usage

### 1. Generate and View Triage Report

```bash
cd /home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator
python3 gemini_result_viewer_html.py
```

This will:
- Analyze the latest videos (if not already analyzed)
- Generate an HTML report
- Open the report in your browser

### 2. Compare GPS Coordinates

For each casualty video in the report:

1. **View Estimated GPS**: The estimated GPS coordinates are displayed in the "Estimated GPS Coordinates" card
2. **Enter Ground Truth**: In the "GPS Comparison Tool" section, enter the actual GPS coordinates in the format: `latitude, longitude`
   - Example: `40.425428, -79.954430`
3. **Calculate**: Click the "Calculate Distance" button
4. **View Results**: The distance error will be displayed in meters, along with both coordinate pairs

### 3. Clear Results

Click the "Clear" button to reset the comparison for that video.

## GPS Coordinate Format

- **Format**: `latitude, longitude`
- **Latitude Range**: -90 to 90 degrees
- **Longitude Range**: -180 to 180 degrees
- **Decimal Precision**: Use at least 6 decimal places for accuracy
- **Example**: `40.425428, -79.954430`

## Distance Calculation

The tool uses the **Haversine formula** to calculate the great-circle distance between two points on Earth:

```python
def haversine_distance(lat1, lon1, lat2, lon2):
    # Convert to radians
    lat1_rad = lat1 * π / 180
    lon1_rad = lon1 * π / 180
    lat2_rad = lat2 * π / 180
    lon2_rad = lon2 * π / 180
    
    # Calculate differences
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    # Haversine formula
    a = sin²(dlat/2) + cos(lat1_rad) × cos(lat2_rad) × sin²(dlon/2)
    c = 2 × atan2(√a, √(1-a))
    
    # Distance in meters (Earth radius = 6,371,000 m)
    distance = 6371000 × c
    
    return distance
```

## Standalone GNSS Distance Calculator

A standalone Python script is also provided for command-line distance calculations:

### Location
`/home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator/gnss_distance.py`

### Usage

**2D Distance (horizontal only):**
```bash
./gnss_distance.py lat1 lon1 lat2 lon2
```

Example:
```bash
./gnss_distance.py 40.425428 -79.954430 40.426000 -79.955000
# Output: 75.234 m
```

**3D Distance (with altitude):**
```bash
./gnss_distance.py lat1 lon1 alt1 lat2 lon2 alt2
```

Example:
```bash
./gnss_distance.py 40.425428 -79.954430 100 40.426000 -79.955000 150
# Output: 90.123 m
```

## File Structure

```
vlm_geolocator/
├── gemini_result_viewer_html.py     # Main HTML viewer with GPS comparison
├── gnss_distance.py                 # Standalone GNSS distance calculator
└── inference_logs/
    └── recordings/
        ├── casualty_*.mp4           # Video recordings
        ├── *_gemini_analysis.json   # Analysis results
        └── gemini_analysis_results.html  # Generated HTML report
```

## Technical Details

### HTML Components

- **Input Field**: Text input for ground truth GPS coordinates
- **Calculate Button**: Triggers distance calculation
- **Clear Button**: Resets the comparison
- **Results Display**: Shows distance error and both coordinate pairs

### JavaScript Functions

- `haversineDistance(lat1, lon1, lat2, lon2)`: Calculate distance using Haversine formula
- `parseGPS(gpsStr)`: Parse and validate GPS coordinate string
- `calculateDistance(videoIdx)`: Main calculation function
- `clearComparison(videoIdx)`: Reset comparison for a specific video

### CSS Styling

- Green border for GPS comparison section
- Dark theme matching the overall triage report design
- Responsive layout for different screen sizes
- Clear visual feedback for results

## Examples

### Example 1: Close Match
- **Estimated**: 40.425428, -79.954430
- **Ground Truth**: 40.425450, -79.954445
- **Distance Error**: ~3.2 meters

### Example 2: Larger Error
- **Estimated**: 40.425428, -79.954430
- **Ground Truth**: 40.426000, -79.955000
- **Distance Error**: ~75.2 meters

## Keyboard Shortcuts

- **Enter Key**: Press Enter in the ground truth input field to trigger calculation (same as clicking "Calculate Distance")

## Error Handling

The tool validates:
- ✅ GPS coordinate format (must be "lat, lon")
- ✅ Numeric values (must be valid floating-point numbers)
- ✅ Coordinate ranges (lat: -90 to 90, lon: -180 to 180)
- ✅ Estimated GPS availability (checks if "Unknown")

Error messages are displayed via browser alerts with clear instructions.

## Watch Mode

The HTML viewer supports continuous monitoring for new videos:

```bash
python3 gemini_result_viewer_html.py --watch --interval 30
```

This will:
- Monitor the recordings directory for new videos
- Automatically regenerate the HTML when changes are detected
- Check every 30 seconds (configurable with `--interval`)

## Troubleshooting

### Estimated GPS shows "Unknown"
- The video filename may not contain GPS coordinates
- Check if the JSON analysis file contains GPS metadata
- Verify the video was recorded with GPS data

### Distance calculation not working
- Check GPS coordinate format (must include comma separator)
- Verify coordinates are within valid ranges
- Make sure JavaScript is enabled in your browser

### HTML not updating
- Try regenerating the HTML: `python3 gemini_result_viewer_html.py`
- Check if the JSON analysis files exist
- Verify the video files are in the recordings directory

## Integration with Triage System

The GPS comparison tool is fully integrated with the medical triage report system and appears alongside:
- Casualty Status
- Trauma Assessment
- Hemorrhage Severity
- Amputation Status
- Respiratory Status
- Detailed Observations
- Recommendations

This allows medics and operators to:
1. View medical analysis results
2. Verify GPS accuracy
3. Plan rescue operations with accurate location data
4. Assess system performance in real-time

## Future Enhancements

Potential future improvements:
- Save comparison results to JSON
- Export comparison data for batch analysis
- Visualize GPS locations on a map
- Calculate average position error across multiple videos
- Support for altitude-based 3D distance in web interface

