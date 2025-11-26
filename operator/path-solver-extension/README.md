# Path Solver Extension

This Foxglove extension subscribes to `/casualty_geolocated` and `/dtc_mrsd_/mavros/global_position/global`.
It provides a "Solve Paths" button that calculates a path visiting all detected geolocations starting from the drone's current position, minimizing distance (Nearest Neighbor heuristic).

## Usage

1.  Install the extension.
2.  Open a Foxglove layout.
3.  Add the "Path Solver" panel.
4.  Ensure topics are being published.
5.  Click "Solve Paths".
