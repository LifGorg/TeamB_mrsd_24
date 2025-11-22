import {
  Immutable,
  MessageEvent,
  PanelExtensionContext,
} from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useMemo, useRef, useState, useCallback } from "react";
import { createRoot } from "react-dom/client";
import L, { LatLngExpression, Map as LeafletMap, Polygon, Marker as LeafletMarker, Circle as LeafletCircle, Polyline as LeafletPolyline } from "leaflet";

// Minimal Leaflet CSS injection
const injectLeafletStyles = () => {
  const id = "leaflet-css-inline";
  if (document.getElementById(id)) return;
  const style = document.createElement("style");
  style.id = id;
  style.textContent = `
    .leaflet-pane,.leaflet-tile,.leaflet-marker-icon,.leaflet-marker-shadow,.leaflet-tile-container,
    .leaflet-pane > svg,.leaflet-pane > canvas,.leaflet-zoom-box,
    .leaflet-image-layer,.leaflet-layer,.leaflet-overlay-pane svg,
    .leaflet-overlay-pane canvas { position:absolute; left:0; top:0; }
    .leaflet-container { overflow:hidden; outline:0; }
    .leaflet-tile { filter: inherit; visibility: hidden; }
    .leaflet-tile-loaded { visibility: inherit; }
    .leaflet-zoom-animated { transform-origin: 0 0; }
    .leaflet-zoom-anim .leaflet-zoom-animated { transition: transform 250ms cubic-bezier(0,0,0.25,1); }
    .leaflet-zoom-anim .leaflet-tile { transition: transform 250ms cubic-bezier(0,0,0.25,1); }
    .leaflet-fade-anim .leaflet-tile { will-change: opacity; }
    .leaflet-fade-anim .leaflet-tile-loaded { transition: opacity 0.2s linear; opacity: 1; }
    .leaflet-control { pointer-events: auto; }
    .leaflet-marker-icon { display: block; }
    .leaflet-container a { color: #0078a8; }
    .leaflet-bar { box-shadow: 0 1px 5px rgba(0,0,0,0.65); border-radius:4px; }
    .leaflet-bar a, .leaflet-bar a:hover { background-color:#fff; border-bottom:1px solid #ccc; width:26px; height:26px; line-height:26px; display:block; text-align:center; text-decoration:none; }
    .leaflet-bar a:last-child { border-bottom:none; }
    .leaflet-control-zoom-in, .leaflet-control-zoom-out { font: bold 18px 'Lucida Console', Monaco, monospace; }
  `;
  document.head.appendChild(style);
};

type GeofencePoint = { lat: number; lon: number };
type HumanGPS = { lat: number; lon: number; id: string; timestamp: number };

function GeofenceHumanPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [messages, setMessages] = useState<undefined | Immutable<MessageEvent[]>>();
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [distances, setDistances] = useState<{ toTarget: number | null, toObstacles: { id: string, distance: number }[] }>({ toTarget: null, toObstacles: [] });

  // Map refs
  const mapRef = useRef<LeafletMap | null>(null);
  const geofencePolygonRef = useRef<Polygon | null>(null);
  const geofenceMarkersRef = useRef<Map<number, LeafletMarker>>(new Map());
  const missionWaypointMarkersRef = useRef<Map<number, LeafletMarker>>(new Map());  // Mission waypoints (persistent)
  const humanMarkersRef = useRef<Map<string, LeafletMarker>>(new Map());
  const obstacleMarkersRef = useRef<Map<string, LeafletCircle>>(new Map());
  const plannedPathRef = useRef<LeafletPolyline | null>(null);
  const selectedWaypointMarkerRef = useRef<LeafletMarker | null>(null);
  const [status, setStatus] = useState<string>("Initializing...");
  const [selectedWaypoint, setSelectedWaypoint] = useState<{lat: number, lon: number} | null>(null);
  const [waypointAltitude, setWaypointAltitude] = useState<number>(6.0); // User-configurable altitude
  const [geofenceMappingAltitude, setGeofenceMappingAltitude] = useState<number>(10.0); // Geofence mapping altitude
  const [surveyAltitude, setSurveyAltitude] = useState<number>(6.0); // Survey altitude
  
  // State to manage the click mode
  const [clickMode, setClickMode] = useState<"waypoint" | "obstacle">("waypoint");
  // Ref to track the latest casualty marker for highlighting
  const lastCasualtyMarkerRef = useRef<LeafletMarker | null>(null);
  
  // Global waypoint counter for unique IDs across missions
  const globalWaypointCounterRef = useRef<number>(0);

  // Topic names
  const GEOFENCE_TOPIC = "/dtc_mrsd_/mavros/geofence/fences";
  const MISSION_WAYPOINTS_TOPIC = "/dtc_mrsd_/mavros/mission/waypoints";  // Mission waypoints
  const CASUALTY_GEOLOCATED_TOPIC = "/casualty_geolocated";  // Casualty position
  const SELECTED_WAYPOINT_TOPIC = "/selected_waypoint";  // New topic for selected waypoint
  const CLICKED_OBSTACLE_TOPIC = "/clicked_obstacle"; // New topic for clicked obstacles
  const GEOFENCE_MAPPING_STATUS_TOPIC = "/geofence_mapping_status"; // Status of the mapping action
  const PLANNED_PATH_TOPIC = "/planned_path"; // The obstacle-free path from the planner
  const DRONE_GPS_TOPIC = "/dtc_mrsd_/mavros/global_position/global";  // Drone position
  const DRONE_HEADING_TOPIC = "/dtc_mrsd_/mavros/global_position/compass_hdg";  // Drone heading
  const GEOFENCE_MAPPING_ALTITUDE_TOPIC = "/geofence_mapping_altitude";  // Geofence mapping altitude
  const SURVEY_ALTITUDE_TOPIC = "/survey_altitude";  // Survey altitude

  // Store drone position and heading
  const dronePositionRef = useRef<{lat: number, lon: number} | null>(null);
  const droneHeadingRef = useRef<number | null>(null);  // Heading in degrees (0-360, 0=North)
  const droneMarkerRef = useRef<LeafletMarker | null>(null);
  const mapCenteredOnDroneRef = useRef(false);  // Track if map has been centered on drone
  
  // Throttle updates to prevent high-frequency refreshing
  const lastUpdateTimeRef = useRef<number>(0);
  const UPDATE_THROTTLE_MS = 100; // Update at most every 100ms (10 Hz)

  const subscriptions = useMemo(
    () => [
      { topic: GEOFENCE_TOPIC },
      { topic: MISSION_WAYPOINTS_TOPIC },
      { topic: CASUALTY_GEOLOCATED_TOPIC },
      { topic: DRONE_GPS_TOPIC },
      { topic: DRONE_HEADING_TOPIC },
      { topic: GEOFENCE_MAPPING_STATUS_TOPIC },
      { topic: PLANNED_PATH_TOPIC },
    ],
    []
  );

  // Advertise selected waypoint topic
  const advertisedRef = useRef(false);
  
  const ensureAdvertised = () => {
    if (!context.advertise) return;
    if (advertisedRef.current) return;
    
    try {
      context.advertise(SELECTED_WAYPOINT_TOPIC, "sensor_msgs/NavSatFix");
      context.advertise(CLICKED_OBSTACLE_TOPIC, "geometry_msgs/PoseStamped");
      context.advertise(GEOFENCE_MAPPING_ALTITUDE_TOPIC, "std_msgs/Float64");
      context.advertise(SURVEY_ALTITUDE_TOPIC, "std_msgs/Float64");
      advertisedRef.current = true;
      console.log("[GeofenceMap] ✓ Advertised:", SELECTED_WAYPOINT_TOPIC, GEOFENCE_MAPPING_ALTITUDE_TOPIC, SURVEY_ALTITUDE_TOPIC);
    } catch (error) {
      console.error("[GeofenceMap] ✗ Advertise failed:", error);
    }
  };
  
  useEffect(() => {
    ensureAdvertised();
  }, [context]);

  // Publish selected waypoint
  const publishSelectedWaypoint = useCallback((lat: number, lon: number, alt?: number) => {
    if (!context.publish) return;
    
    ensureAdvertised();
    
    // Use provided altitude or the user-configured altitude
    const altitude = alt ?? waypointAltitude;

    const msg = {
      header: {
        stamp: { sec: 0, nsec: 0 },
        frame_id: "selected_waypoint",
      },
      status: {
        status: 0,
        service: 1,
      },
      latitude: lat,
      longitude: lon,
      altitude: altitude,
      position_covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0],
      position_covariance_type: 0,
    };

    try {
      context.publish(SELECTED_WAYPOINT_TOPIC, msg);
      console.log("[GeofenceMap] ✓ Published:", lat, lon, altitude);
      setStatus(`Waypoint: ${lat.toFixed(6)}, ${lon.toFixed(6)}, Alt: ${altitude}m`);
    } catch (error) {
      console.error("[GeofenceMap] ✗ Publish failed:", error);
    }
  }, [context, SELECTED_WAYPOINT_TOPIC, waypointAltitude]);

  // Publish clicked obstacle
  const publishClickedObstacle = useCallback((lat: number, lon: number, radius: number) => {
    if (!context.publish) return;
    
    ensureAdvertised();
    
    const msg = {
      header: { stamp: { sec: 0, nsec: 0 }, frame_id: "map" },
      pose: {
        position: { x: lat, y: lon, z: 0 }, // Using x,y for lat,lon as per convention
        orientation: { x: 0, y: 0, z: 0, w: radius } // Encoding radius in w
      }
    };

    try {
      context.publish(CLICKED_OBSTACLE_TOPIC, msg);
      console.log("[GeofenceMap] ✓ Published Obstacle:", lat, lon, `Radius: ${radius}m`);
    } catch (error) {
      console.error("[GeofenceMap] ✗ Obstacle Publish failed:", error);
    }
  }, [context, CLICKED_OBSTACLE_TOPIC]);

  // Publish geofence mapping altitude
  const publishGeofenceMappingAltitude = useCallback(() => {
    if (!context.publish) return;
    
    ensureAdvertised();
    
    const msg = {
      data: geofenceMappingAltitude,
    };

    try {
      context.publish(GEOFENCE_MAPPING_ALTITUDE_TOPIC, msg);
      console.log("[GeofenceMap] ✓ Published geofence_mapping_altitude:", geofenceMappingAltitude);
      setStatus(`Published geofence mapping altitude: ${geofenceMappingAltitude}m`);
    } catch (error) {
      console.error("[GeofenceMap] ✗ Publish geofence_mapping_altitude failed:", error);
    }
  }, [context, GEOFENCE_MAPPING_ALTITUDE_TOPIC, geofenceMappingAltitude]);

  // Publish survey altitude
  const publishSurveyAltitude = useCallback(() => {
    if (!context.publish) return;
    
    ensureAdvertised();
    
    const msg = {
      data: surveyAltitude,
    };

    try {
      context.publish(SURVEY_ALTITUDE_TOPIC, msg);
      console.log("[GeofenceMap] ✓ Published survey_altitude:", surveyAltitude);
      setStatus(`Published survey altitude: ${surveyAltitude}m`);
    } catch (error) {
      console.error("[GeofenceMap] ✗ Publish survey_altitude failed:", error);
    }
  }, [context, SURVEY_ALTITUDE_TOPIC, surveyAltitude]);

  // Zoom to drone position
  const zoomToDrone = () => {
    if (!mapRef.current || !dronePositionRef.current) {
      setStatus("Drone position not available");
      return;
    }
    mapRef.current.setView([dronePositionRef.current.lat, dronePositionRef.current.lon], 18);
    setStatus(`Zoomed to drone: ${dronePositionRef.current.lat.toFixed(6)}, ${dronePositionRef.current.lon.toFixed(6)}`);
  };

  // Zoom to selected waypoint
  const zoomToWaypoint = () => {
    if (!mapRef.current || !selectedWaypoint) {
      setStatus("No waypoint selected");
      return;
    }
    mapRef.current.setView([selectedWaypoint.lat, selectedWaypoint.lon], 18);
    setStatus(`Zoomed to waypoint: ${selectedWaypoint.lat.toFixed(6)}, ${selectedWaypoint.lon.toFixed(6)}`);
  };

  // Setup render handler and subscriptions
  useLayoutEffect(() => {
    injectLeafletStyles();

    context.onRender = (renderState, done) => {
      ensureAdvertised();
      setRenderDone(() => done);
      setMessages(renderState.currentFrame);
    };

    context.watch("currentFrame");
    context.subscribe(subscriptions);

    return () => {
      if (mapRef.current) {
        mapRef.current.remove();
        mapRef.current = null;
      }
    };
  }, [context, subscriptions]);

  // Initialize map
  useEffect(() => {
    if (mapRef.current) return;

    const el = document.getElementById("map-container");
    if (!el) return;

    // Create map centered on default location with high zoom for precise small area flight
    const map = L.map(el, {
      center: [40.414, -79.947],
      zoom: 19,  // High zoom level for precise small area operations
      zoomControl: true,
    });

    // Add Satellite tile layer (Esri World Imagery)
    L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", {
      attribution: 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community',
      maxZoom: 19,
    }).addTo(map);

    // Add click handler to set waypoint or obstacle
    map.on("click", (e) => {
      const { lat, lng } = e.latlng;
      
      if (clickMode === "waypoint") {
        // --- THIS IS THE EXISTING WAYPOINT LOGIC ---
        if (selectedWaypointMarkerRef.current) {
          map.removeLayer(selectedWaypointMarkerRef.current);
        }
        const icon = L.divIcon({
          className: "waypoint-marker",
          html: `<div style="background-color: #0000ff; width: 14px; height: 14px; border-radius: 50%; border: 2px solid white; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 9px; box-shadow: 0 1px 4px rgba(0,0,0,0.5);">W</div>`,
          iconSize: [14, 14],
          iconAnchor: [7, 7],
        });
        const marker = L.marker([lat, lng], { icon }).addTo(map)
          .bindPopup(
            `<b>Selected Waypoint</b><br>Lat: ${lat.toFixed(6)}<br>Lon: ${lng.toFixed(6)}<br><br><i>Click "Navigate to Waypoint" in Behavior Tree Controller to go here</i>`
          )
          .openPopup();
        selectedWaypointMarkerRef.current = marker;
        setSelectedWaypoint({ lat, lon: lng });

      } else if (clickMode === "obstacle") {
        // --- THIS IS THE NEW OBSTACLE LOGIC ---
        const obstacleRadiusMeters = 1.5;
        
        // Draw a circle on the map with a real-world radius
        const circle = L.circle([lat, lng], {
            radius: obstacleRadiusMeters, // Radius in meters
            color: '#ff0000',             // Red outline
            fillColor: '#ff0000',         // Red fill
            fillOpacity: 0.3,              // Slightly transparent
        }).addTo(map);
        
        circle.bindPopup(`<b>Obstacle</b><br>Radius: ${obstacleRadiusMeters}m`);
        
        const obstacleId = `obstacle-${Date.now()}`;
        obstacleMarkersRef.current.set(obstacleId, circle);
        
        // Publish the obstacle position and radius
        publishClickedObstacle(lat, lng, obstacleRadiusMeters);
      }
    });

    mapRef.current = map;
    setStatus("Map loaded - Click to set waypoint");

    // Handle resize
    const resizeObserver = new ResizeObserver(() => {
      map.invalidateSize();
    });
    resizeObserver.observe(el);

    return () => {
      resizeObserver.disconnect();
      map.remove();
      mapRef.current = null;
    };
  }, []); // Remove publishSelectedWaypoint dependency to prevent map reset

  // Publish selected waypoint when it changes or altitude changes
  useEffect(() => {
    if (selectedWaypoint) {
      publishSelectedWaypoint(selectedWaypoint.lat, selectedWaypoint.lon);
    }
  }, [selectedWaypoint, waypointAltitude, publishSelectedWaypoint]);

  // Process messages and update visualization
  useEffect(() => {
    if (!messages || !mapRef.current) {
      renderDone?.();
      return;
    }

    // Throttle updates to prevent high-frequency DOM manipulation
    // But allow high-priority topics (mission waypoints, geofence, casualty) through immediately
    const now = Date.now();
    const hasHighPriorityTopic = messages.some(
      (ev) =>
        ev.topic === MISSION_WAYPOINTS_TOPIC ||
        ev.topic === GEOFENCE_TOPIC ||
        ev.topic === CASUALTY_GEOLOCATED_TOPIC
    );
    
    if (!hasHighPriorityTopic && now - lastUpdateTimeRef.current < UPDATE_THROTTLE_MS) {
      renderDone?.();
      return;
    }
    lastUpdateTimeRef.current = now;

    const map = mapRef.current;

    // Process each message
    for (const messageEvent of messages) {
      const topic = messageEvent.topic;
      const message = messageEvent.message;

      try {
        // --- ADD THIS BLOCK: Handle planned path visualization ---
        if (topic === PLANNED_PATH_TOPIC) {
          const pathMsg = message as any; // Type is nav_msgs/Path
          
          // Remove old path
          if (plannedPathRef.current) {
            map.removeLayer(plannedPathRef.current);
          }

          if (pathMsg.poses && Array.isArray(pathMsg.poses) && pathMsg.poses.length > 1) {
            const latlngs = pathMsg.poses.map((poseStamped: any) => 
              [poseStamped.pose.position.y, poseStamped.pose.position.x] // Assuming planner uses x for lon, y for lat
            );

            const path = L.polyline(latlngs, {
              color: '#00BFFF', // Deep sky blue
              weight: 5,
              opacity: 0.8,
            }).addTo(map);

            plannedPathRef.current = path;
            setStatus(`Visualizing planned path with ${latlngs.length} points.`);
          }
        }

        // --- ADD THIS BLOCK: Handle mapping completion to highlight the last casualty ---
        if (topic === GEOFENCE_MAPPING_STATUS_TOPIC) {
          const statusMsg = message as any; // Type is behavior_tree_msgs/Status
          const SUCCESS_STATUS = 1; // In behavior_tree_msgs/Status, 1 usually means SUCCESS

          if (statusMsg.status === SUCCESS_STATUS) {
            if (lastCasualtyMarkerRef.current) {
              const highlightIcon = L.divIcon({
                className: "custom-marker-final-find",
                html: `<div style="background-color: #ffff00; color: black; width: 24px; height: 24px; border-radius: 50%; border: 3px solid #ff0000; display: flex; align-items: center; justify-content: center; font-weight: bold; font-size: 14px; box-shadow: 0 0 10px #ffff00;">C</div>`,
                iconSize: [24, 24],
                iconAnchor: [12, 12],
              });
              lastCasualtyMarkerRef.current.setIcon(highlightIcon);
              lastCasualtyMarkerRef.current.setPopupContent(`<b>Casualty (Final Find)</b><br>${lastCasualtyMarkerRef.current.getPopup()?.getContent() || ''}`);
              setStatus("Mapping complete. Last casualty highlighted.");
            }
          }
        }


        // Process geofence topic
        if (topic === GEOFENCE_TOPIC) {
          const waypointList = message as any;
          if (waypointList.waypoints && Array.isArray(waypointList.waypoints)) {
            const points: GeofencePoint[] = [];
            for (const wp of waypointList.waypoints) {
              if (wp.x_lat !== undefined && wp.y_long !== undefined) {
                points.push({ lat: wp.x_lat, lon: wp.y_long });
              }
            }

            if (points.length >= 3) {
              // Remove old polygon and markers
              if (geofencePolygonRef.current) {
                map.removeLayer(geofencePolygonRef.current);
              }
              geofenceMarkersRef.current.forEach((marker: LeafletMarker) => {
                map.removeLayer(marker);
              });
              geofenceMarkersRef.current.clear();

              // Create new polygon
              const latlngs: LatLngExpression[] = points.map((p) => [p.lat, p.lon]);
              // Close the polygon
              if (latlngs.length > 0 && latlngs[0] !== undefined) {
                latlngs.push(latlngs[0]);
              }

              const polygon = L.polygon(latlngs, {
                color: "#ff0000",
                fillColor: "#ff0000",
                fillOpacity: 0.2,
                weight: 3,
              }).addTo(map);

              geofencePolygonRef.current = polygon;

              // Add markers for each geofence point
              points.forEach((point, index) => {
                const marker = L.marker([point.lat, point.lon], {
                  icon: L.divIcon({
                    className: "geofence-marker",
                    html: `<div style="background-color: #ff0000; width: 12px; height: 12px; border-radius: 50%; border: 2px solid white; box-shadow: 0 0 0 2px #ff0000;"></div>`,
                    iconSize: [12, 12],
                    iconAnchor: [6, 6],
                  }),
                })
                  .addTo(map)
                  .bindPopup(`<b>Geofence Point ${index + 1}</b><br>Lat: ${point.lat.toFixed(6)}<br>Lon: ${point.lon.toFixed(6)}`);
                geofenceMarkersRef.current.set(index, marker);
              });

              map.fitBounds(polygon.getBounds(), { padding: [50, 50] });

              setStatus(`Geofence: ${points.length} points`);
            } else if (points.length > 0) {
              // If less than 3 points, just show markers
              geofenceMarkersRef.current.forEach((marker: LeafletMarker) => {
                map.removeLayer(marker);
              });
              geofenceMarkersRef.current.clear();

              points.forEach((point, index) => {
                const marker = L.marker([point.lat, point.lon], {
                  icon: L.divIcon({
                    className: "geofence-marker",
                    html: `<div style="background-color: #ff0000; width: 12px; height: 12px; border-radius: 50%; border: 2px solid white; box-shadow: 0 0 0 2px #ff0000;"></div>`,
                    iconSize: [12, 12],
                    iconAnchor: [6, 6],
                  }),
                })
                  .addTo(map)
                  .bindPopup(`<b>Geofence Point ${index + 1}</b><br>Lat: ${point.lat.toFixed(6)}<br>Lon: ${point.lon.toFixed(6)}`);
                geofenceMarkersRef.current.set(index, marker);
              });

              setStatus(`Geofence: ${points.length} points (need 3+ for polygon)`);
            }
          }
        }

        // Process mission waypoints (persistent - never removed)
        if (topic === MISSION_WAYPOINTS_TOPIC) {
          console.log("[GeofenceMap] 📥 Received mission waypoints message");
          const waypointList = message as any;
          console.log("[GeofenceMap] 🔍 Full message:", JSON.stringify(waypointList, null, 2));
          console.log("[GeofenceMap] 🔍 Message structure:", {
            hasWaypoints: !!waypointList.waypoints,
            isArray: Array.isArray(waypointList.waypoints),
            count: waypointList.waypoints?.length,
            currentSeq: waypointList.current_seq
          });

          if (waypointList.waypoints && Array.isArray(waypointList.waypoints)) {
            console.log("[GeofenceMap] 🔍 Processing", waypointList.waypoints.length, "waypoints");
            
            // Extract valid waypoints with GPS coordinates
            const waypoints = waypointList.waypoints.filter((wp: any) => {
              const valid = wp.x_lat !== undefined && 
                     wp.y_long !== undefined && 
                     wp.x_lat !== 0 && 
                     wp.y_long !== 0;
              if (!valid) {
                console.log("[GeofenceMap] ⚠️ Filtered out invalid waypoint:", wp);
              }
              return valid;
            });

            console.log("[GeofenceMap] ✅ Valid waypoints:", waypoints.length);
            console.log("[GeofenceMap] 🗺️ Map exists:", !!map);
            console.log("[GeofenceMap] 📍 Current markers count:", missionWaypointMarkersRef.current.size);

            // Always create new waypoint markers with global IDs (never update existing ones)
            // This ensures all waypoints from all missions are preserved on the map
            waypoints.forEach((wp: any, index: number) => {
              const lat = wp.x_lat;
              const lon = wp.y_long;
              const frame = wp.frame || 0;
              const command = wp.command || 0;
              
              // Assign global unique ID
              const globalId = globalWaypointCounterRef.current;
              globalWaypointCounterRef.current += 1;
              
              console.log(`[GeofenceMap] ➕ Creating NEW waypoint #${globalId} (mission index: ${index}) at [${lat}, ${lon}]`);
              
              // Create new persistent waypoint marker (purple diamond) with global ID
              const icon = L.divIcon({
                className: "mission-waypoint-marker",
                html: `<div style="background-color: #9c27b0; width: 16px; height: 16px; transform: rotate(45deg); border: 2px solid white; display: flex; align-items: center; justify-content: center; box-shadow: 0 1px 4px rgba(0,0,0,0.5);"><span style="transform: rotate(-45deg); color: white; font-weight: bold; font-size: 9px;">${globalId}</span></div>`,
                iconSize: [16, 16],
                iconAnchor: [8, 8],
              });

              try {
                console.log(`[GeofenceMap] 🔨 Creating marker for global waypoint #${globalId}...`);
                const marker = L.marker([lat, lon], { icon });
                console.log(`[GeofenceMap] 🔨 Adding marker to map...`);
                marker.addTo(map);
                console.log(`[GeofenceMap] 🔨 Binding popup...`);
                marker.bindPopup(
                  `<b>Waypoint #${globalId}</b><br>Mission Index: ${index}<br>Lat: ${lat.toFixed(6)}<br>Lon: ${lon.toFixed(6)}<br>Frame: ${frame}<br>Command: ${command}`
                );

                console.log(`[GeofenceMap] 🔨 Storing marker reference...`);
                missionWaypointMarkersRef.current.set(globalId, marker);
                console.log(`[GeofenceMap] ✅ Successfully added global waypoint #${globalId}. Total markers:`, missionWaypointMarkersRef.current.size);
              } catch (error) {
                console.error(`[GeofenceMap] ❌ Failed to create marker for global waypoint #${globalId}:`, error);
              }
            });

            console.log(`[GeofenceMap] ✅ Mission waypoints processing complete. Total markers on map:`, missionWaypointMarkersRef.current.size);
            setStatus(`Waypoints: ${missionWaypointMarkersRef.current.size} total (${waypoints.length} in this mission)`);
          } else {
            console.log("[GeofenceMap] ⚠️ Invalid waypoints data structure");
          }
        }

        // Process drone GPS position
        if (topic === DRONE_GPS_TOPIC) {
          const navSatFix = message as any;
          if (
            navSatFix.latitude !== undefined &&
            navSatFix.longitude !== undefined &&
            navSatFix.latitude !== 0 &&
            navSatFix.longitude !== 0
          ) {
            const newLat = navSatFix.latitude;
            const newLon = navSatFix.longitude;

            // Center map on drone position on first GPS fix
            if (!mapCenteredOnDroneRef.current && mapRef.current) {
              mapRef.current.setView([newLat, newLon], 19);
              mapCenteredOnDroneRef.current = true;
              console.log("[GeofenceMap] Map centered on drone:", newLat, newLon);
              setStatus(`Map centered on drone: ${newLat.toFixed(6)}, ${newLon.toFixed(6)}`);
            }

            // Only update if position changed significantly (> 1cm)
            const shouldUpdate = !dronePositionRef.current ||
              Math.abs(dronePositionRef.current.lat - newLat) > 0.0000001 ||
              Math.abs(dronePositionRef.current.lon - newLon) > 0.0000001;

            if (shouldUpdate) {
              dronePositionRef.current = {
                lat: newLat,
                lon: newLon,
              };

              // Update or create drone marker
              const heading = droneHeadingRef.current ?? 0;
              const rotation = heading; // Heading is already in degrees, 0=North
              
              if (droneMarkerRef.current) {
                droneMarkerRef.current.setLatLng([newLat, newLon]);
                // Update icon with current heading
                const droneIcon = L.divIcon({
                  className: "drone-marker",
                  html: `<div style="background-color: #ff6600; width: 24px; height: 24px; border-radius: 50%; border: 2px solid white; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 14px; box-shadow: 0 2px 6px rgba(0,0,0,0.5); transform: rotate(${rotation}deg);">▲</div>`,
                  iconSize: [24, 24],
                  iconAnchor: [12, 12],
                });
                droneMarkerRef.current.setIcon(droneIcon);
                // Update popup content
                droneMarkerRef.current.setPopupContent(
                  `<b>Drone Position</b><br>Lat: ${newLat.toFixed(6)}<br>Lon: ${newLon.toFixed(6)}<br>Heading: ${heading.toFixed(1)}°`
                );
              } else {
                const droneIcon = L.divIcon({
                  className: "drone-marker",
                  html: `<div style="background-color: #ff6600; width: 24px; height: 24px; border-radius: 50%; border: 2px solid white; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 14px; box-shadow: 0 2px 6px rgba(0,0,0,0.5); transform: rotate(${rotation}deg);">▲</div>`,
                  iconSize: [24, 24],
                  iconAnchor: [12, 12],
                });

                const marker = L.marker([newLat, newLon], { icon: droneIcon })
                  .addTo(map)
                  .bindPopup(
                    `<b>Drone Position</b><br>Lat: ${newLat.toFixed(6)}<br>Lon: ${newLon.toFixed(6)}<br>Heading: ${heading.toFixed(1)}°`
                  );

                droneMarkerRef.current = marker;
              }
            }
          }
        }

        // Process drone heading
        if (topic === DRONE_HEADING_TOPIC) {
          const headingMsg = message as any;
          // The message might be std_msgs/Float64 with 'data' field, or might be a direct number
          let heading: number | null = null;
          
          if (typeof headingMsg === 'number') {
            heading = headingMsg;
          } else if (headingMsg.data !== undefined) {
            heading = headingMsg.data;
          }
          
          if (heading !== null && !isNaN(heading)) {
            // Normalize heading to 0-360 range
            heading = ((heading % 360) + 360) % 360;
            
            const headingChanged = droneHeadingRef.current === null || 
              Math.abs(droneHeadingRef.current - heading) > 0.5; // Update if changed by >0.5 degrees
            
            if (headingChanged) {
              droneHeadingRef.current = heading;
              
              // Update drone marker if it exists and has position
              if (droneMarkerRef.current && dronePositionRef.current) {
                const rotation = heading;
                const droneIcon = L.divIcon({
                  className: "drone-marker",
                  html: `<div style="background-color: #ff6600; width: 24px; height: 24px; border-radius: 50%; border: 2px solid white; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 14px; box-shadow: 0 2px 6px rgba(0,0,0,0.5); transform: rotate(${rotation}deg);">▲</div>`,
                  iconSize: [24, 24],
                  iconAnchor: [12, 12],
                });
                droneMarkerRef.current.setIcon(droneIcon);
                droneMarkerRef.current.setPopupContent(
                  `<b>Drone Position</b><br>Lat: ${dronePositionRef.current.lat.toFixed(6)}<br>Lon: ${dronePositionRef.current.lon.toFixed(6)}<br>Heading: ${heading.toFixed(1)}°`
                );
              }
            }
          }
        }

        // Process casualty geolocated topic (sensor_msgs/NavSatFix)
        if (topic === CASUALTY_GEOLOCATED_TOPIC) {
          const navSatFix = message as any;
          if (
            navSatFix.latitude !== undefined &&
            navSatFix.longitude !== undefined &&
            navSatFix.latitude !== 0 &&
            navSatFix.longitude !== 0
          ) {
            const humanGPS: HumanGPS = {
              lat: navSatFix.latitude,
              lon: navSatFix.longitude,
              id: `${topic}-${Date.now()}`,
              timestamp: Date.now(),
            };

            // Use unique timestamp-based ID to keep all history
            const markerId = `${topic}-${Date.now()}-${Math.random()}`;

            // Always create new marker (never update existing ones)
            const icon = L.divIcon({
              className: "custom-marker",
              html: `<div style="background-color: #ff0000; width: 20px; height: 20px; border-radius: 50%; border: 2px solid white; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 12px;">C</div>`,
              iconSize: [20, 20],
              iconAnchor: [10, 10],
            });

            const marker = L.marker([humanGPS.lat, humanGPS.lon], { icon })
              .addTo(map)
              .bindPopup(`<b>Casualty</b><br>Lat: ${humanGPS.lat.toFixed(6)}<br>Lon: ${humanGPS.lon.toFixed(6)}`);

            humanMarkersRef.current.set(markerId, marker);
            // Track the latest marker
            lastCasualtyMarkerRef.current = marker;

            // Update status
            const casualtyCount = Array.from(humanMarkersRef.current.keys())
              .filter(id => id.startsWith(CASUALTY_GEOLOCATED_TOPIC)).length;
            setStatus(`Casualties: ${casualtyCount} total detected`);
          }
        }
      } catch (error) {
        console.error(`Error processing message from ${topic}:`, error);
        setStatus(`Error: ${error}`);
      }
    }

    // --- Distance Calculation Logic ---
    if (dronePositionRef.current) {
      const droneLatLng = L.latLng(dronePositionRef.current.lat, dronePositionRef.current.lon);
      
      // Distance to target casualty
      let distToTarget: number | null = null;
      if (lastCasualtyMarkerRef.current) {
        const targetLatLng = lastCasualtyMarkerRef.current.getLatLng();
        distToTarget = droneLatLng.distanceTo(targetLatLng);
      }

      // Distances to obstacles
      const distsToObs: { id: string, distance: number }[] = [];
      obstacleMarkersRef.current.forEach((circle, id) => {
        const obstacleLatLng = circle.getLatLng();
        const distToCenter = droneLatLng.distanceTo(obstacleLatLng);
        const distToEdge = Math.max(0, distToCenter - circle.getRadius());
        distsToObs.push({ id, distance: distToEdge });
      });

      setDistances({
        toTarget: distToTarget,
        toObstacles: distsToObs,
      });
    }


    renderDone?.();
  }, [messages, renderDone]);

  return (
    <div style={{ width: "100%", height: "100%", display: "flex", flexDirection: "column" }}>
      <div style={{ padding: "8px", backgroundColor: "#f0f0f0", borderBottom: "1px solid #ccc" }}>
        <div style={{ display: "flex", alignItems: "center", justifyContent: "space-between", marginBottom: "4px" }}>
          <div style={{ fontSize: "12px", fontWeight: "bold" }}>
            Geofence & Human GPS Visualizer + Waypoint Selector
          </div>
          <div style={{ display: "flex", gap: "4px" }}>
            <button
              onClick={zoomToDrone}
              style={{
                padding: "4px 8px",
                fontSize: "11px",
                borderRadius: 4,
                border: "1px solid #ff6600",
                background: "#fff",
                color: "#ff6600",
                cursor: "pointer",
                fontWeight: "600",
              }}
              title="Zoom to drone position"
            >
              ✈ Drone
            </button>
            <button
              onClick={zoomToWaypoint}
              disabled={!selectedWaypoint}
              style={{
                padding: "4px 8px",
                fontSize: "11px",
                borderRadius: 4,
                border: "1px solid #0000ff",
                background: selectedWaypoint ? "#fff" : "#eee",
                color: selectedWaypoint ? "#0000ff" : "#999",
                cursor: selectedWaypoint ? "pointer" : "not-allowed",
                fontWeight: "600",
              }}
              title="Zoom to selected waypoint"
            >
              📍 Waypoint
            </button>
          </div>
        </div>
        <div style={{ fontSize: "11px", color: "#666" }}>{status}</div>
        <div style={{ 
          fontSize: "11px", 
          color: "#333", 
          marginTop: "6px", 
          display: "flex", 
          flexWrap: "wrap", 
          gap: "8px 16px" 
        }}>
          {distances.toTarget != null && (
            <div style={{ fontWeight: "600" }}>
              <span style={{ color: "#007bff" }}>▶ Target: </span>
              {distances.toTarget.toFixed(1)}m
            </div>
          )}
          {distances.toObstacles.map(({ id, distance }) => (
            <div key={id} style={{ fontWeight: "600" }}>
              <span style={{ color: "#dc3545" }}>■ Obstacle: </span>
              {distance.toFixed(1)}m
            </div>
          ))}
        </div>
        {selectedWaypoint && (
          <div style={{ fontSize: "11px", color: "#0000ff", marginTop: "4px", fontWeight: "600" }}>
            📍 Selected Waypoint: {selectedWaypoint.lat.toFixed(6)}, {selectedWaypoint.lon.toFixed(6)} 
            <span style={{ marginLeft: "8px", fontSize: "10px", fontWeight: "normal", color: "#666" }}>
              → Now click "Navigate to Waypoint" in Behavior Tree Controller
            </span>
          </div>
        )}
        <div style={{ 
          display: "flex", 
          alignItems: "center", 
          gap: "8px", 
          marginTop: "6px",
          fontSize: "11px" 
        }}>
          <label style={{ fontWeight: "600", color: "#333" }}>
            Waypoint Altitude (m):
          </label>
          <input
            type="number"
            value={waypointAltitude}
            onChange={(e) => setWaypointAltitude(parseFloat(e.target.value) || 0)}
            min="0"
            max="200"
            step="0.5"
            style={{
              width: "70px",
              padding: "3px 6px",
              fontSize: "11px",
              borderRadius: "3px",
              border: "1px solid #ccc",
              textAlign: "right"
            }}
          />
          <span style={{ fontSize: "10px", color: "#666" }}>
            (Set altitude for selected waypoints)
          </span>
        </div>
        <div style={{ 
          display: "flex", 
          alignItems: "center", 
          gap: "8px", 
          marginTop: "6px",
          fontSize: "11px" 
        }}>
          <label style={{ fontWeight: "600", color: "#333" }}>
            Geofence Mapping Alt (m):
          </label>
          <input
            type="number"
            value={geofenceMappingAltitude}
            onChange={(e) => setGeofenceMappingAltitude(parseFloat(e.target.value) || 0)}
            min="0"
            max="200"
            step="0.5"
            style={{
              width: "70px",
              padding: "3px 6px",
              fontSize: "11px",
              borderRadius: "3px",
              border: "1px solid #ccc",
              textAlign: "right"
            }}
          />
          <button
            onClick={publishGeofenceMappingAltitude}
            style={{
              padding: "3px 8px",
              fontSize: "11px",
              borderRadius: "3px",
              border: "1px solid #4CAF50",
              background: "#4CAF50",
              color: "white",
              cursor: "pointer",
              fontWeight: "600",
            }}
            title="Publish geofence mapping altitude"
          >
            Send
          </button>
          <span style={{ fontSize: "10px", color: "#666" }}>
            → /geofence_mapping_altitude
          </span>
        </div>
        <div style={{ 
          display: "flex", 
          alignItems: "center", 
          gap: "8px", 
          marginTop: "6px",
          fontSize: "11px" 
        }}>
          <label style={{ fontWeight: "600", color: "#333" }}>
            Survey Altitude (m):
          </label>
          <input
            type="number"
            value={surveyAltitude}
            onChange={(e) => setSurveyAltitude(parseFloat(e.target.value) || 0)}
            min="0"
            max="200"
            step="0.5"
            style={{
              width: "70px",
              padding: "3px 6px",
              fontSize: "11px",
              borderRadius: "3px",
              border: "1px solid #ccc",
              textAlign: "right"
            }}
          />
          <button
            onClick={publishSurveyAltitude}
            style={{
              padding: "3px 8px",
              fontSize: "11px",
              borderRadius: "3px",
              border: "1px solid #4CAF50",
              background: "#4CAF50",
              color: "white",
              cursor: "pointer",
              fontWeight: "600",
            }}
            title="Publish survey altitude"
          >
            Send
          </button>
          <span style={{ fontSize: "10px", color: "#666" }}>
            → /survey_altitude
          </span>
        </div>
        {/* ADD THIS: Mode switcher UI */}
        <div style={{ marginTop: "8px", display: "flex", gap: "8px", alignItems: "center" }}>
          <span style={{ fontSize: "11px", fontWeight: "600" }}>Click Mode:</span>
          <button
            onClick={() => setClickMode("waypoint")}
            style={{
              padding: "4px 8px",
              fontSize: "11px",
              border: clickMode === "waypoint" ? "2px solid #0000ff" : "1px solid #ccc",
              background: clickMode === "waypoint" ? "#e0e0ff" : "#fff",
            }}
          >
            Waypoint
          </button>
          <button
            onClick={() => setClickMode("obstacle")}
            style={{
              padding: "4px 8px",
              fontSize: "11px",
              border: clickMode === "obstacle" ? "2px solid #ff0000" : "1px solid #ccc",
              background: clickMode === "obstacle" ? "#ffe0e0" : "#fff",
            }}
          >
            Obstacle
          </button>
        </div>
        <div style={{ fontSize: "10px", color: "#999", marginTop: "4px" }}>
          Click map to select waypoint • Topics: {GEOFENCE_TOPIC}, {SELECTED_WAYPOINT_TOPIC}
        </div>
      </div>
      <div
        id="map-container"
        style={{ flex: 1, width: "100%", height: "100%", position: "relative" }}
      />
    </div>
  );
}

export function initGeofenceHumanPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<GeofenceHumanPanel context={context} />);
  return () => {
    root.unmount();
  };
}

