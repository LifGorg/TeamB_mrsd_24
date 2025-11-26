import { Immutable, MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import L, { Map as LeafletMap, Marker as LeafletMarker, Polyline as LeafletPolyline } from "leaflet";

// Minimal Leaflet CSS injection
const injectLeafletStyles = () => {
  const id = "leaflet-css-inline";
  if (document.getElementById(id)) return;
  const style = document.createElement("style");
  style.id = id;
  style.textContent = `
    .leaflet-container {
      position: relative;
      overflow: hidden;
      outline: 0;
      width: 100%;
      height: 100%;
    }

    .leaflet-pane,
    .leaflet-tile,
    .leaflet-marker-icon,
    .leaflet-marker-shadow,
    .leaflet-tile-container,
    .leaflet-pane > svg,
    .leaflet-pane > canvas,
    .leaflet-zoom-box,
    .leaflet-image-layer,
    .leaflet-layer,
    .leaflet-overlay-pane svg,
    .leaflet-overlay-pane canvas {
      position: absolute;
      left: 0;
      top: 0;
    }

    .leaflet-map-pane      { z-index: 100; }
    .leaflet-tile-pane     { z-index: 200; }
    .leaflet-overlay-pane  { z-index: 400; }
    .leaflet-shadow-pane   { z-index: 500; }
    .leaflet-marker-pane   { z-index: 600; }
    .leaflet-tooltip-pane  { z-index: 650; }
    .leaflet-popup-pane    { z-index: 700; }

    .leaflet-tile { visibility: hidden; }
    .leaflet-tile-loaded { visibility: inherit; }

    .leaflet-zoom-animated { transform-origin: 0 0; }
    .leaflet-zoom-anim .leaflet-zoom-animated {
      transition: transform 250ms cubic-bezier(0,0,0.25,1);
    }
    .leaflet-zoom-anim .leaflet-tile {
      transition: transform 250ms cubic-bezier(0,0,0.25,1);
    }

    .leaflet-fade-anim .leaflet-tile { will-change: opacity; }
    .leaflet-fade-anim .leaflet-tile-loaded {
      transition: opacity 0.2s linear;
      opacity: 1;
    }

    .leaflet-control { pointer-events: auto; }
    .leaflet-marker-icon { display: block; }
    .leaflet-container a { color: #0078a8; }
    .leaflet-bar {
      box-shadow: 0 1px 5px rgba(0,0,0,0.65);
      border-radius:4px;
    }
    .leaflet-bar a, .leaflet-bar a:hover {
      background-color:#fff;
      border-bottom:1px solid #ccc;
      width:26px;
      height:26px;
      line-height:26px;
      display:block;
      text-align:center;
      text-decoration:none;
    }
    .leaflet-bar a:last-child { border-bottom:none; }
    .leaflet-control-zoom-in, .leaflet-control-zoom-out {
      font: bold 18px 'Lucida Console', Monaco, monospace;
    }
  `;
  document.head.appendChild(style);
};

type HumanGPS = { lat: number; lon: number; id: string; timestamp: number };
type Point = { lat: number; lon: number };

const PATH_PANE_NAME = "pathSolverPathPane";

function PathSolverPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [messages, setMessages] = useState<undefined | Immutable<MessageEvent[]>>();
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [status, setStatus] = useState<string>("Waiting for data...");
  const [humanCount, setHumanCount] = useState<number>(0);
  const [hasDronePosition, setHasDronePosition] = useState<boolean>(false);
  const [hasPath, setHasPath] = useState<boolean>(false);

  const mapRef = useRef<LeafletMap | null>(null);
  const mapContainerRef = useRef<HTMLDivElement | null>(null);
  const humanMarkersRef = useRef<Map<string, LeafletMarker>>(new Map());
  const droneMarkerRef = useRef<LeafletMarker | null>(null);
  const startPointMarkerRef = useRef<LeafletMarker | null>(null);
  const pathPolylineRef = useRef<LeafletPolyline | null>(null);
  const pathLayerGroupRef = useRef<L.LayerGroup | null>(null);

  const dronePositionRef = useRef<Point | null>(null);
  const initialDronePositionRef = useRef<Point | null>(null); // Store initial drone position for path solving
  const humansRef = useRef<Map<string, HumanGPS>>(new Map());

  const CASUALTY_GEOLOCATED_TOPIC = "/casualty_geolocated";
  const DRONE_GPS_TOPIC = "/dtc_mrsd_/mavros/global_position/global";

  const subscriptions = useMemo(() => [
    { topic: CASUALTY_GEOLOCATED_TOPIC },
    { topic: DRONE_GPS_TOPIC },
  ], []);

  useLayoutEffect(() => {
    context.watch("currentFrame");
    context.subscribe(subscriptions);
  }, [context, subscriptions]);

  useEffect(() => {
    const onRender = (renderState: any, done: () => void) => {
      setRenderDone(() => done);
      if (renderState.currentFrame) {
        if (renderState.currentFrame.length > 0) {
          setMessages(renderState.currentFrame);
        } else {
          // Still call done even if no messages
          done();
        }
      } else {
        done();
      }
    };
    context.onRender = onRender;
    return () => { context.onRender = undefined; };
  }, [context]);

  useEffect(() => {
    injectLeafletStyles();
  }, []);

  useEffect(() => {
    if (!mapContainerRef.current || mapRef.current) {
      return;
    }

    let resizeObserver: ResizeObserver | undefined;

    try {
      const map = L.map(mapContainerRef.current).setView([40.4406, -79.9959], 13); // Default to Pittsburgh
      L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
        attribution: "&copy; OpenStreetMap contributors",
        maxZoom: 19,
      }).addTo(map);

      const existingPane = map.getPane(PATH_PANE_NAME);
      const pathPane = existingPane ?? map.createPane(PATH_PANE_NAME);
      pathPane.style.zIndex = "650";
      pathPane.style.pointerEvents = "none";

      const pathLayerGroup = L.layerGroup();
      pathLayerGroup.addTo(map);
      pathLayerGroupRef.current = pathLayerGroup;

      mapRef.current = map;

      setStatus("Map initialized - waiting for drone position");
      console.log("[PathSolver] Map initialized successfully with path layer group");

      const containerEl = mapContainerRef.current;
      if (containerEl) {
        resizeObserver = new ResizeObserver(() => {
          map.invalidateSize();
        });
        resizeObserver.observe(containerEl);
      }

      setTimeout(() => {
        map.invalidateSize();
      }, 100);
    } catch (error) {
      console.error("Failed to initialize map:", error);
      setStatus("❌ Map initialization failed");
    }

    return () => {
      resizeObserver?.disconnect();
      if (mapRef.current) {
        mapRef.current.remove();
      }
      mapRef.current = null;
      pathLayerGroupRef.current = null;
      pathPolylineRef.current = null;
      setHasPath(false);
    };
  }, []);

  useEffect(() => {
    if (messages) {
      // Check if path still exists before processing messages
      if (pathPolylineRef.current && pathLayerGroupRef.current) {
        const layerExists = pathLayerGroupRef.current.hasLayer(pathPolylineRef.current);
        if (!layerExists) {
          console.warn("[PathSolver] ⚠️ Path lost from layer group!");
        }
      }
      
      messages.forEach((msg) => {
        if (msg.topic === DRONE_GPS_TOPIC) {
          const navSatFix = msg.message as any;
          const lat = navSatFix.latitude;
          const lon = navSatFix.longitude;
          if (lat !== undefined && lon !== undefined && lat !== 0 && lon !== 0) {
            dronePositionRef.current = { lat, lon };
            
            // Store initial drone position on first GPS fix
            if (!initialDronePositionRef.current) {
              initialDronePositionRef.current = { lat, lon };
              console.log("[PathSolver] Initial drone position saved:", lat, lon);
              
              // Add a marker for the start point
              if (mapRef.current && !startPointMarkerRef.current) {
                const startIcon = L.divIcon({
                  className: "start-marker",
                  html: `<div style="background-color: #00ff00; width: 28px; height: 28px; border-radius: 50%; border: 3px solid white; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 16px; box-shadow: 0 2px 8px rgba(0,0,0,0.6);">S</div>`,
                  iconSize: [28, 28],
                  iconAnchor: [14, 14],
                });
                startPointMarkerRef.current = L.marker([lat, lon], { icon: startIcon })
                  .addTo(mapRef.current)
                  .bindPopup(`<b>Start Point</b><br>Lat: ${lat.toFixed(6)}<br>Lon: ${lon.toFixed(6)}<br><i>Path planning will start from here</i>`);
              }
            }
            
            setHasDronePosition(true);
            if (mapRef.current) {
              if (!droneMarkerRef.current) {
                const droneIcon = L.divIcon({
                  className: "drone-marker",
                  html: `<div style="background-color: #ff6600; width: 24px; height: 24px; border-radius: 50%; border: 2px solid white; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 14px; box-shadow: 0 2px 6px rgba(0,0,0,0.5);">🚁</div>`,
                  iconSize: [24, 24],
                  iconAnchor: [12, 12],
                });
                droneMarkerRef.current = L.marker([lat, lon], { icon: droneIcon })
                  .addTo(mapRef.current)
                  .bindPopup(`<b>Drone</b><br>Lat: ${lat.toFixed(6)}<br>Lon: ${lon.toFixed(6)}`);
                mapRef.current.setView([lat, lon], 18);
                setStatus("Drone position received - waiting for casualties");
                console.log("[PathSolver] Drone GPS received:", lat, lon);
              } else {
                droneMarkerRef.current.setLatLng([lat, lon]);
                droneMarkerRef.current.setPopupContent(`<b>Drone</b><br>Lat: ${lat.toFixed(6)}<br>Lon: ${lon.toFixed(6)}`);
              }
            }
          }
        } else if (msg.topic === CASUALTY_GEOLOCATED_TOPIC) {
          const casualty = msg.message as any;
          const lat = casualty.latitude;
          const lon = casualty.longitude;
          const id = casualty.id || `unknown-${Date.now()}`; // Fallback ID
          
          if (lat !== undefined && lon !== undefined && lat !== 0 && lon !== 0) {
             humansRef.current.set(id, { lat, lon, id, timestamp: Date.now() });
             setHumanCount(humansRef.current.size);
             
             if (mapRef.current) {
                if (!humanMarkersRef.current.has(id)) {
                    const humanIcon = L.divIcon({
                      className: "human-marker",
                      html: `<div style="background-color: #ff0000; width: 20px; height: 20px; border-radius: 50%; border: 2px solid white; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 12px; box-shadow: 0 2px 6px rgba(0,0,0,0.5);">👤</div>`,
                      iconSize: [20, 20],
                      iconAnchor: [10, 10],
                    });
                    const marker = L.marker([lat, lon], { icon: humanIcon })
                      .addTo(mapRef.current)
                      .bindPopup(`<b>Casualty ${id}</b><br>Lat: ${lat.toFixed(6)}<br>Lon: ${lon.toFixed(6)}`);
                    humanMarkersRef.current.set(id, marker);
                    setStatus(`Detected ${humansRef.current.size} casualties`);
                    console.log("[PathSolver] Casualty detected:", id, lat, lon);
                } else {
                    const existing = humanMarkersRef.current.get(id);
                    if (existing) {
                      existing.setLatLng([lat, lon]);
                      existing.setPopupContent(`<b>Casualty ${id}</b><br>Lat: ${lat.toFixed(6)}<br>Lon: ${lon.toFixed(6)}`);
                    }
                }
             }
          }
        }
      });
      if (renderDone) renderDone();
    }
  }, [messages, renderDone]);

  const calculateDistance = (p1: Point, p2: Point) => {
    // Simple Euclidean distance for now, or Haversine if needed. 
    // Since we are doing "trivially through Dijkstra's" (which implies graph edges), 
    // and the prompt says "cost here is purely defined as the distance", 
    // Euclidean on lat/lon is a rough approximation but works for small areas.
    // Better to use Leaflet's distanceTo if available, or Haversine.
    // Leaflet's LatLng has distanceTo.
    return L.latLng(p1.lat, p1.lon).distanceTo(L.latLng(p2.lat, p2.lon));
  };

  const solvePaths = () => {
    // Always use the initial drone position as the starting point for path solving
    const startNode = initialDronePositionRef.current ?? dronePositionRef.current;
    if (!startNode) {
        console.warn("No drone position available.");
        setStatus("❌ No drone position available");
        return;
    }
    
    const unvisited = Array.from(humansRef.current.values());
    
    if (unvisited.length === 0) {
        console.warn("No detected geolocations to visit.");
        setStatus("❌ No casualties detected to visit");
        return;
    }

    setStatus(`🔄 Calculating optimal path for ${unvisited.length} casualties...`);

    // Nearest Neighbor TSP
    const path: Point[] = [startNode];
    let current = startNode;
    let totalDistance = 0;
    const remaining = [...unvisited];

    while (remaining.length > 0) {
        let nearestIndex = -1;
        let minDist = Infinity;

        for (let i = 0; i < remaining.length; i++) {
            const dist = calculateDistance(current, remaining[i]);
            if (dist < minDist) {
                minDist = dist;
                nearestIndex = i;
            }
        }

        if (nearestIndex !== -1) {
            const nextPoint = remaining[nearestIndex];
            path.push(nextPoint);
            totalDistance += minDist;
            current = nextPoint;
            remaining.splice(nearestIndex, 1);
        } else {
            break; // Should not happen
        }
    }

    if (mapRef.current && pathLayerGroupRef.current) {
        pathLayerGroupRef.current.clearLayers();
        pathPolylineRef.current = null;

        const latlngs = path.map(p => [p.lat, p.lon] as [number, number]);
        const polyline = L.polyline(latlngs, { 
          color: "#ff0000", 
          weight: 5,
          opacity: 0.9,
          dashArray: "10, 5",
          lineCap: "round",
          lineJoin: "round",
          pane: PATH_PANE_NAME,
        });

        pathLayerGroupRef.current.addLayer(polyline);
        pathPolylineRef.current = polyline;
        setHasPath(true);

        const bounds = L.latLngBounds(latlngs);
        mapRef.current.fitBounds(bounds, { padding: [50, 50] });

        console.log("[PathSolver] Path added to layer group. Total waypoints:", path.length);
    }
    
    const distanceKm = (totalDistance / 1000).toFixed(2);
    setStatus(`✓ Path calculated: ${path.length - 1} waypoints, ${distanceKm} km`);
    console.log("[PathSolver] Solved Path:", path, `Total distance: ${distanceKm} km, Start:`, startNode);
  };

  const clearPath = () => {
    if (pathLayerGroupRef.current) {
      pathLayerGroupRef.current.clearLayers();
      pathPolylineRef.current = null;
      setHasPath(false);
      setStatus(`Path cleared - ${humanCount} casualties detected`);
      console.log("[PathSolver] Path cleared from layer group");
    }
  };

  return (
    <div style={{ width: "100%", height: "100%", display: "flex", flexDirection: "column" }}>
      <div style={{ padding: "10px", backgroundColor: "#333", color: "white", display: "flex", alignItems: "center", gap: "15px" }}>
        <button 
          onClick={solvePaths} 
          style={{ 
            padding: "8px 16px", 
            cursor: hasDronePosition && humanCount > 0 ? "pointer" : "not-allowed",
            backgroundColor: hasDronePosition && humanCount > 0 ? "#4CAF50" : "#666",
            color: "white",
            border: "none",
            borderRadius: "4px",
            fontWeight: "bold"
          }}
          disabled={!hasDronePosition || humanCount === 0}
        >
          🗺️ Solve Path
        </button>
        <button 
          onClick={clearPath} 
          style={{ 
            padding: "8px 16px", 
            cursor: hasPath ? "pointer" : "not-allowed",
            backgroundColor: hasPath ? "#f44336" : "#555",
            color: "white",
            border: "none",
            borderRadius: "4px",
            fontWeight: "bold"
          }}
          disabled={!hasPath}
        >
          🗑️ Clear Path
        </button>
        <div style={{ fontSize: "14px" }}>
          <span style={{ marginRight: "15px" }}>
            🚁 Drone: {hasDronePosition ? "✓" : "⏳"}
          </span>
          <span style={{ marginRight: "15px" }}>
            👥 Casualties: {humanCount}
          </span>
          <span style={{ color: "#aaa" }}>
            {status}
          </span>
        </div>
      </div>
      <div ref={mapContainerRef} style={{ flex: 1, width: "100%", minHeight: "400px" }}></div>
    </div>
  );
}

export function initPathSolverPanel(context: PanelExtensionContext): void {
  const root = createRoot(context.panelElement);
  root.render(<PathSolverPanel context={context} />);
}
