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

type HumanGPS = { lat: number; lon: number; id: string; timestamp: number };
type Point = { lat: number; lon: number };

function PathSolverPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [messages, setMessages] = useState<undefined | Immutable<MessageEvent[]>>();
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const mapRef = useRef<LeafletMap | null>(null);
  const humanMarkersRef = useRef<Map<string, LeafletMarker>>(new Map());
  const droneMarkerRef = useRef<LeafletMarker | null>(null);
  const pathPolylineRef = useRef<LeafletPolyline | null>(null);

  const dronePositionRef = useRef<Point | null>(null);
  const humansRef = useRef<Map<string, HumanGPS>>(new Map());

  const CASUALTY_GEOLOCATED_TOPIC = "/casualty_geolocated";
  const DRONE_GPS_TOPIC = "/dtc_mrsd_/mavros/global_position/global";

  const subscriptions = useMemo(() => [
    { topic: CASUALTY_GEOLOCATED_TOPIC },
    { topic: DRONE_GPS_TOPIC },
  ], []);

  useLayoutEffect(() => {
    context.subscribe(subscriptions);
  }, [context, subscriptions]);

  useEffect(() => {
    const onRender = (renderState: any, done: () => void) => {
      setRenderDone(() => done);
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        setMessages(renderState.currentFrame);
      }
    };
    context.onRender = onRender;
    return () => { context.onRender = undefined; };
  }, [context]);

  useEffect(() => {
    injectLeafletStyles();
    const mapContainer = document.getElementById("map-container");
    if (mapContainer && !mapRef.current) {
      const map = L.map(mapContainer).setView([0, 0], 2);
      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors'
      }).addTo(map);
      mapRef.current = map;
    }
  }, []);

  useEffect(() => {
    if (messages) {
      messages.forEach((msg) => {
        if (msg.topic === DRONE_GPS_TOPIC) {
          const lat = (msg.message as any).latitude;
          const lon = (msg.message as any).longitude;
          if (lat != null && lon != null) {
            dronePositionRef.current = { lat, lon };
            if (mapRef.current) {
              if (!droneMarkerRef.current) {
                droneMarkerRef.current = L.marker([lat, lon], { title: "Drone" }).addTo(mapRef.current);
                mapRef.current.setView([lat, lon], 18);
              } else {
                droneMarkerRef.current.setLatLng([lat, lon]);
              }
            }
          }
        } else if (msg.topic === CASUALTY_GEOLOCATED_TOPIC) {
          const lat = (msg.message as any).latitude;
          const lon = (msg.message as any).longitude;
          const id = (msg.message as any).id || `unknown-${Date.now()}`; // Fallback ID
          
          if (lat != null && lon != null) {
             humansRef.current.set(id, { lat, lon, id, timestamp: Date.now() });
             
             if (mapRef.current) {
                if (!humanMarkersRef.current.has(id)) {
                    const marker = L.marker([lat, lon], { title: `Human ${id}` }).addTo(mapRef.current);
                    humanMarkersRef.current.set(id, marker);
                } else {
                    humanMarkersRef.current.get(id)?.setLatLng([lat, lon]);
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
    if (!dronePositionRef.current) {
        console.warn("No drone position available.");
        return;
    }
    
    const startNode = dronePositionRef.current;
    const unvisited = Array.from(humansRef.current.values());
    
    if (unvisited.length === 0) {
        console.warn("No detected geolocations to visit.");
        return;
    }

    // Nearest Neighbor TSP
    const path: Point[] = [startNode];
    let current = startNode;
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
            current = nextPoint;
            remaining.splice(nearestIndex, 1);
        } else {
            break; // Should not happen
        }
    }

    // Visualize path
    if (mapRef.current) {
        if (pathPolylineRef.current) {
            pathPolylineRef.current.remove();
        }
        const latlngs = path.map(p => [p.lat, p.lon] as [number, number]);
        pathPolylineRef.current = L.polyline(latlngs, { color: 'red' }).addTo(mapRef.current);
    }
    
    console.log("Solved Path:", path);
  };

  return (
    <div style={{ width: "100%", height: "100%", display: "flex", flexDirection: "column" }}>
      <div style={{ padding: "10px", backgroundColor: "#333", color: "white" }}>
        <button onClick={solvePaths} style={{ padding: "8px 16px", cursor: "pointer" }}>
          Solve Paths
        </button>
      </div>
      <div id="map-container" style={{ flex: 1, width: "100%" }}></div>
    </div>
  );
}

export function initPathSolverPanel(context: PanelExtensionContext): void {
  const root = createRoot(context.panelElement);
  root.render(<PathSolverPanel context={context} />);
}
