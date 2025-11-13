import {
  Immutable,
  PanelExtensionContext,
  RenderState,
  SettingsTree,
  SettingsTreeAction,
} from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { ros1, ros2humble as ros2 } from "@foxglove/rosmsg-msgs-common";
import { createRoot } from "react-dom/client";

// No longer need TeleopCommand - we just send Bool messages

type PanelConfig = {
  captureTopic: string; // e.g. "/trigger_capture"
  recordTopic: string; // e.g. "/trigger_record"
};

const DEFAULT_CONFIG: PanelConfig = {
  captureTopic: "/trigger_capture",
  recordTopic: "/trigger_record",
};

function buildSettingsTree(
  cfg: PanelConfig,
  setCfg: (partial: Partial<PanelConfig>) => void,
): SettingsTree {
  return {
    actionHandler: (action: SettingsTreeAction) => {
      if (action.action !== "update") return;
      const key = action.payload.path[action.payload.path.length - 1] as keyof PanelConfig;
      const raw = (action as any).payload.value as any;
      if (key === "captureTopic") setCfg({ captureTopic: String(raw ?? "") });
      if (key === "recordTopic") setCfg({ recordTopic: String(raw ?? "") });
    },
    nodes: {
      general: {
        label: "Gimbal Capture Controller",
        fields: {
          captureTopic: { label: "Capture topic", input: "string", value: cfg.captureTopic },
          recordTopic: { label: "Record topic", input: "string", value: cfg.recordTopic },
        },
      },
    },
  };
}

// COMMENTED OUT - useInterval not needed without continuous teleoperation
// function useInterval(callback: () => void, hz: number, active: boolean): void {
//   const cbRef = useRef(callback);
//   cbRef.current = callback;
//   useEffect(() => {
//     if (!active || hz <= 0) return;
//     const intervalMs = 1000 / hz;
//     const id = setInterval(() => cbRef.current(), intervalMs);
//     return () => clearInterval(id);
//   }, [hz, active]);
// }

// Simple button for single-shot actions (capture/record)
function Button({ label, onClick, style }: {
  label: string;
  onClick: () => void;
  style?: React.CSSProperties;
}): ReactElement {
  const [active, setActive] = useState(false);
  const handleClick = () => {
    setActive(true);
    onClick();
    setTimeout(() => setActive(false), 200);
  };

  return (
    <button
      onClick={handleClick}
      style={{
        padding: "12px 20px",
        borderRadius: 8,
        border: "2px solid #4a90e2",
        background: active ? "#2e6bc7" : "#f0f8ff",
        color: active ? "#fff" : "#000",
        cursor: "pointer",
        fontWeight: 600,
        fontSize: "14px",
        minWidth: "120px",
        transition: "all 0.2s",
        ...style,
      }}
    >
      {label}
    </button>
  );
}

function GimbalCapturePanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [cfg, setCfgState] = useState<PanelConfig>(() => ({
    ...DEFAULT_CONFIG,
    ...(context.initialState as Partial<PanelConfig> | undefined),
  }));

  // COMMENTED OUT - teleoperation pressed state not needed
  // const [up, setUp] = useState(false);
  // const [down, setDown] = useState(false);
  // const [left, setLeft] = useState(false);
  // const [right, setRight] = useState(false);
  // const [zoomIn, setZoomIn] = useState(false);
  // const [zoomOut, setZoomOut] = useState(false);
  
  const [status, setStatus] = useState<string>("Ready for capture/record");
  const [lastAction, setLastAction] = useState<string>("");

  // persist config
  const setCfg = (partial: Partial<PanelConfig>) => {
    setCfgState((prev) => {
      const next = { ...prev, ...partial };
      context.saveState(next);
      context.updatePanelSettingsEditor(buildSettingsTree(next, setCfg));
      return next;
    });
  };

  useEffect(() => {
    context.updatePanelSettingsEditor(buildSettingsTree(cfg, setCfg));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Render lifecycle: ensure we (re)advertise when data source changes and call done
  useLayoutEffect(() => {
    context.watch("topics");
    context.onRender = (_rs: Immutable<RenderState>, done) => {
      // attempt advertise if not yet successful
      ensureAdvertised();
      setRenderDone(() => done);
    };
  }, [context]);
  useEffect(() => void renderDone?.(), [renderDone]);

  // Detect profile and advertise appropriate schema
  const advertisedCaptureRef = useRef<{ topic?: string; mode?: "ros" | "custom" }>({});
  const advertisedRecordRef = useRef<{ topic?: string; mode?: "ros" | "custom" }>({});
  const useRos = context.dataSourceProfile === "ros1" || context.dataSourceProfile === "ros2";

  const ensureAdvertised = () => {
    const mode: "ros" | "custom" = useRos ? "ros" : "custom";
    if (!context.advertise) return; // publish not supported (yet)
    
    // Advertise capture topic (std_msgs/Bool)
    const captureTopic = cfg.captureTopic;
    if (advertisedCaptureRef.current.topic !== captureTopic || advertisedCaptureRef.current.mode !== mode) {
      try {
        if (mode === "ros") {
          const dt = context.dataSourceProfile === "ros1" ? ros1 : ros2;
          const datatypes = new Map<string, unknown>([
            ["std_msgs/Bool", (dt as any)["std_msgs/Bool"]],
          ]);
          try {
            context.advertise(captureTopic, "std_msgs/Bool", { datatypes });
          } catch {
            context.advertise(captureTopic, "std_msgs/Bool");
          }
        } else {
          context.advertise(captureTopic, "std_msgs/Bool");
        }
        advertisedCaptureRef.current = { topic: captureTopic, mode };
        console.log("[GimbalCapture] ✓ Advertised capture topic:", captureTopic);
      } catch (err) {
        console.error("[GimbalCapture] ✗ Failed to advertise capture:", err);
      }
    }

    // Advertise record topic (std_msgs/Bool)
    const recordTopic = cfg.recordTopic;
    if (advertisedRecordRef.current.topic !== recordTopic || advertisedRecordRef.current.mode !== mode) {
      try {
        if (mode === "ros") {
          const dt = context.dataSourceProfile === "ros1" ? ros1 : ros2;
          const datatypes = new Map<string, unknown>([
            ["std_msgs/Bool", (dt as any)["std_msgs/Bool"]],
          ]);
          try {
            context.advertise(recordTopic, "std_msgs/Bool", { datatypes });
          } catch {
            context.advertise(recordTopic, "std_msgs/Bool");
          }
        } else {
          context.advertise(recordTopic, "std_msgs/Bool");
        }
        advertisedRecordRef.current = { topic: recordTopic, mode };
        console.log("[GimbalCapture] ✓ Advertised record topic:", recordTopic);
      } catch (err) {
        console.error("[GimbalCapture] ✗ Failed to advertise record:", err);
      }
    }
  };

  useEffect(() => {
    ensureAdvertised();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [cfg.captureTopic, cfg.recordTopic, useRos]);

  const publishBool = (topic: string, value: boolean) => {
    try {
      const msg = { data: value };
      context.publish?.(topic, msg);
      console.log("[GimbalCapture] Published Bool to", topic, ":", msg);
    } catch (err) {
      console.error("[GimbalCapture] ✗ Failed to publish to", topic, ":", err);
      setStatus(`发布失败: ${err}`);
    }
  };

  // COMMENTED OUT - Continuous publisher not needed for capture/record only
  // const activeHold = up || down || left || right || zoomIn || zoomOut;
  // useInterval(() => {
  //   const panY = (up ? 1 : 0) + (down ? -1 : 0);
  //   const panX = (right ? 1 : 0) + (left ? -1 : 0);
  //   const scale = cfg.stepSize;
  //   const zoom: -1 | 0 | 1 = zoomIn && !zoomOut ? 1 : zoomOut && !zoomIn ? -1 : 0;
  //   const cmd: TeleopCommand = {
  //     panX: panX * scale,
  //     panY: panY * scale,
  //     zoom,
  //     trigger: false,
  //     modeSwitch: false,
  //     recenter: false,
  //   };
  //   publish(cmd);
  // }, cfg.repeatHz, activeHold);

  const sendCapture = () => {
    publishBool(cfg.captureTopic, true);
    setLastAction("Capture");
    setStatus("Capture triggered");
    setTimeout(() => setStatus("Ready for capture/record"), 2000);
  };

  const sendRecord = () => {
    publishBool(cfg.recordTopic, true);
    setLastAction("Record Toggle");
    setStatus("Record toggled");
    setTimeout(() => setStatus("Ready for capture/record"), 2000);
  };

  return (
    <div style={{ padding: 20, fontFamily: "sans-serif" }}>
      <h3 style={{ margin: "0 0 10px 0", fontSize: "18px", fontWeight: 600 }}>
        Gimbal Capture Controller
      </h3>
      <div style={{ color: "#666", marginBottom: 16, fontSize: "12px" }}>
        {status} · Data source: {context.dataSourceProfile ?? "unknown"} · Publish: {context.publish ? "available" : "unavailable"}
      </div>
      {lastAction && (
        <div style={{ fontSize: "12px", color: "#4a90e2", marginBottom: 16 }}>
          Last action: {lastAction}
        </div>
      )}

      {/* COMMENTED OUT - All teleoperation controls removed */}
      {/* <div style={{ display: "flex", gap: 12, flexWrap: "wrap", alignItems: "center" }}>
        <div style={grid}>
          <div />
          <Button label="Up" onPress={() => setUp(true)} onRelease={() => setUp(false)} />
          <div />
          <Button label="Left" onPress={() => setLeft(true)} onRelease={() => setLeft(false)} />
          <div />
          <Button label="Right" onPress={() => setRight(true)} onRelease={() => setRight(false)} />
          <div />
          <Button label="Down" onPress={() => setDown(true)} onRelease={() => setDown(false)} />
          <div />
        </div>
        <div style={{ display: "grid", gridTemplateRows: "repeat(3, 40px)", gap: 8 }}>
          <Button label="Zoom +" onPress={() => setZoomIn(true)} onRelease={() => setZoomIn(false)} />
          <div style={{ textAlign: "center", fontSize: 12, color: "#666" }}>Zoom</div>
          <Button label="Zoom -" onPress={() => setZoomOut(true)} onRelease={() => setZoomOut(false)} />
        </div>
      </div> */}

      {/* KEPT - Capture and Record controls */}
      <div style={{ marginBottom: 20 }}>
        <h4 style={{ fontSize: "14px", fontWeight: 600, marginBottom: 10 }}>
          Capture & Record
        </h4>
        <div style={{ display: "flex", gap: 12, flexWrap: "wrap" }}>
          <Button label="📷 Capture" onClick={sendCapture} />
          <Button 
            label="🔴 Record Toggle" 
            onClick={sendRecord}
            style={{ borderColor: "#e24a4a", background: "#fff0f0" }}
          />
        </div>
      </div>

      <div style={{ marginTop: 20, fontSize: 12, color: "#666" }}>
        <div style={{ marginBottom: 4 }}>
          <strong>Capture Topic:</strong> {cfg.captureTopic} (std_msgs/Bool)
        </div>
        <div style={{ marginBottom: 4 }}>
          <strong>Record Topic:</strong> {cfg.recordTopic} (std_msgs/Bool)
        </div>
        <div>
          Click buttons to trigger capture or toggle recording.
        </div>
        <div style={{ marginTop: 8, fontStyle: "italic", color: "#999" }}>
          Note: Pan/tilt/zoom teleoperation controls have been removed. Use joystick controller for manual gimbal control.
        </div>
      </div>
    </div>
  );
}

export function initGimbalCapturePanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<GimbalCapturePanel context={context} />);
  return () => root.unmount();
}

