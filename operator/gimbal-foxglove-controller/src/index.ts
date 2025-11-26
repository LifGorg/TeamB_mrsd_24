import { ExtensionContext } from "@foxglove/extension";

import { initVirtualGimbalPanel } from "./VirtualGimbalPanel";
import { initGimbalCapturePanel } from "./GimbalCapturePanel";

export function activate(extensionContext: ExtensionContext): void {
  // Legacy panel with full teleoperation controls
  extensionContext.registerPanel({ name: "gimbal-virtual-controller", initPanel: initVirtualGimbalPanel });
  // New simplified panel with only capture/record controls
  extensionContext.registerPanel({ name: "gimbal-capture-controller", initPanel: initGimbalCapturePanel });
}
