import { ExtensionContext } from "@foxglove/extension";
import { initPathSolverPanel } from "./PathSolverPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "Path Solver",
    initPanel: initPathSolverPanel,
  });
}
