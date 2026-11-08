import { ExtensionContext } from "@foxglove/extension";

import { initControlPanel } from "./ControlPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "TSP05 Controls", initPanel: initControlPanel });
}
