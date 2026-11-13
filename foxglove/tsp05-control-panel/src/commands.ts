// Command and state logic for the TSP05 control panel, kept free of React and
// of the Foxglove API so it can be unit-tested with plain Node (src/commands.test.ts).
//
// Topic names must match ros2_ws/src/tsp05_bridge/tsp05_bridge/topics.py;
// test_foxglove_layout.py checks every "/tsp05/..." string in this file.

export const NS = "/tsp05";

export const CMD = {
  mode: `${NS}/cmd/mode`,
  servoDeg: `${NS}/cmd/servo_deg`,
  dcMoveDeg: `${NS}/cmd/dc_move_deg`,
  dcRpm: `${NS}/cmd/dc_rpm`,
  dcStop: `${NS}/cmd/dc_stop`,
  stepperDeg: `${NS}/cmd/stepper_deg`,
} as const;

export const STATE = {
  mode: `${NS}/mode`,
  interlock: `${NS}/interlock`,
  connected: `${NS}/bridge/connected`,
  servoDeg: `${NS}/servo/deg`,
  dcDeg: `${NS}/dc/deg`,
  dcRpm: `${NS}/dc/rpm`,
  stepperDeg: `${NS}/stepper/deg`,
  stepperTargetDeg: `${NS}/stepper/target_deg`,
  flexFiltDeg: `${NS}/flex/deg_filt`,
  tofRawMm: `${NS}/tof/raw_mm`,
  slotCount: `${NS}/slot/count`,
  // feedback worked out by the bridge (tsp05_bridge/feedback.py)
  dcMode: `${NS}/dc/mode`,
  dcTarget: `${NS}/dc/target`,
  dcArrived: `${NS}/dc/arrived`,
  dcSettled: `${NS}/dc/settled`,
  tofFault: `${NS}/tof/fault`,
  rateHz: `${NS}/bridge/rate_hz`,
  lineAgeMs: `${NS}/bridge/line_age_ms`,
} as const;

/** One line per command from the bridge: applied / ignored / no effect. */
export const STATUS_TOPIC = `${NS}/cmd/status`;

export type CmdTopic = (typeof CMD)[keyof typeof CMD];
type SchemaName = "std_msgs/msg/Int8" | "std_msgs/msg/Float32" | "std_msgs/msg/Empty";

export const SCHEMA: Record<CmdTopic, SchemaName> = {
  [CMD.mode]: "std_msgs/msg/Int8",
  [CMD.servoDeg]: "std_msgs/msg/Float32",
  [CMD.dcMoveDeg]: "std_msgs/msg/Float32",
  [CMD.dcRpm]: "std_msgs/msg/Float32",
  [CMD.dcStop]: "std_msgs/msg/Empty",
  [CMD.stepperDeg]: "std_msgs/msg/Float32",
};

// ROS 2 message definitions that foxglove_bridge needs when a client advertises.
// Inline instead of @foxglove/rosmsg-msgs-common: three one-field messages.
// Both name styles are listed because different Foxglove versions look up either.
type FieldDef = { name: string; type: string; isArray: boolean; isComplex: boolean };
type MessageDefinition = { name: string; definitions: FieldDef[] };
const field = (type: string): FieldDef[] => [
  { name: "data", type, isArray: false, isComplex: false },
];
const DEFS: Record<SchemaName, FieldDef[]> = {
  "std_msgs/msg/Int8": field("int8"),
  "std_msgs/msg/Float32": field("float32"),
  "std_msgs/msg/Empty": [],
};

export function datatypesFor(schema: SchemaName): Map<string, MessageDefinition> {
  const short = schema.replace("/msg/", "/");
  const definitions = DEFS[schema];
  return new Map([
    [schema, { name: schema, definitions }],
    [short, { name: short, definitions }],
  ]);
}

// Limits for GUI input. The firmware clamps too; these only stop obvious typos.
export const LIMITS = {
  servoDeg: [0, 180],
  dcMoveDeg: [-3600, 3600],
  dcRpm: [-150, 150],
  stepperDeg: [-3600, 3600],
} as const;

export type Command = { topic: CmdTopic; message: { data?: number } };

export class CommandError extends Error {}

function checked(value: number, [lo, hi]: readonly [number, number], what: string): number {
  if (!Number.isFinite(value)) {
    throw new CommandError(`${what}: enter a number`);
  }
  if (value < lo || value > hi) {
    throw new CommandError(`${what}: must be between ${lo} and ${hi}`);
  }
  return Math.round(value * 10) / 10; // the protocol carries 1 decimal
}

export const commands = {
  mode: (mode: 0 | 1): Command => ({ topic: CMD.mode, message: { data: mode } }),
  servo: (deg: number): Command => ({
    topic: CMD.servoDeg,
    message: { data: checked(deg, LIMITS.servoDeg, "Servo angle") },
  }),
  dcMove: (deg: number): Command => {
    const data = checked(deg, LIMITS.dcMoveDeg, "DC move");
    if (data === 0) {
      throw new CommandError("DC move: enter a non-zero angle");
    }
    return { topic: CMD.dcMoveDeg, message: { data } };
  },
  dcRpm: (rpm: number): Command => ({
    topic: CMD.dcRpm,
    message: { data: checked(rpm, LIMITS.dcRpm, "DC RPM") },
  }),
  dcStop: (): Command => ({ topic: CMD.dcStop, message: {} }),
  stepper: (deg: number): Command => {
    const data = checked(deg, LIMITS.stepperDeg, "Stepper move");
    if (data === 0) {
      throw new CommandError("Stepper move: enter a non-zero angle");
    }
    return { topic: CMD.stepperDeg, message: { data } };
  },
};

/** STOP must work in both modes, but the firmware ignores motor commands in
 * SENSOR mode, so stop = switch to GUI mode first, then stop the DC motor. */
export function stopSequence(mode: number | undefined): Command[] {
  return mode === 1 ? [commands.mode(0), commands.dcStop()] : [commands.dcStop()];
}

/** Text box -> number, accepting "90", " -45.5 ", "+10". Empty or junk -> NaN. */
export function parseInput(text: string): number {
  const t = text.trim();
  return /^[+-]?(\d+\.?\d*|\.\d+)$/.test(t) ? Number(t) : NaN;
}

// ---------------------------------------------------------------- state
export type PanelState = {
  mode?: number;
  interlock?: boolean;
  connected?: boolean;
  servoDeg?: number;
  dcDeg?: number;
  dcRpm?: number;
  stepperDeg?: number;
  stepperTargetDeg?: number;
  flexFiltDeg?: number;
  tofRawMm?: number;
  slotCount?: number;
  dcMode?: number;
  dcTarget?: number;
  dcArrived?: boolean;
  dcSettled?: boolean;
  tofFault?: boolean;
  rateHz?: number;
  lineAgeMs?: number;
  /** Last time a telemetry-driven topic arrived (not the bridge's own health topics). */
  lastMessageMs?: number;
  /** Most recent command statuses, newest last. */
  statuses?: StatusLine[];
};

export type StatusLine = { text: string; atMs: number };
export const MAX_STATUS_LINES = 4;

// Published by the bridge on a timer even when the Uno is silent: they must not
// make the Uno look alive.
const HEALTH_KEYS = new Set<keyof PanelState>(["connected", "rateHz", "lineAgeMs"]);

const KEY_BY_TOPIC: Record<string, keyof PanelState> = Object.fromEntries(
  Object.entries(STATE).map(([key, topic]) => [topic, key as keyof PanelState]),
);

type MessageLike = { topic: string; message: unknown };

/** Fold the messages of one render frame into the panel state (last value wins). */
export function applyMessages(
  prev: PanelState,
  events: readonly MessageLike[],
  nowMs: number,
): PanelState {
  let next = prev;
  for (const ev of events) {
    const data = (ev.message as { data?: unknown } | undefined)?.data;
    if (ev.topic === STATUS_TOPIC && typeof data === "string") {
      if (next === prev) {
        next = { ...prev };
      }
      next.statuses = [...(next.statuses ?? []), { text: data, atMs: nowMs }].slice(
        -MAX_STATUS_LINES,
      );
      continue;
    }
    const key = KEY_BY_TOPIC[ev.topic];
    if (key == undefined || (typeof data !== "number" && typeof data !== "boolean")) {
      continue;
    }
    if (next === prev) {
      next = { ...prev };
    }
    (next as Record<string, unknown>)[key] = data;
    if (!HEALTH_KEYS.has(key)) {
      next.lastMessageMs = nowMs;
    }
  }
  return next;
}

/** No telemetry for this long = treat the bridge as disconnected in the UI. */
export const STALE_MS = 2000;

export function isLive(state: PanelState, nowMs: number): boolean {
  if (state.connected !== true || state.lastMessageMs == undefined) {
    return false;
  }
  if (nowMs - state.lastMessageMs >= STALE_MS) {
    return false;
  }
  // The bridge measures the age of the last telemetry line itself; trust it when present.
  return state.lineAgeMs == undefined || state.lineAgeMs < STALE_MS;
}

// ---------------------------------------------------------------- feedback views
export type Tone = "good" | "warn" | "bad" | "neutral";

/** Below this the link is degraded (the Uno sends 20 Hz). */
export const RATE_WARN_HZ = 15;
/** 20 missed lines: show that telemetry has stopped. */
export const LINE_AGE_WARN_MS = 1000;

export function linkView(state: PanelState, nowMs: number): { text: string; tone: Tone } {
  if (!isLive(state, nowMs)) {
    return {
      text: state.connected === true ? "No telemetry" : "Uno disconnected",
      tone: "bad",
    };
  }
  if (state.lineAgeMs != undefined && state.lineAgeMs >= LINE_AGE_WARN_MS) {
    // Lines have stopped but not yet for STALE_MS: say so instead of "connected · 0 Hz".
    return { text: `No telemetry for ${(state.lineAgeMs / 1000).toFixed(1)} s`, tone: "bad" };
  }
  if (state.rateHz == undefined) {
    return { text: "Uno connected", tone: "good" };
  }
  const text = `Uno connected · ${state.rateHz.toFixed(1)} Hz`;
  return { text, tone: state.rateHz < RATE_WARN_HZ ? "warn" : "good" };
}

/** A position move that has not settled after this long gets a warning. */
export const DC_SETTLE_WARN_MS = 3000;

export function dcView(
  state: PanelState,
  nowMs: number,
  lastMoveSentMs: number | undefined,
): { text: string; tone: Tone } {
  const f1 = (v: number | undefined) => (v == undefined ? "—" : v.toFixed(1));
  if (state.dcMode === 2) {
    if (state.dcSettled === true) {
      return {
        text: `arrived ✓ at ${f1(state.dcDeg)}° (target ${f1(state.dcTarget)}°)`,
        tone: "good",
      };
    }
    if (lastMoveSentMs != undefined && nowMs - lastMoveSentMs > DC_SETTLE_WARN_MS) {
      return {
        text: `not arrived after ${DC_SETTLE_WARN_MS / 1000} s: ${f1(state.dcDeg)}° vs target ${f1(state.dcTarget)}°`,
        tone: "warn",
      };
    }
    return { text: `moving to ${f1(state.dcTarget)}° (now ${f1(state.dcDeg)}°)`, tone: "neutral" };
  }
  if (state.dcMode === 1) {
    return { text: `holding ${f1(state.dcTarget)} rpm (now ${f1(state.dcRpm)})`, tone: "neutral" };
  }
  if (state.dcMode === 0) {
    return { text: "stopped", tone: "neutral" };
  }
  return { text: "—", tone: "neutral" };
}

export function statusTone(text: string): Tone {
  if (text.includes(": applied")) {
    return "good";
  }
  if (text.includes(": ignored")) {
    return "warn";
  }
  if (text.includes("no effect")) {
    return "bad";
  }
  return "neutral";
}

/** Slider shows the live servo angle unless the user is dragging it. */
export function servoSliderValue({
  dragging,
  dragValue,
  live,
}: {
  dragging: boolean;
  dragValue: number;
  live: number | undefined;
}): number {
  return dragging || live == undefined ? dragValue : Math.round(live);
}

// One-click moves verified on the bench (Track 1): every position move ends within
// +-2 deg in about 0.5 s and velocity holds within 1 %.
export const DC_MOVE_PRESETS = [-360, -90, -10, 10, 90, 360] as const;
export const DC_RPM_PRESETS = [-60, 30, 60, 120] as const;

// SENSOR-mode mappings, from firmware/tsp05/config.h (a test checks them).
export const SENSOR_MAP = {
  flexDeadzoneDeg: 10, // MAP_FLEX_DEADZONE_DEG
  flexFullDeg: 90, // MAP_FLEX_FULL_DEG
  dcMaxRpm: 60, // MAP_DC_MAX_RPM
  swipeServoDegS: 45, // GESTURE_SERVO_DEG_S
  slotJogDeg: 15, // MAP_SLOT_JOG_DEG
} as const;
