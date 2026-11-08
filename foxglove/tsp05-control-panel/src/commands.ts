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
} as const;

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
  lastMessageMs?: number;
};

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
    const key = KEY_BY_TOPIC[ev.topic];
    const data = (ev.message as { data?: unknown } | undefined)?.data;
    if (key == undefined || (typeof data !== "number" && typeof data !== "boolean")) {
      continue;
    }
    if (next === prev) {
      next = { ...prev };
    }
    (next as Record<string, unknown>)[key] = data;
    next.lastMessageMs = nowMs;
  }
  return next;
}

/** No telemetry for this long = treat the bridge as disconnected in the UI. */
export const STALE_MS = 2000;

export function isLive(state: PanelState, nowMs: number): boolean {
  return (
    state.connected === true &&
    state.lastMessageMs != undefined &&
    nowMs - state.lastMessageMs < STALE_MS
  );
}
