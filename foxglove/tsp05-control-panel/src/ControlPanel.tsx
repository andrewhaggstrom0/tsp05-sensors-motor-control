// TSP05 Controls: one Foxglove panel that sends every command by clicking.
// Logic (topics, validation, state folding) lives in commands.ts; this file is UI only.
import { Immutable, MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import {
  CSSProperties,
  ReactElement,
  useCallback,
  useEffect,
  useLayoutEffect,
  useState,
} from "react";
import { createRoot } from "react-dom/client";

import {
  CMD,
  Command,
  CommandError,
  DC_MOVE_PRESETS,
  DC_RPM_PRESETS,
  LIMITS,
  PanelState,
  SCHEMA,
  SENSOR_MAP,
  STATE,
  STATUS_TOPIC,
  Tone,
  applyMessages,
  commands,
  datatypesFor,
  dcView,
  isLive,
  linkView,
  parseInput,
  servoSliderValue,
  statusTone,
  stopSequence,
} from "./commands";

type Theme = {
  bg: string;
  card: string;
  border: string;
  text: string;
  muted: string;
  accent: string;
};
const THEMES: Record<"light" | "dark", Theme> = {
  light: {
    bg: "#ffffff",
    card: "#f5f6f8",
    border: "#d6d9de",
    text: "#1f2328",
    muted: "#5f6b7a",
    accent: "#1a73e8",
  },
  dark: {
    bg: "#15161a",
    card: "#1f2127",
    border: "#33363e",
    text: "#e6e8eb",
    muted: "#9aa3ad",
    accent: "#8ab4f8",
  },
};
const RED = "#d93025";
const GREEN = "#188038";
const PURPLE = "#9334e6";
const AMBER = "#e8710a";

const fmt = (v: number | undefined, digits = 1): string =>
  v == undefined ? "—" : v.toFixed(digits);

function ControlPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [state, setState] = useState<PanelState>({});
  const [now, setNow] = useState(() => Date.now());
  const [colorScheme, setColorScheme] = useState<"light" | "dark">("light");
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [status, setStatus] = useState<{ text: string; error: boolean } | undefined>();

  const [servo, setServo] = useState(90);
  const [servoDragging, setServoDragging] = useState(false);
  const [lastMoveSentMs, setLastMoveSentMs] = useState<number | undefined>();
  const [dcDeg, setDcDeg] = useState("90");
  const [dcRpm, setDcRpm] = useState("30");
  const [stepDeg, setStepDeg] = useState("45");

  const canPublish = context.publish != undefined && context.advertise != undefined;

  useLayoutEffect(() => {
    context.setDefaultPanelTitle("TSP05 Controls");
    context.onRender = (
      renderState: Immutable<{ currentFrame?: readonly MessageEvent[]; colorScheme?: string }>,
      done,
    ) => {
      setRenderDone(() => done);
      if (renderState.colorScheme === "dark" || renderState.colorScheme === "light") {
        setColorScheme(renderState.colorScheme);
      }
      const frame = renderState.currentFrame;
      if (frame && frame.length > 0) {
        setState((prev) => applyMessages(prev, frame, Date.now()));
      }
    };
    context.watch("currentFrame");
    context.watch("colorScheme");
    context.subscribe([...Object.values(STATE), STATUS_TOPIC].map((topic) => ({ topic })));
  }, [context]);

  // Advertise every command topic once; foxglove_bridge needs the ROS 2 definitions.
  useEffect(() => {
    if (!canPublish) {
      return;
    }
    for (const topic of Object.values(CMD)) {
      context.advertise?.(topic, SCHEMA[topic], { datatypes: datatypesFor(SCHEMA[topic]) });
    }
    return () => {
      for (const topic of Object.values(CMD)) {
        context.unadvertise?.(topic);
      }
    };
  }, [context, canPublish]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Re-check staleness twice a second even when no messages arrive.
  useEffect(() => {
    const id = setInterval(() => {
      setNow(Date.now());
    }, 500);
    return () => {
      clearInterval(id);
    };
  }, []);

  const send = useCallback(
    (make: () => Command | Command[]) => {
      try {
        const list = ([] as Command[]).concat(make());
        for (const cmd of list) {
          context.publish?.(cmd.topic, cmd.message);
        }
        const last = list[list.length - 1];
        setStatus({
          text: `sent ${list.map((c) => `${c.topic.replace("/tsp05/cmd/", "")}${c.message.data == undefined ? "" : ` = ${c.message.data}`}`).join(", then ")}`,
          error: false,
        });
        return last;
      } catch (e) {
        setStatus({
          text: e instanceof CommandError ? e.message : `publish failed: ${String(e)}`,
          error: true,
        });
        return undefined;
      }
    },
    [context],
  );

  const t = THEMES[colorScheme];
  const live = isLive(state, now);
  const sensorMode = state.mode === 1;
  const motorsEnabled = canPublish && live && !sensorMode;
  const toneColor = (tone: Tone): string =>
    tone === "good" ? GREEN : tone === "warn" ? AMBER : tone === "bad" ? RED : t.muted;
  const link = linkView(state, now);
  const dc = dcView(state, now, lastMoveSentMs);
  const sliderValue = servoSliderValue({
    dragging: servoDragging,
    dragValue: servo,
    live: state.servoDeg,
  });

  const card: CSSProperties = {
    background: t.card,
    border: `1px solid ${t.border}`,
    borderRadius: 8,
    padding: 10,
  };
  const row: CSSProperties = { display: "flex", gap: 6, alignItems: "center", flexWrap: "wrap" };
  const button = ({
    color = t.accent,
    enabled = motorsEnabled,
  }: { color?: string; enabled?: boolean } = {}): CSSProperties => ({
    background: enabled ? color : t.border,
    color: "#fff",
    border: "none",
    borderRadius: 6,
    padding: "6px 10px",
    fontWeight: 600,
    cursor: enabled ? "pointer" : "not-allowed",
  });
  const input: CSSProperties = {
    width: 54,
    padding: "5px 6px",
    background: t.bg,
    color: t.text,
    border: `1px solid ${t.border}`,
    borderRadius: 6,
  };
  const label: CSSProperties = {
    fontSize: 11,
    color: t.muted,
    textTransform: "uppercase",
    letterSpacing: 0.5,
  };
  const value: CSSProperties = { fontVariantNumeric: "tabular-nums", fontWeight: 600 };

  const sendServo = (deg: number) => {
    setServo(deg);
    send(() => commands.servo(deg));
  };
  const sendDcMove = (deg: number) => {
    if (send(() => commands.dcMove(deg)) != undefined) {
      setLastMoveSentMs(Date.now());
    }
  };

  return (
    <div
      style={{
        height: "100%",
        overflowY: "auto",
        background: t.bg,
        color: t.text,
        padding: 10,
        boxSizing: "border-box",
        fontFamily: "system-ui, sans-serif",
        fontSize: 13,
        display: "flex",
        flexDirection: "column",
        gap: 8,
      }}
    >
      {/* Status strip */}
      <div style={{ ...row, justifyContent: "space-between" }}>
        <span style={{ ...row, gap: 6 }}>
          <span
            style={{ width: 10, height: 10, borderRadius: 5, background: toneColor(link.tone) }}
          />
          <b style={{ color: link.tone === "warn" ? AMBER : undefined }}>{link.text}</b>
          {live && state.tofFault === true && (
            <span
              style={{
                border: `1px solid ${AMBER}`,
                color: AMBER,
                borderRadius: 6,
                padding: "0 6px",
              }}
              title="No valid TOF range for 0.5 s: the TOF values are frozen at their last reading"
            >
              TOF: no target
            </span>
          )}
        </span>
        {state.interlock === true && (
          <span
            style={{
              background: RED,
              color: "#fff",
              borderRadius: 6,
              padding: "2px 8px",
              fontWeight: 700,
            }}
          >
            INTERLOCK: DC stopped
          </span>
        )}
      </div>
      {!canPublish && (
        <div style={{ ...card, borderColor: RED }}>
          This data source cannot publish. Connect with Foxglove WebSocket to ws://localhost:8765.
        </div>
      )}

      {/* Mode */}
      <div style={card}>
        <div style={label}>Mode</div>
        <div style={{ ...row, marginTop: 6 }}>
          {([0, 1] as const).map((m) => (
            <button
              key={m}
              disabled={!canPublish}
              style={{
                ...button({ color: m === 0 ? t.accent : PURPLE, enabled: canPublish }),
                opacity: state.mode === m ? 1 : 0.45,
                flex: 1,
              }}
              onClick={() => send(() => commands.mode(m))}
            >
              {m === 0 ? "GUI" : "SENSOR"}
              {state.mode === m ? " ●" : ""}
            </button>
          ))}
        </div>
        {sensorMode && (
          <div style={{ color: t.muted, marginTop: 6 }}>
            SENSOR mode: sensors drive the motors. Switch to GUI to command. STOP still works.
          </div>
        )}
      </div>

      {/* STOP: always available */}
      <button
        disabled={!canPublish}
        style={{ ...button({ color: RED, enabled: canPublish }), padding: "10px", fontSize: 15 }}
        onClick={() => send(() => stopSequence(state.mode))}
        title={sensorMode ? "Switches to GUI mode, then stops the DC motor" : "Stops the DC motor"}
      >
        STOP DC MOTOR{sensorMode ? " (→ GUI mode)" : ""}
      </button>

      {/* Servo */}
      <div style={card}>
        <div style={{ ...row, justifyContent: "space-between" }}>
          <span style={label}>Servo</span>
          <span>
            actual <span style={value}>{fmt(state.servoDeg)}°</span>
          </span>
        </div>
        <input
          type="range"
          min={LIMITS.servoDeg[0]}
          max={LIMITS.servoDeg[1]}
          step={1}
          value={sliderValue}
          disabled={!motorsEnabled}
          style={{ width: "100%", marginTop: 6 }}
          onPointerDown={() => {
            setServo(sliderValue);
            setServoDragging(true);
          }}
          onChange={(e) => {
            setServo(Number(e.target.value));
          }}
          onPointerUp={() => {
            setServoDragging(false);
            sendServo(servo);
          }}
          onKeyDown={() => {
            setServoDragging(true);
          }}
          onKeyUp={() => {
            setServoDragging(false);
            sendServo(servo);
          }}
        />
        <div style={row}>
          <span style={value}>{sliderValue}°</span>
          {[0, 90, 180].map((deg) => (
            <button
              key={deg}
              disabled={!motorsEnabled}
              style={button()}
              onClick={() => {
                sendServo(deg);
              }}
            >
              {deg}°
            </button>
          ))}
        </div>
      </div>

      {/* DC motor */}
      <div style={card}>
        <div style={{ ...row, justifyContent: "space-between" }}>
          <span style={label}>DC motor (PID)</span>
          <span>
            <span style={value}>{fmt(state.dcDeg)}°</span> ·{" "}
            <span style={value}>{fmt(state.dcRpm)} RPM</span>
          </span>
        </div>
        <div style={{ ...row, marginTop: 6 }}>
          <span style={{ width: 56 }}>Move</span>
          <input
            style={input}
            value={dcDeg}
            disabled={!motorsEnabled}
            onChange={(e) => {
              setDcDeg(e.target.value);
            }}
          />
          <span>°</span>
          <button
            disabled={!motorsEnabled}
            style={button()}
            onClick={() => {
              sendDcMove(parseInput(dcDeg));
            }}
          >
            Move
          </button>
        </div>
        <div style={{ ...row, marginTop: 4 }}>
          {DC_MOVE_PRESETS.map((deg) => (
            <button
              key={deg}
              disabled={!motorsEnabled}
              style={{ ...button(), padding: "4px 7px" }}
              onClick={() => {
                sendDcMove(deg);
              }}
            >
              {deg > 0 ? "+" : "−"}
              {Math.abs(deg)}°
            </button>
          ))}
        </div>
        <div style={{ ...row, marginTop: 6 }}>
          <span style={{ width: 56 }}>Velocity</span>
          <input
            style={input}
            value={dcRpm}
            disabled={!motorsEnabled}
            onChange={(e) => {
              setDcRpm(e.target.value);
            }}
          />
          <span>RPM</span>
          <button
            disabled={!motorsEnabled}
            style={button()}
            onClick={() => send(() => commands.dcRpm(parseInput(dcRpm)))}
          >
            Hold
          </button>
        </div>
        <div style={{ ...row, marginTop: 4 }}>
          {DC_RPM_PRESETS.map((rpm) => (
            <button
              key={rpm}
              disabled={!motorsEnabled}
              style={{ ...button(), padding: "4px 7px" }}
              onClick={() => send(() => commands.dcRpm(rpm))}
            >
              {rpm} rpm
            </button>
          ))}
        </div>
        <div style={{ marginTop: 6, color: toneColor(dc.tone), fontWeight: 600 }} role="status">
          {dc.text}
        </div>
      </div>

      {/* Stepper */}
      <div style={card}>
        <div style={{ ...row, justifyContent: "space-between" }}>
          <span style={label}>Stepper</span>
          <span>
            actual <span style={value}>{fmt(state.stepperDeg, 2)}°</span> / target{" "}
            <span style={value}>{fmt(state.stepperTargetDeg, 2)}°</span>
          </span>
        </div>
        <div style={{ ...row, marginTop: 6 }}>
          <button
            disabled={!motorsEnabled}
            style={button()}
            onClick={() => send(() => commands.stepper(-parseInput(stepDeg)))}
          >
            − {stepDeg}°
          </button>
          <input
            style={input}
            value={stepDeg}
            disabled={!motorsEnabled}
            onChange={(e) => {
              setStepDeg(e.target.value);
            }}
          />
          <button
            disabled={!motorsEnabled}
            style={button()}
            onClick={() => send(() => commands.stepper(parseInput(stepDeg)))}
          >
            + {stepDeg}°
          </button>
        </div>
      </div>

      {/* SENSOR-mode mapping: what each sensor is doing to its motor right now */}
      <div style={{ ...card, opacity: sensorMode ? 1 : 0.6 }}>
        <div style={label}>SENSOR mode mapping{sensorMode ? " (active)" : ""}</div>
        <div
          style={{
            display: "grid",
            gridTemplateColumns: "auto 1fr",
            gap: "4px 10px",
            marginTop: 6,
          }}
        >
          <span>TOF swipe</span>
          <span>
            {state.tofFault === true ? "no target" : `${fmt(state.tofRawMm, 0)} mm`} → servo sweeps
            at {SENSOR_MAP.swipeServoDegS}°/s, now <span style={value}>{fmt(state.servoDeg)}°</span>
          </span>
          <span>Flex bend</span>
          <span>
            <span style={value}>{fmt(state.flexFiltDeg)}°</span> → DC{" "}
            {sensorMode && state.dcMode === 1 ? (
              <span style={value}>{fmt(state.dcTarget)} rpm</span>
            ) : (
              `0–${SENSOR_MAP.dcMaxRpm} rpm`
            )}{" "}
            (dead zone below {SENSOR_MAP.flexDeadzoneDeg}°, full at {SENSOR_MAP.flexFullDeg}°)
          </span>
          <span>Slot pass</span>
          <span>
            count <span style={value}>{fmt(state.slotCount, 0)}</span> → stepper +
            {SENSOR_MAP.slotJogDeg}° each, target{" "}
            <span style={value}>{fmt(state.stepperTargetDeg, 2)}°</span>
          </span>
        </div>
      </div>

      {/* Command feedback: what the panel sent, and what the bridge saw happen */}
      <div style={{ fontSize: 12, display: "flex", flexDirection: "column", gap: 2 }}>
        {status && (
          <div style={{ color: status.error ? RED : t.muted }} role="status">
            {status.text}
          </div>
        )}
        {[...(state.statuses ?? [])].reverse().map((s) => (
          <div key={`${s.atMs}-${s.text}`} style={{ color: toneColor(statusTone(s.text)) }}>
            {s.text}
          </div>
        ))}
      </div>
    </div>
  );
}

export function initControlPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<ControlPanel context={context} />);
  return () => {
    root.unmount();
  };
}
