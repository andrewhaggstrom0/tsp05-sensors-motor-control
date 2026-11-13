// Unit tests for the panel's command and state logic. Runs on plain Node 18+:
//   npm test
import assert from "node:assert/strict";
import { test } from "node:test";

import {
  CMD,
  CommandError,
  SCHEMA,
  STALE_MS,
  STATE,
  applyMessages,
  commands,
  datatypesFor,
  isLive,
  parseInput,
  stopSequence,
  DC_MOVE_PRESETS,
  DC_RPM_PRESETS,
  DC_SETTLE_WARN_MS,
  STATUS_TOPIC,
  dcView,
  linkView,
  servoSliderValue,
  statusTone,
} from "./commands";

test("each command goes to its topic with the protocol's 1-decimal value", () => {
  assert.deepEqual(commands.mode(1), { topic: "/tsp05/cmd/mode", message: { data: 1 } });
  assert.deepEqual(commands.servo(45.06), {
    topic: "/tsp05/cmd/servo_deg",
    message: { data: 45.1 },
  });
  assert.deepEqual(commands.dcMove(-90), {
    topic: "/tsp05/cmd/dc_move_deg",
    message: { data: -90 },
  });
  assert.deepEqual(commands.dcRpm(30.25), { topic: "/tsp05/cmd/dc_rpm", message: { data: 30.3 } });
  assert.deepEqual(commands.dcStop(), { topic: "/tsp05/cmd/dc_stop", message: {} });
  assert.deepEqual(commands.stepper(10), {
    topic: "/tsp05/cmd/stepper_deg",
    message: { data: 10 },
  });
});

test("servo accepts both extremes and rejects values outside 0..180", () => {
  assert.equal(commands.servo(0).message.data, 0);
  assert.equal(commands.servo(180).message.data, 180);
  assert.throws(() => commands.servo(-1), CommandError);
  assert.throws(() => commands.servo(181), CommandError);
});

test("NaN, infinity and zero moves never become commands", () => {
  for (const make of [commands.servo, commands.dcMove, commands.dcRpm, commands.stepper]) {
    assert.throws(() => make(NaN), CommandError);
    assert.throws(() => make(Infinity), CommandError);
  }
  assert.throws(() => commands.dcMove(0), CommandError);
  assert.throws(() => commands.stepper(0.01), CommandError); // rounds to 0
  assert.throws(() => commands.dcRpm(500), CommandError);
});

test("STOP in SENSOR mode first switches to GUI mode", () => {
  assert.deepEqual(stopSequence(0), [commands.dcStop()]);
  assert.deepEqual(stopSequence(undefined), [commands.dcStop()]);
  assert.deepEqual(stopSequence(1), [commands.mode(0), commands.dcStop()]);
});

test("parseInput accepts plain numbers only", () => {
  assert.equal(parseInput(" 90 "), 90);
  assert.equal(parseInput("-45.5"), -45.5);
  assert.equal(parseInput("+10"), 10);
  assert.equal(parseInput(".5"), 0.5);
  for (const bad of ["", " ", "abc", "1e3", "12abc", "--1", "1.2.3"]) {
    assert.ok(Number.isNaN(parseInput(bad)), bad);
  }
});

test("every command topic has a schema with a ROS 2 definition", () => {
  for (const topic of Object.values(CMD)) {
    const schema = SCHEMA[topic];
    const types = datatypesFor(schema);
    assert.ok(types.has(schema), schema);
    assert.ok(types.has(schema.replace("/msg/", "/")), "short name");
  }
  assert.equal(
    datatypesFor("std_msgs/msg/Float32").get("std_msgs/msg/Float32")?.definitions[0]?.type,
    "float32",
  );
  assert.deepEqual(datatypesFor("std_msgs/msg/Empty").get("std_msgs/msg/Empty")?.definitions, []);
});

test("applyMessages keeps the latest value per topic and ignores others", () => {
  const s0 = {};
  const s1 = applyMessages(
    s0,
    [
      { topic: STATE.servoDeg, message: { data: 10 } },
      { topic: STATE.servoDeg, message: { data: 20 } },
      { topic: STATE.interlock, message: { data: true } },
      { topic: "/tsp05/some/other_topic", message: { data: 500 } },
      { topic: STATE.mode, message: { data: "1" } }, // wrong type: ignored
    ],
    1000,
  );
  assert.deepEqual(s1, { servoDeg: 20, interlock: true, lastMessageMs: 1000 });
  assert.equal(applyMessages(s1, [], 2000), s1, "no messages: same object, no re-render");
});

test("isLive needs the connected flag and recent telemetry", () => {
  const s = applyMessages(
    {},
    [
      { topic: STATE.connected, message: { data: true } },
      { topic: STATE.servoDeg, message: { data: 90 } }, // telemetry, not just the bridge flag
    ],
    1000,
  );
  assert.equal(isLive(s, 1000 + STALE_MS - 1), true);
  assert.equal(isLive(s, 1000 + STALE_MS), false);
  assert.equal(isLive({ ...s, connected: false }, 1000), false);
});

// ---------------------------------------------------------------- feedback (PR: bridge feedback)
test("command statuses are kept, newest last, at most 4", () => {
  let s = applyMessages({}, [], 0);
  for (let i = 1; i <= 6; i++) {
    s = applyMessages(
      s,
      [{ topic: STATUS_TOPIC, message: { data: `S,${i}.0: applied in 30 ms` } }],
      i,
    );
  }
  assert.deepEqual(
    s.statuses?.map((x) => x.text),
    [
      "S,3.0: applied in 30 ms",
      "S,4.0: applied in 30 ms",
      "S,5.0: applied in 30 ms",
      "S,6.0: applied in 30 ms",
    ],
  );
});

test("bridge health topics do not make a silent Uno look alive", () => {
  const s0 = applyMessages(
    {},
    [
      { topic: STATE.connected, message: { data: true } },
      { topic: STATE.servoDeg, message: { data: 90 } },
    ],
    1000,
  );
  assert.equal(isLive(s0, 1500), true);
  // The Uno goes quiet; the bridge keeps publishing rate/age/connected.
  const s1 = applyMessages(
    s0,
    [
      { topic: STATE.connected, message: { data: true } },
      { topic: STATE.rateHz, message: { data: 0 } },
      { topic: STATE.lineAgeMs, message: { data: 2500 } },
    ],
    3500,
  );
  assert.equal(isLive(s1, 3500), false);
  assert.equal(linkView(s1, 3500).text, "No telemetry");
});

test("linkView shows the rate and warns below 15 Hz", () => {
  const live = applyMessages(
    {},
    [
      { topic: STATE.connected, message: { data: true } },
      { topic: STATE.servoDeg, message: { data: 90 } },
      { topic: STATE.rateHz, message: { data: 20 } },
      { topic: STATE.lineAgeMs, message: { data: 30 } },
    ],
    0,
  );
  assert.deepEqual(linkView(live, 100), { text: "Uno connected · 20.0 Hz", tone: "good" });
  assert.equal(linkView({ ...live, rateHz: 9.5 }, 100).tone, "warn");
  assert.deepEqual(linkView({ ...live, rateHz: 0, lineAgeMs: 1240 }, 100), {
    text: "No telemetry for 1.2 s",
    tone: "bad",
  });
  assert.deepEqual(linkView({}, 0), { text: "Uno disconnected", tone: "bad" });
});

test("dcView: moving, arrived, not arrived in time, holding, stopped", () => {
  const pos = { dcMode: 2, dcTarget: 90, dcDeg: 40, dcSettled: false };
  assert.equal(dcView(pos, 1000, 900).tone, "neutral");
  assert.match(dcView(pos, 1000, 900).text, /^moving to 90.0°/);
  assert.equal(dcView(pos, 900 + DC_SETTLE_WARN_MS + 1, 900).tone, "warn");
  assert.deepEqual(dcView({ ...pos, dcDeg: 91, dcSettled: true }, 2000, 900), {
    text: "arrived ✓ at 91.0° (target 90.0°)",
    tone: "good",
  });
  assert.equal(
    dcView({ dcMode: 1, dcTarget: -45, dcRpm: -44.2 }, 0, undefined).text,
    "holding -45.0 rpm (now -44.2)",
  );
  assert.equal(dcView({ dcMode: 0 }, 0, undefined).text, "stopped");
  assert.equal(dcView({}, 0, undefined).text, "—");
});

test("status tones", () => {
  assert.equal(statusTone("D,P,90.0: applied in 35 ms"), "good");
  assert.equal(statusTone("P,90.0: ignored (SENSOR mode)"), "warn");
  assert.equal(statusTone("S,120.0: no effect seen within 1 s"), "bad");
});

test("servo slider follows the live angle unless dragging", () => {
  assert.equal(servoSliderValue({ dragging: false, dragValue: 10, live: 89.6 }), 90);
  assert.equal(servoSliderValue({ dragging: true, dragValue: 10, live: 89.6 }), 10);
  assert.equal(servoSliderValue({ dragging: false, dragValue: 10, live: undefined }), 10);
});

test("DC presets are valid commands", () => {
  for (const deg of DC_MOVE_PRESETS) {
    assert.equal(commands.dcMove(deg).message.data, deg);
  }
  for (const rpm of DC_RPM_PRESETS) {
    assert.equal(commands.dcRpm(rpm).message.data, rpm);
  }
});
