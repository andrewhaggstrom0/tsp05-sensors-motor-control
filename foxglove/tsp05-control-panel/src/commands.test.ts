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
      { topic: "/tsp05/tof/raw_mm", message: { data: 500 } },
      { topic: STATE.mode, message: { data: "1" } }, // wrong type: ignored
    ],
    1000,
  );
  assert.deepEqual(s1, { servoDeg: 20, interlock: true, lastMessageMs: 1000 });
  assert.equal(applyMessages(s1, [], 2000), s1, "no messages: same object, no re-render");
});

test("isLive needs the connected flag and recent telemetry", () => {
  const s = applyMessages({}, [{ topic: STATE.connected, message: { data: true } }], 1000);
  assert.equal(isLive(s, 1000 + STALE_MS - 1), true);
  assert.equal(isLive(s, 1000 + STALE_MS), false);
  assert.equal(isLive({ ...s, connected: false }, 1000), false);
});
