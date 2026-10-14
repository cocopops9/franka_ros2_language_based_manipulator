// Lab: the whole controller loop of move_to_pose.cpp, step for step, in simulated time.
// The program below follows main() line by line (wait loop, memory lookup, waypoints,
// sleeps, release flag); motions and gripper actions take modelled durations.

import { h, button, check, slider, palette, makeCanvas, pointer, fmt, loop, logLine, roundRect } from "./ui.js";
import { DEFAULT_MISSION, EXAMPLE_OBJECTS, READY_TCP, INTERMEDIATE_POSE, RELEASE_POSE_RIGHT, pickPoses, ObjectMemory,
  TRAVEL_TIME_OFFSET, PRE_GRASP_PAUSE, POST_GRASP_PAUSE, POST_RELEASE_PAUSE, MIN_PATH_FRACTION, GRIPPER_GRASP_GOAL,
  GRIPPER_OPEN_GOAL, CAMERA_TRANSLATION, CAMERA_ROTATION, applyRotation, TABLE_Z, EXAMPLE_K } from "../core/controller.js";

export const foot = "Exact: order of operations, log lines, wait-loop timing, 5 s lookup, fixed sleeps, pick offsets, grasp tolerance, the release flag. Modelled: motion speed, gripper durations, a reach limit of 0.2 m to 0.855 m from the base axis, noiseless vision at 10 Hz (objects in the gripper are not observed), and a grasp that holds the object only if it is still within 3 cm of the remembered position.";

const P = (pose) => [pose.position.x, pose.position.y, pose.position.z];

function smooth(u) { u = Math.max(0, Math.min(1, u)); return u * u * (3 - 2 * u); }

function reachable(p) {
  const r = Math.hypot(p[0], p[1]);
  return r > 0.2 && r < 0.855 && p[2] > -0.06 && p[2] < 0.9;
}

export function mount(root) {
  let sim;
  let speed = 4;
  let running = false;
  let autoRelease = true;
  let autoDelay = 3;
  let moveSpeed = 0.15;

  const missionArea = h("textarea", { rows: 8, spellcheck: "false", class: "text-input" });
  missionArea.value = JSON.stringify(DEFAULT_MISSION, null, 2);
  const visibleBox = h("div", { class: "row" });
  const statusBox = h("div", { class: "row" });
  const logEl = h("div", { class: "log", style: { height: "250px" } });
  const startBtn = button("Start", () => { if (!sim || sim.done) reset(); running = !running; startBtn.textContent = running ? "Pause" : "Resume"; }, "primary");
  const releaseBtn = button("Say \"release\"", () => publishRelease("operator"), "voice");

  const viewBox = h("div");
  const side = h("div", { class: "stack" });
  root.appendChild(h("div", { class: "row", style: { marginBottom: "10px" } }, startBtn, button("Restart", () => { reset(); running = true; startBtn.textContent = "Pause"; }),
    h("span", { class: "muted small" }, "speed"), ...[1, 4, 10].map((s) => h("button", { class: "chip", type: "button", onclick: () => { speed = s; } }, "x" + s)), releaseBtn));
  root.appendChild(statusBox);
  root.appendChild(h("div", { class: "grid-2", style: { marginTop: "10px" } }, viewBox, side));
  const cv = makeCanvas(viewBox, 0.95, draw, { minHeight: 280 });
  viewBox.appendChild(h("div", { class: "small muted", style: { marginTop: "6px" } }, "Top view of fr3_link0. Drag objects on the table, even after a task has been dispatched. Red dashed rings: memory entries (they never expire). Dashed circles around the base: simplified reach."));
  side.appendChild(h("div", { class: "panel" }, h("h4", {}, "Operator"),
    check("say \"release\" automatically", autoRelease, (v) => { autoRelease = v; }),
    slider({ label: "after waiting", min: 0, max: 15, step: 0.5, value: autoDelay, format: (v) => v.toFixed(1) + " s", onInput: (v) => { autoDelay = v; } })));
  side.appendChild(h("div", { class: "panel" }, h("h4", {}, "Camera sees (with marker)"), visibleBox));
  side.appendChild(h("div", { class: "panel" }, h("h4", {}, "Model"),
    slider({ label: "Cartesian speed", min: 0.05, max: 0.5, step: 0.01, value: moveSpeed, format: (v) => v.toFixed(2) + " m/s", onInput: (v) => { moveSpeed = v; } })));
  side.appendChild(h("details", { class: "panel" }, h("summary", { class: "small" }, "mission file (applied on Restart)"), missionArea));
  root.appendChild(h("div", { style: { marginTop: "12px" } }, h("div", { class: "small muted", style: { marginBottom: "6px" } }, "move_to_pose_node log (simulated seconds)"), logEl));

  function log(text, cls = "") { logLine(logEl, fmt(sim.t, 2).padStart(7), text, cls); }

  function publishRelease(who) {
    if (!sim) return;
    // intent_callback: any message containing "release" sets the flag and prints this line.
    sim.flag = true;
    log("[HMI] RELEASE SIGNAL RECEIVED!", "hmi");
    if (who === "operator" && sim.phase !== "wait for release") log("(the flag will be cleared when the next wait starts)", "dim");
  }

  function* moveTo(target, name) {
    log(`PLANNING: ${name}...`);
    const from = sim.tcp.slice();
    const L = Math.hypot(target[0] - from[0], target[1] - from[1], target[2] - from[2]);
    const steps = Math.floor(L / 0.01) + 1;
    let reached = 0;
    for (let i = 1; i <= steps; i++) {
      const s = i / steps;
      if (!reachable([from[0] + s * (target[0] - from[0]), from[1] + s * (target[1] - from[1]), from[2] + s * (target[2] - from[2])])) break;
      reached = i;
    }
    const fraction = reached / steps;
    if (!(fraction > MIN_PATH_FRACTION)) {
      log(`FAILED: ${name} (Coverage: ${(fraction * 100).toFixed(1)}%)`, "err");
      return false;
    }
    const end = [from[0] + fraction * (target[0] - from[0]), from[1] + fraction * (target[1] - from[1]), from[2] + fraction * (target[2] - from[2])];
    yield { motion: { from, to: end }, dur: Math.max(0.5, (L * fraction) / moveSpeed) };
    return true;
  }

  function* gripperTo(width, dur) {
    yield { gripper: { from: sim.grip, to: width }, dur };
  }

  function* program(mission) {
    log(`[SYSTEM] Loading Project: ${mission.project !== undefined ? mission.project : "Unknown"}`);
    const tasks = (mission.sequence || []).map((s) => ({ tool: String(s.tool), start: Math.trunc(Number(s.startTime)) }));
    if (!tasks.length) { log("Mission is empty or file not found! Exiting.", "err"); return; }
    log(`>>> PROJECT STARTED: ${tasks.length} Tasks Loaded <<<`);
    for (let k = 0; k < tasks.length; k++) {
      const { tool, start } = tasks[k];
      sim.task = `${k + 1}/${tasks.length}: ${tool}`;
      sim.phase = "wait for startTime - 5 s";
      log(`--- NEXT TASK: Pick ${tool} at T=${start}s ---`);
      for (;;) {
        const until = start - sim.t - TRAVEL_TIME_OFFSET;
        if (until <= 0) break;
        if (Math.floor(sim.t) % 5 === 0) { log(`Waiting... T-minus ${until.toFixed(1)} sec`, "dim"); yield { dur: 1.005 }; }
        else yield { dur: 0.1 };
      }
      sim.phase = "memory lookup";
      let target = null;
      for (let i = 0; i < 50; i++) {
        const m = sim.memory.items.get(tool);
        if (m) { target = [m.x, m.y, m.z]; break; }
        if (i === 0) log(`Scanning for '${tool}'...`, "warn");
        yield { dur: 0.1 };
      }
      if (!target) { log(`SKIPPING: Object '${tool}' never found in vision memory!`, "err"); continue; }
      sim.phase = "intermediate";
      yield* moveTo(P(INTERMEDIATE_POSE), "Intermediate");
      const { pick, preGrasp } = pickPoses(...target);
      const pk = [pick.position.x, pick.position.y, pick.position.z];
      const pg = [preGrasp.position.x, preGrasp.position.y, preGrasp.position.z];
      const q = pick.orientation;
      log(`Moving to ${tool}: (${pk[0].toFixed(3)}, ${pk[1].toFixed(3)}, ${pk[2].toFixed(3)}, ${q[0].toFixed(2)}, ${q[1].toFixed(2)}, ${q[2].toFixed(2)})`);
      sim.phase = "sleep 10 s";
      yield { dur: PRE_GRASP_PAUSE };
      sim.phase = "pre-grasp";
      if (!(yield* moveTo(pg, "Pre-Grasp"))) continue;
      sim.phase = "pick";
      if (!(yield* moveTo(pk, "Pick"))) continue;
      sim.phase = "grasp";
      log("ACTION: Grasping (Waiting)...");
      // The pick offsets compensate for physical errors that this simulation does not model, so an
      // object counts as "between the fingers" if it is still within 3 cm of the remembered position.
      const held = sim.objects.find((o) => o.label === tool && !o.gone && !o.held && Math.hypot(o.pos[0] - target[0], o.pos[1] - target[1]) < 0.03);
      const width = held ? held.width : 0.0;
      yield* gripperTo(Math.min(width, GRIPPER_OPEN_GOAL.width), 1.0);
      const ok = width > GRIPPER_GRASP_GOAL.width - GRIPPER_GRASP_GOAL.epsilonInner && width < GRIPPER_GRASP_GOAL.width + GRIPPER_GRASP_GOAL.epsilonOuter;
      log(ok ? "Grasp Successful." : "Grasp Failed.", ok ? "" : "warn");
      if (held) held.held = true; else log("(nothing between the fingers: the object was moved after the memory lookup)", "dim");
      yield { dur: POST_GRASP_PAUSE };
      sim.phase = "retract";
      yield* moveTo(pg, "Retract");
      sim.phase = "intermediate";
      yield* moveTo(P(INTERMEDIATE_POSE), "Intermediate");
      sim.phase = "release zone";
      yield* moveTo(P(RELEASE_POSE_RIGHT), "Release Zone");
      sim.phase = "wait for release";
      log("WAITING FOR RELEASE...", "warn");
      sim.flag = false;
      sim.waitStart = sim.t;
      while (!sim.flag) {
        if (autoRelease && sim.t - sim.waitStart >= autoDelay) publishRelease("auto");
        else yield { dur: 0.1 };
      }
      sim.phase = "open gripper";
      log("ACTION: Opening Gripper (Waiting for completion)...");
      yield* gripperTo(GRIPPER_OPEN_GOAL.width, 0.8);
      log("Gripper Opened Successfully.");
      for (const o of sim.objects) if (o.held) { o.held = false; o.gone = true; }
      yield { dur: POST_RELEASE_PAUSE };
    }
    sim.task = "done";
    sim.phase = "going home";
    log("Project Complete. Going Home.");
    yield { motion: { from: sim.tcp.slice(), to: READY_TCP.slice(), joint: true }, dur: 3 };
  }

  function reset() {
    let mission;
    try { mission = JSON.parse(missionArea.value); } catch (e) { mission = { sequence: [] }; }
    const labels = [...new Set([...(mission.tools || []), ...((mission.sequence || []).map((s) => String(s.tool)))])];
    const objects = labels.map((label, i) => {
      const p = EXAMPLE_OBJECTS[label] || [0.45 + 0.06 * i, -0.15 + 0.08 * i, -0.07];
      return { label, pos: p.slice(), width: label === "bottle" ? 0.03 : 0.012, visible: sim && sim.objects.find((o) => o.label === label) ? sim.objects.find((o) => o.label === label).visible : true, held: false, gone: false };
    });
    sim = { t: 0, tcp: READY_TCP.slice(), grip: GRIPPER_OPEN_GOAL.width, objects, memory: new ObjectMemory(), flag: false, phase: "start", task: "", done: false, current: null, visionAcc: 0 };
    logEl.innerHTML = "";
    sim.gen = program(mission);
    visibleBox.innerHTML = "";
    for (const o of objects) visibleBox.appendChild(check(o.label, o.visible, (v) => { o.visible = v; }));
    observe();
    advance();
  }

  function observe() {
    for (const o of sim.objects) {
      if (!o.visible || o.held || o.gone) continue;
      const r = sim.memory.observe(o.label, ...o.pos);
      if (r === "stored") log(`[VISION] New object stored (Transformed): ${o.label}`, "dim");
      if (r === "updated") log(`[VISION] Memory updated for: ${o.label}`, "dim");
    }
  }

  function advance() {
    const r = sim.gen.next(sim.lastResult);
    if (r.done) { sim.done = true; sim.current = null; running = false; startBtn.textContent = "Start"; return; }
    sim.current = { ...r.value, t0: sim.t, t1: sim.t + (r.value.dur || 0) };
  }

  function step(dt) {
    let remaining = dt;
    while (sim.current && remaining > 0) {
      const c = sim.current;
      const until = c.t1 - sim.t;
      const d = Math.min(remaining, Math.max(0, until));
      sim.t += d;
      remaining -= d;
      sim.visionAcc += d;
      while (sim.visionAcc >= 0.1) { sim.visionAcc -= 0.1; observe(); }
      const u = c.t1 > c.t0 ? (sim.t - c.t0) / (c.t1 - c.t0) : 1;
      if (c.motion) {
        const e = smooth(u);
        if (c.motion.joint) {
          const a0 = Math.atan2(c.motion.from[1], c.motion.from[0]), a1 = Math.atan2(c.motion.to[1], c.motion.to[0]);
          const r0 = Math.hypot(c.motion.from[0], c.motion.from[1]), r1 = Math.hypot(c.motion.to[0], c.motion.to[1]);
          const a = a0 + (a1 - a0) * e, rr = r0 + (r1 - r0) * e;
          sim.tcp = [rr * Math.cos(a), rr * Math.sin(a), c.motion.from[2] + (c.motion.to[2] - c.motion.from[2]) * e];
        } else {
          sim.tcp = c.motion.from.map((v, i) => v + (c.motion.to[i] - v) * e);
        }
      }
      if (c.gripper) sim.grip = c.gripper.from + (c.gripper.to - c.gripper.from) * Math.min(1, u);
      for (const o of sim.objects) if (o.held) o.pos = [sim.tcp[0], sim.tcp[1], sim.tcp[2]];
      if (sim.t >= c.t1 - 1e-9) advance();
    }
  }

  // ---- drawing
  const X0 = -0.12, X1 = 0.86, Y0 = -0.62, Y1 = 0.62;
  function S(x, y, w, hgt) {
    const s = Math.min(w / (Y1 - Y0), hgt / (X1 - X0));
    const ox = (w - s * (Y1 - Y0)) / 2, oy = (hgt - s * (X1 - X0)) / 2;
    return [ox + (Y1 - y) * s, oy + (X1 - x) * s, s];
  }
  function Wd(px, py, w, hgt) {
    const s = Math.min(w / (Y1 - Y0), hgt / (X1 - X0));
    const ox = (w - s * (Y1 - Y0)) / 2, oy = (hgt - s * (X1 - X0)) / 2;
    return [X1 - (py - oy) / s, Y1 - (px - ox) / s];
  }

  function draw(ctx, w, hgt) {
    const c = palette();
    ctx.clearRect(0, 0, w, hgt);
    if (!sim) return;
    const K = EXAMPLE_K;
    const corners = [[0, 0], [K.width, 0], [K.width, K.height], [0, K.height]].map(([u, v]) => {
      const d = applyRotation(CAMERA_ROTATION, [(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1]);
      const t = (TABLE_Z - CAMERA_TRANSLATION[2]) / d[2];
      return S(CAMERA_TRANSLATION[0] + t * d[0], CAMERA_TRANSLATION[1] + t * d[1], w, hgt);
    });
    ctx.fillStyle = c.vision; ctx.globalAlpha = 0.05;
    ctx.beginPath(); corners.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y))); ctx.closePath(); ctx.fill(); ctx.globalAlpha = 1;
    // reach ring
    const [bx, by, s] = S(0, 0, w, hgt);
    ctx.strokeStyle = c.border; ctx.setLineDash([4, 5]);
    ctx.beginPath(); ctx.arc(bx, by, 0.855 * s, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.arc(bx, by, 0.2 * s, 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
    // release and intermediate
    const rp = S(RELEASE_POSE_RIGHT.position.x, RELEASE_POSE_RIGHT.position.y, w, hgt);
    ctx.fillStyle = c.motion; ctx.globalAlpha = 0.15; ctx.beginPath(); ctx.arc(rp[0], rp[1], 0.05 * s, 0, 2 * Math.PI); ctx.fill(); ctx.globalAlpha = 1;
    ctx.fillStyle = c.muted; ctx.font = "11.5px Inter, sans-serif"; ctx.fillText("release pose", rp[0] - 30, rp[1] + 0.05 * s + 14);
    const ip = S(INTERMEDIATE_POSE.position.x, INTERMEDIATE_POSE.position.y, w, hgt);
    ctx.strokeStyle = c.ctrl; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(ip[0] - 5, ip[1] - 5); ctx.lineTo(ip[0] + 5, ip[1] + 5); ctx.moveTo(ip[0] + 5, ip[1] - 5); ctx.lineTo(ip[0] - 5, ip[1] + 5); ctx.stroke(); ctx.lineWidth = 1;
    // camera
    const cp = S(CAMERA_TRANSLATION[0], CAMERA_TRANSLATION[1], w, hgt);
    ctx.fillStyle = c.vision; roundRect(ctx, cp[0] - 12, cp[1] - 6, 24, 12, 3); ctx.fill();
    // memory entries
    for (const [label, m] of sim.memory.items) {
      const p = S(m.x, m.y, w, hgt);
      ctx.strokeStyle = c.warn; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.arc(p[0], p[1], 0.02 * s + 3, 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
      void label;
    }
    // objects
    for (const o of sim.objects) {
      if (o.gone) continue;
      const p = S(o.pos[0], o.pos[1], w, hgt);
      ctx.globalAlpha = o.visible ? 1 : 0.45;
      ctx.fillStyle = o.label === "bottle" ? "#7dd3fc" : "#cbd5e1";
      roundRect(ctx, p[0] - 16, p[1] - 7, 32, 14, 5); ctx.fill();
      ctx.fillStyle = "#0b1220"; ctx.font = "600 9.5px 'JetBrains Mono', monospace"; ctx.fillText(o.label.slice(0, 6), p[0] - 14, p[1] + 3.5);
      ctx.globalAlpha = 1;
    }
    // arm and TCP
    const tp = S(sim.tcp[0], sim.tcp[1], w, hgt);
    ctx.strokeStyle = c.text; ctx.globalAlpha = 0.35; ctx.lineWidth = 10; ctx.lineCap = "round";
    ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(tp[0], tp[1]); ctx.stroke(); ctx.globalAlpha = 1; ctx.lineWidth = 1;
    ctx.fillStyle = c.surface; ctx.strokeStyle = c.border; ctx.beginPath(); ctx.arc(bx, by, 14, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    ctx.strokeStyle = c.motion; ctx.lineWidth = 2.5; ctx.beginPath(); ctx.arc(tp[0], tp[1], 9, 0, 2 * Math.PI); ctx.stroke(); ctx.lineWidth = 1;
    // z gauge
    const gx = w - 26, gy0 = hgt - 20, gy1 = 30;
    const Z = (z) => gy0 - ((z - TABLE_Z) / (0.6 - TABLE_Z)) * (gy0 - gy1);
    ctx.strokeStyle = c.border; ctx.beginPath(); ctx.moveTo(gx, gy0); ctx.lineTo(gx, gy1); ctx.stroke();
    ctx.fillStyle = c.motion; ctx.beginPath(); ctx.arc(gx, Z(sim.tcp[2]), 5, 0, 2 * Math.PI); ctx.fill();
    ctx.fillStyle = c.faint; ctx.font = "10.5px 'JetBrains Mono', monospace"; ctx.textAlign = "right"; ctx.fillText("TCP z " + sim.tcp[2].toFixed(3), gx - 10, Z(sim.tcp[2]) + 4); ctx.textAlign = "left";
  }

  // drag objects
  let dragObj = null;
  cv.canvas.addEventListener("pointerdown", (ev) => {
    if (!sim) return;
    const p = pointer(ev, cv.canvas);
    let best = null, bd = 22;
    for (const o of sim.objects) {
      if (o.gone || o.held) continue;
      const q = S(o.pos[0], o.pos[1], cv.width, cv.height);
      const d = Math.hypot(q[0] - p.x, q[1] - p.y);
      if (d < bd) { bd = d; best = o; }
    }
    if (best) { dragObj = best; cv.canvas.setPointerCapture(ev.pointerId); }
  });
  cv.canvas.addEventListener("pointermove", (ev) => {
    if (!dragObj) return;
    const p = pointer(ev, cv.canvas);
    const [x, y] = Wd(p.x, p.y, cv.width, cv.height);
    dragObj.pos[0] = Math.max(0.1, Math.min(0.84, x));
    dragObj.pos[1] = Math.max(-0.6, Math.min(0.6, y));
  });
  cv.canvas.addEventListener("pointerup", () => { dragObj = null; });

  function renderStatus() {
    statusBox.innerHTML = "";
    statusBox.append(
      h("span", { class: "badge muted" }, "t = " + fmt(sim.t, 1) + " s"),
      h("span", { class: "badge ctrl" }, "task " + (sim.task || "none")),
      h("span", { class: "badge vision" }, sim.phase),
      h("span", { class: "badge muted" }, "gripper " + (sim.grip * 1000).toFixed(0) + " mm"),
      h("span", { class: "badge " + (sim.flag ? "voice" : "muted") }, "release_signal_received = " + sim.flag),
    );
  }

  reset();
  loop(root, (dt) => {
    if (running && sim && !sim.done) step(dt * speed);
    renderStatus();
    cv.redraw();
  });
}
