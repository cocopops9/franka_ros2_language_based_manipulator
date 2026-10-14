// Lab: edit a mission file and replay the controller's scheduling loop.
// The wait loop and the memory lookup are executed step by step exactly as in
// move_to_pose.cpp; only the durations of motions, gripper actions and the human
// reaction are modelled with the sliders below.

import { h, slider, check, palette, makeCanvas, fmt, roundRect } from "./ui.js";
import { DEFAULT_MISSION, EXAMPLE_OBJECTS, pickPoses, TRAVEL_TIME_OFFSET, PRE_GRASP_PAUSE, POST_GRASP_PAUSE, POST_RELEASE_PAUSE } from "../core/controller.js";

export const foot = "Exact: file order, the startTime - 5 s rule, the 100 ms / 1005 ms polling of the wait loop, the 5 s memory lookup and the fixed sleeps. Modelled: motion, gripper and human durations.";

function simulate(text, visible, model) {
  const out = { log: [], tasks: [], warnings: [], error: null, end: 0 };
  let data;
  try {
    data = JSON.parse(text);
  } catch (e) {
    out.error = "JSON syntax error: " + e.message;
    return out;
  }
  const seq = Array.isArray(data.sequence) ? data.sequence : [];
  const tools = Array.isArray(data.tools) ? data.tools.map(String) : [];
  const log = (t, msg, cls = "") => out.log.push([t, msg, cls]);
  log(0, `[SYSTEM] Loading Project: ${data.project !== undefined ? data.project : "Unknown"}`);
  if (!seq.length) {
    log(0, "Mission is empty or file not found! Exiting.", "err");
    out.error = "The sequence is empty: the controller exits.";
    return out;
  }
  log(0, `>>> PROJECT STARTED: ${seq.length} Tasks Loaded <<<`);

  let prev = -Infinity;
  for (const item of seq) {
    const raw = Number(item.startTime);
    if (!Number.isInteger(raw)) out.warnings.push(`startTime ${item.startTime} of '${item.tool}' is truncated to ${Math.trunc(raw)}.`);
    if (Math.trunc(raw) < prev) out.warnings.push(`'${item.tool}' has a smaller startTime than the previous task; tasks still run in file order.`);
    prev = Math.trunc(raw);
    if (!tools.includes(String(item.tool))) out.warnings.push(`'${item.tool}' is not in tools[]: the detector has no such class, so the task will be skipped.`);
  }

  let t = 0;
  for (const item of seq) {
    const tool = String(item.tool);
    const start = Math.trunc(Number(item.startTime));
    const task = { tool, start, threshold: start - TRAVEL_TIME_OFFSET, readyAt: t };
    log(t, `--- NEXT TASK: Pick ${tool} at T=${start}s ---`);
    // 1. time wait loop
    for (let guard = 0; guard < 100000; guard++) {
      const until = start - t - TRAVEL_TIME_OFFSET;
      if (until <= 0) break;
      if (Math.floor(t) % 5 === 0) {
        log(t, `Waiting... T-minus ${until.toFixed(1)} sec`, "dim");
        t += 1.005;
      } else {
        t += 0.1;
      }
    }
    task.dispatch = t;
    // 2. memory lookup
    const detectable = tools.includes(tool) && visible[tool] !== false;
    if (!detectable) {
      log(t, `Scanning for '${tool}'...`, "warn");
      t += 50 * 0.1;
      log(t, `SKIPPING: Object '${tool}' never found in vision memory!`, "err");
      task.skipped = true;
      task.end = t;
      out.tasks.push(task);
      continue;
    }
    // 3. cycle
    const step = (dur, msg, cls = "") => { log(t, msg, cls); t += dur; };
    step(model.move, "PLANNING: Intermediate...");
    const ex = EXAMPLE_OBJECTS[tool];
    const pk = ex ? pickPoses(...ex).pick.position : null;
    step(PRE_GRASP_PAUSE, pk ? `Moving to ${tool}: (${pk.x.toFixed(3)}, ${pk.y.toFixed(3)}, ${pk.z.toFixed(3)}, 0.71, 0.71, 0.00)  [example position]`
      : `Moving to ${tool}: (pick pose of the stored position, 0.71, 0.71, 0.00)`);
    step(model.move, "PLANNING: Pre-Grasp...");
    step(model.move, "PLANNING: Pick...");
    step(model.grasp, "ACTION: Grasping (Waiting)...");
    step(POST_GRASP_PAUSE, "Grasp Successful.");
    step(model.move, "PLANNING: Retract...");
    step(model.move, "PLANNING: Intermediate...");
    step(model.move, "PLANNING: Release Zone...");
    step(model.human, "WAITING FOR RELEASE...", "warn");
    step(0, "[HMI] RELEASE SIGNAL RECEIVED!", "hmi");
    step(model.open, "ACTION: Opening Gripper (Waiting for completion)...");
    step(POST_RELEASE_PAUSE, "Gripper Opened Successfully.");
    task.end = t;
    out.tasks.push(task);
  }
  log(t, "Project Complete. Going Home.");
  out.end = t;
  return out;
}

export function mount(root) {
  const model = { move: 2.0, grasp: 1.5, open: 1.0, human: 4.0 };
  const visible = {};
  const textarea = h("textarea", { rows: 12, spellcheck: "false", class: "text-input" });
  textarea.value = JSON.stringify(DEFAULT_MISSION, null, 2);
  const toggles = h("div", { class: "row" });
  const warnings = h("div", { class: "stack small" });
  const summary = h("div", { class: "small muted" });
  const logEl = h("div", { class: "log" });

  const left = h("div", { class: "stack" },
    h("div", { class: "field" }, "shared/sequence.json", textarea),
    h("div", { class: "panel" }, h("h4", {}, "Seen by the camera (with marker)"), toggles),
    h("div", { class: "panel" }, h("h4", {}, "Duration model"),
      slider({ label: "each Cartesian move", min: 0.5, max: 6, step: 0.1, value: model.move, format: (v) => v.toFixed(1) + " s", onInput: (v) => { model.move = v; update(); } }),
      slider({ label: "grasp action", min: 0.2, max: 4, step: 0.1, value: model.grasp, format: (v) => v.toFixed(1) + " s", onInput: (v) => { model.grasp = v; update(); } }),
      slider({ label: "open action", min: 0.2, max: 4, step: 0.1, value: model.open, format: (v) => v.toFixed(1) + " s", onInput: (v) => { model.open = v; update(); } }),
      slider({ label: "operator says release after", min: 0, max: 20, step: 0.5, value: model.human, format: (v) => v.toFixed(1) + " s", onInput: (v) => { model.human = v; update(); } }),
    ),
  );
  const right = h("div", { class: "stack" });
  root.appendChild(h("div", { class: "grid-2" }, left, right));

  let result = null;
  const chart = makeCanvas(right, 1.35, draw, { minHeight: 260 });
  right.appendChild(summary);
  right.appendChild(warnings);
  root.appendChild(h("div", { style: { marginTop: "14px" } }, h("div", { class: "small muted", style: { marginBottom: "6px" } }, "Controller log (simulated time)"), logEl));

  function draw(ctx, w, hgt) {
    const c = palette();
    ctx.clearRect(0, 0, w, hgt);
    if (!result || result.error && !result.tasks.length) {
      ctx.fillStyle = c.warn;
      ctx.font = "14px Inter, sans-serif";
      ctx.fillText(result ? result.error : "", 16, 30);
      return;
    }
    const tasks = result.tasks;
    const tMin = Math.min(-6, ...tasks.map((k) => k.threshold - 1));
    const tMax = Math.max(result.end + 4, ...tasks.map((k) => k.start + 2));
    const x0 = 70, x1 = w - 16, top = 26;
    const rowH = Math.min(54, (hgt - 70) / Math.max(1, tasks.length));
    const X = (t) => x0 + ((t - tMin) / (tMax - tMin)) * (x1 - x0);
    ctx.font = "11px 'JetBrains Mono', monospace";
    ctx.fillStyle = c.faint;
    const stepT = tMax - tMin > 150 ? 20 : 10;
    for (let t = Math.ceil(tMin / stepT) * stepT; t <= tMax; t += stepT) {
      ctx.strokeStyle = c.border;
      ctx.beginPath(); ctx.moveTo(X(t), top - 8); ctx.lineTo(X(t), top + rowH * tasks.length); ctx.stroke();
      ctx.fillText(t + " s", X(t) - 10, top + rowH * tasks.length + 16);
    }
    tasks.forEach((k, i) => {
      const y = top + i * rowH + rowH / 2;
      ctx.fillStyle = c.text;
      ctx.font = "600 12.5px Inter, sans-serif";
      ctx.fillText(k.tool, 8, y + 4);
      // waiting interval (task ready but threshold not reached)
      if (k.dispatch > k.readyAt + 0.01) {
        ctx.strokeStyle = c.faint; ctx.setLineDash([3, 3]);
        ctx.beginPath(); ctx.moveTo(X(k.readyAt), y); ctx.lineTo(X(k.dispatch), y); ctx.stroke(); ctx.setLineDash([]);
      }
      // threshold and requested start
      ctx.strokeStyle = c.mission; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.moveTo(X(k.threshold), y - rowH * 0.35); ctx.lineTo(X(k.threshold), y + rowH * 0.35); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = c.mission;
      ctx.beginPath(); ctx.moveTo(X(k.start), y - 8); ctx.lineTo(X(k.start) + 6, y); ctx.lineTo(X(k.start), y + 8); ctx.lineTo(X(k.start) - 6, y); ctx.fill();
      // execution bar
      ctx.fillStyle = k.skipped ? c.warn : c.ctrl;
      ctx.globalAlpha = k.skipped ? 0.55 : 0.9;
      roundRect(ctx, X(k.dispatch), y - 9, Math.max(3, X(k.end) - X(k.dispatch)), 18, 5);
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.fillStyle = k.skipped ? c.warn : c.text;
      ctx.font = "11px 'JetBrains Mono', monospace";
      const late = k.dispatch - Math.max(k.threshold, 0);
      const label = k.skipped ? "skipped after 5 s" : `${fmt(k.dispatch, 1)} to ${fmt(k.end, 1)} s` + (late > 0.2 ? `  (+${fmt(late, 1)} s)` : "");
      ctx.fillText(label, Math.min(X(k.dispatch) + 4, x1 - 200), y + 22);
    });
  }

  function rebuildToggles(text) {
    let names = [];
    try {
      const d = JSON.parse(text);
      names = [...new Set([...(d.tools || []), ...((d.sequence || []).map((s) => s.tool))].map(String))];
    } catch (e) {
      return;
    }
    toggles.innerHTML = "";
    for (const n of names) {
      if (!(n in visible)) visible[n] = true;
      toggles.appendChild(check(n, visible[n], (v) => { visible[n] = v; update(); }));
    }
  }

  function update() {
    result = simulate(textarea.value, visible, model);
    warnings.innerHTML = "";
    if (result.error) warnings.appendChild(h("div", { class: "badge warn" }, result.error));
    for (const w of result.warnings) warnings.appendChild(h("div", {}, h("span", { class: "badge warn" }, "note"), " ", w));
    summary.textContent = result.error ? "" :
      `Mission ends at ${fmt(result.end, 1)} s (before the joint-space move to "ready"). One complete cycle lasts ` +
      `${fmt(6 * model.move + model.grasp + model.open + model.human + PRE_GRASP_PAUSE + POST_GRASP_PAUSE + POST_RELEASE_PAUSE, 1)} s with these durations.`;
    logEl.innerHTML = "";
    for (const [t, msg, cls] of result.log) {
      logEl.appendChild(h("div", {}, h("span", { class: "t" }, fmt(t, 2).padStart(7, " ")), h("span", { class: cls }, msg)));
    }
    chart.redraw();
  }

  textarea.addEventListener("input", () => { rebuildToggles(textarea.value); update(); });
  rebuildToggles(textarea.value);
  update();
}
