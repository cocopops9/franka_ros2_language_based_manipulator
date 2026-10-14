// Lab: the object memory of move_to_pose.cpp under noisy observations.

import { h, slider, check, button, palette, makeCanvas, pointer, fmt, loop, logLine, seededRandom, gaussian } from "./ui.js";
import { ObjectMemory, MEMORY_UPDATE_THRESHOLD } from "../core/controller.js";

export const foot = "The update rule is ObjectMemory.observe(), a port of vision_callback(): store on first sight, overwrite only if the new point is more than 2 cm away. The observation noise is a model.";

export function mount(root) {
  const truth = { x: 0.5, y: 0.1, z: -0.07 };
  let sigma = 0.003;
  let rate = 15;
  let running = true;
  let threshold = MEMORY_UPDATE_THRESHOLD;
  let memory = new ObjectMemory();
  let observations = [];
  let stats = { obs: 0, updates: 0, maxLag: 0 };
  let acc = 0;
  let t = 0;
  const rand = seededRandom(42);

  const canvasBox = h("div");
  const side = h("div", { class: "stack" });
  root.appendChild(h("div", { class: "grid-2" }, canvasBox, side));
  canvasBox.appendChild(h("div", { class: "small muted", style: { marginBottom: "6px" } }, "Top view, 30 x 30 cm. Drag the orange object; grey dots are observations, the red ring is the stored entry with its 2 cm dead band."));
  const span = 0.3;
  const cx0 = 0.5, cy0 = 0.1;
  const cv = makeCanvas(canvasBox, 1, draw, { minHeight: 260 });

  function S(x, y, w) { const s = w / span; return [w / 2 - (y - cy0) * s, w / 2 - (x - cx0) * s]; }
  function W(px, py, w) { const s = w / span; return [cx0 - (py - w / 2) / s, cy0 - (px - w / 2) / s]; }

  function draw(ctx, w, hgt) {
    const c = palette();
    ctx.clearRect(0, 0, w, hgt);
    const s = w / span;
    ctx.strokeStyle = c.border;
    for (let k = -0.15; k <= 0.151; k += 0.01) {
      const strong = Math.abs((k * 100) % 5) < 1e-6;
      ctx.globalAlpha = strong ? 0.9 : 0.35;
      ctx.beginPath(); ctx.moveTo(0, w / 2 + k * s); ctx.lineTo(w, w / 2 + k * s); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(w / 2 + k * s, 0); ctx.lineTo(w / 2 + k * s, hgt); ctx.stroke();
    }
    ctx.globalAlpha = 1;
    observations.forEach((o, i) => {
      const [px, py] = S(o[0], o[1], w);
      ctx.globalAlpha = 0.15 + 0.6 * (i / observations.length);
      ctx.fillStyle = c.faint;
      ctx.beginPath(); ctx.arc(px, py, 2.4, 0, 2 * Math.PI); ctx.fill();
    });
    ctx.globalAlpha = 1;
    const m = memory.items.get("fork");
    if (m) {
      const [mx, my] = S(m.x, m.y, w);
      ctx.strokeStyle = c.warn; ctx.setLineDash([5, 4]); ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(mx, my, threshold * s, 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
      ctx.lineWidth = 3; ctx.beginPath(); ctx.arc(mx, my, 7, 0, 2 * Math.PI); ctx.stroke(); ctx.lineWidth = 1;
    }
    const [tx, ty] = S(truth.x, truth.y, w);
    ctx.fillStyle = c.ctrl; ctx.beginPath(); ctx.arc(tx, ty, 8, 0, 2 * Math.PI); ctx.fill();
    ctx.fillStyle = c.text; ctx.font = "600 12px Inter, sans-serif"; ctx.fillText("fork (true)", tx + 12, ty - 10);
    ctx.fillStyle = c.faint; ctx.font = "11px 'JetBrains Mono', monospace";
    ctx.fillText("1 cm grid", 8, hgt - 8);
  }

  let dragging = false;
  cv.canvas.addEventListener("pointerdown", (ev) => { dragging = true; cv.canvas.setPointerCapture(ev.pointerId); move(ev); });
  cv.canvas.addEventListener("pointermove", (ev) => { if (dragging) move(ev); });
  cv.canvas.addEventListener("pointerup", () => { dragging = false; });
  function move(ev) {
    const p = pointer(ev, cv.canvas);
    const [x, y] = W(p.x, p.y, cv.width);
    truth.x = Math.max(cx0 - 0.14, Math.min(cx0 + 0.14, x));
    truth.y = Math.max(cy0 - 0.14, Math.min(cy0 + 0.14, y));
  }

  const statsBox = h("dl", { class: "kv" });
  const logEl = h("div", { class: "log", style: { height: "180px" } });
  side.appendChild(h("div", { class: "panel" }, h("h4", {}, "Observation stream"),
    slider({ label: "noise (1 sigma, per axis)", min: 0, max: 0.012, step: 0.0005, value: sigma, format: (v) => (v * 1000).toFixed(1) + " mm", onInput: (v) => { sigma = v; } }),
    slider({ label: "frames per second", min: 1, max: 30, step: 1, value: rate, format: (v) => v.toFixed(0), onInput: (v) => { rate = v; } }),
    slider({ label: "dead band (code: 2 cm)", min: 0.005, max: 0.05, step: 0.001, value: threshold, format: (v) => (v * 100).toFixed(1) + " cm", onInput: (v) => { threshold = v; } }),
    h("div", { class: "row", style: { marginTop: "8px" } },
      check("camera sees the object", running, (v) => { running = v; }),
      button("Clear memory", () => { memory = new ObjectMemory(); observations = []; stats = { obs: 0, updates: 0, maxLag: 0 }; logEl.innerHTML = ""; }, "small"))));
  side.appendChild(h("div", { class: "panel" }, h("h4", {}, "State"), statsBox));
  side.appendChild(h("div", {}, h("div", { class: "small muted", style: { marginBottom: "6px" } }, "move_to_pose_node log"), logEl));

  function observe() {
    const o = [truth.x + sigma * gaussian(rand), truth.y + sigma * gaussian(rand), truth.z + sigma * gaussian(rand)];
    observations.push(o);
    if (observations.length > 60) observations.shift();
    // ObjectMemory uses the fixed threshold of the code; emulate other thresholds for exploration.
    const old = memory.items.get("fork");
    let result;
    if (!old) { memory.items.set("fork", { x: o[0], y: o[1], z: o[2] }); result = "stored"; }
    else if (threshold === MEMORY_UPDATE_THRESHOLD) result = memory.observe("fork", ...o);
    else if (Math.hypot(o[0] - old.x, o[1] - old.y, o[2] - old.z) > threshold) { old.x = o[0]; old.y = o[1]; old.z = o[2]; result = "updated"; }
    else result = "kept";
    stats.obs += 1;
    if (result === "stored") logLine(logEl, t.toFixed(2), "[VISION] New object stored (Transformed): fork");
    if (result === "updated") { stats.updates += 1; logLine(logEl, t.toFixed(2), "[VISION] Memory updated for: fork"); }
  }

  loop(root, (dt) => {
    t += dt;
    if (running) {
      acc += dt;
      while (acc >= 1 / rate) { acc -= 1 / rate; observe(); }
    }
    const m = memory.items.get("fork");
    const lag = m ? Math.hypot(m.x - truth.x, m.y - truth.y, m.z - truth.z) : NaN;
    if (m) stats.maxLag = Math.max(stats.maxLag, lag);
    statsBox.innerHTML = "";
    const add = (k, v) => { statsBox.appendChild(h("dt", {}, k)); statsBox.appendChild(h("dd", {}, v)); };
    add("observations", String(stats.obs));
    add("memory updates", String(stats.updates) + (stats.obs ? ` (${((100 * stats.updates) / stats.obs).toFixed(1)} % of frames)` : ""));
    add("stored entry", m ? `(${fmt(m.x)}, ${fmt(m.y)}, ${fmt(m.z)})` : "none");
    add("|true - stored|", m ? (lag * 1000).toFixed(1) + " mm" : "n/a");
    add("largest lag so far", m ? (stats.maxLag * 1000).toFixed(1) + " mm" : "n/a");
    cv.redraw();
  });
}
