// Lab: the Cartesian-path algorithm on a planar two-link arm.
// Interpolation with a maximum step, IK per sample seeded by the previous solution,
// stop at the first failure, optional relative joint-jump test, execute if fraction > 0.9.

import { h, slider, button, palette, makeCanvas, pointer, fmt, loop } from "./ui.js";
import { CARTESIAN_EEF_STEP, MIN_PATH_FRACTION } from "../core/controller.js";

export const foot = "Illustration only: a 2-link planar arm (0.45 m and 0.35 m) replaces the 7-joint FR3, and collisions are not modelled. Step count, fraction and decision follow MoveIt's Cartesian interpolator as used by move_cartesian().";

const L1 = 0.45, L2 = 0.35;

function ikSolutions(x, y) {
  const r2 = x * x + y * y;
  const c2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (c2 < -1 || c2 > 1) return [];
  const out = [];
  for (const sgn of [1, -1]) {
    const q2 = sgn * Math.acos(c2);
    const q1 = Math.atan2(y, x) - Math.atan2(L2 * Math.sin(q2), L1 + L2 * Math.cos(q2));
    out.push([q1, q2]);
  }
  return out;
}

function angleDiff(a, b) {
  let d = a - b;
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return d;
}

function plan(start, goal, step, jumpThreshold) {
  const L = Math.hypot(goal[0] - start[0], goal[1] - start[1]);
  const steps = Math.floor(L / step) + 1;
  const seedSols = ikSolutions(...start);
  if (!seedSols.length) return { steps, samples: [], fraction: 0, startInvalid: true };
  let q = seedSols[0];
  const samples = [{ p: start, q, ok: true }];
  let reached = 0;
  for (let i = 1; i <= steps; i++) {
    const s = i / steps;
    const p = [start[0] + s * (goal[0] - start[0]), start[1] + s * (goal[1] - start[1])];
    const sols = ikSolutions(...p);
    if (!sols.length) { samples.push({ p, ok: false }); break; }
    sols.sort((a, b) => (Math.abs(angleDiff(a[0], q[0])) + Math.abs(angleDiff(a[1], q[1]))) - (Math.abs(angleDiff(b[0], q[0])) + Math.abs(angleDiff(b[1], q[1]))));
    q = sols[0];
    samples.push({ p, q, ok: true });
    reached = i;
  }
  let jumpAt = -1;
  if (jumpThreshold > 0 && samples.filter((s) => s.ok).length >= 10) {
    const ok = samples.filter((s) => s.ok);
    const d = [];
    for (let i = 1; i < ok.length; i++) d.push(Math.hypot(angleDiff(ok[i].q[0], ok[i - 1].q[0]), angleDiff(ok[i].q[1], ok[i - 1].q[1])));
    const mean = d.reduce((a, b) => a + b, 0) / d.length;
    for (let i = 0; i < d.length; i++) if (d[i] > jumpThreshold * mean) { jumpAt = i + 1; break; }
    if (jumpAt > 0) reached = Math.min(reached, jumpAt - 1);
  }
  return { steps, samples, fraction: reached / steps, jumpAt, length: L };
}

export function mount(root) {
  const start = [0.55, 0.35];
  const goal = [0.25, -0.45];
  let step = CARTESIAN_EEF_STEP, minFraction = MIN_PATH_FRACTION, jump = 0.0;
  let result = plan(start, goal, step, jump);
  let anim = null;

  const box = h("div");
  const side = h("div", { class: "stack" });
  root.appendChild(h("div", { class: "grid-2" }, box, side));
  box.appendChild(h("div", { class: "small muted", style: { marginBottom: "6px" } }, "Drag the green start or the orange target. Grey ring: unreachable region of the two-link arm."));
  const W0 = 1.9;
  const S = (p, w) => { const s = w / W0; return [w / 2 + p[0] * s, w / 2 - p[1] * s]; };
  const Wd = (x, y, w) => { const s = w / W0; return [(x - w / 2) / s, (w / 2 - y) / s]; };
  const cv = makeCanvas(box, 1, draw, { minHeight: 260 });

  function fk(q) {
    const e = [L1 * Math.cos(q[0]), L1 * Math.sin(q[0])];
    return [e, [e[0] + L2 * Math.cos(q[0] + q[1]), e[1] + L2 * Math.sin(q[0] + q[1])]];
  }

  function draw(ctx, w, hgt) {
    const c = palette();
    const s = w / W0;
    ctx.clearRect(0, 0, w, hgt);
    const o = S([0, 0], w);
    ctx.fillStyle = c.border; ctx.globalAlpha = 0.5;
    ctx.beginPath(); ctx.arc(o[0], o[1], (L1 - L2) * s, 0, 2 * Math.PI); ctx.fill();
    ctx.globalAlpha = 1;
    ctx.strokeStyle = c.border; ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.arc(o[0], o[1], (L1 + L2) * s, 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
    // straight line
    const a = S(start, w), b = S(goal, w);
    ctx.strokeStyle = c.faint; ctx.setLineDash([2, 4]);
    ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke(); ctx.setLineDash([]);
    // samples
    const okCount = Math.round(result.fraction * result.steps);
    result.samples.forEach((smp, i) => {
      const p = S(smp.p, w);
      if (!smp.ok) {
        ctx.strokeStyle = c.warn; ctx.lineWidth = 2.5;
        ctx.beginPath(); ctx.moveTo(p[0] - 6, p[1] - 6); ctx.lineTo(p[0] + 6, p[1] + 6); ctx.moveTo(p[0] + 6, p[1] - 6); ctx.lineTo(p[0] - 6, p[1] + 6); ctx.stroke();
        ctx.lineWidth = 1;
        return;
      }
      ctx.fillStyle = i <= okCount ? c.motion : c.warn;
      ctx.beginPath(); ctx.arc(p[0], p[1], 2.4, 0, 2 * Math.PI); ctx.fill();
    });
    if (result.samples.length < result.steps + 1) {
      const last = result.samples[result.samples.length - 1];
      for (let i = result.samples.length; i <= result.steps; i++) {
        const sp = i / result.steps;
        const p = S([start[0] + sp * (goal[0] - start[0]), start[1] + sp * (goal[1] - start[1])], w);
        ctx.fillStyle = c.faint; ctx.globalAlpha = 0.5; ctx.beginPath(); ctx.arc(p[0], p[1], 1.8, 0, 2 * Math.PI); ctx.fill(); ctx.globalAlpha = 1;
      }
      void last;
    }
    // arm
    const q = anim ? anim.q : (result.samples[0] && result.samples[0].q);
    if (q) {
      const [e, t] = fk(q);
      const pe = S(e, w), pt = S(t, w);
      ctx.strokeStyle = c.text; ctx.lineWidth = 9; ctx.lineCap = "round";
      ctx.beginPath(); ctx.moveTo(...o); ctx.lineTo(...pe); ctx.lineTo(...pt); ctx.stroke();
      ctx.lineWidth = 1;
      ctx.fillStyle = c.surface; ctx.strokeStyle = c.text;
      for (const p of [o, pe]) { ctx.beginPath(); ctx.arc(p[0], p[1], 6, 0, 2 * Math.PI); ctx.fill(); ctx.stroke(); }
    }
    ctx.fillStyle = c.motion; ctx.beginPath(); ctx.arc(a[0], a[1], 8, 0, 2 * Math.PI); ctx.fill();
    ctx.fillStyle = c.ctrl; ctx.beginPath(); ctx.arc(b[0], b[1], 8, 0, 2 * Math.PI); ctx.fill();
  }

  let drag = null;
  cv.canvas.addEventListener("pointerdown", (ev) => {
    const p = pointer(ev, cv.canvas);
    const da = Math.hypot(...[0, 1].map((i) => S(start, cv.width)[i] - [p.x, p.y][i]));
    const db = Math.hypot(...[0, 1].map((i) => S(goal, cv.width)[i] - [p.x, p.y][i]));
    drag = da < db ? start : goal;
    cv.canvas.setPointerCapture(ev.pointerId);
    movePoint(p);
  });
  cv.canvas.addEventListener("pointermove", (ev) => { if (drag) movePoint(pointer(ev, cv.canvas)); });
  cv.canvas.addEventListener("pointerup", () => { drag = null; });
  function movePoint(p) {
    const [x, y] = Wd(p.x, p.y, cv.width);
    drag[0] = Math.max(-0.92, Math.min(0.92, x));
    drag[1] = Math.max(-0.92, Math.min(0.92, y));
    anim = null;
    update();
  }

  const stats = h("dl", { class: "kv" });
  const verdict = h("div", {});
  side.appendChild(h("div", { class: "panel" }, h("h4", {}, "Parameters (code values preselected)"),
    slider({ label: "eef_step", min: 0.005, max: 0.1, step: 0.005, value: step, format: (v) => (v * 100).toFixed(1) + " cm", onInput: (v) => { step = v; update(); } }),
    slider({ label: "execute if fraction >", min: 0.5, max: 1, step: 0.01, value: minFraction, format: (v) => v.toFixed(2), onInput: (v) => { minFraction = v; update(); } }),
    slider({ label: "jump_threshold (0 = off)", min: 0, max: 6, step: 0.5, value: jump, format: (v) => v.toFixed(1), onInput: (v) => { jump = v; update(); } })));
  side.appendChild(h("div", { class: "panel" }, h("h4", {}, "computeCartesianPath()"), stats, verdict,
    h("div", { class: "row", style: { marginTop: "8px" } }, button("Execute", execute, "primary"))));

  function update() {
    result = plan(start, goal, step, jump);
    stats.innerHTML = "";
    const add = (k, v) => { stats.appendChild(h("dt", {}, k)); stats.appendChild(h("dd", {}, v)); };
    add("path length", fmt(result.length || 0) + " m");
    add("samples", `${result.steps} = floor(L / eef_step) + 1`);
    add("reached", `${Math.round(result.fraction * result.steps)} of ${result.steps}`);
    add("fraction", fmt(result.fraction, 3));
    if (result.jumpAt > 0) add("jump test", `step ${result.jumpAt} exceeds ${jump} x mean joint step: path truncated`);
    else add("jump test", jump > 0 ? "passed" : "disabled (jump_threshold = 0.0, as in the code)");
    const ok = result.fraction > minFraction;
    verdict.innerHTML = "";
    verdict.appendChild(h("p", { class: "small", style: { margin: "8px 0 0" } },
      ok ? h("span", { class: "badge ok" }, "execute") : h("span", { class: "badge warn" }, "rejected"), " ",
      ok ? (result.fraction < 1 ? `trajectory executed although ${((1 - result.fraction) * 100).toFixed(1)} % of the line is missing` : "the full line is executed")
         : `log: FAILED: <move> (Coverage: ${(result.fraction * 100).toFixed(1)}%)`));
    cv.redraw();
  }

  function execute() {
    if (!(result.fraction > minFraction)) return;
    const n = Math.round(result.fraction * result.steps);
    const qs = result.samples.slice(0, n + 1).filter((s) => s.ok).map((s) => s.q);
    anim = { qs, i: 0, q: qs[0] };
  }

  loop(root, (dt) => {
    if (!anim || !anim.qs) return;
    anim.i = Math.min(anim.qs.length - 1, anim.i + dt * 40);
    anim.q = anim.qs[Math.floor(anim.i)];
    cv.redraw();
    if (anim.i >= anim.qs.length - 1) anim.qs = null;
  });
  update();
}
