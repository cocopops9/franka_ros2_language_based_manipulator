// Lab: the success test of a Franka grasp goal.

import { h, slider, check, button, palette, makeCanvas, loop, roundRect } from "./ui.js";
import { GRIPPER_GRASP_GOAL, GRIPPER_OPEN_GOAL } from "../core/controller.js";

export const foot = "Rule from libfranka Gripper::grasp(): success if width - epsilon_inner < d < width + epsilon_outer. The code's goal is preselected; move_to_pose.cpp logs the result but continues either way.";

export function mount(root) {
  const goal = { ...GRIPPER_GRASP_GOAL };
  let objectWidth = 0.025;
  let noObject = false;
  let fingers = GRIPPER_OPEN_GOAL.width;
  let target = fingers;

  const box = h("div");
  const side = h("div", { class: "stack" });
  root.appendChild(h("div", { class: "grid-2" }, box, side));
  const cv = makeCanvas(box, 1.5, draw, { minHeight: 220 });
  const verdict = h("div", { class: "stack" });
  side.appendChild(h("div", { class: "panel" }, h("h4", {}, "Object"),
    slider({ label: "object width", min: 0.002, max: 0.1, step: 0.001, value: objectWidth, format: (v) => (v * 1000).toFixed(0) + " mm", onInput: (v) => { objectWidth = v; update(); } }),
    check("no object between the fingers", noObject, (v) => { noObject = v; update(); })));
  side.appendChild(h("div", { class: "panel" }, h("h4", {}, "Grasp goal (code values preselected)"),
    slider({ label: "width", min: 0, max: 0.08, step: 0.001, value: goal.width, format: (v) => v.toFixed(3) + " m", onInput: (v) => { goal.width = v; update(); } }),
    slider({ label: "epsilon inner", min: 0, max: 0.06, step: 0.001, value: goal.epsilonInner, format: (v) => v.toFixed(3) + " m", onInput: (v) => { goal.epsilonInner = v; update(); } }),
    slider({ label: "epsilon outer", min: 0, max: 0.06, step: 0.001, value: goal.epsilonOuter, format: (v) => v.toFixed(3) + " m", onInput: (v) => { goal.epsilonOuter = v; update(); } }),
    h("div", { class: "row", style: { marginTop: "8px" } }, button("Grasp", () => { fingers = GRIPPER_OPEN_GOAL.width; update(); }, "primary"))));
  side.appendChild(h("div", { class: "panel" }, h("h4", {}, "Outcome"), verdict));

  function stopWidth() {
    if (noObject) return 0;
    return objectWidth <= GRIPPER_OPEN_GOAL.width ? objectWidth : null;
  }

  function draw(ctx, w, hgt) {
    const c = palette();
    ctx.clearRect(0, 0, w, hgt);
    const cx = w / 2, scale = (w * 0.55) / 0.1;
    // palm
    ctx.fillStyle = c.border; roundRect(ctx, cx - 0.055 * scale, 22, 0.11 * scale, 22, 6); ctx.fill();
    // object
    const d = stopWidth();
    if (!noObject) {
      ctx.fillStyle = c.ctrl; ctx.globalAlpha = 0.85;
      roundRect(ctx, cx - (objectWidth / 2) * scale, 70, objectWidth * scale, hgt * 0.38, 6); ctx.fill(); ctx.globalAlpha = 1;
    }
    // fingers
    ctx.fillStyle = c.text;
    roundRect(ctx, cx - (fingers / 2) * scale - 14, 40, 14, hgt * 0.45, 4); ctx.fill();
    roundRect(ctx, cx + (fingers / 2) * scale, 40, 14, hgt * 0.45, 4); ctx.fill();
    // number line
    const y = hgt - 34, v0 = -0.02, v1 = 0.11;
    const X = (v) => 20 + ((v - v0) / (v1 - v0)) * (w - 40);
    const lo = goal.width - goal.epsilonInner, hi = goal.width + goal.epsilonOuter;
    ctx.fillStyle = c.ok; ctx.globalAlpha = 0.18; ctx.fillRect(X(lo), y - 14, X(hi) - X(lo), 28); ctx.globalAlpha = 1;
    ctx.strokeStyle = c.faint; ctx.beginPath(); ctx.moveTo(20, y); ctx.lineTo(w - 20, y); ctx.stroke();
    ctx.fillStyle = c.faint; ctx.font = "11px 'JetBrains Mono', monospace";
    for (let v = -0.02; v <= 0.1001; v += 0.02) { ctx.fillRect(X(v), y - 4, 1, 8); ctx.fillText(v.toFixed(2), X(v) - 14, y + 18); }
    ctx.fillStyle = c.border; ctx.fillRect(X(0), y - 3, X(0.08) - X(0), 6);
    ctx.strokeStyle = c.ctrl; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(X(goal.width), y - 18); ctx.lineTo(X(goal.width), y + 6); ctx.stroke(); ctx.lineWidth = 1;
    ctx.fillStyle = c.text; ctx.beginPath(); ctx.moveTo(X(fingers), y - 6); ctx.lineTo(X(fingers) - 6, y - 16); ctx.lineTo(X(fingers) + 6, y - 16); ctx.fill();
    ctx.fillText("d", X(fingers) - 3, y - 20);
    void d;
  }

  function update() {
    const d = stopWidth();
    target = d === null ? GRIPPER_OPEN_GOAL.width : d;
    const lo = goal.width - goal.epsilonInner, hi = goal.width + goal.epsilonOuter;
    verdict.innerHTML = "";
    if (d === null) {
      verdict.appendChild(h("p", { class: "small" }, h("span", { class: "badge warn" }, "does not fit"), ` The object is wider than the ${GRIPPER_OPEN_GOAL.width} m opening.`));
      cv.redraw();
      return;
    }
    const ok = d > lo && d < hi;
    verdict.appendChild(h("div", { class: "mono" }, `${lo.toFixed(3)} < d = ${d.toFixed(3)} < ${hi.toFixed(3)} : ${ok}`));
    verdict.appendChild(h("p", { class: "small" }, ok ? h("span", { class: "badge ok" }, "SUCCEEDED") : h("span", { class: "badge warn" }, "ABORTED"), " ",
      `log: "${ok ? "Grasp Successful." : "Grasp Failed."}" Then the controller retracts and carries on to the release pose in both cases.`));
    if (lo < 0 && hi > GRIPPER_OPEN_GOAL.width) verdict.appendChild(h("p", { class: "small muted" }, "With these values every reachable finger width counts as success, including fingers that closed on nothing."));
    cv.redraw();
  }

  loop(root, (dt) => {
    const speed = GRIPPER_GRASP_GOAL.speed; // m/s, as in the goal
    if (Math.abs(fingers - target) > 1e-4) {
      fingers += Math.sign(target - fingers) * Math.min(Math.abs(target - fingers), speed * dt);
      cv.redraw();
    }
  });
  update();
}
