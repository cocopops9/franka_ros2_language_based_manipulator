// Lab: pick and pre-grasp poses computed from an object position, with the code's offsets.

import { h, slider, palette, makeCanvas, fmt } from "./ui.js";
import { pickPoses, INTERMEDIATE_POSE, RELEASE_POSE_RIGHT, LOW_OBJECT_Z, SAFETY_FLOOR_Z, PRE_GRASP_HOVER,
  PICK_OFFSET_X, PICK_OFFSET_Y, TABLE_Z } from "../core/controller.js";

export const foot = "pickPoses() is checked against move_to_pose.cpp by check_parity.py; the constants are parsed from the C++ source.";

export function mount(root) {
  const obj = { x: 0.52, y: 0.2, z: -0.07 };
  const ctrl = h("div", { class: "panel" }, h("h4", {}, "Object position in memory (fr3_link0)"),
    slider({ label: "x", min: 0.25, max: 0.8, step: 0.005, value: obj.x, format: (v) => v.toFixed(3) + " m", onInput: (v) => { obj.x = v; update(); } }),
    slider({ label: "y", min: -0.45, max: 0.45, step: 0.005, value: obj.y, format: (v) => v.toFixed(3) + " m", onInput: (v) => { obj.y = v; update(); } }),
    slider({ label: "z (marker height)", min: -0.075, max: 0.25, step: 0.001, value: obj.z, format: (v) => v.toFixed(3) + " m", onInput: (v) => { obj.z = v; update(); } }));
  const out = h("div", { style: { overflowX: "auto" } });
  const logBox = h("pre", { class: "codebox" });
  const left = h("div", { class: "stack" }, ctrl, h("div", { class: "panel" }, h("h4", {}, "Log line printed before the 10 s pause"), logBox));
  const right = h("div", { class: "stack" });
  root.appendChild(h("div", { class: "grid-2" }, left, right));
  let poses = pickPoses(obj.x, obj.y, obj.z);
  const cv = makeCanvas(right, 1.6, draw, { minHeight: 220 });
  root.appendChild(h("div", { class: "panel", style: { marginTop: "14px" } }, h("h4", {}, "Targets of one task, in order"), out));

  function draw(ctx, w, hgt) {
    const c = palette();
    ctx.clearRect(0, 0, w, hgt);
    // side view: z axis
    const z0 = -0.1, z1 = 0.3;
    const Y = (z) => hgt - 24 - ((z - z0) / (z1 - z0)) * (hgt - 44);
    const xa = 60, xb = w - 20;
    // Each level is labelled in its own column, so close values do not overlap.
    const level = (z, color, text, dash, col, below) => {
      ctx.strokeStyle = color; ctx.setLineDash(dash || []);
      ctx.beginPath(); ctx.moveTo(xa, Y(z)); ctx.lineTo(xb, Y(z)); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = color; ctx.font = "11.5px 'JetBrains Mono', monospace";
      ctx.fillText(`${text}: ${z.toFixed(3)}`, xa + 6 + col * (xb - xa - 120) * 0.5, Y(z) + (below ? 14 : -5));
    };
    ctx.fillStyle = c.border; ctx.fillRect(xa, Y(TABLE_Z), xb - xa, 6);
    ctx.fillStyle = c.faint; ctx.font = "11px 'JetBrains Mono', monospace"; ctx.fillText("z (m)", 6, 16);
    level(TABLE_Z, c.faint, "table", [2, 3], 0, true);
    level(LOW_OBJECT_Z, c.mission, "clamp below", [5, 4], 1, false);
    level(obj.z, c.vision, "marker", null, 0, false);
    level(poses.pick.position.z, c.ctrl, `pick (${poses.zRule})`, null, 1, true);
    level(poses.preGrasp.position.z, c.motion, "pre-grasp", null, 0, false);
    // gripper icon at pick
    const gx = xb - 70, gy = Y(poses.pick.position.z);
    ctx.fillStyle = c.text;
    ctx.fillRect(gx - 14, gy - 26, 28, 6);
    ctx.fillRect(gx - 14, gy - 22, 4, 22);
    ctx.fillRect(gx + 10, gy - 22, 4, 22);
    ctx.fillRect(gx - 3, gy - 60, 6, 34);
    ctx.strokeStyle = c.motion; ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(gx, Y(poses.preGrasp.position.z)); ctx.lineTo(gx, gy); ctx.stroke(); ctx.setLineDash([]);
  }

  function update() {
    poses = pickPoses(obj.x, obj.y, obj.z);
    const p = poses.pick.position, q = poses.pick.orientation, g = poses.preGrasp.position;
    logBox.textContent = `Moving to <tool>: (${p.x.toFixed(3)}, ${p.y.toFixed(3)}, ${p.z.toFixed(3)}, ${q[0].toFixed(2)}, ${q[1].toFixed(2)}, ${q[2].toFixed(2)})\n` +
      `\nx_pick = x - ${-PICK_OFFSET_X} = ${p.x.toFixed(3)}\ny_pick = y - ${-PICK_OFFSET_Y} = ${p.y.toFixed(3)}\n` +
      (poses.zRule === "safety floor" ? `z = ${obj.z.toFixed(3)} < ${LOW_OBJECT_Z}, so z_pick = ${SAFETY_FLOOR_Z}` : `z = ${obj.z.toFixed(3)} >= ${LOW_OBJECT_Z}, so z_pick = z - 0.08 = ${p.z.toFixed(3)}`) +
      `\nquaternion (x, y, z, w) = (${q.map((v) => v.toFixed(4)).join(", ")})  from RPY (180, 0, 90) deg`;
    const rows = [
      ["1", "intermediate", INTERMEDIATE_POSE],
      ["2", "pre-grasp", poses.preGrasp],
      ["3", "pick", poses.pick],
      ["4", "retract (pre-grasp)", poses.preGrasp],
      ["5", "intermediate", INTERMEDIATE_POSE],
      ["6", "release zone", RELEASE_POSE_RIGHT],
    ];
    const t = h("table", { class: "trace" });
    t.appendChild(h("tr", {}, ...["#", "target", "position (m)", "RPY (deg)", "quaternion (x, y, z, w)"].map((x) => h("th", {}, x))));
    for (const [n, name, pose] of rows) {
      const pp = pose.position;
      t.appendChild(h("tr", {}, h("td", {}, n), h("td", {}, name),
        h("td", { class: "mono" }, `(${fmt(pp.x)}, ${fmt(pp.y)}, ${fmt(pp.z)})`),
        h("td", { class: "mono" }, `(${pose.rpyDeg.join(", ")})`),
        h("td", { class: "mono" }, `(${pose.orientation.map((v) => fmt(v, 4)).join(", ")})`)));
    }
    out.innerHTML = "";
    out.appendChild(t);
    cv.redraw();
  }
  update();
}
