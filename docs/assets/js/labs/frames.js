// Lab: 3-D view of fr3_link0, the calibrated camera_link and a movable point.

import { h, slider, check, palette, makeCanvas, pointer, fmt, vec } from "./ui.js";
import { CAMERA_TRANSLATION as T, CAMERA_ROTATION as R, CAMERA_QUATERNION, baseToCamera, project, applyRotation,
  TABLE_Z, EXAMPLE_K, CALIBRATION_MARKER, INTERMEDIATE_POSE, RELEASE_POSE_RIGHT } from "../core/controller.js";

export const foot = "Camera pose: static transform of move_robot.launch.py. The frustum uses example intrinsics (640 x 480, fx = 615 px).";

export function mount(root) {
  const view = { yaw: -2.25, pitch: 0.62, dist: 2.45, target: [0.34, 0.0, 0.2] };
  const P = { x: 0.52, y: 0.2, z: -0.07 };
  const show = { frustum: true, marker: true, waypoints: true };

  const canvasBox = h("div");
  const info = h("div", { class: "stack" });
  root.appendChild(h("div", { class: "grid-2" }, canvasBox, info));
  const cv = makeCanvas(canvasBox, 1.15, draw, { minHeight: 280 });

  function proj(p, w, hgt) {
    const d = [p[0] - view.target[0], p[1] - view.target[1], p[2] - view.target[2]];
    const cy = Math.cos(view.yaw), sy = Math.sin(view.yaw);
    const x1 = cy * d[0] - sy * d[1];
    const y1 = sy * d[0] + cy * d[1];
    const cp = Math.cos(view.pitch), sp = Math.sin(view.pitch);
    // viewer looks along (0, cos p, -sin p) in the yawed frame, i.e. down onto the table for p > 0
    const depth = cp * y1 - sp * d[2];
    const up = sp * y1 + cp * d[2];
    const z = view.dist + depth;
    const f = Math.min(w, hgt) * 1.6;
    return [w / 2 + (f * x1) / z, hgt / 2 - (f * up) / z, z];
  }

  function draw(ctx, w, hgt) {
    const c = palette();
    ctx.clearRect(0, 0, w, hgt);
    const line = (a, b, color, width = 1, dash = null) => {
      const pa = proj(a, w, hgt), pb = proj(b, w, hgt);
      if (pa[2] <= 0.05 || pb[2] <= 0.05) return;
      ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash || []);
      ctx.beginPath(); ctx.moveTo(pa[0], pa[1]); ctx.lineTo(pb[0], pb[1]); ctx.stroke(); ctx.setLineDash([]);
    };
    const label = (p, text, color) => {
      const q = proj(p, w, hgt);
      ctx.fillStyle = color; ctx.font = "600 12px 'JetBrains Mono', monospace"; ctx.fillText(text, q[0] + 4, q[1] - 4);
    };
    const add = (a, b, s = 1) => [a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]];

    // table grid
    for (let x = 0; x <= 0.901; x += 0.1) line([x, -0.6, TABLE_Z], [x, 0.7, TABLE_Z], c.border);
    for (let y = -0.6; y <= 0.701; y += 0.1) line([0, y, TABLE_Z], [0.9, y, TABLE_Z], c.border);
    // robot column
    line([0, 0, TABLE_Z], [0, 0, 0.333], c.faint, 9);
    // base axes
    const axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    const axisColors = ["#ef4444", "#22c55e", "#3b82f6"];
    axes.forEach((a, i) => { line([0, 0, 0], add([0, 0, 0], a, 0.15), axisColors[i], 3); label(add([0, 0, 0], a, 0.16), "xyz"[i], axisColors[i]); });
    label([0, 0, -0.03], "fr3_link0", c.text);

    // camera frame
    const col = (j) => [R[0][j], R[1][j], R[2][j]];
    ["x_c", "y_c", "z_c"].forEach((name, j) => { line(T, add(T, col(j), 0.13), axisColors[j], 3); label(add(T, col(j), 0.14), name, axisColors[j]); });
    label(add(T, [0, 0, 0.04]), "camera_link", c.text);
    if (show.frustum) {
      const K = EXAMPLE_K;
      const hits = [[0, 0], [K.width, 0], [K.width, K.height], [0, K.height]].map(([u, v]) => {
        const d = applyRotation(R, [(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1]);
        const t = (TABLE_Z - T[2]) / d[2];
        return add(T, d, t);
      });
      hits.forEach((p, i) => { line(T, p, c.vision, 1, [4, 4]); line(p, hits[(i + 1) % 4], c.vision, 1.5); });
      const axisHit = add(T, col(2), (TABLE_Z - T[2]) / R[2][2]);
      line(T, axisHit, c.vision, 1.2);
    }
    if (show.marker) {
      const m = CALIBRATION_MARKER;
      const s = 0.03; // drawn larger than the real 15 mm square for visibility
      const mx = [m.rotation[0][0], m.rotation[1][0], m.rotation[2][0]];
      const my = [m.rotation[0][1], m.rotation[1][1], m.rotation[2][1]];
      const corners = [[-1, 1], [1, 1], [1, -1], [-1, -1]].map(([a, b]) => add(add(m.translation, mx, a * s), my, b * s));
      corners.forEach((p, i) => line(p, corners[(i + 1) % 4], c.text, 2));
      line(m.translation, add(m.translation, mx, 0.08), axisColors[0], 2);
      line(m.translation, add(m.translation, my, 0.08), axisColors[1], 2);
      label(add(m.translation, [0, -0.05, 0]), "marker ID 0", c.muted);
    }
    if (show.waypoints) {
      for (const [pose, name] of [[INTERMEDIATE_POSE, "intermediate"], [RELEASE_POSE_RIGHT, "release"]]) {
        const p = [pose.position.x, pose.position.y, pose.position.z];
        const q = proj(p, w, hgt);
        ctx.fillStyle = c.motion; ctx.beginPath(); ctx.arc(q[0], q[1], 5, 0, 2 * Math.PI); ctx.fill();
        label(p, name, c.motion);
        line(p, [p[0], p[1], TABLE_Z], c.motion, 1, [2, 4]);
      }
    }
    // point and ray
    const p = [P.x, P.y, P.z];
    line(T, p, c.ctrl, 2, [6, 4]);
    line(p, [p[0], p[1], TABLE_Z], c.ctrl, 1, [2, 3]);
    const q = proj(p, w, hgt);
    ctx.fillStyle = c.ctrl; ctx.beginPath(); ctx.arc(q[0], q[1], 6, 0, 2 * Math.PI); ctx.fill();
    label(p, "P", c.ctrl);
  }

  let drag = null;
  cv.canvas.addEventListener("pointerdown", (ev) => { drag = pointer(ev, cv.canvas); cv.canvas.setPointerCapture(ev.pointerId); });
  cv.canvas.addEventListener("pointermove", (ev) => {
    if (!drag) return;
    const p = pointer(ev, cv.canvas);
    view.yaw += (p.x - drag.x) * 0.008;
    view.pitch = Math.max(-0.2, Math.min(1.45, view.pitch + (p.y - drag.y) * 0.006));
    drag = p;
    cv.redraw();
  });
  cv.canvas.addEventListener("pointerup", () => { drag = null; });
  cv.canvas.addEventListener("wheel", (ev) => {
    ev.preventDefault();
    view.dist = Math.max(1.0, Math.min(5, view.dist * (ev.deltaY > 0 ? 1.08 : 0.93)));
    cv.redraw();
  }, { passive: false });

  const out = h("dl", { class: "kv" });
  const matrix = h("pre", { class: "codebox" });
  info.appendChild(h("div", { class: "panel" }, h("h4", {}, "Point P in fr3_link0"),
    slider({ label: "x", min: 0.1, max: 0.9, step: 0.005, value: P.x, format: (v) => v.toFixed(3) + " m", onInput: (v) => { P.x = v; update(); } }),
    slider({ label: "y", min: -0.6, max: 0.7, step: 0.005, value: P.y, format: (v) => v.toFixed(3) + " m", onInput: (v) => { P.y = v; update(); } }),
    slider({ label: "z", min: -0.075, max: 0.5, step: 0.005, value: P.z, format: (v) => v.toFixed(3) + " m", onInput: (v) => { P.z = v; update(); } })));
  info.appendChild(h("div", { class: "panel" }, h("h4", {}, "Same point in camera_link"), out));
  info.appendChild(h("div", { class: "panel" }, h("h4", {}, "Static transform fr3_link0 to camera_link"), matrix,
    h("div", { class: "row", style: { marginTop: "8px" } },
      check("field of view", show.frustum, (v) => { show.frustum = v; cv.redraw(); }),
      check("calibration marker", show.marker, (v) => { show.marker = v; cv.redraw(); }),
      check("fixed poses", show.waypoints, (v) => { show.waypoints = v; cv.redraw(); }))));

  matrix.textContent =
    "t = " + vec(T, 4) + "\nq = " + vec(CAMERA_QUATERNION, 4) + "\nR =\n" +
    R.map((r) => "  [" + r.map((v) => (v >= 0 ? " " : "") + v.toFixed(4)).join("  ") + "]").join("\n") +
    "\n\np_cam = R^T (p_base - t)\np_base = R p_cam + t";

  function update() {
    const pc = baseToCamera([P.x, P.y, P.z]);
    const [u, v] = project(pc, EXAMPLE_K);
    const inside = pc[2] > 0 && u >= 0 && u < EXAMPLE_K.width && v >= 0 && v < EXAMPLE_K.height;
    out.innerHTML = "";
    const add = (k, val) => { out.appendChild(h("dt", {}, k)); out.appendChild(h("dd", {}, val)); };
    add("p_cam", vec(pc));
    add("depth z_c", fmt(pc[2]) + " m, i.e. " + Math.round(pc[2] * 1000) + " mm in the depth image");
    add("pixel (u, v)", `(${u.toFixed(1)}, ${v.toFixed(1)})`);
    add("in the image", inside ? "yes" : "no: the camera cannot see P");
    add("distance", fmt(Math.hypot(pc[0], pc[1], pc[2])) + " m from the camera centre");
    cv.redraw();
  }
  update();
}
