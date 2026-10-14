// Lab: a virtual camera placed with the calibrated extrinsics of move_robot.launch.py.
// Objects are projected with a pinhole model, depth is ray-cast, and the detections are
// processed with the same rules as VisionNode.image_callback().

import { h, slider, check, button, palette, makeCanvas, pointer, fmt, vec, roundRect, seededRandom, gaussian } from "./ui.js";
import { baseToCamera, cameraToBase, project, deproject, CAMERA_TRANSLATION, CAMERA_ROTATION, applyRotation,
  TABLE_Z, EXAMPLE_K, DEFAULT_MISSION, ARUCO_MARKER_LENGTH } from "../core/controller.js";

export const foot = "Extrinsics are the published static transform; intrinsics and object shapes are examples. Depth is the ray-cast distance rounded to whole millimetres, like a 16-bit depth image.";

const TOOLS = DEFAULT_MISSION.tools;

function defaultObjects() {
  return [
    { label: "fork", x: 0.52, y: 0.2, z: -0.07, kind: "flat", marker: true, hole: false },
    { label: "spoon", x: 0.4, y: 0.12, z: -0.07, kind: "flat", marker: true, hole: false },
    { label: "bottle", x: 0.6, y: 0.05, z: 0.1, kind: "tall", marker: true, hole: false },
    { label: "fork", x: 0.66, y: 0.27, z: -0.07, kind: "flat", marker: false, hole: false },
    { label: "mug", x: 0.36, y: -0.06, z: -0.0, kind: "tall", marker: true, hole: false },
  ];
}

// Geometry: flat tools are 16 x 3 cm plates at height z; tall objects are vertical cylinders of radius 3.5 cm.
function samplePoints(o) {
  if (o.kind === "flat") {
    const pts = [];
    for (const dx of [-0.08, 0.08]) for (const dy of [-0.015, 0.015]) pts.push([o.x + dx, o.y + dy, o.z]);
    return pts;
  }
  const pts = [];
  for (const zz of [TABLE_Z, o.z]) for (let a = 0; a < 8; a++) pts.push([o.x + 0.035 * Math.cos(a * Math.PI / 4), o.y + 0.035 * Math.sin(a * Math.PI / 4), zz]);
  return pts;
}

function rayCast(dirBase, objects) {
  const o = CAMERA_TRANSLATION;
  let best = { t: Infinity, what: "none" };
  if (dirBase[2] < 0) {
    const t = (TABLE_Z - o[2]) / dirBase[2];
    if (t > 0) best = { t, what: "table" };
  }
  for (const ob of objects) {
    if (ob.kind === "flat") {
      if (dirBase[2] >= 0) continue;
      const t = (ob.z - o[2]) / dirBase[2];
      const px = o[0] + t * dirBase[0], py = o[1] + t * dirBase[1];
      if (t > 0 && t < best.t && Math.abs(px - ob.x) <= 0.08 && Math.abs(py - ob.y) <= 0.015) best = { t, what: ob.label };
    } else {
      // top disc
      if (dirBase[2] < 0) {
        const t = (ob.z - o[2]) / dirBase[2];
        const px = o[0] + t * dirBase[0], py = o[1] + t * dirBase[1];
        if (t > 0 && t < best.t && Math.hypot(px - ob.x, py - ob.y) <= 0.035) best = { t, what: ob.label };
      }
      // side wall
      const ox = o[0] - ob.x, oy = o[1] - ob.y;
      const a = dirBase[0] ** 2 + dirBase[1] ** 2, b = 2 * (ox * dirBase[0] + oy * dirBase[1]), c = ox * ox + oy * oy - 0.035 ** 2;
      const disc = b * b - 4 * a * c;
      if (disc >= 0 && a > 1e-12) {
        const t = (-b - Math.sqrt(disc)) / (2 * a);
        const z = o[2] + t * dirBase[2];
        if (t > 0 && t < best.t && z >= TABLE_Z && z <= ob.z) best = { t, what: ob.label };
      }
    }
  }
  return best;
}

function depthAt(u, v, K, objects) {
  const dCam = [(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1];
  const dBase = applyRotation(CAMERA_ROTATION, dCam);
  const hit = rayCast(dBase, objects);
  return { zCam: hit.t === Infinity ? 0 : hit.t, what: hit.what }; // t is the optical-axis depth because dCam[2] = 1
}

function analyse(objects, K, noiseMm, rand) {
  const rows = [];
  for (const o of objects) {
    const pc = baseToCamera([o.x, o.y, o.z]);
    const [u, v] = project(pc, K);
    const row = { o, inImage: pc[2] > 0 && u >= 0 && u < K.width && v >= 0 && v < K.height, inTools: TOOLS.includes(o.label), u, v };
    rows.push(row);
    if (!row.inImage || !row.inTools) continue;
    const pts = samplePoints(o).map((p) => project(baseToCamera(p), K));
    let x1 = Math.min(...pts.map((p) => p[0])) - 4, x2 = Math.max(...pts.map((p) => p[0])) + 4;
    let y1 = Math.min(...pts.map((p) => p[1])) - 4, y2 = Math.max(...pts.map((p) => p[1])) + 4;
    x1 = Math.max(0, x1); y1 = Math.max(0, y1); x2 = Math.min(K.width, x2); y2 = Math.min(K.height, y2);
    // YOLO returns integer centre and size; the node recomputes the corners from them.
    const bcx = Math.trunc((x1 + x2) / 2), bcy = Math.trunc((y1 + y2) / 2), bw = Math.trunc(x2 - x1), bh = Math.trunc(y2 - y1);
    row.box = [Math.trunc(bcx - bw / 2), Math.trunc(bcy - bh / 2), Math.trunc(bcx + bw / 2), Math.trunc(bcy + bh / 2)];
    row.method = o.marker ? "ARUCO" : "YOLO";
    const pu = o.marker ? Math.trunc(u) : bcx;
    const pv = o.marker ? Math.trunc(v) : bcy;
    row.pixel = [pu, pv];
    const d = depthAt(pu + 0.5, pv + 0.5, K, objects);
    let mm = Math.round(d.zCam * 1000 + noiseMm * gaussian(rand));
    if (o.hole || d.zCam === 0) mm = 0;
    row.depthMm = Math.max(0, mm);
    row.surface = d.what;
    row.markerPx = (K.fx * ARUCO_MARKER_LENGTH) / pc[2];
    if (row.depthMm > 0) {
      const z = row.depthMm / 1000;
      row.pCam = deproject(pu, pv, z, K);
      row.pBase = cameraToBase(row.pCam);
      row.errMm = 1000 * Math.hypot(row.pBase[0] - o.x, row.pBase[1] - o.y, row.pBase[2] - o.z);
    }
    row.published = row.method === "ARUCO" && row.depthMm > 0;
  }
  return rows;
}

export function mount(root) {
  const objects = defaultObjects();
  const K = { ...EXAMPLE_K };
  let noise = 2;
  let seed = 1;
  let selected = 0;
  let rows = [];

  const topBox = h("div", { class: "stack" });
  const camBox = h("div", { class: "stack" });
  root.appendChild(h("div", { class: "grid-2" }, topBox, camBox));
  topBox.appendChild(h("div", { class: "small muted" }, "Top view of the table (fr3_link0). Drag an object."));
  camBox.appendChild(h("div", { class: "small muted" }, "Colour image 640 x 480, as seen by vision_node, with its annotations."));

  // map extents
  const X0 = 0.12, X1 = 0.86, Y0 = -0.42, Y1 = 0.62;
  let topScale = 1;
  const top = makeCanvas(topBox, 1.0, drawTop, { minHeight: 260 });
  const cam = makeCanvas(camBox, 640 / 480, drawCam, { minHeight: 200 });

  function toScreen(x, y, w, hgt) {
    const s = Math.min(w / (Y1 - Y0), hgt / (X1 - X0));
    topScale = s;
    const ox = (w - s * (Y1 - Y0)) / 2, oy = (hgt - s * (X1 - X0)) / 2;
    return [ox + (Y1 - y) * s, oy + (X1 - x) * s];
  }
  function fromScreen(px, py, w, hgt) {
    const s = Math.min(w / (Y1 - Y0), hgt / (X1 - X0));
    const ox = (w - s * (Y1 - Y0)) / 2, oy = (hgt - s * (X1 - X0)) / 2;
    return [X1 - (py - oy) / s, Y1 - (px - ox) / s];
  }

  function drawTop(ctx, w, hgt) {
    const c = palette();
    ctx.clearRect(0, 0, w, hgt);
    // field of view on the table
    const corners = [[0, 0], [K.width, 0], [K.width, K.height], [0, K.height]].map(([u, v]) => {
      const d = applyRotation(CAMERA_ROTATION, [(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1]);
      const t = (TABLE_Z - CAMERA_TRANSLATION[2]) / d[2];
      return toScreen(CAMERA_TRANSLATION[0] + t * d[0], CAMERA_TRANSLATION[1] + t * d[1], w, hgt);
    });
    ctx.fillStyle = c.vision; ctx.globalAlpha = 0.08;
    ctx.beginPath(); corners.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y))); ctx.closePath(); ctx.fill();
    ctx.globalAlpha = 1; ctx.strokeStyle = c.vision; ctx.setLineDash([4, 4]); ctx.stroke(); ctx.setLineDash([]);
    // grid
    ctx.strokeStyle = c.border; ctx.lineWidth = 1;
    for (let x = 0.2; x <= 0.81; x += 0.1) { const a = toScreen(x, Y0, w, hgt), b = toScreen(x, Y1, w, hgt); ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke(); }
    for (let y = -0.4; y <= 0.61; y += 0.1) { const a = toScreen(X0, y, w, hgt), b = toScreen(X1, y, w, hgt); ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke(); }
    ctx.fillStyle = c.faint; ctx.font = "11px 'JetBrains Mono', monospace";
    const lab = toScreen(X0 + 0.01, Y0 + 0.01, w, hgt);
    ctx.fillText("x = " + X0.toFixed(2), lab[0] - 70, lab[1] - 4);
    // camera
    const [cx, cy] = toScreen(CAMERA_TRANSLATION[0], CAMERA_TRANSLATION[1], w, hgt);
    ctx.fillStyle = c.vision; roundRect(ctx, cx - 14, cy - 7, 28, 14, 4); ctx.fill();
    ctx.fillStyle = c.muted; ctx.font = "12px Inter, sans-serif"; ctx.fillText("camera", cx + 18, cy + 4);
    // objects
    rows.forEach((r, i) => {
      const o = r.o;
      const [sx, sy] = toScreen(o.x, o.y, w, hgt);
      ctx.fillStyle = r.published ? c.warn : r.box ? c.ok : c.faint;
      ctx.globalAlpha = i === selected ? 1 : 0.8;
      if (o.kind === "flat") { roundRect(ctx, sx - 0.015 * topScale, sy - 0.08 * topScale, 0.03 * topScale, 0.16 * topScale, 4); ctx.fill(); }
      else { ctx.beginPath(); ctx.arc(sx, sy, 0.035 * topScale, 0, 2 * Math.PI); ctx.fill(); }
      ctx.globalAlpha = 1;
      if (i === selected) { ctx.strokeStyle = c.accent; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(sx, sy, 0.1 * topScale, 0, 2 * Math.PI); ctx.stroke(); ctx.lineWidth = 1; }
      ctx.fillStyle = c.text; ctx.font = "600 12px Inter, sans-serif"; ctx.fillText(o.label + (o.marker ? "" : " (no marker)"), sx + 12, sy - 10);
    });
  }

  function drawCam(ctx, w, hgt) {
    const c = palette();
    const s = w / K.width;
    ctx.clearRect(0, 0, w, hgt);
    ctx.fillStyle = c.surface2; ctx.fillRect(0, 0, w, hgt);
    // table grid projected
    ctx.strokeStyle = c.border;
    const line = (a, b) => {
      const pa = baseToCamera(a), pb = baseToCamera(b);
      if (pa[2] <= 0 || pb[2] <= 0) return;
      const [ua, va] = project(pa, K), [ub, vb] = project(pb, K);
      ctx.beginPath(); ctx.moveTo(ua * s, va * s); ctx.lineTo(ub * s, vb * s); ctx.stroke();
    };
    for (let x = 0.1; x <= 0.91; x += 0.1) line([x, -0.5, TABLE_Z], [x, 0.7, TABLE_Z]);
    for (let y = -0.5; y <= 0.71; y += 0.1) line([0.1, y, TABLE_Z], [0.9, y, TABLE_Z]);
    for (const r of rows) {
      const o = r.o;
      if (!r.inImage) continue;
      const pts = samplePoints(o).map((p) => project(baseToCamera(p), K));
      ctx.fillStyle = o.kind === "flat" ? "#cbd5e1" : "#7dd3fc";
      ctx.beginPath();
      const hull = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
      // convex hull (monotone chain) for a clean silhouette
      const cross = (o2, a, b) => (a[0] - o2[0]) * (b[1] - o2[1]) - (a[1] - o2[1]) * (b[0] - o2[0]);
      const lower = [], upper = [];
      for (const p of hull) { while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop(); lower.push(p); }
      for (const p of hull.slice().reverse()) { while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop(); upper.push(p); }
      const poly = lower.slice(0, -1).concat(upper.slice(0, -1));
      poly.forEach(([u, v], i) => (i ? ctx.lineTo(u * s, v * s) : ctx.moveTo(u * s, v * s)));
      ctx.closePath(); ctx.fill();
      if (o.marker) {
        const m = Math.max(3, r.markerPx || 6) * s;
        ctx.fillStyle = "#000"; ctx.fillRect(r.u * s - m / 2, r.v * s - m / 2, m, m);
        ctx.fillStyle = "#fff"; ctx.fillRect(r.u * s - m / 6, r.v * s - m / 3, m / 4, m / 1.8);
      }
      if (r.box) {
        const color = r.method === "ARUCO" ? "#ff4d4d" : "#22c55e";
        ctx.strokeStyle = color; ctx.lineWidth = 2;
        ctx.strokeRect(r.box[0] * s, r.box[1] * s, (r.box[2] - r.box[0]) * s, (r.box[3] - r.box[1]) * s);
        ctx.fillStyle = color; ctx.font = "600 11px 'JetBrains Mono', monospace";
        ctx.fillText(r.method, r.box[0] * s, r.box[1] * s - 4);
        if (r.pixel) { ctx.fillStyle = "#ffd60a"; ctx.beginPath(); ctx.arc((r.pixel[0] + 0.5) * s, (r.pixel[1] + 0.5) * s, 3, 0, 2 * Math.PI); ctx.fill(); }
        ctx.lineWidth = 1;
      }
    }
  }

  // drag in the top view
  let dragging = -1;
  top.canvas.addEventListener("pointerdown", (ev) => {
    const p = pointer(ev, top.canvas);
    let best = -1, bd = 24;
    objects.forEach((o, i) => {
      const [sx, sy] = toScreen(o.x, o.y, top.width, top.height);
      const d = Math.hypot(sx - p.x, sy - p.y);
      if (d < bd) { bd = d; best = i; }
    });
    if (best >= 0) { dragging = best; selected = best; top.canvas.setPointerCapture(ev.pointerId); refresh(); }
  });
  top.canvas.addEventListener("pointermove", (ev) => {
    if (dragging < 0) return;
    const p = pointer(ev, top.canvas);
    const [x, y] = fromScreen(p.x, p.y, top.width, top.height);
    objects[dragging].x = Math.max(X0 + 0.02, Math.min(X1 - 0.02, x));
    objects[dragging].y = Math.max(Y0 + 0.02, Math.min(Y1 - 0.02, y));
    refresh();
  });
  top.canvas.addEventListener("pointerup", () => { dragging = -1; });

  // controls
  const objPanel = h("div", { class: "panel" });
  const controls = h("div", { class: "panel" }, h("h4", {}, "Camera and depth"),
    slider({ label: "focal length fx = fy", min: 380, max: 950, step: 5, value: K.fx, format: (v) => v.toFixed(0) + " px", onInput: (v) => { K.fx = v; K.fy = v; refresh(); } }),
    slider({ label: "depth noise (1 sigma)", min: 0, max: 12, step: 0.5, value: noise, format: (v) => v.toFixed(1) + " mm", onInput: (v) => { noise = v; refresh(); } }),
    h("div", { class: "row", style: { marginTop: "8px" } }, button("New noise sample", () => { seed += 1; refresh(); }, "small"), button("Reset scene", () => { objects.splice(0, objects.length, ...defaultObjects()); refresh(); }, "small")));
  const tableBox = h("div", { style: { overflowX: "auto" } });
  const jsonBox = h("pre", { class: "codebox" });
  root.appendChild(h("div", { class: "grid-2", style: { marginTop: "14px" } }, objPanel, controls));
  root.appendChild(h("div", { class: "panel", style: { marginTop: "14px" } }, h("h4", {}, "Per detection, as computed by image_callback()"), tableBox));
  root.appendChild(h("div", { class: "panel", style: { marginTop: "14px" } }, h("h4", {}, "/vision/detected_objects (std_msgs/String)"), jsonBox));

  function renderObjPanel() {
    const o = objects[selected];
    objPanel.innerHTML = "";
    objPanel.appendChild(h("h4", {}, "Selected object"));
    const sel = h("select", { class: "text-input" });
    objects.forEach((ob, i) => sel.appendChild(h("option", { value: i, selected: i === selected ? "selected" : null }, `${i + 1}. ${ob.label}`)));
    sel.addEventListener("change", () => { selected = parseInt(sel.value, 10); refresh(); });
    objPanel.appendChild(sel);
    objPanel.appendChild(h("div", { class: "row", style: { marginTop: "8px" } },
      check("ArUco marker visible", o.marker, (v) => { o.marker = v; refresh(); }),
      check("depth hole at the pixel", o.hole, (v) => { o.hole = v; refresh(); })));
    objPanel.appendChild(h("p", { class: "small muted", style: { margin: "8px 0 0" } },
      TOOLS.includes(o.label) ? `'${o.label}' is in tools[] = [${TOOLS.join(", ")}].` : `'${o.label}' is not in tools[]: YOLO-World is never asked for it, so no box appears.`));
  }

  function refresh() {
    rows = analyse(objects, K, noise, seededRandom(seed));
    renderObjPanel();
    top.redraw();
    cam.redraw();
    const t = h("table", { class: "trace" });
    t.appendChild(h("tr", {}, ...["object", "method", "(u, v)", "depth", "marker", "p_cam (m)", "p_base (m)", "error", "published"].map((x) => h("th", {}, x))));
    for (const r of rows) {
      const name = r.o.label + (r.o.marker ? "" : " *");
      if (!r.inImage) { t.appendChild(h("tr", {}, h("td", {}, name), h("td", { colspan: 8, class: "muted" }, "outside the image"))); continue; }
      if (!r.box) { t.appendChild(h("tr", {}, h("td", {}, name), h("td", { colspan: 8, class: "muted" }, "no box: not a detector class"))); continue; }
      t.appendChild(h("tr", {},
        h("td", {}, name),
        h("td", {}, h("span", { class: "badge " + (r.method === "ARUCO" ? "warn" : "ok") }, r.method)),
        h("td", { class: "mono" }, `(${r.pixel[0]}, ${r.pixel[1]})`),
        h("td", { class: "mono" }, r.depthMm + " mm" + (r.surface !== r.o.label && r.depthMm ? ` (${r.surface})` : "")),
        h("td", { class: "mono" }, r.method === "ARUCO" ? r.markerPx.toFixed(1) + " px" : ""),
        h("td", { class: "mono" }, r.pCam ? vec(r.pCam) : "dropped (z = 0)"),
        h("td", { class: "mono" }, r.pBase ? vec(r.pBase) : ""),
        h("td", { class: "mono" }, r.errMm !== undefined ? r.errMm.toFixed(1) + " mm" : ""),
        h("td", {}, r.published ? h("span", { class: "badge ok" }, "yes") : h("span", { class: "badge muted" }, "no"))));
    }
    tableBox.innerHTML = "";
    tableBox.appendChild(t);
    const msg = rows.filter((r) => r.published).map((r) => ({ label: r.o.label, x: r.pCam[0], y: r.pCam[1], z: r.pCam[2] }));
    jsonBox.textContent = JSON.stringify(msg);
  }
  refresh();
}
