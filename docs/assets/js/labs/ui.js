// Small DOM and canvas helpers shared by the labs.

export function h(tag, attrs = {}, ...children) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (v === undefined || v === null || v === false) continue;
    if (k === "class") el.className = v;
    else if (k === "style" && typeof v === "object") Object.assign(el.style, v);
    else if (k.startsWith("on") && typeof v === "function") el.addEventListener(k.slice(2), v);
    else if (k === "html") el.innerHTML = v;
    else el.setAttribute(k, v === true ? "" : v);
  }
  for (const c of children.flat()) {
    if (c === null || c === undefined || c === false) continue;
    el.appendChild(typeof c === "string" || typeof c === "number" ? document.createTextNode(String(c)) : c);
  }
  return el;
}

export function slider({ label, min, max, step, value, format = (v) => v, onInput }) {
  const out = h("output", {}, format(value));
  const input = h("input", { type: "range", min, max, step, value });
  input.addEventListener("input", () => {
    const v = parseFloat(input.value);
    out.textContent = format(v);
    onInput(v);
  });
  const el = h("label", { class: "slider" }, h("span", {}, label), input, out);
  el.set = (v) => {
    input.value = v;
    out.textContent = format(parseFloat(v));
  };
  el.input = input;
  return el;
}

export function check(label, checked, onChange) {
  const input = h("input", { type: "checkbox" });
  input.checked = checked;
  input.addEventListener("change", () => onChange(input.checked));
  const el = h("label", { class: "check" }, input, label);
  el.input = input;
  return el;
}

export function button(label, onClick, cls = "") {
  return h("button", { class: "btn " + cls, type: "button", onclick: onClick }, label);
}

export function css(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

export function palette() {
  return {
    bg: css("--bg"), surface: css("--surface"), surface2: css("--surface-2"), border: css("--border"),
    text: css("--text"), muted: css("--muted"), faint: css("--faint"),
    vision: css("--vision"), voice: css("--voice"), ctrl: css("--ctrl"), motion: css("--motion"),
    mission: css("--mission"), tf: css("--tf"), warn: css("--warn"), ok: css("--ok"), accent: css("--accent"),
  };
}

// Canvas that keeps a fixed aspect ratio, follows its container width and renders
// at device-pixel resolution. `draw(ctx, width, height)` is called on every resize.
export function makeCanvas(parent, aspect, draw, { minHeight = 0, maxHeight = 10000 } = {}) {
  const wrap = h("div", { class: "canvas-wrap" });
  const canvas = h("canvas");
  wrap.appendChild(canvas);
  parent.appendChild(wrap);
  const state = { canvas, wrap, ctx: canvas.getContext("2d"), width: 0, height: 0 };
  function resize() {
    const w = wrap.clientWidth || 600;
    const hgt = Math.max(minHeight, Math.min(maxHeight, w / aspect));
    const dpr = window.devicePixelRatio || 1;
    wrap.style.height = hgt + "px";
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(hgt * dpr);
    state.width = w;
    state.height = hgt;
    state.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    draw(state.ctx, w, hgt);
  }
  state.redraw = () => draw(state.ctx, state.width, state.height);
  if (typeof ResizeObserver !== "undefined") new ResizeObserver(resize).observe(wrap);
  window.addEventListener("themechange", () => state.redraw());
  resize();
  return state;
}

// Pointer position in CSS pixels relative to an element.
export function pointer(ev, el) {
  const r = el.getBoundingClientRect();
  return { x: ev.clientX - r.left, y: ev.clientY - r.top };
}

export function fmt(v, digits = 3) {
  if (v === null || v === undefined || Number.isNaN(v)) return "n/a";
  const s = Number(v).toFixed(digits);
  return s === "-" + (0).toFixed(digits) ? (0).toFixed(digits) : s;
}

export function vec(v, digits = 3) {
  return "(" + v.map((x) => fmt(x, digits)).join(", ") + ")";
}

// Animation loop that stops itself when the element leaves the document.
export function loop(el, step) {
  let last = performance.now();
  let id = 0;
  function frame(now) {
    if (!el.isConnected) return;
    const dt = Math.min(0.1, (now - last) / 1000);
    last = now;
    step(dt, now / 1000);
    id = requestAnimationFrame(frame);
  }
  id = requestAnimationFrame(frame);
  return () => cancelAnimationFrame(id);
}

export function roundRect(ctx, x, y, w, hgt, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + hgt, r);
  ctx.arcTo(x + w, y + hgt, x, y + hgt, r);
  ctx.arcTo(x, y + hgt, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

export function logLine(logEl, time, text, cls = "") {
  const line = h("div", {}, h("span", { class: "t" }, time), h("span", { class: cls }, text));
  logEl.appendChild(line);
  logEl.scrollTop = logEl.scrollHeight;
}

export function seededRandom(seed) {
  let s = seed >>> 0;
  return () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export function gaussian(rand) {
  const u = Math.max(1e-12, rand());
  const v = rand();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
