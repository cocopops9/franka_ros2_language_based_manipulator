// Lab: the VAD of hmi_server running in the browser, on the microphone or on a synthetic signal.
// Capture chain (gain 1.5, 2048-sample blocks, averaging downsampler, PCM16) follows
// hmi_pkg/public/index.html; the state machine is the port of hmi_pkg/nodes/vad.py.

import { h, button, palette, makeCanvas, loop, seededRandom, gaussian, fmt } from "./ui.js";
import { VADProcessor, CONFIG, downsampleTo16k, floatToInt16, rmsInt16, msOfSamples, asrPreChecks } from "../core/hmi.js";

export const foot = "Nothing is uploaded: the lab stops where hmi_server would call the Whisper API and shows the checks it would apply first.";

function synthChunks() {
  const sr = 16000, rand = seededRandom(7);
  const plan = [["n", 1.1], ["s", 0.28, 0.1], ["n", 0.1], ["s", 0.16, 0.055], ["n", 0.12], ["s", 0.34, 0.085],
    ["n", 0.95], ["s", 0.06, 0.25], ["n", 0.95], ["s", 0.48, 0.09], ["n", 1.2]];
  const samples = [];
  let t = 0;
  for (const [kind, dur, amp] of plan) {
    const n = Math.floor(dur * sr);
    for (let i = 0; i < n; i++) {
      let v = 0.006 * gaussian(rand);
      if (kind === "s") {
        const env = amp * Math.pow(Math.sin((Math.PI * i) / n), 0.6) * Math.SQRT2;
        const tt = (t * sr + i) / sr;
        v += (env * (0.6 * Math.sin(2 * Math.PI * 180 * tt) + 0.3 * Math.sin(2 * Math.PI * 360 * tt) + 0.1 * Math.sin(2 * Math.PI * 720 * tt))) / 0.69;
      }
      samples.push(Math.max(-32768, Math.min(32767, Math.trunc(v * 32768))));
    }
    t += dur;
  }
  const chunks = [];
  for (let i = 0; i + 683 <= samples.length; i += 683) chunks.push(Int16Array.from(samples.slice(i, i + 683)));
  return chunks;
}

export function mount(root) {
  const vad = new VADProcessor();
  const history = [];          // {rms, state}
  const queue = [];            // synthetic chunks waiting to be "received"
  let audio = null;            // {ctx, stream, proc}
  let chunkMs = 42;
  let acc = 0;

  const status = h("span", { class: "badge muted" }, "idle");
  const micBtn = button("Use microphone", toggleMic, "voice");
  const synBtn = button("Play synthetic example", playSynthetic);
  const resetBtn = button("Restart server (recalibrate)", () => { vad.reset(); history.length = 0; }, "small");
  root.appendChild(h("div", { class: "row" }, micBtn, synBtn, resetBtn, status));
  const chartBox = h("div", { style: { marginTop: "12px" } });
  root.appendChild(chartBox);
  const cv = makeCanvas(chartBox, 3.2, draw, { minHeight: 170 });
  const info = h("dl", { class: "kv" });
  const segList = h("div", { class: "stack" });
  root.appendChild(h("div", { class: "grid-2", style: { marginTop: "12px" } },
    h("div", { class: "panel" }, h("h4", {}, "VADProcessor state"), info),
    h("div", { class: "panel" }, h("h4", {}, "Segments (newest first)"), segList)));

  function processChunk(chunk) {
    chunkMs = msOfSamples(chunk.length);
    if (vad.calibrating) {
      vad.processCalibration(chunk);
      history.push({ rms: rmsInt16(chunk), state: "calib" });
    } else {
      const r = vad.processAudio(chunk);
      let state = vad.inVoice ? (r.isVoice ? "voice" : "hang") : "idle";
      if (r.ended) state = "end";
      history.push({ rms: r.rms, state });
      if (r.segment) addSegment(r.segment, r.endForLen);
    }
    if (history.length > 400) history.splice(0, history.length - 400);
  }

  function addSegment(seg, byLength) {
    const pre = asrPreChecks(seg);
    const item = h("div", { class: "panel", style: { background: "var(--surface)" } },
      h("div", { class: "row" },
        h("span", { class: "badge voice" }, `${pre.segMs} ms`),
        h("span", { class: "mono" }, `RMS ${pre.segRms.toFixed(4)}`),
        pre.send ? h("span", { class: "badge ok" }, `upload, gain ${pre.gain.toFixed(2)}`) : h("span", { class: "badge warn" }, "not uploaded"),
        button("Play", () => playSegment(seg), "small")),
      h("div", { class: "small muted", style: { marginTop: "4px" } },
        (byLength ? "closed at the 6 s limit. " : `closed after ${CONFIG.VAD_SIL_HANG_MS} ms of silence. `) + (pre.send ? "Whisper would receive this WAV." : pre.reason)));
    segList.insertBefore(item, segList.firstChild);
    while (segList.children.length > 6) segList.lastChild.remove();
  }

  function playSegment(seg) {
    const ctx = new (window.AudioContext || window.webkitAudioContext)();
    const buf = ctx.createBuffer(1, seg.length, 16000);
    const ch = buf.getChannelData(0);
    for (let i = 0; i < seg.length; i++) ch[i] = seg[i] / 32768;
    const src = ctx.createBufferSource();
    src.buffer = buf;
    src.connect(ctx.destination);
    src.onended = () => ctx.close();
    src.start();
  }

  async function toggleMic() {
    if (audio) { stopMic(); return; }
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: { echoCancellation: true, noiseSuppression: true, autoGainControl: true, channelCount: 1 } });
      const ctx = new (window.AudioContext || window.webkitAudioContext)();
      const src = ctx.createMediaStreamSource(stream);
      const gain = ctx.createGain();
      gain.gain.value = 1.5;
      const proc = ctx.createScriptProcessor(2048, 1, 1);
      src.connect(gain);
      gain.connect(proc);
      proc.connect(ctx.destination);
      proc.onaudioprocess = (ev) => {
        if (!root.isConnected) { stopMic(); return; }
        const ds = downsampleTo16k(ev.inputBuffer.getChannelData(0), ctx.sampleRate);
        processChunk(floatToInt16(ds));
      };
      audio = { ctx, stream, proc };
      micBtn.textContent = "Stop microphone";
      status.className = "badge voice";
      status.textContent = `microphone at ${ctx.sampleRate} Hz`;
    } catch (err) {
      status.className = "badge warn";
      status.textContent = "microphone unavailable: " + err.message;
    }
  }

  function stopMic() {
    if (!audio) return;
    try { audio.proc.disconnect(); } catch (e) { /* already disconnected */ }
    audio.stream.getTracks().forEach((t) => t.stop());
    audio.ctx.close();
    audio = null;
    micBtn.textContent = "Use microphone";
    status.className = "badge muted";
    status.textContent = "idle";
  }

  function playSynthetic() {
    stopMic();
    vad.reset();
    history.length = 0;
    segList.innerHTML = "";
    queue.splice(0, queue.length, ...synthChunks());
    status.className = "badge vision";
    status.textContent = "synthetic signal, real time";
  }

  function draw(ctx, w, hgt) {
    const c = palette();
    ctx.clearRect(0, 0, w, hgt);
    const n = Math.max(120, Math.floor(w / 5));
    const data = history.slice(-n);
    const bw = w / n;
    const base = hgt - 34, topY = 14;
    const vmax = 0.12;
    const Y = (v) => base - (base - topY) * Math.min(v, vmax) / vmax;
    const colors = { calib: c.tf, idle: c.faint, voice: c.voice, hang: c.voice, end: c.ok };
    data.forEach((d, i) => {
      ctx.globalAlpha = d.state === "hang" ? 0.45 : 0.9;
      ctx.fillStyle = colors[d.state];
      ctx.fillRect(i * bw + 0.5, Y(d.rms), Math.max(1, bw - 1), base - Y(d.rms));
      ctx.fillRect(i * bw + 0.5, base + 10, Math.max(1, bw - 1), 10);
    });
    ctx.globalAlpha = 1;
    if (!vad.calibrating) {
      ctx.strokeStyle = c.warn; ctx.setLineDash([6, 4]);
      ctx.beginPath(); ctx.moveTo(0, Y(vad.gateRms)); ctx.lineTo(w, Y(vad.gateRms)); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = c.warn; ctx.font = "11px 'JetBrains Mono', monospace";
      ctx.fillText(`gate ${vad.gateRms.toFixed(4)}`, w - 110, Y(vad.gateRms) - 5);
    }
    ctx.fillStyle = c.faint; ctx.font = "11px 'JetBrains Mono', monospace";
    ctx.fillText(`last ${((n * chunkMs) / 1000).toFixed(1)} s, one bar per ${chunkMs} ms chunk; RMS axis 0 to ${vmax}`, 6, hgt - 4);
  }

  loop(root, (dt) => {
    if (queue.length) {
      acc += dt * 1000;
      while (acc >= chunkMs && queue.length) { acc -= chunkMs; processChunk(queue.shift()); }
      if (!queue.length) { status.className = "badge muted"; status.textContent = "synthetic signal finished"; }
    }
    info.innerHTML = "";
    const add = (k, v) => { info.appendChild(h("dt", {}, k)); info.appendChild(h("dd", {}, v)); };
    const state = vad.calibrating ? `calibrating (${vad.calibMs} / ${Math.trunc(CONFIG.VAD_CALIB_SEC * 1000)} ms)` : vad.inVoice ? (vad.silMs > 0 ? "hang-over" : "in voice") : "idle";
    add("state", state);
    add("noise_rms", vad.calibrating ? "pending" : vad.noiseRms.toFixed(4));
    add("gate_rms", vad.calibrating ? "pending" : `${vad.gateRms.toFixed(4)} = max(0.01, 2.5 x noise, 0.0015)`);
    add("voice_ms / sil_ms", `${vad.voiceMs} / ${vad.silMs}`);
    add("last chunk RMS", history.length ? fmt(history[history.length - 1].rms, 4) : "n/a");
    cv.redraw();
  });
}
