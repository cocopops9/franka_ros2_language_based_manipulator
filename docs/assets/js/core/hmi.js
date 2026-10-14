// Faithful JavaScript port of the HMI server pipeline:
//   hmi_pkg/nodes/audio_utils.py      (RMS, durations, gain)
//   hmi_pkg/nodes/vad.py              (calibration + energy-gated segmentation)
//   hmi_pkg/nodes/asr_whisper.py      (pre/post filters around the Whisper call)
//   hmi_pkg/nodes/websocket_handler.py and hmi_server.py (intent merge, lock, confirm)
//
// The Whisper request itself cannot run in a static page; the simulator receives
// the transcript as an argument instead. Everything before and after the call
// follows the Python code line by line.

import { NLU, postProcessTranscript, isEnglishLike, normText } from "./nlu.js";

// Defaults from hmi_pkg/nodes/config.py.
export const CONFIG = {
  SR_DEFAULT: 16000,
  WHISPER_MIN_RMS: 0.01,
  ASR_MIN_MS: 300,
  ASR_DEDUP_MS: 1700,
  VAD_CALIB_SEC: 0.7,
  VAD_GATE_MULT: 2.5,
  VAD_MIN_RMS_FLOOR: 0.0015,
  VAD_MIN_MS: 180,
  VAD_SIL_HANG_MS: 450,
  VAD_MAX_UTT_MS: 6000,
  INTENT_MERGE_MS: 900,
  LOCK_ON_INTENT: true,
  ASR_DROP_LOWCONF_SINGLE_ACTION: true,
  ASR_LOWCONF_RMS_MULT: 1.25,
  ASR_LOWCONF_MAX_MS: 1000,
  ROS_CONFIRM_MODE: "intent",
  ROS_INTENT_TOPIC: "/hmi/intent_raw",
  ROS_ASR_TOPIC: "/hmi/asr_text",
};

// websocket_handler.STANDALONE_ACTIONS
export const STANDALONE_ACTIONS = new Set(["next", "stop", "cancel", "release"]);

// ---------------------------------------------------------------------------
// audio_utils.py
// ---------------------------------------------------------------------------

// RMS of a PCM16 buffer, normalised to [0, 1]. `samples` is an Int16Array.
export function rmsInt16(samples) {
  if (!samples.length) return 0.0;
  let acc = 0;
  for (let i = 0; i < samples.length; i++) acc += samples[i] * samples[i];
  return Math.sqrt(acc / samples.length) / 32768.0;
}

// ms_of_bytes, expressed in samples instead of bytes (bytes = 2 * samples).
export function msOfSamples(sampleCount, sr = CONFIG.SR_DEFAULT) {
  return Math.trunc((sampleCount / Math.max(1, sr)) * 1000);
}

export function calculateGain(rms, targetRms = 0.03, maxGain = 3.0) {
  if (rms >= targetRms) return 1.0;
  return Math.min(maxGain, targetRms / Math.max(0.001, rms));
}

// public/index.html: downsampling and float -> int16 conversion done in the browser.
export function downsampleTo16k(float32, inRate) {
  if (inRate === 16000) return float32;
  const ratio = inRate / 16000;
  const newLen = Math.round(float32.length / ratio);
  const out = new Float32Array(newLen);
  let o = 0;
  let i = 0;
  while (o < newLen) {
    const ni = Math.round((o + 1) * ratio);
    let acc = 0;
    let c = 0;
    for (let j = i; j < ni && j < float32.length; j++) { acc += float32[j]; c++; }
    out[o++] = acc / (c || 1);
    i = ni;
  }
  return out;
}

export function floatToInt16(float32) {
  const out = new Int16Array(float32.length);
  for (let i = 0; i < float32.length; i++) {
    const s = Math.max(-1, Math.min(1, float32[i]));
    out[i] = s < 0 ? s * 0x8000 : s * 0x7fff;
  }
  return out;
}

// ---------------------------------------------------------------------------
// vad.py
// ---------------------------------------------------------------------------

export class VADProcessor {
  constructor(config = CONFIG) {
    this.config = config;
    this.reset();
  }

  reset() {
    this.calibrating = true;
    this.calibMs = 0;
    this.calibAccRms = 0.0;
    this.noiseRms = 0.0;
    this.gateRms = this.config.WHISPER_MIN_RMS;
    this.inVoice = false;
    this.voiceMs = 0;
    this.silMs = 0;
    this.speech = [];
  }

  updateGate() {
    this.gateRms = Math.max(
      this.config.WHISPER_MIN_RMS,
      this.noiseRms * this.config.VAD_GATE_MULT,
      this.config.VAD_MIN_RMS_FLOOR,
    );
  }

  // Returns true when calibration completes with this chunk.
  processCalibration(chunk, sr = CONFIG.SR_DEFAULT) {
    const chunkMs = msOfSamples(chunk.length, sr);
    const chunkRms = rmsInt16(chunk);
    this.calibMs += chunkMs;
    this.calibAccRms += chunkRms;
    if (this.calibMs >= Math.trunc(this.config.VAD_CALIB_SEC * 1000)) {
      const samples = this.calibMs / (chunkMs || 1);
      this.noiseRms = this.calibAccRms / Math.max(1, samples);
      this.updateGate();
      this.calibrating = false;
      return true;
    }
    return false;
  }

  startVoice() {
    this.inVoice = true;
    this.silMs = 0;
    this.speech = [];
  }

  // Returns the finished segment (Int16Array) or null if it was too short.
  endVoice(sr = CONFIG.SR_DEFAULT) {
    this.inVoice = false;
    this.voiceMs = 0;
    this.silMs = 0;
    let total = 0;
    for (const c of this.speech) total += c.length;
    const segment = new Int16Array(total);
    let offset = 0;
    for (const c of this.speech) { segment.set(c, offset); offset += c.length; }
    this.speech = [];
    if (msOfSamples(segment.length, sr) < this.config.VAD_MIN_MS) return null;
    return segment;
  }

  // Returns {segment, ended, tooShort}. `segment` is non-null only when an
  // utterance ended and was long enough.
  processAudio(chunk, sr = CONFIG.SR_DEFAULT) {
    const chunkMs = msOfSamples(chunk.length, sr);
    const chunkRms = rmsInt16(chunk);
    const isVoice = chunkRms >= this.gateRms;

    if (!this.inVoice) {
      if (isVoice) {
        this.startVoice();
        this.speech.push(chunk);
        this.voiceMs += chunkMs;
      }
      return { segment: null, ended: false, tooShort: false, rms: chunkRms, isVoice };
    }

    this.speech.push(chunk);
    this.voiceMs += chunkMs;
    const endForSil = !isVoice && this.silMs + chunkMs >= this.config.VAD_SIL_HANG_MS;
    const endForLen = this.voiceMs >= this.config.VAD_MAX_UTT_MS;
    if (isVoice) this.silMs = 0;
    else this.silMs += chunkMs;

    if (endForSil || endForLen) {
      const segment = this.endVoice(sr);
      return { segment, ended: true, tooShort: segment === null, rms: chunkRms, isVoice, endForLen };
    }
    return { segment: null, ended: false, tooShort: false, rms: chunkRms, isVoice };
  }
}

// ---------------------------------------------------------------------------
// asr_whisper.py filters
// ---------------------------------------------------------------------------

// Checks applied by transcribe_with_openai() before the audio is uploaded.
export function asrPreChecks(segment, sr = CONFIG.SR_DEFAULT, config = CONFIG) {
  const segMs = msOfSamples(segment.length, sr);
  const segRms = rmsInt16(segment);
  if (segMs < config.ASR_MIN_MS) {
    return { send: false, segMs, segRms, gain: 1.0, reason: `segment ${segMs} ms < ASR_MIN_MS ${config.ASR_MIN_MS} ms` };
  }
  if (segRms < config.WHISPER_MIN_RMS) {
    return { send: false, segMs, segRms, gain: 1.0, reason: `Segment RMS ${segRms.toFixed(3)} < ${config.WHISPER_MIN_RMS}` };
  }
  return { send: true, segMs, segRms, gain: calculateGain(segRms), reason: "" };
}

// sanitize_transcript with no Whisper prompt (WHISPER_USE_PROMPT=0 by default).
export function sanitizeTranscript(text) {
  if (!text) return text;
  let t = text;
  for (const pattern of [/^Robot control interface.*$/gim, /^ACTIONS:.*$/gim, /^OBJECTS:.*$/gim]) {
    t = t.replace(pattern, "");
  }
  return t.replace(/\s{2,}/g, " ").trim();
}

// The text transformations transcribe_with_openai() applies to Whisper's output.
export function cleanWhisperText(rawText) {
  let text = sanitizeTranscript((rawText || "").trim());
  text = postProcessTranscript(text);
  if (text && !isEnglishLike(text)) return "";
  return text;
}

export function shouldDropLowConfidence(text, segRms, segMs, gateRms, config = CONFIG) {
  if (!config.ASR_DROP_LOWCONF_SINGLE_ACTION) return false;
  const mono = new Set(["release", "stop", "next", "cancel", "pick"]);
  const tnorm = text.trim().toLowerCase();
  if (mono.has(tnorm) || /^(release|pick|stop|next|cancel)\s+(cup|bottle|mug|can)$/.test(tnorm)) {
    if (segRms < gateRms * config.ASR_LOWCONF_RMS_MULT && segMs <= config.ASR_LOWCONF_MAX_MS) return true;
  }
  return false;
}

// state.is_confirmable
export function isConfirmable(intent) {
  const act = intent ? intent.action : null;
  const obj = intent ? intent.object : null;
  if (!act) return false;
  if (act === "pick") return obj !== null && obj !== undefined;
  return true;
}

// Compact JSON identical to json.dumps(..., separators=(',', ':')).
export function rosIntentPayload(action, object, allowExecution) {
  return JSON.stringify({
    action: action,
    object: object ? { label: object } : null,
    allow_execution: allowExecution,
  });
}

// move_to_pose.cpp, intent_callback(): a plain substring test.
export function controllerReactsToIntent(payload) {
  return payload.indexOf("release") !== -1;
}

// ---------------------------------------------------------------------------
// websocket_handler.py + hmi_server.py (state machine after the transcript)
// ---------------------------------------------------------------------------

// Each method returns a list of events:
//   {kind: "ws", msg}         message sent to the browser over the WebSocket
//   {kind: "ros", topic, data} message published by the ROS bridge (when enabled)
//   {kind: "log", text}        server log line (abridged)
export class HmiServerSim {
  constructor({ nlu = new NLU(), config = CONFIG, rosEnabled = true } = {}) {
    this.nlu = nlu;
    this.config = config;
    this.rosEnabled = rosEnabled;
    this.reset();
  }

  reset() {
    this.locked = false;
    this.lastText = "";
    this.lastIntent = null;
    this.utterSeq = 0;
    this.currentSig = "";
    this.lastConfirmedSig = "";
    this.lastTxNorm = "";
    this.lastTxTime = 0.0;
    this.pendingAction = null;
    this.pendingObject = null;
    this.pendingTs = 0.0;
  }

  static sig(seq) {
    return "u" + String(seq).padStart(8, "0");
  }

  resetPending() {
    this.pendingAction = null;
    this.pendingObject = null;
    this.pendingTs = 0.0;
  }

  lock(events, why) {
    this.locked = true;
    events.push({ kind: "ws", msg: { type: "locked", value: true } });
    events.push({ kind: "log", text: `System locked after ${why}` });
  }

  // Whisper returned `rawText` for a segment (times in seconds).
  // `segRms`, `segMs` and `gateRms` feed the low-confidence filter.
  onTranscript(rawText, now, { segRms = 0.05, segMs = 800, gateRms = 0.01 } = {}) {
    const events = [];
    if (this.locked) {
      events.push({ kind: "log", text: "System locked: audio chunks are ignored before VAD" });
      return events;
    }

    const text = cleanWhisperText(rawText);

    if (!text) return events.concat(this.onEmptyTranscript(now));

    if (shouldDropLowConfidence(text, segRms, segMs, gateRms, this.config)) {
      events.push({ kind: "log", text: `Dropped low-confidence transcript: '${text}'` });
      return events;
    }

    const norm = normText(text);
    if (this.lastTxNorm && norm === this.lastTxNorm) {
      const since = (now - this.lastTxTime) * 1000;
      if (since < this.config.ASR_DEDUP_MS) {
        events.push({ kind: "log", text: `Duplicate transcript dropped: '${text}' (within ${since.toFixed(0)} ms)` });
        return events;
      }
    }
    this.lastTxNorm = norm;
    this.lastTxTime = now;
    this.lastText = text;
    this.utterSeq += 1;
    this.currentSig = HmiServerSim.sig(this.utterSeq);
    events.push({ kind: "ws", msg: { type: "final", text, sig: this.currentSig } });

    const parsed = this.nlu.parse(text);
    if (!parsed) {
      events.push({ kind: "log", text: `No intent parsed from: '${text}'` });
      return events;
    }
    if (text.trim().length <= 3 && (parsed.action || parsed.object)) {
      events.push({ kind: "log", text: `Rejected likely false match: '${text}'` });
      return events;
    }
    return events.concat(this.handleParsedIntent(parsed, now));
  }

  isCompleteIntent(action, object) {
    if (action === "cancel" && object === "can") return true;
    if (action && object) return true;
    if (action && !object) return STANDALONE_ACTIONS.has(action.toLowerCase());
    return false;
  }

  handleParsedIntent(parsed, now) {
    const events = [];
    let action = parsed.action;
    let object = parsed.object;
    if (action === "cancel" && object === "can") {
      object = null;
      parsed = { action, object: null };
      events.push({ kind: "log", text: "Interpreting 'cancel can' as standalone 'cancel'" });
    }

    if (this.isCompleteIntent(action, object)) {
      this.resetPending();
      this.lastIntent = parsed;
      events.push({ kind: "ws", msg: { type: "intent", intent: parsed, sig: this.currentSig } });
      if (this.config.LOCK_ON_INTENT && isConfirmable(parsed)) this.lock(events, "complete intent");
      if (this.rosEnabled) {
        const data = rosIntentPayload(action, object, false);
        events.push({ kind: "ros", topic: this.config.ROS_INTENT_TOPIC, data });
      }
      return events;
    }
    return events.concat(this.handlePartialIntent(action, object, now));
  }

  handlePartialIntent(action, object, now) {
    const events = [];
    let merged = null;
    const withinWindow = (now - this.pendingTs) * 1000 <= this.config.INTENT_MERGE_MS;

    if (action && !object) {
      if (this.pendingObject && withinWindow) {
        merged = { action, object: this.pendingObject };
        events.push({ kind: "log", text: `Merged action '${action}' with pending object '${this.pendingObject}'` });
        this.resetPending();
      } else {
        this.pendingAction = action;
        this.pendingObject = null;
        this.pendingTs = now;
        events.push({ kind: "log", text: `Storing pending action: ${action}` });
      }
    } else if (object && !action) {
      if (this.pendingAction && withinWindow) {
        merged = { action: this.pendingAction, object };
        events.push({ kind: "log", text: `Merged object '${object}' with pending action '${this.pendingAction}'` });
        this.resetPending();
      } else {
        this.pendingObject = object;
        this.pendingAction = null;
        this.pendingTs = now;
        events.push({ kind: "log", text: `Storing pending object: ${object}` });
      }
    }

    if (merged) {
      // Note: a merged intent is not published to ROS here; only /confirm publishes it.
      this.lastIntent = merged;
      events.push({ kind: "ws", msg: { type: "intent", intent: merged, sig: this.currentSig } });
      if (this.config.LOCK_ON_INTENT && isConfirmable(merged)) this.lock(events, "merged intent");
    }
    return events;
  }

  // handle_empty_transcript: the only place where a stale pending intent is flushed.
  onEmptyTranscript(now) {
    const events = [];
    if (this.pendingAction || this.pendingObject) {
      const waited = (now - this.pendingTs) * 1000;
      if (waited >= this.config.INTENT_MERGE_MS) {
        const pending = { action: this.pendingAction, object: this.pendingObject };
        this.resetPending();
        this.utterSeq += 1;
        this.currentSig = HmiServerSim.sig(this.utterSeq);
        // Note: STATE.last_intent is not updated on this path.
        events.push({ kind: "ws", msg: { type: "intent", intent: pending, sig: this.currentSig } });
        events.push({ kind: "log", text: `Pending intent timeout (${waited.toFixed(0)} ms), sending partial intent` });
        if (this.config.LOCK_ON_INTENT && isConfirmable(pending)) this.lock(events, "intent");
      }
    }
    return events;
  }

  // POST /confirm as sent by public/index.html (no signature in the form).
  confirm() {
    const events = [];
    const sig = this.currentSig;
    if (sig && sig === this.lastConfirmedSig) {
      events.push({ kind: "log", text: `Command already confirmed: ${sig}` });
      return events;
    }
    this.lastConfirmedSig = sig;
    this.locked = false;
    events.push({ kind: "ws", msg: { type: "locked", value: false } });
    if (this.rosEnabled) {
      if (this.config.ROS_CONFIRM_MODE === "intent") {
        if (this.lastIntent) {
          const data = rosIntentPayload(this.lastIntent.action, this.lastIntent.object, true);
          events.push({ kind: "ros", topic: this.config.ROS_INTENT_TOPIC, data });
        }
      } else if (this.lastText) {
        events.push({ kind: "ros", topic: this.config.ROS_ASR_TOPIC, data: this.lastText });
      }
    }
    events.push({ kind: "ws", msg: { type: "confirmed", sig } });
    return events;
  }

  // POST /discard
  discard() {
    this.locked = false;
    this.lastIntent = null;
    return [
      { kind: "ws", msg: { type: "locked", value: false } },
      { kind: "ws", msg: { type: "discarded" } },
    ];
  }
}
