// Lab: type a transcript and follow every decision of nlu_engine.py and websocket_handler.py.

import { h, check } from "./ui.js";
import { NLU, editDistance, phoneticSimilarity, allPhraseMatches } from "../core/nlu.js";
import { HmiServerSim, cleanWhisperText, controllerReactsToIntent } from "../core/hmi.js";

export const foot = "check_parity.py compares this parser with the Python module on 692 phrases under 24 hash seeds. Where Python's answer depends on the seed, this port takes the first candidate in insertion order and the lab flags the tie.";

const EXAMPLES = ["Pick the bottle.", "Release.", "pick up the cup", "let go", "never mind", "Cancel.", "can", "pick next",
  "grab the coffee cup", "peek the botle", "drop it", "rel ease", "stop", "the mug"];

function fuzzyTies(nlu, token, kind) {
  const map = kind === "action" ? nlu.synAct : nlu.synObj;
  const valid = kind === "action" ? nlu.actions : nlu.objects;
  const keys = [...new Set([...map.keys(), ...[...valid].map((v) => v.toLowerCase())])];
  let bestSim = 0, bestD = 999;
  const sims = [], ds = [];
  for (const k of keys) {
    const d = editDistance(token, k, 3);
    const s = phoneticSimilarity(token, k);
    sims.push([k, s]); ds.push([k, d]);
    if (s > bestSim) bestSim = s;
    if (d < bestD) bestD = d;
  }
  const canon = (k) => (map.has(k) ? map.get(k) : k);
  const simCanon = new Set(sims.filter(([, s]) => s === bestSim).map(([k]) => canon(k)));
  const dCanon = new Set(ds.filter(([, d]) => d === bestD).map(([k]) => canon(k)));
  return { simTie: simCanon.size > 1 ? [...simCanon] : null, editTie: dCanon.size > 1 ? [...dCanon] : null };
}

function phraseTie(nlu, text, kind) {
  const map = kind === "action" ? nlu.synAct : nlu.synObj;
  const matches = allPhraseMatches(text, map.keys());
  if (!matches.length) return null;
  const longest = Math.max(...matches.map((m) => m.phrase.length));
  const canon = new Set(matches.filter((m) => m.phrase.length === longest).map((m) => map.get(m.phrase)));
  return canon.size > 1 ? [...canon] : null;
}

export function mount(root) {
  const nlu = new NLU();
  let whisper = true;
  const input = h("input", { type: "text", class: "text-input", value: "Pick the bottle.", "aria-label": "Transcript" });
  const chips = h("div", { class: "row", style: { marginTop: "8px" } }, ...EXAMPLES.map((e) => h("button", { class: "chip", type: "button", onclick: () => { input.value = e; run(); } }, e)));
  root.appendChild(h("div", { class: "stack" },
    h("label", { class: "field" }, "Transcript returned by Whisper", input),
    h("div", {}, check("apply the post-processing of asr_whisper.py (regex rewrites, ASCII filter)", whisper, (v) => { whisper = v; run(); })),
    chips));
  const resultBox = h("div", { class: "stack" });
  const traceBox = h("div", { style: { overflowX: "auto" } });
  root.appendChild(h("div", { class: "grid-2", style: { marginTop: "14px" } },
    h("div", { class: "panel" }, h("h4", {}, "Result"), resultBox),
    h("div", { class: "panel" }, h("h4", {}, "Decision trace of NLU.parse()"), traceBox)));

  function run() {
    const raw = input.value;
    const text = whisper ? cleanWhisperText(raw) : raw.trim();
    const trace = [];
    const parsed = text ? nlu.parse(text, trace) : null;
    resultBox.innerHTML = "";
    const kv = h("dl", { class: "kv" });
    const add = (k, v) => { kv.appendChild(h("dt", {}, k)); kv.appendChild(h("dd", {}, v)); };
    add("text to the parser", JSON.stringify(text));
    add("NLU.parse()", JSON.stringify(parsed));
    // What the server does with it (fresh server state, loud 800 ms segment).
    const sim = new HmiServerSim();
    const events = sim.onTranscript(raw, 1.0, { segRms: 0.05, segMs: 800, gateRms: 0.01 });
    const ros = events.filter((e) => e.kind === "ros");
    const logs = events.filter((e) => e.kind === "log").map((e) => e.text);
    const locked = events.some((e) => e.kind === "ws" && e.msg.type === "locked" && e.msg.value);
    const intent = events.find((e) => e.kind === "ws" && e.msg.type === "intent");
    let verdict;
    if (!text) verdict = "empty transcript: nothing happens (a pending part may be flushed)";
    else if (!parsed) verdict = "no intent";
    else if (logs.some((l) => l.startsWith("Rejected"))) verdict = "rejected: transcript of at most 3 characters";
    else if (intent && locked) verdict = "complete intent: shown, locked until Confirm or Discard";
    else if (logs.some((l) => l.startsWith("Storing pending"))) verdict = "partial intent: kept for 900 ms, waiting for the missing part";
    else verdict = logs.join("; ") || "no action";
    add("hmi_server (full pipeline)", verdict);
    add("/hmi/intent_raw", ros.length ? ros[0].data : "nothing published now");
    const reacts = ros.length && controllerReactsToIntent(ros[0].data);
    add("move_to_pose_node", reacts ? "contains \"release\": opens the gripper if the arm is waiting" : "no effect (only the substring \"release\" is tested)");
    resultBox.appendChild(kv);

    // trace table
    const t = h("table", { class: "trace" });
    t.appendChild(h("tr", {}, h("th", {}, "step"), h("th", {}, "detail")));
    const ties = [];
    for (const s of trace) {
      let detail = "";
      if (s.step === "clean") detail = `"${s.clean}"  tokens: ${s.tokens.join(" | ")}`;
      else if (s.step === "action-phrase" || s.step === "object-phrase") {
        detail = `alias "${s.phrase}" matched, canonical "${s.result}"`;
        const tie = phraseTie(nlu, s.step === "action-phrase" ? text.toLowerCase().replace(/[^a-z0-9]+/g, " ").trim() : (trace.find((x) => x.step === "after-pick") || { post: text }).post, s.step.startsWith("action") ? "action" : "object");
        if (tie) ties.push(`equal-length aliases for ${tie.join(" / ")}`);
      } else if (s.step === "after-pick") detail = `search objects after "pick": "${s.post}"`;
      else if (s.step === "action-alias" || s.step === "object-alias") detail = `token "${s.token}" is an alias of "${s.result}"`;
      else if (s.step === "action-fuzzy" || s.step === "object-fuzzy") {
        detail = `token "${s.token}": best phonetic "${s.bestPhonetic[0]}" ${s.bestPhonetic[1].toFixed(3)} (threshold ${s.threshold}), closest edit "${s.bestEdit[0]}" d = ${s.bestEdit[1]}`;
        const tie = fuzzyTies(nlu, s.token, s.step.startsWith("action") ? "action" : "object");
        if (tie.simTie && s.bestPhonetic[1] >= s.threshold) ties.push(`token "${s.token}": phonetic tie between ${tie.simTie.join(" / ")}`);
        if (tie.editTie && s.bestPhonetic[1] < s.threshold) ties.push(`token "${s.token}": edit-distance tie between ${tie.editTie.join(" / ")}`);
      }
      t.appendChild(h("tr", {}, h("td", { class: "mono" }, s.step), h("td", { class: "mono", style: { whiteSpace: "normal" } }, detail)));
    }
    traceBox.innerHTML = "";
    if (trace.length) traceBox.appendChild(t);
    else traceBox.appendChild(h("p", { class: "muted small" }, "Nothing to parse."));
    if (ties.length) {
      traceBox.appendChild(h("p", { class: "small", style: { marginTop: "10px" } }, h("span", { class: "badge warn" }, "seed-dependent"), " ",
        "Python breaks these ties by set iteration order, which changes with PYTHONHASHSEED: " + ties.join("; ") + "."));
    }
    const multi = text.toLowerCase().match(/\b(pick up|let go|open gripper|drop it|put down|next one|grab next|take next|hold on|never mind|forget it|coffee cup|tea cup|aluminum can|soda can)\b/);
    if (multi) {
      traceBox.appendChild(h("p", { class: "small", style: { marginTop: "6px" } }, h("span", { class: "badge muted" }, "note"), " ",
        `"${multi[1]}" is a multi-word alias; its compiled pattern can never match (Section 5.5), so only single tokens were considered.`));
    }
  }
  input.addEventListener("input", run);
  run();
}
