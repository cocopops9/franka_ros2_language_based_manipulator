// Lab: drive the intent / lock / confirm state machine of hmi_server and see the
// controller's reaction to every message on /hmi/intent_raw.

import { h, button, check, slider, logLine, fmt } from "./ui.js";
import { HmiServerSim, controllerReactsToIntent } from "../core/hmi.js";

export const foot = "Port of websocket_handler.py and the /confirm, /discard handlers of hmi_server.py, verified by check_parity.py on scripted dialogues. Times are server arrival times of transcripts.";

export function mount(root) {
  let sim = new HmiServerSim();
  let t = 0;
  let gap = 1.2;
  let waiting = true;     // is the arm waiting at the release pose?
  let flag = false;       // release_signal_received

  const input = h("input", { type: "text", class: "text-input", value: "release", "aria-label": "Utterance" });
  const sayBtn = button("Say", () => say(input.value), "voice");
  const silBtn = button("Empty segment", () => say(""));
  const confirmBtn = button("Confirm", () => apply(sim.confirm(), "POST /confirm"), "ok");
  const discardBtn = button("Discard", () => apply(sim.discard(), "POST /discard"), "warn");
  const resetBtn = button("Reset", () => { sim = new HmiServerSim(); t = 0; flag = false; logEl.innerHTML = ""; refresh(); }, "small");
  const chips = h("div", { class: "row" }, ...["pick the bottle", "pick", "the mug", "release", "Release.", "stop", "cancel", "next one"].map((p) =>
    h("button", { class: "chip", type: "button", onclick: () => { input.value = p; say(p); } }, p)));
  input.addEventListener("keydown", (ev) => { if (ev.key === "Enter") say(input.value); });
  // Ticking the box starts a new wait: move_to_pose.cpp clears the flag at that moment.
  const waitCheck = check("arm is waiting at the release pose (the wait clears the flag)", waiting, (v) => { waiting = v; if (v) flag = false; refresh(); });

  root.appendChild(h("div", { class: "stack" },
    h("div", { class: "row" }, h("div", { style: { flex: "1 1 220px" } }, input), sayBtn, silBtn, confirmBtn, discardBtn, resetBtn),
    chips,
    slider({ label: "time between segments", min: 0.2, max: 3, step: 0.1, value: gap, format: (v) => v.toFixed(1) + " s", onInput: (v) => { gap = v; } }),
    waitCheck));

  const stateBox = h("dl", { class: "kv" });
  const robotBox = h("dl", { class: "kv" });
  const logEl = h("div", { class: "log", style: { height: "260px" } });
  root.appendChild(h("div", { class: "grid-2", style: { marginTop: "14px" } },
    h("div", { class: "stack" },
      h("div", { class: "panel" }, h("h4", {}, "hmi_server state"), stateBox),
      h("div", { class: "panel" }, h("h4", {}, "move_to_pose_node"), robotBox)),
    h("div", {}, h("div", { class: "small muted", style: { marginBottom: "6px" } }, "WebSocket (ws), ROS and server log"), logEl)));

  function say(text) {
    t += gap;
    if (sim.locked) {
      logLine(logEl, fmt(t, 1).padStart(5), `(locked: the page does not send audio; "${text}" is not heard)`, "dim");
      refresh();
      return;
    }
    logLine(logEl, fmt(t, 1).padStart(5), text ? `Whisper -> "${text}"` : "Whisper -> \"\" (empty transcript)", "hmi");
    apply(sim.onTranscript(text, t), null);
  }

  function apply(events, label) {
    if (label) logLine(logEl, fmt(t, 1).padStart(5), label, "hmi");
    for (const e of events) {
      if (e.kind === "ws") logLine(logEl, "  ws", JSON.stringify(e.msg), "ws");
      else if (e.kind === "log") logLine(logEl, " log", e.text, "dim");
      else if (e.kind === "ros") {
        logLine(logEl, " ros", `${e.topic}  ${e.data}`, "ros");
        if (controllerReactsToIntent(e.data)) {
          if (waiting) {
            flag = true;
            logLine(logEl, " ctl", "[HMI] RELEASE SIGNAL RECEIVED!  -> open_gripper()", "warn");
            waiting = false;
          } else {
            flag = true;
            logLine(logEl, " ctl", "flag set, but the arm is not waiting: it will be cleared when the next wait starts", "dim");
          }
        } else {
          logLine(logEl, " ctl", "no \"release\" substring: ignored", "dim");
        }
      }
    }
    refresh();
  }

  function refresh() {
    stateBox.innerHTML = "";
    const add = (box, k, v) => { box.appendChild(h("dt", {}, k)); box.appendChild(h("dd", {}, v)); };
    add(stateBox, "time", fmt(t, 1) + " s");
    add(stateBox, "locked", sim.locked ? h("span", { class: "badge warn" }, "locked: Confirm or Discard") : h("span", { class: "badge ok" }, "listening"));
    add(stateBox, "pending", sim.pendingAction || sim.pendingObject ? `${JSON.stringify({ action: sim.pendingAction, object: sim.pendingObject })}, age ${fmt(t - sim.pendingTs, 1)} s` : "none");
    add(stateBox, "last_intent", JSON.stringify(sim.lastIntent));
    add(stateBox, "current_sig", sim.currentSig || "none");
    confirmBtn.disabled = !sim.locked;
    discardBtn.disabled = !sim.locked;
    robotBox.innerHTML = "";
    add(robotBox, "phase", waiting ? "WAITING FOR RELEASE..." : "not waiting (moving, or gripper opened)");
    add(robotBox, "release_signal_received", String(flag));
    waitCheck.input.checked = waiting;
  }
  refresh();
}
