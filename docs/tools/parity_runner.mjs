// Helper for check_parity.py: runs the JavaScript ports used by the interactive
// documentation on inputs read from stdin and prints the results as JSON.

import { NLU, CAPABILITIES } from "../assets/js/core/nlu.js";
import { VADProcessor, HmiServerSim, cleanWhisperText, shouldDropLowConfidence } from "../assets/js/core/hmi.js";
import * as controller from "../assets/js/core/controller.js";

const chunks = [];
for await (const chunk of process.stdin) chunks.push(chunk);
const job = JSON.parse(Buffer.concat(chunks).toString("utf8"));

const nlu = new NLU();
const out = {};

out.capabilities = CAPABILITIES;
out.mission = controller.DEFAULT_MISSION;

out.nlu = job.phrases.map((text) => nlu.parse(text));

out.clean = job.transcripts.map((text) => cleanWhisperText(text));

out.lowconf = job.lowconf.map(([text, rms, ms, gate]) => shouldDropLowConfidence(text, rms, ms, gate));

out.vad = job.vadRuns.map((run) => {
  const vad = new VADProcessor();
  const trace = [];
  for (const samples of run) {
    const chunk = Int16Array.from(samples);
    if (vad.calibrating) {
      const done = vad.processCalibration(chunk);
      trace.push(done ? ["calibrated", vad.noiseRms, vad.gateRms] : ["calibrating"]);
      continue;
    }
    const r = vad.processAudio(chunk);
    if (r.segment) trace.push(["segment", r.segment.length]);
    else if (r.ended) trace.push(["too_short"]);
    else trace.push([vad.inVoice ? "voice" : "idle"]);
  }
  return trace;
});

out.dialogues = job.dialogues.map((steps) => {
  const sim = new HmiServerSim();
  const log = [];
  for (const step of steps) {
    let events;
    if (step.op === "say") events = sim.onTranscript(step.text, step.t, { segRms: 0.05, segMs: 800, gateRms: 0.01 });
    else if (step.op === "silence") events = sim.onTranscript("", step.t);
    else if (step.op === "confirm") events = sim.confirm();
    else events = sim.discard();
    for (const e of events) {
      if (e.kind === "ros") log.push(["ros", e.topic, e.data]);
      if (e.kind === "ws" && (e.msg.type === "intent" || e.msg.type === "locked")) log.push(["ws", JSON.stringify(e.msg)]);
    }
  }
  return log;
});

out.poses = job.objects.map(([x, y, z]) => {
  const { pick, preGrasp } = controller.pickPoses(x, y, z);
  return [pick.position.x, pick.position.y, pick.position.z, preGrasp.position.z, ...pick.orientation];
});

out.rpy = job.rpy.map(([r, p, y]) => controller.quaternionFromRPY(r, p, y));

out.constants = {
  intermediate: [INTERMEDIATE()],
  release: [RELEASE()],
  grasp: controller.GRIPPER_GRASP_GOAL,
  open: controller.GRIPPER_OPEN_GOAL,
  offsets: [controller.PICK_OFFSET_X, controller.PICK_OFFSET_Y, controller.LOW_OBJECT_Z,
            controller.SAFETY_FLOOR_Z, controller.TABLE_Z_OFFSET, controller.PRE_GRASP_HOVER],
  scaling: [controller.VELOCITY_SCALING, controller.ACCELERATION_SCALING],
  cartesian: [controller.CARTESIAN_EEF_STEP, controller.CARTESIAN_JUMP_THRESHOLD, controller.MIN_PATH_FRACTION],
  timing: [controller.TRAVEL_TIME_OFFSET, controller.PRE_GRASP_PAUSE, controller.MEMORY_UPDATE_THRESHOLD],
  camera: [...controller.CAMERA_TRANSLATION, ...controller.CAMERA_QUATERNION],
};

function INTERMEDIATE() {
  const p = controller.INTERMEDIATE_POSE;
  return [p.position.x, p.position.y, p.position.z, ...p.rpyDeg];
}

function RELEASE() {
  const p = controller.RELEASE_POSE_RIGHT;
  return [p.position.x, p.position.y, p.position.z, ...p.rpyDeg];
}

process.stdout.write(JSON.stringify(out));
