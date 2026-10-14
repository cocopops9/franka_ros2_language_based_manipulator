#!/usr/bin/env python3
"""Check that the JavaScript code behind the interactive docs matches the project.

The interactive labs in docs/ run JavaScript ports of the HMI pipeline
(VAD, transcript filters, NLU, intent/lock state machine) and of the controller
math (pick poses, quaternions, constants). This script:

  1. runs the original Python modules of hmi_pkg/nodes on generated inputs,
     once per PYTHONHASHSEED value (the NLU breaks ties in set iteration order);
  2. runs the JavaScript ports on the same inputs through Node.js;
  3. extracts the numeric constants from move_to_pose.cpp and move_robot.launch.py
     and compares them with docs/assets/js/core/controller.js.

Usage:  python3 docs/tools/check_parity.py        (requires python3 and node)
No third-party Python package is needed: fastapi and openai are replaced by stubs.
"""

import asyncio
import json
import math
import os
import random
import re
import subprocess
import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NODES = ROOT / "hmi_pkg" / "nodes"
CAPABILITIES = ROOT / "hmi_pkg" / "config" / "capabilities.json"
RUNNER = Path(__file__).resolve().parent / "parity_runner.mjs"
SEEDS = list(range(24))


# ---------------------------------------------------------------------------
# Generated inputs (deterministic)
# ---------------------------------------------------------------------------

def build_phrases():
    caps = json.loads(CAPABILITIES.read_text())
    words = set(caps["actions"]) | set(caps["objects"])
    for group in caps["synonyms"].values():
        for canon, aliases in group.items():
            words.add(canon)
            words.update(aliases)
    words.update(["peak", "peek", "council", "relese", "necks", "spot", "ken", "cub", "bottol", "mag"])
    fillers = ["the", "please", "now", "robot", "a", "to", "me", "it", "and", "then"]
    rng = random.Random(1234)
    letters = "abcdefghijklmnopqrstuvwxyz"

    def misspell(w):
        w = list(w)
        op = rng.choice(["sub", "del", "ins", "swap"])
        i = rng.randrange(len(w))
        if op == "sub":
            w[i] = rng.choice(letters)
        elif op == "del" and len(w) > 2:
            del w[i]
        elif op == "ins":
            w.insert(i, rng.choice(letters))
        elif op == "swap" and i + 1 < len(w):
            w[i], w[i + 1] = w[i + 1], w[i]
        return "".join(w)

    base = sorted(words)
    phrases = list(base)
    phrases += [misspell(w) for w in base for _ in range(2)]
    for _ in range(500):
        parts = []
        for _ in range(rng.randint(1, 4)):
            pool = rng.random()
            if pool < 0.45:
                w = rng.choice(base)
            elif pool < 0.7:
                w = misspell(rng.choice(base))
            else:
                w = rng.choice(fillers)
            parts.append(w)
        phrases.append(" ".join(parts))
    phrases += ["Pick up the bottle.", "RELEASE!", "  stop  ", "", "123", "please", "pick, the: mug?",
                "let go", "never mind", "pick next", "grab the coffee cup", "take next"]
    return phrases


TRANSCRIPTS = [
    "Pick the bottle.", "Council.", "rel ease", "Peak can", "picca botto", "pick bottal",
    "Pické the cup", "pick   the    mug", "Robot control interface.\npick cup", "ike aken",
    "", "   ", "Counsel", "RELEASE", "next one",
]

LOWCONF = [
    ["release", 0.011, 600, 0.01], ["release", 0.02, 600, 0.01], ["release.", 0.011, 600, 0.01],
    ["pick cup", 0.011, 900, 0.01], ["pick cup", 0.011, 1001, 0.01], ["Stop", 0.012, 400, 0.01],
    ["stop now", 0.011, 400, 0.01], ["cancel can", 0.0124, 1000, 0.01], ["next", 0.0126, 1000, 0.01],
]

DIALOGUES = [
    [{"op": "say", "text": "pick the bottle", "t": 1.0}, {"op": "confirm"}],
    [{"op": "say", "text": "release", "t": 1.0}, {"op": "confirm"}],
    [{"op": "say", "text": "release", "t": 1.0}, {"op": "discard"}, {"op": "say", "text": "stop", "t": 3.0}],
    [{"op": "say", "text": "pick", "t": 1.0}, {"op": "say", "text": "the mug", "t": 1.5}, {"op": "confirm"}],
    [{"op": "say", "text": "pick", "t": 1.0}, {"op": "say", "text": "the mug", "t": 2.5}],
    [{"op": "say", "text": "the cup", "t": 1.0}, {"op": "silence", "t": 2.5}, {"op": "confirm"}],
    [{"op": "say", "text": "pick", "t": 1.0}, {"op": "silence", "t": 1.5}, {"op": "silence", "t": 2.2}],
    [{"op": "say", "text": "Council.", "t": 1.0}, {"op": "confirm"}],
    [{"op": "say", "text": "release", "t": 1.0}, {"op": "confirm"}, {"op": "say", "text": "release", "t": 2.0}],
    [{"op": "say", "text": "next", "t": 1.0}, {"op": "confirm"}, {"op": "say", "text": "next", "t": 5.0}, {"op": "confirm"}],
    [{"op": "say", "text": "cup", "t": 1.0}],
    [{"op": "say", "text": "pick the bottle", "t": 1.0}, {"op": "confirm"},
     {"op": "say", "text": "the can", "t": 3.0}, {"op": "silence", "t": 4.5}, {"op": "confirm"}],
]

OBJECTS = [[0.5, 0.15, -0.07], [0.45, -0.1, 0.0559], [0.45, -0.1, 0.056], [0.62, 0.2, 0.125], [0.3, 0.0, 0.3]]
RPY = [[math.pi, 0.0, math.pi / 2], [0.1, -0.2, 0.3], [-3.0, 1.2, 2.5], [0.0, 0.0, 0.0], [3.1, -0.1, -1.6]]


def build_vad_runs():
    rng = random.Random(99)
    runs = []
    for chunk_len, speech_amp in [(683, 0.08), (743, 0.04), (683, 0.012), (2048, 0.2)]:
        def chunk(amp):
            return [max(-32768, min(32767, int(rng.gauss(0, amp) * 32768))) for _ in range(chunk_len)]
        run = [chunk(0.003) for _ in range(20)]
        run += [chunk(speech_amp) for _ in range(12)] + [chunk(0.003) for _ in range(4)]
        run += [chunk(speech_amp) for _ in range(6)] + [chunk(0.003) for _ in range(15)]
        run += [chunk(speech_amp) for _ in range(2)] + [chunk(0.003) for _ in range(15)]
        run += [chunk(speech_amp) for _ in range(160)] + [chunk(0.003) for _ in range(15)]
        runs.append(run)
    return runs


# ---------------------------------------------------------------------------
# Python side (executed in a subprocess for each hash seed)
# ---------------------------------------------------------------------------

def install_stubs():
    fastapi = types.ModuleType("fastapi")

    class FastAPI:
        def __init__(self, *a, **k):
            pass

        def add_middleware(self, *a, **k):
            pass

        def mount(self, *a, **k):
            pass

        def _decorator(self, *a, **k):
            return lambda f: f

        get = post = websocket = on_event = _decorator

    class HTTPException(Exception):
        def __init__(self, status_code=None, detail=None):
            super().__init__(detail)

    fastapi.FastAPI = FastAPI
    fastapi.WebSocket = object
    fastapi.HTTPException = HTTPException
    fastapi.Form = lambda default=None, **k: default
    responses = types.ModuleType("fastapi.responses")
    responses.FileResponse = responses.HTMLResponse = object
    cors = types.ModuleType("fastapi.middleware.cors")
    cors.CORSMiddleware = object
    middleware = types.ModuleType("fastapi.middleware")
    static = types.ModuleType("fastapi.staticfiles")
    static.StaticFiles = lambda *a, **k: None
    for name, mod in [("fastapi", fastapi), ("fastapi.responses", responses), ("fastapi.middleware", middleware),
                      ("fastapi.middleware.cors", cors), ("fastapi.staticfiles", static)]:
        sys.modules[name] = mod

    openai = types.ModuleType("openai")
    openai.OpenAI = lambda **k: None
    sys.modules["openai"] = openai


class FakeTranscriber:
    """Replaces the OpenAI client: returns the scripted text for the next request."""

    def __init__(self):
        self.next_text = ""
        self.audio = types.SimpleNamespace(transcriptions=types.SimpleNamespace(create=self.create))

    def create(self, **kwargs):
        kwargs["file"].close()
        return types.SimpleNamespace(text=self.next_text)


class FakeSocket:
    def __init__(self):
        self.sent = []

    async def send_text(self, text):
        self.sent.append(json.loads(text))


class FakeRos:
    ready = True

    def __init__(self):
        self.published = []

    def publish_intent(self, data):
        self.published.append(["ros", "/hmi/intent_raw", data])

    def publish_asr(self, data):
        self.published.append(["ros", "/hmi/asr_text", data])


def python_worker():
    os.environ["CAPABILITIES_PATH"] = str(CAPABILITIES)
    os.environ["OPENAI_API_KEY"] = "stub"
    install_stubs()
    sys.path.insert(0, str(NODES))
    import logging
    logging.disable(logging.CRITICAL)

    import asr_whisper
    import hmi_server
    import websocket_handler
    from nlu_engine import load_capabilities
    from state import STATE
    from vad import VADProcessor

    job = json.loads(sys.stdin.read())
    out = {}
    model = load_capabilities()
    out["nlu"] = [model.parse(p) for p in job["phrases"]]
    cleaned = []
    for raw in job["transcripts"]:
        text = asr_whisper.sanitize_transcript(raw.strip(), None)
        text = asr_whisper._post_process_transcript(text)
        if text and not asr_whisper._english_like(text):
            text = ""
        cleaned.append(text)
    out["clean"] = cleaned
    out["lowconf"] = [asr_whisper.should_drop_low_confidence(*row) for row in job["lowconf"]]

    vad_traces = []
    for run in job["vadRuns"]:
        vad = VADProcessor()
        trace = []
        for samples in run:
            pcm = b"".join(int(v).to_bytes(2, "little", signed=True) for v in samples)
            if vad.state.calibrating:
                done = vad.process_calibration(pcm)
                trace.append(["calibrated", vad.state.noise_rms, vad.state.gate_rms] if done else ["calibrating"])
                continue
            was_in_voice = vad.state.in_voice
            seg = vad.process_audio(pcm)
            if seg:
                trace.append(["segment", len(seg) // 2])
            elif was_in_voice and not vad.state.in_voice:
                trace.append(["too_short"])
            else:
                trace.append(["voice" if vad.state.in_voice else "idle"])
        vad_traces.append(trace)
    out["vad"] = vad_traces

    # Dialogues: drive the real WebSocketHandler.handle_audio with a scripted
    # transcript, then the real /confirm and /discard handlers.
    transcriber = FakeTranscriber()
    asr_whisper.client = transcriber
    loud_segment = (1638).to_bytes(2, "little", signed=True) * 12800  # RMS 0.05, 800 ms at 16 kHz
    clock = {"t": 0.0}
    websocket_handler.time = types.SimpleNamespace(time=lambda: clock["t"])

    dialogues = []
    for steps in job["dialogues"]:
        STATE.__init__()
        ros = FakeRos()
        websocket_handler.get_ros_thread = lambda: ros
        hmi_server.get_ros_thread = lambda: ros
        handler = websocket_handler.WebSocketHandler(model)
        handler.vad_processor.state.calibrating = False
        handler.vad_processor.process_audio = lambda pcm, sr=16000: loud_segment
        sock = FakeSocket()
        STATE.clients.add(sock)
        log = []

        async def run_steps():
            for step in steps:
                sock.sent.clear()
                ros.published.clear()
                if step["op"] in ("say", "silence"):
                    clock["t"] = step["t"]
                    transcriber.next_text = step.get("text", "")
                    await handler.handle_audio(sock, b"\x00\x00" * 683)
                elif step["op"] == "confirm":
                    await hmi_server.confirm_intent(sig=None)
                else:
                    await hmi_server.discard_intent()
                # Interleave ROS publications and socket messages in emission order is not
                # observable here, so compare them as two ordered streams per step.
                for msg in sock.sent:
                    if msg.get("type") in ("intent", "locked"):
                        log.append(["ws", msg])
                log.extend(ros.published)

        asyncio.run(run_steps())
        dialogues.append(log)
    out["dialogues"] = dialogues
    sys.stdout.write(json.dumps(out))


# ---------------------------------------------------------------------------
# Reference values for the controller
# ---------------------------------------------------------------------------

def cpp_constants():
    cpp = (ROOT / "monitor_cpp" / "src" / "move_to_pose.cpp").read_text()
    launch = (ROOT / "monitor_cpp" / "launch" / "move_robot.launch.py").read_text()
    live = "\n".join(line for line in cpp.splitlines() if not line.strip().startswith("//"))

    def nums(text):
        return [float(v) for v in re.findall(r"-?\d+(?:\.\d+)?", text)]

    def one(pattern, text=live):
        m = re.search(pattern, text, re.M)
        if not m:
            raise SystemExit(f"pattern not found in move_to_pose.cpp: {pattern}")
        return float(m.group(1))

    def body(signature):
        start = live.index(signature)
        return live[start:live.index("\n}", start)]

    open_body = body("void open_gripper(")
    grasp_body = body("void grasp_object(")
    static_tf = re.search(r"static_transform_publisher.*?arguments=\[(.*?)\]", launch, re.S).group(1)
    tf_values = [float(v) for v in re.findall(r"'(-?\d+\.\d+)'", static_tf)]
    return {
        "intermediate": [nums(re.search(r"auto intermediate_pose = create_pose\(([^)]*)\)", live).group(1))],
        "release": [nums(re.search(r"auto release_pose = create_pose\(([^)]*)\)", live).group(1))],
        "grasp": {
            "width": one(r"goal\.width = ([\d.]+)", grasp_body),
            "speed": one(r"goal\.speed = ([\d.]+)", grasp_body),
            "force": one(r"goal\.force = ([\d.]+)", grasp_body),
            "epsilonInner": one(r"goal\.epsilon\.inner = ([\d.]+)", grasp_body),
            "epsilonOuter": one(r"goal\.epsilon\.outer = ([\d.]+)", grasp_body),
        },
        "open": {
            "width": one(r"goal\.width = ([\d.]+)", open_body),
            "speed": one(r"goal\.speed = ([\d.]+)", open_body),
        },
        "offsets": [
            -one(r"pick_pose\.position\.x -= ([\d.]+)"),
            -one(r"pick_pose\.position\.y -= ([\d.]+)"),
            one(r"if \(pick_pose\.position\.z < ([\d.]+)\)"),
            one(r"pick_pose\.position\.z = (-?[\d.]+);"),
            -one(r"pick_pose\.position\.z -= ([\d.]+)"),
            one(r"pre_grasp\.position\.z \+= ([\d.]+)"),
        ],
        "scaling": [one(r"setMaxVelocityScalingFactor\(([\d.]+)\)"),
                    one(r"setMaxAccelerationScalingFactor\(([\d.]+)\)")],
        "cartesian": [one(r"computeCartesianPath\(waypoints, ([\d.]+),"),
                      one(r"computeCartesianPath\(waypoints, [\d.]+, ([\d.]+),"),
                      one(r"if \(fraction > ([\d.]+)\)")],
        "timing": [one(r"double travel_time_offset = ([\d.]+);"),
                   one(r"sleep_for\(std::chrono::seconds\((\d+)\)\);\s*if \(move_cartesian\(move_group, pre_grasp"),
                   one(r"if \(dist > ([\d.]+)\)")],
        "camera": tf_values,
    }


def reference_pick(x, y, z):
    """Python transcription of the pick-pose block of move_to_pose.cpp."""
    px, py, pz = x - 0.02, y - 0.075, z
    pz = -0.024 if pz < 0.056 else pz - 0.08
    return px, py, pz, pz + 0.103


def reference_quaternion(r, p, y):
    """Quaternion of R = Rz(y) Ry(p) Rx(r), computed independently of tf2's formula."""
    def axis(ax, a):
        h = a / 2
        q = [0.0, 0.0, 0.0, math.cos(h)]
        q[ax] = math.sin(h)
        return q

    def mul(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return [aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz]

    return mul(mul(axis(2, y), axis(1, p)), axis(0, r))


# ---------------------------------------------------------------------------
# Comparison
# ---------------------------------------------------------------------------

def close(a, b, tol=1e-9):
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(a - b) <= tol
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(close(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(close(a[k], b[k], tol) for k in a)
    return a == b


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--python-worker":
        python_worker()
        return 0

    job = {
        "phrases": build_phrases(),
        "transcripts": TRANSCRIPTS,
        "lowconf": LOWCONF,
        "vadRuns": build_vad_runs(),
        "dialogues": DIALOGUES,
        "objects": OBJECTS,
        "rpy": RPY,
    }
    payload = json.dumps(job)

    python_runs = []
    for seed in SEEDS:
        env = dict(os.environ, PYTHONHASHSEED=str(seed))
        res = subprocess.run([sys.executable, __file__, "--python-worker"], input=payload,
                             capture_output=True, text=True, env=env, check=True)
        python_runs.append(json.loads(res.stdout))
    js = json.loads(subprocess.run(["node", str(RUNNER)], input=payload, capture_output=True,
                                   text=True, check=True).stdout)

    failures = []

    def report(name, ok, detail=""):
        print(f"[{'PASS' if ok else 'FAIL'}] {name}{(': ' + detail) if detail else ''}")
        if not ok:
            failures.append(name)

    report("capabilities.json copy", js["capabilities"] == json.loads(CAPABILITIES.read_text()))
    report("shared/sequence.json copy", js["mission"] == json.loads((ROOT / "shared" / "sequence.json").read_text()))

    # NLU: exact match when Python is deterministic, membership otherwise.
    deterministic = order_dependent = mismatches = 0
    examples = []
    for i, phrase in enumerate(job["phrases"]):
        outcomes = []
        for run in python_runs:
            if run["nlu"][i] not in outcomes:
                outcomes.append(run["nlu"][i])
        if len(outcomes) == 1:
            deterministic += 1
            ok = js["nlu"][i] == outcomes[0]
        else:
            order_dependent += 1
            ok = js["nlu"][i] in outcomes
            if len(examples) < 6:
                examples.append(f"{phrase!r} -> {outcomes}")
        if not ok:
            mismatches += 1
            if mismatches <= 10:
                print(f"    mismatch: {phrase!r}: js={js['nlu'][i]} python={outcomes}")
    report("NLU parse", mismatches == 0,
           f"{len(job['phrases'])} phrases, {deterministic} deterministic, {order_dependent} depend on "
           f"PYTHONHASHSEED across {len(SEEDS)} seeds, {mismatches} mismatches")
    for e in examples:
        print(f"    seed-dependent in Python: {e}")

    report("transcript cleaning", js["clean"] == python_runs[0]["clean"])
    report("low-confidence filter", js["lowconf"] == python_runs[0]["lowconf"])
    report("VAD traces", close(js["vad"], python_runs[0]["vad"], 1e-12),
           f"{sum(len(r) for r in job['vadRuns'])} chunks in {len(job['vadRuns'])} runs")

    dialogue_ok = True
    for i, steps in enumerate(DIALOGUES):
        py_variants = [run["dialogues"][i] for run in python_runs]
        js_log = [[e[0], json.loads(e[1])] if e[0] == "ws" else e for e in js["dialogues"][i]]
        if js_log not in py_variants:
            dialogue_ok = False
            print(f"    dialogue {i} differs:\n      js={js_log}\n      py={py_variants[0]}")
    report("intent / lock / confirm state machine", dialogue_ok, f"{len(DIALOGUES)} scripted dialogues")

    pose_ok = all(close(js["poses"][i][:4], list(reference_pick(*o)), 1e-12) for i, o in enumerate(OBJECTS))
    report("pick and pre-grasp poses", pose_ok)
    quat_ok = True
    for i, (r, p, y) in enumerate(RPY):
        ref = reference_quaternion(r, p, y)
        q = js["rpy"][i]
        if not (close(q, ref, 1e-12) or close(q, [-v for v in ref], 1e-12)):
            quat_ok = False
    report("tf2 setRPY quaternion", quat_ok)

    ref = cpp_constants()
    for key in ref:
        report(f"constant '{key}' matches C++/launch source", close(js["constants"][key], ref[key], 1e-12),
               "" if close(js["constants"][key], ref[key], 1e-12) else f"js={js['constants'][key]} src={ref[key]}")

    print()
    print("All checks passed." if not failures else f"{len(failures)} check(s) failed.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
