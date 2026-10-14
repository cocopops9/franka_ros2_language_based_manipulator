#!/usr/bin/env python3
"""Generate the animated SVG figures used by README.md and docs/.

The figures are derived from the project sources rather than drawn by hand:
  * waypoints, offsets and gripper goals are parsed from monitor_cpp/src/move_to_pose.cpp
    (through check_parity.cpp_constants);
  * the camera pose comes from the static transform in monitor_cpp/launch/move_robot.launch.py;
  * the default mission is read from shared/sequence.json;
  * the voice-activity trace is computed by running hmi_pkg/nodes/vad.py on a synthetic signal;
  * the NLU examples are parsed by hmi_pkg/nodes/nlu_engine.py.

Usage:  python3 docs/tools/generate_svgs.py      (writes docs/assets/svg/*.svg)
Only the Python standard library is required.
"""

import json
import math
import os
import random
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = ROOT / "docs" / "assets" / "svg"
sys.path.insert(0, str(HERE))

from check_parity import cpp_constants  # noqa: E402

C = {
    "bg": "#0b1220", "panel": "#111a2e", "panel2": "#0e1626", "stroke": "#26344f",
    "text": "#e6edf3", "sub": "#9aabc8", "dim": "#5d6d8a",
    "vision": "#22d3ee", "voice": "#a78bfa", "ctrl": "#f59e0b", "motion": "#34d399",
    "mission": "#f472b6", "tf": "#60a5fa", "warn": "#f87171", "ok": "#4ade80",
}
SANS = "-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,Helvetica,Arial,sans-serif"
MONO = "ui-monospace,SFMono-Regular,'SF Mono',Menlo,Consolas,'Liberation Mono',monospace"


def esc(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace('"', "&quot;")


def fmt(v, nd=1):
    return f"{v:.{nd}f}".rstrip("0").rstrip(".") if nd else f"{v:.0f}"


def document(w, h, body, title, desc, style=""):
    return f"""<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" viewBox="0 0 {w} {h}" width="{w}" height="{h}" role="img" aria-labelledby="title desc">
<title id="title">{esc(title)}</title>
<desc id="desc">{esc(desc)}</desc>
<defs>
  <pattern id="grid" width="24" height="24" patternUnits="userSpaceOnUse">
    <path d="M24 0H0V24" fill="none" stroke="#16213a" stroke-width="1"/>
  </pattern>
  <filter id="glow" x="-50%" y="-50%" width="200%" height="200%">
    <feGaussianBlur stdDeviation="3" result="b"/>
    <feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge>
  </filter>
</defs>
<style>
  .t {{ font-family: {SANS}; fill: {C['text']}; }}
  .s {{ font-family: {SANS}; fill: {C['sub']}; }}
  .d {{ font-family: {SANS}; fill: {C['dim']}; }}
  .m {{ font-family: {MONO}; fill: {C['text']}; }}
  .ms {{ font-family: {MONO}; fill: {C['sub']}; }}
  .b {{ font-weight: 700; }}
  .card {{ fill: {C['panel']}; stroke: {C['stroke']}; stroke-width: 1.2; }}
  {style}
</style>
<rect width="{w}" height="{h}" rx="18" fill="{C['bg']}"/>
<rect width="{w}" height="{h}" rx="18" fill="url(#grid)" opacity="0.55"/>
{body}
</svg>
"""


def card(x, y, w, h, color, title, lines=(), title_size=16, line_size=12.5, mono_lines=False):
    out = [f'<rect class="card" x="{x}" y="{y}" width="{w}" height="{h}" rx="12"/>',
           f'<rect x="{x}" y="{y + 12}" width="4" height="{h - 24}" rx="2" fill="{color}"/>',
           f'<text class="t b" x="{x + 18}" y="{y + 27}" font-size="{title_size}">{esc(title)}</text>']
    cls = "ms" if mono_lines else "s"
    for i, line in enumerate(lines):
        out.append(f'<text class="{cls}" x="{x + 18}" y="{y + 48 + i * (line_size + 5)}" font-size="{line_size}">{esc(line)}</text>')
    return "\n".join(out)


def pill(x, y, text, color, size=12, mono=True, anchor="middle"):
    width = len(text) * size * (0.61 if mono else 0.56) + 14
    if anchor == "middle":
        rx = x - width / 2
    elif anchor == "start":
        rx = x
    else:
        rx = x - width
    cls = "m" if mono else "t"
    tx = rx + width / 2
    return (f'<rect x="{rx:.1f}" y="{y - size + 1:.1f}" width="{width:.1f}" height="{size + 9}" rx="{(size + 9) / 2:.1f}" '
            f'fill="{C["bg"]}" stroke="{color}" stroke-opacity="0.65"/>'
            f'<text class="{cls}" x="{tx:.1f}" y="{y + 3.5:.1f}" font-size="{size}" text-anchor="middle" fill="{color}" style="fill:{color}">{esc(text)}</text>')


def arrow_marker(mid, color):
    return (f'<marker id="{mid}" viewBox="0 0 10 10" refX="8.5" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            f'<path d="M0 0L10 5L0 10z" fill="{color}"/></marker>')


def edge(pid, d, color, dashed=False, width=2.2):
    dash = ' stroke-dasharray="6 6"' if dashed else ""
    return (f'<path id="{pid}" d="{d}" fill="none" stroke="{color}" stroke-opacity="0.55" stroke-width="{width}"{dash} '
            f'marker-end="url(#a-{pid})"/>')


def packets(pid, color, dur, count=2, radius=4.5, begin=0.0, label=None):
    out = []
    for i in range(count):
        b = begin + i * dur / count
        out.append(f'<circle r="{radius}" fill="{color}" filter="url(#glow)" opacity="0">'
                   f'<animate attributeName="opacity" values="0;1;1;0" keyTimes="0;0.08;0.9;1" dur="{dur}s" begin="{b:.2f}s" repeatCount="indefinite"/>'
                   f'<animateMotion dur="{dur}s" begin="{b:.2f}s" repeatCount="indefinite" rotate="auto">'
                   f'<mpath xlink:href="#{pid}" href="#{pid}"/></animateMotion></circle>')
    return "\n".join(out)


# ---------------------------------------------------------------------------
# Shared project data
# ---------------------------------------------------------------------------

CONST = cpp_constants()
MISSION = json.loads((ROOT / "shared" / "sequence.json").read_text())


def quat_to_matrix(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


CAM_T = CONST["camera"][:3]
CAM_R = quat_to_matrix(CONST["camera"][3:])


def base_to_camera(p):
    d = [p[i] - CAM_T[i] for i in range(3)]
    return [sum(CAM_R[r][c] * d[r] for r in range(3)) for c in range(3)]


def camera_to_base(p):
    return [sum(CAM_R[r][c] * p[c] for c in range(3)) + CAM_T[r] for r in range(3)]


# Example colour-stream intrinsics (RealSense D4xx at 640x480). The node reads the
# real values from /camera/camera/color/camera_info at runtime.
K = {"fx": 615.0, "fy": 615.0, "cx": 320.0, "cy": 240.0, "w": 640, "h": 480}


def project(p_cam):
    return K["fx"] * p_cam[0] / p_cam[2] + K["cx"], K["fy"] * p_cam[1] / p_cam[2] + K["cy"]


def pick_from_object(x, y, z):
    off = CONST["offsets"]  # [dx, dy, low_z, floor_z, table_dz, hover]
    px, py = x + off[0], y + off[1]
    pz = off[3] if z < off[2] else z + off[4]
    return (px, py, pz), (px, py, pz + off[5])


# Example object positions in fr3_link0 (they depend on the scene; illustrative only).
# Flat objects carry their marker at table height, the bottle on its cap.
EXAMPLE_OBJECTS = {
    "fork": (0.52, 0.20, -0.07),
    "spoon": (0.40, 0.12, -0.07),
    "bottle": (0.60, 0.05, 0.10),
    "toy": (0.47, -0.05, -0.04),
}
TABLE_Z = -0.075  # height of the calibration marker in extrinsinc_calibration.py


# ---------------------------------------------------------------------------
# Figure: hero banner
# ---------------------------------------------------------------------------

def hero():
    w, h = 1200, 300
    body = []
    body.append(f'<text class="t b" x="64" y="118" font-size="64" letter-spacing="1">Dum-E</text>')
    body.append(f'<text class="s" x="66" y="158" font-size="21">Voice-guided collaborative robot for tool handover in sequential tasks</text>')
    tags = [("ROS 2 Humble", C["tf"]), ("MoveIt 2", C["motion"]), ("Franka FR3", C["ctrl"]),
            ("YOLO-World + ArUco", C["vision"]), ("Whisper ASR + NLU", C["voice"])]
    x = 66
    for label, color in tags:
        wdt = len(label) * 8.2 + 26
        body.append(f'<rect x="{x}" y="186" width="{wdt:.0f}" height="30" rx="15" fill="none" stroke="{color}" stroke-opacity="0.8"/>')
        body.append(f'<text class="t" x="{x + wdt / 2:.0f}" y="206" font-size="14" text-anchor="middle" style="fill:{color}">{esc(label)}</text>')
        x += wdt + 10
    body.append(f'<text class="d" x="66" y="258" font-size="14">CIR-MAI course project: tools are detected by vision, scheduled by a mission file, and released on a spoken command.</text>')

    # Stylised arm on the right: it sways while holding a tool, gripper kept vertical.
    bx, by = 960, 262
    body.append(f'<ellipse cx="{bx}" cy="{by + 4}" rx="70" ry="10" fill="#0f1a30"/>')
    body.append(f'<rect x="{bx - 34}" y="{by - 26}" width="68" height="28" rx="8" fill="#1b2742" stroke="{C["stroke"]}"/>')
    r1, r2 = "-14;-2;-14", "-112;-96;-112"
    r3 = "126;98;126"
    arm = f"""
<g transform="translate({bx} {by - 26})">
  <g>
    <animateTransform attributeName="transform" type="rotate" values="{r1}" dur="6s" repeatCount="indefinite"/>
    <rect x="-12" y="-100" width="24" height="104" rx="12" fill="#e8edf5"/>
    <circle cx="0" cy="-98" r="16" fill="#cfd8e6" stroke="#9aa8bf"/>
    <g transform="translate(0 -98)">
      <g>
        <animateTransform attributeName="transform" type="rotate" values="{r2}" dur="6s" repeatCount="indefinite"/>
        <rect x="-10" y="-92" width="20" height="96" rx="10" fill="#e8edf5"/>
        <circle cx="0" cy="-90" r="13" fill="#cfd8e6" stroke="#9aa8bf"/>
        <g transform="translate(0 -90)">
          <g>
            <animateTransform attributeName="transform" type="rotate" values="{r3}" dur="6s" repeatCount="indefinite"/>
            <rect x="-7" y="0" width="14" height="26" rx="4" fill="#2b3550"/>
            <rect x="-17" y="24" width="34" height="9" rx="3" fill="#cfd8e6"/>
            <rect x="-15" y="31" width="6" height="20" rx="2" fill="#e8edf5"/>
            <rect x="9" y="31" width="6" height="20" rx="2" fill="#e8edf5"/>
            <rect x="-3" y="36" width="6" height="34" rx="3" fill="{C['ctrl']}"/>
          </g>
        </g>
      </g>
    </g>
  </g>
</g>"""
    body.append(arm)
    # Camera with scanning beam
    body.append(f'<rect x="1080" y="44" width="58" height="22" rx="6" fill="#1b2742" stroke="{C["vision"]}" stroke-opacity="0.7"/>')
    body.append(f'<circle cx="1097" cy="55" r="5" fill="{C["vision"]}"/><circle cx="1121" cy="55" r="5" fill="{C["vision"]}" opacity="0.5"/>')
    body.append(f'<path d="M1109 66L1040 262L1170 262Z" fill="{C["vision"]}" opacity="0.07">'
                f'<animate attributeName="opacity" values="0.03;0.12;0.03" dur="3s" repeatCount="indefinite"/></path>')
    # Voice wave
    for i in range(5):
        hgt = [14, 26, 38, 26, 14][i]
        body.append(f'<rect x="{770 + i * 12}" y="{90 - hgt / 2}" width="6" height="{hgt}" rx="3" fill="{C["voice"]}">'
                    f'<animate attributeName="height" values="{hgt};{hgt * 0.35:.0f};{hgt}" dur="{0.8 + i * 0.13:.2f}s" repeatCount="indefinite"/>'
                    f'<animate attributeName="y" values="{90 - hgt / 2};{90 - hgt * 0.175:.1f};{90 - hgt / 2}" dur="{0.8 + i * 0.13:.2f}s" repeatCount="indefinite"/></rect>')
    body.append(f'<text class="m" x="752" y="132" font-size="14" style="fill:{C["voice"]}">"release"</text>')
    return document(w, h, "\n".join(body), "Dum-E", "Banner: a stylised robot arm, a depth camera and a voice waveform.")


# ---------------------------------------------------------------------------
# Figure: system architecture with animated message flow
# ---------------------------------------------------------------------------

def architecture():
    w, h = 1200, 760
    b = []
    b.append(f'<text class="t b" x="40" y="46" font-size="22">System architecture: who talks to whom</text>')
    b.append(f'<text class="s" x="40" y="70" font-size="14">Moving dots are messages. Colours: perception, voice, mission data, transforms, motion.</text>')

    edges = {
        "cam-vis": ("M240 135 L318 135", C["vision"], False),
        "vis-ctl": ("M590 128 L850 128 L850 278", C["vision"], False),
        "tf-ctl": ("M590 265 L708 265", C["tf"], False),
        "mis-ctl": ("M590 366 L708 366", C["mission"], False),
        "mis-vis": ("M330 388 L300 388 L300 172 L318 172", C["mission"], False),
        "fe-mis": ("M240 366 L318 366", C["mission"], True),
        "op-br": ("M220 590 L258 590", C["voice"], False),
        "br-hmi": ("M470 578 L518 578", C["voice"], False),
        "hmi-br": ("M530 606 L482 606", C["voice"], False),
        "hmi-wh": ("M640 646 L640 688", C["voice"], True),
        "wh-hmi": ("M700 690 L700 650", C["voice"], True),
        "hmi-ctl": ("M790 590 L850 590 L850 432", C["voice"], False),
        "ctl-mv": ("M990 300 L1032 300", C["motion"], False),
        "mv-arm": ("M1110 352 L1110 398", C["motion"], False),
        "ctl-gr": ("M990 400 L1012 400 L1012 560 L1032 560", C["ctrl"], False),
    }
    b.append("<defs>" + "".join(arrow_marker(f"a-{k}", v[1]) for k, v in edges.items()) + "</defs>")
    for k, (d, color, dashed) in edges.items():
        b.append(edge(k, d, color, dashed))

    b.append(card(40, 92, 200, 92, C["vision"], "RealSense camera", ["realsense2_camera", "align_depth.enable:=true"]))
    b.append(card(330, 92, 260, 112, C["vision"], "vision_node", ["YOLO-World  yolov8l-worldv2.pt", "ArUco inside each box (15 mm)", "depth lookup + pinhole deprojection"]))
    b.append(card(330, 232, 260, 70, C["tf"], "static_transform_publisher", ["fr3_link0 -> camera_link (calibrated)"]))
    b.append(card(330, 330, 260, 78, C["mission"], "shared/sequence.json", ["tools[]  ->  YOLO-World classes", "sequence[{tool, startTime}]"]))
    b.append(card(40, 330, 200, 78, C["mission"], "frontend/ (static page)", ["edit tools and sequence", "export dum-e-sequence.json"]))
    b.append(card(720, 280, 270, 150, C["ctrl"], "move_to_pose_node", ["object memory (label -> x, y, z)", "  update only if moved > 2 cm", "scheduler: go at startTime − 5 s", "pick -> carry -> wait for 'release'", "C++ / rclcpp / MoveGroupInterface"]))
    b.append(card(1044, 248, 132, 102, C["motion"], "move_group", ["MoveIt 2", "Cartesian paths", "eef_step 1 cm"]))
    b.append(card(1044, 400, 132, 92, C["motion"], "FR3 arm", ["fr3_arm group", "tcp: fr3_hand_tcp"]))
    b.append(card(1044, 516, 132, 92, C["ctrl"], "Franka Hand", ["grasp: 4 cm, 40 N", "move: open 8 cm"]))
    b.append(card(40, 548, 180, 84, C["voice"], "Operator", ["speaks a command", "e.g. \"release\""]))
    b.append(card(270, 548, 200, 84, C["voice"], "HMI web page", ["hmi_pkg/public/index.html", "PCM16 at 16 kHz"]))
    b.append(card(530, 528, 260, 118, C["voice"], "hmi_server (FastAPI)", ["energy VAD -> segment", "Whisper (whisper-1) -> text", "NLU -> {action, object}", "lock until Confirm / Discard"]))
    b.append(card(560, 690, 200, 56, C["voice"], "OpenAI Whisper API", []))
    b.append(f'<text class="s" x="578" y="734" font-size="12">HTTPS, one WAV per segment</text>')

    b.append(pill(720, 112, "/vision/detected_objects", C["vision"]))
    b.append(pill(720, 150, "String: JSON [{label,x,y,z}] (camera_link)", C["vision"], size=10.5))
    b.append(pill(649, 252, "/tf_static", C["tf"], size=11))
    b.append(pill(649, 352, "mission_file", C["mission"], size=11))
    b.append(pill(279, 352, "manual copy", C["mission"], size=10.5, mono=False))
    b.append(pill(850, 452, "/hmi/intent_raw", C["voice"]))
    b.append(pill(850, 482, "{action, object, allow_execution}", C["voice"], size=10.5))
    b.append(pill(494, 520, "WS /asr", C["voice"], size=11))
    b.append(pill(1005, 528, "/franka_gripper/*", C["ctrl"], size=10.5, anchor="end"))
    b.append(pill(279, 120, "RGB + depth + K", C["vision"], size=10.5, mono=False))

    b.append(packets("cam-vis", C["vision"], 1.2, 2))
    b.append(packets("vis-ctl", C["vision"], 2.4, 3))
    b.append(packets("tf-ctl", C["tf"], 3.0, 1))
    b.append(packets("mis-ctl", C["mission"], 4.0, 1))
    b.append(packets("mis-vis", C["mission"], 4.0, 1))
    b.append(packets("op-br", C["voice"], 1.0, 2, radius=3.5))
    b.append(packets("br-hmi", C["voice"], 1.0, 2, radius=3.5))
    b.append(packets("hmi-br", C["voice"], 3.0, 1, radius=3.5, begin=1.0))
    b.append(packets("hmi-wh", C["voice"], 3.0, 1, radius=3.5, begin=0.5))
    b.append(packets("wh-hmi", C["voice"], 3.0, 1, radius=3.5, begin=1.8))
    b.append(packets("hmi-ctl", C["voice"], 3.0, 1, begin=2.2))
    b.append(packets("ctl-mv", C["motion"], 1.6, 2))
    b.append(packets("mv-arm", C["motion"], 1.6, 2))
    b.append(packets("ctl-gr", C["ctrl"], 3.2, 1, begin=1.0))
    return document(w, h, "\n".join(b), "System architecture",
                    "ROS 2 nodes, topics and data files of the project, with animated message flow.")


# ---------------------------------------------------------------------------
# Animation helpers
# ---------------------------------------------------------------------------

def visible(intervals, dur, on="1", off="0"):
    """SMIL <animate> setting opacity to `on` during the given [start, end) intervals (seconds)."""
    values, times = [], []
    state = on if (intervals and intervals[0][0] <= 0) else off
    values.append(state)
    times.append(0.0)
    for a, b in intervals:
        for t, v in ((a, on), (b, off)):
            if t <= 0:
                continue
            if t >= dur:
                continue
            t = max(t, times[-1] + 1e-4)
            values.append(v)
            times.append(t)
    kt = ";".join(f"{t / dur:.5f}" for t in times)
    return (f'<animate attributeName="opacity" values="{";".join(values)}" keyTimes="{kt}" '
            f'dur="{dur:.3f}s" calcMode="discrete" repeatCount="indefinite"/>')


def fade_group(content, intervals, dur):
    return f'<g opacity="0">{visible(intervals, dur)}{content}</g>'


# ---------------------------------------------------------------------------
# Figure: perception chain (vision_node.image_callback)
# ---------------------------------------------------------------------------

def perception():
    w, h = 1200, 560
    D = 16.0
    b = []
    b.append('<text class="t b" x="40" y="46" font-size="22">Perception: from pixels to a 3-D point the robot can reach</text>')
    b.append('<text class="s" x="40" y="70" font-size="14">vision_node.image_callback() for every colour frame. Scene and intrinsics are an example; the steps and formulas are the code\'s.</text>')

    # Image panel (640x480 scaled by 0.62)
    ix, iy, sc = 40, 92, 0.62
    iw, ih = K["w"] * sc, K["h"] * sc
    b.append(f'<rect x="{ix}" y="{iy}" width="{iw:.0f}" height="{ih:.0f}" rx="8" fill="#1a2336" stroke="{C["stroke"]}"/>')
    b.append(f'<text class="d" x="{ix + 8}" y="{iy + ih + 18:.0f}" font-size="12">/camera/camera/color/image_raw  (640 x 480, example)</text>')

    # Table texture: projected grid lines of the table plane
    for gx in [0.2, 0.3, 0.4, 0.5, 0.6, 0.7]:
        pts = []
        for gy in [-0.3, 0.45]:
            u, v = project(base_to_camera([gx, gy, TABLE_Z]))
            pts.append((ix + u * sc, iy + v * sc))
        b.append(f'<line x1="{pts[0][0]:.1f}" y1="{pts[0][1]:.1f}" x2="{pts[1][0]:.1f}" y2="{pts[1][1]:.1f}" stroke="#24304a" stroke-width="1"/>')
    for gy in [-0.2, -0.1, 0.0, 0.1, 0.2, 0.3]:
        pts = []
        for gx in [0.15, 0.8]:
            u, v = project(base_to_camera([gx, gy, TABLE_Z]))
            pts.append((ix + u * sc, iy + v * sc))
        b.append(f'<line x1="{pts[0][0]:.1f}" y1="{pts[0][1]:.1f}" x2="{pts[1][0]:.1f}" y2="{pts[1][1]:.1f}" stroke="#24304a" stroke-width="1"/>')
    b.append(f'<rect x="{ix}" y="{iy}" width="{iw:.0f}" height="{ih:.0f}" rx="8" fill="none" stroke="{C["stroke"]}"/>')

    # Objects (fork with marker, spoon with marker, bottle with marker on the cap, a second fork without marker)
    scene = [
        ("fork", EXAMPLE_OBJECTS["fork"], True, (46, 16)),
        ("spoon", EXAMPLE_OBJECTS["spoon"], True, (42, 16)),
        ("bottle", EXAMPLE_OBJECTS["bottle"], True, (30, 54)),
        ("fork", (0.66, 0.27, -0.07), False, (46, 16)),
    ]
    info = []
    for idx, (label, pos, marker, (bw, bh)) in enumerate(scene):
        pc = base_to_camera(list(pos))
        u, v = project(pc)
        cx_, cy_ = ix + u * sc, iy + v * sc
        info.append((label, pos, marker, u, v, pc, cx_, cy_, bw, bh))
        color = "#c9d3e3" if label != "bottle" else "#7dd3fc"
        if label == "bottle":
            b.append(f'<rect x="{cx_ - 9:.1f}" y="{cy_ - 4:.1f}" width="18" height="40" rx="6" fill="{color}" opacity="0.85"/>')
            b.append(f'<rect x="{cx_ - 5:.1f}" y="{cy_ - 10:.1f}" width="10" height="9" rx="2" fill="#94a3b8"/>')
        else:
            b.append(f'<rect x="{cx_ - bw / 2 + 4:.1f}" y="{cy_ - 3:.1f}" width="{bw - 8}" height="6" rx="3" fill="{color}" transform="rotate(-18 {cx_:.1f} {cy_:.1f})"/>')
        if marker:
            b.append(f'<rect x="{cx_ - 5:.1f}" y="{cy_ - 5:.1f}" width="10" height="10" fill="#0b0f19" stroke="#ffffff" stroke-width="1.6"/>'
                     f'<rect x="{cx_ - 2:.1f}" y="{cy_ - 2:.1f}" width="3" height="5" fill="#ffffff"/>')

    # Stage 1 (2-5 s): YOLO-World boxes
    tools = MISSION["tools"]
    boxes = []
    for label, pos, marker, u, v, pc, cx_, cy_, bw, bh in info:
        boxes.append(f'<rect x="{cx_ - bw / 2:.1f}" y="{cy_ - bh / 2:.1f}" width="{bw}" height="{bh}" rx="3" fill="none" stroke="#ffffff" stroke-width="1.5" stroke-dasharray="4 3"/>'
                     f'<text class="m" x="{cx_ - bw / 2:.1f}" y="{cy_ - bh / 2 - 4:.1f}" font-size="10.5" style="fill:#ffffff">{label} (id {tools.index(label)})</text>')
    b.append(fade_group("".join(boxes), [(2.0, 5.0)], D))

    # Stage 2 (5-8 s): ArUco inside each ROI -> method ARUCO (red) or YOLO (green)
    res = []
    for label, pos, marker, u, v, pc, cx_, cy_, bw, bh in info:
        color = "#ff4d4d" if marker else "#4ade80"
        method = "ARUCO" if marker else "YOLO"
        res.append(f'<rect x="{cx_ - bw / 2:.1f}" y="{cy_ - bh / 2:.1f}" width="{bw}" height="{bh}" rx="3" fill="none" stroke="{color}" stroke-width="2"/>'
                   f'<text class="m b" x="{cx_ - bw / 2:.1f}" y="{cy_ - bh / 2 - 4:.1f}" font-size="10.5" style="fill:{color}">{method}</text>')
        if marker:
            res.append(f'<circle cx="{cx_:.1f}" cy="{cy_:.1f}" r="2.6" fill="#ffd60a"/>')
    b.append(fade_group("".join(res), [(5.0, D)], D))

    # Right side: steps list
    steps = [
        ("1", "YOLO-World proposes boxes", f"classes = tools[] ({', '.join(tools)})", 2.0, 5.0, C["vision"]),
        ("2", "ArUco search inside each box", "DICT_ARUCO_ORIGINAL, crop = box", 5.0, 8.0, C["vision"]),
        ("3", "Depth at one pixel", "z = depth[v, u] / 1000 (marker centre)", 8.0, 10.5, C["vision"]),
        ("4", "Pinhole deprojection", "x = (u-cx) z/fx,  y = (v-cy) z/fy", 10.5, 13.0, C["vision"]),
        ("5", "Publish ARUCO detections only", "/vision/detected_objects", 13.0, D, C["vision"]),
    ]
    sx0, sy0 = 470, 100
    for i, (num, title, sub, a, bnd, color) in enumerate(steps):
        y = sy0 + i * 58
        b.append(f'<rect x="{sx0}" y="{y}" width="330" height="50" rx="10" class="card"/>')
        b.append(f'<rect x="{sx0}" y="{y}" width="330" height="50" rx="10" fill="{color}" opacity="0">{visible([(a, bnd)], D, on="0.16")}</rect>')
        b.append(f'<rect x="{sx0}" y="{y}" width="330" height="50" rx="10" fill="none" stroke="{color}" stroke-width="2" opacity="0">{visible([(a, bnd)], D)}</rect>')
        b.append(f'<circle cx="{sx0 + 24}" cy="{y + 25}" r="13" fill="none" stroke="{color}"/>'
                 f'<text class="t b" x="{sx0 + 24}" y="{y + 30}" font-size="13" text-anchor="middle" style="fill:{color}">{num}</text>')
        b.append(f'<text class="t b" x="{sx0 + 46}" y="{y + 21}" font-size="14">{esc(title)}</text>')
        b.append(f'<text class="ms" x="{sx0 + 46}" y="{y + 39}" font-size="10.5">{esc(sub)}</text>')
    # Detail panel (right-most) for stages 3-5, using the fork with marker
    label, pos, marker, u, v, pc, cx_, cy_, bw, bh = info[0]
    ui, vi = int(round(u)), int(round(v))
    z = round(pc[2] * 1000) / 1000.0
    depth_mm = int(round(pc[2] * 1000))
    xc = (ui - K["cx"]) * z / K["fx"]
    yc = (vi - K["cy"]) * z / K["fy"]
    pb = camera_to_base([xc, yc, z])
    dx0, dy0 = 820, 100
    b.append(f'<rect class="card" x="{dx0}" y="{dy0}" width="340" height="282" rx="12"/>')
    b.append(f'<text class="t b" x="{dx0 + 18}" y="{dy0 + 28}" font-size="14">Worked example: the marked fork</text>')
    d3 = (f'<text class="m" x="{dx0 + 18}" y="{dy0 + 60}" font-size="12.5">marker centre  u = {ui}, v = {vi}</text>'
          f'<text class="m" x="{dx0 + 18}" y="{dy0 + 82}" font-size="12.5">depth[{vi}, {ui}] = {depth_mm} mm</text>'
          f'<text class="m" x="{dx0 + 18}" y="{dy0 + 104}" font-size="12.5" style="fill:{C["vision"]}">z = {z:.3f} m</text>')
    d4 = (f'<text class="ms" x="{dx0 + 18}" y="{dy0 + 134}" font-size="11.5">K: fx = fy = {K["fx"]:.0f}, cx = {K["cx"]:.0f}, cy = {K["cy"]:.0f}</text>'
          f'<text class="m" x="{dx0 + 18}" y="{dy0 + 156}" font-size="12.5">x = ({ui} - {K["cx"]:.0f}) * {z:.3f} / {K["fx"]:.0f} = {xc:.3f}</text>'
          f'<text class="m" x="{dx0 + 18}" y="{dy0 + 178}" font-size="12.5">y = ({vi} - {K["cy"]:.0f}) * {z:.3f} / {K["fy"]:.0f} = {yc:.3f}</text>')
    d5 = (f'<text class="ms" x="{dx0 + 18}" y="{dy0 + 208}" font-size="11">{esc(json.dumps({"label": "fork", "x": round(xc, 3), "y": round(yc, 3), "z": round(z, 3)}))}</text>'
          f'<text class="s" x="{dx0 + 18}" y="{dy0 + 232}" font-size="12">controller: tf2 transform camera_link -> fr3_link0</text>'
          f'<text class="m" x="{dx0 + 18}" y="{dy0 + 256}" font-size="12.5" style="fill:{C["ctrl"]}">p_base = ({pb[0]:.3f}, {pb[1]:.3f}, {pb[2]:.3f})</text>')
    b.append(fade_group(d3, [(8.0, D)], D))
    b.append(fade_group(d4, [(10.5, D)], D))
    b.append(fade_group(d5, [(13.0, D)], D))
    b.append(f'<text class="d" x="{dx0 + 18}" y="{dy0 + 274}" font-size="11">true position used to draw the scene: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})</text>')

    # Bottom notes
    notes = [
        "The unmarked fork is drawn on /debug_image (green) but is not published: only method == ARUCO reaches the controller.",
        "Depth is read at a single pixel (no averaging); pixels with depth 0 are discarded by the z_cam > 0 test.",
        "Coordinates leave the node in the camera frame; the 3-D transform happens in move_to_pose_node through tf2.",
    ]
    for i, n in enumerate(notes):
        b.append(f'<text class="s" x="40" y="{442 + i * 22}" font-size="13">{esc(n)}</text>')
    b.append(f'<text class="d" x="40" y="{520}" font-size="12">Loop: {D:.0f} s. Pixel and depth values are computed from the example scene with the calibrated extrinsics of move_robot.launch.py.</text>')
    return document(w, h, "\n".join(b), "Perception chain",
                    "YOLO-World boxes, ArUco check inside each box, depth lookup, deprojection and publication.")


# ---------------------------------------------------------------------------
# Figure: frames and calibration (side view, y-z plane of fr3_link0)
# ---------------------------------------------------------------------------

def frames():
    w, h = 1200, 520
    b = []
    b.append('<text class="t b" x="40" y="46" font-size="22">Frames: fr3_link0, camera_link and the calibration marker</text>')
    b.append('<text class="s" x="40" y="70" font-size="14">Side view looking along -x (robot y to the right, z up). Camera pose = static transform in move_robot.launch.py.</text>')
    ox, oy, s = 330, 360, 330.0  # screen position of base origin, pixels per metre

    def P(yv, zv):
        return ox + yv * s, oy - zv * s

    # table
    tx0, ty = P(-0.75, TABLE_Z)
    tx1, _ = P(0.62, TABLE_Z)
    b.append(f'<rect x="{tx0:.0f}" y="{ty:.0f}" width="{tx1 - tx0:.0f}" height="14" fill="#1a2438"/>')
    b.append(f'<text class="d" x="{tx0 + 6:.0f}" y="{ty + 32:.0f}" font-size="12">table, z = -0.075 m (height of the calibration marker)</text>')
    # robot column
    bx, by = P(0, 0)
    b.append(f'<rect x="{bx - 22:.0f}" y="{by - 200:.0f}" width="44" height="200" rx="16" fill="#1f2a44" stroke="{C["stroke"]}"/>')
    b.append(f'<rect x="{bx - 40:.0f}" y="{by - 4:.0f}" width="80" height="{ty - by + 4:.0f}" rx="4" fill="#253255"/>')
    # base axes
    def axis(x0, y0, dx, dy, color, label, length=70):
        x1, y1 = x0 + dx * length, y0 - dy * length
        return (f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" stroke="{color}" stroke-width="3" marker-end="url(#ar-{color[1:]})"/>'
                f'<text class="m b" x="{x1 + dx * 14 - 4:.1f}" y="{y1 - dy * 14 + 4:.1f}" font-size="13" style="fill:{color}">{label}</text>')
    colors = ["#f87171", "#4ade80", "#60a5fa"]
    b.append("<defs>" + "".join(arrow_marker(f"ar-{c[1:]}", c) for c in colors + [C["vision"], C["ctrl"]]) + "</defs>")
    b.append(axis(bx, by, 1, 0, colors[1], "y"))
    b.append(axis(bx, by, 0, 1, colors[2], "z"))
    b.append(f'<circle cx="{bx:.1f}" cy="{by:.1f}" r="7" fill="none" stroke="{colors[0]}" stroke-width="2"/><circle cx="{bx:.1f}" cy="{by:.1f}" r="2" fill="{colors[0]}"/>')
    b.append(f'<text class="m b" x="{bx - 26:.1f}" y="{by + 22:.1f}" font-size="13" style="fill:{colors[0]}">x</text>')
    b.append(f'<text class="t b" x="{bx - 112:.1f}" y="{by - 8:.1f}" font-size="13">fr3_link0</text>')

    # camera
    cx_, cy_ = P(CAM_T[1], CAM_T[2])
    R = CAM_R
    zc = (R[1][2], R[2][2])
    yc = (R[1][1], R[2][1])
    b.append(f'<rect x="{cx_ - 26:.1f}" y="{cy_ - 12:.1f}" width="52" height="24" rx="6" fill="#1b2742" stroke="{C["vision"]}" transform="rotate({math.degrees(math.atan2(-zc[1], zc[0])) + 90:.1f} {cx_:.1f} {cy_:.1f})"/>')
    # frustum in this plane: rotate optical axis by +-half vertical FOV about camera x
    half = math.atan((K["h"] / 2) / K["fy"])
    for sgn in (-1, 1):
        dirv = [math.cos(half) * zc[0] + sgn * math.sin(half) * yc[0], math.cos(half) * zc[1] + sgn * math.sin(half) * yc[1]]
        t = (TABLE_Z - CAM_T[2]) / dirv[1]
        ex, ey = P(CAM_T[1] + t * dirv[0], TABLE_Z)
        b.append(f'<line x1="{cx_:.1f}" y1="{cy_:.1f}" x2="{ex:.1f}" y2="{ey:.1f}" stroke="{C["vision"]}" stroke-opacity="0.35" stroke-dasharray="5 5"/>')
    t_axis = (TABLE_Z - CAM_T[2]) / zc[1]
    hit = [CAM_T[i] + t_axis * R[i][2] for i in range(3)]
    hx, hy = P(hit[1], hit[2])
    b.append(f'<line x1="{cx_:.1f}" y1="{cy_:.1f}" x2="{hx:.1f}" y2="{hy:.1f}" stroke="{C["vision"]}" stroke-opacity="0.6"/>')
    b.append(axis(cx_, cy_, zc[0], zc[1], C["vision"], "z_c", 80))
    b.append(axis(cx_, cy_, yc[0], yc[1], "#4ade80", "y_c", 55))
    b.append(f'<circle cx="{cx_:.1f}" cy="{cy_:.1f}" r="7" fill="none" stroke="{colors[0]}" stroke-width="2"/>'
             f'<path d="M{cx_ - 4:.1f} {cy_ - 4:.1f}L{cx_ + 4:.1f} {cy_ + 4:.1f}M{cx_ + 4:.1f} {cy_ - 4:.1f}L{cx_ - 4:.1f} {cy_ + 4:.1f}" stroke="{colors[0]}" stroke-width="1.6"/>')
    b.append(f'<text class="t b" x="{cx_ + 16:.1f}" y="{cy_ - 18:.1f}" font-size="13">camera_link</text>')
    b.append(f'<text class="ms" x="{cx_ + 16:.1f}" y="{cy_ - 2:.1f}" font-size="11">x_c points along -x of the base</text>')

    # calibration marker
    mx, my = P(0.0, TABLE_Z)
    b.append(f'<rect x="{mx - 9:.1f}" y="{my - 5:.1f}" width="18" height="5" fill="#ffffff"/>')
    b.append(f'<text class="s" x="{mx - 120:.1f}" y="{my + 48:.1f}" font-size="12">calibration marker ID 0 at (0.30, 0.00, -0.075), 15 mm</text>')

    # example object and ray (animated)
    obj = EXAMPLE_OBJECTS["fork"]
    ox_, oy_ = P(obj[1], obj[2])
    b.append(f'<circle cx="{ox_:.1f}" cy="{oy_:.1f}" r="6" fill="{C["ctrl"]}"/>')
    b.append(f'<line x1="{cx_:.1f}" y1="{cy_:.1f}" x2="{ox_:.1f}" y2="{oy_:.1f}" stroke="{C["ctrl"]}" stroke-width="2" stroke-dasharray="6 6">'
             f'<animate attributeName="stroke-dashoffset" values="24;0" dur="1s" repeatCount="indefinite"/></line>')
    pc = base_to_camera(list(obj))
    b.append(f'<text class="m b" x="{ox_ + 10:.1f}" y="{oy_ + 18:.1f}" font-size="12" style="fill:{C["ctrl"]}">fork</text>')
    b.append(f'<text class="t b" x="60" y="{oy_ - 196:.1f}" font-size="13">example object (fork)</text>')
    b.append(f'<text class="m" x="60" y="{oy_ - 176:.1f}" font-size="12" style="fill:{C["ctrl"]}">p_cam  = ({pc[0]:.3f}, {pc[1]:.3f}, {pc[2]:.3f})</text>')
    b.append(f'<text class="m" x="60" y="{oy_ - 158:.1f}" font-size="12" style="fill:{C["ctrl"]}">p_base = ({obj[0]:.3f}, {obj[1]:.3f}, {obj[2]:.3f})</text>')

    # release zone marker
    rz = CONST["release"][0]
    rx_, ry_ = P(rz[1], rz[2])
    b.append(f'<circle cx="{rx_:.1f}" cy="{ry_:.1f}" r="8" fill="none" stroke="{C["motion"]}" stroke-width="2"/>')
    b.append(f'<text class="s" x="{rx_ - 30:.1f}" y="{ry_ - 16:.1f}" font-size="12">release pose (y = {rz[1]:.2f})</text>')

    # Equation panel
    px0 = 820
    tilt = math.degrees(math.acos(-R[2][2]))
    rows = [
        ("t", "Calibration chain (extrinsinc_calibration.py)", C["text"], 15, True),
        ("m", "T_base_cam = T_base_marker * inv(T_cam_marker)", C["tf"], 12.5, False),
        ("s", "T_cam_marker: cv2.solvePnP on the 4 marker corners", C["sub"], 12, False),
        ("s", "T_base_marker: measured, R = Rz(+90 deg), t = (0.30, 0, -0.075)", C["sub"], 12, False),
        ("t", "Published static transform", C["text"], 15, True),
        ("m", f"t = ({CAM_T[0]:.4f}, {CAM_T[1]:.4f}, {CAM_T[2]:.4f}) m", C["vision"], 12.5, False),
        ("m", "q = (-0.0077, 0.9606, -0.2777, 0.0022)", C["vision"], 12.5, False),
        ("s", f"optical axis tilted {tilt:.1f} deg from vertical, towards -y", C["sub"], 12, False),
        ("s", f"it meets the table at ({hit[0]:.2f}, {hit[1]:.2f})", C["sub"], 12, False),
        ("t", "Used at run time", C["text"], 15, True),
        ("m", "p_base = R(q) * p_cam + t", C["ctrl"], 12.5, False),
        ("m", "tf_buffer->transform(point, \"fr3_link0\")", C["sub"], 12, False),
    ]
    y = 112
    b.append(f'<rect class="card" x="{px0 - 16}" y="88" width="372" height="{len(rows) * 26 + 44}" rx="12"/>')
    for cls, text, color, size, bold in rows:
        if bold and y > 112:
            y += 8
        weight = " b" if bold else ""
        klass = {"t": "t", "m": "m", "s": "s"}[cls]
        b.append(f'<text class="{klass}{weight}" x="{px0}" y="{y}" font-size="{size}" style="fill:{color}">{esc(text)}</text>')
        y += 26
    return document(w, h, "\n".join(b), "Coordinate frames",
                    "Robot base frame, calibrated camera frame, calibration marker and an example object ray.")


# ---------------------------------------------------------------------------
# Figure: voice activity detection computed by hmi_pkg/nodes/vad.py
# ---------------------------------------------------------------------------

def synth_audio():
    """Synthetic 16 kHz signal: background noise, a 3-syllable command, a click, a 1-word command."""
    sr = 16000
    rng = random.Random(7)
    plan = [("noise", 1.10), ("speech", 0.28, 0.10), ("noise", 0.10), ("speech", 0.16, 0.055), ("noise", 0.12),
            ("speech", 0.34, 0.085), ("noise", 0.95), ("speech", 0.06, 0.25), ("noise", 0.95),
            ("speech", 0.48, 0.09), ("noise", 0.85)]
    samples = []
    t = 0.0
    for item in plan:
        kind, dur = item[0], item[1]
        n = int(dur * sr)
        for i in range(n):
            noise = rng.gauss(0, 0.006)
            if kind == "speech":
                amp = item[2] * math.sin(math.pi * i / n) ** 0.6 * math.sqrt(2)
                tt = (t * sr + i) / sr
                v = amp * (0.6 * math.sin(2 * math.pi * 180 * tt) + 0.3 * math.sin(2 * math.pi * 360 * tt) + 0.1 * math.sin(2 * math.pi * 720 * tt)) / 0.69
                v += noise
            else:
                v = noise
            samples.append(max(-32768, min(32767, int(v * 32768))))
        t += dur
    return samples, sr, plan


def vad():
    sys.path.insert(0, str(ROOT / "hmi_pkg" / "nodes"))
    from vad import VADProcessor  # the real implementation
    from audio_utils import rms_int16, ms_of_bytes
    from config import Config

    samples, sr, plan = synth_audio()
    chunk = 683  # 2048-sample ScriptProcessor buffer at 48 kHz, downsampled to 16 kHz
    proc = VADProcessor()
    rows = []
    seg_start = None
    segments = []
    for i in range(0, len(samples) - chunk + 1, chunk):
        part = samples[i:i + chunk]
        pcm = b"".join(v.to_bytes(2, "little", signed=True) for v in part)
        rms = rms_int16(pcm)
        t0 = i / sr
        if proc.state.calibrating:
            done = proc.process_calibration(pcm, sr)
            rows.append((t0, rms, "calib"))
            continue
        was = proc.state.in_voice
        seg = proc.process_audio(pcm, sr)
        if not was and proc.state.in_voice:
            seg_start = t0
        if seg is not None or (was and not proc.state.in_voice):
            state = "end"
            segments.append({"start": seg_start, "end": t0 + chunk / sr, "ms": ms_of_bytes(len(seg), sr) if seg else None,
                             "rms": rms_int16(seg) if seg else None})
        elif proc.state.in_voice:
            state = "voice" if rms >= proc.state.gate_rms else "hang"
        else:
            state = "idle"
        rows.append((t0, rms, state))
    noise, gate = proc.state.noise_rms, proc.state.gate_rms
    total_t = len(samples) / sr

    w, h = 1200, 492
    D_sweep, D = 9.0, 12.0
    x0, x1 = 60, 1150
    yb, ytop = 352, 150
    vmax = 0.12

    def X(t):
        return x0 + (x1 - x0) * t / total_t

    def Y(v):
        return yb - (yb - ytop) * min(v, vmax) / vmax

    b = []
    b.append('<text class="t b" x="40" y="46" font-size="22">Voice activity detection, computed by hmi_pkg/nodes/vad.py</text>')
    b.append(f'<text class="s" x="40" y="70" font-size="14">Synthetic input in 42 ms chunks (683 samples, as sent by the browser at 48 kHz). Every state below is the real VADProcessor output.</text>')
    b.append(f'<defs><clipPath id="sweep"><rect x="{x0}" y="90" width="0" height="320">'
             f'<animate attributeName="width" values="0;{x1 - x0};{x1 - x0}" keyTimes="0;{D_sweep / D:.4f};1" dur="{D}s" repeatCount="indefinite"/></rect></clipPath></defs>')
    b.append(f'<line x1="{x0}" y1="{yb}" x2="{x1}" y2="{yb}" stroke="{C["stroke"]}"/>')
    for tt in range(0, int(total_t) + 1):
        b.append(f'<line x1="{X(tt):.1f}" y1="{yb}" x2="{X(tt):.1f}" y2="{yb + 5}" stroke="{C["dim"]}"/>'
                 f'<text class="d" x="{X(tt):.1f}" y="{yb + 19}" font-size="11" text-anchor="middle">{tt} s</text>')
    calib_end = next(r[0] for r in rows if r[2] != "calib")
    g = []
    g.append(f'<rect x="{x0}" y="{ytop - 10}" width="{X(calib_end) - x0:.1f}" height="{yb - ytop + 10}" fill="{C["tf"]}" opacity="0.10"/>')
    g.append(f'<text class="s" x="{x0 + 6}" y="{ytop + 4}" font-size="12">calibration {Config.VAD_CALIB_SEC * 1000:.0f} ms</text>')
    bw = (x1 - x0) * (chunk / sr) / total_t
    colors = {"calib": C["tf"], "idle": "#3b4a66", "voice": C["voice"], "hang": "#6d5bb3", "end": C["ok"]}
    for t0, rms, state in rows:
        g.append(f'<rect x="{X(t0) + 0.6:.1f}" y="{Y(rms):.1f}" width="{bw - 1.2:.1f}" height="{yb - Y(rms):.1f}" fill="{colors[state]}" opacity="0.9"/>')
        g.append(f'<rect x="{X(t0) + 0.6:.1f}" y="{yb + 30}" width="{bw - 1.2:.1f}" height="12" fill="{colors[state]}"/>')
    for sgm in segments:
        g.append(f'<rect x="{X(sgm["start"]):.1f}" y="{ytop - 22}" width="{X(sgm["end"]) - X(sgm["start"]):.1f}" height="6" rx="3" fill="{C["ok"]}"/>')
    b.append(f'<g clip-path="url(#sweep)">{"".join(g)}</g>')
    b.append(f'<line x1="{X(calib_end):.1f}" y1="{Y(gate):.1f}" x2="{x1}" y2="{Y(gate):.1f}" stroke="{C["warn"]}" stroke-width="1.6" stroke-dasharray="6 4"/>')
    b.append(f'<text class="m" x="{X(calib_end) + 6:.1f}" y="{Y(gate) - 8:.1f}" font-size="11" style="fill:{C["warn"]}">gate</text>')
    b.append(f'<text class="m" x="{x1}" y="{yb + 68:.1f}" font-size="11.5" text-anchor="end" style="fill:{C["warn"]}">gate = max(0.01, 2.5 x {noise:.4f}, 0.0015) = {gate:.4f}</text>')
    b.append(f'<line x1="{x0}" y1="90" x2="{x0}" y2="{yb + 44}" stroke="#ffffff" stroke-width="1.5">'
             f'<animate attributeName="x1" values="{x0};{x1};{x1}" keyTimes="0;{D_sweep / D:.4f};1" dur="{D}s" repeatCount="indefinite"/>'
             f'<animate attributeName="x2" values="{x0};{x1};{x1}" keyTimes="0;{D_sweep / D:.4f};1" dur="{D}s" repeatCount="indefinite"/></line>')
    labels = ["segment 1", "segment 2 (click)", "segment 3"]
    for i, sgm in enumerate(segments):
        t_show = D_sweep * (sgm["end"] / total_t)
        txt = (f'<text class="m b" x="{X(sgm["start"]):.1f}" y="{ytop - 46}" font-size="11.5" style="fill:{C["ok"]}">'
               f'{labels[i] if i < len(labels) else "segment"}</text>'
               f'<text class="m" x="{X(sgm["start"]):.1f}" y="{ytop - 31}" font-size="11.5" style="fill:{C["ok"]}">'
               f'{sgm["ms"]} ms, RMS {sgm["rms"]:.3f}</text>')
        b.append(fade_group(txt, [(t_show, D)], D))
    # legend and notes
    lg = [("calibrating", C["tf"]), ("idle", "#3b4a66"), ("voice (RMS >= gate)", C["voice"]),
          ("hang-over (silence < 450 ms)", "#6d5bb3"), ("segment closed", C["ok"])]
    lx = 60
    for name, col in lg:
        b.append(f'<rect x="{lx}" y="{yb + 58}" width="12" height="12" rx="2" fill="{col}"/><text class="s" x="{lx + 18}" y="{yb + 68}" font-size="12">{esc(name)}</text>')
        lx += 30 + len(name) * 7
    b.append(f'<text class="s" x="60" y="{yb + 100}" font-size="12.5">A segment ends after {Config.VAD_SIL_HANG_MS} ms of silence, so it always contains that hang-over: the {Config.VAD_MIN_MS} ms minimum-length test never rejects anything with the default settings,</text>')
    b.append(f'<text class="s" x="60" y="{yb + 120}" font-size="12.5">and the 60 ms click becomes a {segments[1]["ms"] if len(segments) > 1 else "?"} ms segment that is uploaded to Whisper. Gate and noise are estimated once per server start.</text>')
    return document(w, h, "\n".join(b), "Voice activity detection",
                    "Chunk RMS, calibration, gate threshold and speech segments produced by the VAD of hmi_server.")


FIGURES_PART2 = {
    "perception.svg": perception,
    "frames.svg": frames,
    "vad.svg": vad,
}


# ---------------------------------------------------------------------------
# Figure: voice command to robot reaction (sequence diagram, animated)
# ---------------------------------------------------------------------------

def python_nlu():
    os.environ.setdefault("CAPABILITIES_PATH", str(ROOT / "hmi_pkg" / "config" / "capabilities.json"))
    sys.path.insert(0, str(ROOT / "hmi_pkg" / "nodes"))
    import logging
    logging.disable(logging.CRITICAL)
    from nlu_engine import load_capabilities
    return load_capabilities()


def ros_payload(action, obj, allow):
    return json.dumps({"action": action, "object": {"label": obj} if obj else None, "allow_execution": allow},
                      separators=(",", ":"))


def intent_flow():
    nlu = python_nlu()
    a_text, b_text = "Pick the bottle.", "Release."
    a = nlu.parse(a_text)
    bb = nlu.parse(b_text)
    w, h = 1200, 760
    lanes = {"op": 92, "page": 300, "srv": 530, "topic": 790, "ctl": 1065}
    names = {"op": "Operator", "page": "HMI web page", "srv": "hmi_server", "topic": "/hmi/intent_raw", "ctl": "move_to_pose_node"}
    colors = {"op": C["voice"], "page": C["voice"], "srv": C["voice"], "topic": C["voice"], "ctl": C["ctrl"]}
    rows = [
        ("head", "A.  \"pick the bottle\": parsed, locked, confirmed, and ignored by the controller"),
        ("msg", "op", "page", "speech", None),
        ("msg", "page", "srv", "WS /asr: PCM16 chunks, 16 kHz", None),
        ("note", "srv", f"VAD segment -> Whisper -> \"{a_text}\""),
        ("msg", "srv", "page", '{"type":"final","text":"' + a_text + '"}', None),
        ("note", "srv", f"NLU -> {json.dumps(a)}  complete -> lock"),
        ("msg", "srv", "topic", ros_payload(a["action"], a["object"], False), None),
        ("msg", "topic", "ctl", "intent_callback: no \"release\" substring -> ignored", C["dim"]),
        ("msg", "page", "srv", "POST /confirm  (operator presses Confirm)", None),
        ("msg", "srv", "topic", ros_payload(a["action"], a["object"], True), None),
        ("head", "B.  \"release\" while the arm waits in the release zone"),
        ("msg", "op", "page", "speech", None),
        ("note", "srv", f"Whisper -> \"{b_text}\" -> NLU {json.dumps(bb)} -> lock"),
        ("msg", "srv", "topic", ros_payload(bb["action"], bb["object"], False), None),
        ("msg", "topic", "ctl", "data contains \"release\" -> release_signal_received = true", C["ctrl"]),
        ("note", "ctl", "open_gripper(): width 0.08 m"),
        ("msg", "page", "srv", "POST /confirm  -> same intent again, allow_execution: true", None),
    ]
    step = 1.1
    D = len(rows) * step + 4.0
    b = []
    b.append('<text class="t b" x="40" y="46" font-size="22">From a spoken word to the gripper</text>')
    b.append('<text class="s" x="40" y="70" font-size="14">Messages exactly as produced by hmi_server and read by move_to_pose.cpp. The intent JSON is published before the operator confirms.</text>')
    b.append("<defs>" + arrow_marker("am-v", C["voice"]) + arrow_marker("am-c", C["ctrl"]) + arrow_marker("am-d", C["dim"]) + "</defs>")
    top, row_h = 128, 35
    bottom = top + 22 + len(rows) * row_h
    for k, x in lanes.items():
        b.append(f'<line x1="{x}" y1="{top - 6}" x2="{x}" y2="{bottom}" stroke="{C["stroke"]}" stroke-dasharray="4 6"/>')
        wdt = len(names[k]) * 8 + 26
        b.append(f'<rect x="{x - wdt / 2:.0f}" y="{top - 38}" width="{wdt:.0f}" height="30" rx="8" fill="{C["panel"]}" stroke="{colors[k]}" stroke-opacity="0.7"/>')
        cls = "m" if k == "topic" else "t"
        b.append(f'<text class="{cls} b" x="{x}" y="{top - 18}" font-size="13" text-anchor="middle">{esc(names[k])}</text>')
    for i, row in enumerate(rows):
        y = top + 26 + i * row_h
        t0 = 0.4 + i * step
        if row[0] == "head":
            g = (f'<rect x="30" y="{y - 18}" width="1140" height="26" rx="6" fill="{C["voice"]}" opacity="0.10"/>'
                 f'<text class="t b" x="44" y="{y}" font-size="13.5">{esc(row[1])}</text>')
        elif row[0] == "note":
            x = lanes[row[1]]
            text = row[2]
            wdt = len(text) * 6.9 + 20
            x_left = min(x - 10, w - 30 - wdt)
            g = (f'<rect x="{x_left:.0f}" y="{y - 16}" width="{wdt:.0f}" height="24" rx="6" fill="{C["panel2"]}" stroke="{C["stroke"]}"/>'
                 f'<text class="m" x="{x_left + 10:.0f}" y="{y}" font-size="11.5">{esc(text)}</text>')
        else:
            _, src, dst, text, color = row
            color = color or C["voice"]
            marker = "am-c" if color == C["ctrl"] else ("am-d" if color == C["dim"] else "am-v")
            x0, x1 = lanes[src], lanes[dst]
            sgn = 1 if x1 > x0 else -1
            length = abs(x1 - x0)
            label_x = min(x0, x1) + 10
            wdt = len(text) * 6.9 + 16
            if label_x + wdt > w - 20:
                label_x = w - 20 - wdt
            g = (f'<line x1="{x0}" y1="{y + 4}" x2="{x1 - sgn * 6}" y2="{y + 4}" stroke="{color}" stroke-width="2" marker-end="url(#{marker})"'
                 f' stroke-dasharray="{length}" stroke-dashoffset="{length}">'
                 f'<animate attributeName="stroke-dashoffset" values="{length};{length};0;0" keyTimes="0;{t0 / D:.4f};{(t0 + 0.45) / D:.4f};1" dur="{D:.2f}s" repeatCount="indefinite"/></line>'
                 f'<rect x="{label_x - 6:.0f}" y="{y - 14}" width="{wdt:.0f}" height="18" rx="5" fill="{C["bg"]}" opacity="0.85"/>'
                 f'<text class="m" x="{label_x:.0f}" y="{y - 1}" font-size="11.5" style="fill:{color if color != C["voice"] else C["text"]}">{esc(text)}</text>')
        b.append(fade_group(g, [(t0, D)], D))
    b.append(f'<text class="d" x="40" y="{bottom + 26}" font-size="12">The controller only tests msg.data.find("release"); confirmation, the "pick" action and the object field are not used by move_to_pose.cpp. '
             f'A release heard before the arm reaches the zone is cleared when the wait starts.</text>')
    return document(w, h + 0, "\n".join(b), "Voice command flow",
                    "Sequence of WebSocket, HTTP and ROS messages for a pick command and a release command.")


# ---------------------------------------------------------------------------
# Figure: the pick-and-handover cycle (top view + side view, animated)
# ---------------------------------------------------------------------------

def smoothstep(u):
    u = max(0.0, min(1.0, u))
    return u * u * (3 - 2 * u)


SH, LA, LB, LG = 0.333, 0.42, 0.42, 0.20  # schematic arm (not the FR3 kinematics)


def schematic_ik(r, z):
    wz = z + LG
    dr, dz = r, wz - SH
    d = max(abs(LA - LB) + 1e-6, min(LA + LB - 1e-6, math.hypot(dr, dz)))
    alpha = math.atan2(dr, dz)
    beta = math.acos(max(-1.0, min(1.0, (LA * LA + d * d - LB * LB) / (2 * LA * d))))
    phi1 = alpha - beta
    er, ez = LA * math.sin(phi1), SH + LA * math.cos(phi1)
    phi2 = math.atan2(r - er, wz - ez)
    return phi1, phi2


def schematic_fk(phi1, phi2):
    r = LA * math.sin(phi1) + LB * math.sin(phi2)
    wz = SH + LA * math.cos(phi1) + LB * math.cos(phi2)
    return r, wz - LG


def pick_cycle():
    inter = tuple(CONST["intermediate"][0][:3])
    release = tuple(CONST["release"][0][:3])
    ready = (0.307, 0.0, 0.487)  # approximate TCP position of the SRDF "ready" state
    tasks = [(item["tool"], item["startTime"]) for item in MISSION["sequence"]]
    grip_width = {"fork": 0.012, "spoon": 0.012, "bottle": 0.03, "toy": 0.03}

    phases = ["dispatch + memory lookup", "Cartesian move: intermediate", "sleep 10 s (fixed)",
              "Cartesian move: pre-grasp (+10.3 cm)", "Cartesian move: pick", "grasp 40 N, then 1 s",
              "Cartesian move: retract", "Cartesian move: intermediate", "Cartesian move: release zone",
              "wait for \"release\" on /hmi/intent_raw", "open gripper to 8 cm, then 1 s",
              "going home: named target \"ready\""]
    segs = []
    state = {"t": 0.0, "p": ready, "grip": 0.08}

    def add(kind, dur, phase, log, p1=None, grip1=None, carry=None, task=None, extra=None):
        seg = {"kind": kind, "t0": state["t"], "t1": state["t"] + dur, "p0": state["p"], "p1": p1 or state["p"],
               "g0": state["grip"], "g1": state["grip"] if grip1 is None else grip1, "phase": phase,
               "log": log, "carry": carry, "task": task, "extra": extra}
        segs.append(seg)
        state["t"] = seg["t1"]
        state["p"] = seg["p1"]
        state["grip"] = seg["g1"]

    def move(dst, phase, log, carry=None, task=None):
        d = math.dist(state["p"], dst)
        add("line", max(0.55, d / 0.30), phase, log, p1=dst, carry=carry, task=task)

    add("hold", 1.0, None, f">>> PROJECT STARTED: {len(tasks)} Tasks Loaded <<<")
    for ti, (tool, start) in enumerate(tasks):
        obj = EXAMPLE_OBJECTS[tool]
        pick, pre = pick_from_object(*obj)
        add("hold", 0.9, 0, f"--- NEXT TASK: Pick {tool} at T={start}s ---", task=ti)
        move(inter, 1, "PLANNING: Intermediate...", task=ti)
        add("hold", 1.4, 2, f"Moving to {tool}: ({pick[0]:.3f}, {pick[1]:.3f}, {pick[2]:.3f}, 0.71, 0.71, 0.00)", task=ti)
        move(pre, 3, "PLANNING: Pre-Grasp...", task=ti)
        move(pick, 4, "PLANNING: Pick...", task=ti)
        add("hold", 0.9, 5, "ACTION: Grasping (Waiting)...", grip1=grip_width[tool], task=ti)
        add("hold", 0.5, 5, "Grasp Successful.", carry=ti, task=ti)
        move(pre, 6, "PLANNING: Retract...", carry=ti, task=ti)
        move(inter, 7, "PLANNING: Intermediate...", carry=ti, task=ti)
        move(release, 8, "PLANNING: Release Zone...", carry=ti, task=ti)
        add("hold", 1.1, 9, "WAITING FOR RELEASE...", carry=ti, task=ti)
        add("hold", 0.7, 9, "[HMI] RELEASE SIGNAL RECEIVED!", carry=ti, task=ti, extra="voice")
        add("hold", 0.7, 10, "ACTION: Opening Gripper (Waiting for completion)...", grip1=0.08, carry=ti, task=ti)
        add("hold", 0.5, 10, "Gripper Opened Successfully.", task=ti)
    add("hold", 0.8, 11, "Project Complete. Going Home.")
    add("joint", 2.0, 11, "Project Complete. Going Home.", p1=ready)
    add("hold", 1.6, None, "Project Complete. Going Home.")
    D = state["t"]

    def sample(t):
        for s_ in segs:
            if s_["t0"] <= t <= s_["t1"] + 1e-9:
                break
        u = (t - s_["t0"]) / max(1e-9, s_["t1"] - s_["t0"])
        if s_["kind"] == "line":
            e = smoothstep(u)
            p = tuple(s_["p0"][i] + (s_["p1"][i] - s_["p0"][i]) * e for i in range(3))
            angles = None
        elif s_["kind"] == "joint":
            e = smoothstep(u)
            th0, th1 = math.atan2(s_["p0"][1], s_["p0"][0]), math.atan2(s_["p1"][1], s_["p1"][0])
            a0 = schematic_ik(math.hypot(*s_["p0"][:2]), s_["p0"][2])
            a1 = schematic_ik(math.hypot(*s_["p1"][:2]), s_["p1"][2])
            th = th0 + (th1 - th0) * e
            ph1 = a0[0] + (a1[0] - a0[0]) * e
            ph2 = a0[1] + (a1[1] - a0[1]) * e
            r, z = schematic_fk(ph1, ph2)
            p = (r * math.cos(th), r * math.sin(th), z)
            angles = (ph1, ph2)
        else:
            p = s_["p0"]
            angles = None
        grip = s_["g0"] + (s_["g1"] - s_["g0"]) * u
        if angles is None:
            angles = schematic_ik(math.hypot(p[0], p[1]), p[2])
        return p, grip, angles

    # Layout
    w, h = 1200, 640
    tx0, ty0, st = 250, 520, 420.0      # top view: base position on screen, px per metre

    def TV(x, y):
        return tx0 - y * st, ty0 - x * st

    sx0, sy0, ss = 600, 470, 360.0      # side view: r = 0, z = 0

    def SV(r, z):
        return sx0 + r * ss, sy0 - z * ss

    N = int(round(D / 0.05)) + 1
    times = [k * D / (N - 1) for k in range(N)]
    samples = [sample(t) for t in times]

    b = []
    b.append('<text class="t b" x="40" y="44" font-size="22">One mission, three handovers (shared/sequence.json)</text>')
    b.append('<text class="s" x="40" y="68" font-size="14">Waypoints, offsets and log lines are taken from move_to_pose.cpp. Object positions are an example; time is compressed.</text>')

    # --- top view panel
    b.append(f'<rect class="card" x="24" y="84" width="512" height="500" rx="14"/>')
    b.append(f'<text class="t b" x="40" y="108" font-size="14">Top view of fr3_link0</text>')
    b.append(f'<text class="d" x="40" y="126" font-size="12">x forward (up), y to the left; straight lines = Cartesian paths</text>')
    # camera footprint
    foot = []
    for u, v in [(0, 0), (K["w"], 0), (K["w"], K["h"]), (0, K["h"])]:
        dc = [(u - K["cx"]) / K["fx"], (v - K["cy"]) / K["fy"], 1.0]
        db = [sum(CAM_R[r_][c] * dc[c] for c in range(3)) for r_ in range(3)]
        s_ = (TABLE_Z - CAM_T[2]) / db[2]
        foot.append(TV(CAM_T[0] + s_ * db[0], CAM_T[1] + s_ * db[1]))
    b.append(f'<clipPath id="tvclip"><rect x="24" y="84" width="512" height="500" rx="14"/></clipPath>')
    b.append(f'<g clip-path="url(#tvclip)"><polygon points="{" ".join(f"{x:.1f},{y:.1f}" for x, y in foot)}" fill="{C["vision"]}" opacity="0.045" stroke="{C["vision"]}" stroke-opacity="0.25" stroke-dasharray="4 4"/></g>')
    b.append(f'<text class="d" x="40" y="570" font-size="11">shaded: camera field of view on the table (example intrinsics)</text>')
    cxs, cys = TV(CAM_T[0], CAM_T[1])
    b.append(f'<rect x="{cxs - 16:.1f}" y="{cys - 8:.1f}" width="32" height="16" rx="4" fill="#1b2742" stroke="{C["vision"]}"/>'
             f'<text class="s" x="{cxs + 22:.1f}" y="{cys + 4:.1f}" font-size="11.5">camera</text>')
    # axes
    b.append(f'<line x1="{tx0}" y1="{ty0}" x2="{tx0}" y2="{ty0 - 50}" stroke="#f87171" stroke-width="2.5"/><text class="m" x="{tx0 + 6}" y="{ty0 - 42}" font-size="12" style="fill:#f87171">x</text>')
    b.append(f'<line x1="{tx0}" y1="{ty0}" x2="{tx0 - 50}" y2="{ty0}" stroke="#4ade80" stroke-width="2.5"/><text class="m" x="{tx0 - 60}" y="{ty0 + 16}" font-size="12" style="fill:#4ade80">y</text>')
    b.append(f'<circle cx="{tx0}" cy="{ty0}" r="22" fill="#1f2a44" stroke="{C["stroke"]}"/><text class="d" x="{tx0}" y="{ty0 + 40}" font-size="11.5" text-anchor="middle">base</text>')
    # release zones and intermediate
    rx_, ry_ = TV(release[0], release[1])
    b.append(f'<circle cx="{rx_:.1f}" cy="{ry_:.1f}" r="{0.05 * st:.1f}" fill="{C["motion"]}" opacity="0.12" stroke="{C["motion"]}" stroke-dasharray="4 3"/>'
             f'<text class="s" x="{rx_ - 196:.1f}" y="{ry_ - 30:.1f}" font-size="11.5">release pose ({release[0]:.3f}, {release[1]:.2f}, {release[2]:.2f})</text>')
    lx_, ly_ = TV(0.219, -0.455)
    b.append(f'<circle cx="{lx_:.1f}" cy="{ly_:.1f}" r="{0.035 * st:.1f}" fill="none" stroke="{C["dim"]}" stroke-dasharray="3 3"/>'
             f'<text class="d" x="{lx_ - 150:.1f}" y="{ly_ + 30:.1f}" font-size="11">left-hand pose (commented out)</text>')
    ix_, iy_ = TV(inter[0], inter[1])
    b.append(f'<path d="M{ix_ - 6:.1f} {iy_ - 6:.1f}L{ix_ + 6:.1f} {iy_ + 6:.1f}M{ix_ + 6:.1f} {iy_ - 6:.1f}L{ix_ - 6:.1f} {iy_ + 6:.1f}" stroke="{C["ctrl"]}" stroke-width="2"/>'
             f'<text class="s" x="{ix_ + 10:.1f}" y="{iy_ + 4:.1f}" font-size="11.5">intermediate</text>')
    # objects (static until grasped)
    ocolors = {"fork": "#cbd5e1", "spoon": "#e2e8f0", "bottle": "#7dd3fc", "toy": "#fda4af"}
    grasp_time = {}
    release_time = {}
    for s_ in segs:
        if s_["log"] == "Grasp Successful." and s_["task"] not in grasp_time:
            grasp_time[s_["task"]] = s_["t0"]
        if s_["log"] == "Gripper Opened Successfully.":
            release_time[s_["task"]] = s_["t0"]

    def obj_icon(x, y, tool):
        return (f'<rect x="{x - 15:.1f}" y="{y - 7:.1f}" width="30" height="14" rx="5" fill="{ocolors[tool]}"/>'
                f'<text class="m" x="{x:.1f}" y="{y + 4:.1f}" font-size="9.5" text-anchor="middle" style="fill:#0b1220">{tool}</text>')

    for ti, (tool, _) in enumerate(tasks):
        ox_, oy_ = TV(*EXAMPLE_OBJECTS[tool][:2])
        b.append(fade_group(obj_icon(ox_, oy_, tool), [(0, grasp_time[ti])], D))
    # arm projection line
    xs = ";".join(f"{TV(p[0], p[1])[0]:.1f}" for p, _, _ in samples)
    ys = ";".join(f"{TV(p[0], p[1])[1]:.1f}" for p, _, _ in samples)
    b.append(f'<line x1="{tx0}" y1="{ty0}" x2="{tx0}" y2="{ty0}" stroke="#e8edf5" stroke-opacity="0.35" stroke-width="10" stroke-linecap="round">'
             f'<animate attributeName="x2" values="{xs}" dur="{D:.3f}s" repeatCount="indefinite"/>'
             f'<animate attributeName="y2" values="{ys}" dur="{D:.3f}s" repeatCount="indefinite"/></line>')
    # TCP marker carrying objects
    tr = ";".join(f"{TV(p[0], p[1])[0]:.1f},{TV(p[0], p[1])[1]:.1f}" for p, _, _ in samples)
    carried = "".join(fade_group(f'<g transform="translate(0 18)">{obj_icon(0, 0, tool)}</g>', [(grasp_time[ti], release_time[ti])], D)
                      for ti, (tool, _) in enumerate(tasks))
    b.append(f'<g><animateTransform attributeName="transform" type="translate" values="{tr}" dur="{D:.3f}s" repeatCount="indefinite"/>'
             f'{carried}<circle r="9" fill="none" stroke="{C["motion"]}" stroke-width="2.5"/><circle r="3" fill="{C["motion"]}"/></g>')
    # voice bubble
    voice = [(s_["t0"], s_["t1"]) for s_ in segs if s_["extra"] == "voice"]
    b.append(fade_group(f'<rect x="{rx_ - 120:.1f}" y="{ry_ + 26:.1f}" width="110" height="26" rx="13" fill="{C["voice"]}" opacity="0.9"/>'
                        f'<text class="m b" x="{rx_ - 65:.1f}" y="{ry_ + 44:.1f}" font-size="13" text-anchor="middle" style="fill:#0b1220">"release"</text>', voice, D))

    # --- side view panel
    b.append(f'<rect class="card" x="548" y="84" width="358" height="500" rx="14"/>')
    b.append(f'<text class="t b" x="564" y="108" font-size="14">Side view (arm plane)</text>')
    b.append(f'<text class="d" x="564" y="126" font-size="12">schematic links; TCP height is exact</text>')
    tz = SV(0, TABLE_Z)[1]
    b.append(f'<rect x="556" y="{tz:.1f}" width="342" height="10" fill="#1a2438"/>')
    b.append(f'<text class="d" x="{SV(0.47, 0)[0]:.0f}" y="{tz + 26:.1f}" font-size="11">table z = -0.075</text>')
    bx_, by_ = SV(0, 0)
    b.append(f'<rect x="{bx_ - 18:.1f}" y="{SV(0, SH)[1]:.1f}" width="36" height="{tz - SV(0, SH)[1]:.1f}" rx="10" fill="#1f2a44" stroke="{C["stroke"]}"/>')
    # target objects in side view and z annotations
    for ti, (tool, _) in enumerate(tasks):
        obj = EXAMPLE_OBJECTS[tool]
        r_obj = math.hypot(obj[0], obj[1])
        pick, pre = pick_from_object(*obj)
        r_pick = math.hypot(pick[0], pick[1])
        ox_, oy_ = SV(r_obj, obj[2])
        _, pz_ = SV(r_pick, pick[2])
        _, prz_ = SV(r_pick, pre[2])
        task_iv = [(s_["t0"], s_["t1"]) for s_ in segs if s_["task"] == ti]
        t_start, t_end = task_iv[0][0], task_iv[-1][1]
        height = max(10.0, (obj[2] - TABLE_Z) * ss)
        content = (f'<rect x="{ox_ - 10:.1f}" y="{oy_:.1f}" width="20" height="{height:.1f}" rx="4" fill="{ocolors[tool]}" opacity="0.9"/>')
        b.append(fade_group(content, [(t_start, grasp_time[ti])], D))
        ann = (f'<line x1="{SV(r_pick, 0)[0] + 34:.1f}" y1="{prz_:.1f}" x2="{SV(r_pick, 0)[0] + 34:.1f}" y2="{pz_:.1f}" stroke="{C["ctrl"]}" stroke-width="1.5"/>'
               f'<text class="m" x="{SV(r_pick, 0)[0] + 40:.1f}" y="{(prz_ + pz_) / 2 + 4:.1f}" font-size="10.5" style="fill:{C["ctrl"]}">+0.103</text>'
               f'<text class="m" x="{SV(r_pick, 0)[0] + 40:.1f}" y="{pz_ + 14:.1f}" font-size="10.5" style="fill:{C["ctrl"]}">z={pick[2]:.3f}</text>')
        b.append(fade_group(ann, [(t_start, t_end)], D))
    r1 = ";".join(f"{math.degrees(a[0]):.1f}" for _, _, a in samples)
    r2 = ";".join(f"{math.degrees(a[1] - a[0]):.1f}" for _, _, a in samples)
    r3 = ";".join(f"{180 - math.degrees(a[1]):.1f}" for _, _, a in samples)
    fl = ";".join(f"{-g / 2 * ss * 1.6 - 4:.1f},0" for _, g, _ in samples)
    fr = ";".join(f"{g / 2 * ss * 1.6:.1f},0" for _, g, _ in samples)
    carried_side = "".join(fade_group(f'<rect x="-7" y="{-LG * ss - 26:.1f}" width="14" height="22" rx="3" fill="{ocolors[tool]}"/>',
                                      [(grasp_time[ti], release_time[ti])], D) for ti, (tool, _) in enumerate(tasks))
    sx_, sy_ = SV(0, SH)
    la, lb, lg = LA * ss, LB * ss, LG * ss
    b.append(f"""<g transform="translate({sx_:.1f} {sy_:.1f})"><g>
<animateTransform attributeName="transform" type="rotate" values="{r1}" dur="{D:.3f}s" repeatCount="indefinite"/>
<rect x="-9" y="{-la:.1f}" width="18" height="{la + 4:.1f}" rx="9" fill="#e8edf5"/>
<circle r="12" fill="#cfd8e6" stroke="#9aa8bf"/>
<g transform="translate(0 {-la:.1f})"><g>
<animateTransform attributeName="transform" type="rotate" values="{r2}" dur="{D:.3f}s" repeatCount="indefinite"/>
<rect x="-8" y="{-lb:.1f}" width="16" height="{lb + 4:.1f}" rx="8" fill="#e8edf5"/>
<circle r="10" fill="#cfd8e6" stroke="#9aa8bf"/>
<g transform="translate(0 {-lb:.1f})"><g>
<animateTransform attributeName="transform" type="rotate" values="{r3}" dur="{D:.3f}s" repeatCount="indefinite"/>
<circle r="8" fill="#cfd8e6" stroke="#9aa8bf"/>
<rect x="-6" y="{-lg + 18:.1f}" width="12" height="{lg - 18:.1f}" rx="4" fill="#2b3550"/>
<rect x="-20" y="{-lg + 14:.1f}" width="40" height="7" rx="3" fill="#cfd8e6"/>
{carried_side}
<g><animateTransform attributeName="transform" type="translate" values="{fl}" dur="{D:.3f}s" repeatCount="indefinite"/><rect x="0" y="{-lg - 4:.1f}" width="4" height="19" rx="1.5" fill="#e8edf5"/></g>
<g><animateTransform attributeName="transform" type="translate" values="{fr}" dur="{D:.3f}s" repeatCount="indefinite"/><rect x="0" y="{-lg - 4:.1f}" width="4" height="19" rx="1.5" fill="#e8edf5"/></g>
<circle cy="{-lg:.1f}" r="2.5" fill="{C['motion']}"/>
</g></g></g></g></g></g>""")

    # --- phase list
    px0 = 918
    b.append(f'<rect class="card" x="{px0}" y="84" width="258" height="500" rx="14"/>')
    b.append(f'<text class="t b" x="{px0 + 14}" y="108" font-size="14">Controller loop</text>')
    for ti, (tool, start) in enumerate(tasks):
        iv = [(s_["t0"], s_["t1"]) for s_ in segs if s_["task"] == ti]
        txt = f'<text class="m" x="{px0 + 14}" y="128" font-size="11.5" style="fill:{C["ctrl"]}">task {ti + 1}/{len(tasks)}: {tool}, startTime {start} s</text>'
        b.append(fade_group(txt, [(iv[0][0], iv[-1][1])], D))
    for pi, name in enumerate(phases):
        y = 144 + pi * 36
        iv = []
        for s_ in segs:
            if s_["phase"] == pi:
                if iv and abs(iv[-1][1] - s_["t0"]) < 1e-9:
                    iv[-1] = (iv[-1][0], s_["t1"])
                else:
                    iv.append((s_["t0"], s_["t1"]))
        b.append(f'<rect x="{px0 + 8}" y="{y}" width="242" height="30" rx="7" fill="{C["motion"]}" opacity="0">{visible(iv, D, on="0.2")}</rect>')
        b.append(f'<circle cx="{px0 + 22}" cy="{y + 15}" r="4" fill="{C["dim"]}"><animate attributeName="fill" values="{C["dim"]};{C["motion"]}" dur="1s" repeatCount="1"/></circle>')
        b.append(f'<text class="t" x="{px0 + 34}" y="{y + 19}" font-size="11.8">{esc(name)}</text>')

    # --- log line
    b.append(f'<rect x="24" y="594" width="1152" height="32" rx="8" fill="#060b16" stroke="{C["stroke"]}"/>')
    b.append(f'<text class="ms" x="38" y="615" font-size="12">[move_to_pose_node]</text>')
    merged = []
    for s_ in segs:
        if merged and merged[-1][0] == s_["log"] and abs(merged[-1][2] - s_["t0"]) < 1e-9:
            merged[-1] = (merged[-1][0], merged[-1][1], s_["t1"])
        else:
            merged.append((s_["log"], s_["t0"], s_["t1"]))
    for text, a, bnd in merged:
        color = C["warn"] if text.startswith("WAITING") else (C["voice"] if text.startswith("[HMI]") else C["text"])
        b.append(fade_group(f'<text class="m" x="196" y="615" font-size="12.5" style="fill:{color}">{esc(text)}</text>', [(a, bnd)], D))
    return document(w, h, "\n".join(b), "Pick and handover cycle",
                    "Animated top and side views of the controller executing the default mission.")


# ---------------------------------------------------------------------------
# Figure: grasp tolerance
# ---------------------------------------------------------------------------

def grasp():
    g = CONST["grasp"]
    lo, hi = g["width"] - g["epsilonInner"], g["width"] + g["epsilonOuter"]
    cases = [("object 25 mm", 0.025), ("object 65 mm", 0.065), ("no object", 0.0)]
    per = 4.5
    D = per * len(cases)
    w, h = 1200, 400
    b = []
    b.append('<text class="t b" x="40" y="46" font-size="22">Grasp goal and its tolerance</text>')
    b.append(f'<text class="s" x="40" y="70" font-size="14">grasp_object(): width {g["width"]} m, epsilon inner/outer {g["epsilonInner"]} m, force {g["force"]:.0f} N, speed {g["speed"]} m/s. '
             f'Success means {lo:.2f} m &lt; d &lt; {hi:.2f} m.</text>')
    # gripper drawing
    gx, gy, sc = 300, 150, 2600.0
    b.append(f'<rect x="{gx - 160}" y="{gy - 40}" width="320" height="30" rx="8" fill="#cfd8e6"/>')
    b.append(f'<rect x="{gx - 30}" y="{gy - 80}" width="60" height="42" rx="8" fill="#2b3550"/>')
    values_l, values_r, kt = [], [], []
    for i, (_, width) in enumerate(cases):
        t0 = i * per
        for t, d in ((t0, 0.08), (t0 + 0.8, 0.08), (t0 + 2.0, width), (t0 + 3.6, width), (t0 + per - 0.01, 0.08)):
            kt.append(t / D)
            values_l.append(f"{-d / 2 * sc:.1f},0")
            values_r.append(f"{d / 2 * sc:.1f},0")
    kt[-1] = 1.0
    kts = ";".join(f"{k:.4f}" for k in kt)
    for vals, side in ((values_l, -1), (values_r, 1)):
        x = -16 if side < 0 else 0
        b.append(f'<g transform="translate({gx} {gy})"><g><animateTransform attributeName="transform" type="translate" values="{";".join(vals)}" keyTimes="{kts}" dur="{D}s" repeatCount="indefinite"/>'
                 f'<rect x="{x}" y="-12" width="16" height="96" rx="4" fill="#e8edf5"/></g></g>')
    for i, (name, width) in enumerate(cases):
        iv = [(i * per, (i + 1) * per)]
        content = ""
        if width > 0:
            content += f'<rect x="{gx - width / 2 * sc:.1f}" y="{gy + 20}" width="{width * sc:.1f}" height="110" rx="6" fill="{C["ctrl"]}" opacity="0.85"/>'
        content += f'<text class="t b" x="{gx}" y="{gy + 170}" font-size="15" text-anchor="middle">{esc(name)}</text>'
        ok = lo < width < hi
        verdict = f"fingers stop at d = {width:.3f} m"
        content += f'<text class="m" x="{gx}" y="{gy + 194}" font-size="12.5" text-anchor="middle">{verdict}</text>'
        outcome = 'SUCCEEDED: "Grasp Successful."' if ok else 'ABORTED: "Grasp Failed."'
        res = fade_group(f'<text class="m b" x="{gx}" y="{gy + 218}" font-size="13" text-anchor="middle" style="fill:{C["ok"] if ok else C["warn"]}">'
                         f'{esc(outcome)}</text>', [(i * per + 2.1, (i + 1) * per)], D)
        b.append(fade_group(content + res, iv, D))
    # number line
    nx0, nx1, ny = 640, 1150, 210
    vmin, vmax = -0.02, 0.10

    def NX(v):
        return nx0 + (nx1 - nx0) * (v - vmin) / (vmax - vmin)

    b.append(f'<rect x="{NX(lo):.1f}" y="{ny - 26}" width="{NX(hi) - NX(lo):.1f}" height="52" rx="8" fill="{C["ok"]}" opacity="0.13"/>')
    b.append(f'<text class="m" x="{NX(lo):.1f}" y="{ny - 34}" font-size="11.5" style="fill:{C["ok"]}">reported as success: ({lo:.2f}, {hi:.2f})</text>')
    b.append(f'<rect x="{NX(0):.1f}" y="{ny - 4}" width="{NX(0.08) - NX(0):.1f}" height="8" rx="4" fill="#cfd8e6" opacity="0.5"/>')
    b.append(f'<text class="s" x="{NX(0):.1f}" y="{ny + 44}" font-size="11.5">finger travel 0 ... 0.08 m (open_gripper width)</text>')
    b.append(f'<line x1="{nx0}" y1="{ny}" x2="{nx1}" y2="{ny}" stroke="{C["dim"]}"/>')
    for v in [i / 100 for i in range(-2, 11, 2)]:
        b.append(f'<line x1="{NX(v):.1f}" y1="{ny - 6}" x2="{NX(v):.1f}" y2="{ny + 6}" stroke="{C["dim"]}"/>'
                 f'<text class="d" x="{NX(v):.1f}" y="{ny + 22}" font-size="11" text-anchor="middle">{v:.2f}</text>')
    b.append(f'<line x1="{NX(g["width"]):.1f}" y1="{ny - 30}" x2="{NX(g["width"]):.1f}" y2="{ny + 30}" stroke="{C["ctrl"]}" stroke-width="2"/>'
             f'<text class="m" x="{NX(g["width"]):.1f}" y="{ny - 50}" font-size="11.5" text-anchor="middle" style="fill:{C["ctrl"]}">goal width {g["width"]}</text>')
    mvals, mkt = [], []
    for i, (_, width) in enumerate(cases):
        t0 = i * per
        for t, d in ((t0, 0.08), (t0 + 0.8, 0.08), (t0 + 2.0, width), (t0 + 3.6, width), (t0 + per - 0.01, 0.08)):
            mkt.append(t / D)
            mvals.append(f"{NX(d):.1f},{ny}")
    mkt[-1] = 1.0
    b.append(f'<g><animateTransform attributeName="transform" type="translate" values="{";".join(mvals)}" keyTimes="{";".join(f"{k:.4f}" for k in mkt)}" dur="{D}s" repeatCount="indefinite"/>'
             f'<path d="M0 -12L7 -24L-7 -24Z" fill="#ffffff"/><text class="m" x="0" y="-30" font-size="11" text-anchor="middle">d</text></g>')
    b.append(f'<text class="s" x="640" y="{ny + 84}" font-size="13">Because {g["width"]} - {g["epsilonInner"]} &lt; 0, fingers that close on nothing also pass the test.</text>')
    b.append(f'<text class="s" x="640" y="{ny + 104}" font-size="13">The action then succeeds and the arm carries an empty gripper to the</text>')
    b.append(f'<text class="s" x="640" y="{ny + 124}" font-size="13">release zone, where it waits for "release" as usual.</text>')
    b.append(f'<text class="d" x="640" y="{ny + 148}" font-size="12">Rule from libfranka Gripper::grasp(): width - epsilon_inner &lt; d &lt; width + epsilon_outer.</text>')
    return document(w, h, "\n".join(b), "Grasp tolerance",
                    "Animated gripper closing on objects of different widths and the tolerance band of the grasp goal.")


# ---------------------------------------------------------------------------
# Figure: mission scheduling
# ---------------------------------------------------------------------------

def mission_timeline():
    tasks = [(item["tool"], item["startTime"]) for item in MISSION["sequence"]]
    offset = CONST["timing"][0]
    cycle = 32.0  # example duration of one pick-carry-handover cycle
    starts, prev_end = [], 0.0
    for tool, st in tasks:
        go = max(st - offset, prev_end)
        starts.append((tool, st, st - offset, go, go + cycle))
        prev_end = go + cycle
    t_end = prev_end + 8
    w, h = 1200, 400
    x0, x1 = 170, 1150
    D = 12.0

    def X(t):
        return x0 + (x1 - x0) * (t + 6) / (t_end + 6)

    b = []
    b.append('<text class="t b" x="40" y="46" font-size="22">Scheduling: startTime is a lower bound, not a deadline</text>')
    b.append(f'<text class="s" x="40" y="70" font-size="14">Default shared/sequence.json. A task may start once elapsed ≥ startTime − {offset:.0f} s, and only after the previous task has finished.</text>')
    ytop = 110
    b.append(f'<line x1="{x0}" y1="{ytop + 3 * 70 + 10}" x2="{x1}" y2="{ytop + 3 * 70 + 10}" stroke="{C["dim"]}"/>')
    t = 0
    while t <= t_end:
        b.append(f'<line x1="{X(t):.1f}" y1="{ytop + 220}" x2="{X(t):.1f}" y2="{ytop + 226}" stroke="{C["dim"]}"/><text class="d" x="{X(t):.1f}" y="{ytop + 242}" font-size="11" text-anchor="middle">{t:.0f} s</text>')
        t += 10
    b.append(f'<defs><clipPath id="run"><rect x="{x0}" y="{ytop - 10}" width="0" height="230">'
             f'<animate attributeName="width" values="0;{x1 - x0};{x1 - x0}" keyTimes="0;0.8;1" dur="{D}s" repeatCount="indefinite"/></rect></clipPath></defs>')
    bars = []
    for i, (tool, st, thr, go, end) in enumerate(starts):
        y = ytop + i * 70
        b.append(f'<text class="t b" x="40" y="{y + 26}" font-size="14">{tool}</text><text class="ms" x="40" y="{y + 44}" font-size="11">startTime {st}</text>')
        b.append(f'<line x1="{x0}" y1="{y + 22}" x2="{x1}" y2="{y + 22}" stroke="{C["stroke"]}" stroke-dasharray="3 5"/>')
        b.append(f'<path d="M{X(st):.1f} {y + 10}l8 12l-8 12l-8 -12z" fill="{C["mission"]}"/>')
        b.append(f'<line x1="{X(thr):.1f}" y1="{y + 4}" x2="{X(thr):.1f}" y2="{y + 40}" stroke="{C["mission"]}" stroke-dasharray="3 3"/>')
        bars.append(f'<rect x="{X(go):.1f}" y="{y + 12}" width="{X(end) - X(go):.1f}" height="20" rx="6" fill="{C["ctrl"]}" opacity="0.85"/>'
                    f'<text class="m" x="{X(go) + 8:.1f}" y="{y + 26}" font-size="11" style="fill:#0b1220">starts at {go:.0f} s</text>')
        if i > 0 and go > thr + 0.01:
            bars.append(f'<line x1="{X(thr):.1f}" y1="{y + 46}" x2="{X(go):.1f}" y2="{y + 46}" stroke="{C["warn"]}" stroke-width="1.5"/>'
                        f'<text class="m" x="{(X(thr) + X(go)) / 2:.1f}" y="{y + 60}" font-size="11" text-anchor="middle" style="fill:{C["warn"]}">waits for the previous task: {go - thr:.0f} s later than allowed</text>')
    b.append(f'<g clip-path="url(#run)">{"".join(bars)}</g>')
    b.append(f'<line x1="{x0}" y1="{ytop - 6}" x2="{x0}" y2="{ytop + 214}" stroke="#ffffff" stroke-width="1.5">'
             f'<animate attributeName="x1" values="{x0};{x1};{x1}" keyTimes="0;0.8;1" dur="{D}s" repeatCount="indefinite"/>'
             f'<animate attributeName="x2" values="{x0};{x1};{x1}" keyTimes="0;0.8;1" dur="{D}s" repeatCount="indefinite"/></line>')
    lg = [("requested startTime", C["mission"], "diamond"), (f"earliest dispatch (startTime − {offset:.0f} s)", C["mission"], "dash"),
          (f"pick-carry-handover cycle (example: {cycle:.0f} s)", C["ctrl"], "bar")]
    lx = 170
    for name, col, kind in lg:
        if kind == "diamond":
            b.append(f'<path d="M{lx + 6} {ytop + 262}l6 8l-6 8l-6 -8z" fill="{col}"/>')
        elif kind == "dash":
            b.append(f'<line x1="{lx + 6}" y1="{ytop + 262}" x2="{lx + 6}" y2="{ytop + 278}" stroke="{col}" stroke-dasharray="3 3"/>')
        else:
            b.append(f'<rect x="{lx}" y="{ytop + 264}" width="16" height="12" rx="3" fill="{col}"/>')
        b.append(f'<text class="s" x="{lx + 22}" y="{ytop + 275}" font-size="12">{esc(name)}</text>')
        lx += 40 + len(name) * 7
    b.append(f'<text class="d" x="40" y="{ytop + 270}" font-size="12">legend</text>')
    return document(w, h, "\n".join(b), "Mission scheduling",
                    "Timeline of the default mission: requested times, dispatch threshold and sequential execution.")


FIGURES_PART3 = {
    "intent_flow.svg": intent_flow,
    "pick_cycle.svg": pick_cycle,
    "grasp.svg": grasp,
    "mission_timeline.svg": mission_timeline,
}


FIGURES = {
    "hero.svg": hero,
    "architecture.svg": architecture,
    **FIGURES_PART2,
    **FIGURES_PART3,
}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    selected = sys.argv[1:] or list(FIGURES)
    for name in selected:
        svg = FIGURES[name]()
        (OUT / name).write_text(svg)
        print(f"wrote {OUT.relative_to(ROOT) / name} ({len(svg) / 1024:.1f} KiB)")


if __name__ == "__main__":
    main()
