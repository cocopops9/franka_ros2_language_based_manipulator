// Lab: clickable graph of processes, topics and files, with the facts read from the sources.

import { h } from "./ui.js";

export const foot = "Node names, topics, types and parameters are taken from the launch files and sources listed in each panel.";

const NODES = {
  camera: { x: 20, y: 40, w: 170, h: 60, title: "realsense2_camera", kind: "ROS node", color: "--vision",
    facts: [
      ["Publishes", "/camera/camera/color/image_raw, /camera/camera/aligned_depth_to_color/image_raw (16-bit, mm), /camera/camera/color/camera_info"],
      ["Started by", "vision_module/launch/vision.launch.py (includes rs_launch.py with align_depth.enable: true, pointcloud.enable: false)"],
    ] },
  vision: { x: 260, y: 30, w: 190, h: 80, title: "vision_node", kind: "ROS node, Python", color: "--vision",
    facts: [
      ["Subscribes", "the three camera topics"],
      ["Publishes", "/vision/detected_objects (std_msgs/String, JSON list in camera_link), /debug_image (sensor_msgs/Image)"],
      ["Reads", "tools[] of shared/sequence.json, from a hard-coded directory"],
      ["Models", "YOLO-World yolov8l-worldv2.pt, conf 0.5; ArUco DICT_ARUCO_ORIGINAL inside each box"],
      ["Source", "vision_module/src/vision_module/vision_node.py"],
    ] },
  statictf: { x: 260, y: 160, w: 190, h: 56, title: "static_transform_publisher", kind: "ROS node (tf2_ros)", color: "--tf",
    facts: [
      ["Publishes", "/tf_static: fr3_link0 to camera_link"],
      ["Values", "t = (0.4427, 0.4566, 0.6999), q = (-0.0077, 0.9606, -0.2777, 0.0022)"],
      ["Origin", "computed with vision_module/src/vision_module/aruco/extrinsinc_calibration.py, pasted into the launch file"],
      ["Source", "monitor_cpp/launch/move_robot.launch.py (node name camera_link_broadcaster)"],
    ] },
  mission: { x: 260, y: 262, w: 190, h: 56, title: "shared/sequence.json", kind: "data file", color: "--mission",
    facts: [
      ["tools[]", "vocabulary of the detector (vision_node)"],
      ["sequence[]", "ordered tasks {tool, startTime} (move_to_pose_node, via the mission_file parameter)"],
      ["project", "printed in the controller log"],
    ] },
  frontend: { x: 20, y: 262, w: 170, h: 56, title: "frontend/ page", kind: "static web page", color: "--mission",
    facts: [
      ["Does", "edits tools and sequence, sets handedness, exports dum-e-sequence.json, imports JSON"],
      ["Does not", "contact ROS; the Start button only shows an alert; the recorded audio stays in the browser"],
      ["Source", "frontend/index.html, frontend/script.js"],
    ] },
  controller: { x: 540, y: 150, w: 200, h: 110, title: "move_to_pose_node", kind: "ROS node, C++", color: "--ctrl",
    facts: [
      ["Subscribes", "/vision/detected_objects, /hmi/intent_raw, /tf, /tf_static"],
      ["Action clients", "/franka_gripper/grasp, /franka_gripper/move"],
      ["MoveIt", "MoveGroupInterface: group fr3_arm, end effector fr3_hand_tcp, velocity and acceleration scaling 0.15"],
      ["Parameter", "mission_file (absolute path)"],
      ["Threads", "MultiThreadedExecutor in a std::thread; mission loop on the main thread"],
      ["Source", "monitor_cpp/src/move_to_pose.cpp, launched by monitor_cpp/launch/move_robot.launch.py"],
    ] },
  movegroup: { x: 800, y: 70, w: 180, h: 60, title: "move_group", kind: "MoveIt 2", color: "--motion",
    facts: [
      ["Provides", "Cartesian path computation, trajectory execution, planning to the named target ready"],
      ["Started by", "franka_fr3_moveit_config/launch/moveit.launch.py, included by move_robot.launch.py (also starts RViz)"],
    ] },
  arm: { x: 800, y: 175, w: 180, h: 56, title: "FR3 controllers", kind: "ros2_control", color: "--motion",
    facts: [
      ["Arguments", "robot_ip (default 172.16.0.2), use_fake_hardware (default false)"],
      ["Started by", "the Franka MoveIt launch file"],
    ] },
  gripper: { x: 800, y: 275, w: 180, h: 56, title: "franka_gripper", kind: "ROS node", color: "--ctrl",
    facts: [
      ["Actions", "grasp (width 0.04, speed 0.1, force 40 N, epsilon 0.05/0.05), move (width 0.08, speed 0.1)"],
      ["Started by", "the Franka MoveIt launch file with load_gripper: true"],
    ] },
  page: { x: 20, y: 430, w: 170, h: 60, title: "HMI web page", kind: "browser", color: "--voice",
    facts: [
      ["Does", "captures the microphone (echo cancellation, noise suppression, gain 1.5), downsamples to 16 kHz, streams PCM16; shows transcript and intent; Confirm / Discard; push-to-talk on Space"],
      ["Source", "hmi_pkg/public/index.html, served at / by hmi_server"],
    ] },
  hmi: { x: 260, y: 420, w: 190, h: 80, title: "hmi_server", kind: "FastAPI + rclpy thread", color: "--voice",
    facts: [
      ["Endpoints", "WS /asr and /ws; POST /confirm, /discard, /unlock; GET /status, /debug, /debug/audio"],
      ["Pipeline", "VAD, Whisper, filters, NLU, intent merge, lock"],
      ["Publishes", "/hmi/intent_raw (always when the bridge is on), /hmi/asr_text (only if HMI_ROS_CONFIRM_MODE is not intent)"],
      ["ROS node", "hmi_bridge, enabled by HMI_ROS_ENABLE (1 in config.py, 0 in docker-compose.yml)"],
      ["Source", "hmi_pkg/nodes/*.py, hmi_pkg/config/capabilities.json"],
    ] },
  whisper: { x: 260, y: 535, w: 190, h: 48, title: "OpenAI Whisper API", kind: "external service", color: "--voice",
    facts: [
      ["Request", "one WAV file per speech segment, model whisper-1, language en"],
      ["Requires", "OPENAI_API_KEY"],
    ] },
};

const EDGES = [
  ["camera", "vision", "image, aligned depth, K", "--vision", "M190 70 C225 70 225 70 260 70"],
  ["vision", "controller", "/vision/detected_objects", "--vision", "M450 70 C600 70 640 90 640 150"],
  ["statictf", "controller", "/tf_static", "--tf", "M450 188 C495 188 495 188 540 188"],
  ["mission", "vision", "tools[]", "--mission", "M260 290 C215 290 215 95 260 95"],
  ["mission", "controller", "sequence[] (mission_file)", "--mission", "M450 290 C500 290 500 235 540 235"],
  ["frontend", "mission", "export, then copy by hand", "--mission", "M190 290 C225 290 225 290 260 290", true],
  ["page", "hmi", "WS /asr: PCM16, 16 kHz", "--voice", "M190 450 C225 450 225 450 260 450"],
  ["hmi", "page", "final, intent, locked", "--voice", "M260 478 C225 478 225 478 190 478"],
  ["hmi", "whisper", "HTTPS, WAV", "--voice", "M355 500 C355 515 355 520 355 535", true],
  ["hmi", "controller", "/hmi/intent_raw", "--voice", "M450 460 C600 460 640 330 640 260"],
  ["controller", "movegroup", "MoveGroupInterface", "--motion", "M740 175 C770 175 770 100 800 100"],
  ["movegroup", "arm", "trajectories", "--motion", "M890 130 C890 150 890 155 890 175"],
  ["controller", "gripper", "/franka_gripper/grasp, /move", "--ctrl", "M740 240 C770 240 770 303 800 303"],
];

export function mount(root) {
  const NS = "http://www.w3.org/2000/svg";
  const svg = document.createElementNS(NS, "svg");
  svg.setAttribute("viewBox", "0 0 1000 600");
  svg.setAttribute("class", "svg-graph");
  svg.style.width = "100%";
  svg.style.height = "auto";
  svg.style.display = "block";

  const defs = document.createElementNS(NS, "defs");
  svg.appendChild(defs);
  const markerFor = {};
  for (const color of new Set(EDGES.map((e) => e[3]))) {
    const id = "rg-arrow-" + color.slice(2);
    const m = document.createElementNS(NS, "marker");
    m.setAttribute("id", id);
    m.setAttribute("viewBox", "0 0 10 10");
    m.setAttribute("refX", "8.5");
    m.setAttribute("refY", "5");
    m.setAttribute("markerWidth", "7");
    m.setAttribute("markerHeight", "7");
    m.setAttribute("orient", "auto-start-reverse");
    const p = document.createElementNS(NS, "path");
    p.setAttribute("d", "M0 0L10 5L0 10z");
    p.setAttribute("style", `fill: var(${color})`);
    m.appendChild(p);
    defs.appendChild(m);
    markerFor[color] = id;
  }

  const edgeEls = [];
  for (const [from, to, label, color, d, dashed] of EDGES) {
    const g = document.createElementNS(NS, "g");
    g.setAttribute("class", "edge");
    const path = document.createElementNS(NS, "path");
    path.setAttribute("d", d);
    path.setAttribute("style", `stroke: var(${color})`);
    path.setAttribute("marker-end", `url(#${markerFor[color]})`);
    if (dashed) path.setAttribute("stroke-dasharray", "6 5");
    g.appendChild(path);
    const text = document.createElementNS(NS, "text");
    text.textContent = label;
    text.setAttribute("text-anchor", "middle");
    text.setAttribute("style", `fill: var(${color})`);
    g.appendChild(text);
    svg.appendChild(g);
    edgeEls.push({ from, to, g, path, text });
  }

  const nodeEls = {};
  for (const [id, n] of Object.entries(NODES)) {
    const g = document.createElementNS(NS, "g");
    g.setAttribute("class", "node");
    g.setAttribute("tabindex", "0");
    const rect = document.createElementNS(NS, "rect");
    for (const [k, v] of Object.entries({ x: n.x, y: n.y, width: n.w, height: n.h, rx: 12 })) rect.setAttribute(k, v);
    g.appendChild(rect);
    const bar = document.createElementNS(NS, "rect");
    for (const [k, v] of Object.entries({ x: n.x, y: n.y + 10, width: 4, height: n.h - 20, rx: 2 })) bar.setAttribute(k, v);
    bar.setAttribute("style", `fill: var(${n.color})`);
    g.appendChild(bar);
    const t1 = document.createElementNS(NS, "text");
    t1.setAttribute("x", n.x + 16);
    t1.setAttribute("y", n.y + n.h / 2 - 2);
    t1.setAttribute("style", "fill: var(--text); font-weight: 650; font-size: 14px");
    t1.textContent = n.title;
    const t2 = document.createElementNS(NS, "text");
    t2.setAttribute("x", n.x + 16);
    t2.setAttribute("y", n.y + n.h / 2 + 16);
    t2.setAttribute("style", "fill: var(--muted); font-size: 11.5px");
    t2.textContent = n.kind;
    g.appendChild(t1);
    g.appendChild(t2);
    g.addEventListener("click", () => select(id));
    g.addEventListener("keydown", (ev) => { if (ev.key === "Enter" || ev.key === " ") select(id); });
    svg.appendChild(g);
    nodeEls[id] = g;
  }

  const details = h("div", { class: "panel", style: { marginTop: "14px" } });
  root.appendChild(svg);
  root.appendChild(details);

  for (const e of edgeEls) {
    const len = e.path.getTotalLength();
    const mid = e.path.getPointAtLength(len / 2);
    e.text.setAttribute("x", mid.x);
    e.text.setAttribute("y", mid.y - 7);
  }

  function select(id) {
    for (const [k, g] of Object.entries(nodeEls)) g.classList.toggle("sel", k === id);
    for (const e of edgeEls) e.g.classList.toggle("hi", e.from === id || e.to === id);
    const n = NODES[id];
    details.innerHTML = "";
    details.appendChild(h("h4", {}, n.kind));
    details.appendChild(h("div", { style: { fontWeight: 650, fontSize: "16px", marginBottom: "8px" } }, n.title));
    const dl = h("dl", { class: "kv" });
    for (const [k, v] of n.facts) {
      dl.appendChild(h("dt", {}, k));
      dl.appendChild(h("dd", {}, v));
    }
    details.appendChild(dl);
    const ins = edgeEls.filter((e) => e.to === id).map((e) => `${NODES[e.from].title}: ${e.text.textContent}`);
    const outs = edgeEls.filter((e) => e.from === id).map((e) => `${NODES[e.to].title}: ${e.text.textContent}`);
    const io = h("dl", { class: "kv", style: { marginTop: "10px" } });
    if (ins.length) { io.appendChild(h("dt", {}, "Inbound")); io.appendChild(h("dd", {}, ins.join("; "))); }
    if (outs.length) { io.appendChild(h("dt", {}, "Outbound")); io.appendChild(h("dd", {}, outs.join("; "))); }
    details.appendChild(io);
  }
  select("controller");
}
