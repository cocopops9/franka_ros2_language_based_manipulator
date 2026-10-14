// Constants and pure functions mirrored from monitor_cpp/src/move_to_pose.cpp,
// monitor_cpp/launch/move_robot.launch.py and vision_module/src/vision_module/vision_node.py.
// Every number below is copied from the source; nothing is tuned for the docs.

// ---------------------------------------------------------------------------
// Rotations (tf2 conventions)
// ---------------------------------------------------------------------------

const DEG = Math.PI / 180;

// tf2::Quaternion::setRPY(roll, pitch, yaw), angles in radians. Returns [x, y, z, w].
export function quaternionFromRPY(roll, pitch, yaw) {
  const hy = yaw * 0.5, hp = pitch * 0.5, hr = roll * 0.5;
  const cy = Math.cos(hy), sy = Math.sin(hy);
  const cp = Math.cos(hp), sp = Math.sin(hp);
  const cr = Math.cos(hr), sr = Math.sin(hr);
  return [
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  ];
}

// Rotation matrix (row-major 3x3) of a unit quaternion [x, y, z, w].
export function matrixFromQuaternion([x, y, z, w]) {
  const n = Math.hypot(x, y, z, w) || 1;
  x /= n; y /= n; z /= n; w /= n;
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ];
}

export function applyRotation(R, v) {
  return [
    R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
    R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
    R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
  ];
}

export function transposeRotation(R) {
  return [
    [R[0][0], R[1][0], R[2][0]],
    [R[0][1], R[1][1], R[2][1]],
    [R[0][2], R[1][2], R[2][2]],
  ];
}

// create_pose(x, y, z, r, p, yw) with angles in degrees, as in move_to_pose.cpp.
export function createPose(x, y, z, rDeg, pDeg, yDeg) {
  return {
    position: { x, y, z },
    rpyDeg: [rDeg, pDeg, yDeg],
    orientation: quaternionFromRPY(rDeg * DEG, pDeg * DEG, yDeg * DEG),
  };
}

// ---------------------------------------------------------------------------
// Fixed poses and motion parameters (move_to_pose.cpp)
// ---------------------------------------------------------------------------

export const PLANNING_GROUP = "fr3_arm";
export const END_EFFECTOR_LINK = "fr3_hand_tcp";
export const VELOCITY_SCALING = 0.15;
export const ACCELERATION_SCALING = 0.15;
export const CARTESIAN_EEF_STEP = 0.01;   // metres between interpolated waypoints
export const CARTESIAN_JUMP_THRESHOLD = 0.0; // 0 disables the joint-space jump check
export const MIN_PATH_FRACTION = 0.9;     // execute only if fraction > 0.90

export const INTERMEDIATE_POSE = createPose(0.45, 0.0, 0.35, 179.5, -5.8, -2.0);
export const RELEASE_POSE_RIGHT = createPose(0.598, -0.49, 0.22, 179.9, -3.9, -92.2);
// Present in the source but commented out (no runtime switch reads it yet).
export const RELEASE_POSE_LEFT = createPose(0.219, -0.455, 0.22, 179.9, -3.9, -92.2);

export const PICK_RPY_DEG = [180, 0.0, 90];
export const PICK_OFFSET_X = -0.02;
export const PICK_OFFSET_Y = -0.075;
export const LOW_OBJECT_Z = 0.056;       // below this, the pick height is clamped
export const SAFETY_FLOOR_Z = -0.024;
export const TABLE_Z_OFFSET = -0.08;
export const PRE_GRASP_HOVER = 0.103;

export const TRAVEL_TIME_OFFSET = 5.0;   // seconds subtracted from every startTime
export const MEMORY_LOOKUP_TRIES = 50;   // x 100 ms = 5 s
export const MEMORY_LOOKUP_PERIOD = 0.1;
export const PRE_GRASP_PAUSE = 10;       // std::this_thread::sleep_for(10 s)
export const POST_GRASP_PAUSE = 1;
export const POST_RELEASE_PAUSE = 1;
export const MEMORY_UPDATE_THRESHOLD = 0.02; // metres

export const GRIPPER_OPEN_GOAL = { width: 0.08, speed: 0.1 };
export const GRIPPER_GRASP_GOAL = { width: 0.04, speed: 0.1, force: 40.0, epsilonInner: 0.05, epsilonOuter: 0.05 };

// Pick and pre-grasp poses computed from an object position in fr3_link0.
export function pickPoses(objX, objY, objZ) {
  const pick = createPose(objX, objY, objZ, ...PICK_RPY_DEG);
  pick.position.x += PICK_OFFSET_X;
  pick.position.y += PICK_OFFSET_Y;
  let zRule;
  if (pick.position.z < LOW_OBJECT_Z) {
    pick.position.z = SAFETY_FLOOR_Z;
    zRule = "safety floor";
  } else {
    pick.position.z += TABLE_Z_OFFSET;
    zRule = "table offset";
  }
  const preGrasp = {
    position: { ...pick.position, z: pick.position.z + PRE_GRASP_HOVER },
    rpyDeg: pick.rpyDeg,
    orientation: pick.orientation,
  };
  return { pick, preGrasp, zRule };
}

// franka_gripper / libfranka Grasp semantics: the grasp is reported as successful
// if width - epsilon_inner < d < width + epsilon_outer, where d is the finger
// distance at which the fingers stopped.
export function graspSucceeds(stoppedWidth, goal = GRIPPER_GRASP_GOAL) {
  return stoppedWidth > goal.width - goal.epsilonInner && stoppedWidth < goal.width + goal.epsilonOuter;
}

// ---------------------------------------------------------------------------
// Mission file (shared/sequence.json) and scheduling
// ---------------------------------------------------------------------------

// load_mission_from_file(): keeps file order, startTime is read as an int.
export function parseMission(jsonText) {
  const data = JSON.parse(jsonText);
  const project = data.project !== undefined ? data.project : "Unknown";
  const sequence = Array.isArray(data.sequence) ? data.sequence : [];
  const tasks = sequence.map((item) => ({ tool: String(item.tool), startTime: Math.trunc(Number(item.startTime)) }));
  const tools = Array.isArray(data.tools) ? data.tools.map(String) : null;
  return { project, tools, tasks };
}

// Time at which the controller leaves the wait loop for a task, given the time
// at which the previous task finished. The loop breaks as soon as
// startTime - elapsed - 5 <= 0, so a late task starts immediately.
export function dispatchTime(startTime, previousEnd) {
  return Math.max(startTime - TRAVEL_TIME_OFFSET, previousEnd);
}

// ---------------------------------------------------------------------------
// Vision: object memory (vision_callback in move_to_pose.cpp)
// ---------------------------------------------------------------------------

export class ObjectMemory {
  constructor() {
    this.items = new Map();
  }

  // Returns "stored", "updated" or "kept" exactly like the C++ callback.
  observe(label, x, y, z) {
    const old = this.items.get(label);
    if (!old) {
      this.items.set(label, { x, y, z });
      return "stored";
    }
    const dist = Math.hypot(x - old.x, y - old.y, z - old.z);
    if (dist > MEMORY_UPDATE_THRESHOLD) {
      old.x = x; old.y = y; old.z = z;
      return "updated";
    }
    return "kept";
  }
}

// ---------------------------------------------------------------------------
// Camera model and extrinsics
// ---------------------------------------------------------------------------

// static_transform_publisher arguments in move_robot.launch.py (fr3_link0 -> camera_link).
export const CAMERA_TRANSLATION = [0.4427, 0.4566, 0.6999];
export const CAMERA_QUATERNION = [-0.0077, 0.9606, -0.2777, 0.0022];
export const CAMERA_ROTATION = matrixFromQuaternion(CAMERA_QUATERNION);

// Point expressed in camera_link -> fr3_link0 (what tf_buffer->transform does).
export function cameraToBase(p) {
  const r = applyRotation(CAMERA_ROTATION, p);
  return [r[0] + CAMERA_TRANSLATION[0], r[1] + CAMERA_TRANSLATION[1], r[2] + CAMERA_TRANSLATION[2]];
}

export function baseToCamera(p) {
  const d = [p[0] - CAMERA_TRANSLATION[0], p[1] - CAMERA_TRANSLATION[1], p[2] - CAMERA_TRANSLATION[2]];
  return applyRotation(transposeRotation(CAMERA_ROTATION), d);
}

// VisionNode.deproject(u, v, z) with K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]].
export function deproject(u, v, z, K) {
  return [((u - K.cx) * z) / K.fx, ((v - K.cy) * z) / K.fy, z];
}

export function project(p, K) {
  return [K.fx * (p[0] / p[2]) + K.cx, K.fy * (p[1] / p[2]) + K.cy];
}

// Calibration marker pose assumed by aruco/extrinsinc_calibration.py.
export const CALIBRATION_MARKER = {
  translation: [0.3, 0.0, -0.075],
  rotation: [
    [0, -1, 0],
    [1, 0, 0],
    [0, 0, 1],
  ],
  size: 0.015,
  id: 0,
};

// ArUco settings used by vision_node.py.
export const ARUCO_DICTIONARY = "DICT_ARUCO_ORIGINAL";
export const ARUCO_MARKER_LENGTH = 0.015;
export const YOLO_MODEL = "yolov8l-worldv2.pt";
export const YOLO_CONFIDENCE = 0.5;

// ---------------------------------------------------------------------------
// Data shared by the labs
// ---------------------------------------------------------------------------

// Copy of shared/sequence.json (checked by docs/tools/check_parity.py).
export const DEFAULT_MISSION = {
  project: "Dum-E",
  tools: ["bottle", "spoon", "fork"],
  sequence: [
    { tool: "fork", startTime: 0 },
    { tool: "spoon", startTime: 5 },
    { tool: "bottle", startTime: 5 },
  ],
};

// Example object positions in fr3_link0, the same as in the generated figures.
// They depend on the scene and are only illustrative.
export const EXAMPLE_OBJECTS = {
  fork: [0.52, 0.2, -0.07],
  spoon: [0.4, 0.12, -0.07],
  bottle: [0.6, 0.05, 0.1],
  toy: [0.47, -0.05, -0.04],
};

export const TABLE_Z = -0.075; // calibration marker height in extrinsinc_calibration.py

// Example colour intrinsics of a RealSense D4xx at 640 x 480; the node reads the real
// values from /camera/camera/color/camera_info.
export const EXAMPLE_K = { fx: 615, fy: 615, cx: 320, cy: 240, width: 640, height: 480 };

// Approximate TCP position of the SRDF "ready" state, for drawing only.
export const READY_TCP = [0.307, 0.0, 0.487];
