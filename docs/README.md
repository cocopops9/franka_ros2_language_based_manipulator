# Dum-E documentation: a guided tour

<p align="center">
  <img src="assets/svg/architecture.svg" alt="Animated architecture of the system" width="100%">
</p>

This folder explains **how the system works and why**, in the order in which data flows through it.
Every chapter builds on the previous one, so reading them in sequence gives a complete, linear picture:
from the mission file, through perception and coordinate frames, to the voice interface and the motion that hands a tool to a person.

> **Interactive version.** The same chapters are published as a website with live labs (a voice parser you can type into, a microphone-driven voice detector, a camera you can move, a mission simulator).
> Open <https://cocopops9.github.io/franka_ros2_language_based_manipulator/> once GitHub Pages is enabled for the `docs/` folder, or serve it locally:
> `python3 -m http.server 8080 --directory docs` and browse to <http://localhost:8080>.

## Reading path

| Step | Chapter | Question it answers | Lab on the website |
| :--: | :------ | :------------------ | :----------------- |
| 1 | [System overview](01-system-overview.md) | Which processes run, and which messages connect them? | ROS graph explorer |
| 2 | [Mission and scheduling](02-mission-and-scheduling.md) | How does the robot know *what* to pick and *when*? | Mission planner |
| 3 | [Perception](03-perception.md) | How does a camera pixel become a 3-D point? | Virtual camera |
| 4 | [Calibration, frames and memory](04-calibration-and-frames.md) | How is that point expressed in the robot base frame, and remembered? | 3-D frames, memory hysteresis |
| 5 | [Voice interface](05-voice-interface.md) | How does a spoken word become a robot event? | Microphone VAD, NLU parser, confirm/lock state machine |
| 6 | [Motion and grasping](06-motion-and-grasping.md) | Which poses does the arm visit, and how are paths and grasps computed? | Pick-pose calculator, Cartesian path, grasp tolerance |
| 7 | [End-to-end run](07-end-to-end-run.md) | What happens second by second, and what can go wrong? | Full mission simulator |

## Conventions

* Units are SI: metres, seconds, radians (degrees only where the code uses degrees, for example `create_pose`).
* `fr3_link0` is the robot base frame: $`x`$ forward, $`y`$ to the robot's left, $`z`$ up.
* `camera_link` is the calibrated camera frame. It follows the optical convention used by the vision node: $`x`$ right, $`y`$ down, $`z`$ along the viewing direction (Chapter 4 explains why).
* Code identifiers are written `like_this`; file paths are relative to the repository root.
* Statements about *external* software (MoveIt, libfranka, Whisper, YOLO-World) are marked as such; everything else is read from this repository.

## How the figures and labs stay faithful to the code

Animations are not drawn by hand. Two scripts in [`tools/`](tools) keep them tied to the implementation:

* [`tools/generate_svgs.py`](tools/generate_svgs.py) builds every animated SVG. Waypoints, offsets and gripper goals are parsed from `monitor_cpp/src/move_to_pose.cpp`, the camera pose from `monitor_cpp/launch/move_robot.launch.py`, the mission from `shared/sequence.json`; the voice-activity trace is produced by running `hmi_pkg/nodes/vad.py` itself.
* [`tools/check_parity.py`](tools/check_parity.py) runs the original Python modules of `hmi_pkg/nodes` and the JavaScript ports used by the website on the same inputs (692 phrases, 996 audio chunks, 12 scripted dialogues), and compares every numeric constant of the controller with the C++ source.

```bash
python3 docs/tools/check_parity.py    # requires python3 and node; no extra packages
python3 docs/tools/generate_svgs.py   # rewrites docs/assets/svg/*.svg
```

Values that depend on the physical setup (object positions, camera intrinsics, motion durations) are labelled **example** wherever they appear.

<p align="right"><b>Start here:</b> <a href="01-system-overview.md">1. System overview</a></p>
