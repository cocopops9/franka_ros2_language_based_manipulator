# 7. End-to-end run

[Docs index](README.md) · [1. Overview](01-system-overview.md) · [2. Mission](02-mission-and-scheduling.md) · [3. Perception](03-perception.md) · [4. Frames](04-calibration-and-frames.md) · [5. Voice](05-voice-interface.md) · [6. Motion](06-motion-and-grasping.md) · **7. End to end**

The previous chapters described each stage in isolation. This chapter follows one complete run of the shipped mission, then lists the failure modes that the code allows and how to recognise them.

## 7.1 Start-up order

The mission clock starts as soon as `move_to_pose_node` has loaded the mission file, and the first task (`startTime` 0) is dispatched immediately. Start the producers first, so that the object memory is already populated:

| Order | Command | Wait for |
| :---: | :------ | :------- |
| 1 | `ros2 launch vision_module vision.launch.py` | `Camera Intrinsics Received!`, then `Detected positions: ...` lines |
| 2 | `python3 hmi_pkg/nodes/hmi_server.py` (with `HMI_ROS_ENABLE=1`) | `ROS bridge enabled ...`; open the page and wait for calibration (700 ms of silence) |
| 3 | `ros2 launch monitor_cpp move_robot.launch.py mission_file:=...` | `[SYSTEM] Loading Project: Dum-E`, `>>> PROJECT STARTED: 3 Tasks Loaded <<<` |

The exact commands, with all environment variables, are in the [README usage section](../README.md#usage).

## 7.2 A run of the shipped mission

Times are relative to the controller's mission clock. Fixed values come from the code; motion and human times are examples.

| t (s) | Event | Controller log |
| ----: | :---- | :------------- |
| 0 | mission loaded; task 1 (fork, `startTime` 0) dispatched at once because $`0 - 5 < 0`$ | `--- NEXT TASK: Pick fork at T=0s ---` |
| 0 | fork found in memory (stored earlier from `/vision/detected_objects`) | |
| 0 to 2 | straight line to the intermediate pose | `PLANNING: Intermediate...` |
| 2 to 12 | fixed pause | `Moving to fork: (0.500, 0.125, -0.024, 0.71, 0.71, 0.00)` |
| 12 to 16 | pre-grasp, then descend by 0.103 m | `PLANNING: Pre-Grasp...`, `PLANNING: Pick...` |
| 16 to 18 | grasp, then 1 s pause | `ACTION: Grasping (Waiting)...`, `Grasp Successful.` |
| 18 to 25 | retract, intermediate, release pose | `PLANNING: Retract...`, `PLANNING: Intermediate...`, `PLANNING: Release Zone...` |
| 25 | waiting; release flag cleared | `WAITING FOR RELEASE...` |
| 29 | operator says "release"; `/hmi/intent_raw` receives `{"action":"release","object":null,"allow_execution":false}` | `[HMI] RELEASE SIGNAL RECEIVED!` |
| 29 to 31 | gripper opens to 8 cm, then 1 s pause | `ACTION: Opening Gripper (Waiting for completion)...` |
| 31 | task 2 (spoon, `startTime` 5): $`5 - 5 = 0 \le 31`$, so it starts immediately | `--- NEXT TASK: Pick spoon at T=5s ---` |
| about 62 | task 3 (bottle) likewise | `--- NEXT TASK: Pick bottle at T=5s ---` |
| about 93 | joint-space move to `ready` | `Project Complete. Going Home.` |

The pick coordinates in the log assume the example fork position $`(0.52, 0.20, -0.07)`$ used throughout these docs.

<!-- lab:e2e -->
> **Interactive lab:** [run the whole mission in the browser](https://cocopops9.github.io/franka_ros2_language_based_manipulator/#/07-end-to-end-run?lab=e2e): choose which tools the camera sees, say "release" with a button, and follow the arm, the schedule and the controller log.

## 7.3 Failure modes and diagnosis

| Symptom | Cause in the code | How to check |
| :------ | :---------------- | :----------- |
| `SKIPPING: Object 'X' never found in vision memory!` | No marker-confirmed detection of `X` reached the controller within 5 s: `X` missing from `tools`, marker not visible, depth hole, vision node reading a different `sequence.json`, or `fr3_link0 → camera_link` not available (points are then dropped silently). | `ros2 topic echo /vision/detected_objects`; view `/debug_image` (red box = published); `ros2 run tf2_ros tf2_echo fr3_link0 camera_link`. |
| `FAILED: Pre-Grasp (Coverage: 42.0%)` | The straight line is infeasible beyond the reported fraction (IK limits, joint limits or collision). The task is abandoned. | Check the logged pick coordinates against the workspace; check calibration. |
| The arm waits forever at the release pose | No message containing `release` after the wait started: ROS bridge disabled (`HMI_ROS_ENABLE=0`, the default in `docker-compose.yml`), DDS discovery between container and host, the word spoken before the arm arrived, or the system locked on a previous intent. | `curl localhost:8000/status` (`ros_ready`, `locked`); `ros2 topic echo /hmi/intent_raw`; test with `ros2 topic pub --once /hmi/intent_raw std_msgs/msg/String "{data: release}"`. |
| The gripper opens as soon as the arm arrives | A message containing `release` arrived after the wait started, for example **Confirm** pressed late for an earlier "release". | Look at the order of `WAITING FOR RELEASE...` and the HMI log. |
| The grasp misses but the arm still carries on | Grasp tolerance accepts any width (Section 6.5) and the result is not checked. | Watch the fingers; the log may still say `Grasp Successful.` |
| The arm reaches for an old position | Memory entries never expire, are updated only beyond 2 cm, and are copied at dispatch time, before the 10 s pause. | Compare `[VISION] Memory updated for: ...` lines with the scene. |
| No transcript | `OPENAI_API_KEY` missing; segment shorter than 300 ms or RMS below 0.01; gate too high because someone spoke during calibration; the system is locked. | `GET /debug` shows `noise_rms`, `gate_rms`, `locked`; restart the server in silence to recalibrate. |
| The parser returns an unexpected intent | Multi-word aliases are unreachable; ties depend on `PYTHONHASHSEED`; `CAPABILITIES_PATH` not set outside Docker (fallback vocabulary). | Try the phrase in the NLU lab; set `CAPABILITIES_PATH`. |
| `Mission is empty or file not found! Exiting.` | `mission_file` points to a missing or invalid file. The process then aborts instead of exiting cleanly, because the executor thread is never joined on this path. | Pass an absolute `mission_file:=...`. |
| `Gripper Action Server not available!` | No `franka_gripper` action server is running (possibly the case with `use_fake_hardware:=true`). The grasp is then skipped silently after 5 s. | `ros2 action list`. |

## 7.4 Known limitations

These follow from the current code and are listed so that results can be interpreted correctly.

1. **Machine-specific paths.** `vision_node.py` reads `/home/userlab/cjimenez/franka_ros2_language_based_manipulator/shared/sequence.json`; the default `mission_file` and a `link_directories` entry in `monitor_cpp/CMakeLists.txt` point to the same user directory.
2. **Handedness is not used.** The frontend exports it, but the left-hand release pose is commented out and nothing reads the field.
3. **The release trigger is a substring test.** `allow_execution` and the confirmation step are ignored by the controller.
4. **Grasp outcome is not verified** (tolerance band of 10 cm, result unused).
5. **Partial Cartesian paths are executed** (`fraction > 0.90`) and the joint-jump check is disabled.
6. **Object memory has no expiry** and is sampled once per task, before a 10 s pause.
7. **Parser quirks**: multi-word aliases never match as phrases; tie-breaking depends on the hash seed.
8. **VAD calibration is global and one-off**; the minimum segment length is ineffective with the default hang-over.
9. **The synchronous Whisper call blocks** the server's event loop during each transcription.
10. **Depth is read at one pixel**, without synchronisation between colour and depth frames.
11. **The frontend is offline**: it exports a file but does not start or monitor the mission.

## 7.5 What to take away

The system is a pipeline of independent producers and one blocking consumer. Knowing the few rules the consumer applies (dispatch at $`\max(s_k - 5, f_{k-1})`$, look up a snapshot, six straight lines, wait for a substring) is enough to predict its behaviour in every situation listed above.

<p align="right"><b>Back to:</b> <a href="README.md">Docs index</a></p>
