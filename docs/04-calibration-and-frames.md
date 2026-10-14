# 4. Calibration, frames and memory

[Docs index](README.md) · [1. Overview](01-system-overview.md) · [2. Mission](02-mission-and-scheduling.md) · [3. Perception](03-perception.md) · **4. Frames** · [5. Voice](05-voice-interface.md) · [6. Motion](06-motion-and-grasping.md) · [7. End to end](07-end-to-end-run.md)

The vision node reports points in the camera frame; the arm needs them in its base frame.
This chapter covers the rigid transformation between the two, how it was estimated, how the controller applies it, and how the resulting positions are remembered.

<p align="center"><img src="assets/svg/frames.svg" alt="Coordinate frames of the setup" width="100%"></p>

## 4.1 Rigid transformations

A pose of frame $`B`$ in frame $`A`$ is a homogeneous matrix

```math
{}^{A}T_{B} = \begin{bmatrix} {}^{A}R_{B} & {}^{A}t_{B} \\ 0 & 1 \end{bmatrix} \in SE(3),
\qquad
{}^{A}p = {}^{A}R_{B}\, {}^{B}p + {}^{A}t_{B},
```

with composition $`{}^{A}T_{C} = {}^{A}T_{B}\,{}^{B}T_{C}`$ and inverse $`{}^{B}T_{A} = \begin{bmatrix} R^{\top} & -R^{\top} t \\ 0 & 1 \end{bmatrix}`$.
ROS stores rotations as unit quaternions $`q = (q_x, q_y, q_z, q_w)`$; the rotation matrix is

```math
R(q) = \begin{bmatrix}
1 - 2(q_y^2 + q_z^2) & 2(q_x q_y - q_z q_w) & 2(q_x q_z + q_y q_w) \\
2(q_x q_y + q_z q_w) & 1 - 2(q_x^2 + q_z^2) & 2(q_y q_z - q_x q_w) \\
2(q_x q_z - q_y q_w) & 2(q_y q_z + q_x q_w) & 1 - 2(q_x^2 + q_y^2)
\end{bmatrix}.
```

The frames involved:

| Frame | Defined by | Meaning |
| :---- | :--------- | :------ |
| `fr3_link0` | robot description (`franka_description`) | Robot base: $`x`$ forward, $`y`$ left, $`z`$ up. All targets are expressed here. |
| `fr3_hand_tcp` | robot description | Tool centre point between the fingertips; the end-effector link of the planning group. |
| `camera_link` | the static transform in `move_robot.launch.py` | Calibrated camera frame in which the vision node's numbers are valid (optical convention). |
| marker frame | `extrinsinc_calibration.py` | Frame of calibration marker ID 0, only used during calibration. |

## 4.2 Extrinsic calibration with one marker

`vision_module/src/vision_module/aruco/extrinsinc_calibration.py` estimates the camera pose from a single marker placed at a **measured** pose on the table.

**Known:** the marker pose in the base frame, hard-coded as

```math
{}^{base}t_{marker} = (0.30,\ 0.00,\ -0.075)\ \text{m},
\qquad
{}^{base}R_{marker} = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix} = R_z(+90^\circ),
```

so the marker's $`x`$ axis points along the base $`+y`$ and its $`y`$ axis along base $`-x`$. (The comments next to the matrix in the script describe the axes differently; the matrix is what the computation uses.)

**Measured:** the marker pose in the camera frame. The four corners of a square of side $`L = 0.015`$ m, in marker coordinates,

```math
P_1 = \left(-\tfrac{L}{2}, \tfrac{L}{2}, 0\right),\;
P_2 = \left(\tfrac{L}{2}, \tfrac{L}{2}, 0\right),\;
P_3 = \left(\tfrac{L}{2}, -\tfrac{L}{2}, 0\right),\;
P_4 = \left(-\tfrac{L}{2}, -\tfrac{L}{2}, 0\right),
```

are matched with the detected image corners $`p_i`$ (same order as OpenCV returns them), and `cv2.solvePnP` finds the pose that minimises the reprojection error

```math
{}^{cam}T_{marker} = \arg\min_{R,\,t} \sum_{i=1}^{4} \left\lVert p_i - \pi\!\left(K (R P_i + t)\right) \right\rVert^2 ,
```

where $`\pi`$ divides by the third coordinate. `cv2.Rodrigues` converts the rotation vector to $`R`$.

**Chain:** since $`{}^{base}T_{marker} = {}^{base}T_{cam}\;{}^{cam}T_{marker}`$,

```math
{}^{base}T_{cam} = {}^{base}T_{marker}\;\left({}^{cam}T_{marker}\right)^{-1}.
```

The translation and the quaternion of $`{}^{base}T_{cam}`$ (SciPy order $`x, y, z, w`$) are printed and then copied by hand into the `static_transform_publisher` arguments.

**Accuracy.** One small marker gives a weakly constrained orientation: an angular error $`\delta\theta`$ moves every point at distance $`d`$ by about $`d\,\delta\theta`$, so $`1^\circ`$ at $`0.8`$ m is $`14`$ mm on the table. For planar squares OpenCV also offers the dedicated solver `SOLVEPNP_IPPE_SQUARE`; the script uses the default iterative method. A larger marker, several markers, or several views reduce this error.

The script is a function (`get_camera_transform_from_marker(color_image, camera_matrix, dist_coeffs)`), not a node; the README shows how to call it. If markers are visible but none has ID 0, the expression `np.where(ids == 0)[0][0]` raises `IndexError`.

## 4.3 The calibrated transform

`monitor_cpp/launch/move_robot.launch.py` publishes

```math
{}^{base}t_{cam} = (0.4427,\ 0.4566,\ 0.6999)\ \text{m}, \qquad
q = (-0.0077,\ 0.9606,\ -0.2777,\ 0.0022),
```

```math
R(q) \approx \begin{bmatrix}
-0.9999 & -0.0136 & +0.0085 \\
-0.0160 & +0.8456 & -0.5335 \\
+0.0000 & -0.5336 & -0.8457
\end{bmatrix}.
```

Reading the columns: the camera's $`x`$ axis is almost exactly the base $`-x`$; its viewing axis (third column) points down and towards $`-y`$, tilted $`32.2^\circ`$ from the vertical. The camera sits 0.70 m above the base origin and 0.46 m to the robot's left, and its optical axis meets the table plane $`z = -0.075`$ near $`(0.45, -0.03)`$.

**Naming convention.** The third column is the *viewing* direction, so `camera_link` here is an optical frame ($`z`$ forward), consistent with the coordinates produced by the pinhole model. In the frame tree of `realsense2_camera`, by contrast, `camera_link` follows the ROS body convention ($`x`$ forward) and the optical frames are its children. The controller only transforms vision points, so its computation is consistent; tools that display the RealSense child frames (for example RViz) will show them rotated relative to reality.

## 4.4 Applying the transform at run time

For each detection, `vision_callback` in `move_to_pose.cpp` does

```cpp
geometry_msgs::msg::PointStamped point_camera;      // header.stamp = 0: use the latest transform
point_camera.header.frame_id = "camera_link";
point_camera.point.x = raw_x;  point_camera.point.y = raw_y;  point_camera.point.z = raw_z;
if (tf_buffer->canTransform("fr3_link0", "camera_link", tf2::TimePointZero))
    tf_buffer->transform(point_camera, point_robot, "fr3_link0");   // p_base = R p_cam + t
```

If the transform is not yet available the point is skipped silently.
Example from the animation: $`p_{cam} = (-0.073, 0.193, 0.789)`$ maps to $`p_{base} = (0.520, 0.200, -0.070)`$.

<!-- lab:frames -->
> **Interactive lab:** [rotate the 3-D scene](https://cocopops9.github.io/franka_ros2_language_based_manipulator/#/04-calibration-and-frames?lab=frames): base frame, calibrated camera frame, field of view and a movable point, with both coordinate vectors updated live.

## 4.5 Object memory

Positions in the base frame are kept in a map `label -> (x, y, z)` guarded by a mutex. For an observation $`p`$ of label $`\ell`$:

```math
m_\ell \leftarrow
\begin{cases}
p & \text{if } \ell \text{ was never seen} \quad (\text{log: } \texttt{New object stored}) \\
p & \text{if } \lVert p - m_\ell \rVert > 0.02\ \text{m} \quad (\text{log: } \texttt{Memory updated}) \\
m_\ell & \text{otherwise.}
\end{cases}
```

The 2 cm dead band suppresses jitter from depth noise, but it has side effects that follow from the code:

* **No expiry.** An entry is never removed. If a tool leaves the camera's view, its last position stays valid for the controller.
* **One entry per label.** Two forks share the key `fork`; whichever observation differs by more than 2 cm from the stored point overwrites it.
* **Snapshot semantics.** The controller copies the entry when a task is dispatched, *before* the 10 s pause and the approach. Moving the tool after that moment does not change the target.
* **Lag.** The comparison is made against the *stored* point, not the previous observation. A slow drift is therefore not followed continuously: the entry jumps only when the total displacement exceeds 2 cm, and it can lag the true position by up to 2 cm.

<!-- lab:memory -->
> **Interactive lab:** [drag an object and watch the memory](https://cocopops9.github.io/franka_ros2_language_based_manipulator/#/04-calibration-and-frames?lab=memory): observations stream in with configurable noise; the memory moves only when the dead band is exceeded.

## 4.6 What to take away

1. The calibration is $`{}^{base}T_{cam} = {}^{base}T_{marker}\,({}^{cam}T_{marker})^{-1}`$, computed once and frozen in the launch file.
2. `camera_link` is used as an optical frame; that is what makes the deprojected numbers and the transform compatible.
3. The memory is a dead-band filter without expiry; the controller reads a snapshot at dispatch time.

<p align="right"><b>Next:</b> <a href="05-voice-interface.md">5. Voice interface</a></p>
