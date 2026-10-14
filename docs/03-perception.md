# 3. Perception

[Docs index](README.md) · [1. Overview](01-system-overview.md) · [2. Mission](02-mission-and-scheduling.md) · **3. Perception** · [4. Frames](04-calibration-and-frames.md) · [5. Voice](05-voice-interface.md) · [6. Motion](06-motion-and-grasping.md) · [7. End to end](07-end-to-end-run.md)

`vision_node` turns every colour frame into a list of 3-D points, one per tool that is both *recognised* by an open-vocabulary detector and *confirmed* by a fiducial marker.
The whole computation happens in `VisionNode.image_callback()` (`vision_module/src/vision_module/vision_node.py`).

<p align="center"><img src="assets/svg/perception.svg" alt="Animated perception pipeline" width="100%"></p>

## 3.1 Inputs

`vision.launch.py` starts `realsense2_camera` with `align_depth.enable: true`, so the driver publishes a depth image **registered to the colour camera**: the depth value at colour pixel $`(u, v)`$ belongs to the surface seen at that pixel.

| Topic | Use in the node |
| :---- | :-------------- |
| `/camera/camera/color/camera_info` | $`K`$ is stored once: $`f_x, f_y, c_x, c_y`$. Distortion coefficients are not used. |
| `/camera/camera/aligned_depth_to_color/image_raw` | Only the latest frame is stored (16-bit, millimetres). There is no time synchronisation with the colour frame. |
| `/camera/camera/color/image_raw` | Triggers the whole pipeline; nothing happens until $`K`$ and one depth frame have arrived. |

## 3.2 Step 1: open-vocabulary detection with YOLO-World

A conventional detector has a fixed output layer, one score per class seen in training.
YOLO-World (Cheng et al., *YOLO-World: Real-Time Open-Vocabulary Object Detection*, CVPR 2024) replaces that layer with a comparison between **region embeddings** produced by the image branch and **text embeddings** of the class names produced by a CLIP text encoder. In the paper's formulation, the score of region $`k`$ for vocabulary word $`j`$ is

```math
s_{k,j} = \alpha \cdot \frac{e_k}{\lVert e_k \rVert} \cdot \left(\frac{w_j}{\lVert w_j \rVert}\right)^{\!\top} + \beta,
```

with region embedding $`e_k`$, text embedding $`w_j`$ and learned scale and shift $`\alpha, \beta`$.
Because $`w_j`$ is computed from text, the vocabulary can be changed at run time without retraining. That is exactly what the node does:

```python
# condensed from vision_module/src/vision_module/yolo_detector_ultralytics.py
self.model = YOLO("yolov8l-worldv2.pt")          # large YOLO-World v2 weights, downloaded on first use
self.model.set_classes(self.class_names)          # class_names = tools[] from shared/sequence.json
results = self.model(image, conf=0.5, verbose=False)
```

Each result gives a box $`(c_x^{box}, c_y^{box}, w, h)`$ in pixels (converted to integers), a confidence and a class index into `tools`.
The node uses `conf=0.5`; Ultralytics' default is 0.25, so the node is deliberately conservative.

## 3.3 Step 2: ArUco confirmation inside each box

For every box, the node crops the image to the box (clamped to the image border) and runs ArUco detection **on the crop only**:

```python
corners, ids, _ = aruco.detectMarkers(roi, aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL), parameters=aruco.DetectorParameters())
```

ArUco (Garrido-Jurado et al., *Pattern Recognition*, 2014) markers are square binary codes with a black border. OpenCV's detector thresholds the image adaptively, keeps contours that approximate convex quadrilaterals, removes the perspective, samples the inner cells and looks the bit pattern up in the dictionary, tolerating a few bit errors. `DICT_ARUCO_ORIGINAL` contains 1024 markers of 5×5 bits.

The decision rule is binary:

| Case | `method` | Pixel used for depth | Published? |
| :--- | :------- | :------------------- | :--------- |
| a marker is found in the crop | `ARUCO` | centre of the first marker: mean of its 4 corners, shifted back to full-image coordinates and truncated to integers | yes |
| no marker in the crop | `YOLO` | centre of the YOLO box | **no**, it is only drawn (green) on `/debug_image` |

Three properties of this design are worth stating explicitly:

* The marker **ID is not used** to identify the tool: the label always comes from the detector class, and any marker of the dictionary confirms any box.
* Searching only inside boxes is cheap and ties each marker to one detection, but a marker cut by the box border is missed.
* Marker size limits robustness. A marker of side $`L`$ at depth $`z`$ spans $`f_x L / z`$ pixels. For the 15 mm markers at $`z \approx 0.8\ \text{m}`$ this is about 11 px with $`f_x \approx 615`$ (640×480 colour stream, approximate), that is, fewer than 2 px for each of the 7 cells across the marker (5 code cells plus the border). Focus, motion blur and lighting therefore have a large effect on detection.

Printable markers are in `vision_module/aruco_markers/`. The `ID*_15mm.pdf` sheets match the dictionary and the 15 mm length used in the code; the `*_4x4_20mm.pdf` sheets appear to come from a 4×4 dictionary and would not be detected without changing `DICT_ARUCO_ORIGINAL`.

## 3.4 Step 3: depth at one pixel

```math
z = \frac{D[v, u]}{1000}\ \text{m},
```

where $`D`$ is the aligned depth image in millimetres and $`(u, v)`$ the chosen pixel.
A single pixel is read, without averaging or outlier rejection. A missing depth value ($`D = 0`$) gives $`z = 0`$, and the detection is dropped by the test `z_cam > 0`.

## 3.5 Step 4: pinhole deprojection

The pinhole model projects a point $`(x, y, z)`$ of the camera frame to pixel coordinates

```math
u = f_x \frac{x}{z} + c_x, \qquad v = f_y \frac{y}{z} + c_y .
```

Knowing $`u, v`$ and $`z`$, `VisionNode.deproject()` inverts it:

```math
x = \frac{(u - c_x)\, z}{f_x}, \qquad y = \frac{(v - c_y)\, z}{f_y}.
```

The result is expressed in the **optical** convention: $`x`$ to the right of the image, $`y`$ down, $`z`$ along the viewing direction.

**Worked example** (the marked fork of the animation, example intrinsics $`f_x = f_y = 615`$, $`c_x = 320`$, $`c_y = 240`$):
$`u = 263`$, $`v = 390`$, $`D = 789`$ mm, hence $`z = 0.789`$, $`x = (263 - 320) \cdot 0.789 / 615 = -0.073`$ m and $`y = (390 - 240) \cdot 0.789 / 615 = 0.192`$ m.

**Error propagation.** Differentiating the deprojection,

```math
\delta x \approx \frac{z}{f_x}\,\delta u + \frac{u - c_x}{f_x}\,\delta z .
```

At $`z = 0.8`$ m a one-pixel error moves the point by $`0.8/615 \approx 1.3`$ mm; the integer truncation of the marker centre is therefore negligible compared with depth noise, which enters with the factor $`(u - c_x)/f_x`$ laterally and fully along $`z`$.
Lens distortion is neglected.

## 3.6 Step 5: publication

Only `ARUCO` detections are serialised, in camera coordinates:

```python
# condensed from VisionNode.image_callback()
published = [{"label": d["label"], "x": d["x"], "y": d["y"], "z": d["z"]} for d in results_3d if d["method"] == "ARUCO"]
self.object_publisher.publish(published)    # std_msgs/String on /vision/detected_objects, json.dumps
```

The transformation to the robot frame is commented out in the node (`transform_to_robot`); it is performed by the controller with tf2 (Chapter 4). The cyan overlay labelled `Rob:` on `/debug_image` therefore still shows camera coordinates.

<!-- lab:camera -->
> **Interactive lab:** [move objects under a virtual camera](https://cocopops9.github.io/franka_ros2_language_based_manipulator/#/03-perception?lab=camera): the scene is projected with the calibrated extrinsics; toggle markers and depth holes and read the exact message the node would publish.

## 3.7 What to take away

1. Detection decides **what** an object is (label); the marker decides **whether** it is published and **where** its reference point is.
2. Depth comes from one pixel of the aligned depth image; there is no filtering.
3. The output is in the camera's optical frame; the controller must transform it.

<p align="right"><b>Next:</b> <a href="04-calibration-and-frames.md">4. Calibration, frames and memory</a></p>
