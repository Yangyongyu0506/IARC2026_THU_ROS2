# IARC2026_THU_ROS2
This repository contains all the code needed to be deployed on quadcopters for IARC mission 10.
# Vision Tutorials
## How to calibrate a camera
ROS2 has provided multiple reusable packages that can relieve us from rebuilding the wheels for some simple but vital functions. Of these packages the most important ones for visual functions are camera_ros, which directly visits the camera on the computer using V4L2, image_proc, which can rectify the raw image from the camera, and camera_calibration, which can help us get the parameter matrices necessary for rectification. 
### Installation
To install them, run:
```bash
sudo apt install ros-$ROS_DISTRO-camera-calibration ros-$ROS_DISTRO-camera-ros ros-$ROS_DISTRO-image-proc
```
### Usage
To check the official documentation of these packages for more details, please visit:
- [camera_ros](https://docs.ros.org/en/jazzy/p/camera_ros/)
- [image_proc](https://docs.ros.org/en/jazzy/p/image_proc/)
- [camera_calibration](https://docs.ros.org/en/jazzy/p/camera_calibration/)

To start the camera node, run:
```bash
ros2 run camera_ros camera_node --ros-args -p camera:=<camera_id: int|string>
```
This will start a camera node that publishes topics `/<namespace>/image_raw`, `/<namespace>/image_raw/compressed` and `/<namespace>/camera_info`.

To start camera calibration, run:
```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/image camera:=/camera
```
Then follow the instructions on the terminal to move the chessboard around until the calibration is done. After that, you can save the calibration result to a yaml file and use it for rectification.

To start the image rectification node, run:
```bash
ros2 run image_proc rectify_node
```

Here is a clean, structured Markdown document that formalizes your frame transformation design and makes it suitable for implementation or documentation.

---

# Frame Transformation Design for IARC Mission 10

## 1. Problem Overview

In IARC Mission 10, the arena geometry is known **a priori** via four measured corner points in geodetic coordinates:

* Latitude (lat)
* Longitude (lon)
* Altitude (alt)

Multiple UAVs are deployed, each running PX4. Each UAV:

* Boots at an **arbitrary position**
* Maintains its own **local NED frame** (PX4-local-frame)
* Has an **unknown offset and yaw alignment** relative to the arena

### Objective

Define a **global, consistent coordinate frame** (ros2-enu-frame) for:

* Setpoint generation
* Target localization
* Multi-UAV coordination

Without modifying PX4 internal estimator behavior.

---

## 2. Coordinate Frames Definition

### 2.1 PX4 Local Frame (px4-local-frame)

* Type: Local NED (North-East-Down)
* Axes:

  * ( x ): North
  * ( y ): East
  * ( z ): Down
* Origin:

  * Determined by PX4 estimator at initialization
* Properties:

  * Not guaranteed to align with true North/East (depends on heading source)
  * Different for each UAV

---

### 2.2 ROS2 Arena Frame (ros2-enu-frame)

Custom-defined frame:

* Type: Right-handed ENU-like frame
* Axes:

  * ( x ): Along bottom edge of arena
  * ( y ): Derived via cross product
  * ( z ): Up (normal of fitted plane)
* Origin:

  * Bottom-left corner of the arena

---

### 2.3 Intermediate Frame (Geodetic / ECEF / ENU)

Used internally for transformations:

* WGS84 (lat, lon, alt)
* ECEF (Earth-Centered Earth-Fixed)
* Local ENU (relative to chosen origin)

---

## 3. Arena Frame Construction

### 3.1 Input

Four known points:

[
P_i = (lat_i, lon_i, alt_i), \quad i = 1..4
]

---

### 3.2 Convert to Local ENU

Using bottom-left corner ( P_0 ) as origin:

[
(e_i, n_i, u_i) = \text{geodetic2enu}(P_i, P_0)
]

---

### 3.3 Define Axes

#### X-axis (arena forward)

[
\mathbf{x} = \frac{P_1 - P_0}{|P_1 - P_0|}
]

---

#### Z-axis (up direction via plane fitting)

Fit plane using least squares:

* Stack centered points:
  [
  A = P_i - \bar{P}
  ]

* Perform SVD:
  [
  A = U \Sigma V^T
  ]

* Plane normal:
  [
  \mathbf{z} = V_{last}
  ]

Ensure upward direction:

[
\text{if } z_z < 0,\quad \mathbf{z} = -\mathbf{z}
]

---

#### Y-axis

[
\mathbf{y} = \mathbf{z} \times \mathbf{x}
]

---

#### Re-orthogonalization

[
\mathbf{x} = \mathbf{y} \times \mathbf{z}
]

---

### 3.4 Rotation Matrix

[
R_{\text{ENU} \rightarrow \text{arena}} =
\begin{bmatrix}
\mathbf{x} & \mathbf{y} & \mathbf{z}
\end{bmatrix}
]

---

## 4. PX4 Frame Reconstruction

### 4.1 Available Data (via ROS2-uORB bridge)

* `vehicle_global_position`: (lat, lon, alt)
* `vehicle_local_position`: (x, y, z) in NED

---

### 4.2 Convert Geodetic → ENU

[
(e, n, u) = \text{geodetic2enu}(lat, lon, alt, lat_0, lon_0, alt_0)
]

---

### 4.3 ENU → NED Conversion

[
\begin{aligned}
x_{ned} &= n \
y_{ned} &= e \
z_{ned} &= -u
\end{aligned}
]

---

## 5. Frame Transformation Estimation

We want:

[
T_{\text{px4-local} \rightarrow \text{ros2-enu}}
]

---

### 5.1 Data Collection

Collect ( N ) corresponding samples:

* PX4 local:
  [
  \mathbf{p}_i^{px4}
  ]

* Arena frame:
  [
  \mathbf{p}_i^{arena}
  ]

---

### 5.2 Solve Rigid Transformation

Find:

[
\mathbf{p}^{arena} = R \mathbf{p}^{px4} + t
]

---

### 5.3 Solution via SVD (Umeyama / Horn method)

Steps:

1. Compute centroids:
   [
   \bar{p}*{px4}, \quad \bar{p}*{arena}
   ]

2. Center data:
   [
   A = p^{px4} - \bar{p}*{px4}, \quad B = p^{arena} - \bar{p}*{arena}
   ]

3. Covariance:
   [
   H = A^T B
   ]

4. SVD:
   [
   H = U \Sigma V^T
   ]

5. Rotation:
   [
   R = V U^T
   ]

6. Reflection correction if needed

7. Translation:
   [
   t = \bar{p}*{arena} - R \bar{p}*{px4}
   ]

---

## 6. Runtime Usage

### 6.1 PX4 → Arena

[
\mathbf{p}^{arena} = R \mathbf{p}^{px4} + t
]

---

### 6.2 Arena → PX4 (for control)

[
\mathbf{p}^{px4} = R^T (\mathbf{p}^{arena} - t)
]

---

## 7. System Architecture

On companion computer (Khadas VIM4):

### Responsibilities:

* Frame construction (once)
* Transformation estimation (once per UAV)
* Runtime coordinate conversion

---

### Data Flow

```
GPS (lat/lon/alt)
        ↓
     ECEF
        ↓
     ENU (global)
        ↓
  Arena Frame (ros2-enu)
        ↓
  Control / Perception

PX4 Local (NED)
        ↓
  Transform (R, t)
        ↓
  Arena Frame
```

---

## 8. Key Properties

### Advantages

* No modification to PX4
* Deterministic global frame
* Works for multi-UAV systems
* Independent of PX4 origin

---

### Limitations

* Requires accurate GPS or global reference
* Sensitive to:

  * GPS noise
  * estimator drift
* PX4 yaw misalignment absorbed into transform

---

## 9. Recommendations

* Use ≥10 correspondence points for robustness
* Synchronize messages (ROS2 `message_filters`)
* Compute transform **per UAV**
* Recompute if EKF resets

---

## 10. Summary

This approach:

* Defines a **physically meaningful global frame**
* Bridges **PX4 local frames → global arena frame**
* Enables:

  * consistent control
  * target fusion
  * multi-agent coordination

Without interfering with PX4 estimator internals.