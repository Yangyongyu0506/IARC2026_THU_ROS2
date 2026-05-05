# IARC2026\_THU\_ROS2

ROS 2 workspace and companion-computer code for the IARC Mission 10 quadcopter system.

## Repository Structure

```
.
├── ros2_ws/               # ROS 2 workspace (colcon)
│   └── src/
│       ├── iarc_main/     # Core mission nodes
│       ├── iarc_msgs/     # Custom ROS 2 interfaces (.srv)
│       ├── iarc_sim/      # Simulation helpers
│       ├── iarc_utils/    # Shared utilities (filters, math)
│       ├── px4_msgs/      # PX4 message/service definitions
│       ├── px4_ros_com/   # PX4-ROS2 bridge examples + frame_transforms
│       └── test_pkg/      # Local integration test package
├── Micro-XRCE-DDS-Agent/  # Vendored C++ agent (independent CMake build)
└── other_materials/       # Misc reference documents
```

## Prerequisites

- **OS:** Ubuntu 22.04 / 24.04
- **ROS 2:** Humble / Jazzy (with `colcon`, `rosidl`, `ament`)
- **PX4 toolchain:** `px4_msgs`, `px4_ros_com` dependencies (Eigen3)
- **Python:** ≥3.10, with `numpy`, `pymap3d`, `tf2_ros`, `message_filters`
- **Micro-XRCE-DDS-Agent:** CMake ≥3.16, C++11 compiler, Fast-CDR, optional Fast-DDS

Install ROS 2 desktop and tools:

```bash
sudo apt install ros-$ROS_DISTRO-desktop python3-colcon-common-extensions
```

## Build

### ROS 2 workspace

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Build only specific packages:

```bash
colcon build --packages-select iarc_msgs iarc_utils iarc_main iarc_sim px4_msgs px4_ros_com
```

### Micro-XRCE-DDS-Agent

```bash
cd Micro-XRCE-DDS-Agent
cmake -S . -B build
cmake --build build -j
```

To enable and run unit tests:

```bash
cmake -S . -B build -DUAGENT_BUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Test

### All ROS 2 packages

```bash
cd ros2_ws
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

### Single package

```bash
colcon test --packages-select iarc_utils
```

### Single pytest file or test case

```bash
colcon test --packages-select iarc_utils --pytest-args test/test_flake8.py
colcon test --packages-select iarc_utils --pytest-args "test/test_flake8.py::test_flake8"
```

### Single Micro-XRCE test

```bash
cd Micro-XRCE-DDS-Agent
ctest --test-dir build -R test-token-bucket --output-on-failure
```

## Package Overview

| Package | Type | Description |
|---------|------|-------------|
| `iarc_msgs` | `ament_cmake` | Custom interfaces (`GetStaticTransform.srv`) |
| `px4_msgs` | `ament_cmake` | PX4 uORB message definitions |
| `px4_ros_com` | `ament_cmake` | PX4-ROS2 bridge examples, `frame_transforms` library |
| `iarc_utils` | `ament_python` | Message filters (`PX4MessageFilter`, `LazyPX4MessageFilter`, `PX4MessageClamper`), math utilities (`rotation_matrix_to_quaternion`, `lerp`, `slerp`, `stamp2us`, plane fitting) |
| `iarc_main` | `ament_python` | Core mission nodes: `FrameTransformerNode`, `OdomTFBroadcasterNode`, `SetpointSenderNode`, `TargetFeedbackNode` |
| `iarc_sim` | `ament_python` | `UDPServerNode` — receives UDP position commands and forwards to PX4 offboard control |
| `test_pkg` | `ament_python` | Side-by-side test harness comparing ROS 2 `ApproximateTimeSynchronizer` with custom `PX4MessageFilter` |

## Run

### Frame Transformer

```bash
ros2 run iarc_main frametransformer_node --ros-args \
  -p local_position_topic:=/fmu/out/vehicle_local_position \
  -p strategy:=no_manual_calib
```

With manual calibration (collects N paired samples, solves via SVD):

```bash
ros2 run iarc_main frametransformer_node --ros-args \
  -p local_position_topic:=/fmu/out/vehicle_local_position \
  -p global_position_topic:=/fmu/out/vehicle_global_position \
  -p strategy:=manual_calib \
  -p msgfilterslop:=10 \
  -p least_paired_msgs_num:=10
```

### Odometry TF Broadcaster

```bash
ros2 run iarc_main odomtfbroadcaster_node --ros-args \
  -p px4_odom_topic:=/fmu/out/vehicle_odometry \
  -p strategy:=odom_callback \
  -p px4_ned_frame_id:=px4_ned \
  -p base_link_frame_id:=base_link
```

Interpolated mode (lerp position, slerp orientation between last two messages):

```bash
ros2 run iarc_main odomtfbroadcaster_node --ros-args \
  -p strategy:=lerp \
  -p publish_rate:=50.0
```

### Setpoint Sender

```bash
ros2 run iarc_main setpointsender_node --ros-args \
  -p arena_frame_id:=arena \
  -p px4_ned_frame_id:=px4_ned \
  -p udp_port_recv:=5005 \
  -p timer_period:=0.1
```

UDP JSON protocol: `{"s": <seq>, "c": "a"|"d"|"m", "x": ..., "y": ..., "z": ..., "yaw": ...}`
- `a` — arm, `d` — disarm, `m` — move to position + heading

### Target Feedback

```bash
ros2 run iarc_main targetfeedback_node --ros-args \
  -p tag_pose_topic:=pose \
  -p arena_frame_id:=arena \
  -p feedback_ip:=192.168.1.100 \
  -p feedback_port:=6006 \
  -p do_map_pub:=true
```

### Sim UDP Server

```bash
ros2 run iarc_sim udpserver_node --ros-args \
  -p udp_port_recv:=5005 \
  -p cmd_resolution_length:=1.0 \
  -p cmd_resolution_angle:=0.1
```

### Filter Test (side-by-side comparison)

```bash
ros2 run test_pkg filtertest_node
```

### Micro-XRCE-DDS Agent (standalone)

```bash
Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888
```

## Code Style

- **Python:** `ament_flake8` + `ament_pep257`; 4-space indent; `snake_case` functions, `PascalCase` classes; use `self.get_logger()` (not `print`); type hints on public APIs
- **C++:** C++14 (`px4_ros_com`) / C++11 (Micro-XRCE); `-Wall -Wextra -Wpedantic`; RAII; trailing underscore on member fields in ROS nodes
- See `AGENTS.md` for full conventions

---

## Frame Transformation Design

The arena geometry is known *a priori* via four geodetic corner points. Each UAV boots at an arbitrary PX4-local NED origin. The system constructs a global **arena ENU frame** and publishes the static transform from that frame to each UAV's PX4-local NED frame.

Two strategies in `FrameTransformerNode`:

1. **`no_manual_calib`** — uses the first valid PX4 local-position message to derive the transform (single-shot).
2. **`manual_calib`** — collects N paired (local-NED, global-geodetic) samples via `PX4MessageFilter`, solves for the rigid transform with SVD (Umeyama method), then broadcasts once.

### Coordinate Frames

| Frame | Axes | Origin |
|-------|------|--------|
| Arena ENU (`arena`) | x = bottom edge, z = plane normal (up), y = z × x | Bottom-left corner |
| PX4 NED (`px4_ned`) | North-East-Down | PX4 estimator init position |
| Intermediate | WGS84 → ECEF → local ENU | Chosen reference point |

### Transform Pipeline

```
Arena corners (geodetic) → geodetic2ned → axes + origin → R, t
                                                              ↓
                                              tf_broadcaster: arena → px4_ned
```

## Vision Tutorials

### Camera Calibration

Install dependencies:

```bash
sudo apt install ros-$ROS_DISTRO-camera-calibration ros-$ROS_DISTRO-camera-ros ros-$ROS_DISTRO-image-proc
```

Start the camera node:

```bash
ros2 run camera_ros camera_node --ros-args -p camera:=0
```

Run calibration:

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/image camera:=/camera
```

Rectify images:

```bash
ros2 run image_proc rectify_node
```

Official docs: [camera_ros](https://docs.ros.org/en/jazzy/p/camera_ros/), [image_proc](https://docs.ros.org/en/jazzy/p/image_proc/), [camera_calibration](https://docs.ros.org/en/jazzy/p/camera_calibration/)
