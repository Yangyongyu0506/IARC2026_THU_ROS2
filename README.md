# IARC2026_THU_ROS2
This repository contains all the code needed to be deployed on quadcopters for IARC mission 10.
# Tutorials
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