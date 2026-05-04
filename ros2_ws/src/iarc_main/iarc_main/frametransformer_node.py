"""
FrameTransformerNode: compute and broadcast the static transform between the arena frame and PX4 NED frame.

Two strategies are supported:
  - no_manual_calib: use the first valid local-position message to derive the transform
    (single-shot, no data collection).
  - manual_calib: collect N paired (local, global) samples via PX4MessageFilter and solve
    for the rigid transform using SVD, then broadcast once.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
from iarc_utils.mathematics import (
    rotation_matrix_to_quaternion,
    force_orthogonal,
    get_xyz_from_points,
)
from iarc_utils.messagefilters import PX4MessageFilter
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleGlobalPosition
import numpy as np
import pymap3d as pm
import json
import os


class FrameTransformerNode(Node):
    def __init__(self):
        super().__init__("frame_transformer_node")
        self._params_init()
        self._ros2_init()
        self.get_logger().info("FrameTransformerNode initialized")

    def _params_init(self):
        """
        Initialize parameters for the node.
        """
        self.local_position_topic = self.declare_parameter(
            "local_position_topic",
            "",
            ParameterDescriptor(
                description="Topic name for local position data. Its message type is px4_msgs/msg/VehicleLocalPosition. Attention do not add namespace here. Instead, set the namespace in the launch file."
            ),
        ).value
        self.package_name = self.declare_parameter(
            "package_name",
            "iarc_main",
            ParameterDescriptor(
                description="Name of the package containing the arena configuration file."
            ),
        ).value
        self.arena_config_folder = self.declare_parameter(
            "arena_config_folder",
            "config",
            ParameterDescriptor(
                description="Name of the folder containing the arena configuration file within the package."
            ),
        ).value
        self.arena_config_file = self.declare_parameter(
            "arena_config_file",
            "arena_config.json",
            ParameterDescriptor(
                description="Name of the JSON file containing the arena configuration. Note that the first point in the file is expected to be the left-down corner of the arena, and the points are expected to be ordered in a counter-clockwise manner."
            ),
        ).value
        self.arena_frame_id = self.declare_parameter(
            "arena_frame_id",
            "odom",
            ParameterDescriptor(description="Frame ID for the arena frame."),
        ).value
        self.px4_ned_frame_id = self.declare_parameter(
            "px4_ned_frame_id",
            "px4_ned",
            ParameterDescriptor(description="Frame ID for the PX4 NED frame."),
        ).value
        self.strategy = self.declare_parameter(
            "strategy",
            "no_manual_calib",
            ParameterDescriptor(
                description="Algorithm for calculating the transform from the arena frame to the PX4 NED frame. Options are: 1. no_manual_calib: directly use the local position data from PX4 to decide the transform. 2. manual_calib: subscribe to several paired global and local position data and calculate the transform using SVD."
            ),
        ).value
        assert self.strategy.lower() in ["no_manual_calib", "manual_calib"], (
            "Invalid strategy parameter. Must be either 'no_manual_calib' or 'manual_calib'."
        )
        match self.strategy:
            case "no_manual_calib":
                self.get_logger().info(
                    "Using no_manual_calib strategy for transform calculation."
                )
                self.is_origin_valid = False
                self.origin_geo = np.zeros(3)  # [lat, lon, alt_amsl]
            case "manual_calib":
                self.get_logger().info(
                    "Using manual_calib strategy for transform calculation."
                )
                self.global_position_topic = self.declare_parameter(
                    "global_position_topic",
                    "",
                    ParameterDescriptor(
                        description="Topic name for global position data. Its message type is px4_msgs/msg/VehicleGlobalPosition."
                    ),
                ).value
                self.msgfilterslop = self.declare_parameter(
                    "msgfilterslop",
                    10,
                    ParameterDescriptor(
                        description="The time tolerance in microseconds for matching the local and global position messages."
                    ),
                ).value
                self.queue_size = self.declare_parameter(
                    "queue_size",
                    10,
                    ParameterDescriptor(
                        description="The queue size for storing the local and global position messages for matching."
                    ),
                ).value
                self.least_paired_msgs_num = self.declare_parameter(
                    "least_paired_msgs_num",
                    10,
                    ParameterDescriptor(
                        description="The least number of paired local and global position messages required for calculating the transform."
                    ),
                ).value
                self.paired_points = {"px4": [], "arena": []}
                self.is_calibrated = False

    def _ros2_init(self):
        """
        Initialize ROS2 components
        """
        # Load arena config
        self._load_arena_config()
        # Init static transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        # Init PX4 qosprofile
        px4_qosprofile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        if self.strategy == "no_manual_calib":
            self.localpos_sub = self.create_subscription(
                VehicleLocalPosition,
                self.local_position_topic,
                self._local_position_callback,
                px4_qosprofile,
            )
            self.get_logger().info(
                f"Subscribed to {self.local_position_topic} for local position data from PX4"
            )
        elif self.strategy == "manual_calib":
            self.arena_corners_ned_temp = np.array(
                [
                    pm.ned.geodetic2ned(
                        self.arena_corners_geo[i][0],
                        self.arena_corners_geo[i][1],
                        self.arena_corners_geo[i][2],
                        self.arena_corners_geo[0][0],
                        self.arena_corners_geo[0][1],
                        self.arena_corners_geo[0][2],
                    )
                    for i in range(len(self.arena_corners_geo))
                ]
            )
            self.R_temp = np.column_stack(
                get_xyz_from_points(self.arena_corners_ned_temp)
            )
            self.R_temp = force_orthogonal(self.R_temp)
            self.px4msgfilter = PX4MessageFilter(
                self,
                self.msgfilterslop,
                [self.local_position_topic, self.global_position_topic],
                [VehicleLocalPosition, VehicleGlobalPosition],
                [px4_qosprofile, px4_qosprofile],
                [self.queue_size, self.queue_size],
                self._filter_callback,
            )

    def _load_arena_config(self):
        """
        Load arena configuration from the specified JSON file
        """
        config_dir = os.path.join(
            get_package_share_directory(self.package_name),
            self.arena_config_folder,
            self.arena_config_file,
        )
        with open(config_dir, "r") as f:
            config = json.load(f)
        self.arena_corners_list = config["arena_corners"]
        self.arena_corners_geo = np.array(
            [
                [corner["lat"], corner["lon"], corner["alt_amsl"]]
                for corner in self.arena_corners_list
            ]
        )

    def _local_position_callback(self, msg: VehicleLocalPosition):
        """
        Callback function for processing local position messages from PX4
        """
        if not self.is_origin_valid:
            # Origin not initialized, parse the origin from the first received message
            if (
                msg.xy_global and msg.z_global
            ):  # Check if the ref pos from autopilot is valid
                self.origin_geo = [
                    msg.ref_lat,
                    msg.ref_lon,
                    msg.ref_alt,
                ]  # The ref_alt is in AMSL
                self.heading_yaw = msg.heading
                self._publish_static_transform()  # Publish the static transform immediately after receiving the first valid local position message to minimize the waiting time for other nodes that depend on the transform
                self.is_origin_valid = True
                self.get_logger().info(f"Origin initialized: {self.origin_geo}")
            else:
                self.get_logger().warn(
                    "Received local position message with invalid global reference. Waiting for a valid message to initialize the origin."
                )

    def _publish_static_transform(self):
        """
        Calculate and publish the arena -> PX4 NED static transform.

        Converts arena corner points from geodetic to NED using the current PX4 origin,
        constructs an orthogonal frame from the corner geometry, and broadcasts the
        corresponding (translation, quaternion) pair.
        """
        # Convert arena corners from geodetic to NED using the current PX4 origin
        arena_corners_ned = np.array(
            [
                pm.ned.geodetic2ned(
                    self.arena_corners_geo[i][0],
                    self.arena_corners_geo[i][1],
                    self.arena_corners_geo[i][2],
                    self.origin_geo[0],
                    self.origin_geo[1],
                    self.origin_geo[2],
                )
                for i in range(len(self.arena_corners_geo))
            ]
        )
        # Build rotation matrix from the arena frame axes, make orthonormal via SVD
        R = np.column_stack(get_xyz_from_points(arena_corners_ned))
        R = force_orthogonal(R)
        Q = rotation_matrix_to_quaternion(R)
        t = arena_corners_ned[0]  # translation = position of bottom-left corner
        tf = TransformStamped()
        tf.header.frame_id = self.arena_frame_id
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.child_frame_id = self.px4_ned_frame_id
        tf.transform.translation.x = t[0]
        tf.transform.translation.y = t[1]
        tf.transform.translation.z = t[2]
        tf.transform.rotation.x = Q[0]
        tf.transform.rotation.y = Q[1]
        tf.transform.rotation.z = Q[2]
        tf.transform.rotation.w = Q[3]
        self.tf_broadcaster.sendTransform(tf)
        self.get_logger().info(
            f"Published static transform from {self.arena_frame_id} to {self.px4_ned_frame_id} with translation {t} and rotation (quaternion) {Q}"
        )

    def _filter_callback(
        self, local_msg: VehicleLocalPosition, global_msg: VehicleGlobalPosition
    ):
        """
        Collect paired (local NED, arena ENU) samples and solve for rigid transform.

        Accumulates until ``least_paired_msgs_num`` is reached, then computes rotation
        and translation via SVD (Umeyama method) and publishes the static transform once.
        Subsequent callbacks are ignored.
        """
        if not self.is_calibrated:
            if (
                len(self.paired_points["px4"]) < self.least_paired_msgs_num
                or len(self.paired_points["arena"]) < self.least_paired_msgs_num
            ):
                self.get_logger().info(
                    f"Received paired local and global position messages."
                )
                self.paired_points["px4"].append(
                    [local_msg.x, local_msg.y, local_msg.z]
                )
                # Convert current global sample to NED relative to arena origin, then rotate
                arena_point_temp = np.array(
                    pm.ned.geodetic2ned(
                        global_msg.lat,
                        global_msg.lon,
                        global_msg.alt,
                        self.arena_corners_geo[0][0],
                        self.arena_corners_geo[0][1],
                        self.arena_corners_geo[0][2],
                    )
                )
                arena_point = self.R_temp.T @ arena_point_temp.T
                self.paired_points["arena"].append(
                    [arena_point[0], arena_point[1], arena_point[2]]
                )
            else:
                assert len(self.paired_points["px4"]) == len(
                    self.paired_points["arena"]
                ), "The number of paired points in px4 and arena should be the same."
                # Solve R, t via SVD:  p_arena = R * p_px4 + t
                px4_points = np.array(self.paired_points["px4"])
                arena_points = np.array(self.paired_points["arena"])
                centroid_px4 = np.mean(px4_points, axis=0)
                centroid_arena = np.mean(arena_points, axis=0)
                H = (px4_points - centroid_px4).T @ (
                    arena_points - centroid_arena
                )  # cross-covariance
                if np.linalg.matrix_rank(H) < 3:
                    self.get_logger().warn(
                        "Degenerated sample points. Cannot solve transform. Please move the drone around slowly."
                    )
                    return
                U, _, Vt = np.linalg.svd(H)
                R = Vt.T @ U.T  # optimal rotation (Umeyama)
                R = force_orthogonal(R)
                t = centroid_arena - R @ centroid_px4
                # Publish the solved static transform and mark as calibrated
                Q = rotation_matrix_to_quaternion(R)
                tf = TransformStamped()
                tf.header.stamp = self.get_clock().now().to_msg()
                tf.header.frame_id = self.arena_frame_id
                tf.child_frame_id = self.px4_ned_frame_id
                tf.transform.translation.x = t[0]
                tf.transform.translation.y = t[1]
                tf.transform.translation.z = t[2]
                tf.transform.rotation.x = Q[0]
                tf.transform.rotation.y = Q[1]
                tf.transform.rotation.z = Q[2]
                tf.transform.rotation.w = Q[3]
                self.tf_broadcaster.sendTransform(tf)
                self.get_logger().info(
                    f"Published static transform from {self.arena_frame_id} to {self.px4_ned_frame_id} with translation {t} and rotation (quaternion) {Q}"
                )
                self.is_calibrated = True

    def on_exit(self):
        """
        Clean up resources on node shutdown if necessary
        """
        self.get_logger().warn("Shutting down FrameTransformerNode")
        pass


def main(args=None):
    rclpy.init(args=args)
    node = FrameTransformerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.on_exit()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
