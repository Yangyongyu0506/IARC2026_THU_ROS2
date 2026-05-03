# frametransformer_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
from iarc_utils.mathematics import rotation_matrix_to_quaternion, get_normal_vector
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleLocalPosition 
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
        # ROS2 parameters' declaration
        self.local_position_topic = self.declare_parameter(
            "local_position_topic", 
            "", 
            ParameterDescriptor(
                description="Topic name for local position data. Its message type is px4_msgs/msg/VehicleLocalPosition. Attention do not add namespace here. Instead, set the namespace in the launch file."
            )
        ).value
        self.package_name = self.declare_parameter(
            "package_name", "iarc_main", ParameterDescriptor(
                description="Name of the package containing the arena configuration file."
            )
        ).value
        self.arena_config_folder = self.declare_parameter(
            "arena_config_folder", "config", ParameterDescriptor(
                description="Name of the folder containing the arena configuration file within the package."
            )
        ).value
        self.arena_config_file = self.declare_parameter(
            "arena_config_file", "arena_config.json", ParameterDescriptor(
                description="Name of the JSON file containing the arena configuration. Note that the first point in the file is expected to be the left-down corner of the arena, and the points are expected to be ordered in a counter-clockwise manner."
            )
        ).value
        self.arena_frame_id = self.declare_parameter(
            "arena_frame_id", "arena_enu", ParameterDescriptor(
                description="Frame ID for the arena frame."
            )
        ).value
        self.px4_ned_frame_id = self.declare_parameter(
            "px4_ned_frame_id", "px4_ned", ParameterDescriptor(
                description="Frame ID for the PX4 NED frame."
            )
        ).value
        self.is_origin_valid = False
        self.origin_wgs84 = np.zeros(3)  # [lat, lon, alt_amsl]
    
    def _ros2_init(self):
        """
        Initialize ROS2 components
        """
        # Load arena config
        self._load_arena_config()
        # Init PX4 qosprofile
        px4_qosprofile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            VehicleLocalPosition,
            self.local_position_topic,
            self._local_position_callback,
            px4_qosprofile
        )
        # Init static transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

    def _load_arena_config(self):
        """
        Load arena configuration from the specified JSON file
        """
        config_dir = os.path.join(get_package_share_directory(self.package_name), self.arena_config_folder, self.arena_config_file)
        with open(config_dir, 'r') as f:
            config = json.load(f)
        self.arena_corners_list = config["arena_corners"]
        self.arena_corners_wgs84 = np.array(
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
            if msg.xy_global and msg.z_global: # Check if the ref pos from autopilot is valid
                self.origin_wgs84 = np.array([msg.ref_lat, msg.ref_lon, msg.ref_alt])
                self.heading_yaw = msg.heading
                self.tf_stamp_us = msg.ref_timestamp
                self._publish_static_transform() # Publish the static transform immediately after receiving the first valid local position message to minimize the waiting time for other nodes that depend on the transform
                self.is_origin_valid = True
                self.get_logger().info(f"Origin initialized: {self.origin_wgs84}")
            else:
                self.get_logger().warn("Received local position message with invalid global reference. Waiting for a valid message to initialize the origin.")

    def _publish_static_transform(self):
        """
        Calculate and publish the static transform from the arena frame to the base_link frame
        """
        arena_corners_ned = np.array(
            [
                pm.ned.geodetic2ned(
                    self.arena_corners_wgs84[i][0], 
                    self.arena_corners_wgs84[i][1], 
                    self.arena_corners_wgs84[i][2],
                    self.origin_wgs84[0], 
                    self.origin_wgs84[1], 
                    self.origin_wgs84[2]) for i in range(len(self.arena_corners_wgs84))
            ]
        )

        arena_x_vec0 = arena_corners_ned[1] - arena_corners_ned[0]
        arena_y_vec0 = arena_corners_ned[3] - arena_corners_ned[0]
        arena_z_vec0 = np.cross(arena_x_vec0, arena_y_vec0)
        arena_z_vec = get_normal_vector(arena_corners_ned)
        arena_z_vec *= np.sign(np.dot(arena_z_vec, arena_z_vec0)) # Ensure the direction of the normal vector is consistent with the right-hand rule defined by the order of the corners in the config file
        arena_y_vec = np.cross(arena_z_vec, arena_x_vec0)
        arena_y_vec /= np.linalg.norm(arena_y_vec)
        arena_x_vec = np.cross(arena_y_vec, arena_z_vec)
        arena_x_vec /= np.linalg.norm(arena_x_vec)
        R_arena2ned = np.column_stack((arena_x_vec, arena_y_vec, arena_z_vec))
        # unify the rotation matrix
        R_arena2ned[:, 2] *= np.sign(np.linalg.det(R_arena2ned))
        Q_arena2ned = rotation_matrix_to_quaternion(R_arena2ned)
        t_arena2ned = arena_corners_ned[0]
        tf = TransformStamped()
        tf.header.stamp.sec = self.tf_stamp_us // 1000000
        tf.header.stamp.nanosec = (self.tf_stamp_us % 1000000) * 1000
        tf.header.frame_id = self.arena_frame_id
        tf.child_frame_id = self.px4_ned_frame_id
        tf.transform.translation.x = t_arena2ned[0]
        tf.transform.translation.y = t_arena2ned[1]
        tf.transform.translation.z = t_arena2ned[2]
        tf.transform.rotation.x = Q_arena2ned[0]
        tf.transform.rotation.y = Q_arena2ned[1]
        tf.transform.rotation.z = Q_arena2ned[2]
        tf.transform.rotation.w = Q_arena2ned[3]
        self.tf_broadcaster.sendTransform(tf)
        self.get_logger().info(f"Published static transform from {self.arena_frame_id} to {self.px4_ned_frame_id} with translation {t_arena2ned} and rotation (quaternion) {Q_arena2ned}")
    
def main(args=None):
    rclpy.init(args=args)
    node = FrameTransformerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Placeholder for any cleanup operations if needed in the future
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()